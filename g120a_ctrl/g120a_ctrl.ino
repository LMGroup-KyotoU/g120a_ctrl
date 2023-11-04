#include "G120aMotor.h"
#include "Vsc3Gamepad.h"
#include "G120aMotionController.h"
#include "utils.h"

/** ROS INCLUDE **/
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16MultiArray.h>

/** ROS DEFINE **/
#define CTRL_FREQ 50
#define SPIN_FREQ 30 // Interval of publish for ros

/** ROVER DEFINE **/
#define MAX_SPD_FWD 0.4 // maximum forward speed in m/s
#define MAX_SPD_ROT 0.8 // maximum rotate speed in rad/s
#define SPD_FWD 0.4
#define SPD_ROT 0.8
#define PIN_DRV_EN 15

/** ROS DECLARATION **/
void twistCallBack(const geometry_msgs::Twist& msg);

/** GAMEPAD PARAMS **/
Vsc3Gamepad gamepad;

/** MOTOR PARAMS **/
HardwareSerial motorSerial(2);
G120aMotionController* g120aMotionController = nullptr;

/** ROS PARAMS **/
ros::NodeHandle nh;
uint64_t lastSpin = millis();
const uint32_t spinInterval = 1000 / SPIN_FREQ;
// Publisher
// geometry_msgs::Twist test_twist;
// ros::Publisher pub_twist("rover_twist", &test_twist);
// Subscriber
ros::Subscriber<geometry_msgs::Twist> sub_twist("rover_twist", &twistCallBack);

/** TIMER PARAMS **/
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool timerFlag = false;

// Timer interrupt function
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  timerFlag = true;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {
  /** SERIAL INIT **/
  Serial.begin(115200);
  Serial.println("Init start");

  /** MOTOR INIT **/
  // Motor control enable
  pinMode(PIN_DRV_EN, OUTPUT);
  digitalWrite(PIN_DRV_EN, HIGH);
  // Set motor serial
  motorSerial.begin(115200, SERIAL_8N1, 16, 17);
  g120aMotionController = new G120aMotionController(motorSerial);

  /** ROS INIT **/
  nh.initNode();
  // nh.advertise(pub_twist);
  nh.subscribe(sub_twist);

  // Init done
  Serial.println("Init done");

  /** TIMER INIT **/
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000000.0/CTRL_FREQ, true);  // 1000,000 = 1 second
  timerAlarmEnable(timer);
}

void loop() {
  if(timerFlag) {
    // Timer critital point
    portENTER_CRITICAL(&timerMux);
    timerFlag = false;
    portEXIT_CRITICAL(&timerMux);

    /** MAIN LOOP **/
    gamepad.update(); // Update gamepad

    // Control by gamepad
    if(gamepad.isBtnPressed(Vsc3Gamepad::BTN_S_L2)) {
      if(gamepad.isBtnPressed(Vsc3Gamepad::BTN_CROSS_U)) {
        g120aMotionController->setMotion(SPD_FWD, 0);
      }
      else if (gamepad.isBtnPressed(Vsc3Gamepad::BTN_CROSS_D)) {
        g120aMotionController->setMotion(-SPD_FWD, 0);
      }
      else if (gamepad.isBtnPressed(Vsc3Gamepad::BTN_CROSS_L)) {
        g120aMotionController->setMotion(0, SPD_ROT);
      }
      else if (gamepad.isBtnPressed(Vsc3Gamepad::BTN_CROSS_R)) {
        g120aMotionController->setMotion(0, -SPD_ROT);
      }
      else {
        g120aMotionController->setMotion(0,0);
      }
    }

  }

  /** ROS OPERATION **/
  if(millis() - lastSpin > spinInterval) {
    // Publish
    // pub_twist.publish(&test_twist);

    // Spin
    nh.spinOnce();
  }
}

/** ROS CALLBACK **/
void twistCallBack(const geometry_msgs::Twist& msg) {
  gamepad.update(); // Update gamepad
  if(!gamepad.isBtnPressed(Vsc3Gamepad::BTN_S_L2)) {                                  
    float spd_fwd = clamp(msg.linear.x, -MAX_SPD_FWD, MAX_SPD_FWD);
    float spd_rot = clamp(msg.angular.z, -MAX_SPD_ROT, MAX_SPD_ROT);
    g120aMotionController->setMotion(spd_fwd, spd_rot);
  }
}
