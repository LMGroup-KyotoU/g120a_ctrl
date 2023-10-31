#include "motor_control.h"
#include "gamepad.h"
#include "motion_controller.h"
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
#define MAX_SPD_FWD 0.2 // maximum forward speed in m/s
#define MAX_SPD_ROT 1   // maximum rotate speed in rad/s
#define SPD_FWD 0.2
#define SPD_ROT 1

/** ROS DECLEARATION **/
void twistCallBack(const geometry_msgs::Twist& msg);

/** GAMEPAD PARAMS **/
Gamepad gamepad;

/** ROS PARAMS **/
ros::NodeHandle nh;
uint64_t lastSpin = millis();
const uint32_t spinInterval = 1000 / SPIN_FREQ;
// Publisher
geometry_msgs::Twist test_twist;
ros::Publisher pub_twist("rover_twist", &test_twist);
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
  initMotorSerial();
  initMotor(M_FL);
  initMotor(M_FR);
  initMotor(M_RL);
  initMotor(M_RR);

  /** GAMEPAD INIT **/
  initGamepad();

  /** ROS INIT **/
  nh.initNode();
  nh.advertise(pub_twist);
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
    readGamepad(gamepad); // Update gamepad

    // Control by gamepad
    if(isBtnPressed(gamepad, BTN_S_L2)) {
      if(isBtnPressed(gamepad, BTN_CROSS_U)) {
        toyDiffController(SPD_FWD, 0);
      }
      else if (isBtnPressed(gamepad, BTN_CROSS_D)) {
        toyDiffController(-SPD_FWD, 0);
      }
      else if (isBtnPressed(gamepad, BTN_CROSS_L)) {
        toyDiffController(0, SPD_ROT);
      }
      else if (isBtnPressed(gamepad, BTN_CROSS_R)) {
        toyDiffController(0, -SPD_ROT);
      }
      else {
        toyDiffController(0,0);
      }
    }

  }

  /** ROS OPERATION **/
  if(!isBtnPressed(gamepad, BTN_S_L2)) {
    if(millis() - lastSpin > spinInterval) {
      // Publish
      pub_twist.publish(&test_twist);

      // Spin
      nh.spinOnce();
    }
  }
}

/** ROS CALLBACK **/
void twistCallBack(const geometry_msgs::Twist& msg) {                                    
  float spd_fwd = clamp(msg.linear.x, -MAX_SPD_FWD, MAX_SPD_FWD);
  float spd_rot = clamp(msg.angular.z, -MAX_SPD_ROT, MAX_SPD_ROT);
  toyDiffController(spd_fwd, spd_rot);
}
