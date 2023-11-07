#include "G120aMotor.h"
#include "G120aMotionController.h"
#include "utils.h"

/** ROS INCLUDE **/
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16MultiArray.h>

/** ROS DEFINE **/
#define CTRL_FREQ 50
#define SPIN_FREQ 30

/** ROVER DEFINE **/
#define MAX_SPD_FWD 0.4 // maximum forward speed in m/s
#define MAX_SPD_ROT 0.8 // maximum rotate speed in rad/s
#define SPD_FWD 0.4
#define SPD_ROT 0.8

/** ROS DECLARATION **/
void twistCallBack(const geometry_msgs::Twist& msg);
uint32_t last_cmd_time = 0;

/** MOTOR PARAMS **/
HardwareSerial motorSerial(2);
G120aMotionController* g120aMotionController = nullptr;

/** ROS PARAMS **/
ros::NodeHandle nh;
uint64_t lastSpin = millis();
const uint32_t spinInterval = 1000 / SPIN_FREQ;
// Publisher
geometry_msgs::Twist rover_odom;
ros::Publisher pub_odom("/rover/odometry", &rover_odom);
// Subscriber
ros::Subscriber<geometry_msgs::Twist> sub_twist("/rover/cmd_vel", &twistCallBack);

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
  motorSerial.begin(115200, SERIAL_8N1, 16, 17);
  g120aMotionController = new G120aMotionController(motorSerial);

  /** ROS INIT **/
  nh.initNode();
  nh.advertise(pub_odom);
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
    g120aMotionController->updateOdom(); // Update odometry information

    // Safety watchdog
    if(millis()-last_cmd_time > 200) {
      g120aMotionController->setMotion(0, 0);
    }
  }

  /** ROS OPERATION **/
  if(millis() - lastSpin > spinInterval) {
    lastSpin = millis();
    // Publish odom
    rover_odom.linear.x = g120aMotionController->accu_pos_x;
    rover_odom.linear.y = g120aMotionController->accu_pos_y;
    rover_odom.angular.z = g120aMotionController->accu_rot_z;
    pub_odom.publish(&rover_odom);

    // Spin
    nh.spinOnce();
  }
}

/** ROS CALLBACK **/
void twistCallBack(const geometry_msgs::Twist& msg) {
  // Safety
  last_cmd_time = millis();

  // Send control command
  float spd_fwd = clamp(msg.linear.x, -MAX_SPD_FWD, MAX_SPD_FWD);
  float spd_rot = clamp(msg.angular.z, -MAX_SPD_ROT, MAX_SPD_ROT);
  g120aMotionController->setMotion(spd_fwd, spd_rot);
}
