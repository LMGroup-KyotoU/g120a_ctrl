#ifndef G120A_MOTION_CONTROLLER_H
#define G120A_MOTION_CONTROLLER_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <vector>
#include "G120aMotor.h"

/** DEFINE **/

class G120aMotionController {
public:
  /** PUBLIC FUNCTIONS **/
  G120aMotionController(HardwareSerial& motorSerial);
  ~G120aMotionController();
  void setMotion(double spd_lin, double spd_rot);
  void updateOdom();
  std::pair<double, double> _getLocalVelocity();

  /** PUBLIC VARIABLES **/
  double accu_rot_z = 0.0; // Accumulated rotation in z direction [rad]
  double accu_pos_x = 0.0; // Accumulated position in x direction [m]
  double accu_pos_y = 0.0; // Accumulated position in y direction [m]

private:
  /** PRIVATE CONSTS **/
  static const uint8_t MOTOR_ID_FL = 1;
  static const uint8_t MOTOR_ID_FR = 2;
  static const uint8_t MOTOR_ID_RL = 3;
  static const uint8_t MOTOR_ID_RR = 4;
  static constexpr float WHEEL_Y = 0.18;  // Half the distance between left and right wheels in m
  static constexpr float WHEEL_X = 0.20;  // Half the distance between front and rear wheels in m

  /** PRIVATE CLASSES **/
  HardwareSerial& motorSerial;
  G120aMotor* m_motorFL;
  G120aMotor* m_motorFR;
  G120aMotor* m_motorRL;
  G120aMotor* m_motorRR;

  /** PRIVATE VARIABLES **/
  uint32_t last_odom_read_time = 0; //[usec]
};

#endif /* G120A_MOTION_CONTROLLER_H */