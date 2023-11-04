#ifndef G120A_MOTION_CONTROLLER_H
#define G120A_MOTION_CONTROLLER_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include "G120aMotor.h"

/** DEFINE **/

class G120aMotionController {
public:
  /** PUBLIC FUNCTIONS **/
  G120aMotionController(HardwareSerial& motorSerial);
  ~G120aMotionController();
  void setMotion(float spd_lin, float spd_rot);

private:
  /** PRIVATE CONSTS **/
  static const uint8_t MOTOR_ID_FL = 1;
  static const uint8_t MOTOR_ID_FR = 2;
  static const uint8_t MOTOR_ID_RL = 3;
  static const uint8_t MOTOR_ID_RR = 4;
  static constexpr float WHEEL_CIRCUMFERENCE = 0.63774; // Circumference of the tire in m
  static constexpr float WHEEL_Y = 0.18;  // Half the distance between left and rigth wheels in m
  static constexpr float WHEEL_X = 0.20;  // Half the distance between front and rear wheels in m
  static constexpr uint32_t ENC_PER_TURN = 4096;
  static constexpr float ENC_PER_MM = static_cast<float>(ENC_PER_TURN) / WHEEL_CIRCUMFERENCE;

  /** PRIVATE CLASSES **/
  HardwareSerial& motorSerial;
  G120aMotor* m_motorFL;
  G120aMotor* m_motorFR;
  G120aMotor* m_motorRL;
  G120aMotor* m_motorRR;
};

#endif /* G120A_MOTION_CONTROLLER_H */