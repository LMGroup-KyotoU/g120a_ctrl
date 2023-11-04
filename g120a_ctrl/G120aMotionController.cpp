#include "G120aMotionController.h"
#include "G120aMotor.h"

G120aMotionController::G120aMotionController(HardwareSerial &serial) : motorSerial(serial) {
  // Motor class
  m_motorFL = new G120aMotor(MOTOR_ID_FL, motorSerial);
  m_motorFR = new G120aMotor(MOTOR_ID_FR, motorSerial);
  m_motorRL = new G120aMotor(MOTOR_ID_RL, motorSerial);
  m_motorRR = new G120aMotor(MOTOR_ID_RR, motorSerial);
}

G120aMotionController::~G120aMotionController() {
  delete m_motorFL;
  delete m_motorFR;
  delete m_motorRL;
  delete m_motorRR;
}

void G120aMotionController::setMotion(float spd_lin, float spd_rot) {
  // Calculate wheel speeds in m/s
  float spd_L = -(spd_lin - spd_rot * WHEEL_Y);
  float spd_R = spd_lin + spd_rot * WHEEL_Y;

  // Convert wheel speeds to rpm
  int32_t rpm_L = (int32_t)(spd_L * 60.0 / WHEEL_CIRCUMFERENCE);
  int32_t rpm_R = (int32_t)(spd_R * 60.0 / WHEEL_CIRCUMFERENCE);

  // Set RPMs for motors
  m_motorFL->setRpm(rpm_L);
  m_motorFR->setRpm(rpm_R);
  m_motorRL->setRpm(rpm_L);
  m_motorRR->setRpm(rpm_R);
}