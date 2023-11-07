#include "G120aMotionController.h"
#include "G120aMotor.h"

G120aMotionController::G120aMotionController(HardwareSerial &serial) : motorSerial(serial) {
  // Init driver ctrl pin
  pinMode(15, OUTPUT);
  digitalWrite(15, HIGH);

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

void G120aMotionController::setMotion(double spd_lin, double spd_rot) {
  // Calculate wheel speeds in m/s
  double spd_L = -(spd_lin - spd_rot * (WHEEL_X+WHEEL_Y)); // Need further calculation, only a approximation
  double spd_R = spd_lin + spd_rot * (WHEEL_X+WHEEL_Y); // Need further calculation, only a approximation
  // Convert wheel speeds to rpm
  double rpm_L = (double)(spd_L * 60.0 / G120aMotor::WHEEL_CIRCUMFERENCE);
  double rpm_R = (double)(spd_R * 60.0 / G120aMotor::WHEEL_CIRCUMFERENCE);
  // Set RPMs for motors
  m_motorFL->setRpm(rpm_L);
  m_motorFR->setRpm(rpm_R);
  m_motorRL->setRpm(rpm_L);
  m_motorRR->setRpm(rpm_R);
}

// Get linear velocity in x direction [m/s] and angular velocity in z direction [rads/s]
std::pair<double, double> G120aMotionController::_getLocalVelocity() {
  // Get linera velocity of each wheel
  double vel_lin_fl = -m_motorFL->getLinearVelocity();
  double vel_lin_fr = m_motorFR->getLinearVelocity();
  double vel_lin_rl = -m_motorRL->getLinearVelocity();
  double vel_lin_rr = m_motorRR->getLinearVelocity();
  // Calculate average linear velocity
  double vel_lin = (vel_lin_fl + vel_lin_fr + vel_lin_rl + vel_lin_rr) / 4.0;
  double vel_rot = (vel_lin_fr - vel_lin_fl + vel_lin_rr - vel_lin_rl) / (4.0 * (WHEEL_X+WHEEL_Y)); // Need further calculation, only a approximation
  return std::make_pair(vel_lin, vel_rot);
}

void G120aMotionController::updateOdom() {
  double current_time = micros();
  double elapsed_time = (double)(current_time - last_odom_read_time) / 1000000.0;
  // Get local velocity
  std::pair<double, double> local_vel = _getLocalVelocity();
  double disp_lin = local_vel.first * elapsed_time;
  double disp_ang = local_vel.second * elapsed_time;
  // Update accumulated position in x and y direction [m]
  accu_pos_x += disp_lin * cos(accu_rot_z + disp_ang / 2.0); // disp_ang/2 for mid point approximation
  accu_pos_y += disp_lin * sin(accu_rot_z + disp_ang / 2.0);
  // Update accumulated rotation in z direction [rad]
  accu_rot_z += disp_ang;
  // Update last read time
  last_odom_read_time = current_time;
}