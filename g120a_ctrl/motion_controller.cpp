#include "motion_controller.h"
#include "motor_control.h"

void toyDiffController(float spd_lin, float spd_rot) {
  // Calculate wheel speeds in m/s
  float spd_L = -(spd_lin - spd_rot * wheel_y);
  float spd_R = spd_lin + spd_rot * wheel_y;

  // Convert wheel speeds to rpm
  int32_t rpm_L = (int32_t)(spd_L * 60.0 / wheel_c);
  int32_t rpm_R = (int32_t)(spd_R * 60.0 / wheel_c);

  // Set RPMs for motors
  motorSetRpm(M_FL, rpm_L);
  motorSetRpm(M_RL, rpm_L);
  motorSetRpm(M_FR, rpm_R);
  motorSetRpm(M_RR, rpm_R);
}