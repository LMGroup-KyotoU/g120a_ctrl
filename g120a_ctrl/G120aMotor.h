#ifndef G120A_MOTOR_H
#define G120A_MOTOR_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <vector>

class G120aMotor {
public:
  /** PUBLIC CONSTANTS **/
  static constexpr double WHEEL_CIRCUMFERENCE = 0.63774; // Circumference of the tire in m
  static constexpr uint32_t ENC_PER_TURN = 4096;
  static constexpr double ENC_PER_MM = static_cast<double>(ENC_PER_TURN) / WHEEL_CIRCUMFERENCE;

  /** PUBLIC FUNCTIONS **/
  G120aMotor(uint8_t motorId, HardwareSerial& serial);
  void initMotor();
  void setRpm(double rpm);
  double getLinearVelocity();
  
private:
  /** PRIVATE VARIABLES **/
  HardwareSerial& motorSerial;
  uint8_t m_motorId;
  int32_t m_last_enc_pos;
  uint32_t m_last_enc_read; //[usec]]
  
  /** PRIVATE CONSTANTS **/
  static const uint8_t CMD_CLRW[10];    // Clear driver warning
  static const uint8_t CMD_TACC[10];    // Trapezoid acc DEC = rps/s * 256 * 4096
  static const uint8_t CMD_TDACC[10];   // Trapezoid Deacc DEC = rps/s * 256 * 4096
  static const uint8_t CMD_MODE[10];    // Working mode set to speed control with acceleration
  static const uint8_t CMD_EN[10];      // Motor enable
  static const uint8_t CMD_DEN[10];     // Motor disenable
  static const uint8_t CMD_RENC[10];    // Read encoder position 0x7071
  static const uint8_t CMD_RENCRES[10]; // Read encoder resolution 0x7033

  /** PRIVATE FUNCTIONS **/
  void _checkSum(uint8_t* data);
  std::vector<uint8_t> _readWrite(const uint8_t* send_buf);
  void _updateEncPos();
};

#endif  // G120A_MOTOR_H
