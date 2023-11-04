#ifndef G120A_MOTOR_H
#define G120A_MOTOR_H

#include <Arduino.h>
#include <HardwareSerial.h>

class G120aMotor {
public:
  /** PUBLIC FUNCTIONS **/
  G120aMotor(uint8_t motorId, HardwareSerial& serial);
  void initMotor();
  void setRpm(double rpm);
  
private:
  HardwareSerial& motorSerial;
  uint8_t m_motorId;

  /** PRIVATE CONSTANTS **/
  static const uint8_t CMD_CLRW[10];  // Clear driver warning
  static const uint8_t CMD_TACC[10];  // Trapezoid acc DEC = rps/s * 256 * 4096
  static const uint8_t CMD_TDACC[10]; // Trapezoid Deacc DEC = rps/s * 256 * 4096
  static const uint8_t CMD_MODE[10];  // Working mode set to speed control with acceleration
  static const uint8_t CMD_EN[10];    // Motor enable

  /** PRIVATE FUNCTIONS **/
  void _checkSum(uint8_t* data);
  void _write(const uint8_t* send_buf);
  void _readWrite(const uint8_t* send_buf, uint8_t* recv_buf);
};

#endif  // G120A_MOTOR_H
