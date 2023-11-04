#include "G120aMotor.h"

/** PRIVATE CONSTANTS **/
const uint8_t G120aMotor::CMD_CLRW[10]   = {0x00, 0x52, 0x70, 0x19, 0x00, 0x00, 0x00, 0x00, 0x86, 0x00}; // Clear driver warning
const uint8_t G120aMotor::CMD_TACC[10]   = {0x00, 0x54, 0x70, 0x99, 0x00, 0x00, 0x00, 0x00, 0xC9, 0x00}; // Trapezoid acc DEC = rps/s * 256 * 4096
const uint8_t G120aMotor::CMD_TDACC[10]  = {0x00, 0x54, 0x70, 0x9a, 0x00, 0x00, 0x00, 0x00, 0xC9, 0x00}; // Trapezoid Deacc DEC = rps/s * 256 * 4096
const uint8_t G120aMotor::CMD_MODE[10]   = {0x00, 0x51, 0x70, 0x17, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00}; // Working mode set to speed control with acceleration
const uint8_t G120aMotor::CMD_EN[10]     = {0x00, 0x52, 0x70, 0x19, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x00}; // Motor enable

G120aMotor::G120aMotor(uint8_t motorId, HardwareSerial& serial) : motorSerial(serial), m_motorId(motorId) {
  _write(CMD_TACC);
  _write(CMD_TDACC);
  _write(CMD_MODE);
  delay(20);
  _write(CMD_EN);
  setRpm(0);
}

// Check sum and put the result on data[9]
void G120aMotor::_checkSum(uint8_t* data_buf) {
  uint16_t sum = 0;
  for(int i = 0; i < 9; i++) sum += data_buf[i];
  data_buf[9] = (uint8_t)sum;
}

void G120aMotor::_write(const uint8_t* data) {
  // Create a mutable copy of the command
  uint8_t send_buf[10];
  memcpy(send_buf, data, 10);
  // Set motor id
  send_buf[0] = m_motorId;
  // Send data to motor
  if(send_buf != nullptr) {
    _checkSum(send_buf);
    while(!motorSerial.availableForWrite())
      delayMicroseconds(100);
    motorSerial.write(send_buf, 10);
  }
  delay(10);    // Wait for communication
}

void G120aMotor::_readWrite(const uint8_t* data, uint8_t* recv_buf) {
  // Create a mutable copy of the command
  uint8_t send_buf[10];
  memcpy(send_buf, data, 10);
  // Set motor id
  send_buf[0] = m_motorId;
  // Send data to motor
  if(send_buf != nullptr) {
    _checkSum(send_buf);
    while(!motorSerial.availableForWrite())
      delayMicroseconds(100);
    motorSerial.write(send_buf, 10);
  }
  // Receive data from motor
  if(recv_buf != nullptr) {
    while(motorSerial.available() < 10)
      delayMicroseconds(100);
    motorSerial.read(recv_buf, 10);
  }
  delay(10);    // Wait for communication
}

// Setting motor speed (rounds per minuite)
void G120aMotor::setRpm(double rpm) {
  uint8_t send_buf[10] = {0x00, 0x54, 0x70, 0xB2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  int32_t dec = rpm * 512 * 4096 / 1875; // [DEC]=([rpm]*512*[反馈精度])/1875, [反馈精度]默认值:4096
  send_buf[5] = dec >> 24;
  send_buf[6] = dec >> 16;
  send_buf[7] = dec >> 8;
  send_buf[8] = dec;
  _write(send_buf);
}
