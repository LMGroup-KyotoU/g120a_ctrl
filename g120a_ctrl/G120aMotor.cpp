#include <vector>
#include "G120aMotor.h"

/** PRIVATE CONSTANTS **/
const uint8_t G120aMotor::CMD_CLRW[10]   = {0x00, 0x52, 0x70, 0x19, 0x00, 0x00, 0x00, 0x00, 0x86, 0x00}; // Clear driver warning
const uint8_t G120aMotor::CMD_TACC[10]   = {0x00, 0x54, 0x70, 0x99, 0x00, 0x00, 0x00, 0x00, 0xC9, 0x00}; // Trapezoid acc DEC = rps/s * 256 * 4096
const uint8_t G120aMotor::CMD_TDACC[10]  = {0x00, 0x54, 0x70, 0x9a, 0x00, 0x00, 0x00, 0x00, 0xC9, 0x00}; // Trapezoid Deacc DEC = rps/s * 256 * 4096
const uint8_t G120aMotor::CMD_MODE[10]   = {0x00, 0x51, 0x70, 0x17, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00}; // Working mode set to speed control with acceleration
const uint8_t G120aMotor::CMD_EN[10]     = {0x00, 0x52, 0x70, 0x19, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x00}; // Motor enable
const uint8_t G120aMotor::CMD_DEN[10]    = {0x00, 0x52, 0x70, 0x19, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00}; // Motor disenable
const uint8_t G120aMotor::CMD_RENC[10]   = {0x00, 0xA0, 0x70, 0x71, 0x00, 0x00, 0x00, 0x00, 0x86, 0x00}; // Read encoder position 0x7071
const uint8_t G120aMotor::CMD_RENCRES[10]= {0x00, 0xA0, 0x70, 0x33, 0x00, 0x00, 0x00, 0x00, 0x86, 0x00}; // Read encoder resolution 0x7033

G120aMotor::G120aMotor(uint8_t motorId, HardwareSerial& serial) : motorSerial(serial), m_motorId(motorId) {
  _readWrite(CMD_TACC); // Set acceleration
  _readWrite(CMD_TDACC); // Set deacceleration
  _readWrite(CMD_MODE); // Set mode
  delay(20); // Wait for the motor to be ready
  setRpm(0); // Set speed to 0
  _updateEncPos(); // Init encoder position
  _readWrite(CMD_EN); // Enable motor
}

// Setting motor speed (rounds per minuite)
void G120aMotor::setRpm(double rpm) {
  uint8_t send_buf[10] = {0x00, 0x54, 0x70, 0xB2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  int32_t dec = rpm * 512 * 4096 / 1875; // [DEC]=([rpm]*512*[反馈精度])/1875, [反馈精度]默认值:4096
  send_buf[5] = dec >> 24;
  send_buf[6] = dec >> 16;
  send_buf[7] = dec >> 8;
  send_buf[8] = dec;
  _readWrite(send_buf);
}

double G120aMotor::getLinearVelocity() {
  // Read and write to motor
  const uint8_t* send_buf = CMD_RENC;
  std::vector<uint8_t> recv_buf = _readWrite(send_buf);
  // Get read value from received message
  int32_t enc_pos = (recv_buf[8]) | 
                    (recv_buf[7] << 8) |
                    (recv_buf[6] << 16) |
                    (recv_buf[5] << 24);
  uint32_t enc_read_time = micros();
  double elapsed_time = static_cast<double>(enc_read_time - m_last_enc_read);
  // Calculate velocity
  double lin_vel = ((static_cast<double>(enc_pos - m_last_enc_pos)/ENC_PER_TURN)*WHEEL_CIRCUMFERENCE)/(elapsed_time/1000000.0);
  // Record
  m_last_enc_pos = enc_pos;
  m_last_enc_read = enc_read_time;
  return lin_vel;
}

// Update encoder position
void G120aMotor::_updateEncPos() {
  // Read and write to motor
  const uint8_t* send_buf = CMD_RENC;
  std::vector<uint8_t> recv_buf = _readWrite(send_buf);
  // Get read value from received message
  int32_t enc_pos = (recv_buf[8]) | 
                    (recv_buf[7] << 8) |
                    (recv_buf[6] << 16) |
                    (recv_buf[5] << 24);
  // Record
  m_last_enc_pos = enc_pos;
  m_last_enc_read = micros();
} 

// Check sum and put the result on data[9]
void G120aMotor::_checkSum(uint8_t* data_buf) {
  uint16_t sum = 0;
  for(int i = 0; i < 9; i++) sum += data_buf[i];
  data_buf[9] = (uint8_t)sum;
}

std::vector<uint8_t> G120aMotor::_readWrite(const uint8_t* data) {
  std::vector<uint8_t> recv_buf;
  uint8_t send_buf[10];
  memcpy(send_buf, data, 10);
  send_buf[0] = m_motorId;
  _checkSum(send_buf);

  // Clear any existing data from serial buffer to prevent reading stale data
  motorSerial.flush();

  // Send to the motor
  motorSerial.write(send_buf, 10);
  motorSerial.flush(false); // Ensure all data is sent
  digitalWrite(15, LOW); // Request to send

  // Wait for a response with timeout
  unsigned long startTime = millis();
  while(motorSerial.available() < 10) {
    if(millis() - startTime > 100) { // Timeout after 100 ms
      digitalWrite(15, HIGH); // Not ready to send
      // Handle the timeout, return all 0
      return std::vector<uint8_t>(10, 0);
    }
  }

  digitalWrite(15, HIGH); // Not ready to send

  // Read from the motor
  for(int i = 0; i < 10; i++) {
    if(motorSerial.available()) {
      recv_buf.push_back(motorSerial.read());
    }
  }
  
  return recv_buf;
}