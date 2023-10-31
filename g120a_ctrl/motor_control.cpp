#include "motor_control.h"

HardwareSerial motorSerial(2);

// Check sum and put the result on data[9]
void _motorCheckSum(uint8_t* data) {
  uint16_t sum = 0;
  for(int i = 0; i < 9; i++) sum += data[i];
  data[9] = (uint8_t)sum;
}

void readWriteMotor(uint8_t id, uint8_t* send_buf, uint8_t* recv_buf) {
  // Set motor id
  send_buf[0] = id;

  // Send data to motor
  if(send_buf != nullptr) {
    _motorCheckSum(send_buf);
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

  // Wait for communication
  delay(10);
}

void writeMotor(uint8_t id, uint8_t* send_buf) {
  // Set motor id
  send_buf[0] = id;

  // Send data to motor
  if(send_buf != nullptr) {
    _motorCheckSum(send_buf);
    while(!motorSerial.availableForWrite())
      delayMicroseconds(100);
    motorSerial.write(send_buf, 10);
  }

  // Wait for communication
  delay(10);
}

void initMotorSerial() {
  // Motor control enable
  pinMode(PIN_DRV_EN, OUTPUT);
  digitalWrite(PIN_DRV_EN, HIGH);

  // Set motor serial
  motorSerial.begin(115200, SERIAL_8N1, 16, 17);
}

void initMotor(uint8_t id) {
  // Write control settings
  writeMotor(id, cmd_tacc);
  writeMotor(id, cmd_tdacc);
  writeMotor(id, cmd_mode);
  delay(20); // Without delay, motor won't response
  writeMotor(id, cmd_en);
  motorSetRpm(id, 0);
}

// Setting motor speed (rounds per minuite)
void motorSetRpm(uint8_t id, int32_t rpm) {
  uint8_t send_com[10] = {0x00, 0x54, 0x70, 0xB2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  int32_t dec = rpm * 512 * 4096 / 1875; // [DEC]=([rpm]*512*[反馈精度])/1875, [反馈精度]默认值:4096
  send_com[5] = dec >> 24;
  send_com[6] = dec >> 16;
  send_com[7] = dec >> 8;
  send_com[8] = dec;
  writeMotor(id, send_com);
}
