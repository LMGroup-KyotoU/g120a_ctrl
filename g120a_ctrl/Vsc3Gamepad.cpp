#include "Vsc3Gamepad.h"
#include <cstring>  // memset

// Command presets
const uint8_t Vsc3Gamepad::CMD_config_mode_enter[] = {0x01, 0x43, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t Vsc3Gamepad::CMD_config_mode_exit[] =  {0x01, 0x43, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t Vsc3Gamepad::CMD_set_mode_and_lock[] = {0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00};
const uint8_t Vsc3Gamepad::CMD_read_data[] =         {0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

Vsc3Gamepad::Vsc3Gamepad() {
  // Start SPI
  m_spi_pad = spiStartBus(VSPI, 2500000, SPI_MODE2, SPI_LSBFIRST);
  // Pin setting
  pinMode(SPI_MISO, INPUT);
  pinMode(SPI_CS, OUTPUT);
  digitalWrite(SPI_CS, HIGH);
  spiAttachSCK(m_spi_pad, SPI_SCK);
  spiAttachMISO(m_spi_pad, SPI_MISO);
  spiAttachMOSI(m_spi_pad, SPI_MOSI);
  // Clear buffer
  memset(m_pad_buf, 0, DATA_LEN);
  // Change to analog mode if in digital mode
  _checkAnalogMode();
}

void Vsc3Gamepad::_readWrite(uint8_t* sendData, uint8_t* rcvData) {
  // Send read date to
  digitalWrite(SPI_CS, LOW);
  for(int i=0; i<DATA_LEN; i++) {
    spiTransferBytes(m_spi_pad, (uint8_t *)&sendData[i], (uint8_t *)&rcvData[i], 1);
    delayMicroseconds(10);
  }
  digitalWrite(5, HIGH);
}

// Change to analog mode if in digital mode
int Vsc3Gamepad::_checkAnalogMode() {
  if (m_pad_buf[ANF_ADDR] != 0x73) {
    _readWrite((uint8_t *)CMD_config_mode_enter, (uint8_t *)m_pad_buf);
    delay(1);
    _readWrite((uint8_t *)CMD_set_mode_and_lock, (uint8_t *)m_pad_buf);
    delay(1);
    _readWrite((uint8_t *)CMD_config_mode_exit, (uint8_t *)m_pad_buf);
    return 1;
  }
  return 0;
}

void Vsc3Gamepad::update() {
  // Check if too close to last read
  if (millis() - m_last_read_t < 15)  // ms
    return;
  m_last_read_t = millis();

  // Read data from the pad
  _readWrite((uint8_t *)CMD_read_data, (uint8_t *)m_pad_buf);

  if (_checkAnalogMode()) return;

  // Get button data
  m_gamepad.button = ~((m_pad_buf[BTN_ADDR] << 8) | m_pad_buf[BTN_ADDR + 1]);

  // Get joystick data
  int8_t tmp[4];
  for (int i = 0; i < 4; i++) {
    tmp[i] = (int8_t)m_pad_buf[ANS_ADDR + i];
    if ((uint8_t)tmp[i] <= 0x7F) 
      tmp[i] = 127 - tmp[i];
    else 
      tmp[i] = -(128 + tmp[i]);
  }

  m_gamepad.right_stick.x = tmp[0];
  m_gamepad.right_stick.y = tmp[1];
  m_gamepad.left_stick.x = tmp[2];
  m_gamepad.left_stick.y = tmp[3];
}

bool Vsc3Gamepad::isBtnPressed(uint16_t btnCode) {
  return (m_gamepad.button & btnCode) != 0x00;
}

int8_t Vsc3Gamepad::getLeftX() {
  return m_gamepad.left_stick.x;
}

int8_t Vsc3Gamepad::getLeftY() {
  return m_gamepad.left_stick.y;
}

int8_t Vsc3Gamepad::getRightX() {
  return m_gamepad.right_stick.x;
}

int8_t Vsc3Gamepad::getRightY() {
  return m_gamepad.right_stick.y;
}