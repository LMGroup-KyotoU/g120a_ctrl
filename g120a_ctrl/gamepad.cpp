#include "gamepad.h"

#include <SPI.h>
#include <cstring>  // memset
#include <stdint.h>
#include <Arduino.h>

#define ANF_ADDR   1
#define BTN_ADDR   3
#define ANS_ADDR   5

#define DATA_LEN   9

// Command presets
const uint8_t CMD_config_mode_enter[] = {0x01, 0x43, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t CMD_config_mode_exit[] =  {0x01, 0x43, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t CMD_set_mode_and_lock[] = {0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00};
const uint8_t CMD_read_data[] =         {0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Data buffer
uint8_t pad_buf[DATA_LEN];

// Valuables
spi_t *spi_pad;
uint64_t last_read_t = 0;

void printPadStatus(Gamepad &pad_data) {
  Serial.print("button: ");
  Serial.print(pad_data.button, BIN);
  Serial.print(" right_x: ");
  Serial.print(pad_data.right_stick.x);
  Serial.print(" right_y: ");
  Serial.print(pad_data.right_stick.y);
  Serial.print(" left_x: ");
  Serial.print(pad_data.left_stick.x);
  Serial.print(" left_y: ");
  Serial.println(pad_data.left_stick.y);
}

void readWritePad(uint8_t* sendData, uint8_t* rcvData) {
  // Send read date to
  digitalWrite(SPI_CS, LOW);
  for(int i=0; i<DATA_LEN; i++) {
    spiTransferBytes(spi_pad, (uint8_t *)&sendData[i], (uint8_t *)&rcvData[i], 1);
    delayMicroseconds(10);
  }
  digitalWrite(5, HIGH);
}

// Change to analog mode if in digital mode
int checkAnalogMode() {
  if (pad_buf[ANF_ADDR] != 0x73) {
    readWritePad((uint8_t *)CMD_config_mode_enter, (uint8_t *)pad_buf);
    delay(1);
    readWritePad((uint8_t *)CMD_set_mode_and_lock, (uint8_t *)pad_buf);
    delay(1);
    readWritePad((uint8_t *)CMD_config_mode_exit, (uint8_t *)pad_buf);
    return 1;
  }
  return 0;
}

void initGamepad() {
  // Init
  spi_pad = spiStartBus(VSPI, 2500000, SPI_MODE2, SPI_LSBFIRST);

  // Pin setting
  pinMode(SPI_MISO, INPUT);
  pinMode(SPI_CS, OUTPUT);
  digitalWrite(SPI_CS, HIGH);
  spiAttachSCK(spi_pad, SPI_SCK);
  spiAttachMISO(spi_pad, SPI_MISO);
  spiAttachMOSI(spi_pad, SPI_MOSI);

  // Clear buffer
  memset(pad_buf, 0, DATA_LEN);

  // Change to analog mode if in digital mode
  checkAnalogMode();
}

void readGamepad(Gamepad &pad_data) {
  // Check if too close to last read
  if (millis() - last_read_t < 15)  // ms
    return;
  last_read_t = millis();

  // Read data from the pad
  readWritePad((uint8_t *)CMD_read_data, (uint8_t *)pad_buf);

  if (checkAnalogMode()) return;

  // Get button data
  pad_data.button = ~((pad_buf[BTN_ADDR] << 8) | pad_buf[BTN_ADDR + 1]);

  // Get joystick data
  int8_t tmp[4];
  for (int i = 0; i < 4; i++) {
    tmp[i] = (int8_t)pad_buf[ANS_ADDR + i];
    if ((uint8_t)tmp[i] <= 0x7F) 
      tmp[i] = 127 - tmp[i];
    else 
      tmp[i] = -(128 + tmp[i]);
  }

  pad_data.right_stick.x = tmp[0];
  pad_data.right_stick.y = tmp[1];
  pad_data.left_stick.x = tmp[2];
  pad_data.left_stick.y = tmp[3];
}

bool isBtnPressed(Gamepad &pad_data, uint16_t btnCode) {
  return (pad_data.button & btnCode) != 0x00;
}