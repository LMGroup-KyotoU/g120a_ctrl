#ifndef GAMEPAD_H
#define GAMEPAD_H

#include <stdint.h>
#include <SPI.h>
#include <Arduino.h>

#define SPI_CS 5
#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23

// Analog stick
struct AStick{
  int8_t x;
  int8_t y;
};

// Gamepad
struct Gamepad{
  uint16_t button;
  AStick  right_stick;
  AStick  left_stick;
};

/**
 * VS-C3 gamepad button list
 * CROSS  -> left four cross buttons
 * START  -> 
 * ANBTN  -> Analog button push
 * SELECT -> 
 * SQUARE -> 
 * CROSS  -> right single cross button
 * CIRCLE -> 
 * TRIANGLE -> 
 * S_     -> shoulder buttons
 */
enum padBtnList{
  BTN_CROSS_L = 0x8000,
  BTN_CROSS_D = 0x4000,
  BTN_CROSS_R = 0x2000,
  BTN_CROSS_U = 0x1000,
  BTN_START = 0x0800,
  BTN_ANBTN_R = 0x0400,
  BTN_ANBTN_L = 0x0200,
  BTN_SELECT = 0x0100,
  BTN_SQUARE = 0x0080,
  BTN_CROSS  = 0x0040,
  BTN_CIRCLE = 0x0020,
  BTN_TRIANGLE = 0x0010,
  BTN_S_R1   = 0x0008,
  BTN_S_L1   = 0x0004,
  BTN_S_R2   = 0x0002,
  BTN_S_L2   = 0x0001
};

extern spi_t *spi_pad;
extern uint64_t last_read_t;

void readWritePad(uint8_t* sendData, uint8_t* rcvData);
int checkAnalogMode();
void initGamepad();
void readGamepad(Gamepad &pad_data);
void printPadStatus(Gamepad &pad_data);
bool isBtnPressed(Gamepad &pad_data, uint16_t btnCode);

#endif /* GAMEPAD_H */