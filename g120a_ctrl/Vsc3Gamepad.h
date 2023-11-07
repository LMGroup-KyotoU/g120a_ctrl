#ifndef VSC3_GAMEPAD_H
#define VSC3_GAMEPAD_H

#include <stdint.h>
#include <SPI.h>
#include <Arduino.h>

class Vsc3Gamepad {
public:
  // Analog stick
  struct AStick {
      int8_t x;
      int8_t y;
  };

  // Gamepad data
  struct GamepadData {
      uint16_t button;
      AStick  right_stick;
      AStick  left_stick;
  };

  // VS-C3 gamepad button list
  enum padBtnList {
      BTN_CROSS_L = 0x8000,
      BTN_CROSS_D = 0x4000,
      BTN_CROSS_R = 0x2000,
      BTN_CROSS_U = 0x1000,
      BTN_START = 0x0800,
      BTN_ANBTN_R = 0x0400,
      BTN_ANBTN_L = 0x0200,
      BTN_SELECT = 0x0100,
      BTN_SQUARE = 0x0080,
      BTN_CROSS = 0x0040,
      BTN_CIRCLE = 0x0020,
      BTN_TRIANGLE = 0x0010,
      BTN_S_R1 = 0x0008,
      BTN_S_L1 = 0x0004,
      BTN_S_R2 = 0x0002,
      BTN_S_L2 = 0x0001
  };

  /** PUBLIC FUNCTIONS **/
  Vsc3Gamepad();
  void update();;
  bool isBtnPressed(uint16_t btnCode);
  int8_t getLeftX();
  int8_t getLeftY();
  int8_t getRightX();
  int8_t getRightY();
  
private:
  /** PRIVATE CONSTANTS **/
  static const uint8_t CMD_config_mode_enter[];
  static const uint8_t CMD_config_mode_exit[];
  static const uint8_t CMD_set_mode_and_lock[];
  static const uint8_t CMD_read_data[];
  static const uint8_t SPI_CS = 5;
  static const uint8_t SPI_SCK = 18;
  static const uint8_t SPI_MISO = 19;
  static const uint8_t SPI_MOSI = 23;
  static const uint8_t ANF_ADDR = 1;
  static const uint8_t BTN_ADDR = 3;
  static const uint8_t ANS_ADDR = 5;
  static const uint8_t DATA_LEN = 9;

  /** PRIVATE VARIABLES **/
  spi_t *m_spi_pad;
  uint64_t m_last_read_t;
  uint8_t m_pad_buf[DATA_LEN];
  GamepadData m_gamepad;

  /** PRIVATE FUNCTIONS **/
  void _readWrite(uint8_t* sendData, uint8_t* rcvData);
  int _checkAnalogMode();
};

#endif /* VSC3_GAMEPAD_H */
