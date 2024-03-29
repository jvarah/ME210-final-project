/**
 * Drive.h contains the exposed methods for the drive subsystem
 * Get/set gyro position
 * Set left/right motor
 */

#ifndef Drive_h
#define Drive_h
#include <Adafruit_I2CDevice.h>
#include <Adafruit_BNO055.h>
#include <inttypes.h>

class Drive {
 public:
  Drive(); // Allows for drive to be initialized in setup
  Drive(uint8_t left_dir_port, uint8_t left_en_port, uint8_t right_dir_port,
        uint8_t right_en_port, uint8_t gyro_prt);
  void setLeftInverted(uint8_t inverted);
  void setRightInverted(uint8_t inverted);
  void setLeftPower(int8_t power);
  void setRightPower(int8_t power);
  void stopMotors();
  double getAngle();
  void setGyroOffset(double offset);
  double calcTurnError(double target_angle);
  void setTurnPIDPowers(double target_angle, double k_p); 
  void printDebug();

 private:
  bool isLeftInverted();
  bool isRightInverted();
  bool leftDir();
  bool rightDir();
  void setRightDir(uint8_t dir);
  void setLeftDir(uint8_t dir);
  void setGyroInit();
  bool getGyroInit();
  uint8_t l_dir_port;
  uint8_t l_en_port;
  uint8_t r_dir_port;
  uint8_t r_en_port;
  uint8_t gyro_port;

  /* Structure of invertsAndState:
    0, 0, 0, gyroInitialized, LeftInverted, RightInverted, LeftDir, RightDir
      */
  uint8_t inverts_and_states;  // Each bit is different info
  Adafruit_BNO055 gyro;
  double gyro_offset;
};

#endif