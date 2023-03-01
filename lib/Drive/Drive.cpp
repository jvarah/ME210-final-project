/**
 * Drive.cpp is the implementation for the drive subsystem
 */
#include "Drive.h"

#define USE_EXT_CRYSTAL true

/* Structure of invertsAndState:
    0, 0, 0, GyroInit, LeftInverted, RightInverted, LeftDir, RightDir
*/
#define GYRO_INIT_MASK (uint8_t)0b10000
#define LEFT_INVERT_MASK (uint8_t)0b1000
#define RIGHT_INVERT_MASK (uint8_t)0b0100
#define LEFT_DIR_MASK (uint8_t)0b0010
#define RIGHT_DIR_MASK (uint8_t)0b0001

// DO NOT CHANGE THESE, bitmasks depend on forward = 0, backward = 1
#define FORWARD (uint8_t)0
#define BACKWARD (uint8_t)1

Drive::Drive(uint8_t left_dir_port, uint8_t left_en_port,
             uint8_t right_dir_port, uint8_t right_en_port) {
  // Gyro initialization
  gyro_offset = 0;

  // Motor initialization
  inverts_and_states = 0;
  l_dir_port = left_dir_port;
  l_en_port = left_en_port;
  r_dir_port = right_dir_port;
  r_en_port = right_en_port;

  pinMode(l_dir_port, OUTPUT);
  pinMode(l_en_port, OUTPUT);
  pinMode(r_dir_port, OUTPUT);
  pinMode(r_en_port, OUTPUT);
  digitalWrite(l_dir_port, LOW);
  digitalWrite(r_dir_port, LOW);
  analogWrite(l_en_port, LOW);
  analogWrite(r_en_port, LOW);

  setLeftDir(FORWARD);
  setRightDir(FORWARD);
  stopMotors();
}

void Drive::initGyro(Adafruit_BNO055 *gyro_ptr) {
  if (gyro_ptr != NULL) {
    gyro = gyro_ptr;
    setGyroInit();
  }
}

// Set direction of the motors
void Drive::setRightDir(uint8_t dir) {
  if (dir == BACKWARD) {
    inverts_and_states |= RIGHT_DIR_MASK;  // Turn on
    digitalWrite(r_dir_port, HIGH);
  } else {
    inverts_and_states &= ~RIGHT_DIR_MASK;  // Turn off
    digitalWrite(r_dir_port, LOW);
  }
}

// Set direction of the motors
void Drive::setLeftDir(uint8_t dir) {
  if (dir == BACKWARD) {
    inverts_and_states |= LEFT_DIR_MASK;  // Turn on
    digitalWrite(l_dir_port, HIGH);
  } else {
    inverts_and_states &= ~LEFT_DIR_MASK;  // Turn off
    digitalWrite(l_dir_port, LOW);
  }
}

bool Drive::isLeftInverted() { return inverts_and_states & LEFT_INVERT_MASK; }

bool Drive::isRightInverted() { return inverts_and_states & RIGHT_INVERT_MASK; }

bool Drive::leftDir() { return inverts_and_states & LEFT_DIR_MASK; }

bool Drive::rightDir() { return inverts_and_states & RIGHT_DIR_MASK; }

void Drive::setGyroInit() { inverts_and_states |= GYRO_INIT_MASK; }

bool Drive::getGyroInit() { return inverts_and_states & GYRO_INIT_MASK; }

void Drive::setLeftInverted(uint8_t inverted) {
  if (inverted == BACKWARD) {
    inverts_and_states |= LEFT_INVERT_MASK;  // Turn on invert
  } else {
    inverts_and_states &= ~LEFT_INVERT_MASK;  // Turn off invert
  }
}

void Drive::setRightInverted(uint8_t inverted) {
  if (inverted == BACKWARD) {
    inverts_and_states |= RIGHT_INVERT_MASK;  // Turn on invert
  } else {
    inverts_and_states &= ~RIGHT_INVERT_MASK;  // Turn off invert
  }
}

// Set left power between -128 and 127
void Drive::setLeftPower(int8_t power) {
  // Make motor go forward with positive power, or backward if it's negative
  // power
  if (power >= 0) {
    if (isLeftInverted()) {
      setLeftDir(BACKWARD);
    } else {
      setLeftDir(FORWARD);
    }
  }
  // Power is negative, change direction of the motor
  else {
    if (isLeftInverted()) {
      setLeftDir(FORWARD);
    } else {
      setLeftDir(BACKWARD);
    }
  }

  // Set PWM power, converting the negative/positive range into a 0-255 output
  // Input is between -127 to 128, which shows direction and magnitude
  // Want the power to be between 0 to 255, so multiply abs(input) by 2
  analogWrite(l_en_port, uint8_t(abs(power)) << 1);
}

// Set right power between -128 and 127
void Drive::setRightPower(int8_t power) {
  // Make motor go forward with positive power, or backward if it's negative
  // power
  if (power >= 0) {
    if (isRightInverted()) {
      setRightDir(BACKWARD);
    } else {
      setRightDir(FORWARD);
    }
  }
  // Power is negative, change direction of the motor
  else {
    if (isRightInverted()) {
      setRightDir(FORWARD);
    } else {
      setRightDir(BACKWARD);
    }
  }

  // Set PWM power, converting the negative/positive range into a 0-255 output
  // Input is between -127 to 128, which shows direction and magnitude
  // Want the power to be between 0 to 255, so multiply abs(input) by 2
  analogWrite(r_en_port, uint8_t(abs(power)) << 1);
}

void Drive::stopMotors() {
  setRightPower(0);
  setLeftPower(0);
}

double Drive::getAngle() {
  if (getGyroInit()) {
    imu::Vector<3> euler = gyro->getVector(Adafruit_BNO055::VECTOR_EULER);
    return euler.x() - gyro_offset;  // Yaw
  } 
  return -1; // Error
}

void Drive::setGyroOffset(double offset) {
  gyro_offset = offset;  // Can be used so 0 is always when the robot IR sensor
                         // is facing the beacon
}
