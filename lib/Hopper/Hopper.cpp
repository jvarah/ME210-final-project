/**
 * Hopper subsystem wrapper
 */

#include <Arduino.h>
#include <Hopper.h>

#define DEFAULT_ANGLE 180
#define DROP_ANGLE 135
#define DROP_ONE_TIME 175 // Was 125 before servo change
#define DROP_ALL_TIME 500

Hopper::Hopper() { _is_attached = false; }

Hopper::Hopper(Servo &servo, uint8_t servo_port) {
  _servo = servo;
  _servo.attach(servo_port);
  _servo.write(DEFAULT_ANGLE);
  _is_attached = true;
}

void Hopper::dropTwoBalls() {
  if (_is_attached) {
    _servo.write(DROP_ANGLE);
    delay(DROP_ONE_TIME);
    _servo.write(DEFAULT_ANGLE);
  } else {
    Serial.println("Servo not attached!");
  }
}

void Hopper::dropAllBalls() {
  if (_is_attached) {
    _servo.write(DROP_ANGLE);
    delay(DROP_ALL_TIME);
    _servo.write(DEFAULT_ANGLE);
  } else {
    Serial.println("Servo not attached!");
  }
}

bool Hopper::isInitialized() {
    return _is_attached;
}