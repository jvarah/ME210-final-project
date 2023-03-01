#include <Arduino.h>
#include <SPI.h>
#include <Drive.h>

// Setup the hardware timer constants, needs to be before #include TimerInterrupt
#define USE_TIMER_1     false
#define USE_TIMER_2     true

#define PRINT_FREQ 1 // Hz

#include "TimerInterrupt.h"

// Motor constants
#define LEFT_MOTOR_DIR_PORT 7
#define LEFT_MOTOR_EN_PORT 6
#define RIGHT_MOTOR_DIR_PORT 5
#define RIGHT_MOTOR_EN_PORT 4

#define IS_LEFT_INVERTED 0
#define IS_RIGHT_INVERTED 1

#define FULL_SPEED 127
#define HALF_SPEED 64 // Max forward is 127

#define IS_TESTING_DRIVE 1
static bool isDriving;
static Drive drivebase = Drive(LEFT_MOTOR_DIR_PORT, LEFT_MOTOR_EN_PORT, RIGHT_MOTOR_DIR_PORT, RIGHT_MOTOR_EN_PORT);

void driveTest();

void outputSensorVals() {
  Serial.print("Gyro angle: ");
  Serial.println(drivebase.getAngle());
}

void setup() {
  Serial.begin(9600);
  while(!Serial);

  // Init drivebase
  drivebase.setLeftInverted(IS_LEFT_INVERTED);
  drivebase.setRightInverted(IS_RIGHT_INVERTED);
  drivebase.setGyroOffset(0.0);
  
  // Timer for slow printing
  ITimer2.init();
  ITimer2.setFrequency(PRINT_FREQ, outputSensorVals);
  isDriving = false;
}

void loop() {
  if (IS_TESTING_DRIVE && !isDriving) {
    isDriving = true;
    driveTest();
  } 
  // drivebase.setLeftPower(HALF_SPEED);
  // drivebase.setRightPower(HALF_SPEED);
  // put your main code here, to run repeatedly:
}

// Note: Very bad style, uses delay which stops program execution
void driveTest() {
  // Drive straight at half speed, then stop
  drivebase.setLeftPower(HALF_SPEED);
  drivebase.setRightPower(HALF_SPEED);
  delay(2000); // 2 Seconds
  drivebase.stopMotors();
  delay(500);

  // Turn 90 degrees left (roughly)
  drivebase.setLeftPower(-HALF_SPEED);
  drivebase.setRightPower(HALF_SPEED);
  while(drivebase.getAngle() < 90); // Turn until 90 degrees
  drivebase.stopMotors();
  delay(500);

  // Turn 90 degrees right (roughly)
  double lastAngle = drivebase.getAngle();
  drivebase.setLeftPower(HALF_SPEED);
  drivebase.setRightPower(-HALF_SPEED);
  // TODO: Find a way to determine diff between 360 and 0
  while(lastAngle - drivebase.getAngle() < 90); // Turn until 90 degrees
  drivebase.stopMotors();
  delay(500);
  
  // Drive straight at full power
  drivebase.setLeftPower(FULL_SPEED);
  drivebase.setRightPower(FULL_SPEED);
  delay(1000);
}