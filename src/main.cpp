#include <Arduino.h>
#include <Drive.h>
#include <SPI.h>

// Setup the hardware timer constants, needs to be before #include
// TimerInterrupt
#define USE_TIMER_1 false
#define USE_TIMER_2 true

#define PRINT_FREQ 1.0  // Hz

#include "TimerInterrupt.h"

// Motor constants
#define LEFT_MOTOR_DIR_PORT 7
#define LEFT_MOTOR_EN_PORT 10
#define RIGHT_MOTOR_EN_PORT 5
#define RIGHT_MOTOR_DIR_PORT 4

#define IS_LEFT_INVERTED 1 // was 0
#define IS_RIGHT_INVERTED 0 // was 1

#define FULL_SPEED 127
#define HALF_SPEED 64  // 64 // Max forward is 127

#define IS_TESTING_DRIVE 0  // Set to 1 to test the driving
static bool isDriving;
static Drive drivebase = Drive(LEFT_MOTOR_DIR_PORT, LEFT_MOTOR_EN_PORT,
                               RIGHT_MOTOR_DIR_PORT, RIGHT_MOTOR_EN_PORT);

#define GYRO_PORT 0x28
static Adafruit_BNO055 gyro = Adafruit_BNO055(GYRO_PORT);

typedef enum {
  DRIVE_FORWARD,
  TURN_RIGHT,
  TURN_LEFT,
  STOP,
  DRIVE_BACKWARD,
  NOTHING
} States_t;

static States_t state = NOTHING;

void driveTest();

void outputSensorVals() {
  Serial.print("Gyro angle: ");
  Serial.println(drivebase.getAngle());
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("Code started");

  // Init gyro to pass to drivebase
  if (!gyro.begin()) {
    Serial.println("No gyro detected");
    while (1);
  }
  gyro.setExtCrystalUse(true);
  drivebase.initGyro(&gyro); // TODO: Maybe not include gyro in the drivebase?

  // Init drivebase
  drivebase.setLeftInverted(IS_LEFT_INVERTED);
  drivebase.setRightInverted(IS_RIGHT_INVERTED);
  isDriving = false;

  Serial.println("Drivebase initialized");
}

void loop() {
  // if (IS_TESTING_DRIVE && !isDriving) {
  //   isDriving = true;
  //   driveTest();
  // }

  // Test forwards, backwards, left right
  if (Serial.available()) {
    uint8_t aKey = Serial.read();
    Serial.println(aKey);
    switch (aKey) {
      case 108:  // l
        state = TURN_LEFT;
        drivebase.setLeftPower(-FULL_SPEED);
        drivebase.setRightPower(FULL_SPEED);
        break;
      case 102:  // f
        state = DRIVE_FORWARD;
        drivebase.setLeftPower(FULL_SPEED);
        drivebase.setRightPower(FULL_SPEED);
        break;
      case 98:  // b
        state = DRIVE_BACKWARD;
        drivebase.setLeftPower(-FULL_SPEED);
        drivebase.setRightPower(-FULL_SPEED);
        break;
      case 115:  // s
        state = STOP;
        drivebase.stopMotors();
        break;
      case 114:  // r
        state = TURN_RIGHT;
        drivebase.setLeftPower(FULL_SPEED);
        drivebase.setRightPower(-FULL_SPEED);
        break;
    }
  }

  // if (millis() % 1000L == 0) {
  //   outputSensorVals();
  // }
}

// Note: Very bad style, uses delay which stops program execution
void driveTest() {
  // Drive straight at half speed, then stop
  Serial.println("Going forward");
  drivebase.setLeftPower(HALF_SPEED);
  drivebase.setRightPower(HALF_SPEED);
  delay(2000);  // 2 Seconds
  drivebase.stopMotors();
  Serial.println("Stopped motors");
  delay(500);

  // Turn 90 degrees right (roughly)
  Serial.println("Turning right 90 deg");
  drivebase.setLeftPower(HALF_SPEED);
  drivebase.setRightPower(-HALF_SPEED);
  uint8_t count = 0;
  while (drivebase.getAngle() < 90) {  // Turn until 90 degrees
    if (count == 0) {                  // Print every 255 cycles
      Serial.println(drivebase.getAngle());
    }
    count++;
  }
  drivebase.stopMotors();
  delay(500);

  // Turn 45 degrees left (roughly)
  Serial.println("Turning left 90 deg");
  double lastAngle = drivebase.getAngle();
  drivebase.setLeftPower(-HALF_SPEED);
  drivebase.setRightPower(HALF_SPEED);
  // TODO: Find a way to determine diff between 360 and 0
  while (lastAngle - drivebase.getAngle() < 45) {
    if (count == 0) {
      Serial.println(drivebase.getAngle());
    }
    count++;
  }  // Turn until 90 degrees
  drivebase.stopMotors();
  delay(500);

  Serial.println("Full power forward");
  // Drive straight at full power
  drivebase.setLeftPower(FULL_SPEED);
  drivebase.setRightPower(FULL_SPEED);
  delay(10000);
}