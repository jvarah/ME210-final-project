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

#define IS_LEFT_INVERTED 1   // was 0
#define IS_RIGHT_INVERTED 0  // was 1

#define FULL_SPEED 127
#define HALF_SPEED 64  // 64 // Max forward is 127

#define IS_TESTING_DRIVE 0  // Set to 1 to test the driving
static bool isDriving;
static Drive drivebase;

#define GYRO_PORT 0x28

// PID Constants
#define MAX_TURN_ERROR 1.0 // Degrees
// #define TURN_K_P 1/180.0 // Full motor output at the max error (need to do a full turn)

#define START_BUTTON_PORT 0
#define START_BUTTON_LED_PORT 12
#define DEBOUNCE_DELAY 50
int startButtonVal = 0;
unsigned long lastStartButtonDBTime = 0;

int startButtonLEDState = LOW;
int startButtonState;
int lastStartButtonState = HIGH;


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
  while (!Serial)
    ;

  Serial.println("Code started");

  // Init drivebase
  drivebase = Drive(LEFT_MOTOR_DIR_PORT, LEFT_MOTOR_EN_PORT,
                    RIGHT_MOTOR_DIR_PORT, RIGHT_MOTOR_EN_PORT, GYRO_PORT);

  drivebase.setLeftInverted(IS_LEFT_INVERTED);
  drivebase.setRightInverted(IS_RIGHT_INVERTED);
  isDriving = false;

  Serial.println("Drivebase initialized");

  pinMode(START_BUTTON_PORT, INPUT);
  pinMode(START_BUTTON_LED_PORT, OUTPUT);
  digitalWrite(START_BUTTON_LED_PORT, LOW);

  Serial.println("Start button initialized");
}

void checkStartButton() {
  // get current reading
  startButtonVal = digitalRead(START_BUTTON_PORT);
  // If the switch changed, due to noise or pressing:
  if (startButtonVal != lastStartButtonState) {
    // reset the debouncing timer
    lastStartButtonDBTime = millis();
  }
  if ((millis() - lastStartButtonDBTime) > DEBOUNCE_DELAY) {
  // whatever the reading is at, it's been there for longer than the debounce
  // delay, so take it as the actual current state:

    // if the button state has changed:
    if (startButtonVal != startButtonState) {
      startButtonState = startButtonVal;
      Serial.println("Button pressed!");

      // only toggle the LED if the new button state is HIGH
      if (startButtonState == HIGH) startButtonLEDState = LOW;
      else startButtonLEDState = HIGH;
    }
  }

  // set the LED:
  digitalWrite(START_BUTTON_LED_PORT, startButtonLEDState);

  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastStartButtonState = startButtonVal;
}

void loop() {
  checkStartButton();
  // End to end drive test
  if (IS_TESTING_DRIVE && !isDriving) {
    isDriving = true;
    driveTest();
  }

  // Test forwards, backwards, left right
  // For drive testing (no gyro)
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
      case 103: // g
        drivebase.setGyroOffset(drivebase.getAngle()); // Zero the gyro
        break;
    }
  }

  // Periodic print without using Timer2 (which messes with deploying the code)
  if (millis() % 1000L == 0) {
    outputSensorVals();
  }
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

  // Turn to (roughly) 90 degrees
  Serial.println("Turning right 90 deg");
  drivebase.setLeftPower(HALF_SPEED);
  drivebase.setRightPower(-HALF_SPEED);
  uint8_t count = 0;
  uint8_t target = 90;
  double error = 0.0;
  do {
    error = drivebase.calcTurnError(target);
    if (count == 0) {                  // Print every 255 cycles
      Serial.println(drivebase.getAngle());
    }
    count++;
  } while (abs(error) >= MAX_TURN_ERROR * 10);
  drivebase.stopMotors();
  delay(500);

  // Turn (roughly) to 45 degrees
  Serial.println("Turning left 45 deg");
  target = 45;
  drivebase.setLeftPower(-HALF_SPEED);
  drivebase.setRightPower(HALF_SPEED);
  
  // Keep turning until you got there
  do {
    error = drivebase.calcTurnError(target);
    if (count == 0) {                  // Print every 255 cycles
      Serial.println(drivebase.getAngle());
    }
    count++;
  } while (abs(error) >= MAX_TURN_ERROR * 10);
  drivebase.stopMotors();
  delay(500);

  Serial.println("Full power forward");
  // Drive straight at full power
  drivebase.setLeftPower(FULL_SPEED);
  drivebase.setRightPower(FULL_SPEED);
  delay(1000);
}