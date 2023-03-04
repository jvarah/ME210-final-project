#include <Arduino.h>
#include <Drive.h>
#include <LineFollow.h>
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

#define GYRO_PORT 0x28

// PID Constants
#define MAX_TURN_ERROR 1.0  // Degrees
#define K_P_TURN 1 / 180.0  // Full motor output at the max error (need to do a
// full turn)

// Tape sensing
#define LEFT_WING_PORT A2
#define LINE_LEFT_PORT A3
#define LINE_RIGHT_PORT A4
#define RIGHT_WING_PORT A5

// Measured 3/2/23 at 11:18pm
// ~4-5 mm between the bottom of the shielding and the bottom of the tape
// sensor, ~3/4 in from bottom of shielding to the floor Using a Vce of 2.5V
// with transresistive amplifier, 10k gain resistor, 5V input to LED

#define MAX_WHITE_VOLTAGE 180  // This and below, white surface
#define MIN_BLACK_VOLTAGE 390  // This and above, black surface

#define CENTER_TO_SIDE_DIFFERENCE \
  MIN_BLACK_VOLTAGE -             \
      MAX_WHITE_VOLTAGE  // Difference between center sensor and left/right tape
                         // sensors when fully centered

typedef enum {
  DRIVE_FORWARD,
  TURN_RIGHT,
  TURN_LEFT,
  STOP,
  DRIVE_BACKWARD,
  NOTHING,
  EXITING_STUDIO,
  DRIVING_STUDIO_TO_GOOD,
  DRIVING_STUDIO_TO_BAD,
  DRIVING_FROM_GOOD_TO_BAD,
  DRIVING_GOOD_TO_STUDIO,
  DISPENSE_ONE_BALL,
} States_t;

typedef enum {
  DRIVING_TO_BLACK_TAPE,
  DRIVING_UNTIL_NO_BLACK_TAPE,
  DRIVING_UNTIL_RED_TAPE,
  LINE_FOLLOW_UNTIL_LEFT_WING,
  DRIVE_UNTIL_NO_LEFT_WING,
  LINE_FOLLOW_UNTIL_BLACK_TAPE,
  TURNING_180_DEG,
  LINE_FOLLOW_UNTIL_RIGHT_WING,
  TURNING_90_DEG_RIGHT,
  TURNING_90_DEG_LEFT_TO_BAD,
  WAIT_FOR_LINE_FOLLOW_INPUT,

  // The following states are to get back onto the red line after leaving the
  // studio
  TURN_LEFT_UNTIL_ON_RED,
  TURN_RIGHT_UNTIL_ON_RED,
} Line_follow_states_t;

typedef enum { GOOD_PRESS, BAD_PRESS } Score_targets_t;

static States_t state = NOTHING;
static Line_follow_states_t line_follow_state = WAIT_FOR_LINE_FOLLOW_INPUT;
static bool isDriving;
static Drive drivebase;
static double turnTarget = 0.0;
static LineFollow lineFollow =
    LineFollow(LEFT_WING_PORT, LINE_LEFT_PORT,
               LINE_RIGHT_PORT, RIGHT_WING_PORT);

// Tunable parameters
static double k_p_turn = K_P_TURN;
static double turn_max_error = MAX_TURN_ERROR;
static uint16_t max_white = MAX_WHITE_VOLTAGE;
static uint16_t min_black = MIN_BLACK_VOLTAGE;
static uint16_t center_to_side_diff = CENTER_TO_SIDE_DIFFERENCE;

void driveTest();
void handleExitStudio(Score_targets_t press_target);
void followLine();
void outputSensorVals();
void printUpdateAndChange(uint16_t &old, uint16_t new_val);
void printUpdateAndChange(double &old, double new_val);
void handleConstantChange();
void handleExitStudio(Score_targets_t press_target);
void handleStudioToGood();
void handleStudioToBad();

void setup() {
  Serial.begin(9600);
  while (!Serial)
    ;

  Serial.println("Code started");

  // // Init drivebase
  drivebase = Drive(LEFT_MOTOR_DIR_PORT, LEFT_MOTOR_EN_PORT,
                    RIGHT_MOTOR_DIR_PORT, RIGHT_MOTOR_EN_PORT, GYRO_PORT);

  drivebase.setLeftInverted(IS_LEFT_INVERTED);
  drivebase.setRightInverted(IS_RIGHT_INVERTED);
  isDriving = false;

  Serial.println("Drivebase initialized");
}

void loop() {
  // // End to end drive test
  // if (IS_TESTING_DRIVE && !isDriving) {
  //   isDriving = true;
  //   driveTest();
  // }

  // Test forwards, backwards, left right
  // For drive testing (no gyro)
  if (Serial.available()) {
    uint8_t aKey = Serial.read();
    while (Serial.available()) {
      Serial.read();
    }
    Serial.println(aKey);
    switch (aKey) {
      // Update values on the fly, only works with the first one
      // case 116:  // t for turn error
      //   Serial.println("Input a new value for turn error");
      //   while (!Serial.available())
      //     ;
      //   double new_error = Serial.parseFloat();
      //   printUpdateAndChange(turn_max_error, new_error);
      //   break;
      // case 112:  // p for turn p
      //   Serial.println("Input a new value for turn p: ");
      //   while (!Serial.available())
      //     ;
      //   double new_p = Serial.parseFloat();
      //   printUpdateAndChange(k_p_turn, new_p);
      //   break;
      // case 119:  // w for white tape thresh
      //   Serial.println("Input a new value for max white: ");
      //   while (!Serial.available())
      //     ;
      //   uint16_t new_white_thresh = Serial.parseInt();
      //   printUpdateAndChange(max_white, new_white_thresh);
      //   break;
      case 98:  // b for backwards
        state = DRIVE_BACKWARD;
        drivebase.setLeftPower(-FULL_SPEED);
        drivebase.setRightPower(-FULL_SPEED);
        // Serial.println("Input a new value for min black: ");
        // while (!Serial.available())
        //   ;
        // uint16_t new_black_thresh = Serial.parseInt();
        // printUpdateAndChange(min_black, new_black_thresh);
        break;
      // case 100:  // d for diff
      //   Serial.println("Input a new value for center to side difference: ");
      //   while (!Serial.available())
      //     ;
      //   uint16_t new_center_side_diff = Serial.parseInt();
      //   printUpdateAndChange(center_to_side_diff, new_center_side_diff);
      //   break;
      // Tests for drivebase
      case 108:  // l
        state = TURN_LEFT;
        drivebase.setLeftPower(-FULL_SPEED);
        drivebase.setRightPower(FULL_SPEED);
        break;
      case 102:  // f
        state = DRIVE_FORWARD;
        Serial.println("forward plz?");
        drivebase.setLeftPower(FULL_SPEED);
        drivebase.setRightPower(FULL_SPEED);
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
      case 103:                                         // g
        drivebase.setGyroOffset(drivebase.getAngle());  // Zero the gyro
        break;
      // Tests for line follow
      case 101:  // e
        state = EXITING_STUDIO;
        line_follow_state = DRIVING_TO_BLACK_TAPE;
        drivebase.setLeftPower(FULL_SPEED);
        drivebase.setRightPower(FULL_SPEED);
        break;
      default:
        break;
    }
  }

  // switch (state) {
  //   case EXITING_STUDIO:
  //     handleExitStudio(GOOD_PRESS);  // TODO: Test bad presss
  //     break;
  //   case DISPENSE_ONE_BALL:
  //     // TODO: Replace this with the servo code once ready
  //     delay(1000);
  //     // TODO: Add going from good to bad or something, depending on the target
  //     state = NOTHING;
  //     break;
  //   case DRIVING_STUDIO_TO_BAD:
  //     handleStudioToBad();
  //     break;
  //   case DRIVING_STUDIO_TO_GOOD:
  //     handleStudioToGood();
  //     break;
  //   default:
  //     break;  // Don't really care about the other states here
  // }

  // Periodic print without using Timer2 (which messes with deploying the code)
  if (millis() % 1000L == 0) {
    Serial.println("Outputting");
    outputSensorVals();
  }
}

void outputSensorVals() {
  //drivebase.printDebug();
  lineFollow.printDebug();
}

// Output the change between and old and new value, and overwrite the old value
// to be the new one
void printUpdateAndChange(double &old, double new_val) {
  Serial.print("Old value: ");
  Serial.print(old);
  Serial.print("New value: ");
  Serial.println(new_val);
  old = new_val;
}

void printUpdateAndChange(uint16_t &old, uint16_t new_val) {
  Serial.print("Old value: ");
  Serial.print(old);
  Serial.print("New value: ");
  Serial.println(new_val);
  old = new_val;
}

/**
 * Exit studio procedure (separate FSM)
 * Will get to the first intersection from starting aligned in the studio
 */
void handleExitStudio(Score_targets_t press_target) {
  switch (line_follow_state) {
    case DRIVING_TO_BLACK_TAPE:
      // Once black tape detected, pass it
      if (lineFollow.testForBlackTape(min_black)) {
        drivebase.setLeftPower(HALF_SPEED);
        drivebase.setRightPower(HALF_SPEED);
        line_follow_state = DRIVING_UNTIL_NO_BLACK_TAPE;
      }
      break;
    case DRIVING_UNTIL_NO_BLACK_TAPE:
      // Once no black tape detected, drive until on line
      if (!lineFollow.testForBlackTape(min_black)) {
        drivebase.stopMotors();
        line_follow_state = DRIVING_UNTIL_RED_TAPE;
        drivebase.setLeftPower(HALF_SPEED);
        drivebase.setRightPower(HALF_SPEED);
      }
      break;
    case DRIVING_UNTIL_RED_TAPE:
      // Check if any of the line sensors are on red first
      if (lineFollow.testForOnLine(max_white, min_black)) {
        line_follow_state = LINE_FOLLOW_UNTIL_LEFT_WING;
        drivebase.stopMotors();
        // Otherwise, if the wing is on the line, turn towards the line
      } else if (lineFollow.testForLeftWingRed(max_white, min_black)) {
        line_follow_state = TURN_LEFT_UNTIL_ON_RED;
        drivebase.setLeftPower(-FULL_SPEED);
        drivebase.setRightPower(FULL_SPEED);
      } else if (lineFollow.testForRightWingRed(max_white, min_black)) {
        line_follow_state = TURN_RIGHT_UNTIL_ON_RED;
        drivebase.setLeftPower(FULL_SPEED);
        drivebase.setRightPower(-FULL_SPEED);
      }
      break;

    case LINE_FOLLOW_UNTIL_LEFT_WING:
      // TODO: Check if it's possible to start on the line but with the left
      // wing detecting red Detected left wing Determine what state to handle
      // depending on the scoring target
      if (lineFollow.testForLeftWingRed(max_white, min_black)) {
        if (press_target == GOOD_PRESS) {
          line_follow_state = DRIVE_UNTIL_NO_LEFT_WING;
          state = DRIVING_STUDIO_TO_GOOD;
          drivebase.setLeftPower(FULL_SPEED);
          drivebase.setRightPower(FULL_SPEED);
        } else if (press_target == BAD_PRESS) {
          state = DRIVING_STUDIO_TO_BAD;
          line_follow_state = TURNING_90_DEG_LEFT_TO_BAD;
          turnTarget = drivebase.getAngle() - 90;
        }
      } else {
        followLine();
      }
      break;
    // Intentional fall through, keep turning until on the line
    case TURN_RIGHT_UNTIL_ON_RED:
    case TURN_LEFT_UNTIL_ON_RED:
      if (lineFollow.testForOnLine(max_white, min_black)) {
        drivebase.stopMotors();
        line_follow_state = LINE_FOLLOW_UNTIL_LEFT_WING;
      }
      break;
    default:
      break;
  }
}

void followLine() {
  // Make it simple, then implement PID
  Motor_powers_t powers = lineFollow.getLineFollowPowers(center_to_side_diff);
  drivebase.setLeftPower(powers.left_power);
  drivebase.setRightPower(powers.right_power);
}

// After getting to the intersection of the tape driving from studio to good
// press, get to good press
void handleStudioToGood() {
  switch (line_follow_state) {
    case DRIVE_UNTIL_NO_LEFT_WING:
      if (!lineFollow.testForLeftWingRed(max_white, min_black)) {
        line_follow_state = LINE_FOLLOW_UNTIL_BLACK_TAPE;
        followLine();
      }
      break;
    case LINE_FOLLOW_UNTIL_BLACK_TAPE:
      if (lineFollow.testForBlackTape(min_black)) {
        drivebase.stopMotors();
        line_follow_state = WAIT_FOR_LINE_FOLLOW_INPUT;
        state = DISPENSE_ONE_BALL;
      } else {
        followLine();
      }
      break;
    default:
      break;
  }
}

// After getting to the intersection of the tape driving from studio to bad
// press, get to bad press
void handleStudioToBad() {
  switch (line_follow_state) {
    case TURNING_90_DEG_LEFT_TO_BAD:
      double turn_error = drivebase.calcTurnError(turnTarget);
      // If turned 90 degrees, follow the line
      if (abs(turn_error) <= turn_max_error) {
        if (!lineFollow.testForOnLine(max_white, min_black)) {
          Serial.println("Turned 90 deg, not on the line, hoping for the best");
        }
        line_follow_state = LINE_FOLLOW_UNTIL_BLACK_TAPE;
        followLine();
        // Otherwise, keep turning 90 degrees
      } else {
        drivebase.setTurnPIDPowers(turnTarget, k_p_turn);
      }
    case LINE_FOLLOW_UNTIL_BLACK_TAPE:
      if (lineFollow.testForBlackTape(min_black)) {
        drivebase.stopMotors();
        state = DISPENSE_ONE_BALL;
        line_follow_state = WAIT_FOR_LINE_FOLLOW_INPUT;
      } else {
        followLine();
      }
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
    if (count == 0) {  // Print every 255 cycles
      Serial.println(drivebase.getAngle());
    }
    count++;
  } while (abs(error) >= turn_max_error * 10);
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
    if (count == 0) {  // Print every 255 cycles
      Serial.println(drivebase.getAngle());
    }
    count++;
  } while (abs(error) >= turn_max_error * 10);
  drivebase.stopMotors();
  delay(500);

  Serial.println("Full power forward");
  // Drive straight at full power
  drivebase.setLeftPower(FULL_SPEED);
  drivebase.setRightPower(FULL_SPEED);
  delay(1000);
}