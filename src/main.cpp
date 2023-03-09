#include <Arduino.h>
#include <Drive.h>
#include <Hopper.h>
#include <LineDataTypes.h>
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
#define LEFT_MOTOR_EN_PORT 6  // 10 stopped working
#define RIGHT_MOTOR_EN_PORT 5
#define RIGHT_MOTOR_DIR_PORT 4

#define IS_LEFT_INVERTED 1   // was 0
#define IS_RIGHT_INVERTED 0  // was 1

#define FULL_SPEED 127
#define HALF_SPEED 64  // 64 // Max forward is 127
#define QUARTER_SPEED 48

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

#define MAX_WHITE_VOLTAGE \
  190  // This and below, white surface 3-7-23 after some tape mounting
#define MIN_BLACK_VOLTAGE \
  300  // This and above, black surface 3-7-23 after some tape mounting

#define CENTER_TO_SIDE_DIFFERENCE \
  MIN_BLACK_VOLTAGE -             \
      MAX_WHITE_VOLTAGE  // Difference between center sensor and left/right tape
                         // sensors when fully centered
// #define TURN_K_P 1/180.0 // Full motor output at the max error (need to do a
// full turn)

// Servo constants
#define HOPPER_BOT_SERVO 11
#define INDICATOR_SERVO 3
#define SERVO_MAX_ANGLE 180
#define SERVO_MIN_ANGLE 0

#define IR_BEACON_MIN \
  300  // TODO: Test, min value where you know you are facing close enough to
       // the IR beacon wasa 250

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
  DRIVING_FROM_BAD_TO_GOOD,
  DRIVING_GOOD_TO_STUDIO,
  DISPENSE_TWO_BALLS,
  DISPENSE_ALL_BALLS,
} States_t;

typedef enum {
  TURNING_TO_IR_BEACON,
  DRIVING_OUT_OF_STUDIO,
  LINE_FOLLOW_UNTIL_LEFT_WING,
  DRIVE_UNTIL_NO_LEFT_WING,
  LINE_FOLLOW_UNTIL_BLACK_TAPE,
  TURNING_180_DEG,
  LINE_FOLLOW_UNTIL_RIGHT_WING,
  TURNING_90_DEG_RIGHT,
  TURNING_90_DEG_LEFT_TO_BAD,
  WAIT_FOR_LINE_FOLLOW_INPUT,
  TURNING_90_DEG_LEFT_TO_GOOD,
  BACKUP,

  // The following states are to get back onto the red line after leaving the
  // studio
} Line_follow_states_t;

typedef enum { GOOD_PRESS, BAD_PRESS } Score_targets_t;


static Line_thresholds_t thresholds = {
    // LW values
    132, 295, 280, // 3-8-23 at 12am
    // LL values
    133, 300, 285, // 3-8-23 at 9pm // Was 205, too low when untetheres, was 255, then 285, then 300
    // LR values
    131, 300, 290, // 3-8-23 at 8pm // Was 205, too low when unteth, was 245, then 275, then 300
    // RW Values
    128, 295, 280, // Untested, just guess for competition Was 200, then 250
};

#define LEAVE_STUDIO_TIME 325
#define EXIT_TURN_DELAY \
  0  // Keep the robot straight after aligning with IR beacon
#define LINE_FOLLOW_WAIT \
  1000  // Follow line for 2 seconds before checking for wings
#define TURN_TIME_90_DEG_ONE_HALF_SPD 1650 // TODO: Tune/measure was 563
#define TURN_TIME_180_BOTH_MOTORS 1000
#define K_P_LINE_FOLLOW 0.0025 // Max diff ~100 units, base power is 0.5, make max error = full power
#define LINE_FOLLOW_BASE_POWER HALF_SPEED * 1.1

#define BACKUP_TIME 200

static States_t state = NOTHING;
static Line_follow_states_t line_follow_state = WAIT_FOR_LINE_FOLLOW_INPUT;
static bool isDriving;
static Drive drivebase;
static double turnTarget = 0.0;
static LineFollow lineFollow =
    LineFollow(LEFT_WING_PORT, LINE_LEFT_PORT, LINE_RIGHT_PORT, RIGHT_WING_PORT,
               thresholds);

// Tunable parameters
static double k_p_turn = K_P_TURN;
static double turn_max_error = MAX_TURN_ERROR;
static uint16_t max_white = MAX_WHITE_VOLTAGE;
static uint16_t min_black = MIN_BLACK_VOLTAGE;
static uint16_t center_to_side_diff = CENTER_TO_SIDE_DIFFERENCE;

#define IR_SENSE_1 A0

#define NUM_SERVOS 2
#define INDICATOR_STEPS 3
#define MOVE_INDICATOR_TIME 1000  // 5 Seconds

static Servo servo;  // Used for all servos
static Hopper hopper;
uint8_t hasIndicated = 0;  // Will be 2 when it's finished

uint32_t last_time = 0;

Score_targets_t scoring_target = BAD_PRESS;

void driveTest();
void handleExitStudio(Score_targets_t press_target);
void followLine();
void outputSensorVals();
void outputStateChanges();
void printUpdateAndChange(uint16_t &old, uint16_t new_val);
void printUpdateAndChange(double &old, double new_val);
void handleConstantChange();
void handleStudioToGood();
void handleStudioToBad();
void moveIndicator();
void handleBadToGood();
void handleGoodToStudio(); // TODO: Finish getting back to studio

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

  pinMode(IR_SENSE_1, INPUT);
  Serial.println("Drivebase initialized");

  // Init more values
  hasIndicated = 0;  // Will be 2 when it's finished
  last_time = 0;
  scoring_target = BAD_PRESS;

  // Indicate
  if (!servo.attach(INDICATOR_SERVO)) {
    Serial.println("Indicator unable to attach to its pin");
  }  // The port for the ball dropper
  // 3 is the indicator servo
  servo.write(SERVO_MAX_ANGLE);
  ITimer2.init();
  ITimer2.setInterval(MOVE_INDICATOR_TIME / INDICATOR_STEPS, moveIndicator,
                      MOVE_INDICATOR_TIME);

  lineFollow.setThresholds(thresholds);
  state = EXITING_STUDIO;
  line_follow_state = TURNING_TO_IR_BEACON;
  // line_follow_state = LINE_FOLLOW_UNTIL_BLACK_TAPE; // For line follow testing only
  drivebase.setLeftPower(-QUARTER_SPEED);
  drivebase.setRightPower(QUARTER_SPEED);
}

void loop() {
  // Re-attach the servo to the hopper
  if (hasIndicated >= INDICATOR_STEPS && !hopper.isInitialized()) {
    hopper = Hopper(servo, HOPPER_BOT_SERVO);
  }

  // End to end drive test
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
      case 99:  // c for calibrate
        state = NOTHING;
        drivebase.setLeftPower(HALF_SPEED);
        drivebase.setRightPower(HALF_SPEED);

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
        line_follow_state = TURNING_TO_IR_BEACON;
        // line_follow_state = LINE_FOLLOW_UNTIL_BLACK_TAPE; // For line follow testing only
        drivebase.setLeftPower(-QUARTER_SPEED);
        drivebase.setRightPower(QUARTER_SPEED);


        // // Testing only 180
        // state = DRIVING_FROM_BAD_TO_GOOD;
        // line_follow_state = TURNING_180_DEG;
        // drivebase.setLeftPower(-HALF_SPEED);
        // drivebase.setRightPower(HALF_SPEED);
        // last_time = millis();
        break;
      case 100:  // d for drop
        hopper.dropTwoBalls();
        break;
      default:
        break;
    }
  }

  // if (state == EXITING_STUDIO) {
  //   switch (line_follow_state) {
  //     case LINE_FOLLOW_UNTIL_BLACK_TAPE:
  //       followLine();
  //       break;
  //   }
  // }

  switch (state) {
    case EXITING_STUDIO:
      handleExitStudio(scoring_target);  // TODO: Test Good presss
      break;
    case DISPENSE_TWO_BALLS:
      hopper.dropTwoBalls();
      if (scoring_target == BAD_PRESS) {
        state = DRIVING_FROM_BAD_TO_GOOD;
      } else {
        state = DRIVING_GOOD_TO_STUDIO;
      }
      line_follow_state = BACKUP;
      Serial.println("Backing up");
      last_time = millis();
      drivebase.setLeftPower(-HALF_SPEED);
      drivebase.setRightPower(-HALF_SPEED);
      break;
    case DISPENSE_ALL_BALLS:
      hopper.dropAllBalls();
      state = DRIVING_GOOD_TO_STUDIO;
      line_follow_state = BACKUP;
      Serial.println("Backing up");
      last_time = millis();
      drivebase.setLeftPower(-HALF_SPEED);
      drivebase.setRightPower(-HALF_SPEED);
    case DRIVING_STUDIO_TO_BAD:
      handleStudioToBad();
      break;
    case DRIVING_STUDIO_TO_GOOD:
      handleStudioToGood();
      break;
    case DRIVING_FROM_BAD_TO_GOOD:
      handleBadToGood();
      break;

    // TODO: Below states for competition
    case DRIVING_GOOD_TO_STUDIO: // Not needed for checkoff
      handleGoodToStudio();
      break;
    // case WAITING_FOR_GO:
    //   // TODO: Something
    //   break;
    default:
      break;  // Don't really care about the other states here
  }

  // Periodic print without using Timer2 (which messes with deploying the code)
  if (millis() % 1000L == 0) {
    // Serial.println("Outputting");
    outputSensorVals();
  }
  // outputStateChanges();
}

void moveIndicator() {
  Serial.println("Indicating");
  if (hasIndicated >= INDICATOR_STEPS) {
    // Do nothing
  } else {
    Serial.println("Movings");
    if (hasIndicated % 2 == 0) {
      servo.write(SERVO_MAX_ANGLE);
    } else {
      servo.write(SERVO_MIN_ANGLE);
    }
    hasIndicated++;
  }
}

void outputStateChanges() {
  if (lineFollow.testForBlackTape()) {
    Serial.println("Found black tape!");
  }
  if (lineFollow.testForLeftWingRed()) {
    Serial.println("Left wing sensing red");
  }
}

void outputSensorVals() {
  // drivebase.printDebug();
  lineFollow.printDebug();
  Serial.print("IR sensor ");
  Serial.println(analogRead(IR_SENSE_1));
  // Serial.println(state);
  // Serial.println(line_follow_state);
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

void handleGoodToStudio() {
  switch (line_follow_state) {
    case BACKUP:
      if (millis() - last_time > BACKUP_TIME) {
        drivebase.setLeftPower(-HALF_SPEED);
        drivebase.setRightPower(HALF_SPEED);
        last_time = millis();
        line_follow_state = TURNING_180_DEG;
      }
      break;
    case TURNING_180_DEG:
      // Follow line after waiting for a ~180 deg turn
      if (millis() - last_time > (TURN_TIME_180_BOTH_MOTORS) && lineFollow.testForOnLine()) {
        Serial.println("Line following until left wing hits");
        last_time = millis();
        line_follow_state = LINE_FOLLOW_UNTIL_RIGHT_WING;
        followLine();
      }
      break;
    // TODO: Line follow past the right wing (drive for time)
  }
}

void handleBadToGood() {
  switch (line_follow_state) {
    case BACKUP:
      if (millis() - last_time > BACKUP_TIME) {
        drivebase.setLeftPower(-HALF_SPEED);
        drivebase.setRightPower(HALF_SPEED);
        last_time = millis();
        line_follow_state = TURNING_180_DEG;
      }
      break;
    case TURNING_180_DEG:
      // Follow line after waiting for a ~180 deg turn
      if ((millis() - last_time > (TURN_TIME_180_BOTH_MOTORS)) && lineFollow.testForOnLine()) {
        Serial.println("Line following until left wing hits");
        last_time = millis();
        line_follow_state = LINE_FOLLOW_UNTIL_LEFT_WING;
        followLine();
      }
      break;
    case LINE_FOLLOW_UNTIL_LEFT_WING:
      if ((millis() - last_time > LINE_FOLLOW_WAIT) && lineFollow.testForLeftWingRed()) {
        Serial.println("Turning 90 deg left to good");
        last_time = millis();
        line_follow_state = TURNING_90_DEG_LEFT_TO_GOOD;
        drivebase.setLeftPower(0);
        drivebase.setRightPower(HALF_SPEED);
      } else {
        followLine();
      }
      break;
    case TURNING_90_DEG_LEFT_TO_GOOD:
      if ((millis() - last_time > TURN_TIME_90_DEG_ONE_HALF_SPD) && lineFollow.testForOnLine()) {
        line_follow_state = LINE_FOLLOW_UNTIL_BLACK_TAPE;
        Serial.println("Line follow until bad tape at good press");
        followLine();
      } 
      break;
    case LINE_FOLLOW_UNTIL_BLACK_TAPE:
      if (lineFollow.testForBlackTape()) {
        Serial.println("Got black tape");
        drivebase.stopMotors();
        line_follow_state = WAIT_FOR_LINE_FOLLOW_INPUT;
        state = DISPENSE_ALL_BALLS;
        scoring_target = GOOD_PRESS;
      } else {
        followLine();
      }
      break;
  }
}

/**
 * Exit studio procedure (separate FSM)
 * Will get to the first intersection from starting aligned in the studio
 */
void handleExitStudio(Score_targets_t press_target) {
  switch (line_follow_state) {
    case TURNING_TO_IR_BEACON:
      if (analogRead(IR_SENSE_1) > IR_BEACON_MIN) {
        delay(EXIT_TURN_DELAY);
        drivebase.setLeftPower(HALF_SPEED);
        drivebase.setRightPower(HALF_SPEED);
        line_follow_state = DRIVING_OUT_OF_STUDIO;
        last_time = millis();
        Serial.println("Exiting studio by driving forward");
      }
      break;
    case DRIVING_OUT_OF_STUDIO:
      // TODO: if timer expired, stop
      // Drive for one second
      if (millis() - last_time > LEAVE_STUDIO_TIME) {
        drivebase.stopMotors();
        line_follow_state = LINE_FOLLOW_UNTIL_LEFT_WING;
        last_time = millis();
        Serial.println("Driving until left wing");
      }
      break;
    case LINE_FOLLOW_UNTIL_LEFT_WING:
      // Line follow for a bit before checking the wing too early
      if (millis() - last_time > LINE_FOLLOW_WAIT &&
          lineFollow.testForLeftWingRed()) {
        // Determine what state to handle depending on the scoring target
        if (press_target == GOOD_PRESS) {
          line_follow_state = DRIVE_UNTIL_NO_LEFT_WING;
          state = DRIVING_STUDIO_TO_GOOD;
          drivebase.setLeftPower(FULL_SPEED);
          drivebase.setRightPower(FULL_SPEED);
          // TODO: Fix this too
        } else if (press_target == BAD_PRESS) {
          state = DRIVING_STUDIO_TO_BAD;
          line_follow_state = TURNING_90_DEG_LEFT_TO_BAD;
          last_time = millis();
          Serial.println("Beginning turn left to hit the bad press line");
          // turnTarget = drivebase.getAngle() - 90;
          drivebase.setLeftPower(0);
          drivebase.setRightPower(HALF_SPEED);
        }
      } else {
        followLine();
      }
      break;

    // case LINE_FOLLOW_UNTIL_BLACK_TAPE: // For testing line follow only, set state to LINE_FOLLOW_UNTIL_BLACK_TAPE in main
    //   followLine();
    //   break;
    default:
      Serial.println("Something is not good in exiting studio");
      break;
  }
}

void followLine() {
  // Make it simple, then implement PID
  Motor_powers_t powers = lineFollow.getLineFollowPowers(K_P_LINE_FOLLOW, LINE_FOLLOW_BASE_POWER);
  // Serial.println(powers.left_power); // TODO: Remove
  drivebase.setLeftPower(powers.left_power);
  drivebase.setRightPower(powers.right_power);
}

// After getting to the intersection of the tape driving from studio to good
// press, get to good press
void handleStudioToGood() {
  switch (line_follow_state) {
    case DRIVE_UNTIL_NO_LEFT_WING:
      if (!lineFollow.testForLeftWingRed()) {
        line_follow_state = LINE_FOLLOW_UNTIL_BLACK_TAPE;
        followLine();
      }
      break;
    case LINE_FOLLOW_UNTIL_BLACK_TAPE:
      if (lineFollow.testForBlackTape()) {
        drivebase.stopMotors();
        line_follow_state = WAIT_FOR_LINE_FOLLOW_INPUT;
        state = DISPENSE_TWO_BALLS;
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
      // Turned enough to start checking if on line
      if ((millis() - last_time > TURN_TIME_90_DEG_ONE_HALF_SPD) && lineFollow.testForOnLine()) {
        line_follow_state = LINE_FOLLOW_UNTIL_BLACK_TAPE;
        Serial.println("Line follow until bad tape at bad press");
        followLine();
      }
      break;
    
      // double turn_error = drivebase.calcTurnError(turnTarget);
      // // If turned 90 degrees, follow the line
      // if (abs(turn_error) <= turn_max_error) {
      //   if (!lineFollow.testForOnLine(max_white, min_black)) {
      //     Serial.println("Turned 90 deg, not on the line, hoping for the
      //     best");
      //   }
      //   line_follow_state = LINE_FOLLOW_UNTIL_BLACK_TAPE;
      //   followLine();
      //   // Otherwise, keep turning 90 degrees
      // } else {
      //   drivebase.setTurnPIDPowers(turnTarget, k_p_turn);
      // }

      // if (!lineFollow.testForOnLine()) {
      //   Serial.println("Turning 90 deg, not on the line, hoping for the best");
      // } else {
      //   line_follow_state = LINE_FOLLOW_UNTIL_BLACK_TAPE;
      //   followLine();
      // }
    case LINE_FOLLOW_UNTIL_BLACK_TAPE:
      if (lineFollow.testForBlackTape()) {
        Serial.println("Dropping in bad press");
        drivebase.stopMotors();
        state = DISPENSE_TWO_BALLS;
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