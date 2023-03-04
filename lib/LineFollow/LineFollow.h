/**
 * LineFollow.h defines all of the exposed methods for the line following subsystem, including using its five tape sensors
*/

#ifndef LineFollow_h
#define LineFollow_h
#include <inttypes.h>

typedef struct {
  int8_t left_power;
  int8_t right_power;
} Motor_powers_t;

class LineFollow {
  public:
    /**
     * Pass in the ports for the tape sensors on the robot
    */
    LineFollow(uint8_t left_wing, uint8_t line_left, uint8_t line_center, uint8_t line_right, uint8_t right_wing);
    bool testForBlackTape(uint16_t min_black_threshold);
    /**
     * Check if the left wing sees red tape (given max value for white, and min value for black tape)
    */
    bool testForLeftWingRed(uint16_t max_white_threshold, uint16_t min_black_threshold);
    /**
     * Check if the left wing sees either red or black tape
    */
    bool testForRightWingRed(uint16_t max_white_threshold, uint16_t min_black_threshold);
    /**
     * Get the error for PID line follow, given the difference between the center and right/left wings when the robot is centered over the line
    */
    Motor_powers_t getLineFollowPowers(uint16_t light_diff_when_centered); // TODO: Add P for smoother line follow
    /**
     * Check that any of the line sensors (middle 3) are on the line
    */
    bool testForOnLine(uint16_t max_white_threshold, uint16_t min_black_threshold);
    void printDebug();

  private:
    uint8_t _left_wing;
    uint8_t _line_left;
    uint8_t _line_center;
    uint8_t _line_right;
    uint8_t _right_wing;
    bool isRed(uint16_t sensor_value, uint16_t max_white, uint16_t min_black);

};
#endif