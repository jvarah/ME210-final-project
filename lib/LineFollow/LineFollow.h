/**
 * LineFollow.h defines all of the exposed methods for the line following subsystem, including using its five tape sensors
*/

#ifndef LineFollow_h
#define LineFollow_h
#include <inttypes.h>
#include <LineDataTypes.h>

class LineFollow {
  public:
    /**
     * Pass in the ports for the tape sensors on the robot
    */
    LineFollow(uint8_t left_wing, uint8_t line_left, uint8_t line_right,
                       uint8_t right_wing, Line_thresholds_t thresholds); 
    bool testForBlackTape();
    bool testForLeftWingBlack(); // added ESL
    bool testForRightWingBlack(); // added ESL
    /**
     * Check if the left wing sees red tape (given max value for white, and min value for black tape)
    */
    bool testForLeftWingRed();
    /**
     * Check if the left wing sees either red or black tape
    */
    bool testForRightWingRed();
    /**
     * Get the error for PID line follow, given the difference between the center and right/left wings when the robot is centered over the line
    */
    Motor_powers_t getLineFollowPowers(double k_p, int8_t base_power); // TODO: Add P for smoother line follow
    /**
     * Check that any of the line sensors (middle 3) are on the line
    */
    bool testForOnLine();
    void printDebug();
    void setThresholds(Line_thresholds_t thresholds);
    Line_sensor_vals_t LineFollow::getSensorReadings();

  private:
    uint8_t _left_wing;
    uint8_t _line_left;
    uint8_t _line_right;
    uint8_t _right_wing;
    Line_thresholds_t _thresholds;
    bool isRed(uint16_t sensor_value, uint16_t min_white,
                       uint16_t max_red, uint16_t max_black);
    bool isBlack(uint16_t sensor_value, uint16_t max_red);

};
#endif