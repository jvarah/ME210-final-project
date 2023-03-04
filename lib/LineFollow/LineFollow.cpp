/**
 * LineFollow.cpp is the class for using 5 tape sensors to follow a line and
 * detect intersections
 */

#include "LineFollow.h"

#include <Arduino.h>

LineFollow::LineFollow(uint8_t left_wing, uint8_t line_left, uint8_t line_right,
                       uint8_t right_wing) {
  _left_wing = left_wing;
  _line_left = line_left;
  _line_right = line_right;
  _right_wing = right_wing;
}

bool LineFollow::testForBlackTape(uint16_t min_black_threshold) {
  // Find max value of all the sensors, see if any of those detect black tape
  uint16_t max_wing_reading =
      max(analogRead(_left_wing), analogRead(_right_wing));
  uint16_t max_line_reading =
      max(analogRead(_line_left), analogRead(_line_right));
  return max(max_wing_reading, max_line_reading) > min_black_threshold;
}

bool LineFollow::testForLeftWingRed(uint16_t max_white_threshold,
                                    uint16_t min_black_threshold) {
  return isRed(analogRead(_left_wing), max_white_threshold,
               min_black_threshold);
}

bool LineFollow::testForRightWingRed(uint16_t max_white_threshold,
                                     uint16_t min_black_threshold) {
  return isRed(analogRead(_right_wing), max_white_threshold,
               min_black_threshold);
}

bool LineFollow::testForOnLine(uint16_t max_white_threshold,
                               uint16_t min_black_threshold) {
  bool is_left_red =
      isRed(analogRead(_line_left), max_white_threshold, min_black_threshold);
  bool is_right_red =
      isRed(analogRead(_line_right), max_white_threshold, min_black_threshold);
  return is_left_red || is_right_red;
}

Motor_powers_t LineFollow::getLineFollowPowers(
    uint16_t light_diff_when_centered) {
  // TODO: Add PID
  uint16_t left_value = analogRead(_line_left);
  uint16_t right_value = analogRead(_line_right);
  if (left_value > right_value) {
    // Turn left
    return {INT8_MIN, INT8_MAX};
  } else if (right_value > left_value) {
    // Turn right
    return {INT8_MAX, INT8_MIN};
  }
  return {INT8_MAX, INT8_MAX};
}

bool LineFollow::isRed(uint16_t sensor_value, uint16_t max_white,
                       uint16_t min_black) {
  return (sensor_value > max_white) && (sensor_value < min_black);
}

void LineFollow::printDebug() {
  Serial.print("LineL: ");
  Serial.println(analogRead(_line_left));
  Serial.print("LineR: ");
  Serial.println(analogRead(_line_right));
  Serial.print("LW: ");
  Serial.println(analogRead(_left_wing));
  Serial.print("RW: ");
  Serial.println(analogRead(_right_wing));
}