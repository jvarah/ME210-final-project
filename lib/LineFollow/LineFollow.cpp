/**
 * LineFollow.cpp is the class for using 5 tape sensors to follow a line and
 * detect intersections
 */

#include "LineFollow.h"

#include <Arduino.h>

#define RED_BLACK_WIGGLE_ROOM 20

LineFollow::LineFollow(uint8_t left_wing, uint8_t line_left, uint8_t line_right,
                       uint8_t right_wing, Line_thresholds_t thresholds) {
  _left_wing = left_wing;
  _line_left = line_left;
  _line_right = line_right;
  _right_wing = right_wing;
  _thresholds = thresholds;
}

void LineFollow::setThresholds(Line_thresholds_t thresholds) {
  _thresholds = thresholds;
}

Line_sensor_vals_t LineFollow::getSensorReadings() {
  return {analogRead(_left_wing), analogRead(_right_wing),
          analogRead(_line_left), analogRead(_line_right)};
}

bool LineFollow::testForBlackTape() {
  // Check if any of the sensors detect black tape
  bool lw_black = isBlack(analogRead(_left_wing), _thresholds.max_lw_red);
  bool ll_black = isBlack(analogRead(_line_left), _thresholds.max_ll_red);
  bool lr_black = isBlack(analogRead(_line_right), _thresholds.max_lr_red);
  bool rw_black = isBlack(analogRead(_right_wing), _thresholds.max_rw_red);
  return lw_black || ll_black || lr_black || rw_black;
}

bool LineFollow::testForLeftWingRed() {
  return isRed(analogRead(_left_wing), _thresholds.min_lw_white,
               _thresholds.max_lw_red, _thresholds.max_lw_black);
}

bool LineFollow::testForRightWingRed() {
  return isRed(analogRead(_right_wing), _thresholds.min_rw_white,
               _thresholds.max_rw_red, _thresholds.max_rw_black);
}

bool LineFollow::testForOnLine() {
  bool is_left_red = isRed(analogRead(_line_left), _thresholds.min_ll_white,
                           _thresholds.max_ll_red, _thresholds.max_ll_black);
  bool is_right_red = isRed(analogRead(_line_right), _thresholds.min_lr_white,
                            _thresholds.max_lr_red, _thresholds.max_lr_black);
  return is_left_red || is_right_red;
}

Motor_powers_t LineFollow::getLineFollowPowers(float k_p, int8_t base_power) {
  uint16_t left_value = analogRead(_line_left);
  uint16_t right_value = analogRead(_line_right);

  uint16_t error = left_value - right_value;
  double pid_output = error * k_p;
  // If left value more than right value, it is more red, therefore, turn left
  return {base_power + int8_t(-pid_output), base_power + int8_t(pid_output)};
}

bool LineFollow::isRed(uint16_t sensor_value, uint16_t min_white,
                       uint16_t max_red, uint16_t max_black) {
  uint16_t max_white = (min_white + max_red) >> 1;  // Divide by 2 for average
  uint16_t min_black = (max_red + RED_BLACK_WIGGLE_ROOM);
  return (sensor_value > max_white) && (sensor_value < min_black);
}

bool LineFollow::isBlack(uint16_t sensor_value, uint16_t max_red) {
  return sensor_value > (max_red + RED_BLACK_WIGGLE_ROOM);
}

void LineFollow::printDebug() {
  Serial.print("LW: ");
  Serial.println(analogRead(_left_wing));
  Serial.print("LineL: ");
  Serial.println(analogRead(_line_left));
  Serial.print("LineR: ");
  Serial.println(analogRead(_line_right));
  Serial.print("RW: ");
  Serial.println(analogRead(_right_wing));
}