#ifndef LineDataTypes_h
#define LineDataTypes_h
#include <inttypes.h>

typedef struct {
  int8_t left_power;
  int8_t right_power;
} Motor_powers_t;

typedef struct {
  uint16_t min_lw_white;
  uint16_t max_lw_red;
  uint16_t max_lw_black;

  uint16_t min_ll_white;
  uint16_t max_ll_red;
  uint16_t max_ll_black;

  uint16_t min_lr_white;
  uint16_t max_lr_red;
  uint16_t max_lr_black;

  uint16_t min_rw_white;
  uint16_t max_rw_red;
  uint16_t max_rw_black;
} Line_thresholds_t;

typedef struct {
  uint16_t lw;
  uint16_t ll;
  uint16_t lr;
  uint16_t rw;
} Line_sensor_vals_t;

#endif