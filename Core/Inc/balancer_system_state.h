#ifndef BALANCER_SYSTEM_STATE_H_
#define BALANCER_SYSTEM_STATE_H_

#include <stdbool.h>

typedef struct {
  int x;  // Value from -1024 to 1024
  int y;  // Value from -1024 to 1024
} BS_PosData;

typedef struct {
  BS_PosData platform_pos;
  BS_PosData des_platform_pos;
  BS_PosData serwos_pos;
  volatile bool* new_accel_gyro_data;
  volatile bool* new_ball_pos_data;
} BS_State;

#endif  // BALANCER_SYSTEM_STATE_H_
