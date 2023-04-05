#ifndef PID_H_
#define PID_H_

#include <stdbool.h>
#include <configurator.h>

typedef struct {
  int target;
  int correction;
  int last_val;
  int last_time;
  double sum;
  double kp;
  double kd;
  double ks;
  bool initialized;
} PID_Handle;

/**
 * @brief Create PID handle
 * 
 * @param config 
 * @return PID_Handle 
 */
PID_Handle PID_Create(PID_Config* config);

/**
 * @brief Update PID values
 * 
 * @param handle PID handle
 * @param value Latest measurement
 * @param time Time of the letest measurement
 */
void PID_Update(PID_Handle* handle, int value, int time);

#endif // PID_H_
