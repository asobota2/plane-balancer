#include "pid.h"

#include "configurator.h"
#include "logger.h"

PID_Handle PID_Create(PID_Config* config) {
  PID_Handle handle;
  handle.target = 0;
  handle.correction = 0;
  handle.last_val = 0;
  handle.last_time = 0;
  handle.kp = config->kp;
  handle.kd = config->kd;
  handle.ks = config->ks;
  handle.initialized = false;
  return handle;
}

void PID_Update(PID_Handle* handle, int value, int time) {
  int error = handle->target - value;

  int dv = value - handle->last_val;
  int dt = time - handle->last_time;
  if (dt < 0) {
    dt = 10000 - dt;
  }
  if (dt == 0) {
    LOG("Received dt = 0!\r\n");
    dt = 1;
  }

  handle->sum += error;
  if (value == 0 ||
      (value > 0 && handle->last_val < 0) ||
      (value < 0 && handle->last_val > 0)) {
    handle->sum = 0;
  }

  int p = error * handle->kp;
  int d = dv / (double)dt * handle->kd;
  int i = handle->sum * handle->ks;

  if (handle->initialized) {
    // TODO - change
    // handle->correction = p - d;
    handle->correction = p - d + i;
  } else {
    handle->correction = p;
    handle->initialized = true;
  }

  handle->last_val = value;
  handle->last_time = time;
}
