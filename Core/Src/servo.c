#include "servo.h"

#include "stm32l0xx_hal.h"

#include "configurator.h"

ServoHandle SERVO_Create(ServoConfig* config) {
  ServoHandle handle;
  handle.htim = config->htim;
  handle.CCR = config->CCR;
  handle.CHANNEL_ID = config->CHANNEL_ID;
  handle.CCR_MAX = config->CCR_MAX;
  handle.CCR_MIN = config->CCR_MIN;
  return handle;
}

void SERVO_Start(ServoHandle* handle) {
  HAL_TIM_PWM_Start(handle->htim, handle->CHANNEL_ID);
}

void SERVO_Stop(ServoHandle* handle) {
  HAL_TIM_PWM_Stop(handle->htim, handle->CHANNEL_ID);
}

void SERVO_SetPos(ServoHandle* handle, uint32_t pos) {
  const uint32_t MAX = 1024;
  pos = (pos / (double)MAX) * (handle->CCR_MAX - handle->CCR_MIN);
  pos = handle->CCR_MIN + pos;
  if (pos > handle->CCR_MAX) {
    pos = handle->CCR_MAX;
  }

  *handle->CCR = pos;
}

void SERVO_ChangePos(ServoHandle* handle, int64_t delta) {
  uint32_t new_ccr = *handle->CCR + delta;
  if (new_ccr > handle->CCR_MAX) {
    new_ccr = handle->CCR_MAX;
  }
  if (new_ccr < handle->CCR_MIN) {
    new_ccr = handle->CCR_MIN;
  }

  *handle->CCR = new_ccr;
}
