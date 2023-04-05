#ifndef SERVO_H_
#define SERVO_H_

#include <stdint.h>

#include "stm32l0xx_hal.h"

#include "configurator.h"

typedef struct {
  TIM_HandleTypeDef* htim;
  volatile uint32_t* CCR;
  uint32_t CHANNEL_ID;
  uint32_t CCR_MAX;
  uint32_t CCR_MIN;
} ServoHandle;

/**
 * @brief Create servo handle
 * 
 * @param config 
 * @return ServoHandle 
 */
ServoHandle SERVO_Create(ServoConfig* config);

/**
 * @brief Start PWM to servos
 * 
 * @param handle 
 */
void SERVO_Start(ServoHandle* handle);

/**
 * @brief Stop PWM to servos
 * 
 * @param handle 
 */
void SERVO_Stop(ServoHandle* handle);

/**
 * @brief Sets angle of the servo
 * 
 * @param handle 
 * @param pos Value between 0 and 1024
 */
void SERVO_SetPos(ServoHandle* handle, uint32_t pos);

/**
 * @brief Change servos position by delta
 * 
 * @param handle 
 * @param delta 
 */
void SERVO_ChangePos(ServoHandle* handle, int64_t delta);


#endif  // SERVO_H_
