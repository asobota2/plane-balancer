#ifndef CONFIGURATOR_H_
#define CONFIGURATOR_H_

#include "stm32l0xx_hal.h"

typedef struct {
  I2C_HandleTypeDef* hi2c;
} AccelGyroConfig;

typedef struct {
  TIM_HandleTypeDef* htim;
  volatile uint32_t* CCR;
  uint32_t CHANNEL_ID;
  uint32_t CCR_MAX;
  uint32_t CCR_MIN;
  uint32_t INIT_POS;
} ServoConfig;

typedef struct {
  UART_HandleTypeDef* huart;
} BallReaderConfig;

typedef struct {
  double kp;
  double kd;
  double ks;
} PID_Config;

typedef struct {
  AccelGyroConfig accel_gyro;
  ServoConfig servo_x;
  ServoConfig servo_y;
  BallReaderConfig ball_reader;
  PID_Config pid;
} DevicesConfig;

/**
 * @brief Create a Devices Config struct
 * 
 * @return DevicesConfig 
 */
DevicesConfig CreateDevicesConfig();

#endif  // CONFIGURATOR_H_
