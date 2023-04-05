#ifndef ACCEL_GYRO_H_
#define ACCEL_GYRO_H_

#include <stdbool.h>

#include "configurator.h"
#include "mpu6050_driver.h"

#define MPU6050_DMA_BUFFER_SIZE 14

typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} AccelGyroData;

typedef struct {
  AccelGyroData accelerometer;
  AccelGyroData gyroscope;
} AccelGyroSample;

typedef struct {
  MPU6050_Handle mpu6050_handle;
  AccelGyroSample sample;
  uint8_t dma_buffer[MPU6050_DMA_BUFFER_SIZE];
  bool new_data;
  bool data_guard;
  bool configured;
} AccelGyroHandle;

/**
 * @brief Create AccelGyro handle
 * 
 * @param config 
 * @return AccelGyroHandle 
 */
AccelGyroHandle AccelGyro_Create(AccelGyroConfig* config);

/**
 * @brief Initialize AccelGyro
 * 
 * @param handle 
 * @return true if succeed
 * @return false if failed
 */
bool AccelGyro_Init(AccelGyroHandle* handle);

/**
 * @brief Get latest measurement
 * 
 * @param handle 
 * @return AccelGyroSample 
 */
AccelGyroSample AccelGyro_GetData(AccelGyroHandle* handle);

/**
 * @brief Callback to be called on interrupt from MPU6050
 * 
 * @param handle 
 */
void AccelGyro_IntPinCallback(AccelGyroHandle* handle);

/**
 * @brief Callback to be called on I2C RX transfer complete
 * 
 * @param handle 
 */
void AccelGyro_RxCpltCallback(AccelGyroHandle* handle);

#endif  // ACCEL_GYRO_H_
