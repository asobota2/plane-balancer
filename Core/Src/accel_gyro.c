#include "accel_gyro.h"

#include "configurator.h"

#include "logger.h"

AccelGyroHandle AccelGyro_Create(AccelGyroConfig* config) {
  AccelGyroHandle handle;
  handle.mpu6050_handle = MPU6050_CreateHandle(false, config->hi2c);
  handle.sample.accelerometer.x = 0;
  handle.sample.accelerometer.y = 0;
  handle.sample.accelerometer.z = 0;
  handle.sample.gyroscope.x = 0;
  handle.sample.gyroscope.y = 0;
  handle.sample.gyroscope.z = 0;
  handle.new_data = false;
  handle.data_guard = false;
  handle.configured = false;
  return handle;
}

bool AccelGyro_Init(AccelGyroHandle* handle) {
  MPU6050_Handle* mpu6050 = &handle->mpu6050_handle;
  MPU6050_Status status = MPU6050_Reset(mpu6050);
  if (status != MPU6050_OK) {
    LOG("MPU6050 I2C Error!\r\n");
    return false;
  }
  status = MPU6050_Init(mpu6050);
  if (status != MPU6050_OK) {
    LOG("MPU6050 I2C Error!\r\n");
    return false;
  }
  status = MPU6050_SetConfig(mpu6050, 0, 4);
  if (status != MPU6050_OK) {
    LOG("MPU6050 I2C Error!\r\n");
    return false;
  }

  status = MPU6050_SetSampleRate(mpu6050, 9);
  if (status != MPU6050_OK) {
    LOG("MPU6050 I2C Error!\r\n");
    return false;
  }

  MPU6050_Interrupts it_config = MPU6050_CreateInterrupts();
  it_config.data_rdy_en = true;
  status = MPU6050_SetItEnabled(mpu6050, &it_config);
  if (status != MPU6050_OK) {
    LOG("MPU6050 I2C Error!\r\n");
    return false;
  }

  LOG("MPU6050 Init success!\r\n");
  handle->configured = true;
  return true;
}

AccelGyroSample AccelGyro_GetData(AccelGyroHandle* handle) {
  handle->data_guard = true;
  AccelGyroSample sample = handle->sample;
  handle->new_data = false;
  handle->data_guard = false;
  return sample;
}

void AccelGyro_RxCpltCallback(AccelGyroHandle* handle) {
  if (!handle->data_guard) {
    uint8_t* data = handle->dma_buffer;

    handle->sample.accelerometer.x = ((int16_t)data[0] << 8) | data[1];
    handle->sample.accelerometer.y = ((int16_t)data[2] << 8) | data[3];
    handle->sample.accelerometer.z = ((int16_t)data[4] << 8) | data[5];

    // 6 bytes of accel data and 2 bytes of temp data
    data += 8;

    handle->sample.gyroscope.x = ((int16_t)data[0] << 8) | data[1];
    handle->sample.gyroscope.y = ((int16_t)data[2] << 8) | data[3];
    handle->sample.gyroscope.z = ((int16_t)data[4] << 8) | data[5];
    handle->new_data = true;
  } else {
    LOG("Missed MPU6050 read due to data guard!\r\n");
  }
}

void AccelGyro_IntPinCallback(AccelGyroHandle* handle) {
  if (!handle->configured) {
    return;
  }
  if (HAL_I2C_GetState(handle->mpu6050_handle.i2c_handle) != HAL_I2C_STATE_READY) {
    LOG("Failed to start next MPU6050 sample read!\r\n");
    return;
  }

  MPU6050_Status status = MPU6050_GetAllData_DMA(&handle->mpu6050_handle, handle->dma_buffer, MPU6050_DMA_BUFFER_SIZE);
  if (status != MPU6050_OK) {
    LOG("Failed to start next MPU6050 sample read!\r\n");
  }
}
