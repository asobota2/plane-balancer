#include "mpu6050_driver.h"

#include "stm32l0xx_hal.h"

MPU6050_Status MPU6050_WriteData(
    MPU6050_Handle* mpu6050_handle,
    uint8_t address,
    uint8_t* data,
    uint16_t size) {
  HAL_StatusTypeDef status = HAL_I2C_Mem_Write(
      mpu6050_handle->i2c_handle,
      mpu6050_handle->address,
      address,
      1, data, size,
      mpu6050_handle->timeout);
  if (status != HAL_OK) {
    return MPU6050_ERROR;
  }
  return MPU6050_OK;
}

MPU6050_Status MPU6050_ReadData(
    MPU6050_Handle* mpu6050_handle,
    uint8_t address,
    uint8_t* data,
    uint16_t size) {
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
      mpu6050_handle->i2c_handle,
      mpu6050_handle->address,
      address,
      1, data, size,
      mpu6050_handle->timeout);
  if (status != HAL_OK) {
    return MPU6050_ERROR;
  }
  return MPU6050_OK;
}

MPU6050_Status MPU6050_ReadData_DMA(
    MPU6050_Handle* mpu6050_handle,
    uint8_t address,
    uint8_t* data,
    uint16_t size) {
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(
      mpu6050_handle->i2c_handle,
      mpu6050_handle->address,
      address,
      1, data, size);
  if (status != HAL_OK) {
    return MPU6050_ERROR;
  }
  return MPU6050_OK;
}

MPU6050_Handle MPU6050_CreateHandle(bool AD0_pin_state, I2C_HandleTypeDef* i2c_handle) {
  const uint8_t base_address = 0x68;
  
  MPU6050_Handle handle;
  handle.address = (base_address + AD0_pin_state) << 1;
  handle.i2c_handle = i2c_handle;
  handle.timeout = MPU6050_I2C_TIMEOUT;
  return handle;
}

MPU6050_ItConfig MPU6050_CreateItConfig() {
  MPU6050_ItConfig it_config;
  it_config.INT_LEVEL = 0;
  it_config.INT_OPEN = 0;
  it_config.LATCH_INT_EN = 0;
  it_config.INT_RD_CLEAR = 0;
  it_config.FSYNC_INT_LEVEL = 0;
  it_config.FSYNC_INT_EN = 0;

  return it_config;
}

MPU6050_Interrupts MPU6050_CreateInterrupts() {
  MPU6050_Interrupts interrupts;
  interrupts.fifo_ovf_en = 0;
  interrupts.i2c_mst_en = 0;
  interrupts.data_rdy_en = 0;
  return interrupts;
}

MPU6050_FIFOConfig MPU6050_CreateFIFOCOnfig() {
  MPU6050_FIFOConfig config;
  config.TEMP_FIFO_EN = 0;
  config.XG_FIFO_EN = 0;
  config.YG_FIFO_EN = 0;
  config.ZG_FIFO_EN = 0;
  config.ACCEL_FIFO_EN = 0;
  return config;
}

MPU6050_Data MPU6050_CreateData() {
  MPU6050_Data data;
  data.x_axis = 0;
  data.y_axis = 0;
  data.z_axis = 0;
  return data;
}

MPU6050_Status MPU6050_Reset(MPU6050_Handle* mpu6050_handle) {
  uint8_t data = 0x0;
  MPU6050_Status status = MPU6050_WriteData(mpu6050_handle, MPU6050_PWR_MGMT_1, &data, 1);
  if (status != MPU6050_OK) {
    return status;
  }

  HAL_Delay(100);
  data = 0xFF;
  status = MPU6050_WriteData(mpu6050_handle, MPU6050_SIGNAL_PATH_RESET, &data, 1);

  HAL_Delay(100);
  return status;
}

MPU6050_Status MPU6050_Init(MPU6050_Handle* mpu6050_handle) {
  return MPU6050_OK;
}

MPU6050_Status MPU6050_SetSampleRate(MPU6050_Handle* mpu6050_handle, uint8_t SMPRT_DIV) {
  return MPU6050_WriteData(mpu6050_handle, MPU6050_SMPRT_DIV, &SMPRT_DIV, 1);
}

MPU6050_Status MPU6050_SetConfig(MPU6050_Handle* mpu6050_handle, uint8_t ext_sync_set, uint8_t dlpf_cfg) {
  uint8_t data = 0;
  data |= (ext_sync_set & 0x7) << 3;
  data |= (dlpf_cfg & 0x7);
  return MPU6050_WriteData(mpu6050_handle, MPU6050_CONFIG, &data, 1);
}

MPU6050_Status MPU6050_SetGyroFullScale(MPU6050_Handle* mpu6050_handle, uint8_t FS_SEL) {
  uint8_t reg;
  MPU6050_Status status = MPU6050_ReadData(mpu6050_handle, MPU6050_GYRO_CONFIG, &reg, 1);
  if (status != MPU6050_OK) {
    return status;
  }

  FS_SEL = FS_SEL & 0x3;
  reg |= FS_SEL << 3;
  return MPU6050_WriteData(mpu6050_handle, MPU6050_GYRO_CONFIG, &reg, 1);
}


MPU6050_Status MPU6050_SetAccelFullScale(MPU6050_Handle* mpu6050_handle, uint8_t AFS_SEL) {
  uint8_t reg;
  MPU6050_Status status = MPU6050_ReadData(mpu6050_handle, MPU6050_ACCEL_CONFIG, &reg, 1);
  if (status != MPU6050_OK) {
    return status;
  }

  AFS_SEL = AFS_SEL & 0x3;
  reg |= AFS_SEL << 3;
  return MPU6050_WriteData(mpu6050_handle, MPU6050_ACCEL_CONFIG, &reg, 1);
}

MPU6050_Status MPU6050_SetItConfig(MPU6050_Handle* mpu6050_handle, MPU6050_ItConfig* it_config) {
  uint8_t reg;
  MPU6050_Status status = MPU6050_ReadData(mpu6050_handle, MPU6050_INT_PIN_CFG, &reg, 1);
  if (status != MPU6050_OK) {
    return status;
  }

  reg |= it_config->INT_LEVEL << 7;
  reg |= it_config->INT_OPEN << 6;
  reg |= it_config->LATCH_INT_EN << 5;
  reg |= it_config->INT_RD_CLEAR << 4;
  reg |= it_config->FSYNC_INT_LEVEL << 3;
  reg |= it_config->FSYNC_INT_EN << 2;

  return MPU6050_WriteData(mpu6050_handle, MPU6050_INT_PIN_CFG, &reg, 1);
}

MPU6050_Status MPU6050_SetItEnabled(MPU6050_Handle* mpu6050_handle, MPU6050_Interrupts* interrupts) {
  uint8_t data = 0;
  data |= interrupts->fifo_ovf_en << 4;
  data |= interrupts->i2c_mst_en << 3;
  data |= interrupts->data_rdy_en;
  return MPU6050_WriteData(mpu6050_handle, MPU6050_INT_ENABLE, &data, 1);
}

MPU6050_Status MPU6050_GetItStatus(MPU6050_Handle* mpu6050_handle, MPU6050_Interrupts* interrupts) {
  uint8_t reg;
  MPU6050_Status status = MPU6050_ReadData(mpu6050_handle, MPU6050_INT_STATUS, &reg, 1);
  if (status != MPU6050_OK) {
    return status;
  }

  interrupts->fifo_ovf_en = (reg & 0x5) != 0;
  interrupts->i2c_mst_en = (reg & 0x4) != 0;
  interrupts->data_rdy_en = (reg & 0x1) != 0;
  return status;
}

MPU6050_Status MPU6050_SetFIFOEnabled(MPU6050_Handle* mpu6050_handle, MPU6050_FIFOConfig* fifo_config) {
  uint8_t data = 0;
  data |= fifo_config->TEMP_FIFO_EN << 7;
  data |= fifo_config->XG_FIFO_EN << 6;
  data |= fifo_config->YG_FIFO_EN << 5;
  data |= fifo_config->ZG_FIFO_EN << 4;
  data |= fifo_config->ACCEL_FIFO_EN << 3;
  return MPU6050_WriteData(mpu6050_handle, MPU6050_FIFO_EN, &data, 1);
}

MPU6050_Status MPU6050_GetAccel(MPU6050_Handle* mpu6050_handle, MPU6050_Data* accel_data) {
  uint8_t x_axis[2];
  uint8_t y_axis[2];
  uint8_t z_axis[2];

  MPU6050_Status status = MPU6050_ReadData(mpu6050_handle, MPU6050_ACCEL_XOUT_H, x_axis, 2);
  if (status != MPU6050_OK) {
    return status;
  }

  status = MPU6050_ReadData(mpu6050_handle, MPU6050_ACCEL_YOUT_H, y_axis, 2);
  if (status != MPU6050_OK) {
    return status;
  }

  status = MPU6050_ReadData(mpu6050_handle, MPU6050_ACCEL_ZOUT_H, z_axis, 2);
  if (status != MPU6050_OK) {
    return status;
  }

  accel_data->x_axis = ((int16_t)x_axis[0] << 8) | x_axis[1];
  accel_data->y_axis = ((int16_t)y_axis[0] << 8) | y_axis[1];
  accel_data->z_axis = ((int16_t)z_axis[0] << 8) | z_axis[1];

  return status;
}

MPU6050_Status MPU6050_GetAccel_DMA(MPU6050_Handle* mpu6050_handle, uint8_t* dma_buffer, uint8_t size) {
  if (size < 6) {
    return MPU6050_ERROR;
  }

  return MPU6050_ReadData_DMA(mpu6050_handle, MPU6050_ACCEL_XOUT_H, dma_buffer, 6);
}

MPU6050_Status MPU6050_GetTemp(MPU6050_Handle* mpu6050_handle, int16_t* temp) {
  int16_t data;
  MPU6050_Status status = MPU6050_ReadData(mpu6050_handle, MPU6050_ACCEL_XOUT_H, (uint8_t*)&data, 2);
  if (status != MPU6050_OK) {
    return status;
  }

  *temp = data/340.0 + 36.52;
  return status;
}

MPU6050_Status MPU6050_GetGyro(MPU6050_Handle* mpu6050_handle, MPU6050_Data* gyro_data) {
  uint8_t x_axis[2];
  uint8_t y_axis[2];
  uint8_t z_axis[2];

  MPU6050_Status status = MPU6050_ReadData(mpu6050_handle, MPU6050_GYRO_XOUT_H, x_axis, 2);
  if (status != MPU6050_OK) {
    return status;
  }

  status = MPU6050_ReadData(mpu6050_handle, MPU6050_GYRO_YOUT_H, y_axis, 2);
  if (status != MPU6050_OK) {
    return status;
  }

  status = MPU6050_ReadData(mpu6050_handle, MPU6050_GYRO_ZOUT_H, z_axis, 2);
  if (status != MPU6050_OK) {
    return status;
  }
  
  gyro_data->x_axis = ((int16_t)x_axis[0] << 8) | x_axis[1];
  gyro_data->y_axis = ((int16_t)y_axis[0] << 8) | y_axis[1];
  gyro_data->z_axis = ((int16_t)z_axis[0] << 8) | z_axis[1];

  return status;
}

MPU6050_Status MPU6050_GetAllData_DMA(MPU6050_Handle* mpu6050_handle, uint8_t* dma_buffer, uint8_t size) {
  if (size < 14) {
    return MPU6050_ERROR;
  }

  return MPU6050_ReadData_DMA(mpu6050_handle, MPU6050_ACCEL_XOUT_H, dma_buffer, 14);
}
