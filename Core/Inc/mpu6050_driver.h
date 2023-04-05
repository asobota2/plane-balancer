#ifndef MPU6050_H_
#define MPU6050_H_

#include <stdint.h>
#include <stdbool.h>

#include "stm32l0xx_hal.h"

#define MPU6050_I2C_TIMEOUT 100

#define MPU6050_SMPRT_DIV 0x19
#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_FIFO_EN 0x23
#define MPU6050_INT_PIN_CFG 0x37
#define MPU6050_INT_ENABLE 0x38
#define MPU6050_INT_STATUS 0x3A
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40
#define MPU6050_TEMP_OUT_H 0x41
#define MPU6050_TEMP_OUT_L 0x42
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_GYRO_XOUT_L 0x44
#define MPU6050_GYRO_YOUT_H 0x45
#define MPU6050_GYRO_YOUT_L 0x46
#define MPU6050_GYRO_ZOUT_H 0x47
#define MPU6050_GYRO_ZOUT_L 0x48
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_SIGNAL_PATH_RESET 0x68
#define MPU6050_FIFO_R_W 0x74

typedef enum {
  MPU6050_OK, 
  MPU6050_ERROR
} MPU6050_Status;

typedef struct {
  uint8_t address;
  I2C_HandleTypeDef* i2c_handle;
  uint32_t timeout;
} MPU6050_Handle;

typedef struct {
  bool INT_LEVEL;
  bool INT_OPEN;
  bool LATCH_INT_EN;
  bool INT_RD_CLEAR;
  bool FSYNC_INT_LEVEL;
  bool FSYNC_INT_EN;
} MPU6050_ItConfig;

typedef struct {
  bool fifo_ovf_en;
  bool i2c_mst_en;
  bool data_rdy_en;
} MPU6050_Interrupts;

typedef struct {
  bool TEMP_FIFO_EN;
  bool XG_FIFO_EN;
  bool YG_FIFO_EN;
  bool ZG_FIFO_EN;
  bool ACCEL_FIFO_EN;
} MPU6050_FIFOConfig;

typedef struct {
  int16_t x_axis;
  int16_t y_axis;
  int16_t z_axis;
} MPU6050_Data;

/**
 * @brief Creates handle used to manipulate MPU6050
 * 
 * @param AD0_pin_state State of AD0 pin sets device address LSB
 * @param i2c_handle Handle to I2C connected to MPU6050
 * @return MPU6050_Handle 
 */
MPU6050_Handle MPU6050_CreateHandle(bool AD0_pin_state, I2C_HandleTypeDef* i2c_handle);

MPU6050_ItConfig MPU6050_CreateItConfig();
MPU6050_Interrupts MPU6050_CreateInterrupts();
MPU6050_FIFOConfig MPU6050_CreateFIFOCOnfig();
MPU6050_Data MPU6050_CreateData();

/**
 * @brief Reset MPU6050. Sets all registers to 0 except:
 * Register 107: 0x40 
 * Register 117: 0x68
 * 
 * @param mpu6050_handle 
 * @return MPU6050_Status 
 */
MPU6050_Status MPU6050_Reset(MPU6050_Handle* mpu6050_handle);

/**
 * @brief Initializes MPU6050.
 * 
 * @param mpu6050_handle 
 * @return MPU6050_Status 
 */
MPU6050_Status MPU6050_Init(MPU6050_Handle* mpu6050_handle);

/**
 * @brief Set sampling rate of MPU6050. Refere to register 25 for more detail.
 * 
 * @param mpu6050_handle 
 * @param SMPRT_DIV 
 * @return MPU6050_Status 
 */
MPU6050_Status MPU6050_SetSampleRate(MPU6050_Handle* mpu6050_handle, uint8_t SMPRT_DIV);

/**
 * @brief Set configuration of MPU6050. For details refer to Register 26.
 * 
 * @param mpu6050_handle 
 * @param ext_sync_set 
 * @param dlpf_cfg 
 * @return MPU6050_Status 
 */
MPU6050_Status MPU6050_SetConfig(MPU6050_Handle* mpu6050_handle, uint8_t ext_sync_set, uint8_t dlpf_cfg);

/**
 * @brief Sets range of gyroscope sensor.
 * 
 * @param mpu6050_handle 
 * @param FS_SEL Full scale range:
 *  - 0: +/- 250
 *  - 1: +/- 500
 *  - 2: +/- 1000
 *  - 3: +/- 2000
 * @return MPU6050_Status 
 */
MPU6050_Status MPU6050_SetGyroFullScale(MPU6050_Handle* mpu6050_handle, uint8_t FS_SEL);

/**
 * @brief Sets range of accelerometer sensor.
 * 
 * @param mpu6050_handle 
 * @param AFS_SEL Full scale range:
 *  - 0: +/- 2g
 *  - 1: +/- 4g
 *  - 2: +/- 8g
 *  - 3: +/- 16g
 * @return MPU6050_Status 
 */
MPU6050_Status MPU6050_SetAccelFullScale(MPU6050_Handle* mpu6050_handle, uint8_t AFS_SEL);

/**
 * @brief Configure interrupt pin.
 * 
 * @param mpu6050_handle 
 * @param it_config 
 * @return MPU6050_Status 
 */
MPU6050_Status MPU6050_SetItConfig(MPU6050_Handle* mpu6050_handle, MPU6050_ItConfig* it_config);

/**
 * @brief Enable interrupts.
 * 
 * @param mpu6050_handle 
 * @param interrupts 
 * @return MPU6050_Status 
 */
MPU6050_Status MPU6050_SetItEnabled(MPU6050_Handle* mpu6050_handle, MPU6050_Interrupts* interrupts);

/**
 * @brief Get interrupts status.
 * 
 * @param mpu6050_handle 
 * @param interrupts Will be filled with status of interrupts.
 * @return MPU6050_Status 
 */
MPU6050_Status MPU6050_GetItStatus(MPU6050_Handle* mpu6050_handle, MPU6050_Interrupts* interrupts);

/**
 * @brief Set data loaded into FIFO buffer.
 * 
 * @param mpu6050_handle 
 * @param fifo_config 
 * @return MPU6050_Status 
 */
MPU6050_Status MPU6050_SetFIFOEnabled(MPU6050_Handle* mpu6050_handle, MPU6050_FIFOConfig* fifo_config);

/**
 * @brief Get accelerometer measurement.
 * 
 * @param mpu6050_handle 
 * @param accel_data Struct to be filled with data.
 * @return MPU6050_Status 
 */
MPU6050_Status MPU6050_GetAccel(MPU6050_Handle* mpu6050_handle, MPU6050_Data* accel_data);

/**
 * @brief Get accelerometer measurement using DMA. Because of register layout bytes need to be rearranged.
 * Refere to regiter 59-64 for more details.
 * 
 * @param mpu6050_handle 
 * @param dma_buffer Buffor for data. Needs to be at least 6 bytes long.
 * @param size Size of the buffer
 * @return MPU6050_Status 
 */
MPU6050_Status MPU6050_GetAccel_DMA(MPU6050_Handle* mpu6050_handle, uint8_t* dma_buffer, uint8_t size);

/**
 * @brief Get temperature measurement.
 * 
 * @param mpu6050_handle 
 * @param temp Value to be filled with temperature.
 * @return MPU6050_Status 
 */
MPU6050_Status MPU6050_GetTemp(MPU6050_Handle* mpu6050_handle, int16_t* temp);

/**
 * @brief Get gyroscope measurement.
 * 
 * @param mpu6050_handle 
 * @param gyro_data Struct to be filled with data.
 * @return MPU6050_Status 
 */
MPU6050_Status MPU6050_GetGyro(MPU6050_Handle* mpu6050_handle, MPU6050_Data* gyro_data);

/**
 * @brief Read accel, gyro and temp from MPU6050 using DMA. 
 *        Data in buffer is going to be: ACCEL TEMP GYRO
 * 
 * @param mpu6050_handle 
 * @param dma_buffer 
 * @param size 
 * @return MPU6050_Status 
 */
MPU6050_Status MPU6050_GetAllData_DMA(MPU6050_Handle* mpu6050_handle, uint8_t* dma_buffer, uint8_t size);

#endif
