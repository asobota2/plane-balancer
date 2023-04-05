#ifndef BALANCER_SYSTEM_H_
#define BALANCER_SYSTEM_H_

#include "accel_gyro.h"
#include "servo.h"
#include "ball_reader.h"
#include "balancer_system_state.h"
#include "balancer_state_machine.h"
#include "pid.h"

typedef struct {
  AccelGyroHandle accel_gyro;
  ServoHandle servo_x;
  ServoHandle servo_y;
  BallReaderHandle ball_reader;
  StateMachineHandle state_machine;
  BS_State system_state;
  PID_Handle pid_x;
  PID_Handle pid_y;
} BalancerSystemHandle;

/**
 * @brief Initialize system
 * 
 * @param handle 
 */
void BS_Init(BalancerSystemHandle* handle);

/**
 * @brief Process one state of state machine and get next state
 * 
 * @param handle 
 */
void BS_RunOnce(BalancerSystemHandle* handle);

/**
 * @brief Callback to be called on interrupt from MPU6050
 * 
 * @param handle 
 */
void BS_MPU6050_IntHandle(BalancerSystemHandle* handle);

/**
 * @brief Callback to be called on UART RX transmision complete
 * 
 * @param handle 
 * @param huart 
 */
void BS_UART_RxCpltCallback(BalancerSystemHandle* handle, UART_HandleTypeDef *huart);

/**
 * @brief Callback to be called on I2C RX transmision complete
 * 
 * @param handle 
 * @param hi2c 
 */
void BS_I2C_MasterRxCpltCallback(BalancerSystemHandle* handle, I2C_HandleTypeDef *hi2c);

#endif  // BALANCER_SYSTEM_H_
