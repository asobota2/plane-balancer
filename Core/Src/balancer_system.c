#include "balancer_system.h"

#include <stdio.h>
#include <math.h>

#include "accel_gyro.h"
#include "configurator.h"
#include "balancer_state_machine.h"
#include "ball_reader.h"

#include "logger.h"

void BS_HandleInit(BalancerSystemHandle* handle);
void BS_HandleWaitForData(BalancerSystemHandle* handle);
void BS_HandleUpdatePID(BalancerSystemHandle* handle);
void BS_HandleCalcPlatformAngle(BalancerSystemHandle* handle);
void BS_HandleSetServos(BalancerSystemHandle* handle);

double LimitDouble(double num, double min, double max);

void BS_Init(BalancerSystemHandle* handle) {
  DevicesConfig devices_config = CreateDevicesConfig();

  handle->accel_gyro = AccelGyro_Create(&devices_config.accel_gyro);
  handle->servo_x = SERVO_Create(&devices_config.servo_x);
  handle->servo_y = SERVO_Create(&devices_config.servo_y);
  handle->state_machine = StateMachine_Create();
  handle->ball_reader = BallReadere_Create(&devices_config.ball_reader);
  handle->system_state.platform_pos.x = 0;
  handle->system_state.platform_pos.y = 0;
  handle->system_state.des_platform_pos.x = 0;
  handle->system_state.des_platform_pos.y = 0;
  handle->system_state.serwos_pos.x = 0;
  handle->system_state.serwos_pos.y = 0;
  handle->system_state.new_accel_gyro_data = &handle->accel_gyro.new_data;
  handle->system_state.new_ball_pos_data = &handle->ball_reader.new_data;
  handle->pid_x = PID_Create(&devices_config.pid);
  handle->pid_y = PID_Create(&devices_config.pid);
}

void BS_RunOnce(BalancerSystemHandle* handle) {
  switch (handle->state_machine.state) {
    case Init:
      BS_HandleInit(handle);
      break;
    case WaitForData:
      BS_HandleWaitForData(handle);
      break;
    case UpdatePID:
      BS_HandleUpdatePID(handle);
      break;
    case CalcPlatformAngle:
      BS_HandleCalcPlatformAngle(handle);
      break;
    case SetServos:
      BS_HandleSetServos(handle);
      break;
    default:
      // TODO - Report error
  }

  StateMachine_GetNextState(&handle->state_machine, &handle->system_state);
}

double LimitDouble(double num, double min, double max) {
  if (num > max) {
    return max;
  }
  if (num < min) {
    return min;
  }
  return num;
}

void BS_HandleInit(BalancerSystemHandle* handle) {
  AccelGyro_Init(&handle->accel_gyro);

  SERVO_SetPos(&handle->servo_x, 0);
  SERVO_SetPos(&handle->servo_y, 0);
  SERVO_Start(&handle->servo_x);
  SERVO_Start(&handle->servo_y);

  BallReader_Start(&handle->ball_reader);
}

void BS_HandleWaitForData(BalancerSystemHandle* handle) {

}

void BS_HandleUpdatePID(BalancerSystemHandle* handle) {
  BS_PosData* des_pos = &handle->system_state.des_platform_pos;

  BallPos ball_pos = BallReader_GetPos(&handle->ball_reader);

  PID_Update(&handle->pid_x, ball_pos.x, ball_pos.time);
  PID_Update(&handle->pid_y, ball_pos.y, ball_pos.time);

  des_pos->x = handle->pid_x.correction;
  des_pos->y = handle->pid_y.correction;
}

void BS_HandleCalcPlatformAngle(BalancerSystemHandle* handle) {
  const AccelGyroSample sample = AccelGyro_GetData(&handle->accel_gyro);
  const AccelGyroData data = sample.accelerometer;

  // char buff[100];
  // sprintf(buff, "X:%d, Y:%d, Z:%d\r\n", data.x, data.y, data.z);
  // LOG(buff);

  int len = sqrt((data.x*data.x) + (data.y*data.y) + (data.z*data.z));

  double x = data.x / (double)len;
  double y = data.y / (double)len;

  x = LimitDouble(x, -1, 1);
  y = LimitDouble(y, -1, 1);

  double tmp = x / sqrt(1 - (y*y));
  tmp = LimitDouble(tmp, -1, 1);

  double beta = asin(y); 
  double alpha = acos(tmp);

  alpha = alpha / M_PI * 180 - 90;
  beta = beta / M_PI * 180;

  alpha = alpha/180 * 1000;
  beta = beta/180 * 1000;

  handle->system_state.platform_pos.x = alpha;
  handle->system_state.platform_pos.y = beta;
}

void BS_HandleSetServos(BalancerSystemHandle* handle) {
  BS_PosData* des_pos = &handle->system_state.des_platform_pos;
  BS_PosData* pos = &handle->system_state.platform_pos;

  int x_delta = des_pos->y - pos->x - 30;
  int y_delta = des_pos->x - pos->y + 12;

  // int x_delta = 0 - pos->x - 30;
  // int y_delta = 0 - pos->y + 12;

  // int temp = x_delta;
  // x_delta = -y_delta;
  // y_delta = -temp;

  x_delta = -x_delta;

  const double magic_coef = 0.5;
  x_delta = x_delta*magic_coef;
  y_delta = y_delta*magic_coef;

  SERVO_ChangePos(&handle->servo_x, x_delta);
  SERVO_ChangePos(&handle->servo_y, y_delta);
}

void BS_MPU6050_IntHandle(BalancerSystemHandle* handle) {
  AccelGyro_IntPinCallback(&handle->accel_gyro);
}

void BS_UART_RxCpltCallback(BalancerSystemHandle* handle, UART_HandleTypeDef *huart) {
  if (huart == handle->ball_reader.huart) {
    BallReader_RxCpltCallback(&handle->ball_reader);
  }
}

void BS_I2C_MasterRxCpltCallback(BalancerSystemHandle* handle, I2C_HandleTypeDef *hi2c) {
  if (hi2c == handle->accel_gyro.mpu6050_handle.i2c_handle) {
    AccelGyro_RxCpltCallback(&handle->accel_gyro);
  }
}
