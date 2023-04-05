#include "configurator.h"

#include "i2c.h"
#include "tim.h"
#include "usart.h"

DevicesConfig CreateDevicesConfig() {
  DevicesConfig config;

  // Configure AccelGyro
  config.accel_gyro.hi2c = &hi2c1;

  // Configure Servo_X
  config.servo_x.htim = &htim2;
  config.servo_x.CCR = &htim2.Instance->CCR1;
  config.servo_x.CHANNEL_ID = TIM_CHANNEL_1;
  config.servo_x.CCR_MAX = 2500;
  config.servo_x.CCR_MIN = 500;
  config.servo_x.INIT_POS = 0;

  // Configure Servo_Y
  config.servo_y.htim = &htim2;
  config.servo_y.CCR = &htim2.Instance->CCR2;
  config.servo_y.CHANNEL_ID = TIM_CHANNEL_2;
  config.servo_y.CCR_MAX = 2500;
  config.servo_y.CCR_MIN = 500;
  config.servo_x.INIT_POS = 0;

  // Configure BallReader
  config.ball_reader.huart = &huart2;

  // Configure PID
  config.pid.kp = 0.03;
  config.pid.kd = 40;
  config.pid.ks = 0.001;

  return config;
}
