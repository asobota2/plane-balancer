#include "logger.h"

#include <stdbool.h>
#include <string.h>

#include "usart.h"
#include "gpio.h"

#define LOG_BUFFER_SIZE 10000

static UART_HandleTypeDef* uart_handle = &huart2;

static char buffer[LOG_BUFFER_SIZE+1];
static int buffer_send_ptr = 0;
static int buffer_write_ptr = 0;

void LOG_PRIV(const char* msg) {
  uint32_t primask_bit = __get_PRIMASK();
  __disable_irq();
  int len = strlen(msg) + 1;
  int tmp_buffer_write_ptr = buffer_write_ptr;
  if (buffer_write_ptr + len > LOG_BUFFER_SIZE) {
    buffer[buffer_write_ptr] = '\0';
    tmp_buffer_write_ptr = 0;
  }

  bool buffer_overflow = len > LOG_BUFFER_SIZE;
  bool buffer_data_overwrite = tmp_buffer_write_ptr < buffer_send_ptr &&
                               tmp_buffer_write_ptr+len >= buffer_send_ptr;
  HAL_UART_StateTypeDef state = HAL_UART_GetState(uart_handle);
  bool is_sending = state == HAL_UART_STATE_BUSY_TX;

  if (buffer_overflow || (buffer_data_overwrite && is_sending)) {
    // TODO - signal log buffer error
    HAL_GPIO_WritePin(ERROR_LD_GPIO_Port, ERROR_LD_Pin, GPIO_PIN_SET);
    __set_PRIMASK(primask_bit);
    return;
  }

  buffer_write_ptr = tmp_buffer_write_ptr;
  if (!is_sending) {
    buffer_send_ptr = buffer_write_ptr;
  }

  strcpy(buffer+buffer_write_ptr, msg);
  buffer_write_ptr += len;
  
  if (!is_sending) {
    HAL_GPIO_WritePin(UART_BUSY_LD_GPIO_Port, UART_BUSY_LD_Pin, GPIO_PIN_SET);
    int len = strlen(buffer+buffer_send_ptr);
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(uart_handle, (uint8_t*)(buffer+buffer_send_ptr), len);
    // HAL_StatusTypeDef status = HAL_UART_Transmit(uart_handle, (uint8_t*)(buffer+buffer_send_ptr), len-1, 1000);
    if (status != HAL_OK) {
      __NOP();
    }
  }
  __set_PRIMASK(primask_bit);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart != uart_handle) {
    return;
  }

  uint32_t primask_bit = __get_PRIMASK();
  __disable_irq();
  buffer_send_ptr += strlen(buffer+buffer_send_ptr) + 1;
  if (buffer_send_ptr != buffer_write_ptr) {
    if (buffer[buffer_send_ptr] == '\0') {
      buffer_send_ptr = 0;
    }
    int len = strlen(buffer+buffer_send_ptr);
    HAL_UART_Transmit_DMA(uart_handle, (uint8_t*)(buffer+buffer_send_ptr), len);
  } else {
    HAL_GPIO_WritePin(UART_BUSY_LD_GPIO_Port, UART_BUSY_LD_Pin, GPIO_PIN_RESET);
  }
  __set_PRIMASK(primask_bit);
}
