#include "ball_reader.h"

#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "stm32l0xx_hal.h"

#include "configurator.h"
#include "logger.h"

bool ParseUARTData(char* data, BallPos* ball_pos) {
  const char* start = strchr(data, '[');
  const char* end = strchr(data, ']');
  int len = end - start + 1;

  if (!start || !end || start > end || len != 16) {
    LOG("Failed to parse UART data!\r\n");
    return false;
  }

  data[5] = '\0';
  data[10] = '\0';
  data[15] = '\0';

  int x = atoi(data+1);
  int y = atoi(data+6);
  int time = atoi(data+11);

  ball_pos->x = x - 500;
  ball_pos->y = y - 500;
  ball_pos->time = time;
  return true;
}

BallReaderHandle BallReadere_Create(BallReaderConfig* config) {
  BallReaderHandle handle;
  handle.huart = config->huart;
  handle.new_data = false;
  handle.data_guard = false;

  return handle;
}

void BallReader_Start(BallReaderHandle* handle) {
  HAL_UART_Receive_DMA(handle->huart, (uint8_t*)&handle->ball_pos_dma_buff, BALL_POS_DMA_BUFF_SIZE);
}

BallPos BallReader_GetPos(BallReaderHandle* handle) {
  handle->data_guard = true;
  BallPos pos = handle->ball_pos;
  handle->new_data = false;
  handle->data_guard = false;

  return pos;
}

void BallReader_RxCpltCallback(BallReaderHandle* handle) {
  if (!handle->data_guard) {
    if (ParseUARTData(handle->ball_pos_dma_buff, &handle->ball_pos)) {
      handle->new_data = true;
    }
  }
}
