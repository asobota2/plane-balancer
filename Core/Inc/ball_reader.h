#ifndef BALL_READER_H_
#define BALL_READER_H_

#include <stdbool.h>

#include "stm32l0xx_hal.h"

#include "configurator.h"

#define BALL_POS_DMA_BUFF_SIZE 16

typedef struct {
  int x;
  int y;
  int time;
} BallPos;

typedef struct {
  UART_HandleTypeDef* huart;
  BallPos ball_pos;
  char ball_pos_dma_buff[BALL_POS_DMA_BUFF_SIZE];
  bool new_data;
  bool data_guard;
} BallReaderHandle;

/**
 * @brief Create ball reader handle
 * 
 * @param config 
 * @return BallReaderHandle 
 */
BallReaderHandle BallReadere_Create(BallReaderConfig* config);

/**
 * @brief Start reading ball position
 * 
 * @param handle 
 */
void BallReader_Start(BallReaderHandle* handle);

/**
 * @brief Get latest ball position
 * 
 * @param handle 
 * @return BallPos 
 */
BallPos BallReader_GetPos(BallReaderHandle* handle);

/**
 * @brief Callback to be called on UART RX transmision complete
 * 
 * @param handle 
 */
void BallReader_RxCpltCallback(BallReaderHandle* handle);

#endif  // BALL_READER_H_
