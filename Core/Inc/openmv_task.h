#ifndef OPENMV_TASK_H
#define OPENMV_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

typedef enum
{
  OPENMV_STATE_IDLE = 0,
  OPENMV_STATE_SENDING_SELECT,
  OPENMV_STATE_WAIT_READY,
  OPENMV_STATE_SENDING_WAKEUP,
  OPENMV_STATE_WAIT_MODE,
  OPENMV_STATE_SENDING_MODE,
  OPENMV_STATE_CAPTURING,
  OPENMV_STATE_SENDING_RECORD_NAME,
  OPENMV_STATE_DONE,
  OPENMV_STATE_ERROR
} OpenMV_State;

typedef enum
{
  OPENMV_ERROR_NONE = 0,
  OPENMV_ERROR_UART,
  OPENMV_ERROR_READY_TIMEOUT,
  OPENMV_ERROR_TX_TIMEOUT
} OpenMV_Error;

typedef struct
{
  uint16_t x;
  uint16_t y;
  uint8_t width;
  uint8_t height;
  uint16_t center_x;
  uint16_t center_y;
} OpenMV_Target;

typedef struct
{
  OpenMV_State state;
  OpenMV_Error error;
  OpenMV_Target target;
  uint32_t target_count;
  uint32_t invalid_frame_count;
  uint32_t rx_overflow_count;
  uint32_t restart_count;
} OpenMV_Status;

void OpenMV_Init(UART_HandleTypeDef *uart);
void OpenMV_Start(void);
void OpenMV_Task(void);
void OpenMV_UART_RxCpltCallback(void);
void OpenMV_UART_TxCpltCallback(void);
void OpenMV_UART_ErrorCallback(void);
const OpenMV_Status *OpenMV_GetStatus(void);

#ifdef __cplusplus
}
#endif

#endif