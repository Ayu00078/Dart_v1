#include "openmv_task.h"
#include <stdio.h>
#include <string.h>

#define OPENMV_RX_RING_SIZE          128U
#define OPENMV_READY_TIMEOUT_MS      6000U
#define OPENMV_TX_TIMEOUT_MS          100U
#define OPENMV_MODE_DELAY_MS          100U
#define OPENMV_RECORD_NAME_DELAY_MS  3000U

#define OPENMV_SELECT_YAW             0x11U
#define OPENMV_WAKEUP                 0xABU
#define OPENMV_NORMAL_MODE            0x55U

static const uint8_t ready_marker[] =
{
  0xFFU, 'R', 'P', 'D', 'A', 'R', 'T'
};

static UART_HandleTypeDef *openmv_uart;
static OpenMV_Status openmv_status;
static uint8_t rx_byte;
static volatile uint8_t rx_ring[OPENMV_RX_RING_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static uint8_t tx_buffer[16];
static uint8_t record_name[16];
static volatile uint8_t tx_busy;
static uint32_t state_tick;
static uint8_t ready_index;
static uint8_t frame[7];
static uint8_t frame_index;

static void SetState(OpenMV_State state)
{
  openmv_status.state = state;
  state_tick = HAL_GetTick();
}

static uint8_t StartTransmit(const uint8_t *data, uint16_t length)
{
  if ((openmv_uart == NULL) || (tx_busy != 0U) || (length > sizeof(tx_buffer)))
  {
    return 0U;
  }

  memcpy(tx_buffer, data, length);
  tx_busy = 1U;
  if (HAL_UART_Transmit_IT(openmv_uart, tx_buffer, length) != HAL_OK)
  {
    tx_busy = 0U;
    return 0U;
  }
  state_tick = HAL_GetTick();
  return 1U;
}

static uint8_t PopRxByte(uint8_t *data)
{
  uint16_t tail = rx_tail;

  if (tail == rx_head)
  {
    return 0U;
  }
  *data = rx_ring[tail];
  rx_tail = (uint16_t)((tail + 1U) % OPENMV_RX_RING_SIZE);
  return 1U;
}

static void ParseReadyByte(uint8_t data)
{
  if (data == ready_marker[ready_index])
  {
    ready_index++;
    if (ready_index >= sizeof(ready_marker))
    {
      ready_index = 0U;
      SetState(OPENMV_STATE_SENDING_WAKEUP);
    }
  }
  else
  {
    ready_index = (data == ready_marker[0]) ? 1U : 0U;
  }
}

static void SaveTargetFrame(void)
{
  uint16_t x = (uint16_t)(((uint16_t)frame[1] << 8) | frame[2]);
  uint16_t y = (uint16_t)(((uint16_t)frame[3] << 8) | frame[4]);

  if ((x >= 320U) || (y >= 240U))
  {
    openmv_status.invalid_frame_count++;
    return;
  }

  openmv_status.target.x = x;
  openmv_status.target.y = y;
  openmv_status.target.width = frame[5];
  openmv_status.target.height = frame[6];
  openmv_status.target.center_x = (uint16_t)(x + ((uint16_t)frame[5] / 2U));
  openmv_status.target.center_y = (uint16_t)(y + ((uint16_t)frame[6] / 2U));
  openmv_status.target_count++;
}

static void ParseTargetByte(uint8_t data)
{
  if (frame_index == 0U)
  {
    if (data == 0xFFU)
    {
      frame[0] = data;
      frame_index = 1U;
    }
    return;
  }

  frame[frame_index++] = data;
  if (frame_index >= sizeof(frame))
  {
    frame_index = 0U;
    SaveTargetFrame();
  }
}

static void ProcessRx(void)
{
  uint8_t data;

  while (PopRxByte(&data) != 0U)
  {
    if (openmv_status.state == OPENMV_STATE_WAIT_READY)
    {
      ParseReadyByte(data);
    }
    else if ((openmv_status.state == OPENMV_STATE_CAPTURING) ||
             (openmv_status.state == OPENMV_STATE_SENDING_RECORD_NAME) ||
             (openmv_status.state == OPENMV_STATE_DONE))
    {
      ParseTargetByte(data);
    }
  }
}

void OpenMV_Init(UART_HandleTypeDef *uart)
{
  openmv_uart = uart;
  memset(&openmv_status, 0, sizeof(openmv_status));
  rx_head = 0U;
  rx_tail = 0U;
  tx_busy = 0U;
  ready_index = 0U;
  frame_index = 0U;
  openmv_status.state = OPENMV_STATE_IDLE;

  if ((openmv_uart == NULL) ||
      (HAL_UART_Receive_IT(openmv_uart, &rx_byte, 1U) != HAL_OK))
  {
    openmv_status.error = OPENMV_ERROR_UART;
    openmv_status.state = OPENMV_STATE_ERROR;
  }
}

void OpenMV_Start(void)
{
  if (openmv_uart == NULL)
  {
    openmv_status.error = OPENMV_ERROR_UART;
    openmv_status.state = OPENMV_STATE_ERROR;
    return;
  }

  openmv_status.error = OPENMV_ERROR_NONE;
  openmv_status.target_count = 0U;
  openmv_status.invalid_frame_count = 0U;
  openmv_status.rx_overflow_count = 0U;
  openmv_status.restart_count++;
  rx_tail = rx_head;
  ready_index = 0U;
  frame_index = 0U;
  SetState(OPENMV_STATE_SENDING_SELECT);
}

void OpenMV_Task(void)
{
  uint32_t now = HAL_GetTick();
  uint8_t command;
  int length;

  ProcessRx();

  switch (openmv_status.state)
  {
    case OPENMV_STATE_SENDING_SELECT:
      command = OPENMV_SELECT_YAW;
      if (StartTransmit(&command, 1U) != 0U)
      {
        SetState(OPENMV_STATE_WAIT_READY);
      }
      break;

    case OPENMV_STATE_WAIT_READY:
      if ((uint32_t)(now - state_tick) > OPENMV_READY_TIMEOUT_MS)
      {
        openmv_status.error = OPENMV_ERROR_READY_TIMEOUT;
        SetState(OPENMV_STATE_ERROR);
      }
      break;

    case OPENMV_STATE_SENDING_WAKEUP:
      command = OPENMV_WAKEUP;
      if (StartTransmit(&command, 1U) != 0U)
      {
        SetState(OPENMV_STATE_WAIT_MODE);
      }
      break;

    case OPENMV_STATE_WAIT_MODE:
      if ((tx_busy == 0U) && ((uint32_t)(now - state_tick) >= OPENMV_MODE_DELAY_MS))
      {
        SetState(OPENMV_STATE_SENDING_MODE);
      }
      break;

    case OPENMV_STATE_SENDING_MODE:
      command = OPENMV_NORMAL_MODE;
      if (StartTransmit(&command, 1U) != 0U)
      {
        SetState(OPENMV_STATE_CAPTURING);
      }
      break;

    case OPENMV_STATE_CAPTURING:
      if ((tx_busy == 0U) &&
          ((uint32_t)(now - state_tick) >= OPENMV_RECORD_NAME_DELAY_MS))
      {
        length = snprintf((char *)record_name, sizeof(record_name),
                          "2%014lu", (unsigned long)now);
        if (length == 15)
        {
          SetState(OPENMV_STATE_SENDING_RECORD_NAME);
        }
        else
        {
          openmv_status.error = OPENMV_ERROR_UART;
          SetState(OPENMV_STATE_ERROR);
        }
      }
      break;

    case OPENMV_STATE_SENDING_RECORD_NAME:
      if (StartTransmit(record_name, 15U) != 0U)
      {
        SetState(OPENMV_STATE_DONE);
      }
      break;

    case OPENMV_STATE_DONE:
    case OPENMV_STATE_IDLE:
    case OPENMV_STATE_ERROR:
    default:
      break;
  }

  if ((tx_busy != 0U) && ((uint32_t)(now - state_tick) > OPENMV_TX_TIMEOUT_MS))
  {
    openmv_status.error = OPENMV_ERROR_TX_TIMEOUT;
    SetState(OPENMV_STATE_ERROR);
  }
}

void OpenMV_UART_RxCpltCallback(void)
{
  uint16_t next = (uint16_t)((rx_head + 1U) % OPENMV_RX_RING_SIZE);

  if (next == rx_tail)
  {
    openmv_status.rx_overflow_count++;
  }
  else
  {
    rx_ring[rx_head] = rx_byte;
    rx_head = next;
  }

  if (HAL_UART_Receive_IT(openmv_uart, &rx_byte, 1U) != HAL_OK)
  {
    openmv_status.error = OPENMV_ERROR_UART;
    openmv_status.state = OPENMV_STATE_ERROR;
  }
}

void OpenMV_UART_TxCpltCallback(void)
{
  tx_busy = 0U;
}

void OpenMV_UART_ErrorCallback(void)
{
  tx_busy = 0U;
  if (HAL_UART_Receive_IT(openmv_uart, &rx_byte, 1U) != HAL_OK)
  {
    openmv_status.error = OPENMV_ERROR_UART;
    openmv_status.state = OPENMV_STATE_ERROR;
  }
}

const OpenMV_Status *OpenMV_GetStatus(void)
{
  return &openmv_status;
}