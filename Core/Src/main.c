/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "imu_task.h"
#include "openmv_task.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  POWER_WAIT_START_RELEASE,
  POWER_WAIT_PRESS,
  POWER_TIMING_HOLD,
  POWER_WAIT_RELEASE,
  POWER_TIMING_RELEASE
} PowerButtonState;
typedef enum
{
  FLIGHT_IDLE, FLIGHT_WAIT_LAUNCH, FLIGHT_ASCENDING, FLIGHT_LANDING,
  FLIGHT_IMU_ERROR
} FlightState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RXBUFFERSIZE 32U
/* Existing endpoint commands used these values; verify actual servo and linkage
   before energizing. Never widen this envelope without hardware validation. */
#define SERVO_MIN_PULSE_US 1000U
#define SERVO_MAX_PULSE_US 2000U
#define SERVO_DEFAULT_PULSE_US 1620U
#define SERVO_1_TAKEOFF_PULSE_US 1620U
#define SERVO_2_TAKEOFF_PULSE_US 1620U
#define SERVO_3_TAKEOFF_PULSE_US 1620U
#define SERVO_4_TAKEOFF_PULSE_US 1620U
#define SERVO_1_LANDING_PULSE_US 1620U
#define SERVO_2_LANDING_PULSE_US 1620U
#define SERVO_3_LANDING_PULSE_US 1620U
#define SERVO_4_LANDING_PULSE_US 1620U
#define SERVO_PWM_PERIOD_US 20000U
#define FLIGHT_AUTO_ARM_MS 10000U
#define ARM_BEEP_LENGTH_MS 150U
#define ARM_BEEP_INTERVAL_MS 300U
#define ARM_BEEP_TOTAL_MS (ARM_BEEP_INTERVAL_MS + ARM_BEEP_LENGTH_MS)
#if (SERVO_DEFAULT_PULSE_US < SERVO_MIN_PULSE_US) || (SERVO_DEFAULT_PULSE_US > SERVO_MAX_PULSE_US) || \
    (SERVO_1_TAKEOFF_PULSE_US < SERVO_MIN_PULSE_US) || (SERVO_1_TAKEOFF_PULSE_US > SERVO_MAX_PULSE_US) || \
    (SERVO_2_TAKEOFF_PULSE_US < SERVO_MIN_PULSE_US) || (SERVO_2_TAKEOFF_PULSE_US > SERVO_MAX_PULSE_US) || \
    (SERVO_3_TAKEOFF_PULSE_US < SERVO_MIN_PULSE_US) || (SERVO_3_TAKEOFF_PULSE_US > SERVO_MAX_PULSE_US) || \
    (SERVO_4_TAKEOFF_PULSE_US < SERVO_MIN_PULSE_US) || (SERVO_4_TAKEOFF_PULSE_US > SERVO_MAX_PULSE_US) || \
    (SERVO_1_LANDING_PULSE_US < SERVO_MIN_PULSE_US) || (SERVO_1_LANDING_PULSE_US > SERVO_MAX_PULSE_US) || \
    (SERVO_2_LANDING_PULSE_US < SERVO_MIN_PULSE_US) || (SERVO_2_LANDING_PULSE_US > SERVO_MAX_PULSE_US) || \
    (SERVO_3_LANDING_PULSE_US < SERVO_MIN_PULSE_US) || (SERVO_3_LANDING_PULSE_US > SERVO_MAX_PULSE_US) || \
    (SERVO_4_LANDING_PULSE_US < SERVO_MIN_PULSE_US) || (SERVO_4_LANDING_PULSE_US > SERVO_MAX_PULSE_US)
#error "Servo preset pulse is outside the configured envelope"
#endif
/* Launch gate: tune acceleration magnitude and continuous time separately. */
#define FLIGHT_LAUNCH_ACCEL_MPS2 13.0f
#define FLIGHT_LAUNCH_CONFIRM_MS 6U
/* Consecutive positive roll updates required after a negative-to-positive crossing. */
#define FLIGHT_ROLL_POSITIVE_SAMPLES 5U
#define FLIGHT_BEEP_MS 3000U
#define FLIGHT_LANDING_HOLD_MS 5000U
#define BATTERY_ADC_SAMPLE_COUNT 4U
#define BATTERY_ADC_FULL_SCALE 4095U
#define BATTERY_ADC_REFERENCE_MV 3300U
#define BATTERY_DIVIDER_RATIO 4U
#define BUZZER_LOW_PULSE 10U
#define BUZZER_MEDIUM_PULSE 20U
#define BUZZER_HIGH_PULSE 40U
#define POWER_OFF_HOLD_MS 1000U
#define POWER_OFF_RELEASE_MS 1000U
#define POWER_RELEASE_BEEP_MS 100U
#define EFFECT_BEEP_LENGTH_MS 180U
#define EFFECT_BEEP_INTERVAL_MS 300U
#define EFFECT_LENGTH_MS ((2U * EFFECT_BEEP_INTERVAL_MS) + EFFECT_BEEP_LENGTH_MS)
#define UART_TX_BUFFER_SIZE 256U
#define IMU_DEBUG_PERIOD_MS 200U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static char RxBuffer[RXBUFFERSIZE];
static char command_buffer[RXBUFFERSIZE];
static uint8_t aRxBuffer;
static uint8_t Uart2_Rx_Cnt;
static volatile uint8_t command_ready;
static volatile uint8_t rx_discard;
static uint32_t takeoff_pulse[4] = { SERVO_1_TAKEOFF_PULSE_US, SERVO_2_TAKEOFF_PULSE_US,
                                     SERVO_3_TAKEOFF_PULSE_US, SERVO_4_TAKEOFF_PULSE_US };
static uint32_t landing_pulse[4] = { SERVO_1_LANDING_PULSE_US, SERVO_2_LANDING_PULSE_US,
                                     SERVO_3_LANDING_PULSE_US, SERVO_4_LANDING_PULSE_US };
static FlightState flight_state;
static uint32_t launch_tick;
static uint32_t launch_candidate_tick;
static uint32_t apex_tick;
static uint32_t flight_last_update;
static uint8_t launch_candidate;
static uint8_t roll_was_negative;
static uint8_t roll_positive_count;
static uint8_t apex_event;
static uint8_t servo_output_active = 1U;
static uint8_t battery_report_pending;
static PowerButtonState power_button_state;
static uint32_t power_button_tick;
static uint32_t power_beep_tick;
static uint32_t startup_effect_tick;
static uint8_t startup_effect_active;
static uint32_t boot_tick;
static uint32_t arm_beep_tick;
static uint8_t arm_beep_active;
static uint8_t uart_tx_buffer[UART_TX_BUFFER_SIZE];
static volatile uint8_t uart_tx_busy;
static volatile uint8_t uart_overflow_pending;
static uint32_t uart_tx_drop_count;
static uint32_t imu_debug_tick;
static uint32_t openmv_reported_target_count;
static OpenMV_State openmv_reported_state;
static OpenMV_Error openmv_reported_error;
static const uint8_t uart_reply[] = "ok\r\n";
static const uint8_t uart_error[] = "error\r\n";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void SetServoPulse(uint8_t index, uint32_t pulse);
static uint8_t ReadBatteryVoltage(uint32_t *voltage_mv);
static void BatteryReportTask(void);
static void HandleFlightCommand(void);
static void FlightTask(uint32_t now);
static uint8_t ArmFlight(uint32_t now);
static void AutoArmTask(uint32_t now);
static uint32_t GetThreeBeepPulse(uint32_t elapsed, uint8_t increasing);
static void UpdateBuzzer(uint32_t now);
static void HandlePowerButton(void);
static uint8_t QueueUartData(const uint8_t *data, uint16_t length);
static void IMUDebugOutputTask(uint32_t now);
static void OpenMVDebugOutputTask(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void SetServoPulse(uint8_t index, uint32_t pulse)
{
  switch (index)
  {
    case 0U: __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse); break;
    case 1U: __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse); break;
    case 2U: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse); break;
    case 3U: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse); break;
    default: break;
  }
}

static uint8_t ReadBatteryVoltage(uint32_t *voltage_mv)
{
  uint32_t sum = 0U;
  uint32_t i;

  if (voltage_mv == NULL) return 0U;
  for (i = 0U; i < BATTERY_ADC_SAMPLE_COUNT; i++)
  {
    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
      (void)HAL_ADC_Stop(&hadc1);
      return 0U;
    }
    if (HAL_ADC_PollForConversion(&hadc1, 1U) != HAL_OK)
    {
      (void)HAL_ADC_Stop(&hadc1);
      return 0U;
    }
    sum += HAL_ADC_GetValue(&hadc1);
    if (HAL_ADC_Stop(&hadc1) != HAL_OK) return 0U;
  }
  *voltage_mv = (((sum / BATTERY_ADC_SAMPLE_COUNT) *
                  BATTERY_ADC_REFERENCE_MV * BATTERY_DIVIDER_RATIO) +
                 (BATTERY_ADC_FULL_SCALE / 2U)) / BATTERY_ADC_FULL_SCALE;
  return 1U;
}

static void BatteryReportTask(void)
{
  char message[UART_TX_BUFFER_SIZE];
  uint32_t voltage_mv;
  int length;

  if ((battery_report_pending == 0U) || (uart_tx_busy != 0U)) return;
  if (ReadBatteryVoltage(&voltage_mv) != 0U)
  {
    length = snprintf(message, sizeof(message), "Battery:%lu.%02luV\r\n",
                      (unsigned long)(voltage_mv / 1000U),
                      (unsigned long)((voltage_mv % 1000U) / 10U));
  }
  else length = snprintf(message, sizeof(message), "Battery ADC error\r\n");
  if ((length > 0) && ((uint32_t)length < sizeof(message)) &&
      (QueueUartData((const uint8_t *)message, (uint16_t)length) != 0U))
    battery_report_pending = 0U;
}

static uint32_t GetThreeBeepPulse(uint32_t elapsed, uint8_t increasing)
{
  static const uint32_t increasing_pulses[3] =
  {
    BUZZER_LOW_PULSE, BUZZER_MEDIUM_PULSE, BUZZER_HIGH_PULSE
  };
  static const uint32_t decreasing_pulses[3] =
  {
    BUZZER_HIGH_PULSE, BUZZER_MEDIUM_PULSE, BUZZER_LOW_PULSE
  };
  uint32_t beep_index = elapsed / EFFECT_BEEP_INTERVAL_MS;

  if ((beep_index >= 3U) ||
      ((elapsed % EFFECT_BEEP_INTERVAL_MS) >= EFFECT_BEEP_LENGTH_MS))
  {
    return 0U;
  }

  return increasing ? increasing_pulses[beep_index]
                    : decreasing_pulses[beep_index];
}

static void UpdateBuzzer(uint32_t now)
{
  uint32_t pulse;

  if ((apex_event != 0U) && ((uint32_t)(now - apex_tick) < FLIGHT_BEEP_MS))
  {
    pulse = BUZZER_HIGH_PULSE;
  }
  else if (power_button_state == POWER_WAIT_RELEASE)
  {
    pulse = ((uint32_t)(now - power_beep_tick) < POWER_RELEASE_BEEP_MS)
              ? BUZZER_HIGH_PULSE : 0U;
  }
  else if (power_button_state == POWER_TIMING_RELEASE)
  {
    pulse = GetThreeBeepPulse((uint32_t)(now - power_button_tick), 0U);
  }
  else if (arm_beep_active != 0U)
  {
    uint32_t elapsed = (uint32_t)(now - arm_beep_tick);
    if (elapsed < ARM_BEEP_TOTAL_MS)
    {
      pulse = ((elapsed < ARM_BEEP_LENGTH_MS) ||
               ((elapsed >= ARM_BEEP_INTERVAL_MS) &&
                (elapsed < ARM_BEEP_TOTAL_MS))) ? BUZZER_HIGH_PULSE : 0U;
    }
    else
    {
      arm_beep_active = 0U;
      pulse = 0U;
    }
  }
  else if (startup_effect_active != 0U)
  {
    uint32_t elapsed = (uint32_t)(now - startup_effect_tick);

    if (elapsed < EFFECT_LENGTH_MS)
    {
      pulse = GetThreeBeepPulse(elapsed, 1U);
    }
    else
    {
      startup_effect_active = 0U;
      pulse = 0U;
    }
  }
  else
  {
    pulse = 0U;
  }

  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse);
}

static void HandlePowerButton(void)
{
  GPIO_PinState button = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
  uint32_t now = HAL_GetTick();

  switch (power_button_state)
  {
    case POWER_WAIT_START_RELEASE:
      if (button == GPIO_PIN_RESET)
      {
        power_button_state = POWER_WAIT_PRESS;
      }
      break;

    case POWER_WAIT_PRESS:
      if (button == GPIO_PIN_SET)
      {
        power_button_tick = now;
        power_button_state = POWER_TIMING_HOLD;
      }
      break;

    case POWER_TIMING_HOLD:
      if (button == GPIO_PIN_RESET)
      {
        power_button_state = POWER_WAIT_PRESS;
      }
      else if ((uint32_t)(now - power_button_tick) >= POWER_OFF_HOLD_MS)
      {
        power_beep_tick = now;
        power_button_state = POWER_WAIT_RELEASE;
      }
      break;

    case POWER_WAIT_RELEASE:
      if (button == GPIO_PIN_RESET)
      {
        power_button_tick = now;
        power_button_state = POWER_TIMING_RELEASE;
      }
      break;

    case POWER_TIMING_RELEASE:
      if (button == GPIO_PIN_SET)
      {
        power_button_state = POWER_WAIT_RELEASE;
      }
      else if ((uint32_t)(now - power_button_tick) >= POWER_OFF_RELEASE_MS)
      {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0U);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
        while (1)
        {
          /* Keep the power-off request asserted until the supply is removed. */
        }
      }
      break;

    default:
      power_button_state = POWER_WAIT_START_RELEASE;
      break;
  }
}

static uint8_t QueueUartData(const uint8_t *data, uint16_t length)
{
  HAL_StatusTypeDef result;

  if ((data == NULL) || (length == 0U) || (length > UART_TX_BUFFER_SIZE))
  {
    return 0U;
  }

  __disable_irq();
  if (uart_tx_busy != 0U)
  {
    __enable_irq();
    uart_tx_drop_count++;
    return 0U;
  }
  memcpy(uart_tx_buffer, data, length);
  uart_tx_busy = 1U;
  result = HAL_UART_Transmit_IT(&huart2, uart_tx_buffer, length);
  if (result != HAL_OK)
  {
    uart_tx_busy = 0U;
  }
  __enable_irq();
  return (result == HAL_OK) ? 1U : 0U;
}

static uint8_t ArmFlight(uint32_t now)
{
  const IMU_Status *status = IMU_GetStatus();
  uint8_t i;

  if ((flight_state == FLIGHT_ASCENDING) ||
      (status->state != IMU_STATE_RUNNING) ||
      ((apex_event != 0U) &&
       ((uint32_t)(now - apex_tick) < FLIGHT_LANDING_HOLD_MS))) return 0U;

  for (i = 0U; i < 4U; i++) SetServoPulse(i, takeoff_pulse[i]);
  servo_output_active = 1U;
  flight_state = FLIGHT_WAIT_LAUNCH;
  launch_candidate = 0U;
  roll_was_negative = 0U;
  roll_positive_count = 0U;
  apex_event = 0U;
  flight_last_update = status->attitude_update_count;
  return 1U;
}

static void AutoArmTask(uint32_t now)
{
  if ((flight_state != FLIGHT_IDLE) ||
      ((uint32_t)(now - boot_tick) < FLIGHT_AUTO_ARM_MS) ||
      (IMU_GetStatus()->state != IMU_STATE_RUNNING)) return;

  if (ArmFlight(now) != 0U)
  {
    startup_effect_active = 0U;
    arm_beep_tick = now;
    arm_beep_active = 1U;
  }
}

static void HandleFlightCommand(void)
{
  char line[RXBUFFERSIZE];
  uint8_t length;
  uint8_t number;
  uint8_t index;
  uint32_t pulse = 0U;
  uint8_t valid = 1U;
  uint8_t i;
  uint8_t digits = 0U;

  if (command_ready == 0U)
  {
    return;
  }
  __disable_irq();
  length = command_ready;
  memcpy(line, command_buffer, length);
  command_ready = 0U;
  __enable_irq();
  line[length] = '\0';
  if ((length == 1U) && (line[0] == '0'))
  {
    battery_report_pending = 1U;
    return;
  }
  if ((length == 1U) && (line[0] == '5'))
  {
    valid = ArmFlight(HAL_GetTick());
  }
  else if ((length >= 6U) && (line[0] >= '1') && (line[0] <= '9') &&
           (line[1] == ' '))
  {
    number = (uint8_t)(line[0] - '0');
    valid = ((number >= 1U && number <= 4U) ||
             (number >= 6U && number <= 9U)) ? 1U : 0U;
    for (i = 2U; i < length; i++)
    {
      if ((line[i] < '0') || (line[i] > '9') || (pulse > SERVO_PWM_PERIOD_US))
      {
        valid = 0U;
        break;
      }
      pulse = pulse * 10U + (uint32_t)(line[i] - '0');
      digits++;
    }
    if ((digits == 0U) ||
        (pulse < SERVO_MIN_PULSE_US) || (pulse > SERVO_MAX_PULSE_US) ||
        (pulse >= SERVO_PWM_PERIOD_US)) valid = 0U;
    if (valid != 0U)
    {
      index = (number <= 4U) ? (uint8_t)(number - 1U) : (uint8_t)(number - 6U);
      if (number <= 4U)
      {
        /* Landing is latched: never silently override its output. */
        if ((flight_state == FLIGHT_LANDING) ||
            (flight_state == FLIGHT_ASCENDING)) valid = 0U;
        else
        {
          takeoff_pulse[index] = pulse;
          SetServoPulse(index, pulse);
          servo_output_active = 1U;
        }
      }
      else if ((flight_state == FLIGHT_WAIT_LAUNCH) ||
               (flight_state == FLIGHT_ASCENDING)) valid = 0U;
      else
      {
        landing_pulse[index] = pulse;
      }
    }
  }
  else valid = 0U;
  (void)QueueUartData(valid ? uart_reply : uart_error,
                      valid ? (sizeof(uart_reply) - 1U) : (sizeof(uart_error) - 1U));
}

static void FlightTask(uint32_t now)
{
  const IMU_Status *imu = IMU_GetStatus();
  uint8_t i;
  if ((flight_state == FLIGHT_LANDING) && (servo_output_active != 0U) &&
      ((uint32_t)(now - apex_tick) >= FLIGHT_LANDING_HOLD_MS))
  {
    for (i = 0U; i < 4U; i++) SetServoPulse(i, 0U);
    servo_output_active = 0U;
  }
  if ((flight_state == FLIGHT_WAIT_LAUNCH) || (flight_state == FLIGHT_ASCENDING))
  {
    if (imu->state != IMU_STATE_RUNNING)
    {
      flight_state = FLIGHT_IMU_ERROR;
      return;
    }
    if (imu->attitude_update_count == flight_last_update) return;
    flight_last_update = imu->attitude_update_count;
    if (flight_state == FLIGHT_WAIT_LAUNCH)
    {
      if (imu->accel_norm_mps2 >= FLIGHT_LAUNCH_ACCEL_MPS2)
      {
        if (launch_candidate == 0U)
        {
          launch_candidate_tick = now;
          launch_candidate = 1U;
        }
        else if ((uint32_t)(now - launch_candidate_tick) >= FLIGHT_LAUNCH_CONFIRM_MS)
        {
          launch_tick = now;
          launch_candidate = 0U;
          roll_was_negative = 0U;
          roll_positive_count = 0U;
          flight_state = FLIGHT_ASCENDING;
        }
      }
      else launch_candidate = 0U;
    }
    else
    {
      /* Only post-launch attitude updates count. Zero breaks a positive run;
         a later negative sample allows another crossing attempt. */
      if (imu->roll_deg < 0.0f)
      {
        roll_was_negative = 1U;
        roll_positive_count = 0U;
      }
      else if (imu->roll_deg > 0.0f)
      {
        if (roll_was_negative != 0U)
        {
          roll_positive_count++;
          if (roll_positive_count >= FLIGHT_ROLL_POSITIVE_SAMPLES)
          {
            flight_state = FLIGHT_LANDING;
            apex_event = 1U;
            apex_tick = now;
            for (i = 0U; i < 4U; i++) SetServoPulse(i, landing_pulse[i]);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, BUZZER_HIGH_PULSE);
          }
        }
      }
      else
      {
        roll_was_negative = 0U;
        roll_positive_count = 0U;
      }
    }
  }
}

static void IMUDebugOutputTask(uint32_t now)
{
  const IMU_Status *status;
  char message[UART_TX_BUFFER_SIZE];
  int length;
  int32_t roll;
  int32_t pitch;
  int32_t yaw;

  if ((uint32_t)(now - imu_debug_tick) < IMU_DEBUG_PERIOD_MS)
  {
    return;
  }
  imu_debug_tick = now;
  status = IMU_GetStatus();
  roll = (int32_t)(status->roll_deg * 10.0f);
  pitch = (int32_t)(status->pitch_deg * 10.0f);
  yaw = (int32_t)(status->yaw_deg * 10.0f);
  /* R/P/Y remain in tenths of a degree for existing parsers; X/Y/Z are
     the same Euler angles (roll/pitch/yaw) formatted in degrees. */
  length = snprintf(message, sizeof(message),
                     "ms:%lu F:%u I:%u e:%u R:%ld P:%ld Y:%ld Xdeg:%s%ld.%01ld Ydeg:%s%ld.%01ld Zdeg:%s%ld.%01ld A:%lu G:%lu C:%u T:%lu V:%u B:%u S:%u U:%lu O:%lu E:%lu\r\n",
                     (unsigned long)now, (unsigned int)flight_state, (unsigned int)status->state,
                     (unsigned int)status->error, (long)roll, (long)pitch, (long)yaw,
                     (roll < 0) ? "-" : "", (long)((roll < 0) ? -roll : roll) / 10L,
                     (long)((roll < 0) ? -roll : roll) % 10L,
                     (pitch < 0) ? "-" : "", (long)((pitch < 0) ? -pitch : pitch) / 10L,
                     (long)((pitch < 0) ? -pitch : pitch) % 10L,
                     (yaw < 0) ? "-" : "", (long)((yaw < 0) ? -yaw : yaw) / 10L,
                     (long)((yaw < 0) ? -yaw : yaw) % 10L,
                     (unsigned long)(status->accel_norm_mps2 * 100.0f),
                     (unsigned long)(status->gyro_norm_rps * 100.0f),
                     (unsigned int)launch_candidate,
                     (unsigned long)((flight_state == FLIGHT_ASCENDING) ? (now - launch_tick) : 0U),
                     (unsigned int)apex_event,
                     (unsigned int)(apex_event && ((uint32_t)(now - apex_tick) < FLIGHT_BEEP_MS)),
                     (unsigned int)servo_output_active,
                     (unsigned long)status->attitude_update_count,
                     (unsigned long)status->gyro_overrun_count,
                     (unsigned long)status->spi_error_count);
  if ((length > 0) && ((uint32_t)length < sizeof(message)))
  {
    (void)QueueUartData((const uint8_t *)message, (uint16_t)length);
  }
}

static void OpenMVDebugOutputTask(void)
{
  const OpenMV_Status *status = OpenMV_GetStatus();
  char message[UART_TX_BUFFER_SIZE];
  int length = 0;

  if (status->target_count != openmv_reported_target_count)
  {
    length = snprintf(message, sizeof(message),
                      "OpenMV x:%u y:%u w:%u h:%u cx:%u cy:%u\r\n",
                      (unsigned int)status->target.x,
                      (unsigned int)status->target.y,
                      (unsigned int)status->target.width,
                      (unsigned int)status->target.height,
                      (unsigned int)status->target.center_x,
                      (unsigned int)status->target.center_y);
    if ((length > 0) &&
        (QueueUartData((const uint8_t *)message, (uint16_t)length) != 0U))
    {
      openmv_reported_target_count = status->target_count;
    }
  }
  else if ((status->state == OPENMV_STATE_ERROR) &&
           ((status->state != openmv_reported_state) ||
            (status->error != openmv_reported_error)))
  {
    length = snprintf(message, sizeof(message), "OpenMV error:%u\r\n",
                      (unsigned int)status->error);
    if ((length > 0) &&
        (QueueUartData((const uint8_t *)message, (uint16_t)length) != 0U))
    {
      openmv_reported_state = status->state;
      openmv_reported_error = status->error;
    }
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_SPI1_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  if ((HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK) ||
      (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK) ||
      (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK) ||
      (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK))
  {
    Error_Handler();
  }
  {
    uint8_t i;
    for (i = 0U; i < 4U; i++) SetServoPulse(i, SERVO_DEFAULT_PULSE_US);
  }
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0U);
  if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  boot_tick = HAL_GetTick();
  startup_effect_tick = boot_tick;
  startup_effect_active = 1U;
  if (HAL_UART_Receive_IT(&huart2, &aRxBuffer, 1) != HAL_OK)
  {
    Error_Handler();
  }
  IMU_Init(&hspi1);
  imu_debug_tick = HAL_GetTick();
  OpenMV_Init(&huart6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();
    IMU_Task();
    HandleFlightCommand();
    AutoArmTask(now);
    FlightTask(now);
    BatteryReportTask();
    OpenMV_Task();
    HandlePowerButton();
    UpdateBuzzer(now);
    if (uart_overflow_pending != 0U)
    {
      static const uint8_t overflow_message[] = "UART RX overflow\r\n";
      if (QueueUartData(overflow_message, sizeof(overflow_message) - 1U))
        uart_overflow_pending = 0U;
    }
    IMUDebugOutputTask(now);
    OpenMVDebugOutputTask();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART6)
  {
    OpenMV_UART_RxCpltCallback();
    return;
  }
  if (huart->Instance != USART2)
  {
    return;
  }

  if (rx_discard != 0U)
  {
    if ((Uart2_Rx_Cnt != 0U) && (RxBuffer[0] == '\r') && (aRxBuffer == '\n'))
    {
      rx_discard = 0U;
      Uart2_Rx_Cnt = 0U;
    }
    else
    {
      RxBuffer[0] = (char)aRxBuffer;
      Uart2_Rx_Cnt = 1U;
    }
  }
  else
  {
    if (Uart2_Rx_Cnt >= (RXBUFFERSIZE - 1U))
    {
      rx_discard = 1U;
      Uart2_Rx_Cnt = 0U;
      uart_overflow_pending = 1U;
    }
    else
    {
      RxBuffer[Uart2_Rx_Cnt++] = (char)aRxBuffer;
      if ((Uart2_Rx_Cnt >= 2U) && (RxBuffer[Uart2_Rx_Cnt - 2U] == '\r') &&
          (RxBuffer[Uart2_Rx_Cnt - 1U] == '\n'))
      {
        if (command_ready == 0U)
        {
          memcpy(command_buffer, RxBuffer, Uart2_Rx_Cnt - 2U);
          command_ready = Uart2_Rx_Cnt - 2U;
          if (command_ready == 0U) uart_overflow_pending = 1U;
        }
        else uart_overflow_pending = 1U;
        Uart2_Rx_Cnt = 0U;
      }
    }
  }

  HAL_UART_Receive_IT(&huart2, &aRxBuffer, 1);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    uart_tx_busy = 0U;
  }
  else if (huart->Instance == USART6)
  {
    OpenMV_UART_TxCpltCallback();
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  IMU_OnDataReady(GPIO_Pin);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    uart_tx_busy = 0U;
    Uart2_Rx_Cnt = 0U;
    rx_discard = 1U;
    HAL_UART_Receive_IT(&huart2, &aRxBuffer, 1);
  }
  else if (huart->Instance == USART6)
  {
    OpenMV_UART_ErrorCallback();
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
