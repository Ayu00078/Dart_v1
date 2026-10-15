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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RXBUFFERSIZE 256
#define SERVO_0_DEG_PULSE 1000U
#define SERVO_180_DEG_PULSE 2000U
#define BUZZER_LOW_PULSE 10U
#define BUZZER_MEDIUM_PULSE 20U
#define BUZZER_HIGH_PULSE 40U
#define POWER_OFF_HOLD_MS 1000U
#define POWER_OFF_RELEASE_MS 1000U
#define POWER_RELEASE_BEEP_MS 100U
#define EFFECT_BEEP_LENGTH_MS 180U
#define EFFECT_BEEP_INTERVAL_MS 300U
#define EFFECT_LENGTH_MS ((2U * EFFECT_BEEP_INTERVAL_MS) + EFFECT_BEEP_LENGTH_MS)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static char RxBuffer[RXBUFFERSIZE];
static uint8_t aRxBuffer;
static uint8_t Uart2_Rx_Cnt;
static volatile uint8_t flag_uart2;
static volatile uint8_t servo_command;
static volatile uint8_t flag_uart2_buzzer;
static uint8_t buzzer_on;
static PowerButtonState power_button_state;
static uint32_t power_button_tick;
static uint32_t power_beep_tick;
static uint32_t startup_effect_tick;
static uint8_t startup_effect_active;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void SetServosPulse(uint32_t pulse);
static uint32_t GetThreeBeepPulse(uint32_t elapsed, uint8_t increasing);
static void UpdateBuzzer(uint32_t now);
static void HandlePowerButton(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void SetServosPulse(uint32_t pulse)
{
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse);
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

  if (power_button_state == POWER_WAIT_RELEASE)
  {
    pulse = ((uint32_t)(now - power_beep_tick) < POWER_RELEASE_BEEP_MS)
              ? BUZZER_HIGH_PULSE : 0U;
  }
  else if (power_button_state == POWER_TIMING_RELEASE)
  {
    pulse = GetThreeBeepPulse((uint32_t)(now - power_button_tick), 0U);
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
      pulse = buzzer_on ? BUZZER_HIGH_PULSE : 0U;
    }
  }
  else
  {
    pulse = buzzer_on ? BUZZER_HIGH_PULSE : 0U;
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
  /* USER CODE BEGIN 2 */
  if ((HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK) ||
      (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK) ||
      (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK) ||
      (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK))
  {
    Error_Handler();
  }
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0U);
  if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  startup_effect_tick = HAL_GetTick();
  startup_effect_active = 1U;
  if (HAL_UART_Receive_IT(&huart2, &aRxBuffer, 1) != HAL_OK)
  {
    Error_Handler();
  }
  static const uint8_t reply[] = "ok\r\n";
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HandlePowerButton();
    UpdateBuzzer(HAL_GetTick());
    if (flag_uart2 != 0U)
    {
      flag_uart2 = 0U;
      HAL_UART_Transmit(&huart2, (uint8_t *)reply, sizeof(reply) - 1U, HAL_MAX_DELAY);
    }
    if (servo_command != 0U)
    {
      uint8_t command;
      __disable_irq();
      command = servo_command;
      servo_command = 0U;
      __enable_irq();

      if (command == 2U)
      {
        SetServosPulse(SERVO_0_DEG_PULSE);
        HAL_UART_Transmit(&huart2, (uint8_t *)reply, sizeof(reply) - 1U, HAL_MAX_DELAY);

      }
      else if (command == 3U)
      {
        SetServosPulse(SERVO_180_DEG_PULSE);
        HAL_UART_Transmit(&huart2, (uint8_t *)reply, sizeof(reply) - 1U, HAL_MAX_DELAY);
      }
    }
    if (flag_uart2_buzzer != 0U)
    {
      flag_uart2_buzzer = 0U;
      buzzer_on = !buzzer_on;
      HAL_UART_Transmit(&huart2, (uint8_t *)reply, sizeof(reply) - 1U, HAL_MAX_DELAY);
    }
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
  if (huart->Instance != USART2)
  {
    return;
  }

  if (Uart2_Rx_Cnt >= 255U)
  {
    static const uint8_t overflow_message[] = "数据溢出";
    Uart2_Rx_Cnt = 0U;
    memset(RxBuffer, 0, sizeof(RxBuffer));
    HAL_UART_Transmit(&huart2, (uint8_t *)overflow_message,
                      sizeof(overflow_message) - 1U, HAL_MAX_DELAY);
  }
  else
  {
    RxBuffer[Uart2_Rx_Cnt++] = aRxBuffer;

    if ((Uart2_Rx_Cnt >= 2U) &&
        (RxBuffer[Uart2_Rx_Cnt - 1U] == '\n') &&
        (RxBuffer[Uart2_Rx_Cnt - 2U] == '\r'))
    {
      if ((Uart2_Rx_Cnt >= 3U) && (RxBuffer[Uart2_Rx_Cnt - 3U] == '1'))
      {
        flag_uart2 = 1U;
      }
      if ((Uart2_Rx_Cnt >= 3U) && (RxBuffer[Uart2_Rx_Cnt - 3U] == '2'))
      {
        servo_command = 2U;
      }
      if ((Uart2_Rx_Cnt >= 3U) && (RxBuffer[Uart2_Rx_Cnt - 3U] == '3'))
      {
        servo_command = 3U;
      }
      if ((Uart2_Rx_Cnt >= 3U) && (RxBuffer[Uart2_Rx_Cnt - 3U] == '4'))
      {
        flag_uart2_buzzer = 1U;
      }
      Uart2_Rx_Cnt = 0U;
      memset(RxBuffer, 0, sizeof(RxBuffer));
    }
    
  }

  HAL_UART_Receive_IT(&huart2, &aRxBuffer, 1);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    Uart2_Rx_Cnt = 0U;
    memset(RxBuffer, 0, sizeof(RxBuffer));
    HAL_UART_Receive_IT(&huart2, &aRxBuffer, 1);
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
