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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "rtos_tasks.h"
#include "stepper.h"
#include "liquidcrystal_i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Global stepper motor control structure.
volatile StepperControl_t gStepperControl = {.MotorMode = MOTOR_MODE_NONE,
        .MotorState = MOTOR_STATE_IDLE, };
TaskHandle_t TaskInputHandle = NULL;
TaskHandle_t TaskUartGatekeeperHandle = NULL;
extern volatile SystemState_e gSysState;
extern QueueHandle_t InputQueue;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* Enforce -O2 optimization even in Debug builds (-O0) to prevent ISR cycle bloat
 * and CPU starvation of lower-priority FreeRTOS tasks (TaskEncoder, TaskInput). */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) __attribute__((optimize("O2")));
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    stepper_init((StepperControl_t*) &gStepperControl);
    HD44780_Init(2);
    RtosTasks_Init();

	/*
	 * Create the FreeRTOS tasks:
	 *   InputTask   (prio 3): 50 Hz GPIO/encoder polling → produces input events
	 *   UITask      (prio 2): Event-driven LCD menu state machine
	 *   MotorTask   (prio 4): Kinematics computation and pump monitoring
	 *   EncoderTask (prio 4): I2C AS5600 encoder reading and processing
	 *   UartTask    (prio 4): UART Gatekeeper (TMC2208 & VCP DMA)
	 */
	xTaskCreate(RtosTasks_TaskInput, "InputTask", 128, NULL, 3, &TaskInputHandle);
	xTaskCreate(RtosTasks_TaskUI, "UITask", 512, NULL, 2, NULL);
	xTaskCreate(RtosTasks_TaskMotor, "MotorTask", 512, NULL, 4, NULL);
	xTaskCreate(RtosTasks_TaskEncoder, "EncoderTask", 512, NULL, 4, NULL);
	xTaskCreate(RtosTasks_TaskUartGatekeeper, "UartTask", 256, NULL, 4, &TaskUartGatekeeperHandle);

	vTaskStartScheduler();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_TIM2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == buttonSelect_Pin)
  {
    if (HAL_GPIO_ReadPin(buttonSelect_GPIO_Port, buttonSelect_Pin) == GPIO_PIN_RESET)
    {
      /* While TaskInput is suspended during pumping, EXTI provides zero-latency pause detection */
      if (gSysState == SYS_PUMPING || gSysState == SYS_PUMPING_CALIBRATION)
      {
        /* Immediately mask EXTI line so contact bounce cannot queue multiple pause events */
        EXTI->IMR &= ~buttonSelect_Pin;

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        if (InputQueue != NULL) {
          InputMessage_t msg;
          msg.event = EVENT_BTN_SELECT_PRESS;
          msg.timestamp = xTaskGetTickCountFromISR();
          xQueueSendFromISR(InputQueue, &msg, &xHigherPriorityTaskWoken);
        }

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
      }
    }
  }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
    /*
     * ===================================================================
     * TIM2 Stepper Motor State Machine (ISR context)
     *
     * Called once per TIM2 period (one step pulse). Executes the
     * trapezoidal/triangular motion profile via inlined StepperProfile_StepTick.
     * Real-time calculation runs in O(1) time with zero call overhead.
     * ===================================================================
     */

    /* Only process TIM2 interrupts (ignore TIM16 systick, etc.) */
    if (htim->Instance != TIM2) {
        return;
    }

    if (gStepperControl.MotorState == MOTOR_STATE_IDLE) {
        return;
    }

    StepperStepAction_e action = StepperProfile_StepTick(
        &gStepperControl,
        s_curve_norm,
        TIMER_CLOCK,
        PRESCALER
    );

    if (action == STEP_ACTION_STOP_IDLE || action == STEP_ACTION_STOP_PAUSED) {
        HAL_TIM_Base_Stop_IT(&htim2);
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    }
    else {
        __HAL_TIM_SET_AUTORELOAD(htim, gStepperControl.ARRCurrent);
        /* ARR/2 compare value maintains a symmetric 50% PWM step pulse duty cycle at all speeds */
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1,
                gStepperControl.ARRCurrent / 2);
    }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
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
