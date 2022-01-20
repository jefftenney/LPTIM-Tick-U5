/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include <stdio.h>
#include <string.h>
#include "testTickTiming.h"
#include "ulp.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define B1_Pin    USER_Button_Pin
#define B1_Port   USER_Button_GPIO_Port

#define LED_On    GPIO_PIN_RESET
#define LED_Off   GPIO_PIN_SET

#define NOTIFICATION_FLAG_B1_PIN    (1UL << 0)
#define NOTIFICATION_FLAG_LED_BLIP  (1UL << 1)
#define NOTIFICATION_FLAG_RESULTS   (1UL << 2)

#define STACK_DEPTH_MAIN_OS_TASK    256
#define TASK_PRIORITY_MAIN_OS_TASK  1

#define STACK_DEPTH_TT_OS_TASK      256
#define TASK_PRIORITY_TT_OS_TASK    2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

LPTIM_HandleTypeDef hlptim3;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

TaskHandle_t  mainTaskHandle;
TimerHandle_t ledTimerHandle;
TimerHandle_t resultsTimerHandle;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_LPTIM3_Init(void);
/* USER CODE BEGIN PFP */

static void mainOsTask(void* argument);
static void ledTimerCallback(TimerHandle_t argument);
static void resultsTimerCallback(TimerHandle_t argument);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define DEMO_STATE_TEST_1  0
#define DEMO_STATE_TEST_2  1
#define DEMO_STATE_TEST_3  2
#define MAX_DEMO_STATE     DEMO_STATE_TEST_3
static void vSetDemoState( BaseType_t state )
{
   //      Assume for now that we're activating a demo state that doesn't need LPTIM3, our stress-test actor.
   //
   if (LPTIM3->CR & LPTIM_CR_ENABLE)
   {
      HAL_LPTIM_Counter_Stop_IT(&hlptim3);
   }

   uint32_t ledIntervalMs = 0;
   if (state == DEMO_STATE_TEST_1)
   {
      //      Demonstrate minimum energy consumption.  With configUSE_TICKLESS_IDLE == 2 (lptimTick.c), we
      // draw only 9uA, with full RAM retention, with RTC, and with FreeRTOS timers/timeouts/delays active.
      //
      ledIntervalMs = 5000UL;
      vTttSetEvalInterval( portMAX_DELAY );
      xTimerStop(resultsTimerHandle, 0);
   }
   else if (state == DEMO_STATE_TEST_2)
   {
      //      Instruct the tick-timing test to sample the tick timing every 10 milliseconds.  Sampling faster
      // would require that we open the pass/fail criteria to accommodate a couple hundred microseconds of
      // jitter or tick "jump" since that criteria is expressed as a percentage of the sampling interval.
      // Sampling much slower might allow large jitter or "jump" that should be caught as an error, again
      // because jitter is measured as a percentage of the sampling interval.
      //
      #define TICK_TEST_SAMPLING_INTERVAL_MS 10

      //      Demonstrate energy consumption waking every 10ms, and test the tick timing.
      //
      ledIntervalMs = 2000UL;
      vTttSetEvalInterval( pdMS_TO_TICKS(TICK_TEST_SAMPLING_INTERVAL_MS) );
      xTimerStart(resultsTimerHandle, 0);
   }
   else if (state == DEMO_STATE_TEST_3)
   {
      //      In state 2, add an actor to stress the tick timing.  Use LPTIM3 interrupts since that timer
      // keeps operating in STOP 2 mode, which allows us to keep demonstrating low-power operation.
      //
      //      A good long soak test is strongly recommended here.
      //
      HAL_LPTIM_Counter_Start_IT(&hlptim3);

      ledIntervalMs = 1000UL;
      vTttSetEvalInterval( pdMS_TO_TICKS(TICK_TEST_SAMPLING_INTERVAL_MS) );
      xTimerStart(resultsTimerHandle, 0);
   }
   else
   {
      configASSERT(0);
   }

   if (ledIntervalMs != 0)
   {
      xTimerChangePeriod(ledTimerHandle, ledIntervalMs, 0);
      xTaskNotify(mainTaskHandle, NOTIFICATION_FLAG_LED_BLIP, eSetBits);
   }

   char banner[100];
   int len = sprintf(banner, "\r\n\r\nRunning Test %d.", (int)state + 1);
   if (state != DEMO_STATE_TEST_1)
   {
      len += sprintf(&banner[len], "  Jumps are shown as %% of %d ms.\r\n",
                     TICK_TEST_SAMPLING_INTERVAL_MS);
   }
   HAL_UART_Transmit(&huart1, (uint8_t*)banner, len, HAL_MAX_DELAY);
}

static int lDescribeTickTestResults(TttResults_t* results, int periodNumber, char* dest)
{
   int durationSeconds = (results->durationSs + results->subsecondsPerSecond/2) / results->subsecondsPerSecond;
   return (sprintf(dest, "Period %d: %d s, drift: %d/%d s, jump: %+d%% (min), %+d%% (max)",
                   periodNumber,
                   durationSeconds,
                   (int)results->driftSs, (int) results->subsecondsPerSecond,
                   results->minDriftRatePct,
                   results->maxDriftRatePct));
}

static BaseType_t xUpdateResults( int xDemoState )
{
   static int resultsCount = 0;

   TttResults_t complete, inProgress;
   vTttGetResults(&complete, &inProgress);

   //      Build the results text here.  Use a "large" buffer because the results might require two lines of
   // terminal output.  The buffer is static so its contents remain valid even after we return from this
   // function.  In turn this allows us to return to our caller without waiting for the UART to finish.
   //
   static char textResults[200];
   static const char* const eraseLine = "\r\x1B[K";
   int resultsLen = sprintf(textResults, "%s", eraseLine);

   if (resultsCount != complete.resultsCounter)
   {
      resultsCount = complete.resultsCounter;
      if (resultsCount != 0)
      {
         resultsLen += lDescribeTickTestResults(&complete, resultsCount, &textResults[resultsLen]);
         resultsLen += sprintf(&textResults[resultsLen], "\r\n");
      }
   }

   resultsLen += lDescribeTickTestResults(&inProgress, resultsCount + 1, &textResults[resultsLen]);

   //      Send the results to the terminal.  In test 3, use interrupt-driven I/O with the terminal as
   // a way to enhance the stress test.  The interrupts induce a rapid sequence of early wake-ups from
   // tickless idle (if enabled).  This barrage of early wake-ups is a great stress test for the tickless
   // logic.  In the other demo states, use busy-wait I/O to avoid adding a test actor when we don't want one.
   //
   if (xDemoState == DEMO_STATE_TEST_3)
   {
      vUlpOnPeripheralsActive(ulpPERIPHERAL_USART1);
      HAL_UART_Transmit_IT(&huart1, (uint8_t*)textResults, resultsLen);
   }
   else
   {
      //      Because busy-wait I/O keeps this task "ready", it also prevents tickless idle, so we don't need
      // to bother notifying the ULP driver that the UART peripheral is active.
      //
      // vUlpOnPeripheralsActive(ulpPERIPHERAL_USART1);
      HAL_UART_Transmit(&huart1, (uint8_t*)textResults, resultsLen, HAL_MAX_DELAY);
      // vUlpOnPeripheralsInactive(ulpPERIPHERAL_USART1);
   }

   //      Apply some pass/fail criteria and report any failures.
   //
   #if ( configLPTIM_DIVIDER == 8 )
      #define TEST_LIMIT_PCT_JITTER_MIN -3
      #define TEST_LIMIT_PCT_JITTER_MAX +5
   #else
      #define TEST_LIMIT_PCT_JITTER_MIN -2
      #define TEST_LIMIT_PCT_JITTER_MAX +2
   #endif
   BaseType_t isFailure = pdFALSE;
   if (inProgress.minDriftRatePct < TEST_LIMIT_PCT_JITTER_MIN ||
       inProgress.maxDriftRatePct > TEST_LIMIT_PCT_JITTER_MAX)
   {
      isFailure = pdTRUE;
   }

   return (isFailure);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
   vUlpOnPeripheralsInactiveFromISR(ulpPERIPHERAL_USART1);
}

void vBlipLed( uint32_t ms )
{
   HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, LED_On);
   vTaskDelay(pdMS_TO_TICKS(ms));
   HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, LED_Off);
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

  //      Correct a bug in CubeMX.  We selected MSIS auto-calibration (not MSIK), but CubeMX fails to set
  // MSIPLLSEL as required.  So we do it here ourselves -- and *before* PLL mode becomes enabled.
  //
  RCC->CR |= RCC_CR_MSIPLLSEL;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  #if (configSYSTICK_CLOCK_HZ == LSE_VALUE)
  {
    //      Select the 32kHz LSE clock as the "other" SysTick clock.  FreeRTOS selects this "other" clock for
    // SysTick when we define configSYSTICK_CLOCK_HZ.  The default section for the "other" clock after reset
    // is the CPU clock divided by 8.
    //
    MODIFY_REG(RCC->CCIPR1, RCC_CCIPR1_SYSTICKSEL_Msk, RCC_CCIPR1_SYSTICKSEL_1);
  }
  #endif

  vUlpInit();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  MX_LPTIM3_Init();
  /* USER CODE BEGIN 2 */

  //      Start with the green and red LEDs off.
  //
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, LED_Off);
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, LED_Off);

  //      Carefully select the interval of the nuisance interrupts to be slightly longer than the tick-
  // test sampling interval.  The sampling interval drives the "expected idle time" in the tickless logic,
  // so a nuisance interval slightly longer ensures lots of different interrupt timing, including some
  // tickless periods lasting the full expected idle time.  For example, with a 10ms sampling interval,
  // there are 327.68 LSE cycles between samplings, on average.  The code below would set up the nuisance
  // interrupt to be every 328 LSE cycles for that case.  (The value passed to the HAL is 327, but the
  // period ends up being 328 due to LPTIM reload behavior.)
  //
  hlptim3.Init.Period = (TICK_TEST_SAMPLING_INTERVAL_MS * LSE_VALUE) / 1000U;
  HAL_LPTIM_Init(&hlptim3);

  #if (configUSE_TICKLESS_IDLE != 2)
  {
    //      The tick-timing tests make more accurate evaluations of the tick timing when FreeRTOS has the best
    // possible value in configCPU_CLOCK_HZ.  Since we define that symbol as SystemCoreClock, we update
    // SystemCoreClock here.  This code helped us "prove" that the MSI PLL behaves like a real PLL (albeit a
    // very jittery one).  There's "no" tick drift as measured by the RTC when tickless idle is disabled and
    // the MSI is in PLL mode driving the core clock.
    //
    //      We don't care if the HAL reverts this change at some point later during the application execution.
    // We only want this updated value to endure long enough for FreeRTOS startup code to use it to calculate
    // the tick timing.
    //
    if (RCC->CR & RCC_CR_MSIPLLEN)  // App Specific.  Assumes MSIPLLEN being set means MSI is the core clock.
    {
      int pllModeMultiplier = ( SystemCoreClock + (LSE_VALUE/4) ) / (LSE_VALUE/2);
      SystemCoreClock = (LSE_VALUE/2) * pllModeMultiplier;
    }
  }
  #endif

  resultsTimerHandle = xTimerCreate("Results", pdMS_TO_TICKS(1000UL), pdTRUE, NULL, resultsTimerCallback);
  ledTimerHandle = xTimerCreate("LED", 1, pdTRUE, NULL, ledTimerCallback);

  xTaskCreate(mainOsTask, "App", STACK_DEPTH_MAIN_OS_TASK, NULL, TASK_PRIORITY_MAIN_OS_TASK, &mainTaskHandle);
  xTaskCreate(vTttOsTask, "TickTest", STACK_DEPTH_TT_OS_TASK, &hrtc, TASK_PRIORITY_TT_OS_TASK, NULL);

  vTaskStartScheduler();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMLOW);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
  __HAL_RCC_PWR_CLK_DISABLE();
}

/**
  * @brief LPTIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPTIM3_Init(void)
{

  /* USER CODE BEGIN LPTIM3_Init 0 */

  /* USER CODE END LPTIM3_Init 0 */

  /* USER CODE BEGIN LPTIM3_Init 1 */

  /* USER CODE END LPTIM3_Init 1 */
  hlptim3.Instance = LPTIM3;
  hlptim3.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim3.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;
  hlptim3.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim3.Init.Period = 65535;
  hlptim3.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  hlptim3.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
  hlptim3.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
  hlptim3.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;
  hlptim3.Init.RepetitionCounter = 0;
  if (HAL_LPTIM_Init(&hlptim3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPTIM3_Init 2 */

  /* USER CODE END LPTIM3_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_PrivilegeStateTypeDef privilegeState = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 3;
  hrtc.Init.SynchPrediv = 8191;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  hrtc.Init.BinMode = RTC_BINARY_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  privilegeState.rtcPrivilegeFull = RTC_PRIVILEGE_FULL_NO;
  privilegeState.backupRegisterPrivZone = RTC_PRIVILEGE_BKUP_ZONE_NONE;
  privilegeState.backupRegisterStartZone2 = RTC_BKP_DR0;
  privilegeState.backupRegisterStartZone3 = RTC_BKP_DR0;
  if (HAL_RTCEx_PrivilegeModeSet(&hrtc, &privilegeState) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, LED_RED_Pin|LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PH3_BOOT0_Pin */
  GPIO_InitStruct.Pin = PH3_BOOT0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PH3_BOOT0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_Button_Pin */
  GPIO_InitStruct.Pin = USER_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_RED_Pin LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin|LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI13_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI13_IRQn);

}

/* USER CODE BEGIN 4 */

static void mainOsTask(void* argument)
{
   //      Stop the timer that provides the HAL tick.  Now it won't needlessly interrupt tickless idle periods
   // that use sleep mode.  (The HAL tick already can't interrupt tickless idle periods that use stop mode,
   // because the HAL timer doesn't operate in stop mode.)  In a real application, the HAL tick might be
   // required by the HAL even after FreeRTOS has control.  It's still best to stop the timer here, and then
   // define HAL_GetTick() and HAL_Delay() to use the FreeRTOS tick (and delay) once available.
   //
   TIM17->CR1 &= ~TIM_CR1_CEN;  // wish CubeMX would generate a symbol for the HAL tick timer

   //      Be sure LPTIM3 ignores its input clock when the debugger stops program execution, and be sure its
   // kernel clock keeps running in STOP2 mode.
   //
   taskDISABLE_INTERRUPTS();
   DBGMCU->APB3FZR |= DBGMCU_APB3FZR_DBG_LPTIM3_STOP;
   RCC->SRDAMR |= RCC_SRDAMR_LPTIM3AMEN;
   taskENABLE_INTERRUPTS();

   //      Start the demo in state 0.
   //
   BaseType_t xDemoState = DEMO_STATE_TEST_1;
   vSetDemoState(xDemoState);

   int isFailureDetected = pdFALSE;
   uint32_t notificationFlags;
   for(;;)
   {
      //      Wait forever for any notification.
      //
      xTaskNotifyWait(0, UINT32_MAX, &notificationFlags, portMAX_DELAY);

      if (notificationFlags & NOTIFICATION_FLAG_B1_PIN)
      {
         //      Advance the demo state in response to the button press.  Clear any previous test failures.
         //
         if (++xDemoState > MAX_DEMO_STATE )
         {
            xDemoState = DEMO_STATE_TEST_1;
         }

         vSetDemoState(xDemoState);

         isFailureDetected = pdFALSE;
      }

      if (notificationFlags & NOTIFICATION_FLAG_RESULTS)
      {
         //      Make failure detections "sticky" so an observer can rely on the LED even for past failures.
         // We clear past failures when we advance the demo state above (for a button press).
         //
         if (xUpdateResults( xDemoState ))
         {
            isFailureDetected = pdTRUE;
         }
      }

      if (notificationFlags & NOTIFICATION_FLAG_LED_BLIP)
      {
         //      Blip the LED for 100ms.  Blip twice if the test has failed.
         //
         vBlipLed(100UL);
         if (isFailureDetected)
         {
            vTaskDelay(pdMS_TO_TICKS(100UL));
            vBlipLed(100UL);
         }
      }
  }
}

static void ledTimerCallback(TimerHandle_t argument)
{
   UNUSED(argument);

   if ( mainTaskHandle != NULL )
   {
      xTaskNotify( mainTaskHandle, NOTIFICATION_FLAG_LED_BLIP, eSetBits );
   }
}

static void resultsTimerCallback(TimerHandle_t argument)
{
   UNUSED(argument);

   if ( mainTaskHandle != NULL )
   {
      xTaskNotify( mainTaskHandle, NOTIFICATION_FLAG_RESULTS, eSetBits );
   }
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
   if (GPIO_Pin == B1_Pin)
   {
      //      Don't attempt to notify a task that isn't yet created.  Drop the button press instead.
      //
      if ( mainTaskHandle != NULL )
      {
         BaseType_t xWasHigherPriorityTaskWoken = pdFALSE;
         xTaskNotifyFromISR( mainTaskHandle, NOTIFICATION_FLAG_B1_PIN, eSetBits, &xWasHigherPriorityTaskWoken);
         portYIELD_FROM_ISR(xWasHigherPriorityTaskWoken);
      }
   }
}

void HAL_LPTIM_AutoReloadMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
   //      The HAL calls this function when LPTIM3 has an ARR match event.  We use LPTIM3 as a stress-test
   // actor during test state 2.  This function has nothing to do with lptimTick.c or LPTIM1, but it does
   // help us *test* the tick timing provided by that code and that timer.

   //      Wake the main task, but for no reason.  We're just trying to stress test the tick timing.
   //
   BaseType_t xWasHigherPriorityTaskWoken = pdFALSE;
   xTaskNotifyFromISR( mainTaskHandle, 0, eNoAction, &xWasHigherPriorityTaskWoken);
   portYIELD_FROM_ISR(xWasHigherPriorityTaskWoken);
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
   Error_Handler();
}

void vApplicationMallocFailedHook( void )
{
   Error_Handler();
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

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
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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

