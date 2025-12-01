/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32L053R8 + LED (PB3) + BUZZER (PB5) + HC-05 (USART2)
  *                   Trimite periodic umiditatea (simulată) către telefon.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* Function prototypes -------------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void Error_Handler(void);

/* MAIN ----------------------------------------------------------------------*/
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  /* Mesaj de start către terminalul Bluetooth */
  char startMsg[] = "=== STM32 Termometru / Umiditate - TEST BT ===\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)startMsg, sizeof(startMsg)-1, HAL_MAX_DELAY);

  float humidity = 40.0f;   // VALOARE SIMULATA LA INCEPUT
  int   direction = 1;      // creste / scade doar ca sa vedem schimbari in terminal

  char msg[64];

  while (1)
  {
    /* 1. Simulam evolutia umiditatii intre 40% si 70% */
    humidity += direction * 1.0f;
    if (humidity >= 70.0f) direction = -1;
    if (humidity <= 40.0f) direction =  1;

    /* 2. Formam mesajul text de trimis prin Bluetooth */
    int len = snprintf(msg, sizeof(msg), "Umiditate = %.1f %%\r\n", humidity);

    /* 3. Trimitem mesajul prin UART2 -> HC-05 -> telefon */
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, HAL_MAX_DELAY);

    /* 4. Clipim LED-ul (PB3) si BUZZER-ul (PB5) scurt, ca indicatie vizuala */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);  // LED ON
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);  // BUZZER ON
    HAL_Delay(150);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET); // LED OFF
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // BUZZER OFF

    /* Pauza 850 ms => total ~1s intre mesaje */
    HAL_Delay(850);
  }
}

/* CLOCK CONFIG --------------------------------------------------------------*/
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
      in the RCC_OscInitTypeDef structure. */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* GPIO INIT -----------------------------------------------------------------*/
/*  PB3 -> LED extern (D3)
 *  PB5 -> BUZZER extern (D4)
 *  restul pinii raman default
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /* PB3 si PB5 ca output */
  GPIO_InitStruct.Pin   = GPIO_PIN_3 | GPIO_PIN_5;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* initial OFF */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
}

/* UART2 INIT (HC-05 | PA2 = TX, PA3 = RX) -----------------------------------*/
static void MX_USART2_UART_Init(void)
{
  huart2.Instance          = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength   = UART_WORDLENGTH_8B;
  huart2.Init.StopBits     = UART_STOPBITS_1;
  huart2.Init.Parity       = UART_PARITY_NONE;
  huart2.Init.Mode         = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* ERROR HANDLER -------------------------------------------------------------*/
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    /* optional: poti aprinde un LED aici ca sa vezi ca ai intrat in eroare */
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* pentru debug avansat, deocamdata nu folosim */
}
#endif /* USE_FULL_ASSERT */
