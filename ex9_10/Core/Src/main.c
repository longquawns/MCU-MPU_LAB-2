/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Columns are the LED anodes, fed from +3.3V through the R1 pack. When ENMc is
 * HIGH the ULN2803 shorts column c to GND and it goes dark, so a column is
 * selected by driving ENMc LOW. Rows are the cathodes: a pixel lights when its
 * ROW is LOW. Both confirmed with a one-pixel test in Proteus. */
#define MATRIX_ENM_ACTIVE   GPIO_PIN_RESET
#define MATRIX_ENM_IDLE     GPIO_PIN_SET

#define MATRIX_DATA_ON      GPIO_PIN_RESET
#define MATRIX_DATA_OFF     GPIO_PIN_SET
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Segment pattern table: segOn[digit][segment], 1 = segment lit */
const uint8_t segOn[10][7] = {
    {1,1,1,1,1,1,0}, // 0
    {0,1,1,0,0,0,0}, // 1
    {1,1,0,1,1,0,1}, // 2
    {1,1,1,1,0,0,1}, // 3
    {0,1,1,0,0,1,1}, // 4
    {1,0,1,1,0,1,1}, // 5
    {1,0,1,1,1,1,1}, // 6
    {1,1,1,0,0,0,0}, // 7
    {1,1,1,1,1,1,1}, // 8
    {1,1,1,1,0,1,1}  // 9
};

/* Signal pins  */
GPIO_TypeDef* segPorts[7] = { SEG0_GPIO_Port, SEG1_GPIO_Port, SEG2_GPIO_Port, SEG3_GPIO_Port,
                              SEG4_GPIO_Port, SEG5_GPIO_Port, SEG6_GPIO_Port };
uint16_t      segPins[7]  = { SEG0_Pin, SEG1_Pin, SEG2_Pin, SEG3_Pin,
                              SEG4_Pin, SEG5_Pin, SEG6_Pin };

/* Enable pins */
GPIO_TypeDef* enPorts[4] = { EN0_GPIO_Port, EN1_GPIO_Port, EN2_GPIO_Port, EN3_GPIO_Port };
uint16_t      enPins[4]  = { EN0_Pin, EN1_Pin, EN2_Pin, EN3_Pin };

int digit_buffer[4] = {1, 2, 3, 0};   /* displays "1 2 3 0" -> 12:30 */
int digit_index     = 0;              /* which seven-segment is currently lit */
int digit_tick      = 0;              /* counts 10ms ticks toward 500ms */

int dot_tick  = 0;                    /* counts 10ms ticks toward 1000ms */
int dot_state = 0;                    /* 0 = off, 1 = on */

void showDigitOnSeg(int num) {
    for (int i = 0; i < 7; i++) {
        HAL_GPIO_WritePin(segPorts[i], segPins[i],
                           segOn[num][i] ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
}

/* ========================= LED matrix (Ex9) ========================= */

/* Column-select pins, driven through the ULN2803 (ENM selects a COLUMN,
 * not a row -- confirmed against the schematic). */
GPIO_TypeDef* enmPorts[8] = { ENM0_GPIO_Port, ENM1_GPIO_Port, ENM2_GPIO_Port, ENM3_GPIO_Port,
                              ENM4_GPIO_Port, ENM5_GPIO_Port, ENM6_GPIO_Port, ENM7_GPIO_Port };
uint16_t      enmPins[8]  = { ENM0_Pin, ENM1_Pin, ENM2_Pin, ENM3_Pin,
                              ENM4_Pin, ENM5_Pin, ENM6_Pin, ENM7_Pin };

/* Row-data pins ROW0..ROW7 = PB8..PB15, they carry the 8 row bits of
 * whichever column is currently selected */
GPIO_TypeDef* rowPorts[8] = { ROW0_GPIO_Port, ROW1_GPIO_Port, ROW2_GPIO_Port, ROW3_GPIO_Port,
                              ROW4_GPIO_Port, ROW5_GPIO_Port, ROW6_GPIO_Port, ROW7_GPIO_Port };
uint16_t      rowPins[8]  = { ROW0_Pin, ROW1_Pin, ROW2_Pin, ROW3_Pin,
                              ROW4_Pin, ROW5_Pin, ROW6_Pin, ROW7_Pin };

/* matrix_buffer[c] = the 8 row bits shown while column c (ENMc) is enabled.
 * Bit 0 -> ROW0 (top row), bit 7 -> ROW7 (bottom row).
 *
 * Character 'A':
 *   . . # # # # . .
 *   . # . . . . # .
 *   . # . . . . # .
 *   . # . . . . # .
 *   . # # # # # # .
 *   . # . . . . # .
 *   . # . . . . # .
 *   . # . . . . # .
 */
uint8_t matrix_buffer[8] = { 0x00, 0xFE, 0x11, 0x11, 0x11, 0x11, 0xFE, 0x00 };

int matrix_index = 0;                 /* which column is currently selected */

/**
  * @brief  Put one column of matrix_buffer on the display.
  *         Only one column is enabled at a time; calling this repeatedly for
  *         index 0..7 fast enough makes the whole 8x8 picture appear.
  * @param  index column to show, 0..7
  */
void updateLEDMatrix(int index) {
    /* 1. switch the previously enabled column off, so the new data cannot
     *    briefly appear on the wrong column (ghosting) */
    HAL_GPIO_WritePin(enmPorts[matrix_index], enmPins[matrix_index], MATRIX_ENM_IDLE);

    /* 2. put the 8 row bits of this column on ROW0..ROW7 */
    for (int bit = 0; bit < 8; bit++) {
        HAL_GPIO_WritePin(rowPorts[bit], rowPins[bit],
                          ((matrix_buffer[index] >> bit) & 0x01) ? MATRIX_DATA_ON
                                                                 : MATRIX_DATA_OFF);
    }

    /* 3. enable this column */
    HAL_GPIO_WritePin(enmPorts[index], enmPins[index], MATRIX_ENM_ACTIVE);

    matrix_index = index;
}

/* ========================= LED matrix animation (Ex10) ========================= */
/* Slide the 'A' one column left every 500ms, wrapping around. The TIM2
 * interrupt only counts ticks and raises matrix_shift_ready; the shift itself
 * runs in main() between full scans, so it can never land halfway through a
 * frame and tear the image. */
#define MATRIX_SHIFT_TICKS  50    /* 50 x 10ms = 500ms per column shift */

int matrix_anim_tick = 0;             /* 10ms ticks since the last shift */
volatile int matrix_shift_ready = 0;  /* set by the ISR, cleared by main() */

/* Shift the picture one column left; column 0 wraps around to column 7. */
void shiftMatrixLeft(void) {
    uint8_t first = matrix_buffer[0];
    for (int c = 0; c < 7; c++) {
        matrix_buffer[c] = matrix_buffer[c + 1];
    }
    matrix_buffer[7] = first;
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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);

  /* start with every seven-segment off, then light the first digit */
  for (int i = 0; i < 4; i++) {
      HAL_GPIO_WritePin(enPorts[i], enPins[i], GPIO_PIN_SET);
  }
  showDigitOnSeg(digit_buffer[0]);
  HAL_GPIO_WritePin(enPorts[0], enPins[0], GPIO_PIN_RESET);

  /* DOT starts off */
  HAL_GPIO_WritePin(DOT_GPIO_Port, DOT_Pin, GPIO_PIN_SET);

  /* start with every matrix column off */
  for (int i = 0; i < 8; i++) {
      HAL_GPIO_WritePin(enmPorts[i], enmPins[i], MATRIX_ENM_IDLE);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      /* The matrix is scanned here rather than in the interrupt, because the
       * interrupt only fires every 10ms: 8 columns x 10ms = 80ms per frame
       * (12.5 Hz) would flicker badly. Here each column is shown for 4ms, so a
       * whole frame takes 32ms (~31 Hz), just past the ~30 Hz threshold where
       * the eye blends the eight columns into one solid 'A'.
       * HAL_Delay does not disturb the seven-segment / DOT, because those are
       * handled by the TIM2 interrupt, which keeps firing during the delay. */
      for (int i = 0; i < 8; i++) {
          updateLEDMatrix(i);
          HAL_Delay(4);
      }

      /* Ex10: shift only between full scans, never mid-frame */
      if (matrix_shift_ready) {
          matrix_shift_ready = 0;
          shiftMatrixLeft();
      }

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DOT_Pin|Led_Red_Pin|EN0_Pin|EN1_Pin
                          |EN2_Pin|EN3_Pin|ENM0_Pin|ENM1_Pin
                          |ENM2_Pin|ENM3_Pin|ENM4_Pin|ENM5_Pin
                          |ENM6_Pin|ENM7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SEG0_Pin|SEG1_Pin|SEG2_Pin|SEG3_Pin
                          |SEG4_Pin|SEG5_Pin|SEG6_Pin|ROW0_Pin
                          |ROW1_Pin|ROW2_Pin|ROW3_Pin|ROW4_Pin
                          |ROW5_Pin|ROW6_Pin|ROW7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DOT_Pin Led_Red_Pin EN0_Pin EN1_Pin
                           EN2_Pin EN3_Pin ENM0_Pin ENM1_Pin
                           ENM2_Pin ENM3_Pin ENM4_Pin ENM5_Pin
                           ENM6_Pin ENM7_Pin */
  GPIO_InitStruct.Pin = DOT_Pin|Led_Red_Pin|EN0_Pin|EN1_Pin
                          |EN2_Pin|EN3_Pin|ENM0_Pin|ENM1_Pin
                          |ENM2_Pin|ENM3_Pin|ENM4_Pin|ENM5_Pin
                          |ENM6_Pin|ENM7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG0_Pin SEG1_Pin SEG2_Pin SEG3_Pin
                           SEG4_Pin SEG5_Pin SEG6_Pin ROW0_Pin
                           ROW1_Pin ROW2_Pin ROW3_Pin ROW4_Pin
                           ROW5_Pin ROW6_Pin ROW7_Pin */
  GPIO_InitStruct.Pin = SEG0_Pin|SEG1_Pin|SEG2_Pin|SEG3_Pin
                          |SEG4_Pin|SEG5_Pin|SEG6_Pin|ROW0_Pin
                          |ROW1_Pin|ROW2_Pin|ROW3_Pin|ROW4_Pin
                          |ROW5_Pin|ROW6_Pin|ROW7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
  * @brief  Period elapsed callback in non blocking mode, invoked every 10ms by TIM2.
  * @param  htim TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* ---- Scan the 4 seven-segment displays, 500ms (50 x 10ms) per digit ---- */
    digit_tick++;
    if (digit_tick >= 50) {
        digit_tick = 0;

        HAL_GPIO_WritePin(enPorts[digit_index], enPins[digit_index], GPIO_PIN_SET); /* turn off current digit */

        digit_index = (digit_index + 1) % 4;

        showDigitOnSeg(digit_buffer[digit_index]);
        HAL_GPIO_WritePin(enPorts[digit_index], enPins[digit_index], GPIO_PIN_RESET); /* turn on next digit */
    }

    /* ---- Blink the DOT LEDs every 1000ms (100 x 10ms) ---- */
    dot_tick++;
    if (dot_tick >= 100) {
        dot_tick = 0;
        dot_state = !dot_state;
        HAL_GPIO_WritePin(DOT_GPIO_Port, DOT_Pin, dot_state ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }

    /* ---- Ex10: request a column shift every 500ms (50 x 10ms) ---- */
    matrix_anim_tick++;
    if (matrix_anim_tick >= MATRIX_SHIFT_TICKS) {
        matrix_anim_tick = 0;
        matrix_shift_ready = 1;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
