/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <stdio.h>
#include "stm32f0xx.h"
#include <lcd_stm32f0.c>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// TODO: Add values for below variables
#define NS  180      // Number of samples in LUT
#define TIM2CLK 8000000  // STM Clock frequency
#define F_SIGNAL 90 // Frequency of output analog signal
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim2_ch1;

/* USER CODE BEGIN PV */
// TODO: Add code for global variables, including LUTs

uint32_t prevdebounce = 0;
uint32_t delay_t = 500;
uint32_t waveform = 1;

uint32_t Sin_LUT[NS] = {512, 529, 547, 565, 583, 601, 618, 636, 653, 670, 687, 704, 721, 737, 753, 769, 784, 799, 814, 828, 842, 855, 868, 881, 893, 905, 916, 927, 937, 947, 956, 965, 973, 980, 987, 993, 999, 1004, 1009, 1013, 1016, 1019, 1021, 1022, 1023, 1023, 1023, 1021, 1020, 1017, 1014, 1011, 1007, 1002, 996, 990, 984, 976, 969, 960, 951, 942, 932, 922, 911, 899, 887, 875, 862, 849, 835, 821, 806, 791, 776, 761, 745, 729, 712, 696, 679, 662, 645, 627, 610, 592, 574, 556, 538, 520, 503, 485, 467, 449, 431, 413, 396, 378, 361, 344, 327, 311, 294, 278, 262, 247, 232, 217, 202, 188, 174, 161, 148, 136, 124, 112, 101, 91, 81, 72, 63, 54, 47, 39, 33, 27, 21, 16, 12, 9, 6, 3, 2, 0, 0, 0, 1, 2, 4, 7, 10, 14, 19, 24, 30, 36, 43, 50, 58, 67, 76, 86, 96, 107, 118, 130, 142, 155, 168, 181, 195, 209, 224, 239, 254, 270, 286, 302, 319, 336, 353, 370, 387, 405, 422, 440, 458, 476, 494, 511};

uint32_t saw_LUT[NS] = {512, 517, 523, 529, 534, 540, 546, 552, 557, 563, 569, 574, 580, 586, 592, 597, 603, 609, 614, 620, 626, 632, 637, 643, 649, 654, 660, 666, 672, 677, 683, 689, 694, 700, 706, 712, 717, 723, 729, 734, 740, 746, 752, 757, 763, 769, 774, 780, 786, 792, 797, 803, 809, 814, 820, 826, 832, 837, 843, 849, 854, 860, 866, 872, 877, 883, 889, 894, 900, 906, 912, 917, 923, 929, 934, 940, 946, 952, 957, 963, 969, 974, 980, 986, 992, 997, 1003, 1009, 1014, 1020, 3, 9, 14, 20, 26, 31, 37, 43, 49, 54, 60, 66, 71, 77, 83, 89, 94, 100, 106, 111, 117, 123, 129, 134, 140, 146, 151, 157, 163, 169, 174, 180, 186, 191, 197, 203, 209, 214, 220, 226, 231, 237, 243, 249, 254, 260, 266, 271, 277, 283, 289, 294, 300, 306, 311, 317, 323, 329, 334, 340, 346, 351, 357, 363, 369, 374, 380, 386, 391, 397, 403, 409, 414, 420, 426, 431, 437, 443, 449, 454, 460, 466, 471, 477, 483, 489, 494, 500, 506, 512};

uint32_t triangle_LUT[NS] = {512, 523, 534, 546, 557, 569, 580, 592, 603, 614, 626, 637, 649, 660, 672, 683, 694, 706, 717, 729, 740, 752, 763, 774, 786, 797, 809, 820, 832, 843, 854, 866, 877, 889, 900, 912, 923, 934, 946, 957, 969, 980, 992, 1003, 1014, 1020, 1009, 997, 986, 974, 963, 952, 940, 929, 917, 906, 894, 883, 872, 860, 849, 837, 826, 814, 803, 792, 780, 769, 757, 746, 734, 723, 712, 700, 689, 677, 666, 654, 643, 632, 620, 609, 597, 586, 574, 563, 552, 540, 529, 517, 506, 494, 483, 471, 460, 449, 437, 426, 414, 403, 391, 380, 369, 357, 346, 334, 323, 311, 300, 289, 277, 266, 254, 243, 231, 220, 209, 197, 186, 174, 163, 151, 140, 129, 117, 106, 94, 83, 71, 60, 49, 37, 26, 14, 3, 9, 20, 31, 43, 54, 66, 77, 89, 100, 111, 123, 134, 146, 157, 169, 180, 191, 203, 214, 226, 237, 249, 260, 271, 283, 294, 306, 317, 329, 340, 351, 363, 374, 386, 397, 409, 420, 431, 443, 454, 466, 477, 489, 500, 512};

// TODO: Equation to calculate TIM2_Ticks
uint32_t TIM2_Ticks = (TIM2CLK)/((F_SIGNAL)*(NS)); // How often to write new LUT value
uint32_t DestAddress = (uint32_t) &(TIM3->CCR3); // Write LUT TO TIM3->CCR3 to modify PWM duty cycle

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
void EXTI0_1_IRQHandler(void);
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
  init_LCD();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  // TODO: Start TIM3 in PWM mode on channel 3
  	 HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  // TODO: Start TIM2 in Output Compare (OC) mode on channel 1.
    HAL_TIM_OC_Start(&htim2,TIM_CHANNEL_1);

  // TODO: Start DMA in IT mode on TIM2->CH1; Source is LUT and Dest is TIM3->CCR3; start with Sine LUT
    HAL_DMA_Start(&hdma_tim2_ch1, Sin_LUT, DestAddress, NS);

  // TODO: Write current waveform to LCD ("Sine")
    lcd_putstring("Sin");
  delay(3000);

  // TODO: Enable DMA (start transfer from LUT to CCR)
  __HAL_TIM_ENABLE_DMA(&htim2,TIM_DMA_CC1);

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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_SetSystemCoreClock(8000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = TIM2_Ticks - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1023;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);

  /**/
  LL_GPIO_SetPinPull(Button0_GPIO_Port, Button0_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinMode(Button0_GPIO_Port, Button0_Pin, LL_GPIO_MODE_INPUT);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_0;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void EXTI0_1_IRQHandler(void)
{
	// TODO: Debounce using HAL_GetTick()
	if(HAL_GetTick()- prevdebounce > delay_t){

	// TODO: Disable DMA transfer and abort IT, then start DMA in IT mode with new LUT and re-enable transfer
	// HINT: Consider using C's "switch" function to handle LUT changes
		__HAL_TIM_DISABLE_DMA(&htim2,TIM_DMA_CC1);
		HAL_DMA_Abort_IT(&hdma_tim2_ch1);

		waveform = waveform +1;

		if (waveform ==4){

			waveform = 1;
		}
		switch (waveform){
		case 1:
			    HAL_DMA_Start(&hdma_tim2_ch1, Sin_LUT, DestAddress, NS);
			    lcd_command(CLEAR);
			    lcd_putstring("Sin");
			  __HAL_TIM_ENABLE_DMA(&htim2,TIM_DMA_CC1);
			  break;

		case 2:
		    HAL_DMA_Start(&hdma_tim2_ch1, saw_LUT, DestAddress, NS);
		    lcd_command(CLEAR);
		    lcd_putstring("Saw ");
		  __HAL_TIM_ENABLE_DMA(&htim2,TIM_DMA_CC1);
		  	  break;

		case 3:
		    HAL_DMA_Start(&hdma_tim2_ch1, triangle_LUT, DestAddress, NS);
		    lcd_command(CLEAR);
		    lcd_putstring("Triangle");
		  __HAL_TIM_ENABLE_DMA(&htim2,TIM_DMA_CC1);
		  break;
		}

		prevdebounce = HAL_GetTick();
	}
	HAL_GPIO_EXTI_IRQHandler(Button0_Pin); // Clear interrupt flags
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
