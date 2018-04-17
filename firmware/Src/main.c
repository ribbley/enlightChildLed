/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#define USE_FULL_LL_DRIVER
#include "stm32f0xx_ll_dma.h"
#include <string.h>

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim3_ch3;
DMA_HandleTypeDef hdma_tim3_ch4_up;
DMA_HandleTypeDef hdma_tim3_ch1_trig;

/* USER CODE BEGIN PV */
//DMA handler
LL_DMA_InitTypeDef dma_set_pins;
LL_DMA_InitTypeDef dma_reset_pins;
LL_DMA_InitTypeDef dma_set_memory;

/* Private variables ---------------------------------------------------------*/
#define LED_COUNT 25

uint8_t grb_array_size = 1;
uint8_t max_power = 0x08;
uint8_t gpio_port_high = 0xFF;
uint8_t gpio_port_low = 0x00;

struct PIXEL {
  uint8_t green[8];
  uint8_t red[8];
  uint8_t blue[8];
};

struct PIXEL grb_array[LED_COUNT]; //600 Byte RAM

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void checkButtons();
void updateColors();
void setPixel(uint8_t x, uint8_t y, uint8_t green, uint8_t red, uint8_t blue);
void setColumn(uint8_t column, uint8_t green, uint8_t red, uint8_t blue);
void setRow(uint8_t row, uint8_t green, uint8_t red, uint8_t blue);
void setAll(uint8_t green, uint8_t red, uint8_t blue);

void transmit_zero();
void transmit_one();
void transmit_reset();

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_ADC_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  //@init
  memset(grb_array, 0xFF, sizeof(struct PIXEL) * LED_COUNT);
  setPixel(2, 2, 0, 0, 10);

  HAL_DMA_Start_IT(&hdma_tim3_ch1_trig, (uint32_t) &gpio_port_high, (uint32_t) &GPIOB->ODR, LED_COUNT * sizeof(struct PIXEL));
  HAL_DMA_Start_IT(&hdma_tim3_ch3, (uint32_t) grb_array, (uint32_t) &GPIOB->ODR, LED_COUNT * sizeof(struct PIXEL));
  HAL_DMA_Start_IT(&hdma_tim3_ch4_up, (uint32_t) &gpio_port_low, (uint32_t) &GPIOB->ODR, LED_COUNT * sizeof(struct PIXEL));

  __HAL_TIM_ENABLE_DMA(&htim3, TIM_DMA_ID_CC3);
  __HAL_TIM_ENABLE_DMA(&htim3, TIM_DMA_ID_UPDATE);
  __HAL_TIM_ENABLE_DMA(&htim3, TIM_DMA_ID_CC1);

  HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_3);
  //HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_4);

  __HAL_TIM_ENABLE(&htim3);
  

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t times = 0;
  uint32_t last_update = 0;

  //setRow(2, 0, 0, max_power);

  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    //@while

    if(HAL_GetTick() > last_update + 1){
      last_update = HAL_GetTick();
      times++;

      if (times%20 == 0){
        HAL_GPIO_TogglePin(SER_OUT_INVERTED_GPIO_Port, SER_OUT_INVERTED_Pin);
      }
    }
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted.
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted.
    */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted.
    */
  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted.
    */
  sConfig.Channel = ADC_CHANNEL_3;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 60;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  //htim3.Init.RepetitionCounter = LED_COUNT * sizeof(struct PIXEL);
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 17;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.Pulse = 35;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.Pulse = 0;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SER_OUT_INVERTED_GPIO_Port, SER_OUT_INVERTED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BUT3_IN_Pin BUT2_IN_Pin BUT1_IN_Pin */
  GPIO_InitStruct.Pin = BUT3_IN_Pin|BUT2_IN_Pin|BUT1_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SER_OUT_INVERTED_Pin */
  GPIO_InitStruct.Pin = SER_OUT_INVERTED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SER_OUT_INVERTED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* set values need to be inverted, because pin is connected to NPN base. */
void setPixel(uint8_t x, uint8_t y, uint8_t green, uint8_t red, uint8_t blue){
  for (uint8_t i = 0; i < 8; i++){
    grb_array[x + (5 * y)].green[i] = (green & (0x80 >> i))?gpio_port_low:gpio_port_high;
  }
  for (uint8_t i = 0; i < 8; i++){
    grb_array[x + (5 * y)].red[i] = (red & (0x80 >> i))?gpio_port_low:gpio_port_high;
  }
  for (uint8_t i = 0; i < 8; i++){
    grb_array[x + (5 * y)].blue[i] = (blue & (0x80 >> i))?gpio_port_low:gpio_port_high;
  }
}

void setColumn(uint8_t column, uint8_t green, uint8_t red, uint8_t blue){
  setPixel(column, 0, green, red, blue);
  setPixel(column, 1, green, red, blue);
  setPixel(column, 2, green, red, blue);
  setPixel(column, 3, green, red, blue);
  setPixel(column, 4, green, red, blue);
}

void setRow(uint8_t row, uint8_t green, uint8_t red, uint8_t blue){
  setPixel(0, row, green, red, blue);
  setPixel(1, row, green, red, blue);
  setPixel(2, row, green, red, blue);
  setPixel(3, row, green, red, blue);
  setPixel(4, row, green, red, blue);
}

void setAll(uint8_t green, uint8_t red, uint8_t blue){
  setRow(0, green, red, blue);
  setRow(1, green, red, blue);
  setRow(2, green, red, blue);
  setRow(3, green, red, blue);
  setRow(4, green, red, blue);
}

void updateColors(){
  //TODO how to trigger color updatE?
}

/*
 * SER_OUT_INVERTED_GPIO_Port->BSRR = (uint32_t)SER_OUT_INVERTED_Pin;
 * SER_OUT_INVERTED_GPIO_Port->BRR = (uint32_t)SER_OUT_INVERTED_Pin;
 */

#define __SER_OUT_SET SER_OUT_INVERTED_GPIO_Port->BSRR = (uint32_t)SER_OUT_INVERTED_Pin
#define __SER_OUT_RESET SER_OUT_INVERTED_GPIO_Port->BRR = (uint32_t)SER_OUT_INVERTED_Pin

void checkButtons(){
	uint8_t buttons = 0;
	if (HAL_GPIO_ReadPin(BUT1_IN_GPIO_Port, BUT1_IN_Pin == GPIO_PIN_RESET)){
		buttons |= 0x01;
	}
	if (HAL_GPIO_ReadPin(BUT2_IN_GPIO_Port, BUT2_IN_Pin) == GPIO_PIN_RESET){
		buttons |= 0x02;
	}
	if (HAL_GPIO_ReadPin(BUT3_IN_GPIO_Port, BUT3_IN_Pin) == GPIO_PIN_RESET){
		buttons |= 0x04;
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
