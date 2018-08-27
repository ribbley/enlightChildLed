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
#include "stm32f0xx_hal_msp.h"

/* USER CODE BEGIN Includes */
#include <string.h>

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

#define LED_X_COUNT 5
#define LED_Y_COUNT 5
#define LED_COUNT 25
struct PIXEL {
  uint8_t data[9];
};
struct PIXEL grb_array[LED_COUNT];
struct PIXEL temp_grb_array[LED_COUNT];

const uint8_t white [] = {0b11011011, 0b01101101, 0b10110110, 0b11011011, 0b01101101, 0b10110110, 0b11011011, 0b01101101, 0b10110110};
const uint8_t black [] = {0b10010010, 0b01001001, 0b10100100, 0b10010010, 0b01001001, 0b10100110, 0b10010010, 0b01001001, 0b10100110};

uint16_t adc_array[4] = {0, 0, 0, 0};

uint8_t select_config = 0;
int8_t selected [] = {0, 0};
uint8_t saving = 0;
uint32_t lastInputTime = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void checkButtonsAndSelected();
void updateColors();
void fetchPotentiometerValues();
void checkStateAndSetColor();
int16_t map(int16_t in, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max);

void setSelected(uint8_t red, uint8_t green, uint8_t blue);
void setPixel(uint8_t x, uint8_t y, uint8_t red, uint8_t green, uint8_t blue);
void setColumn(uint8_t column, uint8_t red, uint8_t green, uint8_t blue);
void setRow(uint8_t row, uint8_t red, uint8_t green, uint8_t blue);
void setAll(uint8_t red, uint8_t green, uint8_t blue);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  //@init
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
  MX_SPI1_Init();
  SPI_TxDeInit(&hspi1);

  for (uint8_t i = 0; i < 5 ; i++){
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
    HAL_Delay(300);
  }

  /* USER CODE BEGIN 2 */

  //init array
  for (uint8_t y = 0; y < 5; y++){
    for (uint8_t x = 0; x < 5; x++){
      setPixel(x, y, 0, 0, 0);
    }
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

#define UPDATE_RATE 50
#define UPDATE_PERIOD (1000 / UPDATE_RATE)
#define ACTIVE_LEDS 25
#define MAX_BRIGHTNESS 40

  while (1)
  {
    //@while
    static uint32_t lastUpdate = 0;

    if ( HAL_GetTick() > (lastUpdate + UPDATE_PERIOD)) {

      checkButtonsAndSelected();
      fetchPotentiometerValues();
      checkStateAndSetColor();

      updateColors();
      lastUpdate = HAL_GetTick();
    }

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL11;
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
  hadc.Init.Resolution = ADC_RESOLUTION_10B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
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

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
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
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

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

  /*Configure GPIO pins : BUT2_IN_Pin BUT1_IN_Pin */
  GPIO_InitStruct.Pin = BUT2_IN_Pin|BUT1_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_1; //old serial pin
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

#define BLINK_HALF_PERIOD 200
#define DESELECT_TIME 6000
#define MINIMAL_CHANGE_COLOR 4

#define STATE_IDLE 0
#define STATE_BLINK_SELECTED_ON 1
#define STATE_BLINK_SELECTED_OFF 2
#define STATE_SETTING_COLOR 3

void checkStateAndSetColor(){
  static uint8_t state = STATE_IDLE;
  static uint32_t blinkTime = 0;
  static uint8_t rgb_buff [] = {0, 0, 0};
  static uint8_t state_select_config = 0;
  static uint8_t state_selected [] = {0, 0};

  switch(state){
    case STATE_IDLE:
      if (select_config != 0){
        state = STATE_BLINK_SELECTED_ON;
        for (uint8_t i = 0; i < 3; i++)
          rgb_buff[i] = adc_array[i]; //init color compare array
        memcpy(temp_grb_array, grb_array, sizeof(struct PIXEL) * LED_COUNT);  //copy color array
        blinkTime = HAL_GetTick();
      }
      break;

    case STATE_BLINK_SELECTED_ON:

      if (adc_array[0] - rgb_buff[0] > MINIMAL_CHANGE_COLOR || rgb_buff[0] - adc_array[0] > MINIMAL_CHANGE_COLOR ||
          adc_array[1] - rgb_buff[1] > MINIMAL_CHANGE_COLOR || rgb_buff[1] - adc_array[1] > MINIMAL_CHANGE_COLOR ||
          adc_array[2] - rgb_buff[2] > MINIMAL_CHANGE_COLOR || rgb_buff[2] - adc_array[2] > MINIMAL_CHANGE_COLOR){
        //color has changed!
        state = STATE_SETTING_COLOR;
        state_select_config = select_config;
        memcpy(state_selected, selected, 2);
        for (uint8_t i = 0; i < 3; i++)
          rgb_buff[i] = adc_array[i];
        lastInputTime = HAL_GetTick();
      }

      if (HAL_GetTick() > lastInputTime + DESELECT_TIME){
        select_config = 0;
        state = STATE_IDLE;
        lastInputTime = HAL_GetTick();
      }

      if (HAL_GetTick() > blinkTime + BLINK_HALF_PERIOD){
        state = STATE_BLINK_SELECTED_OFF;
        setSelected(0, 0, 0);
        blinkTime = HAL_GetTick();
      }
      break;

    case STATE_BLINK_SELECTED_OFF:

      if (adc_array[0] - rgb_buff[0] > MINIMAL_CHANGE_COLOR || rgb_buff[0] - adc_array[0] > MINIMAL_CHANGE_COLOR ||
          adc_array[1] - rgb_buff[1] > MINIMAL_CHANGE_COLOR || rgb_buff[1] - adc_array[1] > MINIMAL_CHANGE_COLOR ||
          adc_array[2] - rgb_buff[2] > MINIMAL_CHANGE_COLOR || rgb_buff[2] - adc_array[2] > MINIMAL_CHANGE_COLOR){
        //color has changed!
        state = STATE_SETTING_COLOR;
        state_select_config = select_config;
        memcpy(state_selected, selected, 2);
        for (uint8_t i = 0; i < 3; i++)
          rgb_buff[i] = adc_array[i];
        lastInputTime = HAL_GetTick();
      }

      if (HAL_GetTick() > lastInputTime + DESELECT_TIME){
        select_config = 0;
        state = STATE_IDLE;
        lastInputTime = HAL_GetTick();
      }

      if (HAL_GetTick() > blinkTime + BLINK_HALF_PERIOD){
        state = STATE_BLINK_SELECTED_ON;
        setSelected(MAX_BRIGHTNESS, MAX_BRIGHTNESS, MAX_BRIGHTNESS);
        blinkTime = HAL_GetTick();
      }
      break;

    case STATE_SETTING_COLOR:
      //check if button input was given
      if ((state_select_config != select_config) || memcmp(state_selected, selected, 2) != 0){
        state = STATE_BLINK_SELECTED_ON;
        for (uint8_t i = 0; i < 3; i++)
          rgb_buff[i] = adc_array[i];
        break;
      }

      if (adc_array[0] - rgb_buff[0] > MINIMAL_CHANGE_COLOR || rgb_buff[0] - adc_array[0] > MINIMAL_CHANGE_COLOR ||
          adc_array[1] - rgb_buff[1] > MINIMAL_CHANGE_COLOR || rgb_buff[1] - adc_array[1] > MINIMAL_CHANGE_COLOR ||
          adc_array[2] - rgb_buff[2] > MINIMAL_CHANGE_COLOR || rgb_buff[2] - adc_array[2] > MINIMAL_CHANGE_COLOR){
        //color has changed!
        state = STATE_SETTING_COLOR;
        memcpy(rgb_buff, adc_array, 3);
        lastInputTime = HAL_GetTick();
      }

      if (HAL_GetTick() > lastInputTime + DESELECT_TIME){
        select_config = 0;
        state = STATE_IDLE;
        lastInputTime = HAL_GetTick();
        setSelected(MAX_BRIGHTNESS, MAX_BRIGHTNESS, MAX_BRIGHTNESS);
        break;
      }

      setSelected(adc_array[0], adc_array[1], adc_array[2]);
      memcpy(temp_grb_array, grb_array, sizeof(struct PIXEL) * LED_COUNT);
      break;

    default:
      break;
  }
}

void fetchPotentiometerValues(){
  for(uint8_t j = 0; j < 4 ; j++){
    HAL_ADC_Start(&hadc);
    for (uint8_t i = 0; i < 4; i++){
      HAL_ADC_PollForConversion(&hadc,5);
      adc_array[i] += HAL_ADC_GetValue(&hadc);
    }
    HAL_ADC_Stop(&hadc);
  }
  for (uint8_t i = 0; i < 4; i++){
    adc_array[i] = adc_array[i] >> 2; // divide by 4 and break down to 8 bit
    adc_array[i] = map(adc_array[i], 0, 1024, 0, MAX_BRIGHTNESS);
  }
}

#define LONG_PRESS_TIME 750

void checkButtonsAndSelected(){
  static uint32_t timePressed = 0;
	static uint8_t buttons = 0;

  //check if a button is pressed
	if (HAL_GPIO_ReadPin(BUT1_IN_GPIO_Port, BUT1_IN_Pin) == GPIO_PIN_RESET && !buttons){
		buttons = 0x01;  //forward
    timePressed = HAL_GetTick();
	}
	if (HAL_GPIO_ReadPin(BUT2_IN_GPIO_Port, BUT2_IN_Pin) == GPIO_PIN_RESET && !buttons){
		buttons = 0x02;  //backward
    timePressed = HAL_GetTick();
	}

  //check if a button is released
  if (((HAL_GPIO_ReadPin(BUT1_IN_GPIO_Port, BUT1_IN_Pin) == GPIO_PIN_SET) && (buttons == 0x01)) ||
      ((HAL_GPIO_ReadPin(BUT2_IN_GPIO_Port, BUT2_IN_Pin) == GPIO_PIN_SET) && (buttons == 0x02))){
    if (HAL_GetTick() > timePressed + LONG_PRESS_TIME){
      buttons <<= 4;  //register button press as longpress!
    }

    if (select_config == 0){
      select_config = 1;
    }

    //reset draw array to persistent state
    memcpy(grb_array, temp_grb_array, sizeof(struct PIXEL) * LED_COUNT);

    switch(buttons){
      case 0x01:
        //next

        //special case for row select
        if (select_config == 2){
          selected[1]++;
          if (selected[1] >= LED_Y_COUNT){
            selected[1] = 0;
          }
          break;
        }

        selected[0]++;
        if (selected[0] >= LED_X_COUNT){
          selected[0] = 0;
          selected[1]++;
          if (selected[1] >= LED_Y_COUNT){
            selected[1] = 0;
          }
        }
        break;
      case 0x02:
        //save
        select_config = 0;
        saving = 1;
        break;

      case 0x10:
        //increment select mode
        select_config++;
        if (select_config >= 5)
          select_config = 1;
        break;

      case 0x20:
        //decrement select mode
        select_config--;
        if (select_config <= 0)
          select_config = 4;
        break;
      default:
        break;
    }
    buttons = 0;
    lastInputTime = HAL_GetTick();
  }

}

void setSelected(uint8_t red, uint8_t green, uint8_t blue){
  switch (select_config) {
    case 0:
      break;

    case 1: //normal pixel mode
      setPixel(selected[0], selected[1], red, green, blue);
      break;

    case 2: //row mode
      setRow(selected[1], red, green, blue);
      break;

    case 3: //column mode
      setColumn(selected[0], red, green, blue);
      break;

    case 4: //fill mode
      setAll(red, green, blue);
      break;

    default:
      select_config = 0;
      break;
  }
}

/* Trying to understand this function is a bit tedious.
 * In this function, an array will be prepared for SPI transmission.
 * For every set bit, 110 (binary) will be written to the array, for each
 * reset bit, 100 (binary). This function handles all the offset and shifting
 * management to accomplish this successfully on a uint8_t array
 */
void setPixel(uint8_t x, uint8_t y, uint8_t red, uint8_t green, uint8_t blue){
  //nullify, so we can use xor to set pins
  memset(&grb_array[x + (5 * y)], 0, sizeof(struct PIXEL));
  uint8_t offset = 0;
  int8_t bit_shift_offset = 5;
  uint8_t color_array[3];
  color_array[0] = green > MAX_BRIGHTNESS ? MAX_BRIGHTNESS : green;
  color_array[1] = red > MAX_BRIGHTNESS ? MAX_BRIGHTNESS : red;
  color_array[2] = blue > MAX_BRIGHTNESS ? MAX_BRIGHTNESS : blue;

  for (uint8_t j = 0; j < 3; j++){
    for (uint8_t i = 0; i < 8; i++){
      //if we have negative value bit_shift_offset, we need to adjust this.
      if (bit_shift_offset < 0){
        uint8_t tmp = -bit_shift_offset;
        if (color_array[j] & (1 << (7-i))){
          //add upto 2 bits to fill up byte
          grb_array[x + (5 * y)].data[offset] |= (uint8_t) ((0b110)) >> tmp;
        }else{
          //add upto 2 bits to fill up byte
          grb_array[x + (5 * y)].data[offset] |= (uint8_t) ((0b100)) >> tmp;
        }
        bit_shift_offset = 8+bit_shift_offset;
        offset++;
      }

      if (color_array[j] & (1 << (7-i))){
        //add 110
        //uint8_t cast is important, bit_shift_offset can be > 5!
        grb_array[x + (5 * y)].data[offset] |= (uint8_t) ((0b110) << bit_shift_offset);
      }else{
        //add 100
        //uint8_t cast is important, bit_shift_offset can be > 5!
        grb_array[x + (5 * y)].data[offset] |= (uint8_t) ((0b100) << bit_shift_offset);
      }

      bit_shift_offset -= 3;
    }
  }
}

void setColumn(uint8_t column, uint8_t red, uint8_t green, uint8_t blue){
  setPixel(column, 0, red, green, blue);
  setPixel(column, 1, red, green, blue);
  setPixel(column, 2, red, green, blue);
  setPixel(column, 3, red, green, blue);
  setPixel(column, 4, red, green, blue);
}

void setRow(uint8_t row, uint8_t red, uint8_t green, uint8_t blue){
  setPixel(0, row, red, green, blue);
  setPixel(1, row, red, green, blue);
  setPixel(2, row, red, green, blue);
  setPixel(3, row, red, green, blue);
  setPixel(4, row, red, green, blue);
}

void setAll(uint8_t red, uint8_t green, uint8_t blue){
  setRow(0, red, green, blue);
  setRow(1, red, green, blue);
  setRow(2, red, green, blue);
  setRow(3, red, green, blue);
  setRow(4, red, green, blue);
}

void updateColors(){
  //start DMA transfer - let SPI control Pins
  SPI_TxInit(&hspi1);
  //HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *) grb_array, sizeof(struct PIXEL) * 1);

  HAL_SPI_Transmit(&hspi1, (uint8_t *) grb_array, sizeof(struct PIXEL) * ACTIVE_LEDS, 3);
  SPI_TxDeInit(&hspi1);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

}

int16_t map(int16_t in, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max){
  int16_t tmp;
  tmp = out_min + ((out_max - out_min) * 1.0 / (in_max - in_min)) * (in - in_min);
  tmp = tmp > MAX_BRIGHTNESS ? MAX_BRIGHTNESS : tmp < 0 ? 0 : tmp;
  return tmp;
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
