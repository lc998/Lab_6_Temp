/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include <math.h>
#include "delay.h"
#include "lcdDisplay.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern uint32_t sekTick, usekTick;
TextLCDType lcd;
uint8_t rows = 2, cols = 16;

typedef enum {
	s_ZERO, s_ONE
} position_state;
//typedef enum {FALSE, TRUE} bool;
position_state s_position = s_ZERO;

uint32_t sekCounter = 0;

uint16_t channel_1 = 0;

uint16_t DHT11_timeout = 10000;
int16_t DHT11_buff_raw[41];
int8_t DHT11_data[5];
int8_t _userTemp = 20;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_ADC1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
//HAL_GPIO_EXTI_IRQHandler(uint16_t GPIO_Pin);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
int16_t read_DHT11(void);
int16_t read_DHT11_bit_raw(void);
int16_t readAnalogTemp(void);
void printClock(TextLCDType *lcd, uint8_t x, uint8_t y, uint8_t time);
void decreaseUserTemp(void);
void increaseUserTemp(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void) {
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
	MX_TIM10_Init();
	MX_TIM11_Init();
	MX_ADC1_Init();
	/* USER CODE BEGIN 2 */
	MX_USART2_UART_Init();

	HAL_TIM_Base_Start_IT(&htim10);
	TextLCD_Init(&lcd, LCD_RS_GPIO_Port, LCD_RS_Pin, LCD_RW_Pin, LCD_E_Pin,
	LCD_D0_GPIO_Port,
			(LCD_D0_Pin | LCD_D1_Pin | LCD_D2_Pin | LCD_D3_Pin | LCD_D4_Pin
					| LCD_D5_Pin | LCD_D6_Pin | LCD_D7_Pin), rows, cols);

	TextLCD_Puts(&lcd, "Goddag! :)");
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	int i = 0;

	uint8_t sekCount = 0;
	sekTick = 39660;
	uint32_t currentSek = sekTick + 1;
	uint16_t str[20];

	int16_t _temp;
	int16_t _DHT11;
	while (1) {

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		if (currentSek != sekTick) {

			uint32_t temp = sekTick;

			uint32_t hours = temp / 3600;
			uint32_t tenHours = hours / 10;

			uint32_t minutes = (temp % 3600) / 60;
			uint32_t tenMinutes = minutes / 10;

			uint32_t seconds = temp % 60;
			uint32_t tenSeconds = seconds / 10;

			TextLCD_Position(&lcd, 1, 0);

			//TextLCD_Position(lcd, x, y);

			TextLCD_Putchar(&lcd, 0x30 + tenHours);
			TextLCD_Putchar(&lcd, 0x30 + hours % 10);
			TextLCD_Putchar(&lcd, ':');
			TextLCD_Putchar(&lcd, 0x30 + tenMinutes);
			TextLCD_Putchar(&lcd, 0x30 + (minutes % 10));
			TextLCD_Putchar(&lcd, ':');
			TextLCD_Putchar(&lcd, 0x30 + (tenSeconds % 10));
			TextLCD_Putchar(&lcd, 0x30 + (seconds % 10));


			//USER TEMP PRINT
			TextLCD_Position(&lcd, 0, 10);
			TextLCD_PutInt(&lcd, _userTemp);
			TextLCD_Putchar(&lcd, 'C');
			TextLCD_Putchar(&lcd, ' ');


			currentSek = sekTick;

			sekCount = (sekCount + 1) % 60;
			i++;

			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

			//HAL_Delay(1000);

		}
		//every 5th second
		if ((sekTick % 2) == 0) {
			HAL_Delay(1000);

			_temp = readAnalogTemp();
			_DHT11 = read_DHT11();

			sprintf(str, "DHT11:%d | temp:%d \n\r", _DHT11, _temp);
			HAL_UART_Transmit(&huart2, str, strlen(str), 1000);

			TextLCD_Position(&lcd, 1, 9);
			TextLCD_PutInt(&lcd, _DHT11);
			TextLCD_Putchar(&lcd, 'C');
			TextLCD_PutInt(&lcd, _temp);
			TextLCD_Putchar(&lcd, 'C');
			TextLCD_Putchar(&lcd, ' ');
		}

		HAL_Delay(10);

	}
	/* USER CODE END 3 */

}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void) {

	ADC_ChannelConfTypeDef sConfig;

	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM10 init function */
static void MX_TIM10_Init(void) {

	htim10.Instance = TIM10;
	htim10.Init.Prescaler = 1343;
	htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim10.Init.Period = 62499;
	htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_Base_Init(&htim10) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM11 init function */
static void MX_TIM11_Init(void) {

	htim11.Instance = TIM11;
	htim11.Init.Prescaler = 0;
	htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim11.Init.Period = 79;
	htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_Base_Init(&htim11) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USART2 init function */
static void MX_USART2_UART_Init(void) {

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOH_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC,
			LCD_D0_Pin | LCD_D1_Pin | LCD_D2_Pin | LCD_D3_Pin | LCD_D4_Pin
					| LCD_D5_Pin | LCD_D6_Pin | LCD_D7_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, DHT_11_Pin | LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LCD_RS_Pin | LCD_RW_Pin | LCD_E_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pins : B1_Pin BUTTON_DOWN_Pin BUTTON_UP_Pin */
	GPIO_InitStruct.Pin = B1_Pin | BUTTON_DOWN_Pin | BUTTON_UP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : LCD_D0_Pin LCD_D1_Pin LCD_D2_Pin LCD_D3_Pin
	 LCD_D4_Pin LCD_D5_Pin LCD_D6_Pin LCD_D7_Pin */
	GPIO_InitStruct.Pin = LCD_D0_Pin | LCD_D1_Pin | LCD_D2_Pin | LCD_D3_Pin
			| LCD_D4_Pin | LCD_D5_Pin | LCD_D6_Pin | LCD_D7_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : DHT_11_Pin */
	GPIO_InitStruct.Pin = DHT_11_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DHT_11_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LCD_RS_Pin LCD_RW_Pin LCD_E_Pin */
	GPIO_InitStruct.Pin = LCD_RS_Pin | LCD_RW_Pin | LCD_E_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

int16_t read_DHT11(void) {
	uint16_t cnt = 0;

	//Initiate conversion
	HAL_Delay(100);
	HAL_GPIO_WritePin(DHT_11_GPIO_Port, DHT_11_Pin, 0);
	HAL_Delay(20);
	HAL_GPIO_WritePin(DHT_11_GPIO_Port, DHT_11_Pin, 1);

	//Make sure GPIO is high before we start to compute data from DHT11 sensor
	while (HAL_GPIO_ReadPin(DHT_11_GPIO_Port, DHT_11_Pin) == 1) {
		cnt++;
		if (cnt > DHT11_timeout) {
			return -99;
		}
	}

	//Read raw bit timing values. Negative values indicates 0 and positive values 1.
	for (uint8_t k = 0; k < 41; k++) {
		DHT11_buff_raw[k] = read_DHT11_bit_raw();
	}

	//Convering raw timing values. Negative values indicates 0 and positive values 1.
	uint8_t val;
	for (uint8_t byte = 0; byte < 5; byte++) {
		val = 0;
		for (uint8_t bit = 0; bit < 8; bit++) {
			if (DHT11_buff_raw[(byte << 3) + bit + 1] >= DHT11_timeout) {
				return -99;
			}
			val = val << 1; //shift left as MSB first
			//Check if we timed out during reading bit
			if (DHT11_buff_raw[(byte << 3) + bit + 1] > 0) {
				val = val | 0x01;
			}
		}
		DHT11_data[byte] = val;
	}

	uint8_t sum = 0;
	for (int byte = 0; byte < 4; byte++) {
		sum = sum + DHT11_data[byte];
	}

	if (sum != DHT11_data[4]) {
		return -99;
	}

	return DHT11_data[2];
}

int16_t read_DHT11_bit_raw(void) {
	uint16_t cnt_low = 0;

	//count how long many loops the GPIO pin is low
	while (HAL_GPIO_ReadPin(DHT_11_GPIO_Port, DHT_11_Pin) == 0) {
		cnt_low++;
		if (cnt_low >= DHT11_timeout)
			return DHT11_timeout;
	}

	uint16_t cnt_high = 0;

	//count how long many loops the GPIO pin is high
	while (HAL_GPIO_ReadPin(DHT_11_GPIO_Port, DHT_11_Pin) == 1) {
		cnt_high++;
		if (cnt_high >= DHT11_timeout) {
			return DHT11_timeout;
		}
	}

	//Return the difference between low and high count.
	//Will be < 0 for a '0'-bit and > 0 for a '1'-bit
	return cnt_high - cnt_low;
}

int16_t readAnalogTemp(void) {
	int16_t temperature = 0;
	float maxADCValue = 4096.0;

	HAL_ADC_Start(&hadc1);
	if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
		channel_1 = HAL_ADC_GetValue(&hadc1);
	}
	HAL_ADC_Stop(&hadc1);

	float Vout = (3.3 / maxADCValue) * channel_1;

	/*Calculate Thermistor Resistance. Code Begin */
	//R1 = -(10k*V/(V-1))		d�r v �r Vin/Vcc
	//!float Vin = channel_1 / (float) maxADCValue;
	//float R1 = (-1.0) * ((10000.0 * Vin) / (Vin - 1.0f));
	//!float R1 = 10000 * (Vin/5) - 1;

	float Rth = ((10000 * 5) / Vout) - 10000;

	/*Calculate Thermistor Resistance. Code End*/

	/*Calculate Thermistor temp. Code Begin*/
	//FORMEL:: temp = 1/( ( ((ln R1 - ln 10000)/B) + (1/298.15) ) )
	float B = 3450;
	//!float C = (1.0f / 298.15);
//
	//!float lgr1 = logf(R1);
	//!float lg10k = logf(10000.0);
//
	//!float X = (lgr1 - lg10k) / B;
//
	//!float T = 1.0f / (X + C);

	float T = (298.15 * B) / ((298.15 * logf(Rth / 10000)) + B);

	temperature = (int16_t) T;
	/*Calculate Thermistor temp. Code End*/
	return temperature - 273;
}

void decreaseUserTemp(void) {
	if (_userTemp > 15)
		_userTemp = _userTemp - 1;
}
void increaseUserTemp(void) {
	if (_userTemp < 30)
		_userTemp = _userTemp + 1;
}

void printClock(TextLCDType *lcd, uint8_t x, uint8_t y, uint8_t time) {
//	uint8_t tempTime = time;
//
//	uint8_t min = tempTime % 10;
//	tempTime = tempTime / 10;
//
//	uint8_t tioMin = (tempTime % 6);
//	tempTime = tempTime / 6;
//
//	uint8_t hour = tempTime % 10;
//	tempTime = tempTime % 10;
//
//	uint8_t tioHours = tempTime;

	uint32_t temp = sekTick;

	uint32_t hours = temp / 3600;
	uint32_t tenHours = hours / 10;

	uint32_t minutes = (temp % 3600) / 60;
	uint32_t tenMinutes = minutes / 10;

	uint32_t seconds = temp % 60;
	uint32_t tenSeconds = seconds / 10;

	TextLCD_Position(lcd, x, y);
	HAL_Delay(5);
	//TextLCD_Putchar(lcd, 0x30+tioHours);
	//TextLCD_Putchar(lcd, 0x30+hour);
	TextLCD_Putchar(lcd, ':');
	HAL_Delay(5);
	//TextLCD_Putchar(lcd, 0x30+tioMin);
	//TextLCD_Putchar(lcd, 0x30+min);
}
//void HAL_GPIO_EXTI_IRQHandler
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	//Echo pin
	if (GPIO_Pin == BUTTON_UP_Pin) {
		increaseUserTemp();
	}
	if (GPIO_Pin == BUTTON_DOWN_Pin) {
		decreaseUserTemp();
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
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
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
