/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "math.h"
#include "MLX90640_I2C_Driver.h"
#include "MLX90640_API.h"
#include <stdio.h>

#define MLX_ADDR 0x33
#define REFRESH_RATE 0x02
#define TA_SHIFT 8
uint16_t MLX_eeData[832];
paramsMLX90640 MLX_params;
uint16_t MLX_dataFrame[834];
float MLX_to[768];
float emissivity = 0.95;
int status;

double MLX_average[3];

void SystemClock_Config(void);

/**
 * @brief  The application entry point.
 * @retval int
 */

int _write(int file, char *data, int len) {
	// Transmit via UART using HAL function
	HAL_UART_Transmit(&huart2, (uint8_t*) data, len, HAL_MAX_DELAY);
	return len;
}

void computeMLXAverage() {
	uint16_t left, middle, right = 0;
	for (size_t row = 0; row < 24; row++) {
		for (size_t col = 0; col < 32; col++) {
			uint16_t index = row * 32 + col;
			// We only count "valid" temp values; i.e. non-zero, NOT NaN
			// Account for how many valid values are in each region for the average
			if (MLX_to[index] != 0 && MLX_to[index] == MLX_to[index]) {
				if (col >= 0 && col < 10) {
					MLX_average[0] += MLX_to[index];
					left++;
				}
				if (col >= 10 && col < 22) {
					MLX_average[1] += MLX_to[index];
					middle++;
				}
				if (col >= 22 && col < 32) {
					MLX_average[2] += MLX_to[index];
					right++;
				}
			}
		}
	}

	// Average only if there is a non-trivial amount of temp values per region
	if (left > 0) MLX_average[0] /= left;
	if (middle > 0) MLX_average[1] /= middle;
	if (right > 0) MLX_average[2] /= right;
}

int main(void) {
	HAL_Init();

	SystemClock_Config();

	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	MLX90640_SetRefreshRate(MLX_ADDR, REFRESH_RATE);
	MLX90640_SetChessMode(MLX_ADDR);
	status = MLX90640_DumpEE(MLX_ADDR, MLX_eeData);
	printf("\r\nLOADING EEPROM PARAMETERS:%d\r\n", status);
	status = MLX90640_ExtractParameters(MLX_eeData, &MLX_params);
	printf("\r\nEXTRACTING PARAMETERS:%d\r\n", status);

	while (1) {
		status = MLX90640_GetFrameData(MLX_ADDR, MLX_dataFrame);
		float tr = MLX90640_GetTa(MLX_dataFrame, &MLX_params) - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
		MLX90640_CalculateTo(MLX_dataFrame, &MLX_params, emissivity, tr,
				MLX_to);
		computeMLXAverage();
	}
}

/**
 * @brief System Clock Configuration
 * @retval None
 */

void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE
			| RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 16;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}

	/** Enable MSI Auto calibration
	 */
	HAL_RCCEx_EnableMSIPLLMode();
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
