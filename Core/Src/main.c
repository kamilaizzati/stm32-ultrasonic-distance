/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include <string.h>
#include <math.h>
#include <stdlib.h>

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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim21;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define TRIG_PIN GPIO_PIN_7
#define TRIG_PORT GPIOA
#define ECHO_PIN GPIO_PIN_6
#define ECHO_PORT GPIOA
//#define buzzer_pin GPIO_PIN_5
//#define buzzer_port GPIOA
uint32_t pMillis;
uint32_t Value1 = 0;
uint32_t Value2 = 0;
uint16_t Distance = 0;  // cm
uint16_t raw = 0;
char msg[20];
uint8_t rxData[20];  // Buffer untuk menerima data
uint8_t rxIndex = 0; // Index untuk buffer
uint8_t dataComplete = 0; // Flag untuk data lengkap
uint16_t receivedValue = 0;
uint16_t out;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM21_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_TIM21_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim21);
	HAL_UART_Receive_IT(&huart2, &rxData[0], 1);
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET); // pull the TRIG pin low
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET); // pull the TRIG pin HIGH
		__HAL_TIM_SET_COUNTER(&htim21, 0);
		while (__HAL_TIM_GET_COUNTER (&htim21) < 10)
			;  // wait for 10 us
		HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET); // pull the TRIG pin low

		pMillis = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
		// wait for the echo pin to go high
		while (!(HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN))
				&& pMillis + 10 > HAL_GetTick())
			;
		Value1 = __HAL_TIM_GET_COUNTER(&htim21);

		pMillis = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
		// wait for the echo pin to go low
		while ((HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN))
				&& pMillis + 50 > HAL_GetTick())
			;
		Value2 = __HAL_TIM_GET_COUNTER(&htim21);

		//	raw = Value2-Value1;
		Distance = (Value2 - Value1) * 0.32;
		if (Distance < 5) {
			out = 1;
		}
		else {
			out = 0;
		}
		snprintf(msg, sizeof(msg), "%u\n", out);
		HAL_UART_Transmit_IT(&huart2, (uint8_t*) msg, strlen(msg));
		HAL_Delay(5);

	    if (dataComplete) {
	        dataComplete = 0;

	        if (receivedValue == 1) {
	            // When distance is less than 5 cm, send 1
	            HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	            __HAL_TIM_SET_AUTORELOAD(&htim2, 5);
	            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 5);
	            HAL_Delay(500);
	            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
	        } else {
	            // When distance is greater than 5 cm, send 0
	            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
	        }
	    }

	}
}

	/* USER CODE END 3 */

	/**
	 * @brief System Clock Configuration
	 * @retval None
	 */
	void SystemClock_Config(void) {
		RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
		RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
		RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

		/** Configure the main internal regulator output voltage
		 */
		__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

		/** Initializes the RCC Oscillators according to the specified parameters
		 * in the RCC_OscInitTypeDef structure.
		 */
		RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
		RCC_OscInitStruct.HSIState = RCC_HSI_ON;
		RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
		RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
		RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
		RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
		RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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

		if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1)
				!= HAL_OK) {
			Error_Handler();
		}
		PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
		PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
		if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
			Error_Handler();
		}
	}

	/**
	 * @brief TIM2 Initialization Function
	 * @param None
	 * @retval None
	 */
	static void MX_TIM2_Init(void) {

		/* USER CODE BEGIN TIM2_Init 0 */

		/* USER CODE END TIM2_Init 0 */

		TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
		TIM_MasterConfigTypeDef sMasterConfig = { 0 };
		TIM_OC_InitTypeDef sConfigOC = { 0 };

		/* USER CODE BEGIN TIM2_Init 1 */

		/* USER CODE END TIM2_Init 1 */
		htim2.Instance = TIM2;
		htim2.Init.Prescaler = 800 - 1;
		htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim2.Init.Period = 200 - 1;
		htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
			Error_Handler();
		}
		sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
		if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
			Error_Handler();
		}
		if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
			Error_Handler();
		}
		sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
		sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
		if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
				!= HAL_OK) {
			Error_Handler();
		}
		sConfigOC.OCMode = TIM_OCMODE_PWM1;
		sConfigOC.Pulse = 0;
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2)
				!= HAL_OK) {
			Error_Handler();
		}
		/* USER CODE BEGIN TIM2_Init 2 */

		/* USER CODE END TIM2_Init 2 */
		HAL_TIM_MspPostInit(&htim2);

	}

	/**
	 * @brief TIM21 Initialization Function
	 * @param None
	 * @retval None
	 */
	static void MX_TIM21_Init(void) {

		/* USER CODE BEGIN TIM21_Init 0 */

		/* USER CODE END TIM21_Init 0 */

		TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
		TIM_MasterConfigTypeDef sMasterConfig = { 0 };

		/* USER CODE BEGIN TIM21_Init 1 */

		/* USER CODE END TIM21_Init 1 */
		htim21.Instance = TIM21;
		htim21.Init.Prescaler = 512 - 1;
		htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim21.Init.Period = 320 - 1;
		htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim21.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		if (HAL_TIM_Base_Init(&htim21) != HAL_OK) {
			Error_Handler();
		}
		sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
		if (HAL_TIM_ConfigClockSource(&htim21, &sClockSourceConfig) != HAL_OK) {
			Error_Handler();
		}
		sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
		sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
		if (HAL_TIMEx_MasterConfigSynchronization(&htim21, &sMasterConfig)
				!= HAL_OK) {
			Error_Handler();
		}
		/* USER CODE BEGIN TIM21_Init 2 */

		/* USER CODE END TIM21_Init 2 */

	}

	/**
	 * @brief USART2 Initialization Function
	 * @param None
	 * @retval None
	 */
	static void MX_USART2_UART_Init(void) {

		/* USER CODE BEGIN USART2_Init 0 */

		/* USER CODE END USART2_Init 0 */

		/* USER CODE BEGIN USART2_Init 1 */

		/* USER CODE END USART2_Init 1 */
		huart2.Instance = USART2;
		huart2.Init.BaudRate = 9600;
		huart2.Init.WordLength = UART_WORDLENGTH_8B;
		huart2.Init.StopBits = UART_STOPBITS_1;
		huart2.Init.Parity = UART_PARITY_NONE;
		huart2.Init.Mode = UART_MODE_TX_RX;
		huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
		huart2.Init.OverSampling = UART_OVERSAMPLING_16;
		huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
		huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
		if (HAL_UART_Init(&huart2) != HAL_OK) {
			Error_Handler();
		}
		/* USER CODE BEGIN USART2_Init 2 */

		/* USER CODE END USART2_Init 2 */

	}

	/**
	 * @brief GPIO Initialization Function
	 * @param None
	 * @retval None
	 */
	static void MX_GPIO_Init(void) {
		GPIO_InitTypeDef GPIO_InitStruct = { 0 };
		/* USER CODE BEGIN MX_GPIO_Init_1 */
		/* USER CODE END MX_GPIO_Init_1 */

		/* GPIO Ports Clock Enable */
		__HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();

		/*Configure GPIO pin Output Level */
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

		/*Configure GPIO pin : PA6 */
		GPIO_InitStruct.Pin = GPIO_PIN_6;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/*Configure GPIO pin : PA7 */
		GPIO_InitStruct.Pin = GPIO_PIN_7;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* USER CODE BEGIN MX_GPIO_Init_2 */
		/* USER CODE END MX_GPIO_Init_2 */
	}

	/* USER CODE BEGIN 4 */
	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
		if (huart->Instance == USART2) {
			// Proses data yang diterima
//        rxData[rxIndex] = rxData[0];  // Salin byte yang diterima

			if (rxData[rxIndex] == '\n')  // Jika menerima newline
					{
				rxData[rxIndex] = '\0';  // Null terminate string
				receivedValue = atoi((char*) rxData); // Konversi string ke integer
				dataComplete = 1;  // Set flag data complete
				rxIndex = 0;  // Reset index
			} else {
				rxIndex++;
				if (rxIndex >= sizeof(rxData) - 1)
					rxIndex = 0;  // Prevent buffer overflow
			}

			// Mulai menerima byte berikutnya
			HAL_UART_Receive_IT(huart, &rxData[rxIndex], 1);
		}
	}
	/* USER CODE END 4 */

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
