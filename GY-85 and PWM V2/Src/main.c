/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "ITG3205.h"
#include <QMC5883L.h>
#include "calculation.h"
#include <stdio.h>
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
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/*??????????????? ?????????? ****************************************************************************** */
float calibDataGyro[3]; /*?????? ??????????????? ?????? ?????????: ??????? ???????? ? ????/???*/ /*?????? ????? ?????? ????????? */
float calibDataMagnet[3]; /*?????? ??????????????? ?????? ????????????: ????????? ???????? ? ????? ????? */
char transmitMagnet[150]; /*?????? ?????? ???????????? ?? ???????? ?? UART*/
float currentAngle;		 /*??????? ????*/
int pinStatus = 0;		 /*?????? ???????? ????? ??? ??? */
int pinStatusX = 2, pinStatusY = 2; /*?????? ???????? ????? ??? ??? */
int uartCount = 0;
//float koeff = 321037;	/*??????????? ??????? ??????? ????????, ????? ?????????? ????????*/
//const float _koeff = 10000;
float gyroCalibZ = 0;
float voltageX = 0, voltageY = 0;
float magnetMedianX = 0;
float magnetMedianY = 0;
float magnetMedianZ = 0;

uint8_t PWM_PA3 = 240; /*?????????? ?? ????????? ??? ?? PA3 */
uint8_t PWM_PA1 = 240; /*?????????? ?? ????????? ??? ?? PA1 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART1_UART_Init(void);
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
	MX_TIM2_Init();
	MX_I2C3_Init();
	MX_USART1_UART_Init();
	I2Cdev_init(&hi2c3);
        HAL_Delay(3000);
	QMC5883L_init();
	initGyro();
	/* USER CODE BEGIN 2 */
	/*???????? ??? **********************************************************************************************/
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	HAL_GPIO_WritePin(GPIOB, IN_4_Pin | IN_2_Pin, GPIO_PIN_RESET);
	/* USER CODE END 2 */
	/*???????? ??????????????? ?????? ? ????????? ? ?????????? ???????? ??????? ???????? ? ??????????????? ??????*/
	getGyroscopeData(calibDataGyro);
	/*???????? ??????????????? ?????? ? ???????????? ? ?????????? ???????? ???? ? µT ? ??????????????? ?????? * */
	QMC5883L_read(calibDataMagnet);
	/*???????????? "???????" ???? ???????? ************************************************************************/
	currentAngle = ((atan2(-calibDataMagnet[1], calibDataMagnet[0])) * 180) / PI;
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		float testMagnet[3] = {-0.0005, 0.0, 0.0};
		float testGyro = 20.0;
		float testAngle = 0;
		getVoltage(gyroCalibZ * PI / 180, calibDataMagnet, currentAngle*PI/180, &voltageX, &voltageY);
		//TEST 
                //getVoltage(testGyro * PI / 180, testMagnet, testAngle, &voltageX, &voltageY);
		scaleVoltage(&voltageX, &voltageY);
                
		getPWM(voltageX, &pinStatusX, &PWM_PA1);
		// PWM_PA1 = m_axis(_koeff, gyroCalibZ * PI / 180, -calibDataMagnet[1] / 1000000, &pinStatusX); // Axis X  ORIG
		if (pinStatusX == 0) {
			HAL_GPIO_WritePin(GPIOB, IN_4_Pin, GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(GPIOB, IN_4_Pin, GPIO_PIN_SET);
		}
		htim2.Instance->CCR2 = PWM_PA1;

		getPWM(voltageY, &pinStatusY, &PWM_PA3);
		if (pinStatusY == 0) {
			HAL_GPIO_WritePin(GPIOB, IN_2_Pin, GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(GPIOB, IN_2_Pin, GPIO_PIN_SET);
		}
		htim2.Instance->CCR4 = PWM_PA3;
                //********************************************************************
                //UART telemetry sending code
                uartCount++;
		if (uartCount > 4) {
			//sprintf(transmitMagnet, "GG m_X = %0.2f   m_Y = %0.2f  angle = %0.2f speed = %0.2f\r\n", calibDataMagnet[0]*1000000 , calibDataMagnet[1]*1000000, currentAngle, gyroCalibZ * PI / 180.0);
			sprintf(transmitMagnet, "Gu dirX = %i  dirY = %i PA1 = %d  PA3 = %d  m_X = %0.2f   m_Y = %0.2f  speed = %0.2f angle = %0.2f \r\n", pinStatusX, pinStatusY, PWM_PA1, PWM_PA3, calibDataMagnet[0] * 1000000, calibDataMagnet[1] * 1000000, gyroCalibZ * PI / 180.0, currentAngle);
                        HAL_UART_Transmit_IT(&huart1, transmitMagnet, 150);
			uartCount = 0;
                        HAL_Delay(10);
		}

                
		/*?????????? ??????? ************************************************** */
		HAL_Delay(100);
		PWM_PA1 = 0;
		PWM_PA3 = 0;
		htim2.Instance->CCR4 = PWM_PA1;
		htim2.Instance->CCR2 = PWM_PA3;
                HAL_GPIO_WritePin(GPIOB, IN_2_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOB, IN_4_Pin, GPIO_PIN_RESET);
                HAL_Delay(30);
		/*???????????? "???????" ???? ???????? ************************************************************************/
		//Magnetometer averaging 
                magnetMedianX = 0;
                magnetMedianY = 0;
                magnetMedianZ = 0;
                for (int i = 0; i < 3; i++) {
			QMC5883L_read(calibDataMagnet);
			magnetMedianY += calibDataMagnet[0];
                        magnetMedianX += calibDataMagnet[1];
                        magnetMedianZ += calibDataMagnet[2];
		}
                calibDataMagnet[0] = (magnetMedianY / 3.0)*1e-6;
		calibDataMagnet[1] = (magnetMedianX / 3.0)*1e-6;
		calibDataMagnet[2] = (magnetMedianY / 3.0)*1e-6;
                //**************************************************
                //Gyro averaging
                gyroCalibZ = 0.0;
                for (int i = 0; i < 3; i++) {
			getGyroscopeData(calibDataGyro);
			gyroCalibZ += calibDataGyro[2];
		}
		gyroCalibZ /= 3;
                if (gyroCalibZ > 6){
             //     resetGyroscope();
               //   initGyro();
                }  
                currentAngle = ((atan2(-calibDataMagnet[1], calibDataMagnet[0])) * 180) / PI;
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;
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
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_I2C3;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
	PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
		Error_Handler();
	}
	/** Enable MSI Auto calibration
	 */
	HAL_RCCEx_EnableMSIPLLMode();
}

/**
 * @brief I2C3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C3_Init(void) {

	/* USER CODE BEGIN I2C3_Init 0 */

	/* USER CODE END I2C3_Init 0 */

	/* USER CODE BEGIN I2C3_Init 1 */

	/* USER CODE END I2C3_Init 1 */
	hi2c3.Instance = I2C3;
	hi2c3.Init.Timing = 0x00707CBB;
	hi2c3.Init.OwnAddress1 = 0;
	hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c3.Init.OwnAddress2 = 0;
	hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c3) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C3_Init 2 */

	/* USER CODE END I2C3_Init 2 */
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 125 - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 255;
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
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, IN_2_Pin | IN_4_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : IN_2_Pin IN_4_Pin */
	GPIO_InitStruct.Pin = IN_2_Pin | IN_4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
void assert_failed(char *file, uint32_t line) {
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	   tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
