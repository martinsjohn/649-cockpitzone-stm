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
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define LEFT_THOLD 87
#define RIGHT_THOLD 67

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;
uint8_t               TxData[4];
uint8_t               RxData[4];
uint32_t              TxMailbox;

//Initial Values of
uint8_t currWheel = 75;
uint8_t currThrottle = 0;
uint8_t currBrake = 0;
uint8_t current_blink = 0;
uint8_t blinking = 0;
uint8_t currBlinkCount = 0;
uint8_t maxBlinkCount = 25;
uint8_t blinkSend = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  volatile uint8_t buf[4];
  uint8_t send_ok[] = "CAN SEND OK\r\n";
  uint8_t send_bad[] = "CAN SEND ERROR\r\n";
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	//strcpy(buf, "hello world\r\n");
	//HAL_UART_Transmit(&huart2, buf, strlen(buf) + 1, 200);
	//HAL_StatusTypeDef recvnum = HAL_UART_Receive(&huart1, buf, 22, HAL_MAX_DELAY);
	//if(recvnum != HAL_OK){
		//strcpy(buf, "ERROR!");
	//}


	//HAL_StatusTypeDef rcvSt =
	HAL_I2C_Slave_Receive(&hi2c1, &buf[0], 1, HAL_MAX_DELAY);
  	//strcpy(buf, "hello\n");
  	//HAL_UART_Transmit(&huart2, buf, 4, 200);
  	//HAL_UART_Transmit(&huart2, "\r\n", 3, 200);
	//HAL_UART_Transmit(&huart2, "first: ", sizeof("first: "), 200);
  	//HAL_UART_Transmit(&huart2, &buf[0], 1, 200);
  	HAL_I2C_Slave_Receive(&hi2c1, &buf[1], 1, HAL_MAX_DELAY);
  	//HAL_UART_Transmit(&huart2, ", second: ", sizeof(", second:"), 200);
  	//HAL_UART_Transmit(&huart2, &buf[1], 1, 200);
  	HAL_I2C_Slave_Receive(&hi2c1, &buf[2], 1, HAL_MAX_DELAY);
  	//HAL_UART_Transmit(&huart2, ", thrid: ", sizeof(", thrid: "), 200);
  	//HAL_UART_Transmit(&huart2, &buf[2], 1, 200);
  	HAL_I2C_Slave_Receive(&hi2c1, &buf[3], 1, HAL_MAX_DELAY);
  	//HAL_UART_Transmit(&huart2, ", fourth:", sizeof(", fourth: "), 200);
	//HAL_UART_Transmit(&huart2, &buf[3], 1, 200);
	//HAL_UART_Transmit(&huart2, "\r\n", 3, 200);
  	/*
  	if(buf[1] > 50){
  		TxData[1] = 100;
  		HAL_UART_Transmit(&huart2, "Throttle\r\n", strlen("Throttle\r\n") + 1, 200);
  	}else{
  		TxData[1] = 0;
  		HAL_UART_Transmit(&huart2, "No Thr\r\n", strlen("No Thr\r\n") + 1, 200);
  	}
  	*/





// blinking
// 0 - no blink
// 1 - left blink
// 2 - right blink
// 3 - both blink


if (current_blink == 0 && buf[3] > 0) { // blink button pressed down
    if (blinking == 1 && buf[3] == 2) {
        blinking = 2;
        blinkSend = 2;
    }
    else if (blinking == 1 && buf[3] == 1){
    	blinking = 0;
    }
    else if (blinking == 2 && buf[3] == 1){
    	blinking = 1;
    	blinkSend = 1;
    }
    else if (blinking == 2 && buf[3] == 2){
    	blinking = 0;
    }
    else {
        blinking = buf[3];
        blinkSend = buf[3];
    }
    current_blink = buf[3];
} else if (current_blink > 0 && buf[3] == 0) { // blink button released
    current_blink = 0;
}


if (blinking == 1 && currWheel >= LEFT_THOLD && buf[0] < LEFT_THOLD) {
    blinking = 0;
    blinkSend = 0;
}

if (blinking == 2 && currWheel <= RIGHT_THOLD && buf[0] > RIGHT_THOLD) {
    blinking = 0;
    blinkSend = 0;
}

currWheel = buf[0];





	/*
  	//Toggle blink on for button press
  	if((currBlink == 0) && (buf[3] > 0)){
  		currBlink = buf[3];
  		if(blinkOn == 0){
  			blinkOn = buf[3];
  		}else{
  			blinkOn = 0;
  		}

  	}
  	//Toggle blink off for button release
  	else if((currBlink > 0) && (buf[3] == 0)){
  		currBlink = 0;
  	}
  	*/


	if(blinking > 0){
		currBlinkCount ++;
		if(currBlinkCount >= maxBlinkCount){
			currBlinkCount = 0;
			if(blinkSend == 0){
				blinkSend = blinking;
			}
			else{
				blinkSend = 0;
			}
		}
	}
	else{
		currBlinkCount = 0;
		blinkSend = 0;
	}




  	TxData[0] = buf[0]; //Wheel Angle (25-125
  	TxData[1] = buf[1]; // Throttle Angle (0-100)
  	TxData[2] = buf[2]; // Brake (0-100)
  	TxData[3] = blinkSend; // Blinkers (0, 1, 2)

  	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  	{

  	  //Transmission request Error

  		HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2,send_bad,sizeof(send_bad),100);// Sending in normal mode
  		if (status == HAL_OK) {
  		}
  	  Error_Handler();
  	}
  	HAL_Delay(10);
  	HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2,send_ok,sizeof(send_ok),100);// Sending in normal mode
  	if (status == HAL_OK) {
  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
  	}








	//strcpy(buf, "YOOOO!\r\n");
	//HAL_UART_Transmit(&huart2, buf, strlen(buf) + 1, HAL_MAX_DELAY);


//	HAL_UART_Receive(&huart1, buf, 20, HAL_MAX_DELAY);
//	HAL_UART_Transmit(&huart2, buf, 20, HAL_MAX_DELAY);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */
	CAN_FilterTypeDef  sFilterConfig;


  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_8TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = 0;
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
	  {
		/* Filter configuration Error */
		Error_Handler();
	  }

  /*##-3- Start the CAN peripheral ###########################################*/
  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
	/* Start Error */
	Error_Handler();
  }

  /*##-4- Activate CAN RX notification #######################################*/
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
	/* Notification Error */
	Error_Handler();
  }

  /*##-5- Configure Transmission process #####################################*/
  TxHeader.StdId = 0x320;
  TxHeader.ExtId = 0x01;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 4;
  TxHeader.TransmitGlobalTime = DISABLE;

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
  * @brief  Rx Fifo 0 message pending callback
  * @param  hcan: pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* Get RX message */
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    /* Reception Error */
    Error_Handler();
  }

  /* Display LEDx */
  if ((RxHeader.StdId == 0x321))
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,RxData[0]);
  }
  uint8_t can_rcv[] = "CAN RCV OK\r\n";
  HAL_StatusTypeDef status1 = HAL_UART_Transmit(&huart2,can_rcv,sizeof(can_rcv),100);// Sending in normal mode
	if (status1 == HAL_OK) {

	}

	if (RxHeader.StdId == 0x322) {

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
