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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ENABLE_CAN_TRANSCEIVER() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET)
#define LED_OFF() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET)
#define LED_ON() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET)

#define SET_ERROR(n) errors |= 1UL << n;
#define CLEAR_ERROR(n) errors &= ~(1UL << n);

#define ABS(x) ((x) > 0 ? (x) : -(x))

#define __AS5048A2_CS_ENABLE() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define __AS5048A2_CS_DISABLE() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)

#define __Read_NOP 0xc000
#define __Read_Clear_Error_Flag 0x4001
#define __Read_Angle 0xffff
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

uint16_t getRawRotation(void);
uint32_t sendCAN(uint16_t id, uint8_t buffer[], uint8_t len);
uint32_t getRX0Count();
uint32_t getRX1Count();
void readCAN();
uint8_t isDeadFace(uint8_t buf[], uint8_t len);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t errors = 0;

CAN_FilterTypeDef     canfil;
CAN_TxHeaderTypeDef   txHeader;
CAN_RxHeaderTypeDef   rxHeader;
uint8_t               rxBuffer[8];
uint32_t              txMailbox;

uint8_t ignition = 0;
uint32_t lastSeen = 0;

int angle = 0;
int lastAngle = 0;

uint32_t counter = 0;
uint8_t readIndex = 0;
int readings[10];
int total;
int average;

uint16_t SPI_TX_DATA[4] = {__Read_Clear_Error_Flag, __Read_NOP, __Read_Angle, __Read_NOP};
uint16_t SPI_RX_DATA[4] = {0};

const uint8_t op_num = 4;
uint16_t origin_value = 0;
uint8_t i;

uint8_t TXD[4] = {0x55, 0xAA, 0x00, 0x00};
uint16_t post_process_value = 0;

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
  MX_SPI1_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  ENABLE_CAN_TRANSCEIVER();

  LED_OFF();
  if (HAL_CAN_Start(&hcan) != HAL_OK) {
	  LED_ON();
  }

  angle = 0;
  lastAngle = getRawRotation();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
	  readCAN();
	  if (HAL_GetTick() - lastSeen > 750) {
		  ignition = 0;
	  }

	  uint16_t rawAngle = getRawRotation();
	  int angleDelta = rawAngle - lastAngle;
	  angleDelta = (angleDelta + 24576) % 16384 - 8192;

	  angle += angleDelta;

	  if (angle > 49149 || angle < -49149) { // check sane (3 full rotations)
		  SET_ERROR(2);
	  } else {
		  CLEAR_ERROR(2);
	  }

	  total -= readings[readIndex];
	  readings[readIndex] = angle;
	  total += readings[readIndex++];
	  if (readIndex >= 10) {
	    readIndex = 0;
	  }

	  if (ignition == 1) {
		  LED_ON();
		  if (counter % 10 == 0) {
			  int raw = total / 10;
			  uint8_t txBuffer[] = {(raw >> 24) & 0xFF, (raw >> 16) & 0xFF,
					  	  	  	  	  (raw >> 8) & 0xFF, raw & 0xFF,
									  0x00, 0x00, 0x00, 0x00};
			  sendCAN(0x23, txBuffer, 8); // steering angle message
		  }

		  if (counter == 100) {
			  uint8_t errBuffer[1] = {errors};
			  sendCAN(0x231, errBuffer, 1); // error flags
			  counter = 0;
		  }

		  lastAngle = rawAngle;
		  counter++;
		  HAL_Delay(1);
	  } else {
		  LED_OFF();
		  HAL_Delay(500);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 2;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_10TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  canfil.FilterMode = CAN_FILTERMODE_IDMASK;
  canfil.FilterScale = CAN_FILTERSCALE_32BIT;
  canfil.FilterIdHigh = 0x0000;
  canfil.FilterIdLow = 0x0000;
  canfil.FilterMaskIdHigh = 0x0000;
  canfil.FilterMaskIdLow = 0x0000;
  canfil.FilterFIFOAssignment = 0;
  canfil.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(&hcan, &canfil);
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

uint16_t getRawRotation() {
	for (i = 0; i < op_num; i++) {
		__AS5048A2_CS_ENABLE();
		HAL_SPI_TransmitReceive(&hspi1, &SPI_TX_DATA[i], (uint8_t*) &SPI_RX_DATA[i], 1, 2710);
		__AS5048A2_CS_DISABLE();
	}
	return SPI_RX_DATA[3] & 0x3fff;
}

uint32_t sendCAN(uint16_t id, uint8_t buffer[], uint8_t len) {
	CAN_TxHeaderTypeDef header;

	header.IDE = CAN_ID_STD;
	header.RTR = CAN_RTR_DATA;
	header.TransmitGlobalTime = DISABLE;
	header.DLC = len;
	header.StdId = id;

	uint32_t mailbox;

	if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0) {
		uint8_t res = HAL_CAN_AddTxMessage(&hcan, &header, buffer, &mailbox);
		if (res != HAL_OK) {
			SET_ERROR(0);
		} else {
			CLEAR_ERROR(0);
		}
	} else {
		if (HAL_CAN_IsTxMessagePending(&hcan, txMailbox)) {
			SET_ERROR(1);
		}
	}

	return mailbox;
}

uint32_t getRX0Count() {
	return HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0);
}

uint32_t getRX1Count() {
	return HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO1);
}

void readCAN() {
	if (getRX0Count() > 0) {
		CAN_RxHeaderTypeDef tmp;
		uint8_t data[8];
		HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &tmp, data);
		if (tmp.StdId == 0x230 && isDeadFace(data, tmp.DLC)) {
			angle = 0;
		}
		if (tmp.StdId == 0x1) {
			ignition = 1;
			lastSeen = HAL_GetTick();
		}
	}
	if (getRX1Count() > 0) {
		CAN_RxHeaderTypeDef tmp;
		uint8_t data[8];
		HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &tmp, data);
		if (tmp.StdId == 0x230 && isDeadFace(data, tmp.DLC)) {
			angle = 0;
		}
		if (tmp.StdId == 0x1) {
			ignition = 1;
			lastSeen = HAL_GetTick();
		}
	}
}

uint8_t isDeadFace(uint8_t buf[], uint8_t len) {
	if (len != 8) {
		return 0;
	}
	return buf[0] = 0xD && buf[1] == 0xE && buf[2] == 0xA && buf[3] ==0xD
			&& buf[4] == 0xF && buf[5] == 0xA && buf[6] == 0xC && buf[7] == 0xE;
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
