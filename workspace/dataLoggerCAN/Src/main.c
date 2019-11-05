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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* xTaskCreate defines */
#define STACK_SIZE_TASK_LED       200
#define STACK_SIZE_TASK_LOG       800
#define STACK_SIZE_TASK_HANDLECAN 400

#define PRIORITY_TASK_LED         1
#define PRIORITY_TASK_LOG         2
#define PRIORITY_TASK_HANDLECAN   4

#define ID_CAN_GPS                0x500
#define MAX_SIZE_MESSAGE_CAN      45
#define SIZE_CAN_MESSAGE_BUFFER   15

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* CAN handler */
CAN_HandleTypeDef hcan1;

/* SD card handles */
SD_HandleTypeDef  hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

/* USER CODE BEGIN PV */

/* FreeRTOS task handles */
TaskHandle_t taskLedHandle;
TaskHandle_t taskLogHandle;
TaskHandle_t taskHandleCANHandle;

/* queue for CAN messages */
QueueHandle_t xQueueCAN;

/* variables for CAN bus configuration */
CAN_FilterTypeDef 	sFilterConfig;
CAN_RxHeaderTypeDef	RxHeader;
uint8_t 			RxData[8];

/* CAN message struct */
struct canMessage {
	uint32_t id;
	uint8_t  data[8];
	uint32_t timestamp;
};


/* struct for datetime */
struct nowDateTime {
	uint32_t day;
	uint32_t month;
	uint32_t year;
	uint32_t hour;
	uint32_t minute;

};

/* flag to control log file creation */
_Bool flagDateSet = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* task for debugging, just a blink LED */
void taskLed(void *pvParameters);

/* task to save CAN messages to SD card */
void taskLog(void *pvParameters);

/* task to read CAN messages (polling) */
void taskHandleCAN(void *pvParameters);

/* function to control whether there was an error with SD card or not */
void sdCardError(FRESULT f_status, uint8_t location);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* config CAN filter */
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment	= CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
    /* Filter configuration Error */
    Error_Handler();
  }

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 2;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GREEN_LED_Pin|RED_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PG4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : GREEN_LED_Pin RED_LED_Pin */
  GPIO_InitStruct.Pin = GREEN_LED_Pin|RED_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
	/* reset of all peripherals, initializes the flash interface and the Systick. */
	HAL_Init();

	/* configure the system clock */
	SystemClock_Config();

	/* initialize all configured peripherals */
	MX_DMA_Init();
	MX_GPIO_Init();
	MX_CAN1_Init();
	MX_SDIO_SD_Init();

	/* initialize interrupts */
	MX_NVIC_Init();

	/* try to create queue */
	xQueueCAN = xQueueCreate(50, sizeof(struct canMessage));
	if (xQueueCAN == NULL) {
		/* queue was not created and must not be used. */
	}

	/* create tasks */
//	xTaskCreate (
//		taskLed, 					// function
//		(signed char *) "TaskLed", 	// name
//		STACK_SIZE_TASK_LED,		// stack size
//		(void *) NULL,				// pvParameters
//		PRIORITY_TASK_LED,			// priority
//		&taskLedHandle				// taskHandle
//	);

	xTaskCreate (
		taskLog, 					// function
		(signed char *) "TaskLog", 	// name
		STACK_SIZE_TASK_LOG,		// stack size
		(void *) NULL,				// pvParameters
		PRIORITY_TASK_LOG,			// priority
		&taskLogHandle				// taskHandle
	);

	xTaskCreate (
		taskHandleCAN, 					// function
		(signed char *) "TaskHadleCAN",	// name
		STACK_SIZE_TASK_HANDLECAN,		// stack size
		(void *) NULL,					// pvParameters
		PRIORITY_TASK_HANDLECAN,		// priority
		&taskHandleCANHandle			// taskHandle
	);

	/* start the scheduler */
	vTaskStartScheduler();

	/* infinite loop */
	for (;;);
}

void taskLed(void *pvParameters) {
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t timeMiliseconds = 1000 / portTICK_PERIOD_MS;

	for (;;) {
		/* red LED for debugging */
		HAL_GPIO_TogglePin(GPIOG, GREEN_LED_Pin);

		vTaskDelayUntil(&xLastWakeTime, timeMiliseconds);
	}
}

void taskHandleCAN (void *pvParameters) {
	uint32_t messagesInFifo;
	HAL_StatusTypeDef status;
	struct canMessage message;

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (HAL_CAN_Start(&hcan1) != HAL_OK) {
		/* start error */
		Error_Handler();
	}

	for (;;) {
		/* waits for a message in CAN bus */
		do {
			messagesInFifo = HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0);
		} while (messagesInFifo == 0);

		/* read CAN message */
		status = HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);

		/* case message is OK */
		if (status == HAL_OK) {
			/* save data to CAN message struct */
			message.id = RxHeader.StdId;
			memcpy(message.data, RxData, sizeof(RxData));
			message.timestamp = xTaskGetTickCount();

			/* add message to queue */
			xQueueSend(xQueueCAN, (const void *) &message, &xHigherPriorityTaskWoken);
		}
	}
}

void taskLog (void *pvParameters) {
	/* FatFS variables */
	FRESULT f_status;
	MX_FATFS_Init();

	/* log file values */
	char folderPath[9]; // yy-mm-dd
	char fileName[15];   // hh-mm

	/* structs */
	struct canMessage message;
	struct nowDateTime logDateTime;

	/* delta time calculation variables */
	uint32_t timeInit;
	uint32_t timeToLog;

	/* init buffer for messages to be logged with empty values */
	char messageToLog[SIZE_CAN_MESSAGE_BUFFER * MAX_SIZE_MESSAGE_CAN];
	memset(messageToLog, 0, sizeof(messageToLog));

	/* number of messages to log */
	uint8_t numberMessagesToLog = 0;

	/* try to mount SD card */
	f_status = f_mount(&SDFatFS, (char const*)SDPath, 1);
	if (f_status != FR_OK) {
		sdCardError(f_status, 0);
	}

	for (;;) {
		/* waits for a message in the queue */
		xQueueReceive(xQueueCAN, (void *) &message, portMAX_DELAY);

		/* if log file is already created */
		if (flagDateSet) {
			timeToLog = (message.timestamp - timeInit) * portTICK_PERIOD_MS;

			if ((int)timeToLog > 0) {
				/* append new message to buffer */
				sprintf(messageToLog, "%s%x;%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x;%d\n",
					messageToLog,
					message.id,
					message.data[0],
					message.data[1],
					message.data[2],
					message.data[3],
					message.data[4],
					message.data[5],
					message.data[6],
					message.data[7],
					timeToLog
				);

				/* number of messages in the buffer */
				numberMessagesToLog++;

				/* if buffer is full */
				if (numberMessagesToLog == SIZE_CAN_MESSAGE_BUFFER) {
					/* open file */
					f_status = f_open(&SDFile, fileName, FA_OPEN_ALWAYS | FA_WRITE);

					if (f_status == FR_OK) {
						/* set pointer to the end of the file */
						f_status = f_lseek(&SDFile, f_size(&SDFile));
						if (f_status != FR_OK) {
							sdCardError(f_status, 5);
						}

						/* write message in log file */
						if (f_puts(messageToLog, &SDFile) > 0);

						/* wait until file is closed */
						do {
							f_status = f_close(&SDFile);
						} while (f_status != FR_OK);
					}

					else {
						sdCardError(f_status, 4);
					}

					/* reset buffer */
					memset(messageToLog, 0, sizeof(messageToLog));
					numberMessagesToLog = 0;

					/* green LED for debugging */
					HAL_GPIO_TogglePin(GPIOG, GREEN_LED_Pin);
				}
			}
		}

		/* waits for a GPS message, in order to get the date for the file name */
		else if ((message.id == ID_CAN_GPS) && !(flagDateSet)) {
			/* struct with date and time */
			logDateTime.day = message.data[0];
			logDateTime.month = message.data[1];
			logDateTime.year = message.data[2];

			logDateTime.hour = message.data[3];
			logDateTime.minute = message.data[4];

			/* create folder in SD card, if does not exist */
			sprintf(folderPath, "%02d-%02d-%02d",
				logDateTime.year,
				logDateTime.month,
				logDateTime.day
			);
			f_status = f_mkdir(folderPath);
			if ((f_status != FR_OK) && (f_status != FR_EXIST)) {
				sdCardError(f_status, 1);
			}

			/* create new file in SD card's folder */
			sprintf(fileName, "%s/%02d-%02d.txt",
				folderPath,
				logDateTime.hour,
				logDateTime.minute
			);
			f_status = f_open(&SDFile, fileName, FA_CREATE_ALWAYS | FA_WRITE);
			if (f_status != FR_OK) {
				sdCardError(f_status, 2);
			}

			/* close file */
			f_status = f_close(&SDFile);
			if (f_status != FR_OK) {
				sdCardError(f_status, 3);
			}

			/* if date and time are set, does not enter here anymore */
			flagDateSet = 1;

			/* get initial timestamp for delta time calculation */
			timeInit = xTaskGetTickCount();
		}
	}

	/* unmount drive, don't forget this! */
	f_mount(0, "", 1);
}

void sdCardError(FRESULT f_status, uint8_t location) {
	/* red LED for debugging */
	HAL_GPIO_TogglePin(GPIOG, RED_LED_Pin);

	while (1);
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
