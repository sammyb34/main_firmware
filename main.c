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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include <stdlib.h>
#include "Uart_manager.h"
#include "UartRingBuffer_multi.h"
#include "SDCardStuff.h"


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
 UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart4;

SD_HandleTypeDef hsd1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART4_Init(void);
static void MX_LPUART1_UART_Init(void);
/* USER CODE BEGIN PFP */

void SD_PWRMGMT(int state);
void XSENS_PWRMGMT(int state);
void XSENS_INIT();

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
	char tx_buf[20];
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  UART_Init();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SDMMC1_SD_Init();
  MX_SPI1_Init();
  MX_UART4_Init();
  MX_LPUART1_UART_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  // Turn off XSENS and SD Card (fix these so its just set low instead of toggle)
  SD_PWRMGMT(0);
  XSENS_PWRMGMT(0);

  // Create some variables for main
  int a = 1;			// dummy variable for a while loop
  float UTC = 0;			// Unix time
  int TotalData = 0;	// Total number of data points collected in this 20 minute interval so far
  int NewData = 0;			// number of new data points since last write to the SD card
  int buffer = 0;		// Variable for storing number of data points in XSENS buffer
  int restart = 0;

  // Allocate dynamic arrays for
  // parameters collected by XSENS. (11)
  // Length 120 in case there's extra data.
  // Should only get up to 100 values each
  float* ax = calloc(120, sizeof(float));
  float* ay = calloc(120, sizeof(float));
  float* az = calloc(120, sizeof(float));
  float* q1 = calloc(120, sizeof(float));
  float* q2 = calloc(120, sizeof(float));
  float* q3 = calloc(120, sizeof(float));
  float* q4 = calloc(120, sizeof(float));
  float* mx = calloc(120, sizeof(float));
  float* my = calloc(120, sizeof(float));
  float* mz = calloc(120, sizeof(float));
  float* timestamp = calloc(120, sizeof(float));

  float ParamCSV[15];

  /* USER CODE END 2 */


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {

	  //poll for serial until receiving "set time %lf\r" where %lf = unix time
	  while (a) {
		  // if receive the aforementioned serial message, store UTC
		  UTC = UART_TimeCheck();
		  if (UTC != 0) {

		  	  // send ack "TIME SET:\r"
		  	  sprintf(tx_buf, "TIME SET:\r\n");
		  	  tx_string(&huart4, tx_buf);

		  	  // Power on XSENS and SD card
		  	  SD_PWRMGMT(1);
		  	  XSENS_PWRMGMT(1);

			  // Wait a tiny bit so the SD or XSENS don't get confused, having just been born.
			  HAL_Delay(20);

		  	  // Init SD card with UTC for filename
		  	  initFileSD(UTC);

		  	  // Power off SD Card
		  	  SD_PWRMGMT(0);

		  	  // Init XSENS with UTC
		  	  XSENS_INIT();

		  	  // break loop
		  	  a = 0;

		  } // End If
	  } // End While


	  // Read from XSENS
	  if (data_ready_xsens(200)) {
		  get_packet_xsens(uint8_t* rx_buf, timestamp, q1, q2, q3, q4, ax, ay, az, mx, my, mz);
		  NewData++;

		  // Turn on SD card now so that it has time to boot up while checking serial
		  if (NewData >= 10) {
			  SD_PWRMGMT(1);
		  }
	  }

	  // poll for serial to wait for message "restart\r"
	  restart = UART_RestartCheck();

	  // If there is 1 second of data (10 pieces), store to SD card. Should only take 50ms
	  if (NewData >= 10 || restart == 1) {
		  writeSD(UTC, ax, ay, az, q1, q2, q3, q4, mx, my, mz, timestamp, NewData);
		  NewData = 0;
		  SD_PWRMGMT(0);

	  }

/*	  // if there are 10 data points in the XSENS Buffer
	  buffer = XSENS_Buffer();
	  if (buffer >= 10) {
		  // Grab contents of buffer and append it to data0, data1, data2 etc....
		  XSENS_Read(NewData, 120, AccX, AccY, AccZ, Orien1, Orien2, Orien3, Orien4, MagX, MagY, MagZ, TimeStamp);
		  NewData += buffer;
	  }


	  // After NewData >= 100, store all the gathered XSENS data to the SD card
	  if (NewData >= 100) {
		  //Add number of new Data (~100) to TotalData
		  TotalData += NewData;

	  	  //turn on SD card
		  SD_PWRMGMT(1);

		  //Wait a tiny bit so the SD doesn't get confused, having just been born.
		  HAL_Delay(20);

	  	  //store data
		  writeSD(UTC, AccX, AccY, AccZ, Orien1, Orien2, Orien3, Orien4, MagX, MagY, MagZ, TimeStamp, TotalData);

	  	  //turn off SD card
		  SD_PWRMGMT(0);
	  }
*/


	  // if Serial has received restart\r
	  if(restart) {
  	  	  // Power off XSENS
  	  	  XSENS_PWRMGMT(0);

	  	  // dump remaining data (free parameter arrays). Up to 10 seconds of data out of 20 mins doesnt matter that much
	  	  // and we need to be quick here so that the glider can dive sooner and we don't hit timeout.
  	  	  free(ax);
  	  	  free(ay);
		  free(az);
		  free(q1);
		  free(q2);
		  free(q3);
		  free(q4);
		  free(mx);
		  free(my);
		  free(mz);
		  free(timestamp);

	  	  // Power on SD Card
	  	  SD_PWRMGMT(1);

	  	  // Run Data analysis function
	  	  DataAnalysis(ParamCSV);

		  // Write CSV to SD Card
	  	  writeColSD(UTC, ParamCSV, 13);

	  	  // Send CSV over serial
	  	  // tx_string(&huart4, tx_buf);

	  } // End If


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  } // End Main While




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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_7B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 32;
  hsd1.Init.Transceiver = SDMMC_TRANSCEIVER_DISABLE;
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

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
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, SD_MGMT_Pin|XSENS_MGMT_Pin|XSENS_RST_Pin|DRDY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SD_MGMT_Pin XSENS_MGMT_Pin XSENS_RST_Pin DRDY_Pin */
  GPIO_InitStruct.Pin = SD_MGMT_Pin|XSENS_MGMT_Pin|XSENS_RST_Pin|DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void XSENS_INIT() {
  uint8_t version;
  uint8_t data_cfg;

  if(!cfg_protocol_xsens(&version, &data_cfg)) {
	  sprintf((char*)uart_tx_buf, "Config Failed!\r\n");
	  HAL_UART_Transmit(&hlpuart1, uart_tx_buf, strlen((char*)uart_tx_buf), HAL_MAX_DELAY);
  }

  sprintf((char*)uart_tx_buf, "Version: %x CFG: %x\r\n", version, data_cfg);
  HAL_UART_Transmit(&hlpuart1, uart_tx_buf, strlen((char*)uart_tx_buf), HAL_MAX_DELAY);

  int reset_attempts = 0;

  while(!reset_xsens()) {
	  HAL_Delay(100);
	  reset_attempts++;
	  if (reset_attempts > 5) {
		  sprintf((char*)uart_tx_buf, "Reset Timeout Failed!\r\n");
		  HAL_UART_Transmit(&hlpuart1, uart_tx_buf, strlen((char*)uart_tx_buf), HAL_MAX_DELAY);
		  break;
	  }
  }

  int config_state_attempts = 0;
  while(!goto_config_xsens()) {
	  HAL_Delay(500);
	  config_state_attempts++;
	  if (config_state_attempts > 5) {
		  sprintf((char*)uart_tx_buf, "Config State Timeout Failed!\r\n");
		  HAL_UART_Transmit(&hlpuart1, uart_tx_buf, strlen((char*)uart_tx_buf), HAL_MAX_DELAY);
		  break;
	  }
  }

  int config_attempts = 0;
  while(!config_xsens()) {
		  HAL_Delay(500);
		  config_attempts++;
		  if (config_attempts > 5) {
			  sprintf((char*)uart_tx_buf, "Config Timeout Failed!\r\n");
			  HAL_UART_Transmit(&hlpuart1, uart_tx_buf, strlen((char*)uart_tx_buf), HAL_MAX_DELAY);
			  break;
		  }
	}

  int measurement_attempts = 0;
	while(!goto_measurement_xsens()) {
	  HAL_Delay(500);
	  measurement_attempts++;
	  if (measurement_attempts > 5) {
		  sprintf((char*)uart_tx_buf, "Measurement Timeout Failed!\r\n");
		  HAL_UART_Transmit(&hlpuart1, uart_tx_buf, strlen((char*)uart_tx_buf), HAL_MAX_DELAY);
		  break;
	  }
  }

}

// SD Power Management Function. Send state = 0 for off or state = 1 for on.
void SD_PWRMGMT(int state) {
	if (state == 0) {
		HAL_GPIO_WritePin(GPIOE, 10, GPIO_PIN_RESET);

	} else {
		HAL_GPIO_WritePin(GPIOE, 10, GPIO_PIN_SET);

	}

}

// XSENS Power Management Function. Send state = 0 for off or state = 1 for on.
void XSENS_PWRMGMT(int state) {
	if (state == 0) {
		HAL_GPIO_WritePin(GPIOE, 11, GPIO_PIN_RESET);

	} else {
		HAL_GPIO_WritePin(GPIOE, 11, GPIO_PIN_SET);

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
