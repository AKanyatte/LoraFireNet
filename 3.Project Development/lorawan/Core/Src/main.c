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

#include "sx1276.h"
#include "polkadot.h"
#include "pk.h"
#include "packet_formats.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PI_NODE 	0
#define DATA_REQ 	55

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint8_t data[16] = "hello world!\r\n";			//USART Transmit Buffer
uint8_t cmd_rx_buf[16];				//receive buffer for mesh commands from Pi
uint8_t cmd_tx_buf[16];
volatile uint8_t tx_len = 5;

volatile unsigned char xdone_flag = 0;
volatile unsigned char rx_flag = 0;
volatile unsigned char spi_flag = 0;
volatile unsigned char spi_rx_tx = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

//prototypes for functions used directly in main
uint8_t spi_transfer(uint8_t data[], uint8_t len);
uint16_t readTemp();

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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  // Reset Lora Module
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);  //activate CS
  HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);  //deactivate CS
  HAL_Delay(10);

  SX1276_Init(SYNC_WORD, 24);  //Initialize the SX1276

  if(PI_NODE){
  	  polkadot_init(4); //the hard-coded address of the Pi Node is 4
    } else {
  	  uint32_t unique_seed = get_UID();	//initialize with a random ID
  	  DEBUG_PRINT("device id: %d\r\n", unique_seed);
  	  srand(unique_seed);
  	  uint32_t new_id;
  	  new_id = rand();
  	  while(new_id < 11){
  		  new_id = rand();
  	  }
  	  DEBUG_PRINT("New ID: %d\r\n", new_id);
  	  polkadot_init(new_id);
  	  rand_delay();
  	  mesh_send_hello();
    }

  SX1276_Start_Receive();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(xdone_flag)
	  {
		  xdone_flag = 0;
		  if(rx_flag)
		  {
			  uint8_t len = SX1276_Read_Register(REG_RX_NB_BYTES); //get packet length
	 		  uint8_t rxbuf[len];
	 		  SX1276_Read_Burst(REG_FIFO, rxbuf, len);
	 		  SX1276_Set_Mode(STDBY);
	 		  DEBUG_PRINT("Entering Packet Handler\r\n");
	 		  receive_packet_handler(rxbuf, len);
	 		  SX1276_Start_Receive();
		  }
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart2.Init.BaudRate = 115200;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 LD3_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_3)
	{
		xdone_flag = 1;
	}

	if(GPIO_Pin == GPIO_PIN_0 && PI_NODE)
	{
		HAL_SPI_TransmitReceive(&hspi1, cmd_tx_buf, cmd_rx_buf, tx_len, 1000);
		while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){}
	}
	if(!spi_rx_tx && PI_NODE){ //!HAL_GPIO_ReadPin(SPI2_AS_GPIO_Port, SPI2_AS_Pin) && PI_NODE){		//wait for AS pin
				spi_flag = 0;
				DEBUG_PRINT("Received command:");
				printarr(cmd_rx_buf, 5);
				DEBUG_PRINT("\r\n");
				if(cmd_rx_buf[0] == DATA_REQ){
					uint32_t node_addr = cmd_rx_buf[1] << 24 | cmd_rx_buf[2] << 16 | cmd_rx_buf[3] << 8 | cmd_rx_buf[4];
					uint8_t data_req_arr[1] = {DATA_REQ};
					DEBUG_PRINT("Requesting data from node #%d\r\n", node_addr);
					mesh_transmit(node_addr, data_req_arr, 1);
				  }
			SX1276_Start_Receive();
		}
}

	//read the temperature sensor for the STM32
	uint16_t readTemp()
	{
	DEBUG_PRINT("\tReading Temperature\r\n");
	uint16_t value = 30;
	return value;}


	//handshaking protocol for communicating with the raspberry pi
	uint8_t spi_transfer(uint8_t data[], uint8_t length){
		DEBUG_PRINT("transferring data to the Pi over SPI\r\n");
		spi_rx_tx = 1;
		for(int i = 0; i < length; i ++){
			cmd_tx_buf[i] = data[i];
		}
		tx_len = length;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	    while (!spi_flag){
	    }
	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	    HAL_SPI_Transmit(&hspi1, data, length,100);
	    HAL_Delay(50);
	    spi_rx_tx = 0;
	    tx_len = 5;
	    return SUCCESS;
	}

	//FUNCTION FOR DOING STUFF WITH DATA
	uint8_t DATA_RX_HANDLER(struct data_packet rx_pkt){
		//Print the received data to the console
		DEBUG_PRINT("Received Data: ");
		DEBUG_PRINT((char *) rx_pkt.packet_data);
		DEBUG_PRINT("\n\r");
		if(PI_NODE){
			uint8_t spi_data[6];
			spi_data[0] = rx_pkt.packet_data[0];
			spi_data[1] = rx_pkt.packet_data[1];
			spi_data[2] = (rx_pkt.source_id & 0xFF);
			spi_data[3] = (rx_pkt.source_id >> 8) & 0xFF;
			spi_data[4] = (rx_pkt.source_id >> 16) & 0xFF;
			spi_data[5] = (rx_pkt.source_id >> 24) & 0xFF;
			spi_transfer(spi_data, 6);
			SX1276_Start_Receive();
		} else {
			if(rx_pkt.packet_data[0] == DATA_REQ){		//reply with my temperature if that was a data request
				uint16_t mytemp = readTemp();
				uint8_t temppack[2] = {(mytemp & 0xFF), (mytemp >> 8)};
				mesh_transmit(4, temppack, 2);
			}
		}
		return SUCCESS;
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
