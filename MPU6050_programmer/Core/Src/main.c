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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR_LSL1	0xD0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c1_rx;

/* USER CODE BEGIN PV */
uint8_t i2c_RX_done = 0;
uint8_t i2c_TX_done = 0;
//uint8_t data_ready = 0; //accelerometer data ready interrupt

uint8_t receive_buffer[20]; //received message buffer, randomly used

uint8_t fifo_count_buffer[2];
uint16_t fifo_count;

int8_t fifo_buffer[20]; //use FIFO to burst read accel and gyro data
int16_t accel_xout;
int16_t accel_yout;
int16_t accel_zout;
int16_t gyro_xout;
int16_t gyro_yout;
int16_t gyro_zout;

uint8_t accel_xout_test_buf[2]; //extracting data without FIFO to compare results with FIFO
uint8_t accel_yout_test_buf[2];
uint8_t accel_zout_test_buf[2];
uint8_t gyro_xout_test_buf[2];
uint8_t gyro_yout_test_buf[2];
uint8_t gyro_zout_test_buf[2];
int16_t accel_xout_test;
int16_t accel_yout_test;
int16_t accel_zout_test;
int16_t gyro_xout_test;
int16_t gyro_yout_test;
int16_t gyro_zout_test;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
HAL_StatusTypeDef i2c_Write_Accelerometer(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t MemAddress, uint8_t *pData, uint16_t len); // <-- Add this line here
HAL_StatusTypeDef i2c_Read_Accelerometer(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t regAddress, uint8_t *pData, uint16_t len);
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
	MX_DMA_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */
	__HAL_DMA_ENABLE(&hdma_i2c1_tx);
	__HAL_DMA_ENABLE(&hdma_i2c1_rx);
	__HAL_DMA_ENABLE_IT(&hdma_i2c1_tx, DMA_IT_TC | DMA_IT_HT);
	__enable_irq();
	//HAL i2c notes:
	//address of MPU6050 device is 1101000, but we shift it to left because the transmit and receive functions require that. So we are left with 0xD0
	//Argument to right of MPU6050_ADDR_LSL1 is the register address, see the register description in onenote.
	uint8_t addr[1];
	/* We compute the MSB and LSB parts of the memory address */
	addr[0] = (uint8_t) (0x6A);
	HAL_Delay(1000); //delay for init functions to see if it stops glitch of i2c transmission never completing
	HAL_StatusTypeDef returnValue = HAL_I2C_Master_Transmit_DMA(&hi2c1, MPU6050_ADDR_LSL1, addr, 1);
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
	if (returnValue != HAL_OK)
	{
		Error_Handler();
	}
	if (__HAL_DMA_GET_FLAG(&hdma_i2c1_tx, (0x00000002U)))
	{  // Transfer error
		printf("DMA Transfer Error\n");
		// Handle error here
	}
	HAL_DMA_IRQHandler(&hdma_i2c1_tx);
	while (!i2c_TX_done);
	i2c_TX_done = 0;
	uint8_t command = 0x00;
	i2c_Write_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x6B, &command, 1); //turn off sleep mode
	//using FIFO to do burst reads on gyroscope and accelerometer

	i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x6B, (uint8_t*) receive_buffer, 1); //PWR_MGMT_1, check if device is asleep, if you see 0x40, it is asleep and every register reads 0

	//Adjust Sample Rate
	i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x1A, (uint8_t*) receive_buffer, 1); //check digital low pass filter settings, Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or 7), and 1kHz when the DLPF is enabled
	i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x19, (uint8_t*) receive_buffer, 1); //check sample rate divider contents
	command = 0x1F; //divide by sample rate divider + 1
	i2c_Write_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x19, (uint8_t*) &command, 1); // adjust sample rate divider
	i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x19, (uint8_t*) receive_buffer, 1); //check sample rate divider contents

	//initialize/enable FIFO
	command = 0x78; //0b01111000
	i2c_Write_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x23, &command, 1); //only enable the gyroscope and accelerometer to be in the FIFO
	i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x23, (uint8_t*) receive_buffer, 1); //FIFO_EN contents

	command = 0x00;
	i2c_Write_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x6A, (uint8_t*) &command, 1); //disable FIFO in USER_CTRL register
	i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x6A, (uint8_t*) receive_buffer, 1); //check USER_CTRL contents, should be 0x00

	command = 0x04;
	i2c_Write_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x6A, (uint8_t*) &command, 1); // reset FIFO, must do it when FIFO_EN is off
	i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x6A, (uint8_t*) receive_buffer, 1); //check USER_CTRL contents, should be 0x00 still

	i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x72, (uint8_t*) receive_buffer, 2); //count items in FIFO

	command = 0x40;
	i2c_Write_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x6A, (uint8_t*) &command, 1); //enable FIFO in USER_CTRL register
	i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x6A, (uint8_t*) receive_buffer, 1); //USER_CTRL should be 0x40 now
	i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x72, (uint8_t*) receive_buffer, 2); //count items in FIFO

	//enable data ready interrupt
	command = 0x11;
	i2c_Write_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x38, &command, 1); //enable fifo overflow and data ready interrupt
	i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x38, (uint8_t*) receive_buffer, 1); //check INT_ENABLE contents default is 0x00 I believe

	i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x3A, (uint8_t*) receive_buffer, 1); //check which interrupt request happened (most important is LSB, DATA_READY_INT)
	i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x3A, (uint8_t*) receive_buffer, 1);

	//check if interrupt is open drain or push pull via INT_OPEN setting
	i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x37, (uint8_t*) receive_buffer, 1); //be default it is 0, push-pull, no pullup and no pulldown needed

	//Gyro Config
	i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x1B, (uint8_t*) receive_buffer, 1); //read GYRO_CONFIG
	command = 0x08;
	i2c_Write_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x1B, &command, 1); // +/- 500 degrees/second
	i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x1B, (uint8_t*) receive_buffer, 1); //read GYRO_CONFIG
	//Accelerometer Config
	i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x1C, (uint8_t*) receive_buffer, 1); //read ACCEL_CONFIG
	command = 0x10;
	i2c_Write_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x1C, &command, 1); // +/- 8g, all self test off
	i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x1C, (uint8_t*) receive_buffer, 1); //read ACCEL_CONFIG

	//Readout contents of sensor registers
	i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x3B, (uint8_t*) receive_buffer, 2); //ACCEL_XOUT, 2 bytes
	i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x3D, (uint8_t*) receive_buffer, 2); //ACCEL_YOUT, 2 bytes
	i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x3F, (uint8_t*) receive_buffer, 2); //ACCEL_ZOUT, 2 bytes

	i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x43, (uint8_t*) receive_buffer, 2); //GYRO_XOUT, 2 bytes
	i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x45, (uint8_t*) receive_buffer, 2); //GYRO_YOUT, 2 bytes
	i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x47, (uint8_t*) receive_buffer, 2); //GYRO_ZOUT, 2 bytes

	i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x74, (uint8_t*) fifo_buffer, 1024); //flush FIFO buffer for data alignment
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

		//read with FIFO, IMPORTANT: must read 1024 bytes from buffer register to flush buffer for data alignment
//		i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x72, (uint8_t*) fifo_count_buffer, 2); //count items in FIFO
//		fifo_count = (fifo_count_buffer[0] << 8) | fifo_count_buffer[1];
//		if (fifo_count < 12)
//		{
//			continue;
//		}
//		i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x74, (uint8_t*) fifo_buffer, 12); //read from FIFO buffer
//		accel_xout = (fifo_buffer[0] << 8) | fifo_buffer[1];
//		accel_yout = (fifo_buffer[2] << 8) | fifo_buffer[3];
//		accel_zout = (fifo_buffer[4] << 8) | fifo_buffer[5];
//		gyro_xout = (fifo_buffer[6] << 8) | fifo_buffer[7];
//		gyro_yout = (fifo_buffer[8] << 8) | fifo_buffer[9];
//		gyro_zout = (fifo_buffer[10] << 8) | fifo_buffer[11];
//
//		i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x3A, (uint8_t*) receive_buffer, 1); //check which interrupt request happened (most important is LSB, DATA_READY_INT)

		//read without FIFO, IMPORTANT: if using interrupt to synchronize, need a series resistor between interrupt pin on sensor and EXTI pin. Helps to form low pass filter to dampen voltage spikes that mess up the i2c bus.
		i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x3B, (uint8_t*) accel_xout_test_buf, 2); //ACCEL_XOUT, 2 bytes
		i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x3D, (uint8_t*) accel_yout_test_buf, 2); //ACCEL_YOUT, 2 bytes
		i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x3F, (uint8_t*) accel_zout_test_buf, 2); //ACCEL_ZOUT, 2 bytes
		i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x43, (uint8_t*) gyro_xout_test_buf, 2); //GYRO_XOUT, 2 bytes
		i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x45, (uint8_t*) gyro_yout_test_buf, 2); //GYRO_YOUT, 2 bytes
		i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x47, (uint8_t*) gyro_zout_test_buf, 2); //GYRO_ZOUT, 2 bytes
		accel_xout_test = (accel_xout_test_buf[0] << 8) | accel_xout_test_buf[1];
		accel_yout_test = (accel_yout_test_buf[0] << 8) | accel_yout_test_buf[1];
		accel_zout_test = (accel_zout_test_buf[0] << 8) | accel_zout_test_buf[1];
		gyro_xout_test = (gyro_xout_test_buf[0] << 8) | gyro_xout_test_buf[1];
		gyro_yout_test = (gyro_yout_test_buf[0] << 8) | gyro_yout_test_buf[1];
		gyro_zout_test = (gyro_zout_test_buf[0] << 8) | gyro_zout_test_buf[1];

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
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit =
	{ 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
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
	hi2c1.Init.Timing = 0x0010020A;
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
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	/* DMA1_Channel3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct =
	{ 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin : PB4 */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
HAL_StatusTypeDef i2c_Read_Accelerometer(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t regAddress, uint8_t *pData, uint16_t len)
{
	HAL_StatusTypeDef returnValue;
	uint8_t addr[1];

	/* We compute the MSB and LSB parts of the memory address */
	addr[0] = (uint8_t) (regAddress);

	/* First we send the memory location address where start reading data */
	returnValue = HAL_I2C_Master_Seq_Transmit_DMA(hi2c, DevAddress, addr, 1, I2C_FIRST_FRAME);
//	while (!i2c_TX_done);
//	i2c_TX_done = 0;
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
	//	uint32_t i2c_error = HAL_I2C_GetError(&hi2c1);
	/* Next we can retrieve the data from EEPROM */
	returnValue = HAL_I2C_Master_Seq_Receive_DMA(hi2c, DevAddress, pData, len, I2C_LAST_FRAME);
	while (!i2c_RX_done);
	i2c_RX_done = 0;
	return returnValue;
}

HAL_StatusTypeDef i2c_Write_Accelerometer(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t regAddress, uint8_t *pData, uint16_t len)
{
	HAL_StatusTypeDef returnValue;
	uint8_t *data;

	data = (uint8_t*) malloc(sizeof(uint8_t));
	/*We compute the MSB and LSB parts of the memory address*/
	data[0] = (uint8_t) (regAddress);

	/*And copy the content of the pData array in the temporary buffer*/
	memcpy(data + 1, pData, len); //inserts data one slot after the register address

	/*We are now ready to transfer the buffer over the I2C bus*/
	returnValue = HAL_I2C_Master_Transmit_DMA(hi2c, DevAddress, data, len + 1);
	while (!i2c_TX_done);
	i2c_TX_done = 0;
	free(data);
	/*We wait until the Accelerometer effectively stores data*/
	while (HAL_I2C_IsDeviceReady(hi2c, DevAddress, 1, HAL_MAX_DELAY) != HAL_OK); //peripheral can only accept the transmission once it finishes doing what it does

	return HAL_OK;
}
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	i2c_TX_done = 1;
}
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	i2c_RX_done = 1;
}
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//	if (GPIO_Pin == GPIO_PIN_4)
//	{
//		data_ready = 1;
//	}
//}
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
