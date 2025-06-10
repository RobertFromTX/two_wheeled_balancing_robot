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
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{	//struct to extract and organize sensor readings. Interestingly enough, having the struct seems to make code
//faster because these variables are in contiguous blocks of memory.

//buffers to extract data from sensor with i2c interface
	uint8_t accel_x_buf[2]; //extracting data without FIFO to compare results with FIFO
	uint8_t accel_y_buf[2];
	uint8_t accel_z_buf[2];
	uint8_t gyro_x_buf[2];
	uint8_t gyro_y_buf[2];
	uint8_t gyro_z_buf[2];

	//raw data from mpu6050 sensor, every 4096 counts is 1g when using +/- 8g mode (which we are using for this project in particular).
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;

	//processed data calculated from raw values
	//euler angles in degrees
	float yaw;
	float pitch;
	float roll;
} mpu6050_sensor;
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
HAL_StatusTypeDef i2c_Write_Accelerometer(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t MemAddress, uint8_t *pData, uint16_t len); // <-- Add this line here
HAL_StatusTypeDef i2c_Read_Accelerometer(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t regAddress, uint8_t *pData, uint16_t len);
void mpu6050_init(I2C_HandleTypeDef *hi2c);
void mpu6050_get_raw_measurements(I2C_HandleTypeDef *hi2c, mpu6050_sensor *sensor);
void calc_accelerometer_tilt(mpu6050_sensor *sensor);
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

//	__HAL_DMA_ENABLE(&hdma_i2c1_tx);
//	__HAL_DMA_ENABLE(&hdma_i2c1_rx);
//	__HAL_DMA_ENABLE_IT(&hdma_i2c1_tx, DMA_IT_TC | DMA_IT_HT);
//	__enable_irq();
	//HAL i2c notes:
	//address of MPU6050 device is 1101000, but we shift it to left because the transmit and receive functions require that. So we are left with 0xD0
	//Argument to right of MPU6050_ADDR_LSL1 is the register address, see the register description in onenote.
	uint8_t reg_addr[1];
	/* We compute the MSB and LSB parts of the memory address */
	reg_addr[0] = (uint8_t) (0x6A);
	HAL_Delay(1000); //delay for init functions to see if it stops glitch of i2c transmission never completing
	HAL_StatusTypeDef returnValue = HAL_I2C_Master_Transmit_DMA(&hi2c1, MPU6050_ADDR_LSL1, reg_addr, 1);
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

	mpu6050_init(&hi2c1); //write to registers in mpu6050 to configure initial settings
	mpu6050_sensor sensor1;

	//define starting position
	sensor1.yaw = 0;
	sensor1.pitch = 0;
	sensor1.roll = 0;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

		//read without FIFO, IMPORTANT: if using interrupt to synchronize, need a series resistor between interrupt pin on sensor and EXTI pin. Helps to form low pass filter to dampen voltage spikes that mess up the i2c bus and probably more importantly decrease current that could drive SDA pin low.
		mpu6050_get_raw_measurements(&hi2c1, &sensor1);
		calc_accelerometer_tilt(&sensor1);
		//get tilt measurement from accelerometer, pitch and roll only

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
	uint8_t reg_addr[1];

	/* We compute the MSB and LSB parts of the memory address */
	reg_addr[0] = (uint8_t) (regAddress);

	/* First we send the memory location address where start reading data */
	returnValue = HAL_I2C_Master_Seq_Transmit_DMA(hi2c, DevAddress, reg_addr, 1, I2C_FIRST_FRAME);
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

	data = (uint8_t*) malloc(sizeof(uint8_t) * (1 + len));
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

void mpu6050_init(I2C_HandleTypeDef *hi2c)
{
	uint8_t command = 0x00;
	i2c_Write_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x6B, &command, 1); //turn off sleep mode
	//using FIFO to do burst reads on gyroscope and accelerometer

	i2c_Read_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x6B, (uint8_t*) receive_buffer, 1); //PWR_MGMT_1, check if device is asleep, if you see 0x40, it is asleep and every register reads 0

	//Adjust Sample Rate
	i2c_Read_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x1A, (uint8_t*) receive_buffer, 1); //check digital low pass filter settings, Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or 7), and 1kHz when the DLPF is enabled
	i2c_Read_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x19, (uint8_t*) receive_buffer, 1); //check sample rate divider contents
	command = 0x1F; //divide by sample rate divider + 1
	i2c_Write_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x19, (uint8_t*) &command, 1); // adjust sample rate divider
	i2c_Read_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x19, (uint8_t*) receive_buffer, 1); //check sample rate divider contents

	//initialize/enable FIFO
	command = 0x78; //0b01111000
	i2c_Write_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x23, &command, 1); //only enable the gyroscope and accelerometer to be in the FIFO
	i2c_Read_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x23, (uint8_t*) receive_buffer, 1); //FIFO_EN contents

	command = 0x00;
	i2c_Write_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x6A, (uint8_t*) &command, 1); //disable FIFO in USER_CTRL register
	i2c_Read_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x6A, (uint8_t*) receive_buffer, 1); //check USER_CTRL contents, should be 0x00

	command = 0x04;
	i2c_Write_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x6A, (uint8_t*) &command, 1); // reset FIFO, must do it when FIFO_EN is off
	i2c_Read_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x6A, (uint8_t*) receive_buffer, 1); //check USER_CTRL contents, should be 0x00 still

	i2c_Read_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x72, (uint8_t*) receive_buffer, 2); //count items in FIFO

	command = 0x40;
	i2c_Write_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x6A, (uint8_t*) &command, 1); //enable FIFO in USER_CTRL register
	i2c_Read_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x6A, (uint8_t*) receive_buffer, 1); //USER_CTRL should be 0x40 now
	i2c_Read_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x72, (uint8_t*) receive_buffer, 2); //count items in FIFO

	//enable data ready interrupt
	command = 0x11;
	i2c_Write_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x38, &command, 1); //enable fifo overflow and data ready interrupt
	i2c_Read_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x38, (uint8_t*) receive_buffer, 1); //check INT_ENABLE contents default is 0x00 I believe

	i2c_Read_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x3A, (uint8_t*) receive_buffer, 1); //check which interrupt request happened (most important is LSB, DATA_READY_INT)
	i2c_Read_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x3A, (uint8_t*) receive_buffer, 1);

	//check if interrupt is open drain or push pull via INT_OPEN setting
	i2c_Read_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x37, (uint8_t*) receive_buffer, 1); //be default it is 0, push-pull, no pullup and no pulldown needed

	//Gyro Config
	i2c_Read_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x1B, (uint8_t*) receive_buffer, 1); //read GYRO_CONFIG
	command = 0x08;
	i2c_Write_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x1B, &command, 1); // +/- 500 degrees/second
	i2c_Read_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x1B, (uint8_t*) receive_buffer, 1); //read GYRO_CONFIG
	//Accelerometer Config
	i2c_Read_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x1C, (uint8_t*) receive_buffer, 1); //read ACCEL_CONFIG
	command = 0x10;
	i2c_Write_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x1C, &command, 1); // +/- 8g, all self test off
	i2c_Read_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x1C, (uint8_t*) receive_buffer, 1); //read ACCEL_CONFIG
}

void mpu6050_get_raw_measurements(I2C_HandleTypeDef *hi2c, mpu6050_sensor *sensor)
{
	//get raw data from mpu6050 with i2c interface
	//read without FIFO, IMPORTANT: if using interrupt to synchronize, need a series resistor between interrupt pin on sensor and EXTI pin. Helps to form low pass filter to dampen voltage spikes that mess up the i2c bus and probably more importantly decrease current that could drive SDA pin low.
	i2c_Read_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x3B, (uint8_t*) sensor->accel_x_buf, 2); //ACCEL_XOUT, 2 bytes
	i2c_Read_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x3D, (uint8_t*) sensor->accel_y_buf, 2); //ACCEL_YOUT, 2 bytes
	i2c_Read_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x3F, (uint8_t*) sensor->accel_z_buf, 2); //ACCEL_ZOUT, 2 bytes
	i2c_Read_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x43, (uint8_t*) sensor->gyro_x_buf, 2); //GYRO_XOUT, 2 bytes
	i2c_Read_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x45, (uint8_t*) sensor->gyro_y_buf, 2); //GYRO_YOUT, 2 bytes
	i2c_Read_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x47, (uint8_t*) sensor->gyro_z_buf, 2); //GYRO_ZOUT, 2 bytes
	sensor->accel_x = (sensor->accel_x_buf[0] << 8) | sensor->accel_x_buf[1];
	sensor->accel_y = (sensor->accel_y_buf[0] << 8) | sensor->accel_y_buf[1];
	sensor->accel_z = (sensor->accel_z_buf[0] << 8) | sensor->accel_z_buf[1];
	sensor->gyro_x = (sensor->gyro_x_buf[0] << 8) | sensor->gyro_x_buf[1];
	sensor->gyro_y = (sensor->gyro_y_buf[0] << 8) | sensor->gyro_y_buf[1];
	sensor->gyro_z = (sensor->gyro_z_buf[0] << 8) | sensor->gyro_z_buf[1];
}

void calc_accelerometer_tilt(mpu6050_sensor *sensor)
{
	sensor->pitch = asin((float) sensor->accel_x / (float) 4096); //get pitch in radians
	sensor->roll = asin(((float) -sensor->accel_y) / (4096 * cos(sensor->pitch))) * (180 / M_PI); //get roll and convert to degrees
	sensor->pitch = sensor->pitch * (180 / M_PI); //convert to degrees after using it pitch in previous calculation as radians
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
