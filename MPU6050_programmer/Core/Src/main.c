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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
HAL_StatusTypeDef i2c_Write_Accelerometer(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t MemAddress, uint8_t *pData, uint16_t len);  // <-- Add this line here
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	//const char wmsg[]="WeloveSTM32!";

  	uint8_t angle_data[] = {0x00, 0x00}; //Angle data buffer, read angle data into this buffer and also write to registers with this buffer for programming
  	uint8_t rmsg[20]; //received message buffer, randomly used


	//HAL i2c notes:
	//address of MPU6050 device is 1101000, but we shift it to left because the transmit and receive functions require that. So we are left with 0xD0
	//Argument to right of MPU6050_ADDR_LSL1 is the register address, see the register description in onenote.


  	//Readout contents of registers
  	i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x6A, (uint8_t*)rmsg, 1); //USER_CTRL

	i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x3B, (uint8_t*)rmsg, 2); //ACCEL_XOUT, 2 bytes
	i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x3D, (uint8_t*)rmsg, 2); //ACCEL_YOUT, 2 bytes
	i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x3F, (uint8_t*)rmsg, 2); //ACCEL_ZOUT, 2 bytes

	i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x43, (uint8_t*)rmsg, 2); //GYRO_XOUT, 2 bytes
	i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x45, (uint8_t*)rmsg, 2); //GYRO_YOUT, 2 bytes
	i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x47, (uint8_t*)rmsg, 2); //GYRO_ZOUT, 2 bytes


//	//Angle Programming through I2C interface (Option A in datasheet)
//	//Run in debug mode, make sure to do the 3.3V wiring in fig.  38
//	//Turn angle to starting position
//	i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x0C, (uint8_t*)angle_data, 2); //Read From starting position Raw Angle, store result in first two bytes of angle_data
//	i2c_Write_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x01, (uint8_t*)angle_data, 2); //Write the starting position raw angle into ZPOS register
//	//Now rotate the magnet to the stop position
//	i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x0C, (uint8_t*)angle_data, 2); //Read From starting position Raw Angle, store result in first two bytes of angle_data
//	i2c_Write_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x03, (uint8_t*)angle_data, 2); //Write the starting position raw angle into MPOS register
//	//Burning the settings to "permanently" program the magnetic encoder.
//	uint8_t command_buffer[] = {0x00, 0x00, 0x00};
//	command_buffer[0] = 0x80; //BURN_ANGLE command
//	i2c_Write_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0xFF, (uint8_t*)command_buffer, 1); //Perform a BURN_ANGLE command by writing 0x80 value into 0xFF register
//	//Verify burn command
//	memcpy(command_buffer, ((uint8_t[3]){0x01, 0x11, 0x10}), 3 * sizeof(uint8_t)); //load the buffer with 3 commands, replaces the entire buffer
//	i2c_Write_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0xFF, (uint8_t*)command_buffer, 3); //Writing these 3 commands sequentially into 0xFF register to load actual OTP non-volatile memory content
//	//Read ZPOS and MPOS registers to verify the BURN_ANGLE command was successful
//	i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x01, (uint8_t*)rmsg, 2); //ZPOS, starting pos
//	i2c_Read_Accelerometer(&hi2c1, MPU6050_ADDR_LSL1, 0x03, (uint8_t*)rmsg, 2); //MPOS, stop pos
//	//Might want to do power up cycle and reread the ZPOS and MPOS registers again to make sure they match with new positions



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
HAL_StatusTypeDef i2c_Read_Accelerometer(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t regAddress, uint8_t *pData, uint16_t len) {
  HAL_StatusTypeDef returnValue;
  uint8_t addr[1];

  /* We compute the MSB and LSB parts of the memory address */
  addr[0] = (uint8_t) (regAddress);

  /* First we send the memory location address where start reading data */
  returnValue = HAL_I2C_Master_Transmit(hi2c, DevAddress, addr, 1, HAL_MAX_DELAY);
  if(returnValue != HAL_OK)
    return returnValue;

  /* Next we can retrieve the data from EEPROM */
  returnValue = HAL_I2C_Master_Receive(hi2c, DevAddress, pData, len, HAL_MAX_DELAY);

  return returnValue;
}

HAL_StatusTypeDef i2c_Write_Accelerometer(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t regAddress, uint8_t *pData, uint16_t len)
{
	HAL_StatusTypeDef returnValue;
	uint8_t *data;

	data = (uint8_t*)malloc(sizeof(uint8_t));
	/*We compute the MSB and LSB parts of the memory address*/
	data[0]=(uint8_t)(regAddress);

	/*And copy the content of the pData array in the temporary buffer*/
	memcpy(data+1,pData,len); //inserts data one slot after the register address

	/*We are now ready to transfer the buffer over the I2C bus*/
	returnValue=HAL_I2C_Master_Transmit(hi2c,DevAddress,data,len+1,HAL_MAX_DELAY);
	if(returnValue!=HAL_OK)
		return returnValue;

	free(data);
	/*We wait until the EEPROM effectively stores data in memory*/
	while(HAL_I2C_Master_Transmit(hi2c,DevAddress,0,0,HAL_MAX_DELAY)!=HAL_OK);//peripheral can only accept the transmission once it finishes doing what it does

	return HAL_OK;
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
