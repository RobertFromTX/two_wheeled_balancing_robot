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
#include <stm32f3xx_hal_cortex.h>
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{ //struct to extract and organize sensor readings. Interestingly enough, having the struct seems to make code
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

	//processed data calculated from raw values, not as accurate as the kalman_angle from kalman filter
	float32_t yaw; //euler angles in degrees
	float32_t pitch;
	float32_t roll;

	float32_t wx; //angular velocities
	float32_t wy;
	float32_t wz;
} mpu6050_sensor_data;

typedef struct
{
	//struct to store data and calculations used in kalman filter

	//state space matrices
	float32_t matrix_A[4][4]; //state transition matrix
	float32_t matrix_B[4][1];
	float32_t matrix_xp[4][1];	//euler parameters (quaternion), the predicted state of the system
	float32_t matrix_x[4][1]; //state of kalman filter, used to output final filtered measurement of kalman filter

	float32_t matrix_H[4][4]; //observation matrix, how x matrix (the state) relates to the output/measurement z
	float32_t matrix_z[4][1]; //combined measured output of kalman filter and accelerometer tilt data

	float32_t matrix_Q[4][4]; //covariance matrix of process noise/disturbance
	float32_t matrix_R[4][4]; //covariance matrix of measurement noise
	float32_t matrix_Pp[4][4]; //predicted error covariance
	float32_t matrix_P[4][4]; //estimated error covariance

	float32_t matrix_K[4][4]; //Kalman gain

	//arm instances
	arm_matrix_instance_f32 matrix_A_arm;
	arm_matrix_instance_f32 matrix_B_arm;
	arm_matrix_instance_f32 matrix_xp_arm;
	arm_matrix_instance_f32 matrix_x_arm;
	arm_matrix_instance_f32 matrix_H_arm;
	arm_matrix_instance_f32 matrix_z_arm;
	arm_matrix_instance_f32 matrix_Q_arm;
	arm_matrix_instance_f32 matrix_R_arm;
	arm_matrix_instance_f32 matrix_Pp_arm;
	arm_matrix_instance_f32 matrix_P_arm;
	arm_matrix_instance_f32 matrix_K_arm;

	//euler angles, measurements used to fill z matrix in later on, which is used in kalman estimation step
	float32_t yaw_z;
	float32_t pitch_z;
	float32_t roll_z;

	//euler angles, output of kalman filter, should be more accurate than yaw, pitch, roll of sensor_data
	float32_t kalman_yaw;
	float32_t kalman_pitch;
	float32_t kalman_roll;

} kalman_filter;
typedef struct
{
	//cannot set values here since no memory allocated in declaration
	float Ts; //sampling time
	float proportional_gain;
	float integral_gain;
	float derivative_gain;

	float tau; // 1/(cutoff frequency of low pass filter after derivative component)

	float proportional_out;
	float integral_out;
	float derivative_out;

	float32_t error; //error is signed
	float32_t measured_pos;

	float total_out;
	float out_max;
	float out_min;
} pid_controller;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//for I2C
#define MPU6050_ADDR_LSL1	0xD0

//for PID
#define SENSOR_HISTORY_LEN	3 //PID only needs 3 max since it is at most second order
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c1_rx;

TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_ch1;

/* USER CODE BEGIN PV */
uint8_t i2c_RX_done = 0;
uint8_t i2c_TX_done = 0;
uint8_t data_ready = 0; //accelerometer data ready interrupt

uint8_t receive_buffer[20]; //received message buffer, randomly used

const float32_t dt = .004;

//setup storage for data
mpu6050_sensor_data sensor_data_1;
kalman_filter filter1;

//setup PID controller
pid_controller motor_controller;

//initialize pitch in degrees
float32_t pitch_adjusted_in_degrees = 0; //upright position will be 90 degrees instead of 0
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
HAL_StatusTypeDef i2c_Write_Accelerometer(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t MemAddress, uint8_t *pData, uint16_t len); // <-- Add this line here
HAL_StatusTypeDef i2c_Read_Accelerometer(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t regAddress, uint8_t *pData, uint16_t len);

//functions that communicate to the physical sensor using i2c interface
void mpu6050_init(I2C_HandleTypeDef *hi2c);
void mpu6050_get_raw_measurements(I2C_HandleTypeDef *hi2c, mpu6050_sensor_data *sensor_data);

//functions that modify sensor_data instances
void sensor_data_init(mpu6050_sensor_data *sensor_data); //sets yaw, pitch, roll to 0, 0, 0
void calc_accelerometer_tilt(mpu6050_sensor_data *sensor_data); //changes pitch and roll according to accelerometer tilt. Basically injects accelerometer data into kalman filter.

//modifies kalman filter instances
void kalman_filter_init(kalman_filter *filter); //sets initial kalman filter yaw, pitch, roll outputs to 0, 0, 0
void get_kalman_prediction(kalman_filter *filter, mpu6050_sensor_data *sensor_data);
void compute_kalman_gain(kalman_filter *filter);
void get_kalman_estimate(kalman_filter *filter, mpu6050_sensor_data *sensor_data);
void update_kalman_filter(kalman_filter *filter, mpu6050_sensor_data *sensor_data);
void update_ypr(kalman_filter *filter);

//PID control related functions
void initialize_PID(pid_controller *controller, uint16_t updated_measured_pos);
void set_gains_PID(pid_controller *controller, float Kp, float Ki, float Kd);
void update_PID(pid_controller *controller, float updated_measured_pos, float set_point);
void update_motor_input(int16_t new_out, uint32_t **active_buffer, uint32_t **inactive_buffer);
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
	MX_TIM1_Init();
	/* USER CODE BEGIN 2 */

	//HAL i2c notes:
	//address of MPU6050 device is 1101000, but we shift it to left because the transmit and receive functions require that. So we are left with 0xD0
	//Argument to right of MPU6050_ADDR_LSL1 is the register address, see the register description in onenote.
	uint8_t reg_addr[1];
	/* We compute the MSB and LSB parts of the memory address */
	reg_addr[0] = (uint8_t) (0x6A);

	//delay for init functions to see if it stops glitch of i2c transmission never completing
	HAL_Delay(1000);
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_9);
	HAL_Delay(100);
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_9);
	HAL_Delay(100);
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_9);
	HAL_Delay(100);
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_9);

	//test if transmission works
	HAL_StatusTypeDef returnValue = HAL_I2C_Master_Transmit_DMA(&hi2c1, MPU6050_ADDR_LSL1, reg_addr, 1);
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
	if (returnValue != HAL_OK)
	{
		Error_Handler();
	}
	if (__HAL_DMA_GET_FLAG(&hdma_i2c1_tx, (0x00000002U)))
	{ // Transfer error
		printf("DMA Transfer Error\n");
		// Handle error here
	}
	HAL_DMA_IRQHandler(&hdma_i2c1_tx);
	while (!i2c_TX_done);
	i2c_TX_done = 0;

	//setup storage for data
	mpu6050_init(&hi2c1); //write to registers in mpu6050 to configure initial settings
	// moved to private variables to see values of attributes in structs in debug mode easier
	//	mpu6050_sensor_data sensor_data_1;
	//	kalman_filter filter1;

	//define starting position
	sensor_data_init(&sensor_data_1); //likely not necessary
	kalman_filter_init(&filter1); //necessary

	//Initialize PWM
	//creating two buffers for a single motor. When changing speed, can update the inactive buffer and swap it with the active buffer
	//to avoid two devices trying to write to the same buffer.
	volatile uint32_t bufferA = 400; //out of 999, determines on time of PWM signal
	volatile uint32_t bufferB = 750;
	volatile uint32_t *active_buffer = &bufferA;
	volatile uint32_t *inactive_buffer = &bufferB;
	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t*) active_buffer, 1); //send the pulse to the timer peripheral, which determines duty cycle

	//setup PID controller
	// pid_controller motor_controller; // moved to private variables to see values of attributes in structs in debug mode easier
	initialize_PID(&motor_controller, 90); //assume 90 degrees is start position (upright)
	//set_gains_PID(&motor_controller, 69, 200, .1); //RED MOTOR (also works for yellow but less aggressive)
	//set_gains_PID(&motor_controller, 78, 0, .01); //RED MOTOR (also works for yellow but less aggressive)
	//set_gains_PID(&motor_controller, 60, 0, .005);
	set_gains_PID(&motor_controller, 25, 500, 1);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		if (data_ready)
		{
			HAL_NVIC_DisableIRQ(EXTI4_IRQn);
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_9);
			//get accelerometer and gyro data and store in struct.
			//reading without FIFO, IMPORTANT: if using interrupt to synchronize, need a series resistor between interrupt pin on sensor and EXTI pin. Helps to form low pass filter to dampen voltage spikes that mess up the i2c bus and probably more importantly decrease current that could drive SDA pin low.
			mpu6050_get_raw_measurements(&hi2c1, &sensor_data_1);
			//get tilt measurement from accelerometer, pitch and roll only
			calc_accelerometer_tilt(&sensor_data_1);
			//get estimate(output of b0,b1,b2,b3 euler parameters) from kalman filter
			update_kalman_filter(&filter1, &sensor_data_1);

			//get pitch
			pitch_adjusted_in_degrees = filter1.kalman_pitch * 180 / M_PI + 90; //upright position will be 90 degrees instead of 0

			//update PID output
			update_PID(&motor_controller, pitch_adjusted_in_degrees, 90);

			//update PWM signal to motors
			update_motor_input((int16_t) motor_controller.total_out, &active_buffer, &inactive_buffer); //make output whole number since pwm requires it

			data_ready = 0;
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_9);
			HAL_NVIC_EnableIRQ(EXTI4_IRQn);
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
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_TIM1;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
	PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
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
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 63;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 999;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

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
	/* DMA1_Channel4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

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
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);

	/*Configure GPIO pin : PA9 */
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PB3 */
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PB4 PB5 */
	GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);

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
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_3)
	{
		data_ready = 1;
	}
}

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
	command = 0x10;
	i2c_Write_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x1B, &command, 1); // +/- 1000 degrees/second full range, each 32.8 counts is 1 degree/second, all selftest off
	i2c_Read_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x1B, (uint8_t*) receive_buffer, 1); //read GYRO_CONFIG
	//Accelerometer Config
	i2c_Read_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x1C, (uint8_t*) receive_buffer, 1); //read ACCEL_CONFIG
	command = 0x10;
	i2c_Write_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x1C, &command, 1); // +/- 8g, every 4096 counts is 1g, all self test off
	i2c_Read_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x1C, (uint8_t*) receive_buffer, 1); //read ACCEL_CONFIG
}

void mpu6050_get_raw_measurements(I2C_HandleTypeDef *hi2c, mpu6050_sensor_data *sensor_data)
{
	//get raw data from mpu6050 with i2c interface
	//read without FIFO, IMPORTANT: if using interrupt to synchronize, need a series resistor between interrupt pin on sensor and EXTI pin. Helps to form low pass filter to dampen voltage spikes that mess up the i2c bus and probably more importantly decrease current that could drive SDA pin low.
	i2c_Read_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x3B, (uint8_t*) sensor_data->accel_x_buf, 2); //ACCEL_XOUT, 2 bytes
	i2c_Read_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x3D, (uint8_t*) sensor_data->accel_y_buf, 2); //ACCEL_YOUT, 2 bytes
	i2c_Read_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x3F, (uint8_t*) sensor_data->accel_z_buf, 2); //ACCEL_ZOUT, 2 bytes
	i2c_Read_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x43, (uint8_t*) sensor_data->gyro_x_buf, 2); //GYRO_XOUT, 2 bytes
	i2c_Read_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x45, (uint8_t*) sensor_data->gyro_y_buf, 2); //GYRO_YOUT, 2 bytes
	i2c_Read_Accelerometer(hi2c, MPU6050_ADDR_LSL1, 0x47, (uint8_t*) sensor_data->gyro_z_buf, 2); //GYRO_ZOUT, 2 bytes
	sensor_data->accel_x = (sensor_data->accel_x_buf[0] << 8) | sensor_data->accel_x_buf[1];
	sensor_data->accel_y = (sensor_data->accel_y_buf[0] << 8) | sensor_data->accel_y_buf[1];
	sensor_data->accel_z = (sensor_data->accel_z_buf[0] << 8) | sensor_data->accel_z_buf[1];
	sensor_data->gyro_x = (sensor_data->gyro_x_buf[0] << 8) | sensor_data->gyro_x_buf[1];
	sensor_data->gyro_y = (sensor_data->gyro_y_buf[0] << 8) | sensor_data->gyro_y_buf[1];
	sensor_data->gyro_z = (sensor_data->gyro_z_buf[0] << 8) | sensor_data->gyro_z_buf[1];
}

void sensor_data_init(mpu6050_sensor_data *sensor_data)
{
	//initialize values that will be calculated later anyways, probably not necessary to do
	sensor_data->yaw = 0;
	sensor_data->pitch = 0;
	sensor_data->roll = 0;
}
void calc_accelerometer_tilt(mpu6050_sensor_data *sensor_data)
{
//	sensor_data->pitch = asin((float) sensor_data->accel_x / (float) 4096); //get pitch in radians
//	sensor_data->roll = asin(((float) -sensor_data->accel_y) / (4096 * cos(sensor_data->pitch))) * (180 / M_PI); //get roll and convert to degrees
//	//convert to degrees after using it pitch in previous calculation as radians

	//using radians for sensor data so that it can be passed into Kalman filter as is, the fmin and fmax functions are to clip the inputs to the arcsin function between [-1,1]
	sensor_data->pitch = asin(fmax(-1, fmin(1, (float) sensor_data->accel_x / (float) 4096))); //get pitch in radians
	sensor_data->roll = asin(fmax(-1, fmin(1, ((float) -sensor_data->accel_y) / (4096 * cos(sensor_data->pitch))))); //get roll in radians

	//for the yaw, we use previous quaternion/euler parameter output from kalman filter and convert it to euler angle that is used as yaw
}
void get_euler_parameters(kalman_filter *filter)
{
	float32_t c1 = cos(filter->yaw_z / 2);
	float32_t s1 = sin(filter->yaw_z / 2);
	float32_t c2 = cos(filter->pitch_z / 2);
	float32_t s2 = sin(filter->pitch_z / 2);
	float32_t c3 = cos(filter->roll_z / 2);
	float32_t s3 = sin(filter->roll_z / 2);

	filter->matrix_z[0][0] = c1 * c2 * c3 + s1 * s2 * s3;
	filter->matrix_z[1][0] = c1 * c2 * s3 - s1 * s2 * c3;
	filter->matrix_z[2][0] = c1 * s2 * c3 + s1 * c2 * s3;
	filter->matrix_z[3][0] = s1 * c2 * c3 - c1 * s2 * s3;

}
void kalman_filter_init(kalman_filter *filter)
{
	//define starting position, definately necessary to get first output from kalman filter
	filter->yaw_z = 0;
	filter->pitch_z = 0;
	filter->roll_z = 0;

	filter->kalman_yaw = 0;
	filter->kalman_pitch = 0;
	filter->kalman_roll = 0;

	//initialize constant matrix contents. The A matrix and other matrices are skipped because it changes at every step
	memcpy(&(filter->matrix_B[0][0]), ((float32_t[4][1]){{0}, {0}, {0}, {0}}), 4 * 1 * sizeof(float32_t));
	memcpy(&(filter->matrix_x[0][0]), ((float32_t[4][1]){{1}, {0}, {0}, {0}}), 4 * 1 * sizeof(float32_t)); //previous output of kalman output on first step //quaternion corresponding to yaw, pitch, roll above is {1,0,0,0}

	float32_t temp_val = 1;
	memcpy(&(filter->matrix_H[0][0]), ((float32_t[4][4]){{temp_val, 0, 0, 0}, {0, temp_val, 0, 0}, {0, 0, temp_val, 0}, {0, 0, 0, temp_val}}), 4 * 4 * sizeof(float32_t));

	//set process noise covariance
	temp_val = 0.08;
	memcpy(&(filter->matrix_Q[0][0]), ((float32_t[4][4]){{temp_val, 0, 0, 0}, {0, temp_val, 0, 0}, {0, 0, temp_val, 0}, {0, 0, 0, temp_val}}), 4 * 4 * sizeof(float32_t));

	//set measurement noise covariance
	temp_val = 1;
	memcpy(&(filter->matrix_R[0][0]), ((float32_t[4][4]){{temp_val, 0, 0, 0}, {0, temp_val, 0, 0}, {0, 0, temp_val, 0}, {0, 0, 0, temp_val}}), 4 * 4 * sizeof(float32_t));
	temp_val = 1;
	memcpy(&(filter->matrix_P[0][0]), ((float32_t[4][4]){{temp_val, 0, 0, 0}, {0, temp_val, 0, 0}, {0, 0, temp_val, 0}, {0, 0, 0, temp_val}}), 4 * 4 * sizeof(float32_t));

	//initialize arm matrix instances
	arm_mat_init_f32(&filter->matrix_A_arm, 4, 4, &filter->matrix_A[0][0]);
	arm_mat_init_f32(&filter->matrix_B_arm, 4, 1, &filter->matrix_B[0][0]);
	arm_mat_init_f32(&filter->matrix_xp_arm, 4, 1, &filter->matrix_xp[0][0]);
	arm_mat_init_f32(&filter->matrix_x_arm, 4, 1, &filter->matrix_x[0][0]);
	arm_mat_init_f32(&filter->matrix_H_arm, 4, 4, &filter->matrix_H[0][0]);
	arm_mat_init_f32(&filter->matrix_z_arm, 4, 1, &filter->matrix_z[0][0]);
	arm_mat_init_f32(&filter->matrix_Q_arm, 4, 4, &filter->matrix_Q[0][0]);
	arm_mat_init_f32(&filter->matrix_R_arm, 4, 4, &filter->matrix_R[0][0]);
	arm_mat_init_f32(&filter->matrix_Pp_arm, 4, 4, &filter->matrix_Pp[0][0]);
	arm_mat_init_f32(&filter->matrix_P_arm, 4, 4, &filter->matrix_P[0][0]);
	arm_mat_init_f32(&filter->matrix_K_arm, 4, 4, &filter->matrix_K[0][0]);
}
void get_kalman_prediction(kalman_filter *filter, mpu6050_sensor_data *sensor_data)
{
	//do matrix multiplication to predict angular positions state "xp" (a priori estimate)
	arm_status arm_status_temp = arm_mat_mult_f32(&filter->matrix_A_arm, &filter->matrix_x_arm, &filter->matrix_xp_arm);
//	arm_status arm_status_temp = arm_mat_mult_f32(&filter->matrix_A_arm, &filter->matrix_xp_arm, &filter->matrix_Axp_arm);

	//predict error covariance "P"
	arm_status_temp = arm_mat_mult_f32(&filter->matrix_A_arm, &filter->matrix_P_arm, &filter->matrix_Pp_arm); //matrix_Pp_arm = A*P_{k-1}
	float32_t matrix_AT[4][4]; //getting transpose of A matrix
	arm_matrix_instance_f32 matrix_AT_arm;
	arm_mat_init_f32(&matrix_AT_arm, 4, 4, &matrix_AT[0][0]);
	arm_status_temp = arm_mat_trans_f32(&filter->matrix_A_arm, &matrix_AT_arm); //calculate transpose of A
	arm_status_temp = arm_mat_mult_f32(&filter->matrix_Pp_arm, &matrix_AT_arm, &filter->matrix_Pp_arm); //matrix_Pp_arm = A*P_{k-1}*A^{T}
	arm_mat_add_f32(&filter->matrix_Pp_arm, &filter->matrix_Q_arm, &filter->matrix_Pp_arm); //matrix_Pp_arm = A*P_{k-1}*A^{T} + Q = P_{k} (a priori)

}
void compute_kalman_gain(kalman_filter *filter)
{
	float32_t matrix_HT[4][4]; //holds transpose of H matrix
	arm_matrix_instance_f32 matrix_HT_arm;
	arm_mat_init_f32(&matrix_HT_arm, 4, 4, &matrix_HT[0][0]);
	arm_status arm_status_temp = arm_mat_trans_f32(&filter->matrix_H_arm, &matrix_HT_arm); //calculate transpose of H
	arm_status_temp = arm_mat_mult_f32(&filter->matrix_Pp_arm, &matrix_HT_arm, &filter->matrix_K_arm); //matrix_K_arm = P * H_{T}

	float32_t matrix_inv_part[4][4]; //will be equal to inv(H * P_{k} * H^{T} + R)
	arm_matrix_instance_f32 matrix_inv_part_arm;
	arm_mat_init_f32(&matrix_inv_part_arm, 4, 4, &matrix_inv_part[0][0]);
	arm_status_temp = arm_mat_mult_f32(&filter->matrix_H_arm, &filter->matrix_Pp_arm, &matrix_inv_part_arm); //matrix_inv_part_arm = H * P_{k}
	arm_status_temp = arm_mat_mult_f32(&matrix_inv_part_arm, &matrix_HT_arm, &matrix_inv_part_arm); // matrix_inv_part_arm = H * P_{k} * H^{T}
	arm_status_temp = arm_mat_add_f32(&matrix_inv_part_arm, &filter->matrix_R_arm, &matrix_inv_part_arm); // matrix_inv_part_arm = H * P_{k} * H^{T} + R
	arm_status_temp = arm_mat_inverse_f32(&matrix_inv_part_arm, &matrix_inv_part_arm); //matrix_inv_part_arm = inv(H * P_{k} * H^{T} + R)

	arm_status_temp = arm_mat_mult_f32(&filter->matrix_K_arm, &matrix_inv_part_arm, &filter->matrix_K_arm); //matrix_K_arm = P * H_{T} * inv(H * P_{k} * H^{T} + R)
}
void get_kalman_estimate(kalman_filter *filter, mpu6050_sensor_data *sensor_data)
{
	//get estimate of state "x"
	//want x = xp + K*(z - H*xp)
	arm_status arm_status_temp = arm_mat_mult_f32(&filter->matrix_H_arm, &filter->matrix_xp_arm, &filter->matrix_x_arm); //matrix_x_arm = H * xp
	arm_status_temp = arm_mat_sub_f32(&filter->matrix_z_arm, &filter->matrix_x_arm, &filter->matrix_x_arm); //matrix_x_arm = z - H * xp
	arm_status_temp = arm_mat_mult_f32(&filter->matrix_K_arm, &filter->matrix_x_arm, &filter->matrix_x_arm); //matrix_x_arm = K*(z - H * xp)
	arm_status_temp = arm_mat_add_f32(&filter->matrix_xp_arm, &filter->matrix_x_arm, &filter->matrix_x_arm); //matrix_x_arm = xp + K*(z - H * xp)

	//get estimate of error covariance "P"
	arm_status_temp = arm_mat_mult_f32(&filter->matrix_K_arm, &filter->matrix_H_arm, &filter->matrix_P_arm); //matrix_P_matrix = K * H
	arm_status_temp = arm_mat_mult_f32(&filter->matrix_P_arm, &filter->matrix_Pp_arm, &filter->matrix_P_arm); //matrix_P_matrix = K * H * Pp
	arm_status_temp = arm_mat_sub_f32(&filter->matrix_Pp_arm, &filter->matrix_P_arm, &filter->matrix_P_arm); //matrix_P_matrix = Pp - K * H * Pp

}
void update_kalman_filter(kalman_filter *filter, mpu6050_sensor_data *sensor_data)
{
	//update angular velocities in radians
	sensor_data->wx = sensor_data->gyro_x / (float32_t) 1879.301568; //1 degree for every 32.8 counts -> 1 radian for every 1879.301568 counts
	sensor_data->wy = sensor_data->gyro_y / (float32_t) 1879.301568;
	sensor_data->wz = sensor_data->gyro_z / (float32_t) 1879.301568;

	//measurements used to fill z matrix, these euler angles will be converted to quaternion form later. Fusion happens here as output of kalman filter and accelerometer are both used as measurement sources.
	filter->yaw_z = filter->kalman_yaw; //comes from output of kalman filter
	//using pitch and roll from accelerometer measurement to "inject" into kalman filter
	filter->pitch_z = sensor_data->pitch;
	filter->roll_z = sensor_data->roll;

	//update discrete A matrix. Using zero order hold estimation: A_discrete = I + A_analog * dt
	//dt is periods

	/*  A_analog = 1/2*[ 0   -wx  -wy  -wz ;
	 *					 wx   0    wz  -wy ;
	 *					 wy  -wz   0    wx ;
	 *					 wz   wy  -wx   0  ]; */

	const float32_t wx_const = sensor_data->wx;
	const float32_t wy_const = sensor_data->wy;
	const float32_t wz_const = sensor_data->wz;
	float32_t temp_A[4][4] = {{1, -wx_const * dt / 2, -wy_const * dt / 2, -wz_const * dt / 2},
								{wx_const * dt / 2, 1, wz_const * dt / 2, -wy_const * dt / 2},
								{wy_const * dt / 2, -wz_const * dt / 2, 1, wx_const * dt / 2},
								{wz_const * dt / 2, wy_const * dt / 2, -wx_const * dt / 2, 1}};
	memcpy(&(filter->matrix_A[0][0]), &temp_A, 4 * 4 * sizeof(float32_t));

	//update z, the measurements. The measurements are combination of previous kalman filter output (yaw) and accelerometer data (pitch and roll)
	get_euler_parameters(filter);

	get_kalman_prediction(filter, sensor_data);
	compute_kalman_gain(filter);
	get_kalman_estimate(filter, sensor_data);
	update_ypr(filter);
}
void update_ypr(kalman_filter *filter)
{
	//turn quaternion (x) to euler angles (yaw pitch roll)

	float32_t q0 = filter->matrix_x[0][0];
	float32_t q1 = filter->matrix_x[1][0];
	float32_t q2 = filter->matrix_x[2][0];
	float32_t q3 = filter->matrix_x[3][0];

	//update kalman filter final output for timestep
	filter->kalman_yaw = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
	filter->kalman_pitch = asin(fmax(-1, fmin(1, -2 * (q1 * q3 - q0 * q2))));
	filter->kalman_roll = atan2(2 * (q2 * q3 + q0 * q1), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);

}

//PID control functions
void initialize_PID(pid_controller *controller, uint16_t updated_measured_pos)
{

	controller->Ts = 0.004; //equates to 250Hz
	controller->tau = .01;

	controller->out_max = 1000;
	controller->out_min = -1000;

	controller->proportional_gain = 0;
	controller->integral_gain = 0;
	controller->derivative_gain = 0;

	controller->proportional_out = 0;
	controller->integral_out = 0;
	controller->derivative_out = 0;
	controller->total_out = 0;

	controller->error = 0;
	controller->measured_pos = updated_measured_pos;
}
void set_gains_PID(pid_controller *controller, float Kp, float Ki, float Kd)
{
	controller->proportional_gain = Kp;
	controller->integral_gain = Ki;
	controller->derivative_gain = Kd;
}
void update_PID(pid_controller *controller, float updated_measured_pos, float set_point)
{
	float adjusted_measured_pos = updated_measured_pos;

	float updated_error = set_point - updated_measured_pos;

	//this block makes sure that if the setpoint is near boundaries (0 or 359 degrees), can still approach the setpoint
	//from the direction that has the angle measurement spike from 0 to 359 degrees or 359 to 0 degrees
	//this is done by adjusting the error term
	if (updated_measured_pos > set_point + 180 && set_point < 180)
	{
		adjusted_measured_pos = adjusted_measured_pos - 360;
	}
	else if (updated_measured_pos < set_point - 180 && set_point >= 180)
	{
		adjusted_measured_pos = adjusted_measured_pos + 360;
	}
	updated_error = set_point - adjusted_measured_pos;

	//calculate difference for derivative term, but need to account for when motor goes from 359->0 and 0->359
	//make sure that when it goes 359->0, the difference is 1, and 0->359 is -1
	float32_t position_difference = updated_measured_pos - controller->measured_pos;
	if (position_difference > 300) //when it goes from 0 to 359
	{
		position_difference = position_difference - 360;
	}
	else if (position_difference < -300) //when it goes from 359 to 0
	{
		position_difference = position_difference + 360;
	}

	//updated the outputs of the P, I, and D components of the controller
	controller->proportional_out = controller->proportional_gain * updated_error;
	controller->integral_out = controller->integral_gain * controller->Ts * (updated_error + controller->error) / 2.0 + controller->integral_out;
	controller->derivative_out = ((controller->derivative_gain * 2) * (position_difference) //
	+ (2 * controller->tau - controller->Ts) * controller->derivative_out) / (2 * controller->tau + controller->Ts);
	//note: derivative term uses measured value instead of error term to avoid kick back

	//clamp integrator implementation
	float integral_min, integral_max;
	//determine integrator limits
	if (controller->out_max > controller->proportional_out - controller->derivative_out)
	{
		integral_max = controller->out_max - controller->proportional_out + controller->derivative_out; //see total_out comment to see why adding derivative term instead of subtracting here
	}
	else
	{
		integral_max = 0;
	}
	if (controller->out_min < controller->proportional_out - controller->derivative_out)
	{
		integral_min = controller->out_min - controller->proportional_out + controller->derivative_out; //see total_out comment to see why adding derivative term instead of subtracting here
	}
	else
	{
		integral_min = 0;
	}

	//get absolute error
	float32_t absval_error = controller->error;
	if (absval_error < 0)
	{
		absval_error = -1 * absval_error;
	}

	if (absval_error < 5) //limit integrator even more once closer to desired angle.
	{
		integral_max = 600;
		integral_min = -600;
	}
	if (absval_error < 3) //limit integrator even more once closer to desired angle.
	{
		integral_max = 300; //RED MOTOR
		integral_min = -300;
	}
	//clamping of integrator
	if (controller->integral_out > integral_max)
	{
		controller->integral_out = integral_max;
	}
	else if (controller->integral_out < integral_min)
	{
		controller->integral_out = integral_min;
	}

	//compute total output of controller
	controller->total_out = controller->proportional_out + controller->integral_out - controller->derivative_out; //note negative sign on derivative term, this is correct since it is on the feedback loop

	//limit total output of controller
	if (controller->total_out > controller->out_max)
	{
		controller->total_out = controller->out_max;
	}
	else if (controller->total_out < controller->out_min)
	{
		controller->total_out = controller->out_min;
	}

	//updated the error and measured position
	controller->error = updated_error;
	controller->measured_pos = updated_measured_pos;

}
void update_motor_input(int16_t new_out, uint32_t **active_buffer_address, uint32_t **inactive_buffer_address)
{
	if (new_out > 0)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);

	}
	else if (new_out < 0)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
		new_out = new_out * -1;
	}

	**inactive_buffer_address = new_out;
	uint32_t *temp_uint32_address = *active_buffer_address;
	*active_buffer_address = *inactive_buffer_address;
	*inactive_buffer_address = temp_uint32_address;

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
