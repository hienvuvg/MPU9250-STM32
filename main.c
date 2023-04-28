/* USER CODE BEGIN Header */
/**
  * Don't call I2C (reading sensor) when it's not in use.
  * MPU can be used with both I2C and SPI. This library supports both.
  *
  * CPS_EndNode_v2
  * 06/07/2022
  *
  * SPI_IMU_CS PA4
  * SPI1_SCK PA5 D13 SCL
  * SPI1_MISO PA6 D12 AD0
  * SPI1_MOSI PA7 D11 SDA
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* Private state prototypes -----------------------------------------------*/

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
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

__IO bool setup_change = 0;

uint8_t mode = 0;

extern uint8_t _mag_adjust[3]; // Returned value from the magnetometer

int16_t AccelRaw[3], GyroRaw[3], MagRaw[3], TempRaw[1];
float CorrectedAccel[3], CorrectedGyro[3];
float CorrectedMag[3], AlignedMag[3][1];
int16_t MagData[3];

float accScale, gyroScale;

const float G = 9.807;

float temp_offset = 0, temp_sens = 333.87;
float TempC;

uint8_t temp;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
/* Private state prototypes -----------------------------------------------*/

void LED_on(void);
void LED_off(void);
void ReadSensors(uint8_t sample);
void SetAccelScale(uint8_t range);
void SetGyroScale(uint8_t range);
void ConvertIMUData();
void RawPrint(void);
void DataPrint(void);
void RawMagPrint(void);
void CorrectedMagPrint(void);
void printbuff(void);
void cleanbuff(void);
void MagDataAlignment(void);
void DummySensor(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, char *ptr, int len){
	HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len, 0xffffff);
  return len;
}


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
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

	// *********************** Start Main ******************** //
//	SetPin(Boost_Gate_GPIO_Port, Boost_Gate_Pin, BOOST_GATE_OFF);
//	SetPin(RFID_SHD_GPIO_Port, RFID_SHD_Pin, RFID_OFF);
//	SetPin(IMU_Gate_GPIO_Port, IMU_Gate_Pin, IMU_GATE_OFF);
//	SetPin(D_GREEN_GPIO_Port, D_GREEN_Pin, LED_OFF);

	// Need to provide power to the IMU separately

	printf("\n Power On \n");

//	SetPin(IMU_Gate_GPIO_Port, IMU_Gate_Pin, IMU_GATE_ON);

//	LED_on();
//	HAL_Delay(300);
//	LED_off();

//	if (AK8963_Init()) printf("AK8963_Init failed\n");
//	else printf("AK8963 Initialized\n");

	SetAccelScale(ACCEL_RANGE);
	SetGyroScale(GYRO_RANGE);

	//uint8_t data2 = readRegister(FIFO_EN);
	//MyWriteRegister(FIFO_EN, data2 & ~TEMP_FIFO_EN); // EDIT disable temperature FIFO


	// enable accelerometer and gyro
	//MyWriteRegister(PWR_MGMNT_2, 0x00); // Enable gryo and accel
	//MyWriteRegister(PWR_MGMNT_2, 0b000111); // EDIT: Enable accel only
	//MyWriteRegister(PWR_MGMNT_2, 0BT111000); // EDIT: Enable gyro only

	//uint8_t data3 = readRegister(FIFO_EN);
	//MyWriteRegister(FIFO_EN, data3 & ~0x08); // EDIT disable accel FIFO, nonsense
    
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	// & ~bit: clear bit at the position (set to 0)
	// ! bit: write bit at the position (set to 1)

	// ODR: Output Data Rate

	setup_change = 0;

    while (1){

    	if(setup_change){
    		setup_change = 0;

			switch(mode){
			case 5:

				MyWriteRegister(PWR_MGMNT_1, SLEEP, RESET_BITS); // exit from sleep
				MyWriteRegister(PWR_MGMNT_2, DISABLE_A, RESET_BITS); // accel wakes up

				printf("\n accelerometer low power mode (no cycle mode) \n");
				// Page 16: lposc_clksel Register 30 – Low Power Accelerometer ODR Control (4 bit), 0x1E
				// Sets the frequency of waking up the chip to take a sample of accel data
				// Set accel output rate from table at the end page 16 v1.4
				MyWriteRegister(lposc_clksel, 5, SET_BITS);

				MyWriteRegister(PWR_MGMNT_1, CYCLE, RESET_BITS); // Exit cycle mode

				MyWriteRegister(PWR_MGMNT_1, PD_PTAT, RESET_BITS); // Power up temperature sensor

				MyWriteRegister(PWR_MGMNT_2, DISABLE_G, SET_BITS); // Disable gyro

				break;

			case 0:

				MyWriteRegister(PWR_MGMNT_1, SLEEP, RESET_BITS); // exit from sleep
				MyWriteRegister(PWR_MGMNT_2, DISABLE_A, RESET_BITS); // accel wakes up

				printf("\n accelerometer low power mode (cycle mode) \n");
				// Page 16: lposc_clksel Register 30 – Low Power Accelerometer ODR Control (4 bit), 0x1E
				// Sets the frequency of waking up the chip to take a sample of accel data
				// Set accel output rate from table at the end page 16 v1.4
				MyWriteRegister(lposc_clksel, 5, SET_BITS);

				// Put into Accelerometer Only Low Power Mode using the following steps:
				// Set CYCLE bit to 1
				// Set SLEEP bit to 0
				// Set TEMP_DIS bit to 1
				// Set DIS_XG, DIS_YG, DIS_ZG bits to 1

				MyWriteRegister(PWR_MGMNT_1, CYCLE, SET_BITS); // Enter cycle mode

				MyWriteRegister(PWR_MGMNT_1, PD_PTAT, RESET_BITS); // Power up temperature sensor

				MyWriteRegister(PWR_MGMNT_2, DISABLE_G, SET_BITS); // Disable gyro

				// MyWriteRegister(USER_CTRL, I2C_MST_EN, RESET_BITS); // Disable I2C Master, this will disable Magnetometer

				//MyWriteRegister(USER_CTRL, DMP_EN, RESET_BITS); // Disable DMP

				//MyWriteRegister(USER_CTRL, FIFO_EN, RESET_BITS); // Disable FIFO

				// Put magnetometer to power-down mode by a soft reset
				//writeAK8963Register(AK8963_CNTL1, 0x00); // Power down
				//writeAK8963Register(AK8963_CNTL2, 0x01); // Soft reset
				// => Not working, solved by not initializing the mag at first

				//MyWriteRegister(PWR_MGMNT_1, SLEEP, SET_BITS); // EDIT go to sleep

				//MyWriteRegister(PWR_MGMNT_2, DISABLE_G, SET_BITS); // EDIT gyro go to sleep
				//MyWriteRegister(PWR_MGMNT_2, DISABLE_A, SET_BITS); // EDIT accel go to sleep
				break;


			case 1: //
				printf("\n Accel & Mag, no cycle mode\n");

				if (MPU6515_Init()) printf("MPU6515_Init failed\n");
				else printf("MPU6515 Initialized\n");

				MyWriteRegister(PWR_MGMNT_1, SLEEP, RESET_BITS); // exit from sleep
				MyWriteRegister(PWR_MGMNT_2, DISABLE_A, RESET_BITS); // accel wakes up

				MyWriteRegister(PWR_MGMNT_1, CYCLE, RESET_BITS); // Exit cycle mode

				MyWriteRegister(USER_CTRL, I2C_MST_EN, SET_BITS); // Enable I2C Master (needed before setting magnetometer)

				if (AK8963_Init()) printf("AK8963_Init failed\n");
				else printf("AK8963 Initialized\n");

				//MyWriteRegister(PWR_MGMNT_1, CYCLE, SET_BITS); // Set cycle mode

				//MyWriteRegister(PWR_MGMNT_1, PD_PTAT, SET_BITS); // Power down temperature sensor

				MyWriteRegister(PWR_MGMNT_2, DISABLE_G, SET_BITS); // Disable gyro

				//MyWriteRegister(USER_CTRL, I2C_MST_EN, RESET_BITS); // Disable I2C Master

				//MyWriteRegister(USER_CTRL, DMP_EN, RESET_BITS); // Disable DMP

				//MyWriteRegister(USER_CTRL, FIFO_EN, RESET_BITS); // Disable FIFO

				break;

			case 2: //
				printf("\n Gyro & Accel & Mag, no cycle mode\n");

				if (MPU6515_Init()) printf("MPU6515_Init failed\n");
				else printf("MPU6515 Initialized\n");

				MyWriteRegister(PWR_MGMNT_1, SLEEP, RESET_BITS); // exit from sleep
				MyWriteRegister(PWR_MGMNT_2, DISABLE_A, RESET_BITS); // accel wakes up
				MyWriteRegister(PWR_MGMNT_2, DISABLE_G, RESET_BITS); // gyro wakes up

				MyWriteRegister(PWR_MGMNT_1, CYCLE, RESET_BITS); // Exit cycle mode

				MyWriteRegister(USER_CTRL, I2C_MST_EN, SET_BITS); // Enable I2C Master (needed before setting magnetometer)

				if (AK8963_Init()) printf("AK8963_Init failed\n");
				else printf("AK8963 Initialized\n");

				//MyWriteRegister(PWR_MGMNT_1, CYCLE, SET_BITS); // Set cycle mode

				//MyWriteRegister(PWR_MGMNT_1, PD_PTAT, SET_BITS); // Power down temperature sensor

				//MyWriteRegister(USER_CTRL, I2C_MST_EN, RESET_BITS); // Disable I2C Master

				//MyWriteRegister(USER_CTRL, DMP_EN, RESET_BITS); // Disable DMP

				//MyWriteRegister(USER_CTRL, FIFO_EN, RESET_BITS); // Disable FIFO

				break;

			case 6: //
				printf("\n Accel & Mag 8 Hz\n");

				MyWriteRegister(PWR_MGMNT_1, SLEEP, RESET_BITS); // exit from sleep
				MyWriteRegister(PWR_MGMNT_2, DISABLE_A, RESET_BITS); // accel wakes up

				MyWriteRegister(PWR_MGMNT_1, CYCLE, RESET_BITS); // Exit cycle mode
				MyWriteRegister(USER_CTRL, I2C_MST_EN, SET_BITS); // Enable I2C Master (needed before setting magnetometer)
				if (AK8963_Init()) printf("AK8963_Init failed\n");
				else printf("AK8963 Initialized\n");
				//writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN); // set AK8963 to Power Down
				//MyWriteRegister(USER_CTRL, I2C_MST_EN, RESET_BITS); // Disable I2C Master

				MyWriteRegister(lposc_clksel, 5, SET_BITS);
				MyWriteRegister(PWR_MGMNT_1, CYCLE, SET_BITS); // Set cycle mode

				MyWriteRegister(PWR_MGMNT_2, DISABLE_G, SET_BITS); // Disable gyro
				MyWriteRegister(USER_CTRL, DMP_EN, RESET_BITS); // Disable DMP
				MyWriteRegister(USER_CTRL, FIFO_EN, RESET_BITS); // Disable FIFO

				break;

			case 4: //
				printf("\n Sleep Mode \n");

				MyWriteRegister(PWR_MGMNT_1, SLEEP, RESET_BITS); // exit from sleep
				MyWriteRegister(PWR_MGMNT_2, DISABLE_A, RESET_BITS); // accel wakes up
				MyWriteRegister(PWR_MGMNT_1, CYCLE, RESET_BITS); // Exit cycle mode

				writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN); // set AK8963 to Power Down

				// Powering down
				MyWriteRegister(PWR_MGMNT_2, DISABLE_G, SET_BITS); // gyro go to sleep
				MyWriteRegister(PWR_MGMNT_2, DISABLE_A, SET_BITS); // accel go to sleep
				MyWriteRegister(PWR_MGMNT_1, PD_PTAT, SET_BITS); // Power down temperature sensor

				MyWriteRegister(USER_CTRL, I2C_MST_EN, RESET_BITS); // Disable I2C Master
				MyWriteRegister(USER_CTRL, DMP_EN, RESET_BITS); // Disable DMP
				MyWriteRegister(USER_CTRL, FIFO_EN, RESET_BITS); // Disable FIFO

				MyWriteRegister(PWR_MGMNT_1, SLEEP, SET_BITS); // go to sleep
				break;

			case 3:
				printf("\n Mag only, 8Hz \n");

				MyWriteRegister(PWR_MGMNT_1, SLEEP, RESET_BITS); // exit from sleep
				MyWriteRegister(PWR_MGMNT_1, CYCLE, RESET_BITS); // Exit cycle mode
				MyWriteRegister(USER_CTRL, I2C_MST_EN, SET_BITS); // Enable I2C Master (needed before setting magnetometer)

				if (AK8963_Init()) printf("AK8963_Init failed\n");
				else printf("AK8963 Initialized\n");

				//MyWriteRegister(lposc_clksel, 5, SET_BITS);
				//MyWriteRegister(PWR_MGMNT_1, CYCLE, SET_BITS); // Set cycle mode

				MyWriteRegister(PWR_MGMNT_2, DISABLE_G, SET_BITS); // gyro go to sleep
				MyWriteRegister(PWR_MGMNT_2, DISABLE_A, SET_BITS); // accel go to sleep
				//MyWriteRegister(PWR_MGMNT_1, SLEEP, SET_BITS); // go to sleep

				MyWriteRegister(USER_CTRL, DMP_EN, RESET_BITS); // Disable DMP
				//MyWriteRegister(PWR_MGMNT_1, PD_PTAT, SET_BITS); // Power down temperature sensor
				//MyWriteRegister(USER_CTRL, FIFO_EN, RESET_BITS); // Disable FIFO

				break;

			default:
				break;
			}
    	}

		ReadSensors(1);
		ConvertIMUData(); // Gyro & Accel Data Conversion
		DataPrint();
		//RawPrint();

		HAL_Delay(100);

    // *********************** End Main ******************** //

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS_PWR;
  RCC_OscInitStruct.HSEDiv = RCC_HSE_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 6;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV6;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00303D5B;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
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
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, FE_CTRL3_Pin|FE_CTRL2_Pin|FE_CTRL1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_IMU_CS_GPIO_Port, SPI_IMU_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : FE_CTRL3_Pin FE_CTRL2_Pin FE_CTRL1_Pin */
  GPIO_InitStruct.Pin = FE_CTRL3_Pin|FE_CTRL2_Pin|FE_CTRL1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BT1_Pin */
  GPIO_InitStruct.Pin = BT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B3_Pin */
  GPIO_InitStruct.Pin = B3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(B3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B2_Pin */
  GPIO_InitStruct.Pin = B2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(B2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_IMU_CS_Pin */
  GPIO_InitStruct.Pin = SPI_IMU_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_IMU_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */


/* =========================== My Code =========================== */

//void LED_on(void){SetPin(D_GREEN_GPIO_Port, D_GREEN_Pin, LED_ON);}
//void LED_off(void){SetPin(D_GREEN_GPIO_Port, D_GREEN_Pin, LED_OFF);}

// Raw value range: +/- 32768 (16 bits)
void ReadSensors(uint8_t sample){
  int32_t GyroTemp[3] = ARRAY_3, AccelTemp[3] = ARRAY_3, MagTemp[3] = ARRAY_3, TempTemp[3] = ARRAY_3;

  for (int j=0; j< sample; j++ ){
    MPU9250_GetData(GyroRaw, AccelRaw, MagRaw, TempRaw);
    MagRaw[3] = MagRaw[3]*(-1); // MPU9520 Sensor Alignment: Mag z axit only
    for (int i=0; i<3; i++ ){
      GyroTemp[i] += GyroRaw[i];
      AccelTemp[i] += AccelRaw[i];
      MagTemp[i] += MagRaw[i];
      TempTemp[i] += TempRaw[i];
    }
  }

  for (int i=0; i<3; i++ ){
    GyroRaw[i] = (int16_t)(GyroTemp[i]/sample);
    AccelRaw[i] = (int16_t)(AccelTemp[i]/sample);
    MagRaw[i] = (int16_t)(MagTemp[i]/sample);
    TempRaw[i] = (int16_t)(TempTemp[i]/sample);
  }
}

void ConvertIMUData(void){
	for (int i=0; i<3; i++ ){
		CorrectedGyro[i]= (float)(GyroRaw[i]*gyroScale);
	}
	for (int i=0; i<3; i++ ){
		CorrectedAccel[i]= (float)(AccelRaw[i]*accScale);
	}

	MagData[0] = (int16_t)((float)MagRaw[0] * (((((float)_mag_adjust[0]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f)); // micro Tesla
	MagData[1] = (int16_t)((float)MagRaw[1] * (((((float)_mag_adjust[1]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f)); // micro Tesla
	MagData[2] = (int16_t)((float)MagRaw[2] * (((((float)_mag_adjust[2]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f)); // micro Tesla

	TempC = ((TempRaw[0] - temp_offset)/temp_sens)+ 21; // Page 33 - datashet v1.6
}


void SetAccelScale(uint8_t range) {
	if (range == ACCEL_FS_SEL_2G) accScale = G * 2.0/32767.5;
	if (range == ACCEL_FS_SEL_4G) accScale = G * 4.0/32767.5;
	if (range == ACCEL_FS_SEL_8G) accScale = G * 8.0/32767.5;
	if (range == ACCEL_FS_SEL_16G) accScale = G * 16.0/32767.5;
}

void SetGyroScale(uint8_t range) {
	if (range == GYRO_FS_SEL_250DPS) gyroScale = 250.0*M_PI/32767.5/180.0;
	if (range == GYRO_FS_SEL_500DPS) gyroScale = 500.0*M_PI/32767.5/180.0;
	if (range == GYRO_FS_SEL_1000DPS) gyroScale = 1000.0*M_PI/32767.5/180.0;
	if (range == GYRO_FS_SEL_2000DPS) gyroScale = 2000.0*M_PI/32767.5/180.0;
}


void RawPrint(void){
	printf("Raw \t%d %d %d \t%d %d %d \t%d %d %d \t%d\n",
	    			GyroRaw[0], GyroRaw[1], GyroRaw[2],
					AccelRaw[0], AccelRaw[1], AccelRaw[2],
					MagRaw[0], MagRaw[1], MagRaw[2], TempRaw[0]);
}

void DataPrint(void){
	printf("%4.3f %4.3f %4.3f \t%4.3f %4.3f %4.3f  \t%d %d %d \t%.2f\n",
	    			CorrectedGyro[0], CorrectedGyro[1], CorrectedGyro[2],
					CorrectedAccel[0], CorrectedAccel[1], CorrectedAccel[2],
					MagData[0], MagData[1], MagData[2], TempC);
}

void RawMagPrint(void){
	printf("%4.0f %4.0f %4.0f\n", AlignedMag[0][0], AlignedMag[1][0], AlignedMag[2][0]);
}


/**
  * @brief  Retargets the C library printf state to the USART.
  * @param  None
  * @retval None
  */
/*
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF); 

  return ch;
}
*/

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
