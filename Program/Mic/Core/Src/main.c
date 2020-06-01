/* USER CODE BEGIN Header */
/*

�?�?�?╗░░░�?�?�?╗�?�?╗░�?�?�?�?�?╗░�?�?�?�?�?�?╗░░�?�?�?�?�?╗░�?�?�?╗░░░�?�?�?╗░�?�?�?�?�?╗░�?�?╗░░░�?�?╗░�?�?�?�?�?�?╗�?�?�?�?�?�?�?╗
�?�?�?�?╗░�?�?�?�?║�?�?║�?�?╔�?�?�?�?╗�?�?╔�?�?�?�?╗�?�?╔�?�?�?�?╗�?�?�?�?╗░�?�?�?�?║�?�?╔�?�?�?�?╗�?�?║░░░�?�?║�?�?╔�?�?�?�?╝�?�?╔�?�?�?�?╝
�?�?╔�?�?�?�?╔�?�?║�?�?║�?�?║░░╚�?╝�?�?�?�?�?�?╔╝�?�?║░░�?�?║�?�?╔�?�?�?�?╔�?�?║�?�?║░░�?�?║�?�?║░░░�?�?║╚�?�?�?�?�?╗░�?�?�?�?�?╗░░
�?�?║╚�?�?╔╝�?�?║�?�?║�?�?║░░�?�?╗�?�?╔�?�?�?�?╗�?�?║░░�?�?║�?�?║╚�?�?╔╝�?�?║�?�?║░░�?�?║�?�?║░░░�?�?║░╚�?�?�?�?�?╗�?�?╔�?�?╝░░
�?�?║░╚�?╝░�?�?║�?�?║╚�?�?�?�?�?╔╝�?�?║░░�?�?║╚�?�?�?�?�?╔╝�?�?║░╚�?╝░�?�?║╚�?�?�?�?�?╔╝╚�?�?�?�?�?�?╔╝�?�?�?�?�?�?╔╝�?�?�?�?�?�?�?╗
╚�?╝░░░░░╚�?╝╚�?╝░╚�?�?�?�?╝░╚�?╝░░╚�?╝░╚�?�?�?�?╝░╚�?╝░░░░░╚�?╝░╚�?�?�?�?╝░░╚�?�?�?�?�?╝░╚�?�?�?�?�?╝░╚�?�?�?�?�?�?╝

░�?�?�?�?�?�?╗░░░�?�?�?╗░░░�?�?�?╗�?�?�?�?╗░░░�?�?╗░░�?�?�?╗░░
�?�?╔�?�?�?�?╝░░░�?�?�?�?╗░�?�?�?�?║�?�?�?�?║░░░�?�?║░�?�?�?�?║░░
╚�?�?�?�?�?╗░░░░�?�?╔�?�?�?�?╔�?�?║�?�?╚�?�?╗░�?�?╔╝�?�?╔�?�?║░░
░╚�?�?�?�?�?╗░░░�?�?║╚�?�?╔╝�?�?║�?�?░╚�?�?�?�?╔╝░╚�?╝�?�?║░░
�?�?�?�?�?�?╔╝�?�?╗�?�?║░╚�?╝░�?�?║�?�?░░╚�?�?╔╝░░�?�?�?�?�?�?�?╗
╚�?�?�?�?�?╝░╚�?╝╚�?╝░░░░░╚�?╝�?�?░░░╚�?╝░░░╚�?�?�?�?�?�?╝
*/
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MY_NRF24.h"
//#include "TJ_MPU6050.h"
#include "AccelGyro.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Funkcja to paczka danych
#define FUNC_ACCEL_GYRO_DATA	128
#define FUNC_JOYSTICK_DATA		129
#define FUNC_ACCEL_GYRO_COMBO	130
// Tryb to trybie pracy
#define MODE_ACCELERATION_BURST 160
#define MODE_VELOCITY_BURST		161
#define MODE_DISTANCE_BURST		162
// Program to jakies zadanie do zrealizowania
#define PROG_CALLIBRATE			192
#define PROG_RESET_VEL_DIST		193
#define PROG_MOVE_BREAK			194
#define PROG_ROTATE				195
// Odpowiedz ze cos gotowe i zwraca wartosc
#define OK						255
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint64_t PipeAddres = 0x11223344AA;
char Msg[32] = "HI";
char RF_RxData[32];
int Flag = 0;
int Counter = 0;
int mode = MODE_ACCELERATION_BURST;


void MotorSetValue(int16_t Left, int16_t Right)
{
	if (Right > 0) {
		SetPwm(Right, 3);
		SetPwm(0, 4);
	} else {
		SetPwm(0, 3);
		SetPwm(-1 * Right, 4);
	}
	if (Left > 0) {
		SetPwm(Left, 2);
		SetPwm(0, 1);
	} else {
		SetPwm(0, 2);
		SetPwm(-1 * Left, 1);
	}
}

void SetPwm(uint16_t Value, uint16_t Channel) {
    TIM_OC_InitTypeDef sConfigOC;

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = Value;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    switch(Channel) {
    case 1: {
    	HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
		break;
    }
    case 2: {
		HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
		break;
	}
    case 3: {
		HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
		break;
	}
    case 4: {
		HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4);
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
		break;
	}
    default : {
    	// Blad XDD
    	break;
    }
    }

}

double MapValue(double Val, double FromLow,double FromHigh,double ToLow,double ToHigh){
	return ToLow + ((ToHigh - ToLow) / (FromHigh - FromLow)) * (Val - FromLow);
}

double MapValue2(double input, double input_start,double input_end,double output_start,double output_end) {
	double slope = 1.0 * (output_end - output_start) / (input_end - input_start);
	return output_start + slope * (input - input_start);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance == htim3.Instance){
		Flag = 1;
	}
}

int Serial_Send(uint8_t* Buf, uint32_t *Len){
	CDC_Transmit_FS(Buf, Len);
	return &Len;
}

void SendInt(int Number){
	char in[10], out[10];
	int a = 0;
	int num;
	while ((Number / 10 > 0) || (Number % 10) > 0){
		num = (Number % 10);
		in[a] = num + '0';
		Number = Number / 10;
		a++;
	}
	//for (int b = a - 1; b >= 0; b--) {
	//	out[a - 1 - b] = in[b];
	//}
	if (a > 0) {
		CDC_Transmit_FS(in, a);
	}
}

void SendReturn(short val){
	Msg[0] = OK;
	Msg[1] = val;
	Msg[2] = val >> 8;
	if (NRF24_write(Msg, 3))
	  {
		  //HAL_Delay(1);
		  NRF24_read(RF_RxData, 16);
	  }
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
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  // Timer
  HAL_TIM_Base_Start_IT(&htim3);

  // NRF
  NRF24_begin(CSN_GPIO_Port, CSN_Pin, CE_Pin, hspi1);
  NRF24_stopListening();
  NRF24_openWritingPipe(PipeAddres);
  NRF24_setAutoAck(true);
  NRF24_setChannel(52);
  NRF24_setPayloadSize(13);
  NRF24_enableDynamicPayloads();
  NRF24_enableAckPayload();

  // MPU
  __HAL_RCC_I2C2_FORCE_RESET();
  __HAL_RCC_I2C2_RELEASE_RESET();
  MX_I2C2_Init();
  __HAL_RCC_I2C2_FORCE_RESET();
  __HAL_RCC_I2C2_RELEASE_RESET();
  MX_I2C2_Init();
  Init(&hi2c2, &htim2);	// NIC NIE MOZE BYC PO TYM, URUCHAMIA SIE ZEGAR DO CALKOWANIA
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //MotorSetValue(0, 255);
	  Update();
	  switch (mode){
	  case MODE_ACCELERATION_BURST: {
		  GetAcceleration(Msg, 1);
		  break;
	  }
	  case MODE_VELOCITY_BURST: {
		  GetVelocity(Msg, 1);
		  break;
	  }
	  case MODE_DISTANCE_BURST: {
		  GetDistance(Msg, 1);
		  break;
	  }
	  }

	  Msg[0] = FUNC_ACCEL_GYRO_DATA;

	  if (NRF24_write(Msg, 13))
	  {
		  //HAL_Delay(1);
		  NRF24_read(RF_RxData, 16);
	  }
	  //Counter ++;
	  HAL_Delay(1);
	  switch (RF_RxData[0]) {
	  case FUNC_JOYSTICK_DATA: {
		  //HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		  int16_t Forward = (unsigned char)RF_RxData[1] | (((uint16_t)RF_RxData[2]) << 8);
		  int16_t Direction = (unsigned char)RF_RxData[3] | (((uint16_t)RF_RxData[4]) << 8);
		  int16_t Left = Forward + Direction;
		  int16_t Right = Forward - Direction;

		  MotorSetValue(Left, Right);
		  break;
	  }
	  case PROG_CALLIBRATE: {
		  Callibrate(RF_RxData[1]);
		  SendReturn(0);
		  break;
	  }
	  case MODE_ACCELERATION_BURST: {
		  mode = MODE_ACCELERATION_BURST;
		  break;
	  }
	  case MODE_VELOCITY_BURST: {
		  mode = MODE_VELOCITY_BURST;
		  break;
	  }
	  case MODE_DISTANCE_BURST: {
		  mode = MODE_DISTANCE_BURST;
		  break;
	  }
	  case PROG_MOVE_BREAK: {
		  // PROG - START - ACCEL - MAX - DIST - STOP
		  short SSpeed = (unsigned char)RF_RxData[1] | (((uint16_t)RF_RxData[2]) << 8);
		  short ASpeed = (unsigned char)RF_RxData[3] | (((uint16_t)RF_RxData[4]) << 8);
		  short MSpeed = (unsigned char)RF_RxData[5] | (((uint16_t)RF_RxData[6]) << 8);
		  short Dist = (unsigned char)RF_RxData[7] | (((uint16_t)RF_RxData[8]) << 8);
		  short ReturnDist = 0;
		  short Stop = (unsigned char)RF_RxData[9] | (((uint16_t)RF_RxData[10]) << 8);
		  double Rot;
		  double P = 0.05, I = 0.2;
		  Callibrate(255);
		  Update();
		  if (Dist > 0){
			  while(ReturnDist < Dist){
				  Rot = P * Acceleration[5] + I * Velocity[5]/1000000.0;
				  if (SSpeed < MSpeed) {
					  MSpeed -= ASpeed;
				  }
				  MotorSetValue(-MSpeed -Rot, -MSpeed + Rot);
				  ReturnDist = Distance[0]/1000000000000;
				  Update();
				  if ((Acceleration[0] > Stop) || (Acceleration[1] > Stop) || (Acceleration[2] > Stop)){
					  break;	// Uderzenie
				  }
			  }
		  } else {
			  while(ReturnDist > Dist){
				  Rot = P * Acceleration[5] + I * Velocity[5]/1000000.0;
				  if (SSpeed < MSpeed) {
					  MSpeed -= ASpeed;
				  }
				  MotorSetValue(MSpeed -Rot, MSpeed + Rot);
				  ReturnDist = Distance[0]/1000000000000;
				  Update();
				  if ((Acceleration[0] > Stop) || (Acceleration[1] > Stop) || (Acceleration[2] > Stop)){
					  break;	// Uderzenie
				  }
			  }
		  }
		  MotorSetValue(0, 0);
		  SendReturn(ReturnDist);
		  break;
	  }/*
	  if (Dist > 0){
			  while(ReturnDist < Dist){
				  Rot = P * Acceleration[5] + I * Velocity[5]/1000000.0;
				  if (SSpeed < MSpeed) {
					  SSpeed += ASpeed;
				  }
				  MotorSetValue(-SSpeed -Rot, -SSpeed + Rot);
				  ReturnDist = Distance[0]/1000000000000;
				  Update();
				  if ((Acceleration[0] > Stop) || (Acceleration[1] > Stop) || (Acceleration[2] > Stop)){
					  break;	// Uderzenie
				  }
			  }
		  } else {
			  while(ReturnDist > Dist){
				  Rot = P * Acceleration[5] + I * Velocity[5]/1000000.0;
				  if (SSpeed < MSpeed) {
					  SSpeed += ASpeed;
				  }
				  MotorSetValue(SSpeed -Rot, SSpeed + Rot);
				  ReturnDist = Distance[0]/1000000000000;
				  Update();
				  if ((Acceleration[0] > Stop) || (Acceleration[1] > Stop) || (Acceleration[2] > Stop)){
					  break;	// Uderzenie
				  }
			  }
		  }
		  MotorSetValue(0, 0);
		  SendReturn(ReturnDist);
		  break;
	  }*/
	  case PROG_ROTATE: {
		  // PROG - START - ACCEL - MAX - ANGLE
		  short SSpeed = (unsigned char)RF_RxData[1] | (((uint16_t)RF_RxData[2]) << 8);
		  short ASpeed = (unsigned char)RF_RxData[3] | (((uint16_t)RF_RxData[4]) << 8);
		  short MSpeed = (unsigned char)RF_RxData[5] | (((uint16_t)RF_RxData[6]) << 8);
		  short Angle = (unsigned char)RF_RxData[7] | (((uint16_t)RF_RxData[8]) << 8);
		  short ReturnAngle = 0;
		  Callibrate(255);
		  Update();
		  if (Angle > 0){
			  while(ReturnAngle < Angle){
				  if (SSpeed <= MSpeed) {
					  MotorSetValue(MSpeed, -MSpeed);
					  MSpeed -= ASpeed;
				  }
				  ReturnAngle = Velocity[5]/1000000;
				  Update();
			  }
		  } else {
			  while(ReturnAngle > Angle){
				  if (SSpeed <= MSpeed) {
					  MotorSetValue(-MSpeed, MSpeed);
					  MSpeed -= ASpeed;
				  }
				  ReturnAngle = Velocity[5]/1000000;
				  Update();
			  }
		  }
		  MotorSetValue(0, 0);
		  SendReturn(ReturnAngle);
		  break;
	  }
	  }

	  if (Flag == 1) {
		  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		  //char c[3] = {'<', '-'};
		  //SendInt(Counter);
		  //Serial_Send(c, 2);
		  //Counter = 0;
		  Flag = 0;
	  }
	  //HAL_Delay(1);
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7200-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = MOTOR_PRESCALLER;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = MOTOR_COUNTER;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CSN_Pin|CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CSN_Pin CE_Pin */
  GPIO_InitStruct.Pin = CSN_Pin|CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
