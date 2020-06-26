
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * source code by hatab . Lutsk,UA'2019
  * Scales v2
  ******************************************************************************
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
//#include "float.h"
//#include "stdio.h"
#include "string.h"
#include "hx711.h"
#include "TM1638.h"
#include "eeprom.h"
#include "dwt_delay.h"
//#include "lcd.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osSemaphoreId myBinarySem01Handle;

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;
osThreadId KeyboardTaskHandle;
HX711 hx711_config;
const char* INFO_PAGE = {"Scales v2.0, by Sergik (06.2020)\n\r\n\r"
		"usage:\n\r"
		"INFO        -this page\n\r"
		"CONFIG      -get config data\n\r"
		"BULK_ON     -bulk mode on\n\r"
		"BULK_OFF    -bulk mode off\n\r"
		"TARE        -taring scales (ZERO)\n\r"
		"AUTO_TAR:x  -auto taring mode (x=0or!0)\n\r"
		"TARE1KG     -taring with 1kg\n\r"
		"TARE2KG     -taring with 2g\n\r"
		"TARE_ETALON -change etalon weight for taring (in process...)\n\r"
		"SAVE_CFG    -store cal.data to EEPROM\n\r"
		"Send data to host DISABLED !\nSend any command to continue.\n"};

const char* CONFIG_STRING = {"BULK_ON:%01u;AUTO_TAR:%01u;CFG1:%06lu;CFG2:%06lu;ZERO:%06lu;TARE_WEIGHT:%03lu;CURRENT_WEIGHT:%05lu\nSend data to host DISABLED !\nSend any command to continue.\n"};
const char* TARE1KG_OK = {"TARE1KG:OK, UNITS:%06lua\n\r"};
const char* TARE2KG_OK = {"TARE2KG:OK, UNITS:%06lua\n\r"};

uint8_t snd_flag = 0;
uint8_t bulk_mode = 0;
uint8_t auto_taring_mode = 0;
uint16_t taring_etalon_weight = 1000.0f;
uint8_t old_flag_stab ,flag_stab;
uint8_t old_flag_usb ,flag_usb ;
uint8_t key = 0;
uint8_t cnt_auto_tare = 0;
uint8_t cnt_show_weight = 0;
uint8_t cnt_value_read = 0;
uint8_t us_const1 = 20;
//
// 20kg china sensor:BULK_ON:0;AUTO_CAL:0;CFG1:336996;CFG2:350075;ZERO:323121;TARE_WEIGHT:1000
// 50kg Zemic :BULK_ON:0;AUTO_CAL:0;CFG1:434080;CFG2:442853;ZERO:425397;TARE_WEIGHT:1000
//
int32_t RealZeroLevel = 478191;
int32_t ZeroLevel = 323121;
int32_t Real1kgLevel = 568495;
int32_t Real2kgLevel = 659005;
int32_t _1kgLevel = 336996;
int32_t _2kgLevel = 350075;
int32_t value;
int32_t weight_array[MAX_WEIGHT_ARRAY];
int32_t DataStore[NUM_CAL];// ={0,1,2,3};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void StartKeyboardTask(void const * argument);
void beep(uint16_t i);
void WriteDefaultsConstantToEEP(void);
void ReadConstantsFromEEP(void);
void TareScale();
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
//
// SPI low level functions
//
void spi_Send(uint8_t data) {
	HAL_SPI_Transmit(&hspi1, &data, 1, 1000);
}

void spi_Read_Reg( uint8_t data_mas[]) {//+
	uint8_t key_Reg_Adr =0x42;

	SPI1->CR1 |= SPI_CR1_BIDIOE;
	HAL_SPI_Transmit(&hspi1,&key_Reg_Adr , 1, 1000);

	SPI1->CR1 &= ~SPI_CR1_BIDIOE;
	HAL_SPI_Receive(&hspi1, data_mas, 4, 1000);
	SPI1->CR1 |= SPI_CR1_BIDIOE;
}
//
// EEPROM api functions
//
void ReadEEprom(void) {
	uint32_t data_;
	EE_Read(EEP_CHECK_ADDR,&data_);
	if (data_!= CHECK_DATA) {
		beep(500);
		HAL_Delay(50);
		beep(100);
		HAL_Delay(50);
		beep(100);
		HAL_Delay(50);
		beep(100);
		EE_Format();
		WriteDefaultsConstantToEEP();
		EE_Write(EEP_CHECK_ADDR,CHECK_DATA);
	 } else {
		 beep(10);
		 ReadConstantsFromEEP();
	 }
}

void WriteDefaultsConstantToEEP(void) {
	EE_Write(EE_1kgLevel, _1kgLevel); //
	EE_Write(EE_2kgLevel, _2kgLevel); //
	EE_Write(EE_ZeroLevel, ZeroLevel); //
	EE_Write(EE_BulkMode, bulk_mode); //
	EE_Write(EE_AutoCalMode,auto_taring_mode);
	EE_Write(EE_TareWeight,taring_etalon_weight);
	EE_Write(EE_UsConstant1,us_const1); // constant for delay in low level function for hx711
}

void ReadConstantsFromEEP(void) {
	uint32_t data_;
	EE_Read(EE_1kgLevel, &data_); _1kgLevel = data_;
	EE_Read(EE_2kgLevel,&data_); _2kgLevel = data_;
	EE_Read(EE_ZeroLevel,&data_); ZeroLevel = data_;
	EE_Read(EE_TareWeight,&data_); taring_etalon_weight = data_;
	EE_Read(EE_BulkMode,&data_); bulk_mode = data_;
	EE_Read(EE_AutoCalMode,&data_); auto_taring_mode = data_; //

	Real1kgLevel = _1kgLevel ;
	Real2kgLevel = _2kgLevel ;
	RealZeroLevel =  ZeroLevel ;
}

// hx711
int32_t ReadCount(void) {
    return HX711_Value(hx711_config) - 8000000;
}

int32_t GetMiddleFilterValue(int32_t rc) {
	uint32_t max = 0;
	uint32_t min = 0;
	uint32_t sum = 0;
	uint8_t i = 0;

	max = min = sum = DataStore[0] = rc;

	for(i=NUM_CAL-1; i!= 0; i--) {
		if(DataStore[i] > max)
			max = DataStore[i];
		else if(DataStore[i] < min)
			min = DataStore[i];

		sum = sum + DataStore[i];
		DataStore[i] = DataStore[i - 1];
	}

	i = NUM_CAL - 2;
	sum = sum - max - min;// + i/2;
	sum = sum/i;

	return sum;
}

// tm1638
void DynamicBright(uint8_t num) {
	for (uint8_t j=1;j<num;j++) {
		osDelay(TM1638_DELAY);
		for (uint8_t i=MAX_SEGMENTS+1;i>0;i--) {
			if (myBinarySem01Handle != pdFALSE) {
				xSemaphoreTake(myBinarySem01Handle, portMAX_DELAY );
				tm1638_Bright(i);
				osDelay(TM1638_DELAY);
				xSemaphoreGive(myBinarySem01Handle );
			}
		}
		for (uint8_t i=0;i<MAX_SEGMENTS+1;i++) {
			if (myBinarySem01Handle != pdFALSE) {
				xSemaphoreGive(myBinarySem01Handle );
				tm1638_Bright(i);
				osDelay(TM1638_DELAY);
				xSemaphoreGive(myBinarySem01Handle );
			}
		}
	}
}

// other UI functions
void beep(uint16_t i) {
	HAL_GPIO_WritePin(BUZZER_GPIO_Port,BUZZER_Pin,GPIO_PIN_SET);
	HAL_Delay(i);
	HAL_GPIO_WritePin(BUZZER_GPIO_Port,BUZZER_Pin,GPIO_PIN_RESET);
}

void ShowVar(uint32_t data) {
	if (myBinarySem01Handle != pdFALSE) {
		xSemaphoreTake(myBinarySem01Handle, portMAX_DELAY );
		tm1638_Clear(0);
		osDelay(TM1638_DELAY);
		tm1638_Digit( data ,1);
		osDelay(200);
		DynamicBright(10);
		xSemaphoreGive(myBinarySem01Handle);
	}
}

void TareScale() {
	int32_t Values = 0;
	for (uint8_t i = 0;i<NUM_CAL;i++) Values += GetMiddleFilterValue(ReadCount());
	RealZeroLevel = Values / NUM_CAL;
	ZeroLevel = GetMiddleFilterValue(ReadCount());

	uint32_t Delta = RealZeroLevel - ZeroLevel;
	Real1kgLevel =	_1kgLevel + Delta;
	Real2kgLevel = _2kgLevel + Delta;
}

uint32_t WeightCorrect(uint32_t _data) {
	uint32_t weight = (Real2kgLevel!=Real1kgLevel) ? (float)(_data - RealZeroLevel) / (float)(Real2kgLevel - Real1kgLevel) * TARE_WEIGHT : 0.0f;
	if (( (((uint8_t)weight >0) & ((uint8_t)weight<WEIGHT_LIMIT_FOR_TARE)) | (weight<0)) & (auto_taring_mode)) { // Check for auto calibration mode
		cnt_auto_tare++;
		if (cnt_auto_tare>CNT_AUTO_TARE) { // how times to read float weight
			if (snd_flag) beep(1);
			cnt_auto_tare = 0;
			TareScale();
			weight = (Real2kgLevel!=Real1kgLevel) ? (float)(_data - RealZeroLevel) / (float)(Real2kgLevel - Real1kgLevel) * TARE_WEIGHT : 0.0f;
			osDelay(AFTER_TARE_DELAY);
		}
	}
	if (weight > 99999 || weight < 0 ) {
		//if (snd_flag) beep(1);
		return 0;
	} else return weight;
}

// function return flag:
// 	0 - weight not change
// 	1 - weight stabilized
// 	2 - weight zero
//	3 - weight in stab progress
uint8_t CheckChangeWeight(uint32_t weight) {
    for (uint8_t i = MAX_WEIGHT_ARRAY-1;i>0;i--) weight_array[i] = weight_array[i-1];
    weight_array[0] = weight;

    if ( (weight_array[0] > weight_array[MAX_WEIGHT_ARRAY-1]-1) & (weight_array[0] < weight_array[MAX_WEIGHT_ARRAY-1]+5) ) return (WEIGHT_STATUS_STABILIZED);
    else return (WEIGHT_STATUS_STAB_PROGRESS);
}

void StartKeyboardTask(void const * argument ) {
	uint8_t debonce_Threshold =5;
	uint8_t debonce_Counter = 0;
	uint8_t data = 0;

	for(;;)	{
		if (myBinarySem01Handle != pdFALSE) {
			xSemaphoreTake(myBinarySem01Handle, portMAX_DELAY );
			data = tm1638_ReadKeys();
			osDelay(TM1638_DELAY);
			xSemaphoreGive(myBinarySem01Handle);
		}
		switch (data) {
			case 0x00:
				debonce_Counter=0;
				key = 0;
				break;
			case 0x01:
				if (debonce_Counter < debonce_Threshold) debonce_Counter++; else key = 1;
				break;
			case 0x02:
				if(debonce_Counter<debonce_Threshold)debonce_Counter++; else key = 3;
				break;
			case 0x10:
				if(debonce_Counter<debonce_Threshold)debonce_Counter++; else key = 2;
				break;
			case 0x20:
				if(debonce_Counter<debonce_Threshold)debonce_Counter++; else key =  4;
				break;
		}
		osDelay(1);
	}
}

void SendWeightToUsb(uint32_t weight) {
	char answer[25];

	if (old_flag_usb == 0) {
		snprintf(answer, sizeof(answer),"WEIGHT:%05lu\n\r",weight);
		CDC_Transmit_FS(answer, strlen(answer));
		CDC_Transmit_FS('\0', 0);
	}
}

void ShowWeight(uint32_t _data,bool check) {
	uint32_t weight;
	weight = WeightCorrect(_data);

	if (bulk_mode==1) SendWeightToUsb(weight);

	flag_stab = CheckChangeWeight(weight);
    if (flag_stab == WEIGHT_STATUS_STABILIZED) {
    	if (old_flag_stab != flag_stab) {
    		old_flag_stab = WEIGHT_STATUS_STABILIZED;
    		if (myBinarySem01Handle != pdFALSE) {
        		xSemaphoreGive(myBinarySem01Handle );
        		tm1638_Clear(0);
        		osDelay(TM1638_DELAY);
        		tm1638_Digit(weight ,1);
        		xSemaphoreGive(myBinarySem01Handle);
    		}
    	}
    	if (bulk_mode == 0)
    		if (old_flag_usb == 0) SendWeightToUsb(weight);
    } else  old_flag_stab =WEIGHT_STATUS_IDLE;
    osDelay(AFTER_SHOW_WEIGHT_DELAY);
}

void ProcessUsbFlag(void) {
	char str[255];

	switch (flag_usb ) {
	case TARE_FLAG:		// Taring scales
		beep(3);
		osDelay(55);
		beep(7);
		TareScale();
		snprintf(str, strlen(str),"TARE_ZERO:OK, UNITS:%06lua\n\r",ZeroLevel);
		CDC_Transmit_FS(str, strlen(str));
		flag_usb = old_flag_usb = 0;
		break;
	case WEIGHT_FLAG: // Send original units value data to the host
		value = ReadCount();
		beep(2);
		SendWeightToUsb(WeightCorrect(value));
		flag_usb = old_flag_usb=0;
		break;
	case CONFIG_FLAG:	// Send Configuration (conatants, current values) to the host
		if (old_flag_usb != flag_usb ) {
			beep(2);
			uint32_t temp = RealZeroLevel;
			ReadConstantsFromEEP();
			snprintf(str, strlen(CONFIG_STRING),CONFIG_STRING,bulk_mode,auto_taring_mode,_1kgLevel,_2kgLevel,ZeroLevel,taring_etalon_weight,WeightCorrect(value));
			RealZeroLevel = temp;

			CDC_Transmit_FS(str, strlen(str));
			CDC_Transmit_FS('\0', 0);
			bulk_mode = 0;
			old_flag_usb = flag_usb;
		}
		break;
	case INFO_FLAG:		// About page
		if (old_flag_usb != flag_usb ) {
			beep(5);
			osDelay(55);

			CDC_Transmit_FS(INFO_PAGE, strlen(INFO_PAGE));
			CDC_Transmit_FS('\0', 0);
			bulk_mode = 0;
			old_flag_usb = flag_usb;
		}
		break;
	case TARE1KG_FLAG:	// Taring with fixed 1xxx gr weight
		beep(20);
		value = ReadCount();
		_1kgLevel =  GetMiddleFilterValue(value); // 1kg

		snprintf(str, sizeof(str),TARE1KG_OK,value);
		CDC_Transmit_FS(str, strlen(str));
		CDC_Transmit_FS('\0', 0);
		flag_usb = old_flag_usb = 0;
		break;
	case TARE2KG_FLAG:	// Taring with fixed 2xxx gr weight
		beep(20);
		value = ReadCount();
		_2kgLevel =  GetMiddleFilterValue(value); // 2kg
		snprintf(str, sizeof(str),TARE2KG_OK,value);
		CDC_Transmit_FS(str, strlen(str));
		CDC_Transmit_FS('\0', 0);
		flag_usb = old_flag_usb = 0;
		break;
	case SAVE_CFG_FLAG:
		beep(5);
		osDelay(100);
		beep(5);
		osDelay(100);
		beep(5);

		EE_Write(EE_1kgLevel, _1kgLevel); //
		EE_Write(EE_2kgLevel, _2kgLevel); //
		EE_Write(EE_ZeroLevel, ZeroLevel); //
		EE_Write(EE_AutoCalMode, auto_taring_mode); //
		EE_Write(EE_TareWeight, taring_etalon_weight); //

		CDC_Transmit_FS("SAVED:OK\n\r", 10);
		CDC_Transmit_FS('\0', 0);
		flag_usb = old_flag_usb = 0;
		break;
	case BULK_ON_FLAG:
		beep(2);
		bulk_mode = 1;
		flag_usb = old_flag_usb = 0;

		CDC_Transmit_FS("BULK_ON:OK\n\r",12 );
		CDC_Transmit_FS('\0', 0);
		EE_Write(EE_BulkMode, 1);
		break;
	case BULK_OFF_FLAG:
		beep(2);
		bulk_mode = 0;
		flag_usb = old_flag_usb = 0;

		CDC_Transmit_FS("BULK_OFF:OK\n\r", 13);
		CDC_Transmit_FS('\0', 0);
		EE_Write(EE_BulkMode, 0);
		break;
	case SCALES_FLAG:
		beep(2);
		auto_taring_mode = 0;
		CDC_Transmit_FS((uint8_t*)"SCALES\n", 7);
		CDC_Transmit_FS('\0', 0);
		break;
	case AUTOTAR1_FLAG:
		beep(2);
		auto_taring_mode = 1;
		CDC_Transmit_FS((uint8_t*)"AUTO_TAR:1\n\rPlease Save Me !\n\r", (uint8_t)30);
		CDC_Transmit_FS('\0', 0);
		break;
	case AUTOTAR0_FLAG:
		beep(2);
		auto_taring_mode = 0;
		CDC_Transmit_FS((uint8_t*)"AUTO_TAR:1\n\rPlease Save Me !\n\r", (uint8_t)30);
		CDC_Transmit_FS('\0', 0);
		break;
	case CONTINUE_FLAG:
		beep(2);
		flag_usb = 0;
		old_flag_usb=0;
		break;
	} // end switch
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  hx711_config.gpioSck = GPIOB;// GPIOB;
  hx711_config.gpioData = GPIOB; //GPIOB;
  hx711_config.pinSck = GPIO_PIN_11; // GPIO_PIN_9;
  hx711_config.pinData = GPIO_PIN_10; // GPIO_PIN_8;
  hx711_config.gain = 1;
  hx711_config.offset = 0;
  beep(1);
  hx711_config = HX711_Tare(hx711_config,20);
  ReadEEprom();
  beep(1);

  tm1638_Init(7);
  tm1638_Digit(0,1);
  beep(10);
  tm1638_Clear(0);
  tm1638_Bright(5);
  tm1638_Digit(12345,0);
  HAL_Delay(150);

  for (uint8_t i = 7;i>0;i--) {
	  led_Set(i, 1);
	  HAL_Delay(50);
	  tm1638_Bright(i);
  }

  DWT_Init();
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of myBinarySem01 */
  osSemaphoreDef(myBinarySem01);
  myBinarySem01Handle = osSemaphoreCreate(osSemaphore(myBinarySem01), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */

  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadDef(KeyboardTask, StartKeyboardTask, osPriorityNormal, 0, 256);
  KeyboardTaskHandle = osThreadCreate(osThread(KeyboardTask), NULL);

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TM1638_STB_Pin|USB_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BUZZER_Pin|LED_Pin|HX711_SCK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : TM1638_STB_Pin */
  GPIO_InitStruct.Pin = TM1638_STB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TM1638_STB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUZZER_Pin LED_Pin HX711_SCK_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin|LED_Pin|HX711_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : HX711_DATA_Pin */
  GPIO_InitStruct.Pin = HX711_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(HX711_DATA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_EN_Pin */
  GPIO_InitStruct.Pin = USB_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_EN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 5 */
  HAL_GPIO_WritePin(USB_EN_GPIO_Port,USB_EN_Pin,SET);
  /* Infinite loop */
	for(;;)	{
		value = GetMiddleFilterValue(ReadCount());
		ShowWeight(value,true);

		if (flag_usb) ProcessUsbFlag();

		switch (key) {
		case 1:
			beep(5);
			osDelay(100);
			TareScale();
			CDC_Transmit_FS("TARE:OK\n\r", 9);
			CDC_Transmit_FS('\0', 0);
			key=0;
			break; // Set ZERO (weight placed on the scales must be null)
		case 2:
			beep(5);
			osDelay(100);
			_1kgLevel =  value; // 100g
			CDC_Transmit_FS("TARE_1KG:OK\n\r", 9);
			CDC_Transmit_FS('\0', 0);
			key=0;
			break; // Set 100g LEVEL (weight on scales must equ 100 gramm )
		case 3:
			beep(5);
			osDelay(100);
			_2kgLevel =  value; // 200g
			CDC_Transmit_FS("TARE_2KG:OK\n\r", 9);
			CDC_Transmit_FS('\0', 0);
			key=0;
			break; // Set 200g LEVEL (weight on scales must equ 200 gramm )
		case 4:
			beep(5);
			osDelay(100);
			beep(5);
			WriteDefaultsConstantToEEP();
			for (uint8_t i = 1; i<MAX_SEGMENTS-1; i++) {
				if (myBinarySem01Handle != pdFALSE) {
					xSemaphoreTake(myBinarySem01Handle, portMAX_DELAY );
					tm1638_Clear(0);
					tm1638_Digit(0 ,i);
					osDelay(50);
					xSemaphoreGive(myBinarySem01Handle );
				}
			}
			for (uint8_t i = MAX_SEGMENTS; i>0; i--) {
				if (myBinarySem01Handle != pdFALSE) {
					xSemaphoreTake(myBinarySem01Handle, portMAX_DELAY );
					tm1638_Clear(0);
					tm1638_Digit(8 ,i);
					osDelay(50);
					xSemaphoreGive(myBinarySem01Handle );
				}
			}
			CDC_Transmit_FS("SAVE_CFG:OK\n\r", 9);
			CDC_Transmit_FS('\0', 0);

			key=0;
			break;
		default: // show
		{}
		} // end switch
		osDelay(1);
	}
  /* USER CODE END 5 */ 
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
