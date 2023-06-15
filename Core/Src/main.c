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
  * This LIDAR example source code requires following equipment:
  * - TDC Device under test, e.g. AS6500
  * - Waveform Generator (W1/W2), (tx) triggered by STM32 and
  * 						(Rx) generates amount of pulses
  *	Used connection(s):
  *	- STM32 GPIO (PC3) --> AS6500 Trigger (RSTIDX)
  *	- STM32 GPIO (PC2) --> WaveFrom Trigger (T1)
  *	- Waveform (W1)    --> STOP2
  *	- WaveFrom (W2)    --> STOP1
  *	- Probe (1)        --> STOP2
  *	- Probe (2)        --> STOP1
  *	Optional connection(s):
  *	- STM32 GPIO (PC0) --> Waveform (DIO 5) - Debugging, showing Post Processing
  *	- STM32 GPIO (PC1) --> Waveform (DIO 6) - Simulate Noise Mask, Valid Data and Timeout Sequence
  *
  *	SPI Interface:
  * - STM32 GPIO (GND) --> (GND)
  * - STM32 GPIO (PA5) --> Waveform (DIO 1) - SCK
  * - STM32 GPIO (PA6) --> Waveform (DIO 3) - MISO
  * - STM32 GPIO (PA7) --> Waveform (DIO 2) - MOSI
  * - STM32 GPIO (PB6) --> Waveform (DIO 0) - SSN
  *
  *	- STM32 GPIO (PA9) --> Waveform (DIO 4) - INTN
  *
  * The mode of using SPI interface can be chosen with parameter My_SPI_Choice
  *  - 0 = HAL SPI     (default setting)
  *  - 1 = HAL DMA     (using DMA controller, but insufficient usage of SSN)
  *  - 2 = My DMA      (usage of simplified DMA, including SSN handling)
  *  - 3 = Combination (the try to use HAL SPI and HAL DMA together)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "user_spi_dma_interface.c"
#include "user_spi_interface.c"
#include "user_tools.c"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Opcode Hex / BIN Description / One Byte
#define spiopc_power        0x30	// = 0b00110000 Power on reset and stop measurement
#define spiopc_init         0x18	// = 0b00011000 Initializes Chip and starts measurement
#define spiopc_write_config 0x80	// = 0b100XXXXX 0x60 = 0b011XXXXX
#define spiopc_read_results 0x60	// = 0b011XXXXX Read opcode for result and status register X=8..31
#define spiopc_read_config  0x40	// = 0b010XXXXX Readout of configuration register X=0..16

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

// AS6500 Trigger (GPIO C3) --> RSTIDX
#define TDC_RSTIDX_PULSE {\
	AS6500_Trigger_GPIO_Port->BSRR = (uint32_t)AS6500_Trigger_Pin;\
	AS6500_Trigger_GPIO_Port->BRR = (uint32_t)AS6500_Trigger_Pin;\
	}

// Triggering WaveForm Generator via GPIO
// WaveFrom Trigger (GPIO C2) --> STOP1
#define LASER_TRIGGER_PULSE {\
	WaveForm_Trigger_GPIO_Port->BSRR = (uint32_t)WaveForm_Trigger_Pin;\
	WaveForm_Trigger_GPIO_Port->BRR = (uint32_t)WaveForm_Trigger_Pin;\
	}

#define DEBUG_PC0 {\
	Debugging_Pin_GPIO_Port->BSRR = (uint32_t)Debugging_Pin_Pin;\
	Debugging_Pin_GPIO_Port->BRR = (uint32_t)Debugging_Pin_Pin;\
	}

#define DEBUG_PC0_HIGH (Debugging_Pin_GPIO_Port->BSRR = (uint32_t)Debugging_Pin_Pin)

#define DEBUG_PC0_LOW (Debugging_Pin_GPIO_Port->BRR = (uint32_t)Debugging_Pin_Pin)

#define DEBUG_PC1_HIGH (Debugging_ValidData_GPIO_Port->BSRR = (uint32_t)Debugging_ValidData_Pin)

#define DEBUG_PC1_LOW (Debugging_ValidData_GPIO_Port->BRR = (uint32_t)Debugging_ValidData_Pin)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

/* USER CODE BEGIN PV */

/* empty configuration*/
uint8_t CFG_Empty [] = {
		0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF
};

/* default configuration*/
uint8_t CFG_Registers [] = {
		0x31, 0x01, 0x1F, 0x40,
		0x0D, 0x03, 0xC0, 0x53,
		0xA1, 0x13, 0x00, 0x0A,
		0xCC, 0xCC, 0x31, 0x8E,
		0x04, 0x00, 0x00, 0x00
};

/* 5MHzRef_on_board config */
uint8_t CFG_DUT_Registers [] = {
		0x83, 0x03, 0x1D, 0x20,  //CFG 0 - PIN_ENA_RSTIDX bit[7] instead of 0x03 -->0x83
		0x4E, 0x00, 0xC0, 0xD3,  //CFG 3,4,5 - REFCLK_DEVISION = 20000
		0xA1, 0x13, 0x00, 0x0A,
		0xCC, 0xCC, 0xF1, 0x7D,
		0x04, 0x00, 0x00, 0x00
};

uint8_t CFG_DUT_Registers_DMA [] = {
		0x80,                    //spiopc_write_config
		0x83, 0x03, 0x1D, 0x20,  //CFG 0 - PIN_ENA_RSTIDX bit[7] instead of 0x03 -->0x83
		0x4E, 0x00, 0xC0, 0xD3,  //CFG 3,4,5 - REFCLK_DEVISION = 20000
		0xA1, 0x13, 0x00, 0x0A,
		0xCC, 0xCC, 0xF1, 0x7D,
		0x04, 0x00, 0x00, 0x00
};

volatile uint8_t My_SPI_Choice = 2; // 0 = HAL SPI // 1 = HAL DMA // 2 = My DMA // 3 = Combination
uint8_t mh_size = 18;

/* using cycle counter */
unsigned long t1 = 0; // current tick

float noisemask = 0.0;
float timeout = 0.0;
float postprocess = 0.0;
float trigger = 0.0;

int timeout_ONE = 0;
int process_DONE = 0;

static uint8_t Read_REG [20];

volatile uint32_t My_INTN_Counter = 0;
volatile uint8_t My_INTN_State = 1;
volatile uint8_t My_Valid_Data_State = 0;

volatile uint8_t MyResetIndex = 1;

uint32_t RD_Trigger = 0;

uint32_t timeout_count = 12800;
uint32_t RD_Timeout = 128;
volatile uint32_t Raw_Array[1000][4];

volatile float Last_REFID1 = 0, Last_REFID2 = 0;
volatile float Last_TSTOP1 = 0, Last_TSTOP2 = 0;

#define max_index 100 //defines the maximum of expected hits (e.g. 32)

volatile float DiffTSTOP[max_index] , DiffREFID[max_index] ;
volatile float Final_CALC[max_index];

volatile float My_Hit_01 = 0;
volatile float My_Hit_02 = 0;
volatile float My_Last_Hit = 0;
volatile uint32_t My_Clear_FIFO = 0;


volatile uint8_t Valid_TSTOP1 = 0, Valid_TSTOP2 = 0, Calc_Diff = 0;

uint32_t fifo_size = 0;

uint32_t FIFO_Expect_Burst = 32; // number of expected STOP pulses, max.

volatile uint32_t Calc_Index = 0; 		// Index of calculated arrays
volatile uint32_t WR_Index = 0; // Write into Raw_Array[]
volatile uint32_t RD_Index = 0; // Read from Raw_Array[]

//uint32_t REFCLK_DIVISIONS = 200000; //REF Clock = 5 MHz, Period = 200ns
volatile uint32_t REFCLK_DIVISIONS = 20000;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef * hspi);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef * hspi);

void POST_Processing(void);
void Trigger_Generation(void);

void HAL_SPI_MY_TransmitReceive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);
void HAL_SPI_MY_Transmit_DMA(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint16_t Size);


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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  ENABLE_CYCLE_COUNTER;

  noisemask = TIME_MICRO(100);
  timeout = TIME_MICRO(1000);
  postprocess = TIME_MILLI(2.5);
  trigger = TIME_MILLI(50);

  uint8_t TX_Array[1];

  // Illustrates the different read/write sequences
  switch(My_SPI_Choice){
  case 0:
	  // with HAL SPI - works
	  Write_Opcode(spiopc_power); // --> INTN goes high
	  Delay_100us(1); // 100us, Delay between power-on or initialisation reset and next communication
	  Write_Byte_Auto_Incr_Lite(spiopc_write_config, 0, CFG_DUT_Registers, 16);
	  Read_Byte_Auto_Incr_Lite(spiopc_read_config, 0, Read_REG, 16);
	  Write_Opcode(spiopc_init); // --> INTN goes low
	  break;
  case 1:
	  // with HAL DMA - doesn't work, due to insufficient handling of SSN (HARDWARE/SOFTWARE)
	  TX_Array[0] = spiopc_power;
	  SSN_GPIO_Port->BRR = (uint32_t)SSN_Pin;
	  HAL_SPI_Transmit_DMA(&hspi1, TX_Array, 1); //HAL_SPI_TxCpltCallback (max size = 2)
	  Delay_100us(1);
	  SSN_GPIO_Port->BRR = (uint32_t)SSN_Pin;
	  HAL_SPI_Transmit_DMA(&hspi1, CFG_DUT_Registers_DMA, mh_size); //18
	  HAL_DMA_Abort(&hdma_spi1_tx);
	  SSN_GPIO_Port->BSRR = (uint32_t)SSN_Pin;
	  CFG_Empty[0]=spiopc_read_config;
	  SSN_GPIO_Port->BRR = (uint32_t)SSN_Pin;
	  HAL_SPI_TransmitReceive_DMA(&hspi1, CFG_Empty, Read_REG, 18);
	  TX_Array[0] = spiopc_init;
	  SSN_GPIO_Port->BRR = (uint32_t)SSN_Pin;
	  HAL_SPI_Transmit_DMA(&hspi1, TX_Array, 1); // _MH: INTN is toggling ?!?
	  break;
  case 2:
	  // with MY DMA - works
	  TX_Array[0] = spiopc_power;
	  TX_Array[0] = 0;
	  My_Transmit_DMA(TX_Array, 1);
	  Delay_100us(1);
	  My_Transmit_DMA(CFG_DUT_Registers_DMA, 18);
	  CFG_Empty[0]=spiopc_read_config;
	  My_TransmitReceive_DMA(CFG_Empty, Read_REG, 18);
	  TX_Array[0] = spiopc_init;
	  TX_Array[0] = 0;
	  My_Transmit_DMA(TX_Array, 1); // _MH: INTN is toggling ?!?
	  break;
  case 3:
	  // combine without DMA and DMA -
	  Write_Opcode(spiopc_power); // --> INTN goes high
	  Delay_100us(1); // 100us, Delay between power-on or initialisation reset and next communication
	  Write_Byte_Auto_Incr_Lite(spiopc_write_config, 0, CFG_DUT_Registers, 16);
	  //Read_Byte_Auto_Incr_Lite(spiopc_read_config, 0, Read_REG, 16);
	  Read_Byte_Auto_Incr_Lite(spiopc_read_config, 0, Read_REG, 16);
	  CFG_Empty[0]=spiopc_read_config;
	  My_TransmitReceive_DMA(CFG_Empty, Read_REG, 18);
	  Write_Opcode(spiopc_init); // --> INTN goes low
	  break;
  default:
	  My_SPI_Choice = 0;
  }

  //Trigger_Generation();

  // TEST Loop
  // STOP2 -> Rx channel with 32 pulses
  // STOP1 -> Tx channel, Start Trigger with 20 Hz (period = 50 ms)
  // After evaluation, reset RSTIDX

  // Note for HAL_SPI_TransmitReceive()
  // It is needed to uses same array size for spiTx and spiRX
  // for example: 1x spiTX and 12x spiRX --> array size = 13
  uint8_t spiTX[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  uint8_t spiRX[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // Max. Reading 4x (REF_IDX[3 Bytes], TSTOP[3 Bytes]), plus 8 Bytes
  uint8_t RD_x_Bytes = 12;

  spiTX[0] = 0x68; // Opcode + Address

  uint8_t shift_index = 0;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  RD_Trigger++;

	  /*******************************************************************/
	  /* Improvement of Noise Mask Window                                */
	  /* SOFTWARE ENABLE (HIT_ENA_STOP1…4)                               */
	  /*   Setting the configuration bits HIT_ENA_STOP1 to HIT_ENA_STOP4 */
	  /*   applies a software enable for stop channels 1 to 4.           */
	  /*******************************************************************/
	  if ( (TIME_DIFF_TO(t1) > noisemask) && (timeout_ONE == 0) ) {
		  DEBUG_PC1_HIGH;
		  My_Valid_Data_State = 1;
	  }

	  // Read Loop as soon as INTN = 0
	  while ( (My_INTN_State == 0) )//&& (RD_Trigger > RD_Timeout) )
	  {
		  // Note: AS6500 Data Sheet, Section 8.8.4
		  //       FIFOs for Adapting Peak and Average Conversion Rate

		  spiTX[0] = 0x68;
		  shift_index = 1; // Related to TransmitReceive, Offset of Rx Array
		  CLEAR(spiRX); // clearing array take about 1.5us

		  switch(My_SPI_Choice){
		  case 0:
			  // HAL SPI
			  SSN_GPIO_Port->BRR = (uint32_t)SSN_Pin;		// 1. Put SSN low - Activate, SSN -> CLK = 3.6us
			  HAL_SPI_TransmitReceive(&hspi1, spiTX, spiRX, 1 + RD_x_Bytes, 10); // interbyte gaps = 2.2us
			  SSN_GPIO_Port->BSRR = (uint32_t)SSN_Pin;	// 4. Put SSN high - Deactivate, CLK -> SSN = 8us
			  break;
		  case 1: // reading does not work, due to insufficient handling of SSN (HARDWARE/SOFTWARE)
			  //HAL_SPI_DMAResume(&hspi1);
			  SSN_GPIO_Port->BRR = (uint32_t)SSN_Pin;

			  HAL_SPI_TransmitReceive_DMA(&hspi1, spiTX, spiRX, (uint16_t)(1 + RD_x_Bytes));
			  // readout incorrect!!

			  // Note:
			  /* It's called buffer overflow. Provider that spi_dma_tx is your MOSI message. Check
			   * your memory map if spi_dma_tx is allocated just after spi_dma_rx_. Use a watchpoint
			   * to break on all writes to spi_dma_tx. Are you sure that the fourth parameter of
			   * HAL_SPI_TransmitReceive_DMA is valid on both transmission and reception?
			   * Check the omitted routine after reception. */

			  break;
		  case 2:
			  // MY DMA
			  My_TransmitReceive_DMA(spiTX, spiRX, 1 + RD_x_Bytes);
			  break;
		  case 3: // reading does not work!!
			  //REFID1[ 8:10] TSTOP1 [11:13]
			  //REFID2[14:16] TSTOP2 [17:19]
			  //REFID3[20:22] TSTOP3 [23:25]
			  //REFID4[26:28] TSTOP4 [29:31]

			  SSN_GPIO_Port->BRR = (uint32_t)SSN_Pin;

			  HAL_SPI_Transmit_DMA(&hspi1, spiTX, 1);
			  shift_index=0;
			  HAL_SPI_Receive_DMA(&hspi1, spiRX, RD_x_Bytes);
			  // readout incorrect!!

			  break;
		  default:
			  My_SPI_Choice = 0;
		  }

		  /*****************************************************************/
		  /* Improvement of Valid Data State                               */
		  /* Verify calculated time difference with used Noise Mask Window */
		  /* and add only valid data into array                            */
		  /*****************************************************************/
		  if ( spiRX[shift_index+3] != 0xFF ||  //if (MSB of REFID1 != 0xFF)
				  (My_Valid_Data_State == 1)) { //My_Valid_Data_State == 1
			  // Concatenate of bytes (from MSB to LSB)
			  Raw_Array[WR_Index][0] = (spiRX[shift_index+0]<<16) + (spiRX[shift_index+1] <<8) + (spiRX[shift_index+2] );
			  Raw_Array[WR_Index][1] = (spiRX[shift_index+3]<<16) + (spiRX[shift_index+4] <<8) + (spiRX[shift_index+5] );
			  Raw_Array[WR_Index][2] = (spiRX[shift_index+6]<<16) + (spiRX[shift_index+7] <<8) + (spiRX[shift_index+8] );
			  Raw_Array[WR_Index][3] = (spiRX[shift_index+9]<<16) + (spiRX[shift_index+10]<<8) + (spiRX[shift_index+11]);

			  // Increase index of array to accumulate next value
			  WR_Index++;

	  	  }

	  } // End of WHILE() or IF(My_INTN_State==0)


	  if ( (TIME_DIFF_TO(t1) > timeout) && (timeout_ONE == 0) ) {
		  DEBUG_PC1_LOW;
		  timeout_ONE = 1; //set timeout, only once
		  My_Valid_Data_State = 0;
	  }


	  // Post Processing after reading new RAW Data, including timeout after last STOPx
	  if ( (RD_Index != WR_Index) && (TIME_DIFF_TO(t1) > postprocess) )
	  {
//-->
		  // Debugging (GPIO C0)
		  DEBUG_PC0_HIGH;

		  POST_Processing();
		  process_DONE = 1; // post processing is done!

		  My_Hit_01 = Final_CALC[0];
		  My_Hit_02 = Final_CALC[1];
		  if (Calc_Index > 0)
		  {
			  My_Last_Hit = Calc_Index;
		  }

//-->
		  // Debugging (GPIO C0)
		  DEBUG_PC0_LOW;

	  } // End of WHILE() or IF(My_INTN_State==1)


	  if (TIME_DIFF_TO(t1) > trigger)
	  {
		  // Reset Index
		  if (MyResetIndex != 0)
		  {
			  WR_Index = 0;
			  RD_Index = 0;
			  Calc_Index = 0;
		  }

		  Trigger_Generation();
		  RD_Trigger = 0;

		  // Set Cycle Counter t1
		  t1 = DWT->CYCCNT;
		  DEBUG_PC1_LOW;
		  timeout_ONE = 0; // clear timeout
		  process_DONE = 0; // clear postprocess
	  }


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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Debugging_Pin_Pin|Debugging_ValidData_Pin|WaveForm_Trigger_Pin|AS6500_Trigger_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SSN_GPIO_Port, SSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : Debugging_Pin_Pin Debugging_ValidData_Pin WaveForm_Trigger_Pin AS6500_Trigger_Pin */
  GPIO_InitStruct.Pin = Debugging_Pin_Pin|Debugging_ValidData_Pin|WaveForm_Trigger_Pin|AS6500_Trigger_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : INTN_Pin */
  GPIO_InitStruct.Pin = INTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SSN_Pin */
  GPIO_InitStruct.Pin = SSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SSN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/* stm32l4xx_hal_gpio.c */

	/* Prevent unused argument(s) compilation warning */
	UNUSED(GPIO_Pin);

	// Note: It takes about 1us after INTN
	//Debugging_Pin_GPIO_Port->BSRR = (uint32_t)Debugging_Pin_Pin; // SET
	//Debugging_Pin_GPIO_Port->BRR = (uint32_t)Debugging_Pin_Pin; // RESET

	if (GPIO_Pin == INTN_Pin) {
		My_INTN_State = (HAL_GPIO_ReadPin(INTN_GPIO_Port, INTN_Pin) == GPIO_PIN_SET); /* low active */
		if (My_INTN_State == 0) {
			My_INTN_Counter += 1;
		}
	}

	//HAL_GPIO_TogglePin(Debugging_Pin_GPIO_Port, Debugging_Pin_Pin);
}


/**
* @brief This function handles DMA1 Stream4 global interrupt.
*/

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    // TX-RX Done .. Do Something ...

	//SSN_GPIO_Port->BSRR = (uint32_t)SSN_Pin;			// Put SSN high - Deactivate
	// Debugging (GPIO C0)
	DEBUG_PC0;
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef * hspi)
{
    // TX-RX Done .. Do Something ...

	SSN_GPIO_Port->BSRR = (uint32_t)SSN_Pin;			// Put SSN high - Deactivate
	// Debugging (GPIO C0)
	DEBUG_PC0;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef * hspi)
{
    // TX-RX Done .. Do Something ...
	while((hspi->Instance->SR & SPI_SR_RXNE)!=RESET);	// Receive buffer Not Empty
	while((hspi->Instance->SR & SPI_SR_BSY)!=RESET); 	// Busy flag
	SSN_GPIO_Port->BSRR = (uint32_t)SSN_Pin;			// Put SSN high - Deactivate
	// Debugging (GPIO C0)
	DEBUG_PC0;
	//HAL_SPI_Abort(&hspi1); // trail to remove second read out, using HAL DMA
	//HAL_SPI_DMAPause(&hspi1);

}


void POST_Processing(void)
{
	for (int i = 0; i < (WR_Index-RD_Index); i++)
	{
		// Recognition of Last Valid Result for STOP Channel (TX)
		if ( (Raw_Array[RD_Index][0] != 0x00FFFFFF) && (Raw_Array[RD_Index][1] != 0x00FFFFFF) )
		{
			Last_REFID1 = Raw_Array[RD_Index][0];
			Last_TSTOP1 = Raw_Array[RD_Index][1];
			Valid_TSTOP1 = 1;

			Valid_TSTOP2 = 0;
			Calc_Diff = 0;
		}

		// Recognition of Last Valid Result for STOP Channel (RX)
		if ( (Raw_Array[RD_Index][2] != 0x00FFFFFF) && (Raw_Array[RD_Index][3] != 0x00FFFFFF) )
		{
			Last_REFID2 = Raw_Array[RD_Index][2];
			Last_TSTOP2 = Raw_Array[RD_Index][3];
			Valid_TSTOP2 = 1;

		}

		RD_Index++;

		//Execution, only as soon as both channels have last value.
		if ((Valid_TSTOP1 == 1) && (Valid_TSTOP2 == 1))
		{
			// Calculation of STOP2 minus STOP1
			DiffTSTOP[Calc_Index] = ( Last_TSTOP2 - Last_TSTOP1 );
			DiffREFID[Calc_Index] = ( Last_REFID2 - Last_REFID1 );

			Final_CALC[Calc_Index] = ( (DiffTSTOP[Calc_Index]) + (DiffREFID[Calc_Index] * REFCLK_DIVISIONS) );
			Final_CALC[Calc_Index] /= 100000; //Result in [us]

			if ( (Final_CALC[Calc_Index] > 0) )
			{
				Calc_Index++; // pointer at next address.
				Calc_Diff++; // counting calculated difference (STOP2 - STOP1)

			}

		} // End of IF (..Validation)

	} // End of FOR Loop

	return;
}

void Trigger_Generation(void)
{
	// Unlock Blocked FIFO
	// FIFO full -> FIFO blocked, then further pulses are lost.
	// There must be unlocked with two dummy pulses.

	// Unlocking of FIFO with 2 dummy STOPS (plus 1 safety), (because of synchronisation) when FIFO was full once
	// Or unlocking of FIFO by POR + WR configuration once again

	// t.b.d.


	//clear FIFO before restarting
	My_Clearing_All_FIFOs();
	//Clearing_All_FIFOs();


	// Debugging (GPIO C0)
	//HAL_GPIO_TogglePin(Debugging_Pin_GPIO_Port, Debugging_Pin_Pin);

	// Clearing RSTIDX: Reference Index Counter Reset, has to be activated with PIN_ENA_RSTIDX.
	// AS6500 Trigger (GPIO C3) --> RSTIDX
	//AS6500_Trigger_GPIO_Port->BSRR = (uint32_t)AS6500_Trigger_Pin; // SET
	//AS6500_Trigger_GPIO_Port->BRR = (uint32_t)AS6500_Trigger_Pin; // RESET

	TDC_RSTIDX_PULSE;

	// Triggering WaveForm Generator via GPIO
	// WaveFrom Trigger (GPIO C2) --> STOP1
	//WaveForm_Trigger_GPIO_Port->BSRR = (uint32_t)WaveForm_Trigger_Pin; // SET
	//WaveForm_Trigger_GPIO_Port->BRR = (uint32_t)WaveForm_Trigger_Pin; // RESET

	LASER_TRIGGER_PULSE;

	return;
}




/*git hub - spi.c*/
void HAL_SPI_MY_TransmitReceive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size)
{
	hspi->Instance->CR2 |= SPI_CR2_TXDMAEN;							// Enable Tx DMA Request
	hspi->Instance->CR2 |= SPI_CR2_RXDMAEN;							// Enable Rx DMA Request

	hdma_spi1_tx.Instance->CCR &= ~DMA_CCR_EN;						// Channel enable
	hdma_spi1_tx.Instance->CPAR = (uint32_t)&hspi->Instance->DR;	// DMA channel x peripheral address register
	hdma_spi1_tx.Instance->CMAR = (uint32_t)pTxData;				// DMA channel x memory address register
	hdma_spi1_tx.Instance->CNDTR = Size;							// DMA channel x number of data register

	// Put SSN low - Activate
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	SSN_GPIO_Port->BRR = (uint32_t)SSN_Pin;

	hdma_spi1_tx.Instance->CCR |= DMA_CCR_EN;						// Channel enable
	hdma_spi1_rx.Instance->CCR &= ~DMA_CCR_EN;						// Channel enable
	hdma_spi1_rx.Instance->CPAR = (uint32_t)&hspi->Instance->DR;	// DMA channel x peripheral address register
	hdma_spi1_rx.Instance->CMAR = (uint32_t)pRxData;				// DMA channel x memory address register
	hdma_spi1_rx.Instance->CNDTR = Size;							// DMA channel x number of data register
	hdma_spi1_rx.Instance->CCR |= DMA_CCR_EN;						// Channel enable

	hspi->Instance->CR1 |= SPI_CR1_SPE;								// SPI Enable

	while((hspi->Instance->SR & SPI_SR_RXNE)!=RESET); // Receive buffer Not Empty
	while((hspi->Instance->SR & SPI_SR_BSY)!=RESET); // Busy flag

	// Put SSN high - Deactivate
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
	SSN_GPIO_Port->BSRR = (uint32_t)SSN_Pin;
}

void HAL_SPI_MY_Transmit_DMA(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint16_t Size)
{
	// Work Around
	// Tx and Rx uses the same DMA channel x memory address register
	hspi->Instance->CR2 |= SPI_CR2_TXDMAEN;							// Enable Tx DMA Request
	hspi->Instance->CR2 |= SPI_CR2_RXDMAEN;							// Enable Rx DMA Request

	hdma_spi1_tx.Instance->CCR &= ~DMA_CCR_EN;						// Channel enable
	hdma_spi1_tx.Instance->CPAR = (uint32_t)&hspi->Instance->DR;	// DMA channel x peripheral address register
	hdma_spi1_tx.Instance->CMAR = (uint32_t)pTxData;				// DMA channel x memory address register
	hdma_spi1_tx.Instance->CNDTR = Size;							// DMA channel x number of data register

	// Put SSN low - Activate
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	SSN_GPIO_Port->BRR = (uint32_t)SSN_Pin;

	hdma_spi1_tx.Instance->CCR |= DMA_CCR_EN;						// Channel enable
	hdma_spi1_rx.Instance->CCR &= ~DMA_CCR_EN;						// Channel enable
	hdma_spi1_rx.Instance->CPAR = (uint32_t)&hspi->Instance->DR;	// DMA channel x peripheral address register
	hdma_spi1_rx.Instance->CMAR = (uint32_t)pTxData;				// DMA channel x memory address register
	hdma_spi1_rx.Instance->CNDTR = Size;							// DMA channel x number of data register
	hdma_spi1_rx.Instance->CCR |= DMA_CCR_EN;						// Channel enable

	hspi->Instance->CR1 |= SPI_CR1_SPE;								// SPI Enable

	while((hspi->Instance->SR & SPI_SR_RXNE)!=RESET); // Receive buffer Not Empty
	while((hspi->Instance->SR & SPI_SR_BSY)!=RESET); // Busy flag

	// Put SSN high - Deactivate
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
	SSN_GPIO_Port->BSRR = (uint32_t)SSN_Pin;
}
/*
// Put SSN low - Activate
SSN_GPIO_Port->BRR = (uint32_t)SSN_Pin;
// Put SSN high - Deactivate
SSN_GPIO_Port->BSRR = (uint32_t)SSN_Pin;
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
