/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l476g_discovery.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel2;
DMA_HandleTypeDef hdma_dfsdm1_flt0;

I2C_HandleTypeDef hi2c1;

QSPI_HandleTypeDef hqspi;

RTC_HandleTypeDef hrtc;

SAI_HandleTypeDef hsai_BlockA1;
DMA_HandleTypeDef hdma_sai1_a;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

QSPI_HandleTypeDef QSPIHandle;
__IO uint8_t CmdCplt, RxCplt, TxCplt, StatusMatch, TimeOut;

uint8_t aTxBuffer[] = " ****QSPI communication based on IT****  ****QSPI communication based on IT****  ****QSPI communication based on IT****  ****QSPI communication based on IT****  ****QSPI communication based on IT****  ****QSPI communication based on IT**** ";
uint8_t aRxBuffer[BUFFERSIZE];





/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_SAI1_Init(void);
static void MX_DMA_Init(void);
static void MX_RTC_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_QUADSPI_Init(void);
/* USER CODE BEGIN PFP */
void SystemClock_Config(void);
void Error_Handler(void);
static void QSPI_WriteEnable(QSPI_HandleTypeDef *hqspi);
static void QSPI_AutoPollingMemReady(QSPI_HandleTypeDef *hqspi);
static void QSPI_DummyCyclesCfg(QSPI_HandleTypeDef *hqspi);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define AUDIO_I2C_ADDR	0x94
int32_t                     RecBuff[200];
int16_t	PlayBuff[800];
uint32_t                     DmaRecHalfBuffCplt  = 0;
uint32_t                     DmaRecBuffCplt      = 0;

volatile int				 zaDuzyHalas		 = 0;
volatile int				 zaDuzeDrgania 		= 0;
int							 pom				 = 0;
int							 tmp				 = 0;
float					     wartoscSredniaHalasu= 0;
int32_t						 progHalasu			 = 2000;

RTC_TimeTypeDef RtcTime;
RTC_DateTypeDef RtcDate;
volatile int licznik=0;
int stop =0;

#define BUFFER_SIZE		500
#define SaturaLH(N, L, H) (((N)<(L))?(L):(((N)>(H))?(H):(N)))

void send_string(char* s)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), 100);
}


void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
    DmaRecHalfBuffCplt = 1;
}

void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
    DmaRecBuffCplt = 1;
}

void natezenie()
{
    uint32_t i;
    wartoscSredniaHalasu=0;
    pom = 0;
    tmp = 0;

    if(DmaRecHalfBuffCplt==1)
    {
        for(i=0; i<100; i++)
        {
            PlayBuff[2*i]     = SaturaLH((RecBuff[i] >> 8), -32768, 32767);
            PlayBuff[(2*i)+1] = PlayBuff[2*i];

        }
    }


    if(DmaRecBuffCplt==1)
    {
        for(i=100; i<200; i++)
        {
            PlayBuff[2*i]     = SaturaLH((RecBuff[i] >> 8), -32768, 32767);
            PlayBuff[(2*i)+1] = PlayBuff[2*i];
            // if (PlayBuff[i] < 0)
            //  PlayBuff[i] =0;
            pom = pom + abs(PlayBuff[i]);
        }
        DmaRecBuffCplt=0;
    }

    char c[20];
    wartoscSredniaHalasu = (int)pom/100;
    DmaRecHalfBuffCplt=0;
    /// sprintf(c, "%d", (int)wartoscSredniaHalasu);

    // send_string(c);
    // send_string("\r\n");
    pom=0;

    if(wartoscSredniaHalasu>=progHalasu)
    {
        zaDuzyHalas=1;
    }
}


static void cs43l22_write(uint8_t reg, uint8_t value)
{
    HAL_I2C_Mem_Write(&hi2c1, AUDIO_I2C_ADDR, reg, 1, &value, sizeof(value), HAL_MAX_DELAY);
}
static int16_t audio_data[2 * BUFFER_SIZE];
static void cs43l22_init(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

    cs43l22_write(0x04, 0xaf);

    cs43l22_write(0x06, 0x07);
    cs43l22_write(0x02, 0x9e);

    for (int i = 0; i < BUFFER_SIZE; i++)
    {
        int16_t value = (int16_t)(100000.0 * sin(2.0 * M_PI * i / 22.0));
        audio_data[i * 2] = value;
        audio_data[i * 2 + 1] = value;
    }
}




void play_tone(void)
{

//while(1)
    HAL_SAI_Transmit(&hsai_BlockA1, (uint8_t*)audio_data, 2 * BUFFER_SIZE, HAL_MAX_DELAY);

}
int flaga1=1, flaga2=1;
int16_t pData[3];
int16_t pnRawData[3];
uint8_t ctrlx[2]= {0,0};
uint8_t buffer[6];
uint8_t i = 0;
uint8_t sensitivity = LSM303C_ACC_SENSITIVITY_2G;
float raw_float[3];
float scaled[3];

void akcelerometr_init()
{
    ACCELERO_IO_Init();
    ACCELERO_IO_Write(LSM303C_CTRL_REG1_A, 0b01100111);
    ACCELERO_IO_Write(LSM303C_CTRL_REG4_A, 0b00110111);
}
void akcelerometr_read()
{
    buffer[0] = ACCELERO_IO_Read(LSM303C_OUT_X_L_A);
    buffer[1] = ACCELERO_IO_Read(LSM303C_OUT_X_H_A);
    buffer[2] = ACCELERO_IO_Read(LSM303C_OUT_Y_L_A);
    buffer[3] = ACCELERO_IO_Read(LSM303C_OUT_Y_H_A);
    buffer[4] = ACCELERO_IO_Read(LSM303C_OUT_Z_L_A);
    buffer[5] = ACCELERO_IO_Read(LSM303C_OUT_Z_H_A);

    for(i=0; i<3; i++)
    {
        pnRawData[i]=((int16_t)((uint16_t)buffer[2*i+1] << 8) + buffer[2*i]);
        raw_float[i] = (float)pnRawData[i];
        scaled[i] = (raw_float[i] * 0.244) / 1000;
    }
    char c[20];
    sprintf(c, "%f", scaled[1]);

    //  send_string(c);
    //  send_string("\r\n");

    if(fabs(0.00- scaled[0]) >  0.25  ||(fabs(0.00- scaled[1]) >  0.25))
        zaDuzeDrgania=1;


}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim == &htim8 && stop == 0)
    {
        licznik=0;

        if (zaDuzyHalas == 1)
        {
            send_string("Za duzy halas \r\n");
            zaDuzyHalas = 0;
            flaga1=0;
        }
        else
        {
            send_string("Halas ok \r\n");
            flaga1 =1;
        }

        if(zaDuzeDrgania == 1)
        {
            send_string("Za duze drgania \r\n");
            zaDuzeDrgania = 0;
            flaga2=0;
        }
        else
        {
            send_string("Drgania ok \r\n");
            flaga2=1;
        }
    }
    if(htim == &htim1 && (flaga1 ==0 || flaga2 == 0) && stop ==0)
    {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
    }
    if(htim == &htim2 && (flaga1 ==0 || flaga2 == 0) && stop ==0)
    {
        if (licznik < 16)
        {
            play_tone();
            licznik++;
        }
    }
    if(htim == &htim3 && stop ==1)
    {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
    }

}


void HAL_QSPI_CmdCpltCallback(QSPI_HandleTypeDef *hqspi)
{
  CmdCplt++;
}

void HAL_QSPI_RxCpltCallback(QSPI_HandleTypeDef *hqspi)
{
  RxCplt++;
}

void HAL_QSPI_TxCpltCallback(QSPI_HandleTypeDef *hqspi)
{
  TxCplt++;
}

void HAL_QSPI_StatusMatchCallback(QSPI_HandleTypeDef *hqspi)
{
  StatusMatch++;
}

void HAL_QSPI_ErrorCallback(QSPI_HandleTypeDef *hqspi)
{
  Error_Handler();
}


static void QSPI_AutoPollingMemReady(QSPI_HandleTypeDef *hqspi)
{
  QSPI_CommandTypeDef     sCommand;
  QSPI_AutoPollingTypeDef sConfig;

  /* Configure automatic polling mode to wait for memory ready ------ */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = READ_STATUS_REG_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_1_LINE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  sConfig.Match           = 0x00;
  sConfig.Mask            = 0x01;
  sConfig.MatchMode       = QSPI_MATCH_MODE_AND;
  sConfig.StatusBytesSize = 1;
  sConfig.Interval        = 0x10;
  sConfig.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

  if (HAL_QSPI_AutoPolling_IT(&QSPIHandle, &sCommand, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}


static void QSPI_DummyCyclesCfg(QSPI_HandleTypeDef *hqspi)
{
  QSPI_CommandTypeDef sCommand;
  uint8_t reg;

  /* Read Volatile Configuration register --------------------------- */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = READ_VOL_CFG_REG_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_1_LINE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
  sCommand.NbData            = 1;

  if (HAL_QSPI_Command(&QSPIHandle, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_QSPI_Receive(&QSPIHandle, &reg, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

  /* Enable write operations ---------------------------------------- */
  QSPI_WriteEnable(&QSPIHandle);

  /* Write Volatile Configuration register (with new dummy cycles) -- */
  sCommand.Instruction = WRITE_VOL_CFG_REG_CMD;
  MODIFY_REG(reg, 0xF0, (DUMMY_CLOCK_CYCLES_READ_QUAD << POSITION_VAL(0xF0)));

  if (HAL_QSPI_Command(&QSPIHandle, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_QSPI_Transmit(&QSPIHandle, &reg, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
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
	  QSPI_CommandTypeDef sCommand;
	  uint32_t address = 0;
	  uint16_t index;
	  __IO uint8_t step = 0;
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
  MX_SPI2_Init();
  MX_SAI1_Init();
  MX_DMA_Init();
  MX_RTC_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_DFSDM1_Init();
  MX_USART2_UART_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_QUADSPI_Init();
  /* USER CODE BEGIN 2 */
//play_tone();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

    HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, RecBuff, 200);
    cs43l22_init();

    akcelerometr_init();
    HAL_Delay(1000);

    HAL_TIM_Base_Start_IT(&htim1); //1Hz --> otrzezenie
    HAL_TIM_Base_Start_IT(&htim3); //4Hz
    HAL_TIM_Base_Start_IT(&htim8);
    HAL_TIM_Base_Start_IT(&htim2);
    send_string("START \r\n");

    /* USER CODE BEGIN 2 */
    while(1)
      {
        switch(step)
        {
          case 0:
            CmdCplt = 0;

            /* Initialize Reception buffer --------------------------------------- */
            for (index = 0; index < BUFFERSIZE; index++)
            {
              aRxBuffer[index] = 0;
            }

            /* Enable write operations ------------------------------------------- */
            QSPI_WriteEnable(&QSPIHandle);

            /* Erasing Sequence -------------------------------------------------- */
            sCommand.Instruction = SECTOR_ERASE_CMD;
            sCommand.AddressMode = QSPI_ADDRESS_1_LINE;
            sCommand.Address     = address;
            sCommand.DataMode    = QSPI_DATA_NONE;
            sCommand.DummyCycles = 0;

            if (HAL_QSPI_Command_IT(&QSPIHandle, &sCommand) != HAL_OK)
            {
              Error_Handler();
            }

            step++;
            break;

          case 1:
            if(CmdCplt != 0)
            {
              CmdCplt = 0;
              StatusMatch = 0;

              /* Configure automatic polling mode to wait for end of erase ------- */
              QSPI_AutoPollingMemReady(&QSPIHandle);

              step++;
            }
            break;

          case 2:
            if(StatusMatch != 0)
            {
              StatusMatch = 0;
              TxCplt = 0;

              /* Enable write operations ----------------------------------------- */
              QSPI_WriteEnable(&QSPIHandle);

              /* Writing Sequence ------------------------------------------------ */
              sCommand.Instruction = EXT_QUAD_IN_FAST_PROG_CMD;
              sCommand.AddressMode = QSPI_ADDRESS_4_LINES;
              sCommand.DataMode    = QSPI_DATA_4_LINES;
              sCommand.NbData      = BUFFERSIZE;

              if (HAL_QSPI_Command(&QSPIHandle, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
              {
                Error_Handler();
              }

              if (HAL_QSPI_Transmit_IT(&QSPIHandle, aTxBuffer) != HAL_OK)
              {
                Error_Handler();
              }

              step++;
            }
            break;

          case 3:
            if(TxCplt != 0)
            {
              TxCplt = 0;
              StatusMatch = 0;

              /* Configure automatic polling mode to wait for end of program ----- */
              QSPI_AutoPollingMemReady(&QSPIHandle);

              step++;
            }
            break;

          case 4:
            if(StatusMatch != 0)
            {
              StatusMatch = 0;
              RxCplt = 0;

              /* Configure Volatile Configuration register (with new dummy cycles) */
              QSPI_DummyCyclesCfg(&QSPIHandle);

              /* Reading Sequence ------------------------------------------------ */
              sCommand.Instruction = QUAD_INOUT_FAST_READ_CMD;
              sCommand.DummyCycles = DUMMY_CLOCK_CYCLES_READ_QUAD;

              if (HAL_QSPI_Command(&QSPIHandle, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
              {
                Error_Handler();
              }

              if (HAL_QSPI_Receive_IT(&QSPIHandle, aRxBuffer) != HAL_OK)
              {
                Error_Handler();
              }
              step++;
            }
            break;

          case 5:
            if (RxCplt != 0)
            {
              RxCplt = 0;

              /* Result comparison ----------------------------------------------- */
              for (index = 0; index < BUFFERSIZE; index++)
              {
                if (aRxBuffer[index] != aTxBuffer[index])
                {
                  BSP_LED_On(LED_RED);
                }
              }
              BSP_LED_Toggle(LED_GREEN);

              address += QSPI_PAGE_SIZE;
              if(address >= QSPI_END_ADDR)
              {
                address = 0;
              }
              step = 0;
            }
            break;

          default :
            Error_Handler();
        }
      }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        //HAL_RTC_GetTime(&hrtc, &RtcTime, RTC_FORMAT_BIN);
        //HAL_RTC_GetDate(&hrtc, &RtcDate, RTC_FORMAT_BIN);

        natezenie();
        akcelerometr_read();
        HAL_Delay(10);
        if (zaDuzyHalas == 1 && zaDuzeDrgania ==1 )
        {
            send_string("STOP");
            while(1)
            {
                stop = 1;
                play_tone();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 8;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_filter0.Instance = DFSDM1_Filter0;
  hdfsdm1_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter0.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter0.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC3_ORDER;
  hdfsdm1_filter0.Init.FilterParam.Oversampling = 64;
  hdfsdm1_filter0.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter0) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel2.Instance = DFSDM1_Channel2;
  hdfsdm1_channel2.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel2.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_AUDIO;
  hdfsdm1_channel2.Init.OutputClock.Divider = 4;
  hdfsdm1_channel2.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel2.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel2.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel2.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel2.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel2.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel2.Init.Awd.Oversampling = 32;
  hdfsdm1_channel2.Init.Offset = 0;
  hdfsdm1_channel2.Init.RightBitShift = 0x02;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter0, DFSDM_CHANNEL_2, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

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
  hi2c1.Init.Timing = 0x10707DBC;
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
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 1;
  hqspi.Init.FifoThreshold = 4;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 1;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
    return;
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_SATURDAY;
  sDate.Month = RTC_MONTH_APRIL;
  sDate.Date = 23;
  sDate.Year = 22;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SAI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI1_Init(void)
{

  /* USER CODE BEGIN SAI1_Init 0 */

  /* USER CODE END SAI1_Init 0 */

  /* USER CODE BEGIN SAI1_Init 1 */

  /* USER CODE END SAI1_Init 1 */
  hsai_BlockA1.Instance = SAI1_Block_A;
  hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_22K;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  if (HAL_SAI_InitProtocol(&hsai_BlockA1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI1_Init 2 */

  /* USER CODE END SAI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_1LINE;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 32000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
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
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 1000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 8000;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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
  htim3.Init.Prescaler = 1000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 8000;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 10000;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 32000;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3|GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE3 PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
	  BSP_LED_On(LED_RED);
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
        HAL_Delay(100);
        BSP_LED_Toggle(LED_RED);
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
