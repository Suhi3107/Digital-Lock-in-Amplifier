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
#include <stdbool.h>
#include <string.h>
#include "arm_math.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Ns 128 // Sine 128 point LUT
#define Periodos 25 // No of sine waves
#define pntsScan 550 //No of scan points/ No of ramp steps each with delay 30msec and step size 0x05

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* Internal Sine wave of 2kHz frequency and sin amp 76mV p-p
 * Max amp 1665 Min amp 300 no of pnts 128 peak to peak 76mV*/
uint32_t LUT76[Ns] = {983,1016,1049,1083,1116,1148,1181,1212,1244,1274,1304,1333,1362,1389,
		1415,1441,1465,1488,1510,1531,1550,1568,1584,1599,1613,1625,1636,1645,1652,1658,1662,
		1664,1665,1664,1662,1658,1652,1645,1636,1625,1613,1599,1584,1568,1550,1531,1510,1488,
		1465,1441,1415,1389,1362,1333,1304,1274,1244,1212,1181,1148,1116,1083,1049,1016,983,949,
		916,882,849,817,784,753,721,691,661,632,603,576,550,524,500,477,455,434,415,397,381,366,
		352,340,329,320,313,307,303,301,300,301,303,307,313,320,329,340,352,366,381,397,415,434,
		455,477,500,524,550,576,603,632,661,691,721,753,784,817,849,882,916,949};

bool flag = false;
bool transfer=false;

char recebido[10]; 	// buffer que armazena a string recebida.
char resp1[4] = {'n', 'e', 'x', 't'};
char resp2[4] = {'a', 'b', 'c', 'd'};
char dados[4];
int C;


int ret;
int i = 0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart9;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t AdcRead[(Periodos*Ns)];
uint32_t AdcRead_1[(Periodos*Ns)];
uint32_t resultado[(Periodos*Ns)];	// Array that temporarily stores 1st and 2nd Harmonics
uint32_t adc_val_pot=0;
float arg1 = 2 * 3.14159265358979323 * 1;
float arg2 = 2 * 3.14159265358979323 * 2;
float arg3 = 2 * 3.14159265358979323 * 3;
float arg4 = 2 * 3.14159265358979323 * 4;
float ph=0.0;
//float mod4h[pntsScan];
//float mod3h[pntsScan];
float mod2h[pntsScan]; //2f M
float mod1h[pntsScan]; //1f M
//float fase4h[pntsScan];
//float fase3h[pntsScan];
float fase2h[pntsScan]; //2fX
float fase1h[pntsScan]; // 1fX
//float quad4h[pntsScan];
float quad3h[pntsScan]; //3fY
float quad2h[pntsScan]; //2fY
float quad1h[pntsScan]; //1fY

int j = 0;
//float mod3h_m[pntsScan];
//float mod3h_2m[pntsScan];
//float mod2h_m[pntsScan];
//float mod1h_m[pntsScan];
//float mod4h_avg[pntsScan];
//float mod2h_avg[pntsScan];
//float mod1h_avg[pntsScan];
//float det_avg[pntsScan];
int scan_nr=0;
float32_t t[Ns];
float32_t step;
uint32_t LUT[Ns]={};
float32_t sinLUT[Ns], cosLUT[Ns];
float soma = 0;
float media[5];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_DAC1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_UART9_Init(void);
/* USER CODE BEGIN PFP */
//uint16_t zero_detection(float* mod_3h);
void moving_average(float32_t* arr, uint32_t n, uint32_t window_size, float32_t* result);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart9,(uint8_t*)ptr,len,HAL_MAX_DELAY);
	return len;
}
void moving_average(float32_t* arr, uint32_t n, uint32_t window_size, float32_t* result) {
    for (int i = 0; i < n - window_size + 1; i++) {
        float sum = 0;
        for (int j = 0; j < window_size; j++) {
            sum += arr[i + j];
        }
        result[i] = sum / window_size;
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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_DAC1_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_UART9_Init();
  /* USER CODE BEGIN 2 */
  /****Reference sine wave LUT**8*/
  for (int k = 0; k < Ns; ++k) {
    float phase = 2.0f * 3.14159265358979323846f * k / Ns;
    sinLUT[k] = arm_sin_f32(phase);
    cosLUT[k] = arm_cos_f32(phase);
  }
  /******Internal Sine LUT********/
  memcpy(LUT,LUT76,sizeof(LUT));
  for(int k=0;k<Ns;k++)
  {
  	LUT[k]=(uint32_t)((LUT[k]/15)+40); // P-p 75mV offset 104mV (default offset)
  }

  /*Initialise Peripheral*/
    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)LUT, Ns, DAC_ALIGN_12B_R);//76mV default sine wave
    HAL_ADC_Start_DMA(&hadc1, AdcRead, (Periodos*Ns));
     HAL_ADC_Start_DMA(&hadc2, AdcRead_1, (Periodos*Ns));
     HAL_DAC_Start(&hdac1, DAC_CHANNEL_2); // feedback for ramp correction
     HAL_TIM_Base_Start(&htim1);
     HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
     HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


     step=30;
     for (int i = 0; i <Ns; i++) {
        	    	     t[i] = 0 + ((double)i * step);
        	    	     //printf("%f\n\r",500*t[i]);
        	    	     }
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
	  // HAL_UART_Receive(&huart9, command, 5, 10);
	  //ret1=strcmp(command, "phase",5);
	  //if(ret1==0){
	  //command[0]="a";
	  //phase=
	  //}
	  // **************************DSP Lock In Amplifier************************************
	  HAL_UART_Receive(&huart2, (uint8_t*)recebido, 4, 10); //timeout to be reduced to 10msec
	 //clock_speed = HAL_RCC_GetSysClockFreq();
	  ret = strncmp(recebido, resp1, 4);
	  	 	  if (ret == 0){		// If Received string "next"?
	  	 		  recebido[0] = 'a';
	  	 		 HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
	  	 		  flag = true; //Perform Lock in det on 128*25 points
	  	 	  }
	  ret = strncmp(recebido, resp2, 4);
	  if (ret == 0){		// If Received the string "abcd"?
		  	  	  	  	  	//Reached end of Ramp. Send result on UART
	  	 		  		  recebido[0] = 'l';
	  	 		 // HAL_UART_Transmit(&huart9, "2Hm MODL,1Hm MODL,4Hm MODL" , 22, 200);
	  	 		  	/*for (i = 0; i <= pntsScan; i++){
	  	 		  	//printf("%f,",fase1h[i]);
	  	 		  	printf("%f\n\r",quad3h[i]);
	  	 		  	//printf("%f,",fase2h[i]);
	  	 		  	//printf("%f\n\r",quad2h[i]);
	  	 		  	//HAL_Delay(10);
	  	 		  	}*/
	  	 		 //printf("\n\r");

	  	 		  	/*****Send Identifier for wave denoising*******/
	  	 		 	  	 		printf("2fX,");

	  	 		 	  	 		    // 2) send all X values, comma‐separated, then newline
	  	 		 	  	 		    for (int i = 0; i < pntsScan; ++i) {
	  	 		 	  	 		        printf("%f", fase2h[i]);
	  	 		 	  	 		        if (i < pntsScan-1)  printf(",");
	  	 		 	  	 		    }
	  	 		 	  	 		    printf("\n");

	  	 		 	  	 		    // 3) header for the Y‐stream
	  	 		 	  	 		    printf("2fY,");

	  	 		 	  	 		    // 4) send all Y values
	  	 		 	  	 		    for (int i = 0; i < pntsScan; ++i) {
	  	 		 	  	 		        printf("%f", quad2h[i]);
	  	 		 	  	 		        if (i < pntsScan-1)  printf(",");
	  	 		 	  	 		    }
	  	 		 	  	 		    printf("\n");

	  	 		 	  	 		    printf("3fX,");

	  	 		 	  	 		    // 2) send all X values, comma‐separated, then newline
	  	 		 	  	 			for (int i = 0; i < pntsScan; ++i) {
	  	 		 	  	 				printf("%f", quad3h[i]);
	  	 		 	  	 			 	if (i < pntsScan-1)  printf(",");
	  	 		 	  	 				}
	  	 		 	  	 			printf("\n");

	  	 		 	  	 		    /*printf("1fX,%d\n", pntsScan);

	  	 		 	  	 		    // 2) send all X values, comma‐separated, then newline
	  	 		 	  	 		    for (int i = 0; i < pntsScan; ++i) {
	  	 		 	  	 		        printf("%.6f", fase1h[i]);
	  	 		 	  	 		        if (i < pntsScan-1)  printf(",");
	  	 		 	  	 		    }
	  	 		 	  	 		    printf("\n");

	  	 		 	  	 		    printf("1fY,%d\n", pntsScan);

	  	 		 	  	 		    // 2) send all X values, comma‐separated, then newline
	  	 		 	  	 		    for (int i = 0; i < pntsScan; ++i) {
	  	 		 	  	 		        printf("%.6f", quad1h[i]);
	  	 		 	  	 		        if (i < pntsScan-1)  printf(",");
	  	 		 	  	 		    }
	  	 		 	  	 		    printf("\n");

	  	 		 	  	 		    */



	  	 		  	j=0;
	  	  		for (int k = 0; k < pntsScan; k++){	// Clearing all vectors
	  	  						//fase3h[k] = 0;
	  	  						fase2h[k] = 0;
	  	  						quad3h[k] = 0;
	  	  						quad2h[k] = 0;
	  	  						//fase1h[k] = 0;
	  	  						//quad1h[k] = 0;
	  	  						//mod3h[k] = 0;
	  	  						mod2h[k] = 0;
	  	  					}
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 31;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 2048;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 4;
  PeriphClkInitStruct.PLL2.PLL2N = 12;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_CC1;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_FALLING;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_16B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_CC2;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_FALLING;
  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc2.Init.OversamplingMode = DISABLE;
  hadc2.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.DataAlign = ADC3_DATAALIGN_RIGHT;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = DISABLE;
  hadc3.Init.Oversampling.Ratio = ADC3_OVERSAMPLING_RATIO_2;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC3_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSign = ADC3_OFFSET_SIGN_NEGATIVE;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T1_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 938;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 300;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 600;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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

}

/**
  * @brief UART9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART9_Init(void)
{

  /* USER CODE BEGIN UART9_Init 0 */

  /* USER CODE END UART9_Init 0 */

  /* USER CODE BEGIN UART9_Init 1 */

  /* USER CODE END UART9_Init 1 */
  huart9.Instance = UART9;
  huart9.Init.BaudRate = 115200;
  huart9.Init.WordLength = UART_WORDLENGTH_8B;
  huart9.Init.StopBits = UART_STOPBITS_1;
  huart9.Init.Parity = UART_PARITY_NONE;
  huart9.Init.Mode = UART_MODE_TX_RX;
  huart9.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart9.Init.OverSampling = UART_OVERSAMPLING_16;
  huart9.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart9.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart9.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart9) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart9, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart9, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart9) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART9_Init 2 */

  /* USER CODE END UART9_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint16_t zero_detection(float* mod_3h)
{

}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1){
	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);

	if (flag == true){
		flag = false;
		 float32_t acc2X = 0, acc2Y = 0, acc1X=0, acc1Y=0, acc3Y = 0;
		  int total = Periodos * Ns;
		  // accumulate 2ƒ on AdcRead_1, and 3ƒ quadrature on AdcRead
		    for (i = 0; i < total; ++i) {
		      int idx = i % Ns;

		      // 1st harmonic
		      acc1X += (float32_t)AdcRead_1[i] * sinLUT[idx];
		      acc1Y += (float32_t)AdcRead_1[i] * sinLUT[idx];

		      int idx2 = (2 * idx) % Ns;
		      // 2nd harmonic
		      acc2X += (float32_t)AdcRead_1[i] * sinLUT[idx2];
		      acc2Y += (float32_t)AdcRead_1[i] * cosLUT[idx2];

		      // 3rd harmonic quadrature only
		      int idx3 = (3 * idx) % Ns;
		      acc3Y += (float32_t)AdcRead[i] * cosLUT[idx3];
		    }
		    float32_t norm = 1.0f / total;

		     // store exactly into your arrays
		     fase1h[j]  = acc1X * norm * 1000.0f;                       // phase of 1ƒ
		     quad1h[j]  = acc1Y * norm * 1000.0f;                       // quadrature of 1ƒ
		     mod1h[j]   = sqrtf(acc1X*acc1X + acc1Y*acc1Y) * norm * 14.0f; // magnitude of 1ƒ

		     fase2h[j]  = acc2X * norm * 1000.0f;                       // phase of 2ƒ
		     quad2h[j]  = acc2Y * norm * 1000.0f;                       // quadrature of 2ƒ
		     mod2h[j]   = sqrtf(acc2X*acc2X + acc2Y*acc2Y) * norm * 14.0f; // magnitude of 2ƒ

		     quad3h[j]  = acc3Y * norm * 1000.0f;                       // quadrature of 3ƒ
	j = j + 1;

	}
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
