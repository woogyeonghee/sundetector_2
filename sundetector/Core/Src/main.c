/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include "rb.h"
#include "DFRobot_queue.h"
//#include "DFRobot_wifi_iot.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WIFISSID     "woo"
#define WIFIPWS      "12345678"

#define SERVER        "iot.dfrobot.com.cn"  //服务器地址
#define PORT          "1883"                //端口号
#define DEVICENAME    "rHpr0RcWR"           //用户名称
#define DEVICESECRET  "9NtrAg5ZRz"          //用户登录密码
#define TOPIC         "OSpwrHHMg"           //订阅频道
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
volatile uint16_t uwADCxConvertedValue[4];

RingFifo_t gtUart2Fifo;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the UART2 and Loop until the end of transmission */
  if (ch == '\n')
    HAL_UART_Transmit (&huart2, (uint8_t*) "\r", 1, 0xFFFF);
  HAL_UART_Transmit (&huart2, (uint8_t*) &ch, 1, 0xFFFF);

  return ch;
}

char *splitdata[5];
void splitString(void){
	uint8_t count = 0;
	static struct sQueueData *p = NULL;
	p = cuappDequeue();
	if(p != NULL){
		splitdata[count] = strtok((char*)p->data, "|");
		while(splitdata[count])
		{
			splitdata[++count] = strtok(NULL, "|");
		}
	}
	free(p);
}
void mqtt(const char* url, const char *port, const char *iotid, const char *iotpwd , const char *topic)
{

	uint8_t state = 1;
	uint8_t i,j,k;

	char _mqtt[50];
	char _topic[50];
	strcat(_mqtt,"|4|1|1|");
	strcat(_mqtt,url);
	strcat(_mqtt,"|");
	strcat(_mqtt,port);
	strcat(_mqtt,"|");
	strcat(_mqtt,iotid);
	strcat(_mqtt,"|");
	strcat(_mqtt,iotpwd);
	strcat(_mqtt,"|\\r");
	printf("%s\n",_mqtt);
	for(k=0;k<50;k++)
			HAL_UART_Transmit (&huart3, (uint8_t*) _mqtt+k, 1, 0xFFFF);
	//Usart_SendStriong(USART3, _mqtt);
	memset(_mqtt,'\0',50);
	while(state == 1){
		splitString();
		if(strcmp("4",splitdata[0]) == 0){
			if(strcmp("1",splitdata[2]) == 0){
				if(strcmp("1",splitdata[3]) == 0){
					state=0;
				}else{
					printf(".");
				}
			}
		}else{
			printf(".");
		}
		for(i=0; i<5; i++){
			splitdata[i] = "\0";
		}
		HAL_Delay(100);
	}
	printf("MQTT Connect SUCCESS\n");
	state=1;
	strcat(_topic,"|4|1|2|");
	strcat(_topic,topic);
	strcat(_topic,"|\r");
	printf("%s\n",_topic);
	//HAL_UART_Transmit(&huart3, _topic, sizeof(_topic), 10);
	//Usart_SendStriong(USART3, _topic);
	for(j=0;j<50;j++)
			HAL_UART_Transmit (&huart3, (uint8_t*) _topic+j, 1, 0xFFFF);
	memset(_topic,'\0',50);
	while(state == 1){
		splitString();
		if(strcmp("4",splitdata[0]) == 0){
			if(strcmp("2",splitdata[2]) == 0){
				if(strcmp("1",splitdata[3]) == 0){
					state=0;
				}else if(strcmp("1",splitdata[4]) == 0){
					printf("Subscription limit reached");
				}else if(strcmp("2",splitdata[4]) != 0){
					printf("Subscription failed");
				}else{
					printf(".");
				}
		}else{
			printf(".");
			}
		}
		for(i=0; i<5; i++){
			splitdata[i] = "\0";
		}
		HAL_Delay(100);
	}
	printf("Subscribe Topic SUCCESS\n");
}
void publish(const char *topic,const char *masag)
{

	uint8_t state = 1;
	uint8_t i,j,k=0;
	char _data[50];
	strcat(_data,"|4|1|3|");
	strcat(_data,topic);
	strcat(_data,"|");
	strcat(_data,masag);
	strcat(_data,"|\r");
	printf("%s\n",_data);
	//HAL_UART_Transmit(&huart3, _data, sizeof(_data), 10);
	for(k=0;k<50;k++)
			HAL_UART_Transmit (&huart3, (uint8_t*) _data+k, 1, 0xFFFF);

	while(state == 1){
		splitString();
		if(strcmp("4",splitdata[0]) == 0){
			if(strcmp("3",splitdata[2]) == 0){
				if(strcmp("1",splitdata[3]) == 0){
					state=0;
					printf("Successfully sent\n");
				}else{
					printf("Failed to send message\n");
				}
			}
		}else if(j == (uint8_t)100){
			printf("time out");
			state = 0;
		}
		for(i=0; i<5; i++){
			splitdata[i] = "\0";
		}
		j++;
		HAL_Delay(100);
	}
	memset(_data,'\0',50);
}
void loop(void)
{

	uint8_t state = 1;
	uint8_t i,j=0;

	while(state == 1){
		splitString();
		if(strcmp("4",splitdata[0]) == 0){
			if(strcmp("5",splitdata[2]) == 0){
				state=0;
				printf("topic:%s\n",splitdata[3]);
				printf("massage:%s\n",splitdata[4]);
			}
		}else if(strcmp("3",splitdata[0]) == 0){
			if(strcmp("200",splitdata[1]) == 0){
				state=0;
				printf("Sent successfully\n");
				printf("massage:%s\n",splitdata[2]);
			}else if(strcmp("-1",splitdata[1]) == 0){
				printf("-1");
			}else if(strcmp("1",splitdata[1]) == 0){
				printf("1");
			}
		}else if(j == (uint8_t)100){
			printf("time out");
			state = 0;
		}
		for(i=0; i<5; i++){
			splitdata[i] = "\0";
		}
		j++;
		HAL_Delay(100);
	}
}

void connectWifi(const char *ssid , const char *pwd)
{

	char _wifi[100] = "";

	uint8_t state = 1;
	uint8_t i;
	uint8_t j;
	uint8_t k;
	uint8_t ch;
	strcat(_wifi,"|2|1|");
	strcat(_wifi,ssid);
	strcat(_wifi,",");
	strcat(_wifi,pwd);
	strcat(_wifi,"|\r");
	printf("%s\n",_wifi);
	//HAL_UART_Transmit(&huart3,_wifi,sizeof(_wifi),10);
	//USART2TransferCompleted = 0;
	//HAL_UART_Transmit(&huart2,_wifi,100,10);

	for(j=0;j<50;j++)
		HAL_UART_Transmit (&huart3, (uint8_t*) _wifi+j, 1, 0xFFFF);

	//while(!USART2TransferCompleted);
	//Usart_SendString(USART3, _wifi);
	memset(_wifi,'\0',50);
	while(1){
	  if (!RB_isempty (&gtUart2Fifo))
	  {
		  ch = RB_read (&gtUart2Fifo);
	      HAL_UART_Transmit (&huart2, &ch, 1, 0xFF);
	  }
	}
	while(state == 1){
		splitString();

		if(strcmp("2",splitdata[0]) == 0){
			if(strcmp("1",splitdata[1]) == 0){/*"3*/
				state=0;
			}else{
				printf(".");
			}
		}else{
		printf(".");
		}
		for(i=0; i<5; i++){
			splitdata[i] = "\0";
		}
		HAL_Delay(100);
	}

	for(k=0;k<5;k++){
		printf(".");
		HAL_Delay(1000);
	}

	printf("Wifi Connect SUCCESS\n");
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
  MX_TIM4_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
  /*
  if (RB_init (&gtUart2Fifo, 16)) // buffer size is power of 2
    {
      //assert(0);
    }
    */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  if (HAL_ADCEx_Calibration_Start (&hadc1,10) != HAL_OK)
    {
      Error_Handler ();
    }

  if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&uwADCxConvertedValue, 4)!=HAL_OK)
	   	{
	  	  Error_Handler ();
	   	}
  //uint8_t ch;

  connectWifi(WIFISSID,WIFIPWS);
  //mqtt(SERVER,PORT,DEVICENAME,DEVICESECRET,TOPIC);
  while (1)
  {
	  //publish(TOPIC,"HI TANG");
	  //loop();
	  /*
	  if (!RB_isempty (&gtUart2Fifo))
	  {
		  ch = RB_read (&gtUart2Fifo);
	      HAL_UART_Transmit (&huart2, &ch, 1, 0xFF);
	  }
	  */
	  printf("hello world\n");
	  //HAL_UART_Transmit(&huart3, "hello world\n", sizeof("hello world\n"), 10);
	  HAL_Delay(1000);

	  	  /*
		 printf ("CDS1=%4d, CDS2=%4d ", uwADCxConvertedValue[0],uwADCxConvertedValue[1]);
		 printf ("CDS3=%4d, CDS4=%4d\n", uwADCxConvertedValue[2],uwADCxConvertedValue[3]);
		 HAL_Delay (500);
	     //__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1000); // position 0°
	     HAL_Delay(1000);
	     //__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 2750); // position 0°
	     HAL_Delay(1000);*/
	  /*
     __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1000); // position -90°
     HAL_Delay(1000);
     __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1500); // position 0°
     HAL_Delay(1000);
     __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1750); // position 45°
     HAL_Delay(1000);
     __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 2000); // position 90°
      HAL_Delay(1000);

	 if(HAL_ADC_Start(&hadc1) != HAL_OK)
	    {
	  	  Error_Handler ();
	    }
	 for (uint8_t i = 0; i < 4; i++)
	 	 {
	           HAL_ADC_PollForConversion (&hadc1, 100);

	           if ((HAL_ADC_GetState (&hadc1) & HAL_ADC_STATE_REG_EOC) == HAL_ADC_STATE_REG_EOC)
	             {
	               uwADCxConvertedValue[i] = HAL_ADC_GetValue (&hadc1);
	             }
	 	 }

	 HAL_ADC_Stop (&hadc1);*/


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 32-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 20000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigOC.Pulse = 1750;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 2000;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void
HAL_UART_RxCpltCallback (UART_HandleTypeDef *UartHandle)
{
  uint8_t rx;

  if (UartHandle->Instance == USART3)
    {
      rx = (uint8_t) (UartHandle->Instance->RDR & (uint8_t) 0x00FF);
      RB_write (&gtUart2Fifo, rx);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
