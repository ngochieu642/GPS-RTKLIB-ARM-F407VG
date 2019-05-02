#include "main.h"

#define TIMER_FREQ_HZ 5;

#ifdef TIME_MEASURE /*Defined in rtk.h */
uint32_t t=0,start=0;
#endif

/*DMA interrupt variables*/
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart6_rx;

/* Test Variables */
uint8_t UART2Buffer[10],UART6Buffer[10];
uint8_t UART4Buffer[10]={1,2,3,4,5,6,7,8,9,10};

/*Memory Optimization*/
UART_HandleTypeDef 	UartResultHandle,	/*UART4 PC10 PC11*/
										UartGPSHandle,		/*UART2 PA2  PA3*/
										UartRFHandle __attribute__((section("IRAM2")));		/*UART6 PC6  PC7*/
TIM_HandleTypeDef TimerHandle __attribute__((section("IRAM2")));

TIM_IC_InitTypeDef sConfig __attribute__((section("IRAM2")));
TIM_SlaveConfigTypeDef sSlaveConfig __attribute__((section("IRAM2")));

static obsd_t obsd[2*MAX_SAT] __attribute__((section("IRAM2")));
static rtksvr_t svr __attribute__((section("IRAM2")));

static volatile bool flagTimeout=0;
static int fobs[2];
static volatile Error RError=INCOMPLETE;//rover data error
static volatile Error BError=INCOMPLETE;//base data error

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void ConfigLED(void);
void ConfigTimer(void);
void ConfigUART(int baseFormat);
void ConfigDMA(void);
int decode_raw(rtksvr_t* svr, int index);

/*UART Function*/
void sendRequest(int baseFormat);
void SendInt(int num);
void SendIntStr(int num);
void SendStr(const char *str);

int main(void)
{
  HAL_Init();
	
  SystemClock_Config();
	ConfigLED();
	ConfigDMA();
	ConfigTimer();
	
	rtksvrstart(&svr);
	ConfigUART(svr.format[0]);
	

	/*Test code*/
	HAL_UART_Receive_DMA(&UartGPSHandle,&UART2Buffer[0],10);
	HAL_UART_Receive_DMA(&UartRFHandle,&UART6Buffer[0],10);
	/*----End of Test Code--------*/
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		for(int i = 0;i<10;i++)
			UART4Buffer[i]+=1;
//		HAL_UART_Transmit_DMA(&UartResultHandle,&UART4Buffer[0],10);
//		SendInt(0x31323334);
//		SendIntStr(45678);
		SendStr("xin chao cac ban day la 1 str\n");
		HAL_Delay(1000);
  }
}

/*Configuration Function*/
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
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
}
void ConfigLED(void)
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
	
	GPIO_InitTypeDef GPIO_InitStruct;
	__GPIOD_CLK_ENABLE();
	
	GPIO_InitStruct.Mode 		= GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pin 		= GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_InitStruct.Pull 		= GPIO_NOPULL;
	GPIO_InitStruct.Speed 	= GPIO_SPEED_MEDIUM;
	HAL_GPIO_Init(GPIOD,&GPIO_InitStruct);
	
	/*Set all LEDs Off*/
	LED3_OFF;
	LED4_OFF;
	LED5_OFF;
	LED6_OFF;
}
void ConfigTimer(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
	
  TimerHandle.Instance = TIM3;
  TimerHandle.Init.Prescaler = 8199;
  TimerHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  TimerHandle.Init.Period = 19999;
  TimerHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	
	__TIM3_CLK_ENABLE();
  HAL_TIM_Base_Init(&TimerHandle);
		
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&TimerHandle, &sClockSourceConfig);
 
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&TimerHandle, &sMasterConfig);
	
	HAL_NVIC_SetPriority(TIM3_IRQn,2,0);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
	HAL_TIM_Base_Start_IT(&TimerHandle);
}
void ConfigUART(int baseFormat)
{
	/*Uart GPS*/
	UartGPSHandle.Instance = USART2;
	switch(baseFormat)
	{
		case(STRFMT_UBX):
		{
//			UartGPSHandle.Init.BaudRate = 230400;
			UartGPSHandle.Init.BaudRate = 9600; /*Because Baudrate default is 9600*/
			break;		
		}
		case(STRFMT_SS2):
		{
			//			UartGPSHandle.Init.BaudRate = 19200;
			UartGPSHandle.Init.BaudRate = 9600; /*Because Baudrate default is 9600*/
			break;
		}
		default:
		{
			UartGPSHandle.Init.BaudRate = 9600;
		}
	}
	
  UartGPSHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartGPSHandle.Init.StopBits = UART_STOPBITS_1;
  UartGPSHandle.Init.Parity = UART_PARITY_NONE;
  UartGPSHandle.Init.Mode = UART_MODE_TX_RX;
  UartGPSHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  UartGPSHandle.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&UartGPSHandle);
	
	/*Uart Server*/
	UartRFHandle.Instance = USART6;
//	UartRFHandle.Init.BaudRate		= 57600;	
  UartRFHandle.Init.BaudRate = 9600;
  UartRFHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartRFHandle.Init.StopBits = UART_STOPBITS_1;
  UartRFHandle.Init.Parity = UART_PARITY_NONE;
  UartRFHandle.Init.Mode = UART_MODE_TX_RX;
  UartRFHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  UartRFHandle.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&UartRFHandle);
	
	/*Uart Result*/
	UartResultHandle.Instance = UART4;
  UartResultHandle.Init.BaudRate = 19200;
  UartResultHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartResultHandle.Init.StopBits = UART_STOPBITS_1;
  UartResultHandle.Init.Parity = UART_PARITY_NONE;
  UartResultHandle.Init.Mode = UART_MODE_TX_RX;
  UartResultHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  UartResultHandle.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&UartResultHandle);
}
void ConfigDMA(void)
{
/* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
}
void Error_Handler(void)
{
}
int decode_raw(rtksvr_t* svr, int index)
{
	uint8_t* i;
	Error err; /*enum defined in rtk.h*/
	char str[20];
	int res = 0, res2 = 0, res3 = 0;
	switch(svr->format[index])
	{
		case STRFMT_UBX:
		{
			if(svr->buffPtr[index]+svr->nb[index]<=MAX_RAW_LEN)
			{
				for(i=svr->buff[index]+svr->buffPtr[index]; i<svr->buff[index]+svr->buffPtr[index]+svr->nb[index]; i++)
				{
					err = input_ubx(&svr->raw[index],*i);
					if(err>=NO_ERROR1) /*Defined in Error enum rtk.h*/
					{
						updatesvr(svr,err,index);
						if(err==OBS)
							res+=1;
						else if (err==EPHEMERIS)
							res2+=1;
						else if (err==SOLUTION)
							res3+=1;
					}
				}
			}
			else
			{
				for(i=svr->buff[index] + svr->buffPtr[index];i<svr->buff[index] + MAX_RAW_LEN;i++)
				{
					err = input_ubx(&svr->raw[index],*i);
					if (err>=NO_ERROR1)
					{
						updatesvr(svr,err,index);
						if (err==OBS)
							res+=1;
						else if (err==EPHEMERIS)
							res2+=1;
						else if (err==SOLUTION)
							res3+=1;
					}
				}
				
				for(i=svr->buff[index];i<svr->buff[index] + svr->nb[index] + svr->buffPtr[index] - MAX_RAW_LEN;i++)
				{
					err = input_ubx(&svr->raw[index],*i);
					if(err > NO_ERROR1)
					{
						updatesvr(svr,err,index);
						if(err == OBS)
							res+=1;
						else if(err == EPHEMERIS)
							res2+=1;
						else if(err == SOLUTION)
							res3+=1;
					}
				}
			}
			break;
		}
		case STRFMT_SS2:
		{
			if(svr->buffPtr[index]+svr->nb[index]<=MAX_RAW_LEN)
			{
				for(i=svr->buff[index]+svr->buffPtr[index]; i<svr->buff[index]+svr->buffPtr[index]+svr->nb[index]; i++)
				{
					err = input_ss2(&svr->raw[index],*i);
					if(err>=NO_ERROR1) /*Defined in Error enum rtk.h*/
					{
						updatesvr(svr,err,index);
						if(err==OBS)
							res+=1;
						else if (err==EPHEMERIS)
							res2+=1;
						else if (err==SOLUTION)
							res3+=1;
					}
				}
			}
			else
			{
				for(i=svr->buff[index] + svr->buffPtr[index];i<svr->buff[index] + MAX_RAW_LEN;i++)
				{
					err = input_ss2(&svr->raw[index],*i);
					if (err>=NO_ERROR1)
					{
						updatesvr(svr,err,index);
						if (err==OBS)
							res+=1;
						else if (err==EPHEMERIS)
							res2+=1;
						else if (err==SOLUTION)
							res3+=1;
					}
				}
				
				for(i=svr->buff[index];i<svr->buff[index] + svr->nb[index] + svr->buffPtr[index] - MAX_RAW_LEN;i++)
				{
					err = input_ss2(&svr->raw[index],*i);
					if(err > NO_ERROR1)
					{
						updatesvr(svr,err,index);
						if(err == OBS)
							res+=1;
						else if(err == EPHEMERIS)
							res2+=1;
						else if(err == SOLUTION)
							res3+=1;
					}
				}
			}
			break;
		}
	}
	return res;
}







/*UART Function*/
void sendRequest(int baseFormat)
{
	switch(baseFormat)
	{
		case(STRFMT_SS2):
		{
			uint8_t SS2_RQ[19] = {
														1,0x94,0x6B,0,0,1, 	//msg ID 20 continuous
														1,0x96,0x69,0,0,1,	//msg ID 22 (ephemeris) continuous	
														1,0x97,0x68,1,0,1,1}; //msg ID 23(Observation data)
			HAL_UART_Transmit(&UartGPSHandle,SS2_RQ,19,1000);
			break;
		}
	}
}

void SendInt(int num)
{/*vd SendInt(0x31323334), hercules will receive 1234*/
	uint8_t array[5];
	array[0] = num >> 24; // 8 MSB
	array[1] = num >> 16;
	array[2] = num >> 8;
	array[3] = num;
	array[4] = '\n';
	HAL_UART_Transmit(&UartResultHandle,array,5,5);
}

void SendIntStr(int num)
{/*Send an int with 4 number, end with \n, SendIntStr(8765), hercules will receive 8765\n */
	uint8_t str[5];
	sprintf((char*)str,"%4d\n",num);
	HAL_UART_Transmit(&UartResultHandle,(unsigned char*)str,5,1);
}

void SendStr(const char *str)
	{/*Send any String, SendStr("hello world"), hercules will receive "hello world" */
	HAL_UART_Transmit_DMA(&UartResultHandle,(unsigned char*)str,strlen(str));
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM3)
	{
		flagTimeout=1;
		LED3_TOGGLE;
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)//PC
{			
}
#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif
