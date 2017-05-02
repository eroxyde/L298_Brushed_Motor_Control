/**
  ******************************************************************************
  * @file    ... /Src/main.c
  * @author  Eric Cloutier
  * @version V1.0
  * @date    28-Mars-2017
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"
#include "stdlib.h"
#include "motor.h"
#include "trajectory.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* TIM handle declaration */
TIM_HandleTypeDef    	TimHandle;
TIM_OC_InitTypeDef 		sConfigOC;
TIM_MasterConfigTypeDef sMasterConfig;

TIM_HandleTypeDef		TimHandleISR;
TIM_SlaveConfigTypeDef	sSlaveConfig;

TIM_HandleTypeDef    	EncoderHandle;
TIM_Encoder_InitTypeDef sEncoderConfig;

/* Prescaler declaration */
uint32_t uwPrescalerValue;

static GPIO_InitTypeDef  GPIO_InitStruct;

/* UART handler declaration */
UART_HandleTypeDef UartHandle;

/* Buffers used for transmission */
uint8_t aTxBuffer[] = "\n\r ****Servomotor UART communication****\n\r Enter a 6 characters command using keyboard :\n\r";
char aStatusTxBuf[15];

/* Buffer used for reception */
uint8_t aRxBuffer[RXBUFFERSIZE];

/* Flags */
uint8_t initF;

/* Motor */
uint16_t aDutyCycle;
Motor_State aState;
uint8_t aDirection;
int32_t aPos1;
int32_t aPos2;
int32_t aSpeed;

/* Encoder */
uint32_t aCount;

/* Trajectory */
int32_t aPosCommand;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Error_Handler(void);
void setDutyCycle(uint8_t dir, uint16_t dutyCycle);
void chkBuf(void);
void parseCom(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{

  /* STM32F103xB HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
		 */
	HAL_Init();

	/* Configure the system clock to 64 MHz */
	SystemClock_Config();

	/* Configure LED2 */
	BSP_LED_Init(LED2);

	/*## Configure the TIM3 peripheral #######################################*/
	/* -----------------------------------------------------------------------
	TIM3 input clock (TIM3CLK)  is set to APB1 clock (PCLK1) x2,
	since APB1 prescaler is set to 4 (0x100).
	 TIM3CLK = PCLK1*2
	 PCLK1   = HCLK/2
	=> TIM3CLK = PCLK1*2 = (HCLK/2)*2 = HCLK = SystemCoreClock
	To get TIM3 counter clock at 16 MHz, the Prescaler is computed as following:
	Prescaler = (TIM3CLK / TIM3 counter clock) - 1
	Prescaler = (SystemCoreClock /16 MHz) - 1
	-------------------------------------------------------------------------*/

	uwPrescalerValue = (uint32_t)(SystemCoreClock / 16000000) - 1;

	/* Set TIM3 instance */
	TimHandle.Instance = TIM3;

	/* Initialize TIM3 peripheral as follows:
	 + Period (ARR)   = (TIM3 counter clock / TIM3 output clock ) - 1 = (16000000 /10000) - 1 = 1599
	 + TIM3 output clock = 10 kHz
	 + Prescaler = (SystemCoreClock/ 16 MHz) - 1
	 + ClockDivision = 0
	 + Counter direction = Up
	*/
	TimHandle.Init.Prescaler         = uwPrescalerValue;
	TimHandle.Init.Period            = 1599;
	TimHandle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
	TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
	TimHandle.Init.RepetitionCounter = 0;

	if(HAL_TIM_PWM_Init(&TimHandle) != HAL_OK)	// il faut coder l'interface MSP
	{
		Error_Handler();
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	sConfigOC.Pulse = 0;

	if(HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}

	/* Configure TIM3 as master & use the update event as Trigger Output (TRGO) */
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_ENABLE;
	if(HAL_TIMEx_MasterConfigSynchronization(&TimHandle, &sMasterConfig) != HAL_OK)
	{
		/* Configuration Error */
		Error_Handler();
	}

	/*##-1- Configure the TIM4 peripheral as slave ##########################*/
	/* The TIM4 is running:
     - At (TIM3 frequency)/ (TIM4 period + 1) = 10Khz/10 = 1khz  */
	TimHandleISR.Instance = TIM4;
	TimHandleISR.Init.Period = 9;
	TimHandleISR.Init.Prescaler = 0;
	TimHandleISR.Init.ClockDivision = 0;
	TimHandleISR.Init.CounterMode = TIM_COUNTERMODE_UP;
	TimHandleISR.Init.RepetitionCounter = 0;

	if (HAL_TIM_Base_Init(&TimHandleISR) != HAL_OK)		// Il faut coder l'interface MSP
	{
		/* Initialization Error */
		Error_Handler();
	}

	/* Configure TIM4 in external clock mode in slave mode */
	sSlaveConfig.SlaveMode     = TIM_SLAVEMODE_GATED;
	sSlaveConfig.InputTrigger  = TIM_TS_ITR2;
	sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_NONINVERTED;
	sSlaveConfig.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
	sSlaveConfig.TriggerFilter = 0;
	if (HAL_TIM_SlaveConfigSynchronization_IT(&TimHandleISR, &sSlaveConfig) != HAL_OK)
	{
		/* Starting Error */
		Error_Handler();
	}

	/*## Configure the TIM1 peripheral #######################################*/
	/* -----------------------------------------------------------------------
	* Rotary encoders counters
	*/
	// Set TIM1 instance //
	EncoderHandle.Instance = TIM1;
	EncoderHandle.Init.Period = 65535;
	EncoderHandle.Init.Prescaler = 0;
	EncoderHandle.Init.ClockDivision = 0;
	EncoderHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	EncoderHandle.Init.RepetitionCounter = 0;

	sEncoderConfig.EncoderMode = TIM_ENCODERMODE_TI12;

	sEncoderConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sEncoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sEncoderConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sEncoderConfig.IC1Filter = 0;

	sEncoderConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sEncoderConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sEncoderConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sEncoderConfig.IC2Filter = 0;

	if(HAL_TIM_Encoder_Init(&EncoderHandle, &sEncoderConfig) != HAL_OK)	// il faut coder l'interface MSP
	{
		Error_Handler();
	}

	/* Start the encoder interface */
	HAL_TIM_Encoder_Start(&EncoderHandle, TIM_CHANNEL_ALL);

	/*************************** L298 functions *********************************/
	/****************************************************************************/
	L298_GPIOC_PORT();
	/* Bridge A inputs */
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pin = GPIO_PIN_6;								// PC6 for L298 C
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);

	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pin = GPIO_PIN_7;								// PC7 for L298 D
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

	/************************** Init var ****************************************/
	aState = PWM;
	initF = 0;
	aCount = 0;
	aPos1 = 0;
	aPos2 = 0;
	aDutyCycle = 0;
	aDirection = CW;
	setDutyCycle(aDirection, aDutyCycle);

	/*##  Start the TIM4 Base generation in interrupt mode ####################*/
	if (HAL_TIM_Base_Start_IT(&TimHandleISR) != HAL_OK)
	{
		Error_Handler();
	}

	/*##  Configure the UART peripheral #########################################*/
	/* Put the USART peripheral in the Asynchronous mode (UART Mode) */
	/* UART configured as follows:
	 - Word Length = 8 Bits
	 - Stop Bit    = One Stop bit
	 - Parity      = none
	 - BaudRate    = 9600 baud
	 - Hardware flow control disabled (RTS and CTS signals) */
	UartHandle.Instance          = USART3;
	UartHandle.Init.BaudRate     = 9600;
	UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits     = UART_STOPBITS_1;
	UartHandle.Init.Parity       = UART_PARITY_NONE;
	UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode         = UART_MODE_TX_RX;

	if(HAL_UART_DeInit(&UartHandle) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_UART_Init(&UartHandle) != HAL_OK)
	{
	/* Initialization Error */
		Error_Handler();
	}

	/*## Start the transmission process ########################################*/
	/* User start transmission data through "TxBuffer" buffer */
	if (HAL_UART_Transmit(&UartHandle, (uint8_t *)aTxBuffer, TXBUFFERSIZE, 5000) != HAL_OK)
	{
		/* Transfer error in transmission process */
		Error_Handler();
	}

	/* Infinite loop */
	// Foreground processing : round robin scheduling
	while (1)
	{
		chkBuf();
		parseCom();
		// ## Send the status of operation or info ##############################//
		if (HAL_UART_Transmit(&UartHandle, (uint8_t *)aStatusTxBuf, 15, 5000) != HAL_OK)
		{
		// Transfer error in transmission process //
			Error_Handler();
		}
		//## Wait for the end of the transfer ###################################//
		/*  Before starting a new communication transfer, you need to check the current
		  state of the peripheral; if it’s busy you need to wait for the end of current
		  transfer before starting a new one.
		  For simplicity reasons, this example is just waiting till the end of the
		  transfer, but application may perform other tasks while transfer operation
		  is ongoing. */
		while (HAL_UART_GetState(&UartHandle) != HAL_UART_STATE_READY)
		{
		}
	}
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 64000000
  *            HCLK(Hz)                       = 64000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            PLLMUL                         = 16
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef clkinitstruct = {0};
  RCC_OscInitTypeDef oscinitstruct = {0};
  
  /* Configure PLL ------------------------------------------------------*/
  /* PLL configuration: PLLCLK = (HSI / 2) * PLLMUL = (8 / 2) * 16 = 64 MHz */
  /* PREDIV1 configuration: PREDIV1CLK = PLLCLK / HSEPredivValue = 64 / 1 = 64 MHz */
  /* Enable HSI and activate PLL with HSi_DIV2 as source */
  oscinitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSI;
  oscinitstruct.HSEState        = RCC_HSE_OFF;
  oscinitstruct.LSEState        = RCC_LSE_OFF;
  oscinitstruct.HSIState        = RCC_HSI_ON;
  oscinitstruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  oscinitstruct.HSEPredivValue    = RCC_HSE_PREDIV_DIV1;
  oscinitstruct.PLL.PLLState    = RCC_PLL_ON;
  oscinitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSI_DIV2;
  oscinitstruct.PLL.PLLMUL      = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&oscinitstruct)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
  clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
	BSP_LED_Toggle(LED2);
	while (1)
	{
	}
}

/**
  * @brief  This function set the direction and duty cycle for the PWM
  * @param  dir 		: CW or CCW
  * @param	dutyCycle	: 0 to 1599
  * @retval None
  */
void setDutyCycle(uint8_t dir, uint16_t dutyCycle)
{
	if(dutyCycle>1599)
	{
		sConfigOC.Pulse = 1599;
	}
	else
	{
		sConfigOC.Pulse = dutyCycle;
	}

	if(HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK)	// Bridge A V_enable
	{
		Error_Handler();
	}

	if(dir==CW)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);		// PC6 for L298 C
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);	// PC7 for L298 D
	}
	else
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);	// PC6 for L298 C
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);		// PC7 for L298 D
	}

}

/**
  * @brief  This function check if there is a command received in the Uart buffer
  * @param  None
  * @retval None
  */
void chkBuf(void)
{
	/*## Put UART peripheral in reception process ###########################*/
	/* Any data received will be stored in "RxBuffer" buffer : the number max of
	data received is 6 */
	if (HAL_UART_Receive(&UartHandle, (uint8_t *)aRxBuffer, RXBUFFERSIZE, 0x1FFFFFF) != HAL_OK)
	{
		/* Transfer error in reception process */
		Error_Handler();
	}

	/*## Wait for 6 char (one command) ######################################*/
	/*  Before starting a new communication transfer, you need to check the current
	  state of the peripheral; if it’s busy you need to wait for the end of current
	  transfer before starting a new one.*/
	while (HAL_UART_GetState(&UartHandle) != HAL_UART_STATE_READY)
	{
	}


}

/**
  * @brief  This function read the command in the buf
  * @param  None
  * @retval None
  */
void parseCom(void)
{
	switch(aRxBuffer[0])
	{
		char num[5];
		case 'D' :
			memcpy(num, aRxBuffer+2, 4);
			aDutyCycle = (uint16_t)(atoi(num));
			if(aRxBuffer[1] == '+')
			{
				aDirection = CW;
			}
			else
			{
				aDirection = CCW;
			}
			strcpy(aStatusTxBuf, "PWM updated\n\r");
			aState = PWM;
			break;
		case 'P' :
			memcpy(num, aRxBuffer+2, 4);
			aPosCommand = (int32_t)(atoi(num));
			strcpy(aStatusTxBuf, "POS updated\n\r");
			aState = POS;
			break;
		/*case 'S' :
			aState = SPEED;
			break;*/
		default :
			strcpy(aStatusTxBuf, "Command Error\n\r");
			break;
	}
}

/**
  * @brief  ISR main
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//BSP_LED_Toggle(LED2);
	uint32_t count = 0;
	//int32_t posErr = 0;
	count = EncoderHandle.Instance->CNT;
	aSpeed = (count - aCount);
	if(aDirection == CW)
	{
		if(aSpeed < 0)
		{
			aSpeed = (65535 - aCount) + count;
		}
	}
	else		// CCW
	{
		if(aSpeed > 0)
		{
			aSpeed = (count - 65535) - aCount;
		}
	}
	aPos2 = aPos1;
	aPos1 += aSpeed * 30;
	aCount = count;

	if(initF)									// nouvelle trajectoire
	{
		initF = 0;
	}

	/*if(aState != PWM)							// calcul des buts
	{
		posErr = aPosCommand - aPos1;
	}*/

	setDutyCycle(aDirection, aDutyCycle);
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
