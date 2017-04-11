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

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* TIM handle declaration */
TIM_HandleTypeDef    	TimHandle;
TIM_OC_InitTypeDef 		sConfigOC;
TIM_HandleTypeDef    	TimEncoderHandle;
TIM_Encoder_InitTypeDef sConfigEncoder;

/* Prescaler declaration */
uint32_t uwPrescalerValue = 0;

static GPIO_InitTypeDef  GPIO_InitStruct;

/* UART handler declaration */
UART_HandleTypeDef UartHandle;

/* Buffer used for transmission */
uint8_t aTxBuffer[] = "\n\r ****UART-Hyperterminal communication based on DMA****\n\r Enter 10 characters using keyboard :\n\r";

/* Buffer used for reception */
uint8_t aRxBuffer[RXBUFFERSIZE];

/* PWM */
uint16_t duty_cycle = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Error_Handler(void);
void setDutyCycle(uint8_t dir, uint16_t dutyCycle);

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

	/*##-1- Configure the TIM3 peripheral #######################################*/
	/* -----------------------------------------------------------------------
	In this example TIM3 input clock (TIM3CLK)  is set to APB1 clock (PCLK1) x2,
	since APB1 prescaler is set to 4 (0x100).
	 TIM3CLK = PCLK1*2
	 PCLK1   = HCLK/2
	=> TIM3CLK = PCLK1*2 = (HCLK/2)*2 = HCLK = SystemCoreClock
	To get TIM3 counter clock at 16 MHz, the Prescaler is computed as following:
	Prescaler = (TIM3CLK / TIM3 counter clock) - 1
	Prescaler = (SystemCoreClock /16 MHz) - 1
	Note:
	SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f1xx.c file.
	Each time the core clock (HCLK) changes, user had to update SystemCoreClock
	variable value. Otherwise, any configuration based on this variable will be incorrect.
	This variable is updated in three ways:
	1) by calling CMSIS function SystemCoreClockUpdate()
	2) by calling HAL API function HAL_RCC_GetSysClockFreq()
	   each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
	----------------------------------------------------------------------- */

	/* Compute the prescaler value to have TIMx counter clock equal to 32 MHz */
	uwPrescalerValue = (uint32_t)(SystemCoreClock / 16000000) - 1;

	/* Set TIMx instance */
	TimHandle.Instance = TIMx;

	/* Initialize TIMx peripheral as follows:
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
	sConfigOC.Pulse = duty_cycle;

	if(HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}

	/*if(HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK)	// Bridge A V_enable
	{
		Error_Handler();
	}*/

	/*##-1- Configure the TIM2 peripheral #######################################*/
	/* -----------------------------------------------------------------------
	* Rotary encoders counters
	*/
	// Set TIM2 instance //
	/*TimEncoderHandle.Instance = TIM2;
	TimEncoderHandle.Init.Period = 20000;
	TimEncoderHandle.Init.Prescaler = TIM_CLOCKPRESCALER_DIV1;
	TimEncoderHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	TimEncoderHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	TimEncoderHandle.Init.RepetitionCounter = 0;

	sConfigEncoder.EncoderMode = TIM_ENCODERMODE_TI1;
	sConfigEncoder.IC1Filter = 8;
	sConfigEncoder.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfigEncoder.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfigEncoder.IC1Selection = TIM_ICSELECTION_DIRECTTI; 	*/

	/*if(HAL_TIM_Encoder_Init(&TimEncoderHandle, &sConfigEncoder) != HAL_OK)	// il faut coder l'interface MSP
	{
		Error_Handler();
	}*/


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
	//duty_cycle = 800;
	//setDutyCycle(CW, duty_cycle);


	/*##-1- Configure the UART peripheral ######################################*/
	/* Put the USART peripheral in the Asynchronous mode (UART Mode) */
	/* UART configured as follows:
	 - Word Length = 8 Bits (7 data bit + 1 parity bit) :
				  BE CAREFUL : Program 7 data bits + 1 parity bit in PC HyperTerminal
	 - Stop Bit    = One Stop bit
	 - Parity      = ODD parity
	 - BaudRate    = 9600 baud
	 - Hardware flow control disabled (RTS and CTS signals) */
	UartHandle.Instance          = USARTx;

	UartHandle.Init.BaudRate     = 9600;
	UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits     = UART_STOPBITS_1;
	UartHandle.Init.Parity       = UART_PARITY_ODD;
	UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode         = UART_MODE_TX_RX;

	if (HAL_UART_Init(&UartHandle) != HAL_OK)
	{
	/* Initialization Error */
		Error_Handler();
	}

	/*## Start the transmission process #####################################*/
	/* User start transmission data through "TxBuffer" buffer */
	if (HAL_UART_Transmit_DMA(&UartHandle, (uint8_t *)aTxBuffer, TXBUFFERSIZE) != HAL_OK)
	{
		/* Transfer error in transmission process */
		Error_Handler();
	}

	/*## Put UART peripheral in reception process ###########################*/
	/* Any data received will be stored in "RxBuffer" buffer : the number max of
	 data received is 10 */
	if (HAL_UART_Receive_DMA(&UartHandle, (uint8_t *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)
	{
	/* Transfer error in reception process */
		Error_Handler();
	}

	/* Infinite loop */
	// Foreground processing
	while (1)
	{

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
