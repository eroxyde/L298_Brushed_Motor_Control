/*
 * motor.c
 *
 *  Created on: 9 avr. 2017
 *      Author: Eric
 */

/* Includes ------------------------------------------------------------------*/
#include "motor.h"

/* Private variables ---------------------------------------------------------*/
static GPIO_InitTypeDef  GPIO_InitStruct;		// static utile ???

/* Private function prototypes -----------------------------------------------*/
void motor_Init(void);

/* Private functions ---------------------------------------------------------*/
void motor_Init(void)
{
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

	setRotation(CW);
}

/* exported functions ---------------------------------------------------------*/
void setRotation(uint8_t dir)
{
	/* Bridge A inputs */
	if(dir == CW)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);				// PC6 for L298 C
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);			// PC7 for L298 D
	}
	else
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);			// PC6 for L298 C
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);				// PC7 for L298 D
	}
}
