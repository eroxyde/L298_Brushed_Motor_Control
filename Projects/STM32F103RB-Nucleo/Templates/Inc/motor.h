/*
 * motor.h
 *
 *  Created on: 9 avr. 2017
 *      Author: Eric Cloutier
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MOTOR_H_
#define MOTOR_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Exported types ------------------------------------------------------------*/
/**
  * @brief Motor State structure definition
  */
typedef enum
{
	PWM				= 0x00,
	SPEED			= 0x01,
	POS				= 0x02,
}Motor_State;

/**
  * @brief Motor specs structure definition
  */
typedef struct
{
	uint8_t 	nomV;
	uint8_t 	gearsRatio;
	uint16_t	inertia;
	uint16_t	friction;
	uint16_t	Kt;
}Motor_Specs;

/* Exported constants --------------------------------------------------------*/
#define CCW		0x00
#define CW		0x01

/* Exported functions ------------------------------------------------------- */

#endif /* MOTOR_H_ */
