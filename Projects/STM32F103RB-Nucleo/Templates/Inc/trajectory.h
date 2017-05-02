/*
 * trajectory.h
 *
 *  Created on: 27 avr. 2017
 *      Author: Eric
 */

#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

/* Exported types ------------------------------------------------------------*/

/**
  * @brief Motor specs structure definition
  */
typedef struct
{
	uint32_t 	goalPos;
	uint32_t 	goalSpeed;
	uint32_t	goalAcc;
}Trajectory;


#endif /* TRAJECTORY_H_ */
