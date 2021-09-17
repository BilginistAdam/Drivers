/*
 * L298N.h
 *
 *  Created on: Jan 12, 2021
 *      Author: bilginistAdam
 * 		E-Mail: bilginenesismail@gmail.com
 * 		Info  : This driver developed for L298N dual motor driver. If you are using this driver,
 * 		you must do some spesific settings. This document is for it.
 *
 * 		 Note : I think you should examine the sample document after reading this note. It will be more efficient for you.
 *
 * 		Before using;
 *	!!!	0 - This driver uses the HAL API. So, if your application doesn't use the HAL API, you need to make some changes like the timer type.
 *
 *		1 - Set a timer to PWM generation for speed control and set the 2 GPIO pin to output for each motor.
 *		(Advise your PWM frequency is higher than 20MHz, or lower than 20Hz.)
 *
 *		2 - Include driver and add file path to compiler include path list for IDE.
 *
 *		3 - Create a handler variable(@L298N_Handler_t) for your motor. (If you have 2 motor, define @L298N_Handler_t variable for each 2 motor.) And, define such
 *				configuration parameters the timer, the timer channel, etc. (@L298N_Config_t)
 *
 *		4 - Call L298N_Init() function and initialize the motor driver.
 *
 *		5 - If you want to increase or decrease speed of the motor, you should use L298N_setMotorSpeed(). Also, If you want to change direction of the motor,
 *			you should use L298N_setMotorDirection() function.
 *
 *		Note : If you want to stop motor,you should set speed to L298N_MOTOR_SPEED_STOP and direction to L298N_MOTOR_DIR_STOP.
 *
 *		6 - If you want to get speed and direction value, you should use get function like L298N_getMotorSpeed() or L298N_getMotorDirection().
 *
 *		That is all. If you encounter an error or you can't run it, feel free to contact, you can write to above e-mail address.
 *
 **/

#ifndef __L298N_H_
#define __L298N_H_

/* INCLUDES **********************************************************/
#include "main.h"

/* DEFINES ***********************************************************/
//@L298N_MOTOR_SPEED
#define L298N_MOTOR_SPEED_MIN 		0UL						//This is min speed for each motors.
#define L298N_MOTOR_SPEED_MAX		1000UL					//This is max speed for each motors.
#define L298N_MOTOR_SPEED_STOP 		0UL						//This is stop speed for each motors.

/* TYPEDEFS **********************************************************/
typedef enum{
	L298N_RESET = 0U,					/* It is the status of the flag. */
	L298N_SET	= !L298N_RESET			/* It is the status of the flag. */
}L298N_StatusFlag_t;

//@L298N_MOTOR_DIRECTION
typedef enum{
	L298N_Stop		= 0,					//This is forward direction to motor rotation.
	L298N_Forward 	= 1,					//This is forward direction to motor rotation.
	L298N_Backward	= 2,					//This is backward direction to motor rotation.
}L298N_Direction_t;

typedef struct{
	TIM_HandleTypeDef 	* Timer;		/* It is the address of the timer. */
	uint32_t 			  Channel;		/* It is the channel of the timer. */

	GPIO_TypeDef		* port[2];		/* This parameter holds GPIO ports. port[0] -> 0. pin port; port[1] -> 1. pin port. */
	uint16_t			  pin[2];		/* This parameter holds GPIO pins. pin[0] -> 0. pin; pin[1] -> 1. pin. */
}L298N_Config_t;

typedef struct{
	L298N_Config_t 		  	Config;		/* It is the address of the config. */
	uint32_t 			  	speed;		/* This parameter holds motor speed. It is able to range @L298N_MOTOR_SPEED_MIN - @L298N_MOTOR_SPEED_MAX. */
	L298N_Direction_t 		direction;	/* This parameter holds motor direction. @L298N_MOTOR_DIRECTION */
}L298N_Handler_t;

/* Function Definitions **********************************************/
/* Init */
HAL_StatusTypeDef L298N_MotorInit(L298N_Handler_t * hmotor);

/* Get */
uint32_t L298N_getMotorSpeed(L298N_Handler_t* hmotor);
L298N_Direction_t L298N_getMotorDirection(L298N_Handler_t* hmotor);

//Set
HAL_StatusTypeDef L298N_setMotorSpeed(L298N_Handler_t* hmotor, uint32_t speed);
HAL_StatusTypeDef L298N_setMotorDirection(L298N_Handler_t* hmotor, int8_t direction);




#endif /* __L298N_H_ */
