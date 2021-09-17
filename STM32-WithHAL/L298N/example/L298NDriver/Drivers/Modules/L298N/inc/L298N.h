/*
 * L298N.h
 *
 *  Created on: Jan 2, 2021
 *      Author: bilginist
 *      E-mail: bilginenesismail@gmail.com
 */

#ifndef MODULES_L298N_INC_L298N_H_
#define MODULES_L298N_INC_L298N_H_

//  INCLUDE
#include "stdint.h"
#include "main.h"
#include "stm32f3xx_hal_tim.h"
#include "stm32f3xx_hal.h"

//DEFINE
//@L298N_MOTOR_SPEED
#define L298N_MOTOR_SPEED_MIN 		0UL						//This is min speed for each motors.
#define L298N_MOTOR_SPEED_MAX		100UL						//This is max speed for each motors.
#define L298N_MOTOR_SPEED_STOP 		0UL						//This is stop speed for each motors.

//@L298N_MOTOR_DIRECTION
#define L298N_MOTOR_DIRECTION_FORWARD   1		//This is forward direction to motor rotation.
#define L298N_MOTOR_DIRECTION_STOP		0		//This is stop position to motor rotation.
#define L298N_MOTOR_DIRECTION_BACKWARD -1		//This is backward direction to motor rotation.

//Typedef
typedef enum{
	L298N_OK 	= 0x00U,
	L298N_ERROR = 0x01U
}L298N_StatusTypeDef;

typedef enum
{
  L298N_RESET = 0U,
  L298N_SET = !RESET
} L298N_FlagStatus;

typedef struct{
	TIM_HandleTypeDef* tim;			//This parameter holds timer address.
	uint32_t channel;				//This parameter holds timer channel for motor pwm singal output.

	GPIO_TypeDef* port[2];			//This parameter holds GPIO ports. port[0] -> 0. pin port; port[1] -> 1. pin port.
	uint16_t pin[2];				//This parameter holds GPIO pins. pin[0] -> 0. pin; pin[1] -> 1. pin.
}L298N_MotorConfig_t;

typedef struct{
	L298N_MotorConfig_t *L298N_MotorConfig;

	uint32_t speed;			//This parameter holds motor speed. It is able to range @L298N_MOTOR_SPEED_MIN - @L298N_MOTOR_SPEED_MAX.
	int8_t direction;		//This parameter holds motor direction. @L298N_MOTOR_DIRECTION
}L298N_MotorHandler_t;

//Function Prototype
L298N_StatusTypeDef L298N_MotorInit(L298N_MotorHandler_t* motor, L298N_MotorConfig_t* motorConfig);
L298N_StatusTypeDef L298N_setMotorSpeed(L298N_MotorHandler_t* motor, uint32_t speed);
uint32_t L298N_getMotorSpeed(L298N_MotorHandler_t* motor);
L298N_StatusTypeDef L298N_setMotorDirection(L298N_MotorHandler_t* motor, int8_t direction);
int8_t L298N_getMotorDirection(L298N_MotorHandler_t* motor);



#endif /* MODULES_L298N_INC_L298N_H_ */
