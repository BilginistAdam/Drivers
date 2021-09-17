/*
 * L298N.c
 *
 *  Created on: Jan 2, 2021
 *      Author: bilginist
 *      e-mail: bilginenesismail@gmail.com
 */

//Includes
#include "L298N.h"

//Defines

//Variables


//Functions
/************************************
 * @fn 	  	 : L298N_MotorInit
 * @brief 	 : This function initializes L298N Motor Driver.
 *
 * @param[0] : This parameter holds motor info.
 * @return	 : L298N_StatusTypeDef
 *
 * @Note     : None
 ************************************/

L298N_StatusTypeDef L298N_MotorInit(L298N_MotorHandler_t* motor, L298N_MotorConfig_t *motorConfig){
	//0 - Set Init Motor
	motor->L298N_MotorConfig->tim = motorConfig->tim;
	motor->L298N_MotorConfig->channel = motorConfig->channel;
	motor->L298N_MotorConfig->port[0] = motorConfig->port[0];
	motor->L298N_MotorConfig->port[1] = motorConfig->port[1];
	motor->L298N_MotorConfig->pin[0] = motorConfig->pin[0];
	motor->L298N_MotorConfig->pin[1] = motorConfig->pin[1];

	//1 - Start Timer PWM Output
	  if (HAL_OK != HAL_TIM_PWM_Start(motor->L298N_MotorConfig->tim, motor->L298N_MotorConfig->channel))
	  {
	    Error_Handler();
	  }

	//2- Set Motor Speed to Stop
	if(L298N_OK != L298N_setMotorSpeed(motor, L298N_MOTOR_SPEED_STOP)){
		return L298N_ERROR;
	}

	//3- Set Motor Direction to Stop
	if(L298N_OK !=L298N_setMotorDirection(motor, L298N_MOTOR_DIRECTION_STOP)){
		return L298N_ERROR;
	}

	return L298N_OK;
}

/************************************
 * @fn 	  	 : L298N_setMotorSpeed
 * @brief 	 : This function sets motor speed.
 *
 * @param[0] : This parameter holds motor info.
 * @param[1] : This parameter holds motor speed.
 * @return	 : L298N_StatusTypeDef
 *
 * @Note     : None
 ************************************/
L298N_StatusTypeDef L298N_setMotorSpeed(L298N_MotorHandler_t* motor, uint32_t speed){
	if(speed > L298N_MOTOR_SPEED_MAX || speed < L298N_MOTOR_SPEED_MIN){
		//Update speed data from motor.
		motor->speed = speed;
		//Change motor speed.
		__HAL_TIM_SET_COMPARE(motor->L298N_MotorConfig->tim, motor->L298N_MotorConfig->channel, motor->speed);
		return L298N_OK;
	}else{
		return L298N_ERROR;
	}
}

/************************************
 * @fn 	  	 : L298N_getMotorSpeed
 * @brief 	 : This function gets motor speed.
 *
 * @param[0] : This parameter holds motor datas.
 * @return	 : Speed data (uint32_t)
 *
 * @Note     : None
 ************************************/
uint32_t L298N_getMotorSpeed(L298N_MotorHandler_t* motor){
	return motor->speed;
}

/************************************
 * @fn 	  	 : L298N_setMotorDirection
 * @brief 	 : This function sets motor direction.
 *
 * @param[0] : This parameter holds motor datas.
 * @param[1] : This parameter holds motor direction data.
 * @return	 : L298N_StatusTypeDef
 *
 * @Note     : None
 ************************************/
L298N_StatusTypeDef L298N_setMotorDirection(L298N_MotorHandler_t* motor, int8_t direction){
	//Direction Forward
	if(direction == L298N_MOTOR_DIRECTION_FORWARD){
		//SET GPIO-1 1 and RESET GPIO-2 0
		HAL_GPIO_WritePin(motor->L298N_MotorConfig->port[0], motor->L298N_MotorConfig->pin[0], L298N_SET);
		HAL_GPIO_WritePin(motor->L298N_MotorConfig->port[1], motor->L298N_MotorConfig->pin[1], L298N_RESET);
		return L298N_OK;
	}
	//Direction Backward
	else if (direction == L298N_MOTOR_DIRECTION_BACKWARD){
		//SET GPIO-1 0 and RESET GPIO-2 1
		HAL_GPIO_WritePin(motor->L298N_MotorConfig->port[0], motor->L298N_MotorConfig->pin[0], L298N_RESET);
		HAL_GPIO_WritePin(motor->L298N_MotorConfig->port[1], motor->L298N_MotorConfig->pin[1], L298N_SET);
		return L298N_OK;
	}
	//Direction Stop
	else if(direction == L298N_MOTOR_DIRECTION_STOP){
		//SET GPIO-1 0 and RESET GPIO-2 0
		HAL_GPIO_WritePin(motor->L298N_MotorConfig->port[0], motor->L298N_MotorConfig->pin[0], L298N_RESET);
		HAL_GPIO_WritePin(motor->L298N_MotorConfig->port[1], motor->L298N_MotorConfig->pin[1], L298N_RESET);
		return L298N_OK;
	}
	//Error
	else{
		return L298N_ERROR;
	}
}

/************************************
 * @fn 	  	 : L298N_setMotorDirection
 * @brief 	 : This function gets motor direction.
 *
 * @param[0] : This parameter holds motor datas.
 * @param[1] : This parameter holds motor direction data.
 * @return	 : Motor Direction.
 *
 * @Note     : None
************************************/
int8_t L298N_getMotorDirection(L298N_MotorHandler_t *motor){
	return motor->direction;
}

