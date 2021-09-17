/*
 * L298N.c
 *
 *  Created on: Jan 12, 2021
 *      Author: bilgi
 */
/* INCLUDES **********************************************************/
#include "L298N.h"

/* DEFINES ***********************************************************/

/* TYPEDEFS **********************************************************/

/* Function Definitions **********************************************/
/* Init */

/************************************
 * @fn 	  	 : L298N_MotorInit
 * @brief 	 : This function initializes L298N Motor Driver.
 *
 * @param[0] : This parameter holds the address of the hmotor handler.
 * @return	 : HAL_StatusTypeDef
 *
 * @Note     : None
 ************************************/
HAL_StatusTypeDef L298N_MotorInit(L298N_Handler_t * hmotor){
		//Start PWM Signal
		if(HAL_OK != HAL_TIM_PWM_Start(hmotor->Config.Timer, hmotor->Config.Channel)){
			return HAL_ERROR;
		}

		//Set Motor Speed to Stop Speed
		if(HAL_OK != L298N_setMotorSpeed(hmotor, L298N_MOTOR_SPEED_STOP)){
			return HAL_ERROR;
		}

		//Set Motor Direction to Stop
		if(HAL_OK != L298N_setMotorDirection(hmotor, L298N_Stop)){
			return HAL_ERROR;
		}

		return HAL_OK;
}

/* Get */
/************************************
 * @fn 	  	 : L298N_getMotorSpeed
 * @brief 	 : This function gives speed value of hmotor.
 *
 * @param[0] : This parameter holds the address of the hmotor handler.
 * @return	 : uint32_t the speed value of the hmotor.
 *
 * @Note     : None
 ************************************/
uint32_t L298N_getMotorSpeed(L298N_Handler_t* hmotor){
	return hmotor->speed;
}

/************************************
 * @fn 	  	 : L298N_getMotorSpeed
 * @brief 	 : This function gives direction data of hmotor.
 *
 * @param[0] : This parameter holds the address of the hmotor handler.
 * @return	 : L298N_Direction_t the direction data of the hmotor.
 *
 * @Note     : None
 ************************************/
L298N_Direction_t L298N_getMotorDirection(L298N_Handler_t* hmotor){
	return hmotor->direction;
}

//set

/************************************
 * @fn 	  	 : L298N_setMotorSpeed
 * @brief 	 : This function set  motor speed.
 *
 * @param[0] : This parameter holds the address of the hmotor handler.
 * @param[1] : This parameter is speed value which will set.
 * @return	 : HAL_StatusTypeDef
 *
 * @Note     : None
 ************************************/
HAL_StatusTypeDef L298N_setMotorSpeed(L298N_Handler_t * hmotor, uint32_t speed){
	if((L298N_MOTOR_SPEED_MIN <= speed) && (L298N_MOTOR_SPEED_MAX >= speed)){
		//Update Motor Speed
		hmotor->speed = speed;
		//Set PWM Signal
		__HAL_TIM_SET_COMPARE(hmotor->Config.Timer, hmotor->Config.Channel, speed);

		return HAL_OK;
	}else{
		return HAL_ERROR;
	}
}

/************************************
 * @fn 	  	 : L298N_setMotorSpeed
 * @brief 	 : This function set  motor direction.
 *
 * @param[0] : This parameter holds the address of the hmotor handler.
 * @param[1] : This parameter is direction value which will set.
 * @return	 : HAL_StatusTypeDef
 *
 * @Note     : None
 ************************************/
HAL_StatusTypeDef L298N_setMotorDirection(L298N_Handler_t * hmotor, int8_t direction){
	if(direction == L298N_Forward){
			//SET GPIO-1 1 and RESET GPIO-2 0
			HAL_GPIO_WritePin(hmotor->Config.port[0], hmotor->Config.pin[0], SET);
			HAL_GPIO_WritePin(hmotor->Config.port[1], hmotor->Config.pin[1], RESET);
			//Update direction
			hmotor->direction = direction;
			return HAL_OK;
		}
		//Direction Backward
		else if (direction == L298N_Backward){
			//SET GPIO-1 0 and RESET GPIO-2 1
			HAL_GPIO_WritePin(hmotor->Config.port[0], hmotor->Config.pin[0], RESET);
			HAL_GPIO_WritePin(hmotor->Config.port[1], hmotor->Config.pin[1], SET);
			//Update direction
			hmotor->direction = direction;
			return HAL_OK;
		}
		//Direction Stop
		else if(direction == L298N_Stop){
			//SET GPIO-1 0 and RESET GPIO-2 0
			HAL_GPIO_WritePin(hmotor->Config.port[0], hmotor->Config.pin[0], RESET);
			HAL_GPIO_WritePin(hmotor->Config.port[1], hmotor->Config.pin[1], RESET);
			//Update direction
			hmotor->direction = direction;
			return HAL_OK;
		}
		//Error
		else{
			return HAL_ERROR;
		}
}
