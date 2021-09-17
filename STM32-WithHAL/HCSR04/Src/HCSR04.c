/*
 * HCSR04.c
 *
 *  Created on: Feb 15, 2021
 *      Author: bilginistAdam
 * 		E-Mail: bilginenesismail@gmail.com
 * 		Info  : Check header file info before using.
 *
 */
/* Includes *******************************************/
#include "hcsr04.h"
/* Defines ********************************************/

/* Variabels ******************************************/

/* Function Prototypes ********************************/
/* Init */
/************************************
 * @fn		: HCSR_Init()
 * @brief	: This function initializations the requirements of the setup for the module.
 *
 * @param[0]: This parameter holds the address of the module's handler variable.
 * @return	: HAL_StatusTypeDef
 *
 * @Note	: None
 ************************************/
HAL_StatusTypeDef HCSR_Init(HCSR_Handlder_t *hhcsr){
	//Start timer for input capturing
	if(HAL_OK != HAL_TIM_IC_Start_IT(hhcsr->config.htim, hhcsr->config.channel)){
		return HAL_ERROR;
	}

	//SET Timer IT CMD
	if(HAL_OK != HCSR_setTimItCmd(hhcsr)){
		return HAL_ERROR;
	}


	//Reset flags
	if(HAL_OK != HCSR_setDoneFlag(hhcsr, HCSR_SET)){
		return HAL_ERROR;
	}

	if(HAL_OK != HCSR_setFirstCaptured(hhcsr, HCSR_RESET)){
		return HAL_ERROR;
	}

	return HAL_OK;
}

/* SET */
/************************************
 * @fn		: HCSR_setFirstCaptured()
 * @brief	: This function sets the status of the first capture flag.
 *
 * @param[0]: This parameter holds the address of the module's handler variable.
 * @param[1]: This parameter holds the status of the flag.
 * @return	: HAL_StatusTypeDef
 *
 * @Note	: None
 ************************************/
HAL_StatusTypeDef HCSR_setFirstCaptured(HCSR_Handlder_t * hhcsr, HCSR_FlagStatus_t FlagStatus){

	if(FlagStatus == HCSR_SET || FlagStatus == HCSR_RESET){
		hhcsr->IC_Data.isFirstCaptured = FlagStatus;
		return HAL_OK;
	}

	return HAL_ERROR;
}

/************************************
 * @fn		: HCSR_setDoneFlag()
 * @brief	: This function sets the status of the done flag.
 *
 * @param[0]: This parameter holds the address of the module's handler variable.
 * @param[1]: This parameter holds the status of the flag.
 * @return	: HAL_StatusTypeDef
 *
 * @Note	: None
 ************************************/
HAL_StatusTypeDef HCSR_setDoneFlag(HCSR_Handlder_t * hhcsr, HCSR_FlagStatus_t FlagStatus){

	if(FlagStatus == HCSR_SET || FlagStatus == HCSR_RESET){
		hhcsr->isDone = FlagStatus;
		return HAL_OK;
	}

	return HAL_ERROR;
}

/************************************
 * @fn		: HCSR_setTimIT()
 * @brief	: This function sets the status of the timer input capture interrupt.
 *
 * @param[0]: This parameter holds the address of the module's handler variable.
 * @param[1]: This parameter holds the status of the flag.
 * @return	: HAL_StatusTypeDef
 *
 * @Note	: None
 ************************************/
HAL_StatusTypeDef HCSR_setTimIT(HCSR_Handlder_t * hhcsr, HCSR_FlagStatus_t FlagStatus){

	if(FlagStatus == HCSR_SET){
		__HAL_TIM_ENABLE_IT(hhcsr->config.htim, hhcsr->config.ITcmd);
		return HAL_OK;
	}
	else if(FlagStatus == HCSR_RESET){
		__HAL_TIM_DISABLE_IT(hhcsr->config.htim, hhcsr->config.ITcmd);
		return HAL_OK;
	}else{
		return HAL_ERROR;
	}
}

/************************************
 * @fn		: HCSR_setTimItCmd()
 * @brief	: This function sets the definition value of the timer channel interrupt.
 *
 * @param[0]: This parameter holds the address of the module's handler variable.
 * @param[1]: This parameter holds the status of the flag.
 * @return	: HAL_StatusTypeDef
 *
 * @Note	: None
 ************************************/
HAL_StatusTypeDef HCSR_setTimItCmd(HCSR_Handlder_t * hhcsr){
	#ifdef TIM_CHANNEL_1
		if(hhcsr->config.channel == TIM_CHANNEL_1){
			hhcsr->config.ITcmd = TIM_IT_CC1;
			return HAL_OK;
		}
	#endif	/* TIM_CHANNEL_1*/
	#ifdef TIM_CHANNEL_2
		else if(hhcsr->config.channel == TIM_CHANNEL_2){
			hhcsr->config.ITcmd = TIM_IT_CC2;
			return HAL_OK;
		}
	#endif	/* TIM_CHANNEL_2*/
	#ifdef TIM_CHANNEL_3
		else if(hhcsr->config.channel == TIM_CHANNEL_3){
			hhcsr->config.ITcmd = TIM_IT_CC3;
			return HAL_OK;
		}
	#endif	/* TIM_CHANNEL_3*/
	#ifdef TIM_CHANNEL_4
		else if(hhcsr->config.channel == TIM_CHANNEL_4){
			hhcsr->config.ITcmd = TIM_IT_CC4;
			return HAL_OK;
		}
	#endif	/* TIM_CHANNEL_4*/
	#ifdef TIM_CHANNEL_5
		else if(hhcsr->config.channel == TIM_CHANNEL_5){
			hhcsr->config.ITcmd = TIM_IT_CC5;
			return HAL_OK;
		}
	#endif	/* TIM_CHANNEL_5*/
	#ifdef TIM_CHANNEL_6
		else if(hhcsr->config.channel == TIM_CHANNEL_6){
			hhcsr->config.ITcmd = TIM_IT_CC6;
			return HAL_OK;
		}
	#endif	/* TIM_CHANNEL_6*/

	return HAL_ERROR;
}

/* GET */
/************************************
 * @fn		: HCSR_getFirstCaptured()
 * @brief	: This function gives the status of the first captured flag.
 *
 * @param[0]: This parameter holds the address of the module's handler variable.
 *
 * @return	: HCSR_FlagStatus_t
 *
 * @Note	: None
 ************************************/
HCSR_FlagStatus_t HCSR_getFirstCaptured(HCSR_Handlder_t * hhcsr){
	return hhcsr->IC_Data.isFirstCaptured;
}

/************************************
 * @fn		: HCSR_getDone()
 * @brief	: This function gives the status of the done flag.
 *
 * @param[0]: This parameter holds the address of the module's handler variable.
 *
 * @return	: HCSR_FlagStatus_t
 *
 * @Note	: None
 ************************************/
HCSR_FlagStatus_t HCSR_getDone(HCSR_Handlder_t * hhcsr){
	return hhcsr->isDone;
}

/************************************
 * @fn		: HCSR_getDistance()
 * @brief	: This function gives the value of the distance.
 *
 * @param[0]: This parameter holds the address of the module's handler variable.
 *
 * @return	: float distance
 *
 * @Note	: None
 ************************************/
float HCSR_getDistance(HCSR_Handlder_t * hhcsr){
	return hhcsr->distance;
}

/* CAL*/

/************************************
 * @fn		: HCSR_calElpasedTime()
 * @brief	: This function calculates the value of the elapsed time from rising interrupt to falling interrupt.
 *
 * @param[0]: This parameter holds the address of the module's handler variable.
 *
 * @return	: None
 *
 * @Note	: None
 ************************************/
void HCSR_calElpasedTime(HCSR_Handlder_t * hhcsr){
	if(hhcsr->IC_Data.fallingEdge > hhcsr->IC_Data.risingEdge){
		hhcsr->IC_Data.elapsedTime = hhcsr->IC_Data.fallingEdge - hhcsr->IC_Data.risingEdge;
	}else if(hhcsr->IC_Data.fallingEdge < hhcsr->IC_Data.risingEdge){
		hhcsr->IC_Data.elapsedTime = (0xffff - hhcsr->IC_Data.risingEdge) + hhcsr->IC_Data.fallingEdge;
	}
}

/************************************
 * @fn		: HCSR_calElpasedTime()
 * @brief	: This function calculates the value of the distance.
 *
 * @param[0]: This parameter holds the address of the module's handler variable.
 *
 * @return	: None
 *
 * @Note	: None
 ************************************/
void HCSR_calDistance(HCSR_Handlder_t * hhcsr){
	hhcsr->distance = hhcsr->IC_Data.elapsedTime * HCSR04_DISTANCE_COEFFICIENT / 2;
}

/* READ */
/************************************
 * @fn		: HCSR_Read()
 * @brief	: This function set trigger pin and enable timer IC interrupt.
 *
 * @param[0]: This parameter holds the address of the module's handler variable.
 *
 * @return	: None
 *
 * @Note	: None
 ************************************/
void HCSR_Read(HCSR_Handlder_t * hhcsr){
	if(HCSR_getDone(hhcsr)){
		HAL_GPIO_WritePin(hhcsr->config.trigPort, hhcsr->config.trigPin, SET);
		HCSR_Delay(hhcsr, HCSR04_TRIG_TIME);
		HAL_GPIO_WritePin(hhcsr->config.trigPort, hhcsr->config.trigPin, RESET);

		HCSR_setTimIT(hhcsr, HCSR_SET);
	}
}

/* Handlers */
/************************************
 * @fn		: HCSR_Handlder()
 * @brief	: This function is handler for calculate to distance.
 *
 * @param[0]: This parameter holds the address of the module's handler variable.
 *
 * @return	: None
 *
 * @Note	: None
 ************************************/
void HCSR_Handler(HCSR_Handlder_t * hhcsr){
	if(!HCSR_getDone(hhcsr)){
		//Cal Elapsed Time
		HCSR_calElpasedTime(hhcsr);

		//Cal Distance
		HCSR_calDistance(hhcsr);

		//Set Done Flag
		HCSR_setDoneFlag(hhcsr, HCSR_SET);
	}
}

/************************************
 * @fn		: HCSR_IC_Handler()
 * @brief	: This function is handler for getting values from the interrupt of the timer IC CallBack.
 *
 * @param[0]: This parameter holds the address of the module's handler variable.
 *
 * @return	: None
 *
 * @Note	: You must use this function in Timer Input Capture Callback function.
 ************************************/
void HCSR_IC_Handler(HCSR_Handlder_t * hhcsr, TIM_HandleTypeDef * htim){
	//Check timer and channel
	if(htim == hhcsr->config.htim){
		if(htim->Channel == hhcsr->config.htim->Channel){
			//check done Flag
			if (HCSR_getDone(hhcsr)){
				//check is first captured flag
				if(HCSR_getFirstCaptured(hhcsr) == HCSR_RESET){
					//Get rising Edge
					hhcsr->IC_Data.risingEdge = __HAL_TIM_GET_COMPARE(hhcsr->config.htim, hhcsr->config.channel);

					//Set is first captured flag
					HCSR_setFirstCaptured(hhcsr, HCSR_SET);

					//Change timer IC polarity to falling edge
					__HAL_TIM_SET_CAPTUREPOLARITY(htim, hhcsr->config.channel, TIM_INPUTCHANNELPOLARITY_FALLING);
				}else if(HCSR_getFirstCaptured(hhcsr) == HCSR_SET){
					//Get Falling Edge
					hhcsr->IC_Data.fallingEdge =  __HAL_TIM_GET_COMPARE(hhcsr->config.htim, hhcsr->config.channel);

					//Reset Counter
					__HAL_TIM_SET_COUNTER(htim, 0);

					//Change timer IC Polarity to rising edge
					__HAL_TIM_SET_CAPTUREPOLARITY(htim, hhcsr->config.channel, TIM_INPUTCHANNELPOLARITY_RISING);

					//Reset Done Flag
					HCSR_setDoneFlag(hhcsr, HCSR_RESET);

					//Reset is fisrt captured flag
					HCSR_setFirstCaptured(hhcsr, HCSR_RESET);

					//SET Tim IC
					HCSR_setTimIT(hhcsr, HCSR_RESET);
				}
			}
		}
	}
}

/* us Delay */
/************************************
 * @fn		: HCSR_Delay()
 * @brief	: This function delays the system at level of microsecond.
 *
 * @param[0]: This parameter holds the address of the module's handler variable.
 *
 * @return	: None
 *
 * @Note	: TODO
 ************************************/
void HCSR_Delay(HCSR_Handlder_t * hhcsr, uint32_t time){
	uint32_t _STime = __HAL_TIM_GET_COUNTER(hhcsr->config.htim);
	uint32_t _cValue = 0;
	while(_cValue < time){
		_cValue = __HAL_TIM_GET_COUNTER(hhcsr->config.htim) - _STime;
	}
}



















