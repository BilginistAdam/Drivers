/*
 * TCRT5000.c
 *
 *  Created on: Feb 18, 2021
 *      Author: bilginistAdam
 * 		E-Mail: bilginenesismail@gmail.com
 * 		Info  : Check header file info before using.
 */

/* Includes ******************************************/
#include "TCRT5000.h"

/* Defines *******************************************/

/* External Variable *********************************/

/* Function Definitions ******************************/
/* Init */
HAL_StatusTypeDef TCRT_Init(TCRT_Handler_t * htcrt){
	//SET status to zero
	if(HAL_OK != TCRT_setStatus(htcrt, HAL_GPIO_ReadPin(htcrt->Config.trigPort, htcrt->Config.trigPin))){
		return HAL_ERROR;
	}

	return HAL_OK;
}

/* Set */
HAL_StatusTypeDef TCRT_setStatus(TCRT_Handler_t * htcrt, TCRT_FlagStatus_t FlagStatus){
	if(FlagStatus == TCRT_Set || FlagStatus == TCRT_Reset){
		htcrt->Status = FlagStatus;
		return HAL_OK;
	}
	return HAL_ERROR;
}

/* Get */
TCRT_FlagStatus_t TCRT_getStatus(TCRT_Handler_t * htcrt){
	return htcrt->Status;
}

/* Handler */
void TCRT_IT_Handler(TCRT_Handler_t * htcrt, uint16_t GPIO_Pin){
	if(htcrt->Config.trigPin == GPIO_Pin){
		htcrt->Status = HAL_GPIO_ReadPin(htcrt->Config.trigPort, htcrt->Config.trigPin);
	}
}
