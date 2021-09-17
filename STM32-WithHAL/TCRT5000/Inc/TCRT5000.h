/*
 * TCRT5000.h
 *
 *  Created on: Feb 18, 2021
 *      Author: bilginistAdam
 * 		E-Mail: bilginenesismail@gmail.com
 * 		Info  : None
 */

#ifndef __TCRT5000_H_
#define __TCRT5000_H_

/* Includes *******************************************/
#include "main.h"

/* Defines ********************************************/

/* TypeDef ********************************************/
typedef enum{
	TCRT_Reset	=	0U,									/*< />*/
	TCRT_Set	=	!TCRT_Reset,						/*< />*/
}TCRT_FlagStatus_t;

typedef struct{
	GPIO_TypeDef			*trigPort;					/*<This parameter holds the Trig pin port. />*/
	uint16_t				trigPin;					/*<This parameter holds the trig pin number. />*/
}TCRT_Config_t;

typedef struct{
	TCRT_Config_t 			Config;						/*< />*/
	TCRT_FlagStatus_t		Status;						/*< />*/
}TCRT_Handler_t;

/* Function Prototypes ********************************/
/* Init */
HAL_StatusTypeDef TCRT_Init(TCRT_Handler_t * htcrt);

/* Set */
HAL_StatusTypeDef TCRT_setStatus(TCRT_Handler_t * htcrt, TCRT_FlagStatus_t FlagStatus);

/* Get */
TCRT_FlagStatus_t TCRT_getStatus(TCRT_Handler_t * htcrt);

/* Toogle */

/* Handler */
void TCRT_IT_Handler(TCRT_Handler_t * htcrt, uint16_t GPIO_Pin);

#endif /* __TCRT5000_H_ */
