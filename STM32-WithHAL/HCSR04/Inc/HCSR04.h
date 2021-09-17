/*
 * HCSR04.h
 *
 *  Created on: Feb 15, 2021
 *      Author: bilgi
 *
 *      NOTE:
 *      This driver written to HC SR04 Ultrasonic distance sensor. If you are using this driver,
 *      you have to meet some requirements. (ref : @HCSR04_REQS)
 *
 *      First step initialization :
 *         	0 - Setup your device for your requirement.
 *      	1 - Set a timer and a channel input capture direct mode. (@HCSR04_REQ_TIMER)
 *      	2 - Set a GPIO Pin output for trigger.
 *      	3 - Create a handler variable for HCSR and its type is HCSR_Handler_t also
 *      		initialization the this variable parameter like a peripheral parameter
 *      		(for example htimx initialization).
 *      	4 - Call HAL_TIM_IC_CaptureCallback function and use HCSR_IT_Handler() in here.
 *      	5 - Write a case for the read and use HCSR_Read() in this case.
 *      		(For example: while loop in main function.) (@HCSR_REQ_READ)
 *			6 - Use HCSR_Handler for calculating distance.
 *
 *		@HCSR04_REQS
 *			@HCSR04_REQ_TIMER
 *				0 - You must set the IC mode conf. (like ARR value
 *				Note: if your timer ARR is 16bit, ARR value is 0xffff - 1)
 *				1 - You must set the frequency of the timer to 1MHz.
 *				2 - You must set timer trigger to rising edge.
 *				3 - You must set timer interrupt or IC interrupt.
 *			@HCSR04_REQ_READ
 *				1 - You should use delay, if you are using this function. Maybe, Eco signal
 *					from previos reading confiuses your sensor.
 */

#ifndef __HCSR04_H_
#define __HCSR04_H_

/* Includes *******************************************/
#include "main.h"

/* Defines ********************************************/
#define HCSR04_TRIG_TIME					10U						/*< Using IO trigger for at least 10us high level signal />*/
#define HCSR04_DISTANCE_COEFFICIENT			0.034					/*< This parameter is the velocity of the ultra sonic. (340M/S). />*/
/* Typedef ********************************************/
typedef enum{
	HCSR_RESET		= 0U,
	HCSR_SET		= !HCSR_RESET,
}HCSR_FlagStatus_t;

typedef struct{
	TIM_HandleTypeDef		*htim;						/*<This parameter is a pointer and points to timer address and parameters address. />*/
	uint32_t				channel;					/*<This parameter holds the channel of the timer for echo pin. />*/
	uint8_t					ITcmd;
	GPIO_TypeDef			*trigPort;					/*<This parameter holds the Trig pin port. />*/
	uint16_t				trigPin;					/*<This parameter holds the trig pin number. />*/
}HCSR_Config_t;

typedef struct{
	uint16_t 				risingEdge;					/*<This parameter holds the counter value of the timer when timer detect rising edge interrupt. />*/
	uint16_t				fallingEdge;				/*<This parameter holds the counter value of the timer when timer detect falling edge interrupt. />*/
	uint16_t				elapsedTime;				/*<This parameter holds the time value of the between two interrupt. *>*/
	HCSR_FlagStatus_t		isFirstCaptured;			/*<This parameter is a flag for the first capture done or not. />*/
}HCSR_IC_Data_t;

typedef struct{
	HCSR_Config_t			config;						/*<This parameter holds the data of the config for initialization. />*/
	HCSR_IC_Data_t			IC_Data;					/*<This parameter holds the datas of the capturing from timer. />*/
	HCSR_FlagStatus_t		isDone;						/*<This parameter is a flag for the measurement is done or not. />*/
	float					distance;					/*<This parameter holds the value of the distance. />*/
}HCSR_Handlder_t;

/* Function Prototypes ********************************/
/* Init */
HAL_StatusTypeDef HCSR_Init(HCSR_Handlder_t *hhcsr);

/* SET */
HAL_StatusTypeDef HCSR_setFirstCaptured(HCSR_Handlder_t * hhcsr, HCSR_FlagStatus_t FlagStatus);
HAL_StatusTypeDef HCSR_setDoneFlag(HCSR_Handlder_t * hhcsr, HCSR_FlagStatus_t FlagStatus);
HAL_StatusTypeDef HCSR_setTimIT(HCSR_Handlder_t * hhcsr, HCSR_FlagStatus_t FlagStatus);
HAL_StatusTypeDef HCSR_setTimItCmd(HCSR_Handlder_t * hhcsr);

/* GET */
HCSR_FlagStatus_t HCSR_getFirstCaptured(HCSR_Handlder_t * hhcsr);
HCSR_FlagStatus_t HCSR_getDone(HCSR_Handlder_t * hhcsr);
float HCSR_getDistance(HCSR_Handlder_t * hhcsr);

/* CAL*/
void HCSR_calElpasedTime(HCSR_Handlder_t * hhcsr);
void HCSR_calDistance(HCSR_Handlder_t * hhcsr);

/* READ */
void HCSR_Read(HCSR_Handlder_t * hhcsr);

/* Handlers */
void HCSR_Handler(HCSR_Handlder_t * hhcsr);
void HCSR_IC_Handler(HCSR_Handlder_t * hhcsr, TIM_HandleTypeDef * htim);

/* us Delay */
void HCSR_Delay(HCSR_Handlder_t * hhcsr, uint32_t time);









#endif /* __HCSR04_H_ */
