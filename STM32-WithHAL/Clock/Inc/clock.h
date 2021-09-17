/*
 * clock.h
 *
 *  Created on: 5 Oca 2021
 *      Author: bilginistAdam
 *      E-Mail: bilginenesismail@gmail.com
 */
/* NOTES:
 *
 *	If you are using this module, you must use a timer for periodic counter.
 *	Also, you must add function of clk_inc in this timer interrupt or sysTick timer interrupt.
 *	You should define time freq in function of clk_init().
 *
 */

#ifndef _CLOCK_H_
#define _CLOCK_H_

/* Includes *************************************/
#include "main.h"

/* Defines **************************************/
#define CLK_TIM_MAX_VALUE 0xFFFFFFFFU

/* TypeDefs *************************************/

typedef enum{
	CLK_OK		= 0x00U,
	CLK_ERROR	= 0x01U
}CLK_Status_t;

typedef enum{
	  CLK_RESET 	= 0U,
	  CLK_SET 		= !CLK_RESET
}CLK_FlagStatus_t;

typedef enum{
	CLK_TICK_FREQ_1HZ		= 1000000U,
	CLK_TICK_FREQ_10HZ		= 100000U,
	CLK_TICK_FREQ_100HZ		= 10000U,
	CLK_TICK_FREQ_1KHZ		= 1000U,
	CLK_TICK_FREQ_10KHZ		= 100U,
	CLK_TICK_FREQ_100KHZ	= 10U,
	CLK_TICK_FREQ_1MHZ		= 1U,
	CLK_TICK_FREQ_DEFAULT	= CLK_TICK_FREQ_1KHZ
}CLK_TickFreq_t;

typedef struct{
	CLK_TickFreq_t 				Freq;
	TIM_HandleTypeDef 		* 	htim;
}CLK_Config_t;

typedef struct{
	CLK_Config_t	Config;		/* It holds the Config of the Clock. */
	uint32_t 		Time;		/* It holds time data. */
}CLK_Handler_t;

/* Define Functions Prototype *******************/
// Init
CLK_Status_t CLK_Init(CLK_Handler_t* hclk);

// GET
uint32_t CLK_getTime(CLK_Handler_t * hclk);

// Calculator
uint32_t CLK_calElapsedTime(uint32_t StartTime, uint32_t Now);

// IT Handler
void CLK_incTick(CLK_Handler_t * hclk, TIM_HandleTypeDef * htim);

#endif /* _CLOCK_H_ */
