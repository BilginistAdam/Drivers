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
#define CLK_MAX_TIME	4294967295UL

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
  CLK_TICK_FREQ_10HZ		= 100U,
  CLK_TICK_FREQ_100HZ		= 10U,
  CLK_TICK_FREQ_1KHZ		= 1U,
  CLK_TICK_FREQ_DEFAULT		= CLK_TICK_FREQ_1KHZ
}CLK_TickFreq_t;

typedef struct{
	CLK_TickFreq_t	Freq;		/* It holds counter freq. */
	uint32_t 		Time;		/* It holds time data. */
}CLK_Handler_t;

/* Define Functions Prototype *******************/
CLK_Status_t CLK_Init(CLK_Handler_t* Clcok, CLK_TickFreq_t TickFreq);
void CLK_Inc(CLK_Handler_t * Clock);
uint32_t CLK_Now(CLK_Handler_t * Clock);
uint32_t CLK_ElapsedTime(uint32_t StartTime, uint32_t Now);


#endif /* _CLOCK_H_ */
