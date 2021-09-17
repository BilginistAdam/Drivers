/*
 * clock.c
 *
 *  Created on: 5 Oca 2021
 *      Author: bilginist
 *	E-Main: bilginenesismail@gmail.com
 */

/* Includes *************************************/
#include "clock.h"
/* Defines *************************************/

/* Private Variables ***************************/

/* Define Functions ****************************/
/************************************
 * @fn 	  	 : CLK_Init
 * @brief 	 : This function initializes clock.
 *
 * @param[0] : This parameter holds Clock pointer for init.
 * @param[1] : This parameter holds TickFreq of timer.
 * @return	 : CLK_Status_t
 *
 * @Note     : None
 ************************************/
CLK_Status_t CLK_Init(CLK_Handler_t* Clock, CLK_TickFreq_t TickFreq){
	if (TickFreq == CLK_TICK_FREQ_1KHZ || TickFreq == CLK_TICK_FREQ_100HZ || TickFreq == CLK_TICK_FREQ_10HZ){
		Clock->Freq = TickFreq;
		Clock->Time = 0;
		return CLK_OK;
	}else{
		return CLK_ERROR;
	}
}

/************************************
 * @fn 	  	 : CLK_Inc
 * @brief 	 : This function is increased time.
 *
 * @param[0] : This parameter holds Clock pointer for increment
 * @return	 : None
 *
 * @Note     : None
 ************************************/
void CLK_Inc(CLK_Handler_t * Clock){
	Clock->Time += Clock->Freq;
}

/************************************
 * @fn 	  	 : CLK_Inc
 * @brief 	 : This function is increased time.
 *
 * @param[0] : This parameter holds Clock pointer for increment
 * @return	 : uint32_t time data that type of ms.
 *
 * @Note     : None
 ************************************/
uint32_t CLK_Now(CLK_Handler_t * Clock){
	return Clock-> Time;
}

/************************************
 * @fn 	  	 : CLK_ElapsedTime
 * @brief 	 : This function is increased time.
 *
 * @param[0] : This parameter holds dat of start time.
 * @param[1] : This parameter holds data of now time.
 * @return	 : uint32_t elapsed time data that type of ms.
 *
 * @Note     : Also, if variable of counter was overflow,
 * 			   this function calculates elapsed time.
 ************************************/
uint32_t CLK_ElapsedTime(uint32_t StartTime, uint32_t Now){
	if(StartTime > Now){
		uint32_t u32_carry = CLK_MAX_TIME - StartTime;
		return (u32_carry + Now) + 1;
	}else{
		return Now - StartTime;
	}
}
