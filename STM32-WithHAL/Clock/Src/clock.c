/*
 * clock.c
 *
 *  Created on: 5 Oca 2021
 *      Author: bilgi
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
 * @param[0] : This parameter holds hclk pointer.
 *
 * @return	 : CLK_Status_t
 *
 * @Note     : None
 ************************************/
CLK_Status_t CLK_Init(CLK_Handler_t* hclk){
	// Start htim
	if (HAL_OK != HAL_TIM_Base_Start_IT(hclk->Config.htim)){
		return CLK_ERROR;
	}

	hclk->Time = 0;
	return CLK_OK;
}

// GET
/************************************
 * @fn 	  	 : CLK_getTime
 * @brief 	 : This function is increased time.
 *
 * @param[0] : This parameter holds hclk pointer.
 * @return	 : uint32_t time data
 *
 * @Note     : None
 ************************************/
uint32_t CLK_getTime(CLK_Handler_t * hclk){
	return hclk->Time;
}


// Calculator
/************************************
 * @fn 	  	 : CLK_ElapsedTime
 * @brief 	 : This function is increased time.
 *
 * @param[0] : This parameter holds dat of start time.
 * @param[1] : This parameter holds data of now time.
 *
 * @return	 : uint32_t elapsed time between with two inputs.
 *
 * @Note     : None
 ************************************/

uint32_t CLK_calElapsedTime(uint32_t StartTime, uint32_t Now){
	if(StartTime > Now){
		uint32_t u32_carry = CLK_TIM_MAX_VALUE - StartTime;
		return (u32_carry + Now) + 1;
	}else{
		return Now - StartTime;
	}
}

/************************************
 * @fn 	  	 : CLK_incTick
 * @brief 	 : This function is increased time.
 *
 * @param[0] : This parameter holds hclk pointer.
 * @parma[1] : This parameter holds tim pointer in callback function that interrupted timer.
 * @return	 : None
 *
 * @Note     : None
 ************************************/
// IT Handler
void CLK_incTick(CLK_Handler_t * hclk, TIM_HandleTypeDef * htim){
	if (htim == hclk->Config.htim){
		hclk->Time += hclk->Config.Freq;
	}
}

