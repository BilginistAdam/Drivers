/**
 * @file PID.h
 * @author İsmail Enes Bilgin (bilginenesismail.gmail.com)
 * @brief 
 * @version 0.1
 * @date 2021-05-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef __PID_H_
#define __PID_H_

#include "stdint.h"

typedef enum{
    PID_OK      = 0U,
    PID_ERROR   = 1U,
}PID_Status_t;

typedef enum{
    PID_RESET   = 0U,
    PID_SET     = !PID_RESET,
}PID_StatusFlag_t;

typedef struct 
{
    float A0;          /**< The derived gain, A0 = Kp + Ki + Kd . */
    float A1;          /**< The derived gain, A1 = -Kp - 2Kd. */
    float A2;          /**< The derived gain, A2 = Kd . */
    float state[3];    /**< The state array of length 3. */
    float Kp;          /**< The proportional gain. */
    float Ki;          /**< The integral gain. */
    float Kd;          /**< The derivative gain. */
}PID_Handler_t;

/* Init Functions */
PID_Status_t PID_Init(PID_Handler_t * hpid, PID_StatusFlag_t resetStateFlag);
void PID_Reset(PID_Handler_t * hpid);
float PID_Handler(PID_Handler_t * hpid, float in);




#endif /* __PID_H_ */