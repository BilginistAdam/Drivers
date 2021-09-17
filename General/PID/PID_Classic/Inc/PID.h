/**
 * @file PID.h
 * @author İsmail Enes Bilgin (bilginenesismail@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2021-05-16
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef __PID_H_
#define __PID_H_

#include "stdio.h"
#include "stdint.h"

#define _IO volatile

//@PID_ERASE
#define PID_ERASE_PREV_DATA_UPDATED 1

//@PID_DEFAULT_VALUES
#define PID_KP_DEFAULT_VALUE 0
#define PID_KI_DEFAULT_VALUE 0
#define PID_KD_DEFAULT_VALUE 0

//@PID_OUPUT_LIMITS
#define PID_MAX_OUTPUT   999
#define PID_MIN_OUTPUT  -999

typedef enum{
    PID_ERROR   =   0U,
    PID_OK      =   1U,
}PID_Status_t;

typedef enum{
    PID_RESET   =   0x00U,
    PID_SET     =   !PID_RESET,
}PID_FlagStatus_t;

typedef struct{
    float Kp;
    float Ki;
    float Kd;
}PID_Config_t;

typedef struct{
    PID_Config_t Config;

    PID_FlagStatus_t calculateFlag;

    float Integral;
    float Derivate;
    
    float lastError;
    float prevOutput;
    float SetPoint;

    float Output;
}PID_Handler_t;

/* Init */
PID_Status_t PID_Init(PID_Handler_t * hpid);

/* Reset */
PID_Status_t PID_resetParameter(PID_Handler_t * hpid);
PID_Status_t PID_resetPrevDatas(PID_Handler_t * hpid);

/* Update */
PID_Status_t PID_updateParameter(PID_Handler_t * hpid, PID_Config_t newConfig);

/* Gets */
float PID_getKp(PID_Handler_t * hpid);
float PID_getKi(PID_Handler_t * hpid);
float PID_getKd(PID_Handler_t * hpid);
float PID_getSetPoint(PID_Handler_t * hpid);
float PID_getOutput(PID_Handler_t * hpid);
PID_FlagStatus_t PID_getCalculateFlag(PID_Handler_t * hpid);

/* Sets */
static PID_Status_t PID_setKp(PID_Handler_t * hpid, float newKp);
static PID_Status_t PID_setKi(PID_Handler_t * hpid, float newKi);
static PID_Status_t PID_setKd(PID_Handler_t * hpid, float newKd);
static PID_Status_t PID_setSetPoint(PID_Handler_t * hpid, float newSetPoint);
static PID_Status_t PID_setCalculateFlag(PID_Handler_t * hpid, PID_FlagStatus_t newFlagStatus);

/* Handlers */
PID_Status_t PID_ControllerHandler(PID_Handler_t * hpid, float input);
void PID_IT_ControllerHandler(PID_Handler_t * hpid);

#endif /* __PID_H_ */