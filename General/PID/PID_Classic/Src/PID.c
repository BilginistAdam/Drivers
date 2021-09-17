/**
 * @file PID.c
 * @author İsmail Enes Bilgin (bilginenesismail@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2021-05-16
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "PID.h"

/* Init */
/**
 * @brief Initialize the PID Controller.
 * 
 * @param hpid
 * @return PID_Status_t 
 */
PID_Status_t PID_Init(PID_Handler_t * hpid){
    // Clear PID Controller Prev datas
    if (PID_OK != PID_resetPrevDatas(hpid)){
        return PID_ERROR;
    }

    // Clear PID Controller colculate Flag
    if (PID_OK != PID_setCalculateFlag(hpid, PID_RESET)){
        return PID_ERROR;
    }

    return PID_OK;
}

/* Reset */
/**
 * @brief Reset PID Controller Parameters
 * 
 * @param hpid 
 * @return PID_Status_t 
 */
PID_Status_t PID_resetParameter(PID_Handler_t * hpid){
    // Reset PID Kp
    if (PID_OK != PID_setKp(hpid, PID_KP_DEFAULT_VALUE)){
        return PID_ERROR;
    }

    // Reset PID Ki
    if (PID_OK != PID_setKi(hpid, PID_KI_DEFAULT_VALUE)){
        return PID_ERROR;
    }

    // Reset PID Kd
    if (PID_OK != PID_setKd(hpid, PID_KD_DEFAULT_VALUE)){
        return PID_ERROR;
    }

    #if PID_ERASE_PREV_DATA_UPDATED
    // Clear prev data in PID Controller
    if (PID_OK != PID_resetPrevDatas(hpid)){
        return PID_ERROR;
    }
    #endif /* PID_ERASE_PREV_DATA_UPDATED */

    return PID_OK;
}

/**
 * @brief Reset Prev data. (Integral, Derivate)
 * 
 * @param hpid 
 * @return PID_Status_t 
 */
PID_Status_t PID_resetPrevDatas(PID_Handler_t * hpid){
    // Reset Prev data. (Integral, Derivate)
    hpid->Integral = 0;
    hpid->Derivate = 0;
    hpid->SetPoint = 0;
    hpid->Output = 0;
    hpid->prevOutput = 0;

    return PID_ERROR;
}

/* Update */
/**
 * @brief Update the Config parameter of the PID Controller. (Kp, Ki, Kd)
 * 
 * @param hpid 
 * @param newConfig 
 * @return PID_Status_t 
 */
PID_Status_t PID_updateParameter(PID_Handler_t * hpid, PID_Config_t newConfig){
    // Update Kp
    if (PID_OK != PID_setKp(hpid, newConfig.Kp)){
        return PID_ERROR;
    }

    // Update Ki
    if (PID_OK != PID_setKi(hpid, newConfig.Ki)){
        return PID_ERROR;
    }

    // Update Kd
    if (PID_OK != PID_setKd(hpid, newConfig.Kd)){
        return PID_ERROR;
    }

    // Clear Prev Data
    #if PID_ERASE_PREV_DATA_UPDATED
    if (PID_OK != PID_resetPrevDatas(hpid)){
        return PID_ERROR;
    }
    #endif /* PID_ERASE_PREV_DATA_UPDATED */

    return PID_OK;
}

/* Gets */
/**
 * @brief Get the Kp of the PID Controller.
 * 
 * @param hpid 
 * @return float 
 */
float PID_getKp(PID_Handler_t * hpid){
    // return Kp
    return hpid->Config.Kp;
}

/**
 * @brief Get the Ki of the PID Controller.
 * 
 * @param hpid 
 * @return float 
 */
float PID_getKi(PID_Handler_t * hpid){
    //return Ki
    return hpid->Config.Ki;
}

/**
 * @brief Get the Kd of the PID Controller.
 * 
 * @param hpid 
 * @return float 
 */
float PID_getKd(PID_Handler_t * hpid){
    // return Kd
    return hpid->Config.Kd;
}

/**
 * @brief Get the Set Point of the PID Controller.
 * 
 * @param hpid 
 * @return float 
 */
float PID_getSetPoint(PID_Handler_t * hpid){
    // return Set Point 
    return hpid->SetPoint;
}

/**
 * @brief Get the output of the PID Controller.
 * 
 * @param hpid 
 * @return float 
 */
float PID_getOutput(PID_Handler_t * hpid){
    // return output
    return hpid->Output;
}

/**
 * @brief get the calculate flag of the PID Controller.
 * 
 * @param hpid 
 * @return PID_FlagStatus_t 
 */
PID_FlagStatus_t PID_getCalculateFlag(PID_Handler_t * hpid){
    // return calculate Flag
    return hpid->calculateFlag;
}

/* Sets */
/**
 * @brief Set the Kp of the PID Controller.
 * 
 * @param hpid 
 * @param newKp 
 * @return PID_Status_t 
 */
static PID_Status_t PID_setKp(PID_Handler_t * hpid, float newKp){
    // set the kp
    hpid->Config.Kp = newKp;

    return PID_OK;
}

/**
 * @brief Set the Ki of the PID Controller.
 * 
 * @param hpid 
 * @param newKi 
 * @return PID_Status_t 
 */
static PID_Status_t PID_setKi(PID_Handler_t * hpid, float newKi){
    // set the Ki
    hpid->Config.Ki = newKi;
    
    return PID_OK;
}

/**
 * @brief Set the Kd of the PID Controller.
 * 
 * @param hpid 
 * @param newKd 
 * @return PID_Status_t 
 */
static PID_Status_t PID_setKd(PID_Handler_t * hpid, float newKd){
    // set the Kd
    hpid->Config.Kd = newKd;

    return PID_OK;
}

/**
 * @brief Set the Set Point of the PID Controller.
 * 
 * @param hpid 
 * @param newSetPoint 
 * @return PID_Status_t 
 */
PID_Status_t PID_setSetPoint(PID_Handler_t * hpid, float newSetPoint){
    // set Set Point
    if (PID_MAX_OUTPUT > newSetPoint && newSetPoint < PID_MIN_OUTPUT){
        hpid->SetPoint = newSetPoint;
        return PID_OK;
    }

    return PID_ERROR;
}

/**
 * @brief Set the calculate flag of the PID Controller.
 * 
 * @param hpid 
 * @param newFlagStatus 
 * @return PID_Status_t 
 */
static PID_Status_t PID_setCalculateFlag(PID_Handler_t * hpid, PID_FlagStatus_t newFlagStatus){
    // set calculate flag
    if (PID_SET == newFlagStatus || PID_RESET == newFlagStatus){
        hpid->calculateFlag = newFlagStatus;
        return PID_OK;
    }
    return PID_ERROR;
}

/* Handlers */
/**
 * @brief Controller handler, calculate the output.
 * 
 * @param hpid 
 * @param input 
 */
PID_Status_t PID_ControllerHandler(PID_Handler_t * hpid, float input){
    // Check the calculate Flag
    if (PID_getCalculateFlag(hpid) == PID_SET){
        // Get the current position.
        float currentPosition = input;

        // Calculate the error;
        float error = hpid->SetPoint - currentPosition;

        // Calculate the integral.
        if (error == hpid->lastError){
            hpid->Integral += 0;   
        }else{
            hpid->Integral += error;
        }

        // Calculate the derivative.
        hpid->Derivate = error - hpid->prevOutput;

        // Calculate the Control Variable.
        hpid->Output = (hpid->Config.Kp * error) + (hpid->Config.Ki * hpid->Integral) + 
                       (hpid->Config.Kd * hpid->Derivate);

        // Limit the Control Variable to within +-.
        if (hpid->Output > PID_MAX_OUTPUT){
            hpid->Output = PID_MAX_OUTPUT;
        }else if (hpid->Output < PID_MIN_OUTPUT){
            hpid->Output = PID_MIN_OUTPUT;
        }

        // Save the current error as last error for next iteration.
        hpid->prevOutput = error;

        // Change the calculate flag
        PID_setCalculateFlag(hpid, PID_RESET);
    }
}

/**
 * @brief IT controller Handler, set the calculateFlag;
 * 
 * @param hpid 
 */
void PID_IT_ControllerHandler(PID_Handler_t * hpid){
    PID_setCalculateFlag(hpid, PID_SET);
}