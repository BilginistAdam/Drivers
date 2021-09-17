/**
 * @file PID.c
 * @author İsmail Enes Bilgin (bilginenesismail.gmail.com)
 * @brief 
 * @version 0.1
 * @date 2021-05-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "PID.h"

/**
 * @brief  Initialization function for the floating-point PID Control.
 * @param[in,out] hpid               points to an instance of the PID structure.
 * @param[in]     resetStateFlag  flag to reset the state. 0 = no change in state 1 = reset the state.
 */
PID_Status_t PID_Init(PID_Handler_t * hpid, PID_StatusFlag_t resetStateFlag){
  /* Derived coefficient A0 */
  hpid->A0 = hpid->Kp + hpid->Ki + hpid->Kd;

  /* Derived coefficient A1 */
  hpid->A1 = (-hpid->Kp) - ((float) 2.0 * hpid->Kd);

  /* Derived coefficient A2 */
  hpid->A2 = hpid->Kd;

  /* Check whether state needs reset or not */
  if(resetStateFlag)
  {
    /* Clear the state buffer.  The size will be always 3 samples */
    memset(hpid->state, 0, 3u * sizeof(float));
  }

    return PID_OK;
}

/**    
  * @brief  Reset function for the floating-point PID Control.   
  * @param[in] *S	Instance pointer of PID control data structure.   
  * @return none.    
  * \par Description:   
  * The function resets the state buffer to zeros.    
  */
void PID_Reset(PID_Handler_t * hpid){
    /* Clear the state buffer.  The size will be always 3 samples */
    memset(hpid->state, 0, 3u * sizeof(float));
}

/**
 * @brief  Process function for the floating-point PID Control.
 * @param[in,out]  hpid   is an instance of the floating-point PID Control structure
 * @param[in]     in  input sample to process
 * @return out processed output sample.
 */
float PID_Handler(PID_Handler_t * hpid, float in){
  float out;

  /* y[n] = y[n-1] + A0 * x[n] + A1 * x[n-1] + A2 * x[n-2]  */
  out = (hpid->A0 * in) +
    (hpid->A1 * hpid->state[0]) + (hpid->A2 * hpid->state[1]) + (hpid->state[2]);
  
  /* Update States */
  hpid->state[1] = hpid->state[0];
  hpid->state[0] = in;
  hpid->state[2] = out;

  /* Return to application. */
  return (out);
}