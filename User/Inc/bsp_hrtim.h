#pragma once
#ifndef __BSP_HRTIM_H__
    #define __BSP_HRTIM_H__
    #ifdef __cplusplus
extern "C"
{
    #endif
#include "hrtim.h"
    extern void HRTIM_PWM_init(void);
    extern void HRTIM_SetTimerAOutput1Polarity(HRTIM_HandleTypeDef *hhrtim, uint32_t polarity);
    extern void HRTIM_PWM_output_start(void);
    extern void HRTIM_PWM_output_stop(void);
    extern void HRTIM_PWM_duty_set(float dutyCycle, char channel);
    extern void HRTIM_PWM_frq_set(uint32_t frequency, char channel);
    extern void HRTIM_PWM_phase_set(HRTIM_HandleTypeDef *hrtim, char target,uint32_t phaseAngle);
    extern void HRTIM_PWM_Configure(HRTIM_HandleTypeDef *hrtim, 
        char channel, 
        uint32_t frequency, 
        float dutyCycle, 
        int32_t phaseAngle); 

    #ifdef __cplusplus
}
    #endif
#endif // !__BSP_HRTIM_H__