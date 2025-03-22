#pragma once
#ifndef __FSBB_CTRL_H__
    #define __FSBB_CTRL_H__
    #ifdef __cplusplus
extern "C"
{
    #endif

    extern void FSBB_CTRL_INIT(void);
    extern void FSBB_output_start(void);
    extern void FSBB_pwm_set_factor(float scaling_factor);
    extern void FSBB_pwm_set(float general_duty,char channel);

    #ifdef __cplusplus
}
    #endif
#endif // !__FSBB_CTRL_H__
