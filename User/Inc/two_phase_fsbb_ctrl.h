#pragma once
#include <stdint.h>
#ifndef __TWO_PHASE_FSBB_CTRL_H__
#define __TWO_PHASE_FSBB_CTRL_H__
#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FSBB_PHASE_A   = 0,
    FSBB_PHASE_B   = 1,
    FSBB_PHASE_ALL = 2,
} fsbb_phase_t;

typedef enum {
    NORMAL_MODE        = 0, // 常规双相交错BUCKBOOST
    LOWPOWER_MODE      = 1, // 关闭B相输出，仅保留A相，降低静态功耗
    WIRELESSINPUT_MODE = 2, // 关闭A相输出和背靠背MOS管，防止无线充电的能量返回电池

    ERROR_MODE = 3 // 异常错误
} fsbb_mode_t;

extern void FSBB_CTRL_INIT(void);
extern void FSBB_output_start(fsbb_phase_t phase);
extern void FSBB_pwm_set_factor(float scaling_factor, fsbb_phase_t phase);
extern void FSBB_pwm_set(float general_duty, uint32_t timerIndex);

#ifdef __cplusplus
}
#endif
#endif // !__TWO_PHASE_FSBB_CTRL_H__
