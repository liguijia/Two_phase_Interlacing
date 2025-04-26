#pragma once
#ifndef __SCHMITT_TRIGGER_H__
#define __SCHMITT_TRIGGER_H__
#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "stm32g474xx.h"

typedef struct {
    float upper_threshold; // 上阈值（Vhigh）
    float lower_threshold; // 下阈值（Vlow）
    float output_state;    // 当前输出状态
} SchmittTrigger;

typedef struct {
    uint16_t upper_threshold;
    uint16_t lower_threshold;
    uint8_t output_state; // 0或1
} SchmittTriggerInt;

void schmitt_float_init(SchmittTrigger *st, float upper, float lower);
float schmitt_float_process(SchmittTrigger *st, float input);

#ifdef __cplusplus
}
#endif
#endif // !__SCHMITT_TRIGGER_H__