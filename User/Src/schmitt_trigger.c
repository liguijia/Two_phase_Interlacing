#include "schmitt_trigger.h"
#include <stdint.h>

// 初始化施密特触发器（float）
void schmitt_float_init(SchmittTrigger *st, float upper, float lower)
{
    st->upper_threshold = upper;
    st->lower_threshold = lower;
    st->output_state    = 0.0f; // 默认初始状态为0
}
// 初始化施密特触发器（uint16_t）
void schmitt_int_init(SchmittTriggerInt *st, int16_t upper, int16_t lower)
{
    st->upper_threshold = upper;
    st->lower_threshold = lower;
    st->output_state    = 0;
}

// 施密特触发器处理函数

float schmitt_float_process(SchmittTrigger *st, float input)
{
    if (input >= st->upper_threshold) {
        st->output_state = 1.0f; // 输入超过上限，输出高电平
    } else if (input <= st->lower_threshold) {
        st->output_state = 0.0f; // 输入低于下限，输出低电平
    }
    // 在滞后区间内保持原状态
    return st->output_state;
}

uint8_t schmitt_int_process(SchmittTriggerInt *st, int16_t input)
{
    if (input >= st->upper_threshold) {
        st->output_state = 1;
    } else if (input <= st->lower_threshold) {
        st->output_state = 0;
    }
    return st->output_state;
}
