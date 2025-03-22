#include "fsbb_ctrl.h"
#include "incremental_pid.h"
#include "bsp_hrtim.h"
#include "analog_signal.h"

#include "hrtim.h"
#include "stm32g474xx.h"
#include "stm32g4xx_hal.h"
#include "tim.h"

#define FSBB_GENERAL_TO_NARROW_RATIO 0.9f // 广义占空比到狭义占空比的比例

incremental_pid_t pid_voltage_h;
incremental_pid_t pid_voltage_l;
incremental_pid_t pid_current;
incremental_pid_t pid_power;

float test_P = 0.05f;
float test_I = 0.005f;
float test_D = 0.0f;

//
void FSBB_PID_init()
{
    // 初始化pid
    incremental_pid_init(&pid_voltage_h, 0.5f, 0.05f, 0, 15.0f, -15.0f);
    incremental_pid_init(&pid_voltage_l, 0.5f, 0.05f, 0, 15.0f, -15.0f);
    incremental_pid_init(&pid_power, 0.04f, 0.008f, 0, 15.0f, -15.0f);
    incremental_pid_init(&pid_current, 0.005f, 0.0005f, 0, 2.0f, -2.0f);

    pid_voltage_h.setValue = 26.0f;
    pid_voltage_l.setValue = 8.0f;
    pid_power.setValue     = 30.0f;
    // pid_current.setValue   = 1.0f;
}
//
void FSBB_CTRL_INIT(void)
{
    // HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_MASTER);
    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_E);
    HAL_Delay(2);
    FSBB_output_start();
    FSBB_PID_init();
    HAL_Delay(2);

    //
    // HAL_HRTIM_WaveformCountStart_IT(&hhrtim1, HRTIM_TIMERID_MASTER);
    //
    // HAL_TIM_Base_Start_IT(&htim6);
}

void FSBB_pwm_set_factor(float scaling_factor)
{
    const float factor_range_min = 0.2f; // 允许的倍数的最小值
    const float factor_range_max = 1.8f; // 允许的倍数的最大值

    // 把倍数限制在范围内
    scaling_factor = (scaling_factor < factor_range_min)   ? factor_range_min
                     : (scaling_factor > factor_range_max) ? factor_range_max
                                                           : scaling_factor;

    if (scaling_factor <= 1.0f) {
        FSBB_pwm_set(scaling_factor, 'A');
        FSBB_pwm_set(1.0f, 'E');
    } else {
        FSBB_pwm_set(1.0f, 'A');
        FSBB_pwm_set(1.0f / scaling_factor, 'E');
    }
}

void FSBB_pwm_set(float general_duty, char channel)
{
    const float general_duty_min = 0.2f; // 广义占空比最低值
    const float general_duty_max = 1.0f; // 广义占空比占空比最高值

    // 使用条件运算符限制数值
    general_duty = (general_duty < general_duty_min)   ? general_duty_min
                   : (general_duty > general_duty_max) ? general_duty_max
                                                       : general_duty;

    float duty_cycle = general_duty * FSBB_GENERAL_TO_NARROW_RATIO;

    HRTIM_PWM_duty_set(duty_cycle, channel);
    HRTIM_PWM_duty_set(duty_cycle, channel);
}

// 开启 FSBB PWM 输出
void FSBB_output_start(void)
{
    float input_voltage = get_pha_input_voltage();
    // float output_voltage = get_fsbb_output_voltage();
    FSBB_pwm_set_factor(12 / input_voltage);
    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2);
}

// 关闭 FSBB PWM 输出
void fsbb_pwm_output_stop(void)
{
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2);
}

//
void FSBB_output_set(float output_voltage, float current_limit, float max_power)
{
    pid_voltage_h.setValue = 26.0f;
    pid_voltage_l.setValue = 8.0f;
    pid_power.setValue     = 15.0f;
}


//
//
// test values
float general_duty = 0;
// adc线性映射
float Vin  = 0;
float Iin  = 0;
float Vout = 0;
float Iout = 0;

//
void HAL_HRTIM_RepetitionEventCallback(HRTIM_HandleTypeDef *hhrtim, uint32_t TimerIdx)
{

    // // adc线性映射
    // float Vin  = get_fsbb_input_voltage();
    // float Iin  = get_fsbb_input_current();
    // float Vout = get_fsbb_output_voltage();
    // float Iout = get_fsbb_output_current();

    // // error检查

    // // pid环路计算
    // const float target_power = 45.0f;
    // //
    // pid_power.setValue             = target_power;
    // float pid_cap_voltage_h_output = incremental_pid_compute(&pid_voltage_h, Vout);
    // float pid_cap_voltage_l_output = incremental_pid_compute(&pid_voltage_l, Vout);
    // float pid_power_output         = incremental_pid_compute(&pid_power, Vin * Iin);

    // float current_ref = pid_power_output;
    // if (current_ref > pid_cap_voltage_h_output) {
    //     pid_power.output = pid_cap_voltage_h_output;
    //     current_ref      = pid_cap_voltage_h_output;
    // } else if (current_ref < pid_cap_voltage_l_output) {
    //     pid_power.output = pid_cap_voltage_l_output;
    //     current_ref      = pid_cap_voltage_l_output;
    // } else {
    //     pid_voltage_h.output = current_ref;
    //     pid_voltage_l.output = current_ref;
    // }

    // pid_current.setValue = current_ref;
    // general_duty         = incremental_pid_compute(&pid_current, Iout);

    // // pwm输出
    // // FSBB_pwm_set_factor(general_duty);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {

        // adc线性映射
        // Vin  = get_fsbb_input_voltage();
        // Iin  = get_fsbb_input_current();
        Vout = get_output_voltage();
        // Iout = get_fsbb_output_current();

        // error检查

        // // pid环路计算
        // const float target_power = 45.0f;
        // //
        // pid_power.setValue             = target_power;
        // float pid_cap_voltage_h_output = incremental_pid_compute(&pid_voltage_h, Vout);
        // float pid_cap_voltage_l_output = incremental_pid_compute(&pid_voltage_l, Vout);
        // float pid_power_output         = incremental_pid_compute(&pid_power, Vin * Iin);

        // float current_ref = pid_power_output;
        // if (current_ref > pid_cap_voltage_h_output) {
        //     pid_power.output = pid_cap_voltage_h_output;
        //     current_ref      = pid_cap_voltage_h_output;
        // } else if (current_ref < pid_cap_voltage_l_output) {
        //     pid_power.output = pid_cap_voltage_l_output;
        //     current_ref      = pid_cap_voltage_l_output;
        // } else {
        //     pid_voltage_h.output = current_ref;
        //     pid_voltage_l.output = current_ref;
        // }

        // pid_current.setValue = current_ref;
        // general_duty         = incremental_pid_compute(&pid_current, Iout);

        general_duty = incremental_pid_compute(&pid_voltage_h, Vout);
        // pwm输出
        FSBB_pwm_set_factor(general_duty);
    }
}
