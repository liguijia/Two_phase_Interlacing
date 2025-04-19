#include "two_phase_fsbb_ctrl.h"
#include "incremental_pid.h"
#include "bsp_hrtim.h"
#include "analog_signal.h"

#include "hrtim.h"
#include "stm32g474xx.h"
#include "stm32g4xx_hal.h"
#include "tim.h"
#include <stdint.h>

#define FSBB_GENERAL_TO_NARROW_RATIO 0.9f // 广义占空比到狭义占空比的比例

//
incremental_pid_t pid_ref_current;
incremental_pid_t pid_delta_current;
//
incremental_pid_t pid_output_vlotage;
//
incremental_pid_t pid_power;

float test_P = 0.05f;
float test_I = 0.005f;
float test_D = 0.0f;
//
analogdata_t analogdata;
//
void FSBB_PID_init()
{
    // 初始化pid

    // current PID
    incremental_pid_init(&pid_ref_current, 0.0000001f, 0.006f, 0.000001, -2.0f, 2.0f);
    // phase delta current PID
    incremental_pid_init(&pid_delta_current, 0.00005f, 0.0005f, 0.0, 0.0f, +0.5f);
    // voltage PID
    incremental_pid_init(&pid_output_vlotage, 0.5f, 0.003f, 0.0f, -5.0f, +5.0f);
    // power PID
    incremental_pid_init(&pid_power, 0.0000001f, 0.005f, 0.0f, -27.0f, +27.0f);

    // 设置PID参数
    pid_power.setValue          = 45.0f;
    pid_output_vlotage.setValue = 19.0f;
    pid_ref_current.setValue    = 5.0f;
    pid_delta_current.setValue  = 0.0f;
}
//
void FSBB_CTRL_INIT(void)
{
    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_MASTER);
    // HRTIM_PWM_init();
    HAL_Delay(2);
    FSBB_PID_init();
    HAL_Delay(2);
    BSP_ADC_Convert_Start();
    HAL_Delay(2);
    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_A);
    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_B);
    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_C);
    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_D);

    //
    FSBB_output_start();

    // 开启中断计算PID
    HAL_HRTIM_WaveformCountStart_IT(&hhrtim1, HRTIM_TIMERID_MASTER);
    //
    HAL_TIM_Base_Start_IT(&htim6);
}

void FSBB_pwm_set_factor(float scaling_factor, char phase)
{
    const float factor_range_min = 0.2f; // 允许的倍数的最小值
    const float factor_range_max = 1.8f; // 允许的倍数的最大值

    // 把倍数限制在范围内
    scaling_factor = (scaling_factor < factor_range_min)   ? factor_range_min
                     : (scaling_factor > factor_range_max) ? factor_range_max
                                                           : scaling_factor;

    if (scaling_factor <= 1.0f) {
        if (phase == 'A') {
            FSBB_pwm_set(scaling_factor, 'B');
            FSBB_pwm_set(1.0f, 'A');
        } else if (phase == 'B') {
            FSBB_pwm_set(scaling_factor, 'D');
            FSBB_pwm_set(1.0f, 'C');
        } else {
            // 处理无效的通道
            Error_Handler();
        }

    } else {
        if (phase == 'A') {
            FSBB_pwm_set(1.0f, 'B');
            FSBB_pwm_set(1.0f / scaling_factor, 'A');
        } else if (phase == 'B') {
            FSBB_pwm_set(1.0f, 'D');
            FSBB_pwm_set(1.0f / scaling_factor, 'C');
        } else {
            // 处理无效的通道
            Error_Handler();
        }
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
    float input_voltage  = get_pha_input_voltage();
    float output_voltage = get_output_voltage();
    FSBB_pwm_set_factor(output_voltage / input_voltage, 'A');
    FSBB_pwm_set_factor(output_voltage / input_voltage, 'B');
    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 |
                                                HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TC2 | HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2);
}

// 关闭 FSBB PWM 输出
void fsbb_pwm_output_stop(void)
{
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2);
}

uint32_t hrtimrepe_test_value = 0;
//
void HAL_HRTIM_RepetitionEventCallback(HRTIM_HandleTypeDef *hhrtim, uint32_t TimerIdx)
{
    if (hhrtim->Instance == HRTIM1) {
        // 处理定时器的重复事件
        if (TimerIdx == HRTIM_TIMERID_MASTER) {
            // 处理主定时器的重复事件
            // hrtimrepe_test_value++; // 设置一个测试值
        }
    }
}
//
float target_power = 25.0f; // 目标功率
//
float ref_duty    = 0;
float delta_duty  = 0;
float ref_current = 0;
float ref_voltage = 0;
//

// 定时器中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {

        // 获取模拟数据
        get_all_analog_data(&analogdata);

        // PID控制器

        // power PID
        ref_voltage = incremental_pid_compute(&pid_power, analogdata.i_input * analogdata.v_pha_input, target_power);
        //  vlotage PID
        ref_current = incremental_pid_compute(&pid_output_vlotage, analogdata.v_output, ref_voltage);
        // current PID
        ref_duty = incremental_pid_compute(&pid_ref_current, analogdata.i_output, ref_current);

        // phase delta current PID
        delta_duty = incremental_pid_compute(&pid_delta_current, analogdata.i_pha_output - analogdata.i_phb_output, 0.0f);
        // pwm输出
        FSBB_pwm_set_factor(ref_duty + delta_duty, 'A');
        FSBB_pwm_set_factor(ref_duty - delta_duty, 'B');
    }
}
