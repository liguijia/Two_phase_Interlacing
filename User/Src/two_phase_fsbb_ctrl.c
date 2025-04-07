#include "two_phase_fsbb_ctrl.h"
#include "incremental_pid.h"
#include "bsp_hrtim.h"
#include "analog_signal.h"

#include "hrtim.h"
#include "stm32g474xx.h"
#include "stm32g4xx_hal.h"
#include "tim.h"

#define FSBB_GENERAL_TO_NARROW_RATIO 0.9f // 广义占空比到狭义占空比的比例

incremental_pid_t pid_all_current;
//
incremental_pid_t pid_pha_voltage_h;
incremental_pid_t pid_pha_voltage_l;
incremental_pid_t pid_pha_current;
incremental_pid_t pid_pha_power;

//
incremental_pid_t pid_phb_voltage_h;
incremental_pid_t pid_phb_voltage_l;
incremental_pid_t pid_phb_current;
incremental_pid_t pid_phb_power;

float test_P = 0.05f;
float test_I = 0.005f;
float test_D = 0.0f;
//
analogdata_t analogdata;
//
void FSBB_PID_init()
{
    // 初始化pid
    incremental_pid_init(&pid_all_current, 0.08f, 0.01f, 0, -2.0f, 2.0f);
    // Phase A PID
    incremental_pid_init(&pid_pha_current, 0.08f, 0.01f, 0, -2.0f, 2.0f);
    // Phase B PID
    incremental_pid_init(&pid_phb_current, 0.08f, 0.01f, 0, -2.0f, 2.0f);

    //
    // float set_current   = 2.0f;
    // float set_vlotage_h = 26.0f;
    pid_all_current.setValue = 5.0f;

    // pid_pha_power.setValue     = 30.0f;
    // pid_pha_voltage_h.setValue = set_vlotage_h;
    // pid_pha_voltage_l.setValue = 8.0f;
    pid_pha_current.setValue = 1.0f;

    //
    // pid_pha_power.setValue     = 30.0f;
    // pid_phb_voltage_h.setValue = set_vlotage_h;
    // pid_pha_voltage_l.setValue = 8.0f;
    pid_phb_current.setValue = 1.0f;
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
    //  HAL_HRTIM_WaveformCountStart_IT(&hhrtim1, HRTIM_TIMERID_MASTER);
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
    FSBB_pwm_set_factor(24 / input_voltage, 'A');
    FSBB_pwm_set_factor(24 / input_voltage, 'B');
    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 |
                                                HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TC2 | HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2);
}

// 关闭 FSBB PWM 输出
void fsbb_pwm_output_stop(void)
{
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2);
}

//
void HAL_HRTIM_RepetitionEventCallback(HRTIM_HandleTypeDef *hhrtim, uint32_t TimerIdx)
{
}

float pha_general_duty = 0;
float phb_general_duty = 0;
float all_general_duty = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        //
        // const float target_power = 45.0f;
        //
        get_all_analog_data(&analogdata);
        // float pid_pha_voltage_h_output = incremental_pid_compute(&pid_pha_voltage_h, analogdata.v_output);
        // float pid_phb_voltage_h_output = incremental_pid_compute(&pid_phb_voltage_h, analogdata.v_output);

        // pha_current_ref = pid_pha_voltage_h_output;
        // phb_current_ref = pid_phb_voltage_h_output;

        // pid_pha_current.setValue = pha_current_ref;
        // pid_phb_current.setValue = phb_current_ref;

        pha_general_duty = incremental_pid_compute(&pid_pha_current, analogdata.i_pha_output);
        phb_general_duty = incremental_pid_compute(&pid_phb_current, analogdata.i_phb_output);
        all_general_duty = incremental_pid_compute(&pid_all_current, analogdata.i_phb_output + analogdata.i_phb_output);

        // pwm输出
        FSBB_pwm_set_factor(all_general_duty, 'A');
        FSBB_pwm_set_factor(all_general_duty, 'B');
    }
}
