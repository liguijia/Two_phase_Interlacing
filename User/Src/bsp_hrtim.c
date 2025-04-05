#include "stm32g4xx_hal.h"
#include "main.h"
#include "hrtim.h"
#include "stm32g4xx_hal_hrtim.h"
#include <stdint.h>

#include "bsp_hrtim.h"

// 定义系统时钟频率 (单位: Hz)
#define SYSTEM_CLOCK_FREQ 170000000U // 170 MHz
// 定义预分频系数
#define PRESCALER_RATIO (32) // mul32
// 定义最小和最大频率范围
#define MIN_FREQUENCY 100000U    // 最小频率 100 kHz
#define MAX_FREQUENCY 100000000U // 最大频率 10 MHz
// 定义完整周期对应的角度
#define FULL_CYCLE_DEGREES 360

// 初始化参数，使用全局变量
uint16_t PWM_FSBB_PERIOD_FULL = PERIOD_FSBB;     // FSBB周期长度全位置
uint16_t PWM_FSBB_PERIOD_HALF = PERIOD_FSBB / 2; // FSBB周期长度半位置
uint16_t PWM_FSBB_PERIOD_ZERO = 0;               // FSBB周期长度零位置


uint16_t MASTER_TIMER_PERIOD_MAX = PERIOD_MASTERTIMER; // Master Timer 最大周期

// 测试用HRITM PWM输出初始化（已在CUBEMX中配置）
void HRTIM_PWM_init(void)
{
    //
    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_MASTER);
    //
    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_A);
    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_B);
    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_C);
    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_D);
    //
    HAL_Delay(10);
    // 初始化设置

    // FSBB通道 A&E
    //  设置定时器 A 的 PWM 占空比为 0
    HRTIM_PWM_duty_set(0.50f, 'A');
    // 设置定时器 B 的 PWM 占空比为 0
    HRTIM_PWM_duty_set(0.50f, 'B');

    // LCC通道 D&C
    // 设置定时器 C 的 PWM 占空比为 0
    HRTIM_PWM_duty_set(0.50f, 'C');
    // 设置定时器 D 的 PWM 占空比为 0
    HRTIM_PWM_duty_set(0.50f, 'D');

    HRTIM_PWM_output_start();
}

// 开启全部 HRTIM PWM 输出
void HRTIM_PWM_output_start(void)
{
    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TC2 | HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2 | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);
}

// 关闭全部 HRTIM PWM 输出
void HRTIM_PWM_output_stop(void)
{
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TC2 | HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2 | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);
}

//
void HRTIM_PWM_duty_set(float dutyCycle, char channel)
{
    // 限制占空比在 0.0 到 1.0 之间
    if (dutyCycle < 0.0f) dutyCycle = 0.0f;
    if (dutyCycle > 1.0f) dutyCycle = 1.0f;

    uint32_t compare1 = 0U;
    uint32_t compare3 = 0U;

    // 根据通道选择对应的定时器 A 或 E
    switch (channel) {
        case 'A':
            compare1 = PWM_FSBB_PERIOD_ZERO + dutyCycle * PWM_FSBB_PERIOD_HALF;
            compare3 = PWM_FSBB_PERIOD_FULL - dutyCycle * PWM_FSBB_PERIOD_HALF;
            __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, compare1);
            __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_3, compare3);
            break;
        case 'B':
            compare1 = PWM_FSBB_PERIOD_ZERO + dutyCycle * PWM_FSBB_PERIOD_HALF;
            compare3 = PWM_FSBB_PERIOD_FULL - dutyCycle * PWM_FSBB_PERIOD_HALF;
            __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, compare1);
            __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_3, compare3);
            break;
        case 'C':
            compare1 = PWM_FSBB_PERIOD_ZERO + dutyCycle * PWM_FSBB_PERIOD_HALF;
            compare3 = PWM_FSBB_PERIOD_FULL - dutyCycle * PWM_FSBB_PERIOD_HALF;
            __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_COMPAREUNIT_1, compare1);
            __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_COMPAREUNIT_3, compare3);
            break;
        case 'D':
            compare1 = PWM_FSBB_PERIOD_ZERO + dutyCycle * PWM_FSBB_PERIOD_HALF;
            compare3 = PWM_FSBB_PERIOD_FULL - dutyCycle * PWM_FSBB_PERIOD_HALF;
            __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_COMPAREUNIT_1, compare1);
            __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_COMPAREUNIT_3, compare3);
            break;
        default:
            // 处理无效的通道
            Error_Handler();
            break;
    }
}