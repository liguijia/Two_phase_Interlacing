#pragma once
#ifndef __ANALOG_SIGNAL_H__
#define __ANALOG_SIGNAL_H__

// 模拟量数据结构体
typedef struct {
    float v_pha_input;    // Phase A 输入电压
    float v_phb_input;    // Phase B 输入电压
    float v_output; // 输出电压
    float i_pha_input;    // Phase A 输入电流
    float i_phb_input;    // Phase B 输入电流
    float i_pha_output;   // Phase A 输出电流
    float i_phb_output;   // Phase B 输出电流
    float output_current; // 总输出电流
} analogdata_t;

extern analogdata_t analogdata;

extern void BSP_ADC_Convert_Start(void);

extern float get_pha_input_voltage();
extern float get_phb_input_voltage();
extern float get_output_voltage();

extern float get_pha_input_current();
extern float get_phb_input_current();
extern float get_pha_output_current();
extern float get_phb_output_current();

extern void get_all_analog_data(analogdata_t *data);


#endif // !__ANALOG_SIGNAL_H__
