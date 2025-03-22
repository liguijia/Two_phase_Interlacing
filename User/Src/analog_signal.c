#include "analog_signal.h"
#include "mean_filter.h"

#include "adc.h"
#include "stm32g474xx.h"

#include <stdint.h>

//
#define ADC1_DATA_LEN      (4U)
#define ADC2_DATA_LEN      (3U)

#define FILTER_WINDOW_SIZE 8 // 定义滤波窗口大小

uint16_t adc1_data[ADC1_DATA_LEN * 2] = {0};
uint16_t adc2_data[ADC2_DATA_LEN * 2] = {0};

// 定义数据类型和结构体

// ADC校准参数结构体
typedef struct
{
    float k;
    float b;
} adc_calibration_t;

typedef struct
{
    uint32_t stm32id[3]; // 96 bit stm32 id
    adc_calibration_t v_pha_input;
    adc_calibration_t v_phb_input;
    adc_calibration_t v_output;
    adc_calibration_t i_pha_input;
    adc_calibration_t i_phb_input;
    adc_calibration_t i_pha_output;
    adc_calibration_t i_phb_output;
} board_adc_calibration_t;

static board_adc_calibration_t adc_cali_array[] = {
    // board_adc_calibration insert start
    {
        {0x00000000, 0x00000000, 0x00000222}, // STM32 唯一 ID

        // voltage ：R1=49.9k  R2=3.3k
        {0.0008117675f, -0.0000000000f}, // pha_input_voltage 校准参数 (k, b)
        {0.0008117675f, -0.0000000000f}, // phb_input_voltage 校准参数 (k, b)
        {0.0008117675f, -0.0000000000f}, // output_voltage 校准参数 (k, b)
        // INA240A2 0.002R VREF=3.3V or INA240A1 0.005R VREF=3.3V
        {0.0004990299f, -16.300082146f}, // pha_input_current 校准参数 (k, b)
        {0.0004990299f, -16.300082146f}, // phb_input_current 校准参数 (k, b)
        {0.0004990299f, -16.300082146f}, // pha_output_current 校准参数 (k, b)
        {0.0004990299f, -16.300082146f}, // phb_output_current 校准参数 (k, b)
    },
    {
        {0x00000000, 0x00000000, 0x00000111}, // STM32 唯一 ID
        {0.0000000000f, 0.0000000000f},
        {0.0000000000f, 0.0000000000f},
        {0.0000000000f, 0.0000000000f},
        {0.0000000000f, 0.0000000000f},
        {0.0000000000f, 0.0000000000f},
        {0.0000000000f, 0.0000000000f},
        {0.0000000000f, 0.0000000000f},
    }
    // board_adc_calibration insert stop
};

static mean_filter_t v_pha_input_filter  = {0};
static mean_filter_t v_phb_input_filter  = {0};
static mean_filter_t v_output_filter     = {0};
static mean_filter_t i_pha_input_filter  = {0};
static mean_filter_t i_phb_input_filter  = {0};
static mean_filter_t i_pha_output_filter = {0};
static mean_filter_t i_phb_output_filter = {0};

// 线性映射,用于将ADC采样值映射到实际值
static inline float linear_map(float value, adc_calibration_t calibration)
{
    return calibration.k * value + calibration.b;
}

// 整合后的通用映射函数（单函数完成滤波+线性映射）
static inline float get_mapped_value(mean_filter_t *filter, const adc_calibration_t *calib)
{
    // 1. 执行均值滤波 → 2. 应用线性校准公式
    return calib->k * mean_filter_calculate_average(filter) + calib->b;
}

// 获取模拟量

// 获取全部模拟量
void get_all_analog_data(analogdata_t *data)
{
    // 电压采集
    data->v_pha_input = get_mapped_value(&v_pha_input_filter,
                                         &adc_cali_array[0].v_pha_input);
    data->v_phb_input = get_mapped_value(&v_phb_input_filter,
                                         &adc_cali_array[0].v_phb_input);
    data->v_output    = get_mapped_value(&v_output_filter,
                                         &adc_cali_array[0].v_output);

    // 电流采集
    data->i_pha_input  = get_mapped_value(&i_pha_input_filter,
                                          &adc_cali_array[0].i_pha_input);
    data->i_phb_input  = get_mapped_value(&i_phb_input_filter,
                                          &adc_cali_array[0].i_phb_input);
    data->i_pha_output = get_mapped_value(&i_pha_output_filter,
                                          &adc_cali_array[0].i_pha_output);
    data->i_phb_output = get_mapped_value(&i_phb_output_filter,
                                          &adc_cali_array[0].i_phb_output);

    // 计算总输出电流
    data->output_current = data->i_pha_output + data->i_phb_output;
}

// 获取单独模拟量
float get_pha_input_voltage()
{
    uint16_t adc_value_average = mean_filter_calculate_average(&v_pha_input_filter);
    float mapped_value         = linear_map(adc_value_average, adc_cali_array[0].v_pha_input);
    return mapped_value;
}

float get_phb_input_voltage()
{
    uint16_t adc_value_average = mean_filter_calculate_average(&v_phb_input_filter);
    float mapped_value         = linear_map(adc_value_average, adc_cali_array[0].v_phb_input);
    return mapped_value;
}

float get_output_voltage()
{
    uint16_t adc_value_average = mean_filter_calculate_average(&v_output_filter);
    float mapped_value         = linear_map(adc_value_average, adc_cali_array[0].v_output);
    return mapped_value;
}

float get_pha_input_current()
{
    uint16_t adc_value_average = mean_filter_calculate_average(&i_pha_input_filter);
    float mapped_value         = linear_map(adc_value_average, adc_cali_array[0].i_pha_input);
    return mapped_value;
}
float get_phb_input_current()
{
    uint16_t adc_value_average = mean_filter_calculate_average(&i_phb_input_filter);
    float mapped_value         = linear_map(adc_value_average, adc_cali_array[0].i_phb_input);
    return mapped_value;
}
float get_pha_output_current()
{
    uint16_t adc_value_average = mean_filter_calculate_average(&i_pha_output_filter);
    float mapped_value         = linear_map(adc_value_average, adc_cali_array[0].i_pha_output);
    return mapped_value;
}
float get_phb_output_current()
{
    uint16_t adc_value_average = mean_filter_calculate_average(&i_phb_output_filter);
    float mapped_value         = linear_map(adc_value_average, adc_cali_array[0].i_phb_output);
    return mapped_value;
}
float get_output_current()
{
    return get_pha_output_current() + get_phb_output_current();
}

// ADC采样初始化
void BSP_ADC_Convert_Start(void)
{
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_Delay(5);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_Delay(5);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_data, ADC1_DATA_LEN * 2);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2_data, ADC2_DATA_LEN * 2);

    mean_filter_init(&v_pha_input_filter, FILTER_WINDOW_SIZE);
    mean_filter_init(&v_phb_input_filter, FILTER_WINDOW_SIZE);
    mean_filter_init(&v_output_filter, FILTER_WINDOW_SIZE);
    mean_filter_init(&i_pha_input_filter, FILTER_WINDOW_SIZE);
    mean_filter_init(&i_phb_input_filter, FILTER_WINDOW_SIZE);
    mean_filter_init(&i_pha_output_filter, FILTER_WINDOW_SIZE);
    mean_filter_init(&i_phb_output_filter, FILTER_WINDOW_SIZE);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        mean_filter_update(&v_pha_input_filter, adc1_data[0]);
        mean_filter_update(&i_pha_input_filter, adc1_data[1]);
        mean_filter_update(&v_phb_input_filter, adc1_data[2]);
        mean_filter_update(&i_phb_input_filter, adc1_data[3]);
    }
    if (hadc->Instance == ADC2) {
        mean_filter_update(&v_output_filter, adc2_data[0]);
        mean_filter_update(&i_pha_output_filter, adc2_data[1]);
        mean_filter_update(&i_phb_output_filter, adc2_data[2]);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        mean_filter_update(&v_pha_input_filter, adc1_data[0 + ADC1_DATA_LEN]);
        mean_filter_update(&i_pha_input_filter, adc1_data[1 + ADC1_DATA_LEN]);
        mean_filter_update(&v_phb_input_filter, adc1_data[2 + ADC1_DATA_LEN]);
        mean_filter_update(&i_phb_input_filter, adc1_data[3 + ADC1_DATA_LEN]);
    }
    if (hadc->Instance == ADC2) {
        mean_filter_update(&v_output_filter, adc2_data[0 + ADC2_DATA_LEN]);
        mean_filter_update(&i_pha_output_filter, adc2_data[1 + ADC2_DATA_LEN]);
        mean_filter_update(&i_phb_output_filter, adc2_data[2 + ADC2_DATA_LEN]);
    }
}