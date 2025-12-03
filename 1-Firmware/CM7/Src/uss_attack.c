/**
  ******************************************************************************
  * @file    uss_attack.c
  * @brief   Ultrasonic radar attack module implementation
  *          Implements two attack modes: interference and spoofing
  ******************************************************************************
  */

#include "uss_attack.h"
#include <string.h>

/* 私有变量 */
static TIM_HandleTypeDef *htim_uss = NULL;
static ADC_HandleTypeDef *hadc_uss = NULL;

static uint8_t uss_mode = 0;  // 0=关闭, 1=干扰, 2=欺骗
static uint16_t spoof_delay_cm = 30;  // 欺骗延迟距离（厘米）

/* 欺骗模式状态机 */
typedef enum {
    USS_STATE_IDLE,
    USS_STATE_IN_PULSE
} uss_state_t;

static uss_state_t uss_state = USS_STATE_IDLE;
static uint32_t last_pulse_time = 0;
static uint32_t pulse_period = 0;
static int samples_out_of_range = 0;
static int samples_in_range = 0;

static uint8_t waiting_to_emit = 0;
static uint32_t emit_start_time = 0;
static uint8_t emitting = 0;
static uint32_t pulse_start_time = 0;

/**
  * @brief  初始化超声波攻击模块
  * @param  htim: 定时器句柄（用于PWM输出）
  * @param  hadc: ADC句柄（用于回波检测）
  * @retval None
  */
void uss_attack_init(TIM_HandleTypeDef *htim, ADC_HandleTypeDef *hadc)
{
    htim_uss = htim;
    hadc_uss = hadc;
    
    // 启动ADC连续转换
    if(hadc_uss != NULL) {
        HAL_ADC_Start(hadc_uss);
    }
    
    uss_mode = 0;
    uss_state = USS_STATE_IDLE;
}

/**
  * @brief  启动干扰模式（持续输出48kHz）
  * @retval None
  */
void uss_jamming_start(void)
{
    if(htim_uss == NULL) return;
    
    uss_mode = 1;
    
    // 启动PWM输出（互补输出）
    HAL_TIM_PWM_Start(htim_uss, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(htim_uss, TIM_CHANNEL_2);
}

/**
  * @brief  启动欺骗模式（检测回波并延迟发射）
  * @retval None
  */
void uss_spoof_start(void)
{
    if(htim_uss == NULL || hadc_uss == NULL) return;
    
    uss_mode = 2;
    uss_state = USS_STATE_IDLE;
    last_pulse_time = 0;
    samples_out_of_range = 0;
    samples_in_range = 0;
    waiting_to_emit = 0;
    emitting = 0;
}

/**
  * @brief  停止超声波攻击
  * @retval None
  */
void uss_attack_stop(void)
{
    uss_mode = 0;
    
    if(htim_uss != NULL) {
        HAL_TIM_PWM_Stop(htim_uss, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(htim_uss, TIM_CHANNEL_2);
    }
    
    uss_state = USS_STATE_IDLE;
    emitting = 0;
    waiting_to_emit = 0;
}

/**
  * @brief  设置欺骗延迟距离
  * @param  delay_cm: 延迟距离（厘米）
  * @retval None
  */
void uss_set_spoof_delay(uint16_t delay_cm)
{
    spoof_delay_cm = delay_cm;
}

/**
  * @brief  处理超声波攻击（主循环中调用）
  * @retval None
  */
void uss_attack_process(void)
{
    if(uss_mode == 0) return;
    
    if(uss_mode == 1) {
        // 干扰模式：持续输出，无需处理
        return;
    }
    
    if(uss_mode == 2 && hadc_uss != NULL) {
        // 欺骗模式：检测回波并延迟发射
        // 注意：需要定期读取ADC值，这里假设在主循环中已经启动连续转换
        uint32_t adc_value = 0;
        if(HAL_ADC_PollForConversion(hadc_uss, 1) == HAL_OK) {
            adc_value = HAL_ADC_GetValue(hadc_uss);
        }
        
        switch(uss_state) {
            case USS_STATE_IDLE:
                // 电压越界检测
                if(adc_value < ADC_LOW_THRESHOLD || adc_value > ADC_HIGH_THRESHOLD) {
                    samples_out_of_range++;
                    if(samples_out_of_range >= SAMPLES_TO_TRIGGER) {
                        // 满足触发条件，记录周期
                        uint32_t current_time = HAL_GetTick();
                        if(last_pulse_time != 0) {
                            pulse_period = current_time - last_pulse_time;
                        }
                        last_pulse_time = current_time;
                        samples_out_of_range = 0;
                        uss_state = USS_STATE_IN_PULSE;
                    }
                } else {
                    samples_out_of_range = 0;
                }
                break;
                
            case USS_STATE_IN_PULSE:
                // 判断是否回到正常范围
                if(adc_value >= ADC_LOW_THRESHOLD && adc_value <= ADC_HIGH_THRESHOLD) {
                    samples_in_range++;
                    if(samples_in_range >= SAMPLES_TO_END) {
                        uss_state = USS_STATE_IDLE;
                        samples_in_range = 0;
                        waiting_to_emit = 1;
                        emit_start_time = HAL_GetTick();
                    }
                } else {
                    samples_in_range = 0;
                }
                break;
        }
        
        // 计算延迟时间（毫秒）
        // 延迟 = 2 * delay_cm / 34.3 (声速340m/s = 34.3cm/ms)
        uint32_t delay_ms = (uint32_t)(spoof_delay_cm * 2.0f / 34.3f);
        
        // 延迟时间到达，开始发射
        uint32_t current_time = HAL_GetTick();
        if(waiting_to_emit && (current_time - emit_start_time >= delay_ms)) {
            if(htim_uss != NULL) {
                HAL_TIM_PWM_Start(htim_uss, TIM_CHANNEL_1);
                HAL_TIM_PWM_Start(htim_uss, TIM_CHANNEL_2);
            }
            emitting = 1;
            pulse_start_time = current_time;
            waiting_to_emit = 0;
        }
        
        // 计算脉冲持续时间（毫秒）
        // 脉冲长度 = 60个周期 / 48000Hz = 1.25ms
        uint32_t pulse_duration_ms = (uint32_t)(1000.0f / USS_FREQUENCY_HZ * USS_PULSE_CYCLES);
        
        // 超过脉冲长度，停止PWM
        if(emitting && (current_time - pulse_start_time >= pulse_duration_ms)) {
            if(htim_uss != NULL) {
                HAL_TIM_PWM_Stop(htim_uss, TIM_CHANNEL_1);
                HAL_TIM_PWM_Stop(htim_uss, TIM_CHANNEL_2);
            }
            emitting = 0;
        }
    }
}

