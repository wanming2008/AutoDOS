/**
  ******************************************************************************
  * @file    uss_attack.h
  * @brief   Ultrasonic radar attack module header file
  *          Supports two attack modes: interference and spoofing
  ******************************************************************************
  */

#ifndef __USS_ATTACK_H
#define __USS_ATTACK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

/* 超声波频率定义 */
#define USS_FREQUENCY_HZ       48000U
#define USS_PULSE_CYCLES       60U    // 脉冲周期数

/* ADC阈值定义 */
#define ADC_LOW_THRESHOLD      450    // 约2.2V (12位ADC, 3.3V参考)
#define ADC_HIGH_THRESHOLD     570    // 约2.8V
#define SAMPLES_TO_TRIGGER     2      // 连续越界采样次数
#define SAMPLES_TO_END         2      // 连续恢复正常采样次数

/* 函数声明 */
void uss_attack_init(TIM_HandleTypeDef *htim, ADC_HandleTypeDef *hadc);
void uss_jamming_start(void);
void uss_spoof_start(void);
void uss_attack_stop(void);
void uss_attack_process(void);
void uss_set_spoof_delay(uint16_t delay_cm);

#ifdef __cplusplus
}
#endif

#endif /* __USS_ATTACK_H */

