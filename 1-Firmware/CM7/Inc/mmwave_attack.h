/**
  ******************************************************************************
  * @file    mmwave_attack.h
  * @brief   Millimeter-wave radar attack module header file
  *          Implements millimeter-wave radar attacks through CAN FD message modification
  ******************************************************************************
  */

#ifndef __MMWAVE_ATTACK_H
#define __MMWAVE_ATTACK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_fdcan.h"

/* 毫米波雷达CAN ID定义（需要根据实际车辆调整） */
#define MMWAVE_RADAR_ID_BASE       0x100U
#define MMWAVE_TARGET_DETECT_ID    0x101U
#define MMWAVE_DISTANCE_ID         0x102U

/* 函数声明 */
void mmwave_attack_init(void);
void mmwave_process_message(FDCAN_RxHeaderTypeDef *rx_header, uint64_t *rx_data);
void mmwave_set_attack_params(uint8_t enable, uint16_t fake_distance, uint8_t fake_target_count);

#ifdef __cplusplus
}
#endif

#endif /* __MMWAVE_ATTACK_H */

