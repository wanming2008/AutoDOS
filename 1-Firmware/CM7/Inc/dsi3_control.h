/**
  ******************************************************************************
  * @file    dsi3_control.h
  * @brief   DSI3 Bus Control Module Header File
  *          Controls DSI3 conversion chips via SPI to implement ultrasonic radar DSI3 bus communication
  ******************************************************************************
  */

#ifndef __DSI3_CONTROL_H
#define __DSI3_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

/* DSI3配置定义 */
#define DSI3_MAX_SENSORS          4       // 最大支持4个DSI3传感器
#define DSI3_SPI_TIMEOUT          100     // SPI超时时间（ms）

/* DSI3命令定义 */
#define DSI3_CMD_RESET            0x01
#define DSI3_CMD_READ_DISTANCE     0x02
#define DSI3_CMD_READ_STATUS       0x03
#define DSI3_CMD_SET_MODE          0x04
#define DSI3_CMD_SPOOF_DISTANCE    0x05    // 欺骗距离命令
#define DSI3_CMD_JAMMING           0x06    // 干扰命令

/* DSI3模式定义 */
#define DSI3_MODE_NORMAL           0x00
#define DSI3_MODE_SNIFF            0x01    // 嗅探模式
#define DSI3_MODE_SPOOF            0x02    // 欺骗模式
#define DSI3_MODE_JAM              0x03    // 干扰模式

/* DSI3传感器数据结构 */
typedef struct {
    uint8_t sensor_id;             // 传感器ID (0-3)
    uint16_t distance;              // 距离值（厘米）
    uint8_t status;                 // 状态字节
    uint8_t enabled;                // 使能标志
} DSI3_Sensor_t;

/* 函数声明 */
void dsi3_control_init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);
void dsi3_set_mode(uint8_t sensor_id, uint8_t mode);
void dsi3_read_distance(uint8_t sensor_id, uint16_t *distance);
void dsi3_spoof_distance(uint8_t sensor_id, uint16_t fake_distance);
void dsi3_jamming_start(uint8_t sensor_id);
void dsi3_jamming_stop(uint8_t sensor_id);
void dsi3_sniff_start(uint8_t sensor_id);
uint8_t dsi3_sniff_get_data(uint8_t sensor_id, uint8_t *data, uint16_t *len);
void dsi3_process(void);

#ifdef __cplusplus
}
#endif

#endif /* __DSI3_CONTROL_H */

