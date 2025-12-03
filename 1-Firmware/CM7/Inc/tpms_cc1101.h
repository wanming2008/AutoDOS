/**
  ******************************************************************************
  * @file    tpms_cc1101.h
  * @brief   Tire Pressure Monitoring System (TPMS) attack module header file
  *          Simulates 433MHz tire pressure signals using the TI CC1101 chip
  ******************************************************************************
  */

#ifndef __TPMS_CC1101_H
#define __TPMS_CC1101_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

/* CC1101配置定义 */
#define CC1101_SPI_TIMEOUT        100
#define TPMS_FREQUENCY_433MHZ     433920000UL  // 433.92MHz
#define TPMS_MAX_SENSORS         4            // 最大支持4个轮胎

/* TPMS传感器位置定义 */
#define TPMS_POSITION_FL          0  // 左前轮
#define TPMS_POSITION_FR          1  // 右前轮
#define TPMS_POSITION_RL          2  // 左后轮
#define TPMS_POSITION_RR          3  // 右后轮

/* CC1101寄存器地址 */
#define CC1101_REG_IOCFG2         0x00
#define CC1101_REG_IOCFG1         0x01
#define CC1101_REG_IOCFG0         0x02
#define CC1101_REG_FIFOTHR        0x03
#define CC1101_REG_SYNC1          0x04
#define CC1101_REG_SYNC0          0x05
#define CC1101_REG_PKTLEN         0x06
#define CC1101_REG_PKTCTRL1       0x07
#define CC1101_REG_PKTCTRL0       0x08
#define CC1101_REG_ADDR           0x09
#define CC1101_REG_CHANNR         0x0A
#define CC1101_REG_FSCTRL1        0x0B
#define CC1101_REG_FSCTRL0        0x0C
#define CC1101_REG_FREQ2          0x0D
#define CC1101_REG_FREQ1          0x0E
#define CC1101_REG_FREQ0          0x0F
#define CC1101_REG_MDMCFG4        0x10
#define CC1101_REG_MDMCFG3        0x11
#define CC1101_REG_MDMCFG2        0x12
#define CC1101_REG_MDMCFG1        0x13
#define CC1101_REG_MDMCFG0        0x14
#define CC1101_REG_DEVIATN        0x15
#define CC1101_REG_MCSM2          0x16
#define CC1101_REG_MCSM1          0x17
#define CC1101_REG_MCSM0          0x18
#define CC1101_REG_FOCCFG         0x19
#define CC1101_REG_BSCFG          0x1A
#define CC1101_REG_AGCTRL2        0x1B
#define CC1101_REG_AGCTRL1        0x1C
#define CC1101_REG_AGCTRL0        0x1D
#define CC1101_REG_WORCTRL        0x1E
#define CC1101_REG_FREND1         0x1F
#define CC1101_REG_FREND0         0x20
#define CC1101_REG_FSCAL3         0x21
#define CC1101_REG_FSCAL2         0x22
#define CC1101_REG_FSCAL1         0x23
#define CC1101_REG_FSCAL0         0x24
#define CC1101_REG_RCCTRL1        0x25
#define CC1101_REG_RCCTRL0        0x26
#define CC1101_REG_FSTEST         0x29
#define CC1101_REG_PTEST          0x2A
#define CC1101_REG_AGCTEST        0x2B
#define CC1101_REG_TEST2          0x2C
#define CC1101_REG_TEST1          0x2D
#define CC1101_REG_TEST0          0x2E
#define CC1101_REG_PATABLE        0x3E  // PATABLE寄存器地址
#define CC1101_REG_RXBYTES        0x3B  // RX字节数寄存器

/* CC1101命令 */
#define CC1101_CMD_SRES           0x30  // 复位
#define CC1101_CMD_SFSTXON        0x31  // 使能并校准频率合成器
#define CC1101_CMD_SXOFF          0x32  // 关闭晶体振荡器
#define CC1101_CMD_SCAL           0x33  // 校准频率合成器
#define CC1101_CMD_SRX            0x34  // 使能RX
#define CC1101_CMD_STX            0x35  // 使能TX
#define CC1101_CMD_SIDLE          0x36  // 空闲
#define CC1101_CMD_SWOR            0x38  // 唤醒
#define CC1101_CMD_SPWD            0x39  // 掉电
#define CC1101_CMD_SFRX           0x3A  // 刷新RX FIFO
#define CC1101_CMD_SFTX           0x3B  // 刷新TX FIFO
#define CC1101_CMD_SWORRST        0x3C  // 复位实时时钟
#define CC1101_CMD_SNOP           0x3D  // 空操作

/* CC1101状态寄存器 */
#define CC1101_STATUS_CHIP_RDYn_BM    0x80
#define CC1101_STATUS_STATE_BM        0x70

/* TPMS数据结构 */
typedef struct {
    uint8_t position;              // 轮胎位置
    uint32_t sensor_id;            // 传感器ID（通常是固定的，如BMW X3左前轮ID相同）
    uint16_t pressure;             // 胎压（kPa，0-1000）
    uint8_t temperature;            // 温度（摄氏度，-40到125）
    uint8_t battery;                // 电池状态（0-100%）
    uint8_t enabled;                // 使能标志
} TPMS_Sensor_t;

/* 函数声明 */
void tpms_cc1101_init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, 
                      GPIO_TypeDef *gdo0_port, uint16_t gdo0_pin);
void tpms_set_sensor_id(uint8_t position, uint32_t sensor_id);
void tpms_spoof_pressure(uint8_t position, uint16_t pressure, uint8_t temperature);
void tpms_spoof_low_pressure(uint8_t position);  // 模拟低胎压警告
void tpms_spoof_high_pressure(uint8_t position);  // 模拟高胎压警告
void tpms_spoof_burst(uint8_t position);          // 模拟爆胎
void tpms_sniff_start(void);                      // 启动嗅探模式
uint8_t tpms_sniff_get_packet(uint8_t *data, uint8_t *len);  // 获取嗅探到的数据包
void tpms_process(void);

#ifdef __cplusplus
}
#endif

#endif /* __TPMS_CC1101_H */

