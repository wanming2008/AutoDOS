/**
  ******************************************************************************
  * @file    dsi3_control.c
  * @brief   DSI3 bus control module implementation
  *          Controls DSI3 conversion chips via SPI
  ******************************************************************************
  */

#include "dsi3_control.h"
#include <string.h>

/* 私有变量 */
static SPI_HandleTypeDef *hspi_dsi3 = NULL;
static GPIO_TypeDef *dsi3_cs_port = NULL;
static uint16_t dsi3_cs_pin = 0;
static DSI3_Sensor_t dsi3_sensors[DSI3_MAX_SENSORS];
static uint8_t dsi3_initialized = 0;

/* 私有函数 */
static void dsi3_cs_low(void);
static void dsi3_cs_high(void);
static uint8_t dsi3_spi_transfer(uint8_t data);
static void dsi3_send_command(uint8_t sensor_id, uint8_t cmd, uint8_t *data, uint8_t len);
static void dsi3_receive_response(uint8_t sensor_id, uint8_t *data, uint8_t len);

/**
  * @brief  CS片选拉低
  */
static void dsi3_cs_low(void)
{
    if(dsi3_cs_port != NULL) {
        HAL_GPIO_WritePin(dsi3_cs_port, dsi3_cs_pin, GPIO_PIN_RESET);
    }
}

/**
  * @brief  CS片选拉高
  */
static void dsi3_cs_high(void)
{
    if(dsi3_cs_port != NULL) {
        HAL_GPIO_WritePin(dsi3_cs_port, dsi3_cs_pin, GPIO_PIN_SET);
    }
}

/**
  * @brief  SPI数据传输
  */
static uint8_t dsi3_spi_transfer(uint8_t data)
{
    uint8_t rx_data = 0;
    if(hspi_dsi3 != NULL) {
        HAL_SPI_TransmitReceive(hspi_dsi3, &data, &rx_data, 1, DSI3_SPI_TIMEOUT);
    }
    return rx_data;
}

/**
  * @brief  发送DSI3命令
  */
static void dsi3_send_command(uint8_t sensor_id, uint8_t cmd, uint8_t *data, uint8_t len)
{
    if(sensor_id >= DSI3_MAX_SENSORS) return;
    
    dsi3_cs_low();
    HAL_Delay(1);  // 等待芯片就绪
    
    // 发送命令头：传感器ID + 命令
    dsi3_spi_transfer((sensor_id << 4) | cmd);
    
    // 发送数据
    for(uint8_t i = 0; i < len; i++) {
        dsi3_spi_transfer(data[i]);
    }
    
    dsi3_cs_high();
    HAL_Delay(1);
}

/**
  * @brief  接收DSI3响应
  */
static void dsi3_receive_response(uint8_t sensor_id, uint8_t *data, uint8_t len)
{
    if(sensor_id >= DSI3_MAX_SENSORS) return;
    
    dsi3_cs_low();
    HAL_Delay(1);
    
    // 发送读取命令
    dsi3_spi_transfer((sensor_id << 4) | DSI3_CMD_READ_STATUS);
    
    // 接收数据
    for(uint8_t i = 0; i < len; i++) {
        data[i] = dsi3_spi_transfer(0xFF);
    }
    
    dsi3_cs_high();
    HAL_Delay(1);
}

/**
  * @brief  初始化DSI3控制模块
  * @param  hspi: SPI句柄
  * @param  cs_port: CS片选端口
  * @param  cs_pin: CS片选引脚
  */
void dsi3_control_init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    hspi_dsi3 = hspi;
    dsi3_cs_port = cs_port;
    dsi3_cs_pin = cs_pin;
    
    // 初始化传感器结构
    for(uint8_t i = 0; i < DSI3_MAX_SENSORS; i++) {
        dsi3_sensors[i].sensor_id = i;
        dsi3_sensors[i].distance = 0;
        dsi3_sensors[i].status = 0;
        dsi3_sensors[i].enabled = 0;
    }
    
    // 初始化CS引脚
    dsi3_cs_high();
    HAL_Delay(10);
    
    // 复位所有传感器
    for(uint8_t i = 0; i < DSI3_MAX_SENSORS; i++) {
        uint8_t reset_cmd = 0;
        dsi3_send_command(i, DSI3_CMD_RESET, &reset_cmd, 1);
        HAL_Delay(50);
    }
    
    dsi3_initialized = 1;
}

/**
  * @brief  设置DSI3传感器模式
  * @param  sensor_id: 传感器ID (0-3)
  * @param  mode: 模式（NORMAL/SNIFF/SPOOF/JAM）
  */
void dsi3_set_mode(uint8_t sensor_id, uint8_t mode)
{
    if(sensor_id >= DSI3_MAX_SENSORS || !dsi3_initialized) return;
    
    dsi3_send_command(sensor_id, DSI3_CMD_SET_MODE, &mode, 1);
    dsi3_sensors[sensor_id].enabled = (mode != DSI3_MODE_NORMAL);
}

/**
  * @brief  读取DSI3传感器距离
  * @param  sensor_id: 传感器ID
  * @param  distance: 距离值输出（厘米）
  */
void dsi3_read_distance(uint8_t sensor_id, uint16_t *distance)
{
    if(sensor_id >= DSI3_MAX_SENSORS || !dsi3_initialized) return;
    
    uint8_t rx_data[2] = {0};
    dsi3_receive_response(sensor_id, rx_data, 2);
    
    *distance = (rx_data[0] << 8) | rx_data[1];
    dsi3_sensors[sensor_id].distance = *distance;
}

/**
  * @brief  欺骗DSI3传感器距离
  * @param  sensor_id: 传感器ID
  * @param  fake_distance: 虚假距离（厘米）
  */
void dsi3_spoof_distance(uint8_t sensor_id, uint16_t fake_distance)
{
    if(sensor_id >= DSI3_MAX_SENSORS || !dsi3_initialized) return;
    
    uint8_t data[2] = {
        (uint8_t)(fake_distance >> 8),
        (uint8_t)(fake_distance & 0xFF)
    };
    
    dsi3_send_command(sensor_id, DSI3_CMD_SPOOF_DISTANCE, data, 2);
    dsi3_sensors[sensor_id].distance = fake_distance;
}

/**
  * @brief  启动DSI3干扰
  * @param  sensor_id: 传感器ID
  */
void dsi3_jamming_start(uint8_t sensor_id)
{
    if(sensor_id >= DSI3_MAX_SENSORS || !dsi3_initialized) return;
    
    uint8_t enable = 1;
    dsi3_send_command(sensor_id, DSI3_CMD_JAMMING, &enable, 1);
    dsi3_sensors[sensor_id].enabled = 1;
}

/**
  * @brief  停止DSI3干扰
  * @param  sensor_id: 传感器ID
  */
void dsi3_jamming_stop(uint8_t sensor_id)
{
    if(sensor_id >= DSI3_MAX_SENSORS || !dsi3_initialized) return;
    
    uint8_t enable = 0;
    dsi3_send_command(sensor_id, DSI3_CMD_JAMMING, &enable, 1);
    dsi3_sensors[sensor_id].enabled = 0;
}

/**
  * @brief  启动DSI3嗅探
  * @param  sensor_id: 传感器ID
  */
void dsi3_sniff_start(uint8_t sensor_id)
{
    if(sensor_id >= DSI3_MAX_SENSORS || !dsi3_initialized) return;
    
    dsi3_set_mode(sensor_id, DSI3_MODE_SNIFF);
}

/**
  * @brief  获取DSI3嗅探数据
  * @param  sensor_id: 传感器ID
  * @param  data: 数据缓冲区
  * @param  len: 数据长度（输入输出参数）
  * @retval 0=成功, 1=失败
  */
uint8_t dsi3_sniff_get_data(uint8_t sensor_id, uint8_t *data, uint16_t *len)
{
    if(sensor_id >= DSI3_MAX_SENSORS || !dsi3_initialized) return 1;
    
    uint8_t status = 0;
    dsi3_receive_response(sensor_id, &status, 1);
    
    if(status & 0x01) {  // 有数据可用
        uint8_t data_len = (status >> 1) & 0x3F;
        if(data_len > *len) data_len = *len;
        
        dsi3_receive_response(sensor_id, data, data_len);
        *len = data_len;
        return 0;
    }
    
    return 1;
}

/**
  * @brief  处理DSI3模块（主循环中调用）
  */
void dsi3_process(void)
{
    if(!dsi3_initialized) return;
    
    // 定期更新传感器状态
    static uint32_t last_update = 0;
    uint32_t current_time = HAL_GetTick();
    
    if(current_time - last_update > 100) {  // 每100ms更新一次
        for(uint8_t i = 0; i < DSI3_MAX_SENSORS; i++) {
            if(dsi3_sensors[i].enabled) {
                dsi3_read_distance(i, &dsi3_sensors[i].distance);
            }
        }
        last_update = current_time;
    }
}

