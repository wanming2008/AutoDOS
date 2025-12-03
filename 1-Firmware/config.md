# AutoDOS 固件配置说明

## 1. HAL模块配置

在使用本固件前，需要在 `stm32h7xx_hal_conf.h` 中启用以下HAL模块：

### 必须启用的模块

```c
#define HAL_TIM_MODULE_ENABLED    // 用于超声波PWM输出
#define HAL_ADC_MODULE_ENABLED    // 用于超声波回波检测
#define HAL_FDCAN_MODULE_ENABLED  // 用于CAN中间人（通常已启用）
#define HAL_SPI_MODULE_ENABLED    // 用于SPI通信（通常已启用）
#define HAL_PCD_MODULE_ENABLED    // 用于USB通信（通常已启用）
#define HAL_GPIO_MODULE_ENABLED   // GPIO（通常已启用）
#define HAL_DMA_MODULE_ENABLED    // DMA（通常已启用）
#define HAL_RCC_MODULE_ENABLED    // 时钟配置（通常已启用）
#define HAL_PWR_MODULE_ENABLED    // 电源管理（通常已启用）
#define HAL_CORTEX_MODULE_ENABLED // Cortex内核（通常已启用）
```

### 配置方法

在 `CM7/Inc/stm32h7xx_hal_conf.h` 文件中，找到对应的模块定义，取消注释：

```c
// 修改前
/* #define HAL_TIM_MODULE_ENABLED   */
/* #define HAL_ADC_MODULE_ENABLED   */

// 修改后
#define HAL_TIM_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED
```

## 2. 定时器配置（TIM1）

TIM1用于超声波PWM输出，需要配置为：
- **模式**: PWM模式1
- **通道**: CH1和CH2（互补输出）
- **频率**: 48kHz
- **占空比**: 50%
- **预分频器**: 根据系统时钟调整
- **自动重装载值**: 计算得出（例如：系统时钟240MHz，预分频1，ARR = 240000000/48000 = 5000）

### 配置示例

使用STM32CubeMX配置：
1. 选择TIM1
2. 启用Channel 1和Channel 2
3. 设置为PWM Generation CH1和PWM Generation CH2
4. 配置参数：
   - Prescaler: 0
   - Counter Period: 4999 (对于240MHz系统时钟)
   - Pulse: 2500 (50%占空比)
   - Mode: PWM mode 1
   - Fast Mode: Disable

## 3. ADC配置（ADC1）

ADC1用于超声波回波检测，需要配置为：
- **分辨率**: 12位
- **模式**: 连续转换模式
- **通道**: ADC1_IN0（PA0，可根据硬件调整）
- **采样时间**: 建议1.5个周期或更长
- **数据对齐**: 右对齐

### 配置示例

使用STM32CubeMX配置：
1. 选择ADC1
2. 启用IN0通道
3. 配置参数：
   - Resolution: 12 bits
   - Data Alignment: Right
   - Scan Conversion Mode: Disable
   - Continuous Conversion Mode: Enable
   - Discontinuous Conversion Mode: Disable
   - End Of Conversion Selection: EOC flag at the end of single conversion
   - Conversion Data Management: DMA disabled (or enabled for better performance)
   - Overrun: Data preserved
   - Sampling Time: 1.5 Cycles

## 4. CAN配置（FDCAN1和FDCAN2）

FDCAN1和FDCAN2用于CAN中间人功能，配置为：
- **模式**: Normal mode
- **帧格式**: Classic CAN（或CANFD）
- **波特率**: 500Kbps（可调整）
- **自动重传**: Disable
- **接收FIFO**: 16个元素
- **发送FIFO**: 16个元素

### 配置参数

```
Nominal Prescaler: 24
Nominal Sync Jump Width: 1
Nominal Time Seg1: 2
Nominal Time Seg2: 2
Data Prescaler: 1
Data Sync Jump Width: 1
Data Time Seg1: 1
Data Time Seg2: 1
```

## 5. USB配置

USB配置为虚拟串口（CDC）：
- **模式**: Device mode
- **速度**: Full Speed
- **端点数量**: 9
- **类**: CDC

## 6. SPI配置（SPI6）

SPI6用于备用通信：
- **模式**: Master mode
- **数据大小**: 8位
- **时钟极性**: Low
- **时钟相位**: 1 Edge
- **波特率预分频器**: 32

## 7. 系统时钟配置

系统时钟配置为240MHz：
- **PLL源**: HSI
- **PLLM**: 16
- **PLLN**: 240
- **PLLP**: 2
- **PLLQ**: 8
- **PLLR**: 8
- **AHB分频**: 2
- **APB分频**: 2

## 8. GPIO配置

### LED配置
- **LED1 (PB0)**: 输出，推挽，低速
- **LED2 (PE1)**: 输出，推挽，低速
- **LED3 (PB14)**: 输出，推挽，低速

### CAN GPIO配置
- **FDCAN1_RX (PD0)**: 复用功能，AF9
- **FDCAN1_TX (PD1)**: 复用功能，AF9
- **FDCAN2_RX (PB12)**: 复用功能，AF9
- **FDCAN2_TX (PB6)**: 复用功能，AF9

### TIM1 GPIO配置
- **TIM1_CH1 (PA8)**: 复用功能，AF1
- **TIM1_CH2 (PA9)**: 复用功能，AF1

### ADC GPIO配置
- **ADC1_IN0 (PA0)**: 模拟输入

### USB GPIO配置
- **USB_DM (PA11)**: 复用功能，AF10
- **USB_DP (PA12)**: 复用功能，AF10

### SPI GPIO配置
- **SPI6_NSS (PA4)**: 复用功能，AF8
- **SPI6_SCK (PA5)**: 复用功能，AF8
- **SPI6_MISO (PA6)**: 复用功能，AF8
- **SPI6_MOSI (PA7)**: 复用功能，AF8

## 9. 中断配置

需要配置的中断：
- **FDCAN1_IT0_IRQn**: CAN1中断（优先级0）
- **FDCAN_CAL_IRQn**: CAN校准中断（优先级0）
- **OTG_FS_IRQn**: USB中断（优先级0）

## 10. 使用STM32CubeMX配置

推荐使用STM32CubeMX进行配置：

1. 打开STM32CubeMX
2. 选择STM32H745ZITx
3. 配置系统时钟为240MHz
4. 启用所需外设：
   - TIM1（PWM模式）
   - ADC1（连续转换）
   - FDCAN1和FDCAN2
   - USB_OTG_FS（Device模式）
   - SPI6
5. 配置GPIO
6. 配置中断
7. 生成代码
8. 将生成的代码与本固件代码合并

## 11. 编译配置

### 编译器设置
- **优化级别**: -O2（推荐）或-Os（代码大小优化）
- **浮点运算**: 硬件FPU（STM32H7支持）

### 链接器设置
- **堆大小**: 至少0x2000
- **栈大小**: 至少0x2000

## 12. 调试配置

### ST-Link配置
- **接口**: SWD
- **速度**: 4MHz（推荐）
- **复位模式**: Hardware reset

### 调试信息
- 启用调试信息输出（printf重定向到SWO或UART）

## 13. 常见配置问题

### 问题1: TIM1无法输出PWM
- 检查TIM1时钟是否使能
- 检查GPIO复用功能配置
- 检查PWM通道配置
- 检查自动重装载值和预分频器

### 问题2: ADC无法读取
- 检查ADC时钟是否使能
- 检查GPIO配置为模拟输入
- 检查ADC通道配置
- 检查采样时间设置

### 问题3: CAN无法通信
- 检查CAN收发器连接
- 检查终端电阻（120Ω）
- 检查波特率配置
- 检查GPIO复用功能

### 问题4: USB无法识别
- 检查USB连接
- 检查USB驱动安装
- 检查USB时钟配置（HSI48）
- 检查GPIO配置

## 14. 验证配置

配置完成后，验证以下功能：
1. LED2闪烁（系统运行）
2. USB连接后LED1点亮
3. CAN消息可以正常转发
4. 超声波PWM可以输出（用示波器测量）
5. ADC可以读取值（用万用表测量）

## 15. 注意事项

1. **时钟配置**: 确保系统时钟正确配置，TIM1和ADC的时钟源正确
2. **GPIO复用**: 确保GPIO复用功能正确配置
3. **中断优先级**: 合理配置中断优先级，避免冲突
4. **DMA配置**: 如果需要高性能，可以配置DMA用于ADC和CAN
5. **电源管理**: 确保电源稳定，建议使用LDO稳压器

