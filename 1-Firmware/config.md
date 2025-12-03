# AutoDOS Firmware Configuration Guide  

## 1. HAL Module Configuration  

Before using this firmware, the following HAL modules must be enabled in `stm32h7xx_hal_conf.h`:  

### Required Modules  

```c
#define HAL_TIM_MODULE_ENABLED    // For ultrasonic PWM output
#define HAL_ADC_MODULE_ENABLED    // For ultrasonic echo detection
#define HAL_FDCAN_MODULE_ENABLED  // For CAN man-in-the-middle (usually enabled)
#define HAL_SPI_MODULE_ENABLED    // For SPI communication (usually enabled)
#define HAL_PCD_MODULE_ENABLED    // For USB communication (usually enabled)
#define HAL_GPIO_MODULE_ENABLED   // GPIO (usually enabled)
#define HAL_DMA_MODULE_ENABLED    // DMA (usually enabled)
#define HAL_RCC_MODULE_ENABLED    // Clock configuration (usually enabled)
#define HAL_PWR_MODULE_ENABLED    // Power management (usually enabled)
#define HAL_CORTEX_MODULE_ENABLED // Cortex core (usually enabled)
```

### Configuration Method  

In the `CM7/Inc/stm32h7xx_hal_conf.h` file, locate the corresponding module definitions and uncomment them:  

```c
// Before modification
/* #define HAL_TIM_MODULE_ENABLED   */
/* #define HAL_ADC_MODULE_ENABLED   */

// After modification
#define HAL_TIM_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED
```

## 2. Timer Configuration (TIM1)  

TIM1 is used for ultrasonic PWM output and must be configured as follows:  
- **Mode**: PWM mode 1  
- **Channels**: CH1 and CH2 (complementary output)  
- **Frequency**: 48 kHz  
- **Duty Cycle**: 50%  
- **Prescaler**: Adjusted based on system clock  
- **Auto-reload Register (ARR)**: Calculated (e.g., for a 240 MHz system clock with prescaler 1, ARR = 240000000 / 48000 = 5000)  

### Configuration Example  

Using STM32CubeMX:  
1. Select TIM1  
2. Enable Channel 1 and Channel 2  
3. Set to PWM Generation CH1 and PWM Generation CH2  
4. Configure parameters:  
   - Prescaler: 0  
   - Counter Period: 4999 (for a 240 MHz system clock)  
   - Pulse: 2500 (50% duty cycle)  
   - Mode: PWM mode 1  
   - Fast Mode: Disable  

## 3. ADC Configuration (ADC1)  

ADC1 is used for ultrasonic echo detection and must be configured as follows:  
- **Resolution**: 12-bit  
- **Mode**: Continuous conversion mode  
- **Channel**: ADC1_IN0 (PA0, adjustable based on hardware)  
- **Sampling Time**: Recommended 1.5 cycles or longer  
- **Data Alignment**: Right-aligned  

### Configuration Example  

Using STM32CubeMX:  
1. Select ADC1  
2. Enable IN0 channel  
3. Configure parameters:  
   - Resolution: 12 bits  
   - Data Alignment: Right  
   - Scan Conversion Mode: Disable  
   - Continuous Conversion Mode: Enable  
   - Discontinuous Conversion Mode: Disable  
   - End Of Conversion Selection: EOC flag at the end of single conversion  
   - Conversion Data Management: DMA disabled (or enabled for better performance)  
   - Overrun: Data preserved  
   - Sampling Time: 1.5 Cycles  

## 4. CAN Configuration (FDCAN1 and FDCAN2)  

FDCAN1 and FDCAN2 are used for CAN man-in-the-middle functionality, configured as follows:  
- **Mode**: Normal mode  
- **Frame Format**: Classic CAN (or CAN FD)  
- **Baud Rate**: 500 kbps (adjustable)  
- **Automatic Retransmission**: Disable  
- **Receive FIFO**: 16 elements  
- **Transmit FIFO**: 16 elements  

### Configuration Parameters  

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

## 5. USB Configuration  

USB is configured as a virtual COM port (CDC):  
- **Mode**: Device mode  
- **Speed**: Full Speed  
- **Number of Endpoints**: 9  
- **Class**: CDC  

## 6. SPI Configuration (SPI6)  

SPI6 is used for auxiliary communication:  
- **Mode**: Master mode  
- **Data Size**: 8-bit  
- **Clock Polarity**: Low  
- **Clock Phase**: 1 Edge  
- **Baud Rate Prescaler**: 32  

## 7. System Clock Configuration  

System clock is configured to 240 MHz:  
- **PLL Source**: HSI  
- **PLLM**: 16  
- **PLLN**: 240  
- **PLLP**: 2  
- **PLLQ**: 8  
- **PLLR**: 8  
- **AHB Prescaler**: 2  
- **APB Prescaler**: 2  

## 8. GPIO Configuration  

### LED Configuration  
- **LED1 (PB0)**: Output, push-pull, low speed  
- **LED2 (PE1)**: Output, push-pull, low speed  
- **LED3 (PB14)**: Output, push-pull, low speed  

### CAN GPIO Configuration  
- **FDCAN1_RX (PD0)**: Alternate function, AF9  
- **FDCAN1_TX (PD1)**: Alternate function, AF9  
- **FDCAN2_RX (PB12)**: Alternate function, AF9  
- **FDCAN2_TX (PB6)**: Alternate function, AF9  

### TIM1 GPIO Configuration  
- **TIM1_CH1 (PA8)**: Alternate function, AF1  
- **TIM1_CH2 (PA9)**: Alternate function, AF1  

### ADC GPIO Configuration  
- **ADC1_IN0 (PA0)**: Analog input  

### USB GPIO Configuration  
- **USB_DM (PA11)**: Alternate function, AF10  
- **USB_DP (PA12)**: Alternate function, AF10  

### SPI GPIO Configuration  
- **SPI6_NSS (PA4)**: Alternate function, AF8  
- **SPI6_SCK (PA5)**: Alternate function, AF8  
- **SPI6_MISO (PA6)**: Alternate function, AF8  
- **SPI6_MOSI (PA7)**: Alternate function, AF8  

## 9. Interrupt Configuration  

Required interrupts to configure:  
- **FDCAN1_IT0_IRQn**: CAN1 interrupt (priority 0)  
- **FDCAN_CAL_IRQn**: CAN calibration interrupt (priority 0)  
- **OTG_FS_IRQn**: USB interrupt (priority 0)  

## 10. Using STM32CubeMX for Configuration  

It is recommended to use STM32CubeMX for configuration:  

1. Open STM32CubeMX  
2. Select STM32H745ZITx  
3. Configure the system clock to 240 MHz  
4. Enable required peripherals:  
   - TIM1 (PWM mode)  
   - ADC1 (continuous conversion)  
   - FDCAN1 and FDCAN2  
   - USB_OTG_FS (Device mode)  
   - SPI6  
5. Configure GPIO  
6. Configure interrupts  
7. Generate code  
8. Merge the generated code with this firmware code  

## 11. Compilation Configuration  

### Compiler Settings  
- **Optimization Level**: -O2 (recommended) or -Os (code size optimization)  
- **Floating-Point Unit**: Hardware FPU (STM32H7 supports it)  

### Linker Settings  
- **Heap Size**: At least 0x2000  
- **Stack Size**: At least 0x2000  

## 12. Debug Configuration  

### ST-Link Configuration  
- **Interface**: SWD  
- **Speed**: 4 MHz (recommended)  
- **Reset Mode**: Hardware reset  

### Debug Information  
- Enable debug information output (printf redirected to SWO or UART)  

## 13. Common Configuration Issues  

### Issue 1: TIM1 Unable to Output PWM  
- Check if TIM1 clock is enabled  
- Check GPIO alternate function configuration  
- Check PWM channel configuration  
- Check auto-reload value and prescaler  

### Issue 2: ADC Unable to Read Values  
- Check if ADC clock is enabled  
- Check GPIO configured as analog input  
- Check ADC channel configuration  
- Check sampling time settings  

### Issue 3: CAN Communication Failure  
- Check CAN transceiver connection  
- Check termination resistor (120 Ω)  
- Check baud rate configuration  
- Check GPIO alternate function  

### Issue 4: USB Not Recognized  
- Check USB connection  
- Check USB driver installation  
- Check USB clock configuration (HSI48)  
- Check GPIO configuration  

## 14. Configuration Verification  

After configuration, verify the following functionalities:  
1. LED2 blinking (system running)  
2. LED1 lighting up when USB is connected  
3. CAN messages can be forwarded normally  
4. Ultrasonic PWM output (measure with an oscilloscope)  
5. ADC can read values (measure with a multimeter)  

## 15. Notes  

1. **Clock Configuration**: Ensure the system clock is correctly configured, and TIM1 and ADC clock sources are correct  
2. **GPIO Alternate Functions**: Ensure GPIO alternate functions are correctly configured  
3. **Interrupt Priority**: Configure interrupt priorities reasonably to avoid conflicts  
4. **DMA Configuration**: For better performance, configure DMA for ADC and CAN if needed  
5. **Power Management**: Ensure stable power supply; an LDO voltage regulator is recommended