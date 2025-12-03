# AutoDOS Hardware Interface Documentation

## 1. Overview

AutoDOS is an ADAS (Advanced Driver Assistance Systems) security testing tool based on the STM32H7 dual-core (Cortex-M7 and Cortex-M4) platform. It integrates CAN man-in-the-middle, ultrasonic radar attack, and millimeter-wave radar attack functionalities.

## 2. Hardware Platform

- **Main Controller**: STM32H745ZITx (Dual-core: CM7 + CM4)
- **Operating Voltage**: 3.3V
- **Operating Frequency**: 240MHz (CM7 Core)

## 3. Hardware Interface List

### 3.1 CAN Bus Interfaces

#### FDCAN1 Interface
- **RX**: PD0 (FDCAN1_RX)
- **TX**: PD1 (FDCAN1_TX)
- **Function**: CAN Bus 1, used to connect to vehicle ECU or sensors
- **Protocol**: CAN 2.0 / CAN FD
- **Baud Rate**: 500 kbps (configurable)

#### FDCAN2 Interface
- **RX**: PB12 (FDCAN2_RX)
- **TX**: PB6 (FDCAN2_TX)
- **Function**: CAN Bus 2, used to connect to vehicle ECU or sensors
- **Protocol**: CAN 2.0 / CAN FD
- **Baud Rate**: 500 kbps (configurable)

**Connection Notes**:
- FDCAN1 and FDCAN2 function as CAN man-in-the-middle, capable of bidirectional forwarding and modification of CAN messages
- Typical application: FDCAN1 connects to a millimeter-wave radar, FDCAN2 connects to an ECU

### 3.2 Ultrasonic Radar Interface

#### PWM Output Interface (Ultrasonic Transmission)
- **TIM1_CH1**: PA8 (Ultrasonic Transducer Positive)
- **TIM1_CH2**: PA9 (Ultrasonic Transducer Negative, Complementary Output)
- **Function**: Outputs 48kHz PWM signal to drive the ultrasonic transducer
- **Frequency**: 48 kHz
- **Duty Cycle**: 50%
- **Output Mode**: Complementary PWM (Push-Pull Output)

#### ADC Input Interface (Ultrasonic Echo Detection)
- **ADC1_IN0**: PA0 (Echo Signal Detection)
- **Function**: Detects ultrasonic echo signal voltage
- **Resolution**: 12-bit
- **Reference Voltage**: 3.3V
- **Detection Thresholds**:
  - Low Threshold: 2.2V (ADC Value: 450)
  - High Threshold: 2.8V (ADC Value: 570)

**Connection Notes**:
- The ultrasonic transducer must be connected to TIM1_CH1 and TIM1_CH2
- The echo detection circuit output should be connected to ADC1_IN0
- It is recommended to use an operational amplifier for signal conditioning

### 3.3 Communication Interfaces

#### USB Interface
- **DM**: PA11 (USB_OTG_FS_DM)
- **DP**: PA12 (USB_OTG_FS_DP)
- **Function**: USB Virtual COM Port (CDC), used for communication with a PC
- **Speed**: Full Speed (12 Mbps)
- **Purpose**:
  - Receive attack commands
  - Send CAN data packets
  - Configure attack parameters

#### SPI Interfaces

##### SPI6 Interface (For DSI3 and Communication)
- **NSS**: PA4 (SPI6_NSS) or software-controlled
- **SCK**: PA5 (SPI6_SCK)
- **MISO**: PA6 (SPI6_MISO)
- **MOSI**: PA7 (SPI6_MOSI)
- **Function**: SPI Master Mode, used for DSI3 control and communication
- **Baud Rate**: System Clock / 32
- **Purpose**:
  - DSI3 conversion chip control
  - Alternate communication interface

##### SPI3 Interface (For CC1101)
- **SCK**: PC10 (SPI3_SCK)
- **MISO**: PC11 (SPI3_MISO)
- **MOSI**: PC12 (SPI3_MOSI)
- **CS**: Software-controlled (suggested PB5)
- **Function**: SPI Master Mode, used for CC1101 chip control
- **Baud Rate**: System Clock / 8
- **Purpose**: TPMS CC1101 communication

##### DSI3 Interface
- **CS**: Software-controlled (suggested PB3 or PB4)
- **Function**: DSI3 conversion chip chip select
- **Purpose**: Controls SPI-to-DSI3 chip

##### CC1101 Interface
- **CS**: Software-controlled (suggested PB5)
- **GDO0**: Software-controlled (suggested PB6 or PB7)
- **Function**: CC1101 chip select and packet detection
- **Purpose**: TPMS signal transmission and reception

### 3.4 Status Indicator LEDs

- **LED1**: PB0
  - Function: USB connection status indicator
  - On: USB connected
  - Off: USB not connected

- **LED2**: PE1
  - Function: System operation status indicator
  - Blinking: System operating normally (100ms period)

- **LED3**: PB14
  - Function: Reserved

### 3.5 Other Interfaces

- **RDY**: PE6
  - Function: Ready signal input (pull-down)
  - Purpose: External device ready detection

- **PD8**:
  - Function: General-purpose output (used by CM4 core)

## 4. Power Supply Interfaces

- **VDD**: 3.3V
- **GND**: Ground
- **VBUS**: USB power (5V, used for USB detection)

## 5. Hardware Connection Diagram

```
                    ┌─────────────────┐
                    │   STM32H745     │
                    │                 │
    CAN1_H ────────┤ PD0 (FDCAN1_RX) │
    CAN1_L ────────┤ PD1 (FDCAN1_TX) │
                    │                 │
    CAN2_H ────────┤ PB12(FDCAN2_RX) │
    CAN2_L ────────┤ PB6 (FDCAN2_TX) │
                    │                 │
 Ultrasonic Tx+ ───┤ PA8 (TIM1_CH1)  │
 Ultrasonic Tx- ───┤ PA9 (TIM1_CH2)  │
                    │                 │
  Echo Detect Sig ─┤ PA0 (ADC1_IN0)  │
                    │                 │
    USB_DM ────────┤ PA11            │
    USB_DP ────────┤ PA12            │
                    │                 │
    SPI_NSS ───────┤ PA4             │
    SPI_SCK ───────┤ PA5             │
    SPI_MISO ──────┤ PA6             │
    SPI_MOSI ──────┤ PA7             │
                    └─────────────────┘
```

## 6. Hardware Interface Summary

| Interface Type | Quantity | Pins | Function |
|---------|------|------|------|
| CAN Bus | 2 | PD0/PD1, PB12/PB6 | CAN man-in-the-middle forwarding |
| Ultrasonic PWM | 2 | PA8, PA9 | 48kHz ultrasonic output |
| ADC Input | 1 | PA0 | Echo signal detection |
| USB | 1 | PA11/PA12 | PC communication |
| SPI6 | 1 | PA4-PA7 | DSI3 control and communication |
| SPI3 | 1 | PC10-PC12 | CC1101 communication |
| DSI3 CS | 1 | Configurable (PB3/PB4) | DSI3 chip select |
| CC1101 CS | 1 | Configurable (PB5) | CC1101 chip select |
| CC1101 GDO0 | 1 | Configurable (PB6/PB7) | Packet detection |
| LED Indicators | 3 | PB0, PE1, PB14 | Status indication |
| GPIO | 2 | PE6, PD8 | General-purpose I/O |

**Total Hardware Interfaces**: 17 main interfaces

## 7. Important Notes

1. **CAN Bus**: Requires a 120Ω termination resistor. It is recommended to add one between CAN_H and CAN_L.
2. **Ultrasonic**: Requires a driver circuit. Using an H-bridge or dedicated driver chip is recommended.
3. **ADC Input**: The echo signal requires signal conditioning. Using an operational amplifier for amplification and filtering is recommended.
4. **Power Supply**: Ensure stable 3.3V power supply. Using an LDO voltage regulator is recommended.
5. **Grounding**: All GND connections must be well-connected to avoid interference.

## 8. Hardware Configuration Recommendations

### 8.1 Ultrasonic Driver Circuit
- Use an H-bridge driver chip (e.g., DRV8833) or a dedicated ultrasonic driver chip.
- Adjust output power according to transducer specifications.
- Adding a protection circuit is recommended.

### 8.2 Echo Detection Circuit
- Use an operational amplifier for signal amplification.
- Add a band-pass filter (center frequency 48 kHz).
- ADC input range: 0-3.3V.

### 8.3 CAN Bus Interface
- Use a CAN transceiver (e.g., TJA1051).
- Add ESD protection.
- Include a 120Ω termination resistor.

## 9. Test Interfaces

For ease of testing, it is recommended to add the following to the PCB:
- Test points: For all key signals.
- Jumpers: For configuring different operating modes.
- Debug interface: SWD interface for program download and debugging.