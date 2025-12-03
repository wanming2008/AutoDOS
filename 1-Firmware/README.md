# AutoDOS

## Project Overview

AutoDOS is an ADAS (Advanced Driver Assistance Systems) security testing tool firmware based on the STM32H7 dual-core architecture. This project integrates CAN man-in-the-middle, ultrasonic radar attack, and millimeter-wave radar attack functionalities, providing a comprehensive hardware solution for ADAS security research.

## Features

### 1. CAN Man-in-the-Middle Function
- Bidirectional forwarding of CAN messages between FDCAN1 and FDCAN2
- Supports real-time modification of CAN message IDs and data
- Supports 7 independent tampering rule configurations
- Supports CAN 2.0 and CAN FD protocols

### 2. Ultrasonic Radar Attack
- **Interference Mode**: Continuously outputs 48kHz ultrasonic signals to interfere with ultrasonic radar
- **Spoofing Mode**: Detects radar transmission signals and delays transmitting false echoes to achieve distance spoofing
- Configurable spoofing delay distance

### 3. Millimeter-Wave Radar Attack
- Identifies millimeter-wave radar CAN messages
- Modifies key data such as distance and target count
- Forwards modified messages via the CAN man-in-the-middle function

### 4. DSI3 Bus Control
- Controls DSI3 conversion chip via SPI
- Supports DSI3 bus sniffing, spoofing, and interference
- Supports up to 4 DSI3 sensor channels
- Implements ultrasonic radar DSI3 protocol attacks

### 5. TPMS (Tire Pressure Monitoring System) Attack
- Uses TI CC1101 chip for 433MHz signal simulation
- Supports tire pressure spoofing (low pressure, high pressure, tire burst)
- Supports 4 tire positions
- Supports TPMS signal sniffing

## Hardware Platform

- **Main Controller**: STM32H745ZITx (Dual-core: Cortex-M7 + Cortex-M4)
- **Operating Frequency**: 240MHz (CM7 Core)
- **Operating Voltage**: 3.3V

## Project Structure

```
AutoDOS_Integrated/
├── CM7/                    # Cortex-M7 Core Code
│   ├── Inc/               # Header Files
│   │   ├── main.h
│   │   ├── uss_attack.h      # Ultrasonic Attack Module
│   │   ├── mmwave_attack.h   # Millimeter-Wave Attack Module
│   │   ├── dsi3_control.h    # DSI3 Control Module
│   │   └── tpms_cc1101.h     # TPMS Attack Module
│   └── Src/                  # Source Files
│       ├── main.c            # Main Program
│       ├── uss_attack.c      # Ultrasonic Attack Implementation
│       ├── mmwave_attack.c   # Millimeter-Wave Attack Implementation
│       ├── dsi3_control.c    # DSI3 Control Implementation
│       └── tpms_cc1101.c     # TPMS Attack Implementation
├── CM4/                   # Cortex-M4 Core Code (Reserved)
├── Hardware_Interface_Description.md        # Detailed Hardware Interface Description
├── Firmware_Functionality_and_Usage_Guide.md  # Complete Firmware Functionality and Usage Guide (Includes DSI3 and TPMS)
└── README.md              # This File
```

## Quick Start

### 1. Hardware Preparation

Refer to `Hardware_Interface_Description.md` for hardware connections:
- CAN bus interfaces (FDCAN1 and FDCAN2)
- Ultrasonic transducer and driver circuit
- Echo detection circuit
- USB interface

### 2. Software Compilation

Open the project using STM32CubeIDE or Keil MDK:
1. Configure project paths and toolchain
2. Compile CM7 and CM4 core code
3. Download firmware to the target board

### 3. Usage Instructions

Refer to `Firmware_Functionality_and_Usage_Guide.md` for:
- Detailed functionality
- Configuration methods
- Usage examples
- Communication protocols

## Hardware Interface Summary

| Interface Type | Quantity | Function |
|---------|------|------|
| CAN Bus | 2 | CAN man-in-the-middle forwarding |
| Ultrasonic PWM | 2 | 48kHz ultrasonic output |
| ADC Input | 1 | Echo signal detection |
| USB | 1 | PC communication |
| SPI | 2 | SPI6 (DSI3), SPI3 (CC1101) |
| SPI CS | 2 | DSI3 and CC1101 chip select |
| CC1101 GDO0 | 1 | Packet detection |
| LED Indicators | 3 | Status indication |
| GPIO | 2 | General-purpose I/O |

**Total**: 17 main hardware interfaces

## Main Functional Modules

### CAN Man-in-the-Middle Module
- Bidirectional message forwarding
- Real-time message modification
- 7 independent tampering rules

### Ultrasonic Attack Module
- PWM output (48kHz)
- ADC echo detection
- Interference and spoofing modes

### Millimeter-Wave Radar Attack Module
- CAN message identification
- Data modification
- Automatic forwarding

## Communication Interfaces

- **USB Virtual COM Port**: Communicates with PC to receive commands and send data
- **SPI Interface**: Alternate communication interface

## Documentation

- [hardware-interface.md](hardware-interface.md) - Detailed hardware interface description
- [firmware-fuction.md](firmware-fuction.md) - Complete firmware functionality and usage guide (includes DSI3 and TPMS)
- [config.md](config.md) - HAL module and peripheral configuration instructions
- [uss.md](uss.md) - Ultrasonic transmitter hardware principles

## Security Warning

⚠️ **IMPORTANT**: This firmware is intended for security research and testing purposes only. It must not be used for illegal attacks on real vehicles. Testing should be conducted in controlled environments and comply with local laws and regulations.

## Version Information

- **Version**: 1.0.0
- **Release Date**: 2023
- **Supported Platform**: STM32H745ZITx

## Development Environment

- STM32CubeIDE 1.x
- Keil MDK-ARM 5.x
- STM32CubeMX (for configuration)

## Dependencies

- STM32H7 HAL Library
- USB Device Library (CDC Class)
- STM32H7 BSP Library

## License

This project is based on the AutoDOS open-source project and follows the corresponding open-source license.

## Contribution

Issues and Pull Requests are welcome.

## Contact

For questions, please refer to the documentation or submit an Issue.

---

**Note**: This project integrates the following original projects:
- ST_CAN_0.0: CAN man-in-the-middle firmware
- uss_attack_program: Ultrasonic attack program

## New Features

### DSI3 Bus Control
- Supports SPI-to-DSI3 chip control
- Implements DSI3 bus sniffing, spoofing, and interference
- Supports synchronous control of 4 sensor channels

### TPMS Tire Pressure Attack
- Uses TI CC1101 for 433MHz signal simulation
- Supports multiple tire pressure attack scenarios
- Supports sensor ID configuration and signal sniffing