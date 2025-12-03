# AutoDOS Firmware Functionality and User Manual

## 1. Firmware Overview

The AutoDOS firmware is an ADAS (Advanced Driver Assistance Systems) security testing tool firmware based on the STM32H7 dual-core architecture, integrating the following functionalities:

1. **CAN Man-in-the-Middle Functionality**: Bidirectional forwarding and modification of CAN messages.
2. **Ultrasonic Radar Attack**: Supports both interference and spoofing modes.
3. **Millimeter-Wave Radar Attack**: Implements attacks by modifying CAN messages.
4. **DSI3 Bus Control**: Controls DSI3 conversion chips via SPI to implement DSI3 bus sniffing, spoofing, and interference.
5. **TPMS (Tire Pressure Monitoring System) Attack**: Uses TI CC1101 chip to simulate 433MHz tire pressure signals.

## 2. Firmware Architecture

### 2.1 Dual-Core Architecture

- **CM7 Core (Main Core)**:
  - Handles CAN man-in-the-middle functionality.
  - Handles ultrasonic radar attack.
  - Handles millimeter-wave radar attack.
  - Handles DSI3 bus control.
  - Handles TPMS attack.
  - USB/SPI communication.
  - Main system loop.

- **CM4 Core (Secondary Core)**:
  - Reserved for extended functionalities.
  - Currently implements simple LED blinking.

### 2.2 Module Structure

```
AutoDOS Firmware
├── CAN Man-in-the-Middle Module
│   ├── FDCAN1/FDCAN2 Initialization
│   ├── Message Forwarding
│   └── Message Tampering (7 slot configurations)
├── Ultrasonic Attack Module
│   ├── PWM Output (48kHz)
│   ├── ADC Echo Detection
│   ├── Interference Mode
│   └── Spoofing Mode
├── Millimeter-Wave Radar Attack Module
│   ├── CAN Message Identification
│   └── Message Modification
├── DSI3 Control Module
│   ├── SPI-to-DSI3 Control
│   ├── Sniffing Mode
│   ├── Spoofing Mode
│   └── Jamming Mode
├── TPMS Attack Module
│   ├── CC1101 Control
│   ├── Tire Pressure Spoofing
│   └── Signal Sniffing
└── Communication Module
    ├── USB Virtual COM Port
    └── SPI Interface
```

## 3. Functionality Details

### 3.1 CAN Man-in-the-Middle Functionality

#### Functional Description
- Bidirectionally forwards CAN messages between FDCAN1 and FDCAN2.
- Supports real-time modification of CAN message IDs and data.
- Supports 7 independent tampering rules (slots).
- Supports CAN 2.0 and CAN FD protocols.

#### Working Principle
1. Receive message from FDCAN1 → Check tampering rules → Modify message → Forward to FDCAN2.
2. Receive message from FDCAN2 → Check tampering rules → Modify message → Forward to FDCAN1.

#### Configuration Method
Send configuration data packets (45 bytes) via USB or SPI:

```
Packet Format:
[0xFA][0x21][Slot Index][ID Mask (4 bytes)][Data Mask (8 bytes)][Length (1 byte)]
[ID Target (4 bytes)][Data Target (8 bytes)][ID Tamper Enable (1 byte)][Data Tamper Enable (1 byte)]
[ID Tamper Target (4 bytes)][Length Target (1 byte)][Data Tamper Target (8 bytes)][0x22][0xFB]
```

#### Usage Example
```c
// Configure slot 0: Modify message with ID 0x123, change data to 0xAA
uint8_t config[45] = {
    0xFA, 0x21, 0x00,  // Header and slot index
    // ID Mask: 0xFFFFFFFF (exact match)
    0xFF, 0xFF, 0xFF, 0xFF,
    // Data Mask: 0xFFFFFFFFFFFFFFFF (exact match)
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    // Length: 8 bytes
    0x08,
    // ID Target: 0x123
    0x23, 0x01, 0x00, 0x00,
    // Data Target: Any
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // ID Tamper Enable: No, Data Tamper Enable: Yes
    0x00, 0x01,
    // ID Tamper Target: 0x123 (unchanged)
    0x23, 0x01, 0x00, 0x00,
    // Length Target: 8
    0x08,
    // Data Tamper Target: 0xAA
    0xAA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x22, 0xFB  // Footer
};
```

### 3.2 Ultrasonic Radar Attack Functionality

#### Functional Description
- **Interference Mode**: Continuously outputs 48kHz ultrasonic signals to interfere with normal ultrasonic radar operation.
- **Spoofing Mode**: Detects the ultrasonic radar's transmission signal, delays for a specific time, and then transmits false echoes, causing the radar to detect a false distance.

#### Working Principle

**Interference Mode**:
1. Start TIM1 PWM output.
2. Continuously output 48kHz complementary PWM signals.
3. Drive the ultrasonic transducer to generate interference signals.

**Spoofing Mode**:
1. ADC continuously samples the echo detection signal.
2. Detect voltage crossing boundaries (trigger signal).
3. Record trigger cycles.
4. Delay for a specific time (corresponding to false distance).
5. Transmit 60 cycles of 48kHz pulses.
6. Stop transmission and wait for the next trigger.

#### Parameter Configuration

**Interference Mode Parameters**:
- Frequency: 48kHz (fixed)
- Duty Cycle: 50%
- Output Mode: Complementary PWM

**Spoofing Mode Parameters**:
- Trigger Thresholds:
  - Low Threshold: 2.2V
  - High Threshold: 2.8V
- Delay Distance: Configurable (default 30cm)
- Pulse Cycle Count: 60 cycles

#### Control Commands

Send attack commands via USB or SPI:

```
Command Format:
[0xFA][0x30][Command][Parameter1][Parameter2]...

Command List:
0x01: Ultrasonic Attack Control
  Parameter: 0=Off, 1=Interference Mode, 2=Spoofing Mode

0x03: Set Spoofing Delay Distance
  Parameter: [High Byte][Low Byte] (distance, unit: cm)
```

#### Usage Example
```c
// Start interference mode
uint8_t cmd_jamming[] = {0xFA, 0x30, 0x01, 0x01};

// Start spoofing mode
uint8_t cmd_spoof[] = {0xFA, 0x30, 0x01, 0x02};

// Set spoofing delay to 50cm
uint8_t cmd_delay[] = {0xFA, 0x30, 0x03, 0x00, 0x32};  // 0x0032 = 50

// Stop attack
uint8_t cmd_stop[] = {0xFA, 0x30, 0x01, 0x00};
```

### 3.3 Millimeter-Wave Radar Attack Functionality

#### Functional Description
- Identifies millimeter-wave radar CAN messages.
- Modifies key data such as distance and target count.
- Forwards modified messages via the CAN man-in-the-middle functionality.

#### Working Principle
1. Check message ID in the CAN message tampering function.
2. If it's a millimeter-wave radar-related message, perform data modification.
3. Forward the modified message via CAN man-in-the-middle.

#### Configuration Method

Configure via attack commands:

```
Command Format:
[0xFA][0x30][0x02][Enable][Distance High][Distance Low][Target Count]

Enable: 0=Off, 1=On
Distance: 16-bit, unit: cm
Target Count: 8-bit
```

#### Usage Example
```c
// Enable millimeter-wave radar attack, set false distance to 100cm, false target count to 2
uint8_t cmd_mmwave[] = {
    0xFA, 0x30, 0x02,  // Command header
    0x01,              // Enable
    0x00, 0x64,        // Distance: 100cm
    0x02               // Target Count: 2
};
```

### 3.4 DSI3 Bus Control Functionality

#### Functional Description
DSI3 (Distributed System Interface 3) is a serial communication protocol used for ultrasonic sensors. This firmware controls DSI3 conversion chips via SPI to implement sniffing, spoofing, and interference functionalities on the DSI3 bus.

#### Working Principle
- **Sniffing Mode**: Monitors communication on the DSI3 bus, captures sensor data packets, and analyzes protocols and cycles.
- **Spoofing Mode**: Injects false distance data into the DSI3 bus, allowing arbitrary distance values to be set.
- **Interference Mode**: Continuously transmits interference signals, preventing normal DSI3 communication and disabling sensors.

#### Hardware Connections
- **SPI Interface**: Uses SPI6
  - **SCK/MISO/MOSI**: Uses existing SPI6 configuration.
  - **CS**: Software-controlled (suggested PB3 or PB4).
- **DSI3 Conversion Chip**: Requires a dedicated SPI-to-DSI3 chip supporting up to 4 DSI3 sensor channels.

#### Control Commands
Send commands via USB or SPI:

```
Command Format:
[0xFA][0x30][0x04][Command][Sensor ID][Parameters...]

Command List:
0x01: Start Sniffing Mode
  Parameter: Sensor ID (0-3)

0x02: Spoof Distance
  Parameter: Sensor ID, Distance High Byte, Distance Low Byte

0x03: Start Interference
  Parameter: Sensor ID

0x00: Stop/Restore Normal Mode
  Parameter: Sensor ID
```

#### Usage Example
```c
// Start sniffing mode for sensor 0
uint8_t cmd_sniff[] = {0xFA, 0x30, 0x04, 0x01, 0x00};

// Spoof distance of sensor 1 to 50cm
uint8_t cmd_spoof[] = {0xFA, 0x30, 0x04, 0x02, 0x01, 0x00, 0x32};  // 0x0032 = 50

// Start interference for sensor 2
uint8_t cmd_jam[] = {0xFA, 0x30, 0x04, 0x03, 0x02};

// Stop sensor 0
uint8_t cmd_stop[] = {0xFA, 0x30, 0x04, 0x00, 0x00};
```

### 3.5 TPMS Tire Pressure Attack Functionality

#### Functional Description
Uses the TI CC1101 chip to simulate 433MHz RF signals for attacking TPMS (Tire Pressure Monitoring System). Can simulate various tire pressure anomalies, including low pressure, high pressure, and tire bursts.

#### Working Principle
1. Configure CC1101 to operate at 433.92MHz.
2. Set TPMS sensor ID (configured based on the target vehicle).
3. Encode TPMS data packets (including tire pressure, temperature, etc.).
4. Transmit 433MHz RF signals via CC1101.
5. The vehicle's TPMS receiver receives the signal and triggers warnings.

#### Hardware Connections
- **SPI Interface**: Uses SPI3
  - **SCK**: PC10
  - **MISO**: PC11
  - **MOSI**: PC12
  - **CS**: Software-controlled (suggested PB5)
- **CC1101 Chip**:
  - **GDO0**: Packet detection pin (suggested PB6 or PB7)
  - **Crystal**: 26MHz
- **Antenna**: 433MHz matched antenna

#### Functional Features
- **Tire Pressure Spoofing**: Simulates arbitrary tire pressure values (0-1000kPa), can set temperature values.
- **Attack Scenarios**:
  - Low tire pressure warning: Simulates abnormally low pressure (e.g., 150kPa).
  - High tire pressure warning: Simulates abnormally high pressure (e.g., 350kPa).
  - Tire burst simulation: Simulates zero pressure, triggering emergency warnings.
- **Sniffing Function**: Listens to TPMS signals in the 433MHz band, captures sensor IDs and pressure data.

#### Sensor ID Configuration
According to AutoDOS documentation, TPMS sensor IDs for many vehicle models are fixed. For example:
- **BMW X3 Front Left Tire**: All vehicles share the same sensor ID.
- Requires configuring the correct sensor ID based on the target vehicle.

#### Control Commands
Send commands via USB or SPI:

```
Command Format:
[0xFA][0x30][0x05][Command][Tire Position][Parameters...]

Tire Positions:
0x00: Front Left (FL)
0x01: Front Right (FR)
0x02: Rear Left (RL)
0x03: Rear Right (RR)

Command List:
0x01: Set Sensor ID
  Parameter: Sensor ID (4 bytes, big-endian)

0x02: Spoof Tire Pressure
  Parameter: Pressure High Byte, Pressure Low Byte, Temperature

0x03: Low Tire Pressure Warning
  Parameter: None

0x04: High Tire Pressure Warning
  Parameter: None

0x05: Tire Burst Simulation
  Parameter: None

0x06: Start Sniffing Mode
  Parameter: None
```

#### Usage Example
```c
// Set front left tire sensor ID (BMW X3 example)
uint8_t cmd_set_id[] = {
    0xFA, 0x30, 0x05, 0x01, 0x00,  // Command and position
    0x12, 0x34, 0x56, 0x78  // Sensor ID (needs to be configured based on the actual vehicle)
};

// Spoof front left tire pressure to 150kPa, temperature 25°C
uint8_t cmd_spoof[] = {
    0xFA, 0x30, 0x05, 0x02, 0x00,  // Command and position
    0x00, 0x96,  // 150kPa = 0x0096
    0x19  // 25°C
};

// Simulate low tire pressure warning for front left tire
uint8_t cmd_low[] = {0xFA, 0x30, 0x05, 0x03, 0x00};

// Simulate tire burst for front left tire
uint8_t cmd_burst[] = {0xFA, 0x30, 0x05, 0x05, 0x00};

// Start sniffing mode
uint8_t cmd_sniff[] = {0xFA, 0x30, 0x05, 0x06, 0x00};
```

## 4. Communication Protocols

### 4.1 USB Virtual COM Port

- **Baud Rate**: N/A (USB virtual COM port)
- **Data Format**: 8 data bits, no parity, 1 stop bit
- **Purpose**: Communication with PC

### 4.2 Data Packet Formats

#### CAN Data Packet Output Format
```
[0xFA][0x51][0x00][Timestamp (4 bytes)][CAN ID (4 bytes)][Data Length (1 byte)]
[Data (variable length)][0x52][0xFB]
```

#### Command Data Packet Format
```
[Length High Byte][Length Low Byte][0x00][0x00][Data...]
```

## 5. Usage Workflow

### 5.1 Basic Usage Workflow

1. **Hardware Connection**
   - Connect CAN buses to FDCAN1 and FDCAN2.
   - Connect ultrasonic transducer to TIM1 output.
   - Connect echo detection circuit to ADC1.
   - Connect USB to PC.

2. **Firmware Programming**
   - Use ST-Link or J-Link to download the firmware.
   - Ensure both CM7 and CM4 cores are correctly programmed.

3. **System Startup**
   - After power-on, LED2 should blink, indicating system operation.
   - LED1 lights up when USB is connected.

4. **Configure Attack Parameters**
   - Send configuration commands via USB.
   - Configure CAN tampering rules.
   - Configure attack modes.

5. **Start Attack**
   - Send attack start commands.
   - Observe attack effects.

### 5.2 Typical Application Scenarios

#### Scenario 1: Ultrasonic Radar Interference
```
1. Send command to start interference mode.
2. Ultrasonic transducer continuously outputs 48kHz signals.
3. Interfere with the vehicle's ultrasonic radar normal operation.
```

#### Scenario 2: Ultrasonic Radar Spoofing
```
1. Configure spoofing delay distance (e.g., 30cm).
2. Send command to start spoofing mode.
3. System detects radar transmission signals.
4. Transmits false echoes after delay.
5. Radar detects false distance.
```

#### Scenario 3: Millimeter-Wave Radar Attack
```
1. Configure CAN tampering rules to identify millimeter-wave radar messages.
2. Enable millimeter-wave radar attack, set false distance and targets.
3. CAN man-in-the-middle automatically modifies and forwards messages.
4. Vehicle ECU receives false radar data.
```

#### Scenario 4: DSI3 Bus Attack
```
1. Start DSI3 sniffing mode to analyze vehicle DSI3 protocols.
2. Start DSI3 spoofing mode to set false distance values.
3. Vehicle ultrasonic radar receives false data.
4. Affect judgments for functions like automatic parking.
```

#### Scenario 5: TPMS Tire Pressure Attack
```
1. Configure TPMS sensor ID for the target vehicle.
2. Send low tire pressure or tire burst signals.
3. Vehicle ECU receives abnormal signals.
4. Immediately exit autonomous driving mode.
5. Trigger emergency stopping.
```

#### Scenario 6: Combined Attack
```
1. Simultaneously enable ultrasonic interference, millimeter-wave radar attack, DSI3 interference, and TPMS attack.
2. Implement multi-sensor coordinated attacks.
3. Test ADAS system's multi-sensor fusion capabilities.
```

## 6. Debugging and Troubleshooting

### 6.1 LED Status Indicators

- **LED1**: USB connection status.
- **LED2**: System operation status (should blink when normal).

### 6.2 Common Issues

1. **CAN Messages Not Forwarding**
   - Check CAN bus connections.
   - Check termination resistors.
   - Check baud rate configuration.

2. **No Ultrasonic Output**
   - Check TIM1 configuration.
   - Check PWM output pin connections.
   - Check driver circuit.

3. **Echo Detection Not Working**
   - Check ADC configuration.
   - Check echo detection circuit.
   - Adjust detection thresholds.

4. **USB Communication Failure**
   - Check USB connection.
   - Check driver installation.
   - Check baud rate settings.

5. **DSI3 Not Working**
   - Check SPI6 connections.
   - Check DSI3 conversion chip.
   - Check CS pin configuration.
   - Verify SPI communication timing.

6. **TPMS Transmission Failure**
   - Check SPI3 connections.
   - Check CC1101 chip.
   - Check antenna connection.
   - Verify 433MHz frequency configuration.
   - Confirm sensor ID is correctly configured.

## 7. Security Precautions

⚠️ **IMPORTANT WARNINGS**:

1. This firmware is intended for security research and testing purposes only.
2. Must not be used for illegal attacks on real vehicles.
3. Testing should be conducted in controlled environments.
4. Comply with local laws and regulations.
5. Ensure the vehicle is in a safe state before testing.

## 8. Firmware Version Information

- **Version**: 1.0.0
- **Release Date**: 2023
- **Supported Platform**: STM32H745ZITx
- **Development Environment**: STM32CubeIDE / Keil MDK

## 9. Technical Support

For issues, please refer to:
- Hardware Interface Documentation
- STM32H7 Reference Manual
- CAN Protocol Specifications
- Ultrasonic Radar Technical Documentation

## 10. Update Log

### v1.0.0 (2023)
- Initial version.
- Implemented CAN man-in-the-middle functionality.
- Implemented ultrasonic radar attack (interference and spoofing).
- Implemented millimeter-wave radar attack.
- Implemented DSI3 bus control (sniffing, spoofing, interference).
- Implemented TPMS tire pressure attack (using CC1101).
- Supports USB and SPI communication.