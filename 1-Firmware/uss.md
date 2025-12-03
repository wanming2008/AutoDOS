# Ultrasonic Transmitter Hardware Description

## 1. Hardware for Transmitting PWM Signals

### STM32 Timer (TIM1)
- **Hardware**: STM32H7's TIM1 timer
- **Function**: Generates 48kHz PWM signals
- **Output Pins**:
  - **TIM1_CH1**: PA8 (positive output)
  - **TIM1_CH2**: PA9 (negative output, complementary)
- **Output Mode**: Complementary PWM (push-pull output)
- **Frequency**: 48 kHz
- **Duty Cycle**: 50%

### Working Principle
```
STM32 TIM1 Timer
    ↓ (Generates 48kHz PWM signal)
PA8 (TIM1_CH1) ──┐
                 ├──→ Driver Circuit ──→ Ultrasonic Transducer ──→ Transmits 48kHz ultrasonic waves
PA9 (TIM1_CH2) ──┘ (Complementary PWM, push-pull output)
```

## 2. Ultrasonic Transducer

### What is an Ultrasonic Transducer?
- **Definition**: A device that converts electrical signals into ultrasonic signals
- **Type**: Piezoelectric ceramic ultrasonic transducer
- **Operating Frequency**: 40-48 kHz (this system uses 48 kHz)
- **Functions**:
  - Receives PWM electrical signals
  - Vibrates to generate ultrasonic waves
  - Transmits ultrasonic waves into the air

### Transducer Connections
```
TIM1_CH1 (PA8) ──→ Transducer positive terminal
TIM1_CH2 (PA9) ──→ Transducer negative terminal
```

## 3. Driver Circuit

### Why is a Driver Circuit Needed?
STM32's GPIO output current is limited (typically <25mA), while ultrasonic transducers require higher driving power. Therefore, a driver circuit is necessary.

### Driver Circuit Solutions

#### Option 1: H-Bridge Driver (Recommended)
```
TIM1_CH1 ──→ H-Bridge Driver Chip (e.g., DRV8833) ──→ Transducer
TIM1_CH2 ──→ H-Bridge Driver Chip
```
- **Advantages**: High power, high efficiency, supports bidirectional driving
- **Applicability**: High-power ultrasonic transducers

#### Option 2: Push-Pull Amplifier
```
TIM1_CH1 ──→ Push-Pull Amplifier ──→ Transducer
TIM1_CH2 ──→ Push-Pull Amplifier
```
- **Advantages**: Simple circuit
- **Applicability**: Medium/low-power transducers

#### Option 3: Dedicated Ultrasonic Driver Chip
- Use specialized ultrasonic driver ICs
- High integration, good performance

## 4. Complete Signal Chain

```
STM32 TIM1 Timer
    ↓
Generates 48kHz PWM signal (complementary output)
    ↓
PA8 (TIM1_CH1) ──┐
                 ├──→ Driver Circuit (H-bridge/Push-pull amplifier)
PA9 (TIM1_CH2) ──┘
    ↓
Driver circuit amplifies PWM signal
    ↓
Ultrasonic Transducer (piezoelectric ceramic)
    ↓
Transducer vibrates to generate 48kHz ultrasonic waves
    ↓
Ultrasonic waves propagate through air
```

## 5. Two Operating Modes

### Mode 1: Interference Mode (Jamming)
```
TIM1 continuously outputs 48kHz PWM
    ↓
Driver circuit continuously drives
    ↓
Transducer continuously transmits 48kHz ultrasonic waves
    ↓
Interferes with normal operation of vehicle's ultrasonic radar
```

### Mode 2: Spoofing Mode (Spoof)
```
1. ADC detects vehicle radar's transmitted ultrasonic waves (via echo detection circuit)
2. After detecting trigger signal
3. Delays for a specific time (corresponding to false distance)
4. TIM1 outputs 60 cycles of 48kHz PWM pulses
5. Driver circuit drives the transducer
6. Transducer transmits false echoes
7. Vehicle radar receives false echoes, misjudging distance
```

## 6. Hardware Components Summary

| Component | Function | Location |
|---|---|---|
| **STM32 TIM1** | Generates 48kHz PWM signal | Inside STM32H7 chip |
| **PA8/PA9** | PWM output pins | STM32 GPIO |
| **Driver Circuit** | Amplifies PWM signal power | External circuit board |
| **Ultrasonic Transducer** | Converts electrical signals to ultrasonic waves | External device |

## 7. Key Parameters

### PWM Parameters
- **Frequency**: 48 kHz (matches ultrasonic radar operating frequency)
- **Duty Cycle**: 50%
- **Output Mode**: Complementary PWM (push-pull)
- **Voltage**: 3.3V (STM32 output)

### Driver Circuit Parameters
- **Input**: 3.3V PWM signal
- **Output**: According to transducer requirements (typically 5-12V)
- **Power**: According to transducer specifications (typically a few watts to tens of watts)

### Transducer Parameters
- **Operating Frequency**: 48 kHz
- **Impedance**: Typically several hundred to several thousand ohms
- **Power**: Select according to application requirements

## 8. Important Notes

1. **Driver Circuit Must**:
   - Provide sufficient driving power
   - Support 48 kHz frequency response
   - Have good linearity

2. **Transducer Selection**:
   - Operating frequency must match 48 kHz
   - Power must meet application requirements
   - Directionality should match application scenario

3. **PCB Layout**:
   - Keep PWM signal traces as short as possible
   - Avoid interference
   - Place driver circuit close to the transducer

4. **Power Supply**:
   - Driver circuit requires an independent power supply
   - Ensure stable power supply
   - Consider power consumption

## 9. Summary

**What transmits PWM**: STM32's TIM1 timer (hardware timer)

**What transmits ultrasonic waves**: Ultrasonic transducer (piezoelectric ceramic device)

**Intermediate stage**: Driver circuit (amplifies PWM signal, drives transducer)

The complete system is: **STM32 Timer → Driver Circuit → Ultrasonic Transducer → Ultrasonic Waves**