# AutoDoS: An Open Source ADAS Security Toolkit 
 
 
AutoDoS is an open-source hardware toolkit dedicated to security research on Level 2 (L2) Advanced Driver Assistance Systems (ADAS). It enables low-cost and user-friendly solutions for attacking and interfering with
various types of in-vehicle sensors. The toolkit supports bus sniffing, signal simulation, and Denial-of-Service (DoS) attacks, aiming to assist security researchers and engineers in evaluating the security and robustness of ADAS in real-world scenarios.
 
## This repository contains the following content:
#### 1.Hardware images of AutoDoS and introductions to its supported modules.
#### 2.Hardware block diagrams, schematic designs, and PCB Gerber files for AutoDoS.
#### 3.Firmware files for AutoDoS.
#### 4.3D printablecase design files for AutoDoS.
## 1.Hardware images of AutoDoS and introductions to its supported modules.
#### hardware images 
Currently, it isthe V1 version, constructed using one mainboard and multiple daughter-module breadboards. The V2 version will integrate all modules onto a single circuit board, housed in the shell shown in the image below, with various interfaces reserved.The circuit diagram for the V1 version is as follows.
<img width="1259" height="813" alt="582324dcb61aeaf0f56b6560a91402ad" src="https://github.com/user-attachments/assets/bfd86c75-c29b-44d9-baf5-9a9a84a72142" />

### Currently supported features:
#### a.Real-vehicle USS frequency detection 
Currently supports detection in the 44kHz–55kHz range.
####  b.USS sniffing and spoofing DoS 
Currently supports detection in the 44kHz–55kHz range.Currently supports spoofing for 2 USS channels, with future support for up to 12 USS channels (matching the number of USS sensors in real vehicles).
#### c.mmWave MITM(Man-in-the-Middle) 
Currently supports MITM over CAN FD, enabling sniffing, data parsing, and injection of malicious commands (e.g., incorrecttimestamps, false distance data).
#### d.GNSS reference io control and time synchronization.
#### e.TPMS tire pressure DoS and spoofing 
Supports 433MHz and 315Khz frequency.
#### f.We will support lidar laser Dos in the future.

 
## 2. Hardware block diagram and schematic of AutoDoS 
The block diagram is as follows. It is currently the schematic of version V1. The V2 version will be uploaded in 2 days. The V2 version will integrate all the external modules in the image onto a single printed circuit board and place them inside the metal CNC housing of AutoDo
![fffd554b858e57654d01dfdc1a4bea44](https://github.com/user-attachments/assets/61aabead-8c54-4c5f-8a3d-3c78a23dd4d9)

The green part mean already supported  The red pard mean will suppoet in the future release V3
## 3. AutoDoS firmware file
Firmware for ADAS (Advanced Driver Assistance System) security testing tool based on the STM32H7 dual-core architecture. This project integrates CAN man-in-the-middle, ultrasonic radar attack, millimeter-wave
radar attack, TPMS tire pressure monitoring system attack, and more. Features
#### 3.1 CAN Man-in-the-Middle Function
a. Bidirectional forwarding of CAN messages between FDCAN1 and FDCAN2
b. Supports real-time modification of CAN message ID and data
c. Supports configuration of 7 independent tampering rules
d. Supports CAN 2.0 and CANFD protocols
#### 3.2 Ultrasonic Radar Attack
a. Jamming Mode: Continuously emits 48kHz ultrasonic signals to interfere with ultrasonic radar
b. Spoofing Mode: Detects radar transmitted signals and delays the emission of false echoes to achieve distance spoofing
c. Spoofing delay distance is configurable
#### 3.3 Millimeter-Wave Radar Attack
a. Identifies millimeter-wave radar CAN messages
b. Modifies key data such as distance and number of targets
c. Forwards modified messages via CAN man-in-the-middle function
#### 3.4 DSI3 Bus Control
a. Controls DSI3 conversion chip via SPI
b. Supports DSI3 bus sniffing, spoofing, and jamming
c. Supports up to 4 DSI3 sensor channels
d. Implements ultrasonic radar DSI3 protocol attack
#### 3.5 TPMS Tire Pressure Monitoring System Attack
a. Simulates 433MHz or 315Mhz signals using TI CC1101 chip
b. Supports tire pressure spoofing (low pressure, high pressure, flat tire)
c. Supports 4 tire positions
d. Supports TPMS signal sniffing
 
## 4. AutoDoS cnc case 3D print file
You can create a casing and laser engrave your own preferred design patterns.
## 5 AutoDOS pictures
#### AutoDOS for USS
<img width="1280" height="720" alt="IMG_8349" src="https://github.com/user-attachments/assets/bdeaea02-a164-4448-960b-482399ce1374" />
AutoDOS for mmWave mitm
<img width="1280" height="720" alt="07b7ffc990f73971d85833cd4a3430e6" src="https://github.com/user-attachments/assets/ac9283c5-49ca-4b43-a3be-a42b21959b29" />

#### AutoDOS for tpms
<img width="871" height="556" alt="58ea90d8d2b12cea81be773297c32c55" src="https://github.com/user-attachments/assets/90b04dec-4282-4c5d-ad31-01ac61595741" />

# AutoDOS
AutoDOS
