# 2209_control - 5-Axis Stepper & Closed-Loop Encoder Notes

ESP-IDF (v6.0) project for the ESP32-S3 driving a 5-DOF robotic arm. Controls 5 Trinamic TMC2209 stepper drivers and reads 5 AS5600 12-bit magnetic encoders through a TCA9548A I2C multiplexer.

---

### Quick Project Summary

* **5 Motors**:
  * Motor 0: 60mm Bipolar 2A NEMA 17 for the waist/base (`IRUN=25`, ~1.44A RMS).
  * Motors 1 to 4: 17HS3401S NEMA 17s for the arm joints (`IRUN=16`, ~0.94A RMS).
* **Dual UART Buses**:
  * TMC2209 drivers only have 4 unique hardware addresses (`0..3`), so we split them across two separate hardware UART ports:
    * **Bus 0 (UART1)**: Motors 0, 1, 2, 3 (addresses 0 to 3).
    * **Bus 1 (UART2)**: Motor 4 (address 0).
* **Encoders (AS5600)**:
  * Mounted directly on the **rear motor shafts** (before the 20:1 gear reduction), giving 4,096 counts per motor rev (81,920 counts per joint output turn).
  * All 5 sensors share address `0x36`, so they run through channels 0 to 4 of a TCA9548A multiplexer.
* **Movement Engines**:
  * Step/Dir pulses generated in software using an ESP32 periodic timer (`esp_timer`) running at 50 µs tick resolution with acceleration ramping.
  * Direct UART velocity mode (`VACTUAL`) for homing and independent axis moves.
* **Safety & Sensorless Detection**:
  * StallGuard4 active in StealthChop mode. Stall checks are gated so they don't false-trigger at standstill (where back-EMF is 0).
* **Web OTA Updates**:
  * Built-in Wi-Fi access point (`ESP32-Robot-OTA`) with a web page for flashing `.bin` updates directly in a browser.

---

### Hardware Pinout & Wiring

#### Motor Addresses & UART Bus Setup
Addresses are configured with jumpers on `MS1` and `MS2`:

| Joint / Motor | Hardware Role | Motor Model | Bus | Address | MS1 | MS2 | Run Current (IRUN) |
| :--- | :--- | :--- | :---: | :---: | :---: | :---: | :--- |
| **Motor 0** | Base / Waist | 60mm 2.0A NEMA 17 | Bus 0 (UART1) | 0 | GND | GND | 25 (~1.44A RMS) |
| **Motor 1** | Shoulder | 17HS3401S NEMA 17 | Bus 0 (UART1) | 3 | 3.3V (VIO) | 3.3V (VIO) | 16 (~0.94A RMS) |
| **Motor 2** | Elbow | 17HS3401S NEMA 17 | Bus 0 (UART1) | 2 | GND | 3.3V (VIO) | 16 (~0.94A RMS) |
| **Motor 3** | Wrist Pitch | 17HS3401S NEMA 17 | Bus 0 (UART1) | 1 | 3.3V (VIO) | GND | 16 (~0.94A RMS) |
| **Motor 4** | Wrist Roll | 17HS3401S NEMA 17 | Bus 1 (UART2) | 0 | GND | GND | 16 (~0.94A RMS) |

#### ESP32-S3 Pin Mapping
* **Enable (EN)**: `GPIO 15` (Shared across all 5 drivers, active LOW).
* **Speed Potentiometer**: `GPIO 4` (ADC1 Channel 3, includes internal pull-down and 5% deadband).

| Motor | STEP Pin | DIR Pin |
| :--- | :--- | :--- |
| **Motor 0** | GPIO 42 | GPIO 41 |
| **Motor 1** | GPIO 40 | GPIO 39 |
| **Motor 2** | GPIO 38 | GPIO 37 |
| **Motor 3** | GPIO 36 | GPIO 35 |
| **Motor 4** | GPIO 48 | GPIO 47 |

* **TMC Bus 0 (UART1)**: TX = `GPIO 2` (through 1kΩ resistor to driver RX), RX = `GPIO 1`.
* **TMC Bus 1 (UART2)**: TX = `GPIO 12` (through 1kΩ resistor to driver RX), RX = `GPIO 11`.
* **I2C Multiplexer (TCA9548A)**: SDA = `GPIO 17`, SCL = `GPIO 18` (pull-ups enabled).

#### TCA9548A Channels to AS5600 Encoders
Encoders sit on the motor shafts (4096 counts/rev) feeding 20:1 gearboxes:

| Motor | TCA9548A Mux Channel | I2C Addr | Gearbox Ratio | Resolution at Output Shaft |
| :--- | :--- | :---: | :---: | :--- |
| **Motor 0 (Base)** | Channel 0 | 0x36 | 20:1 | 81,920 counts/rev (~0.0044°/count) |
| **Motor 1 (Shoulder)** | Channel 1 | 0x36 | 20:1 | 81,920 counts/rev (~0.0044°/count) |
| **Motor 2 (Elbow)** | Channel 2 | 0x36 | 20:1 | 81,920 counts/rev (~0.0044°/count) |
| **Motor 3 (Wrist P)** | Channel 3 | 0x36 | 20:1 | 81,920 counts/rev (~0.0044°/count) |
| **Motor 4 (Wrist R)** | Channel 4 | 0x36 | 20:1 | 81,920 counts/rev (~0.0044°/count) |

---

### Key Software Notes

* **Trinamic Single-Wire UART**:
  * Half-duplex communication with an 8-bit CRC polynomial ($x^8 + x^2 + x + 1$, seed `0x07`).
  * Drivers are automatically scanned on boot. If a driver drops out or is plugged in late, the loop keeps probing every second.
* **StallGuard4 & Back-EMF Gating**:
  * StallGuard measures back-EMF in StealthChop mode.
  * When a motor is stopped or moving very slowly (<400 Hz), back-EMF is naturally near zero, so `SG_RESULT` reads 0. The code gates checks so standstill does not trigger false stall errors.
* **Timer Step Engine**:
  * Runs on a 50 µs tick timer.
  * Accelerates smoothly (+2 Hz per 50 µs tick = ~40,000 Hz/s²) to prevent steppers from losing sync on sudden speed changes.
* **Encoder Unwrapping**:
  * AS5600 angle registers (`0x0E`/`0x0F`) are polled over the multiplexer.
  * Crossings across the 0/4096 boundary are tracked continuously to maintain total revolutions and joint degrees.
  * Discrepancies between expected step position and encoder angle show up in telemetry as tracking error.

---

### Project File Layout

```text
2209_control/
├── main/
│   ├── main.c           # Speed pot ADC, step timer ISR, main telemetry loop
│   ├── tmc2209.c        # Dual UART bus driver, CRC8, StallGuard, current setup
│   ├── tmc2209.h        # Driver registers, structs, and function prototypes
│   ├── as5600.c         # TCA9548A multiplexer control, angle tracking & unwrap math
│   ├── as5600.h         # AS5600 registers and encoder structures
│   ├── ota_update.c     # SoftAP, web portal UI, partition flashing, motor interlock
│   ├── ota_update.h     # OTA config and network headers
│   └── CMakeLists.txt   # Main component build sources
├── scripts/
│   └── ota_flash.py     # Python script to push updates over Wi-Fi
├── partitions.csv       # Flash map (two 4MB OTA app partitions)
├── sdkconfig            # ESP-IDF settings for ESP32-S3 N16R8 (16MB flash)
└── CMakeLists.txt       # Root project file
```

---

### Flashing & Working with the Board

#### 1. First-Time Flash (USB Cable)
Because of the custom partition table (`partitions.csv`), the very first upload must be done over USB so the flash memory gets partitioned:

```powershell
# Build the project
idf.py build

# Flash bootloader, partition table, and app (replace COM4 with your port)
idf.py -p COM4 flash monitor
```

#### 2. Wireless Updates (Over Wi-Fi)
Once the board has the OTA firmware running, you don't need the USB cable anymore:

1. Connect your computer or phone to the Wi-Fi network:
   * **SSID**: `ESP32-Robot-OTA`
   * **Password**: `robotics2026`
2. Open your browser and go to `http://192.168.4.1`.
3. Drop `build/2209_control.bin` into the page and hit flash.
4. The ESP32 disables motor power stages while writing flash, verifies the image checksum, and reboots into the new partition.

Alternatively, flash directly from a terminal without opening a browser:
```powershell
python scripts/ota_flash.py build/2209_control.bin --ip 192.168.4.1
```
