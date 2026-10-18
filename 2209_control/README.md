# TMC2209 Multi-Motor Stepper Controller (`2209_control`)

An **ESP-IDF (v6.0)** embedded C firmware project developed for the **ESP32-S3** microcontroller to configure, control, and monitor **up to 4 Trinamic TMC2209** stepper motor drivers simultaneously over a shared half-duplex UART bus.

---

## 📌 Project Overview

This project provides real-time multi-axis stepper motion control and UART hardware management for TMC2209 silent stepper motor drivers. It features:
* **Simultaneous 4-Motor Control**: Addresses `0`, `1`, `2`, and `3` (selected via `MS1` and `MS2` pins on each driver) running concurrently on a single-wire UART bus.
* **Dual Motion Engines**:
  * **Hardware STEP/DIR**: Low-jitter periodic multi-motor pulse generation via ESP-IDF high-resolution timer (`esp_timer`).
  * **Direct UART Velocity (`VACTUAL`)**: Silicon-level autonomous step generation driven over UART commands.
* **StallGuard4™ & Sensorless Load Telemetry**: Read mechanical load (`SG_RESULT`), configure sensitivity thresholds (`SGTHRS`), and execute sensorless homing.
* **CoolStep™ Dynamic Current Control**: Automatically reduces energy consumption and motor heating under light loads.
* **Interactive Speed Regulation**: Potentiometer ADC input mapped dynamically to all active joints.
* **Extensible Robotics Hooks**: Ready-to-use dummy and callback APIs for inverse kinematics, 4-DOF trajectory streaming, and joint safety interlocks.

---

## 🛠 Hardware & Pin Configuration

All 4 TMC2209 drivers share the single-wire UART communication lines (half-duplex). Each driver is assigned an individual address via its `MS1` and `MS2` pins.

### TMC2209 Address Selection
| Motor Index | Device Address | `MS1` Pin | `MS2` Pin | Typical Joint Role |
| :---: | :---: | :---: | :---: | :---: |
| **Motor 0** | `0` (`0b00`) | `GND` | `GND` | Base / Waist |
| **Motor 1** | `1` (`0b01`) | `VIO` (3.3V) | `GND` | Shoulder |
| **Motor 2** | `2` (`0b10`) | `GND` | `VIO` (3.3V) | Elbow |
| **Motor 3** | `3` (`0b11`) | `VIO` (3.3V) | `VIO` (3.3V) | Wrist / End-Effector |

### ESP32-S3 GPIO Mapping
| Function | Motor 0 (Addr 0) | Motor 1 (Addr 1) | Motor 2 (Addr 2) | Motor 3 (Addr 3) |
| :--- | :--- | :--- | :--- | :--- |
| **STEP Pin** | `GPIO_NUM_9` | `GPIO_NUM_5` | `GPIO_NUM_7` | `GPIO_NUM_12` |
| **DIR Pin** | `GPIO_NUM_2` | `GPIO_NUM_6` | `GPIO_NUM_8` | `GPIO_NUM_13` |
| **EN Pin** | `GPIO_NUM_4` *(Shared or dedicated)* | `GPIO_NUM_4` | `GPIO_NUM_4` | `GPIO_NUM_4` |

| Peripheral | ESP32-S3 GPIO | Description |
| :--- | :--- | :--- |
| **UART TX** | `GPIO_NUM_10` | Connected to TMC2209 `PDN_UART` bus (with 1kΩ series resistor) |
| **UART RX** | `GPIO_NUM_11` | Connected to TMC2209 `PDN_UART` bus |
| **POT** | `GPIO_NUM_1` | Analog potentiometer input (`ADC_UNIT_1`, `ADC_CHANNEL_0`) |
| **I2C SDA** | `GPIO_NUM_17` | Connected to TCA9548A `SDA` input pin |
| **I2C SCL** | `GPIO_NUM_18` | Connected to TCA9548A `SCL` input pin |

### TCA9548A I2C Multiplexer & AS5600 Encoder Mapping
| Motor / Joint | TCA9548A Channel | AS5600 Address | Gear Ratio | Output Direction | Total Counts / Output Turn |
| :---: | :---: | :---: | :---: | :---: | :---: |
| **Motor 0 (Base)** | `Channel 0` | `0x36` | `20:1` | Inverted | 81,920 counts ($0.00439^\circ/\text{count}$) |
| **Motor 1 (Shoulder)**| `Channel 1` | `0x36` | `20:1` | Inverted | 81,920 counts ($0.00439^\circ/\text{count}$) |
| **Motor 2 (Elbow)** | `Channel 2` | `0x36` | `20:1` | Inverted | 81,920 counts ($0.00439^\circ/\text{count}$) |
| **Motor 3 (Wrist)** | `Channel 3` | `0x36` | `20:1` | Inverted | 81,920 counts ($0.00439^\circ/\text{count}$) |

---

## ⚙️ Key Specifications & Parameters

* **Target Microcontroller:** ESP32-S3 (ESP-IDF v6.0)
* **UART Baud Rate:** 115,200 baud (8-N-1)
* **Base Step Engine Resolution:** 50 µs tick rate (20 kHz periodic accumulator)
* **Step Pulse Width:** 2 µs (`STEP_PULSE_US`, exceeds TMC2209 100 ns minimum requirement)
* **Dynamic Speed Range:** 100 Hz to 12,000 Hz (`MIN_SPEED_HZ` to `MAX_SPEED_HZ`)
* **ADC Resolution:** 12-bit (0–4095 range) with 12 dB attenuation (`ADC_ATTEN_DB_12`)

---

## 🧩 Firmware Architecture & Modules

### 1. Multi-Device Single-Wire UART & CRC8 Protocol ([`tmc2209.c`](file:///d:/dev/Eduvos/2026%20Year%203/ITPPA3%20-%20Project%20Python%20Robotics/code/2209_control/main/tmc2209.c))
* **CRC8 Calculation (`tmc_calc_crc`)**: Implements Trinamic polynomial $x^8 + x^2 + x^1 + 1$ with seed `0x07`.
* **Register Write (`tmc_write_reg`)**: 8-byte frames: `0x05` + `ADDR (0..3)` + `REG | 0x80` + 4-byte payload + CRC.
* **Register Read (`tmc_read_reg_ex`)**: 4-byte query, extracts 8-byte response frame (`0xFF` address filter + CRC validation).
* **Bus Discovery (`tmc_scan_and_configure_all`)**: Probes all 4 addresses at boot. Automatically initializes detected drivers and provides non-blocking hot-plug re-detection in the main loop.

### 2. StallGuard4™ & Sensorless Homing
* **Sensitivity Threshold (`tmc_stallguard_set_threshold`)**: Sets `REG_SGTHRS` (0..255). Lower values require higher torque to flag stalls; higher values increase sensitivity.
* **Real-Time Load Monitoring (`tmc_stallguard_get_result`)**: Reads `REG_SG_RESULT` (0..510). 0 indicates maximum mechanical load / stall condition.
* **Velocity Gate (`REG_TCOOLTHRS`)**: Enables StallGuard when velocity exceeds threshold (`TSTEP < TCOOLTHRS`).
* **Sensorless Homing Routine (`tmc_sensorless_homing`)**: Moves joint toward physical hard-stop, detects stall event via StallGuard, and stops motor cleanly.

### 3. CoolStep™ Smart Energy
* **Dynamic Current Scaling (`tmc_coolstep_enable`)**: Configures `REG_COOLCONF` (`semin`, `semax`, `seup`, `sedn`, `seimin`) to automatically drop motor current by up to 75% during low-torque phases, reducing heat dissipation.

### 4. Direct UART Motion Mode (`VACTUAL`)
* **Velocity Streaming (`tmc_set_velocity_vactual`)**: Sets `REG_VACTUAL` (signed 24-bit). The driver generates step pulses in internal silicon without requiring CPU step pin pulses.
* **Mode Restore (`tmc_stop_vactual`)**: Writing 0 restores hardware `STEP`/`DIR` control.

### 5. Multi-Motor Step Pulse Engine
* Utilizes a periodic ESP-IDF high-resolution timer (`esp_timer_start_periodic` at 50 µs tick).
* Features microsecond time accumulators per motor, enabling all 4 motors to run at independent or synchronized frequencies without jitter.

### 6. Diagnostics & Telemetry Table
* Decodes `REG_DRV_STATUS` into human-readable flags:
  * `STANDSTILL`: Motor stopped
  * `OTPW` / `OVERTEMP`: Thermal pre-warning & shutdown
  * `S2GA` / `S2GB`: Short to ground Phase A / B
  * `OPEN_A` / `OPEN_B`: Open load Phase A / B
  * `CS`: Active current scale ($0-31$)
  * `SG4`: Live mechanical load metric ($0-510$)

### 7. Robotics Trajectory & Kinematics Hooks
* `robot_arm_joint_move_dummy(joint_id, angle_deg, speed_deg_s)`: Maps joint angular velocity into step pulse rate.
* `robot_arm_stream_trajectory_dummy(...)`: Multi-axis trajectory waypoint executor.
* `robot_arm_stallguard_event_callback(...)`: Safety interlock that disables a stalled joint when an obstacle is struck.
* `robot_arm_sync_home_all_dummy(...)`: Coordinated homing sequence for robotic arms.

---

## 📁 Repository Structure

```text
2209_control/
├── .devcontainer/         # Dev container environment settings
├── .vscode/               # VS Code configuration (ESP-IDF target, OpenOCD, clangd)
├── main/                  # Application source files
│   ├── CMakeLists.txt     # IDF component registration (main.c, tmc2209.c, as5600.c, ota_update.c)
│   ├── ota_update.h       # Dual-mode Wi-Fi & OTA Web Server API definitions
│   ├── ota_update.c       # Embedded HTTP server, Web Portal UI, flash streaming & safety locks
│   ├── as5600.h           # AS5600 12-bit encoder & TCA9548A multiplexer API definitions
│   ├── as5600.c           # I2C driver, 20:1 gearbox kinematics, multi-turn unwrapping
│   ├── tmc2209.h          # Register definitions, structs & full API prototypes
│   ├── tmc2209.c          # Multi-driver UART protocol, StallGuard, CoolStep implementation
│   └── main.c            # Multi-motor coordination, ADC speed input, telemetry loop
├── scripts/
│   └── ota_flash.py       # Python CLI utility for wireless OTA flashing over Wi-Fi
├── partitions.csv         # 16MB Flash partition table (dual 4MB OTA partitions + 7MB storage)
├── CMakeLists.txt         # Root ESP-IDF project definition
├── sdkconfig              # ESP-IDF Kconfig settings (configured for ESP32-S3 N16R8)
└── README.md              # Project summary & documentation
```

---

## 🌐 Wireless Over-The-Air (OTA) Firmware Updates

The firmware features an integrated **Dual-Mode Wi-Fi Over-The-Air (OTA)** update system with an embedded web management portal and motor safety interlocks.

### 1. Wi-Fi Access Point (SoftAP)
* **SSID:** `ESP32-Robot-OTA`
* **Password:** `robotics2026`
* **Portal URL:** `http://192.168.4.1`
*(If station router credentials are provided in `ota_system_init()`, the device also connects to your local network and reports its DHCP IP address on the serial console).*

### 2. Browser-Based Wireless Update
1. Connect your laptop, tablet, or phone to the `ESP32-Robot-OTA` Wi-Fi network.
2. Open your web browser and navigate to **`http://192.168.4.1`**.
3. Drag and drop `build/2209_control.bin` into the upload zone (or click to browse).
4. Click **"Flash Firmware Over-The-Air"**.
5. The web interface will display live transfer progress.
6. **Safety Interlock**: During flashing, the pulse generator is halted and motor driver coils are freewheeled to prevent movement.
7. Upon completion, the ESP32-S3 automatically verifies checksums and reboots into the updated partition!

### 3. Command-Line Wireless Update (CLI)
You can also flash directly from VS Code or PowerShell without opening a web browser:
```powershell
# Flash wirelessly using the Python utility:
python scripts/ota_flash.py build/2209_control.bin --ip 192.168.4.1

# Or flash wirelessly using curl:
curl -X POST --data-binary @build/2209_control.bin http://192.168.4.1/update
```

---

## 🚀 Building and Flashing

### Initial Setup (First Flash over USB)
Because a new custom partition table (`partitions.csv` for 16MB flash) is introduced, the initial setup must be written **once via USB cable** to partition the physical flash memory:
```bash
# 1. Build the firmware
idf.py build

# 2. Flash partition table, bootloader, and firmware over USB (e.g. COM4)
idf.py -p COM4 flash monitor
```

### Subsequent Updates (100% Wireless over Wi-Fi)
After the initial USB flash, all future firmware iterations can be deployed wirelessly:
1. Build code: `ninja -C build` (or `idf.py build`).
2. Upload via Web Portal at `http://192.168.4.1` or via `python scripts/ota_flash.py`.

