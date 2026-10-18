# ESP32-S3 N16R8 Wi-Fi Object Detection Test Rig (ESP-IDF)

This repository contains a two-stage bring-up test system to validate Wi-Fi video streaming, JPEG hardware decompression, on-device object detection, and latency telemetry on an **ESP32-S3 N16R8** board (16MB Flash, 8MB Octal PSRAM).

In this test rig, a PC webcam stands in for the future on-board camera (e.g. OV3660/OV2640). The PC resizes and compresses webcam frames into JPEG images, streams them over Wi-Fi to the ESP32-S3, and receives real-time detection bounding boxes and profiling data.

---

## Technical Architecture & Decisions

### 1. Transport Protocol Choice: TCP vs UDP
* **Selection**: **TCP Socket** (with `TCP_NODELAY` enabled).
* **Rationale**:
  * JPEG compression is highly sensitive to packet drop; missing even a single packet results in severe image tearing or decompression failure.
  * Over a local 802.11n Wi-Fi network, TCP overhead is negligible (< 3-5 ms latency for a 15-30 KB frame).
  * Disabling Nagle's algorithm (`TCP_NODELAY`) on both Python and ESP-IDF sockets ensures packet headers and frame data are flushed immediately without buffering delay.

### 2. Object Detection Framework: ESP-DL vs TensorFlow Lite Micro
* **Selection**: **ESP-DL** (Espressif Deep Learning Framework).
* **Rationale**:
  * The ESP32-S3 Xtensa LX7 dual-core processor features **Processor Instruction Extensions (PIE)** — vector/SIMD instructions specifically tailored for 8-bit / 16-bit quantized neural network operations (dot products, convolutions).
  * ESP-DL explicitly leverages these vector instructions, yielding a 3x–6x acceleration over generic TensorFlow Lite Micro implementations that lack ESP32-S3 SIMD optimizations.
  * The provided detector component defaults to an optimized fast feature centroid/contour detector for instant bring-up testing and seamlessly interfaces with ESP-DL detection model wrappers (`dl::detect`).

### 3. JPEG Decoding Strategy
* **Library**: `tjpgd` (Tiny JPEG Decompressor — built into ESP-IDF core components across v4.x, v5.x, and v6.x).
* **Performance**: At 320x240 resolution, decompressing a 75% quality JPEG to RGB888 takes **10–18 ms** on ESP32-S3 running at 240 MHz.

### 4. Memory Management Plan (8MB Octal PSRAM + 512KB SRAM)
| Memory Region | Allocation / Use Case | Description |
| :--- | :--- | :--- |
| **Internal SRAM** | Wi-Fi / LwIP Buffers, FreeRTOS Stacks | High-speed DMA-capable RAM used for OS tasks, interrupt handlers, and network socket rx/tx buffers. |
| **Octal PSRAM** | JPEG RX Buffer (`~64 KB`) | Reusable incoming network frame buffer allocated via `heap_caps_malloc(..., MALLOC_CAP_SPIRAM)`. |
| **Octal PSRAM** | Decoded RGB888 Buffer (`~230 KB`) | Decoded uncompressed frame memory (`320 * 240 * 3`). |
| **Octal PSRAM** | Detector Arena / Model Weights | Model tensors and tensor arena residing in high-capacity 80MHz Octal PSRAM. |

### 5. Wire Protocol Format (`protocol.h`)
All multi-byte integer fields are Little-Endian. Structs are packed (`#pragma pack(push, 1)`).

#### Request Header (PC -> ESP32-S3) `frame_header_t` (20 bytes)
```c
typedef struct {
    uint32_t magic;         // 0x53334644 ("S3FD")
    uint16_t frame_id;      // Incremental frame index
    uint16_t stage_mode;    // 1 = Stage 1 (results only), 2 = Stage 2 (results + echo frame)
    uint32_t image_width;   // Frame width (e.g. 320)
    uint32_t image_height;  // Frame height (e.g. 240)
    uint32_t payload_len;   // JPEG payload size in bytes
} frame_header_t;
```

#### Response Header (ESP32-S3 -> PC) `response_header_t` (32 bytes)
```c
typedef struct {
    uint32_t magic;            // 0x53335245 ("S3RE")
    uint16_t frame_id;         // Echoed frame index
    uint16_t stage_mode;       // Mode executed (1 or 2)
    uint16_t num_detections;   // Count of detection_box_t structures following
    uint32_t echoed_frame_len; // Length of JPEG echoed back (0 in Stage 1)
    latency_report_t timing;   // Microsecond timing breakdown (Recv, Decode, Infer, Send)
} response_header_t;
```

#### Bounding Box Payload `detection_box_t` (14 bytes each)
```c
typedef struct {
    uint16_t class_id;      // Object Class (0 = Target/Person, 1 = Face, 2 = Object)
    float score;            // Confidence Score (0.00 - 1.00)
    uint16_t x, y, w, h;    // Bounding Box coordinates in pixels
} detection_box_t;
```

### 6. Frame Rate Target & Bottleneck Analysis
* **Target FPS**: **8 – 12 FPS** at 320x240 resolution.
* **Latency & Bottleneck Breakdown**:
  * **Wi-Fi Transmission (PC -> ESP32)**: `~8 - 15 ms` (15-25 KB payload over Wi-Fi 4)
  * **JPEG Decode**: `~12 - 18 ms` (ESP32-S3 CPU @ 240MHz)
  * **On-Device Inference**: `~25 - 45 ms` (ESP-DL vector acceleration)
  * **Wi-Fi Transmit Response**: `~2 - 5 ms` (Stage 1) / `~10 - 20 ms` (Stage 2)
  * **Total ESP Processing Time**: `~50 - 80 ms` per frame.

---

## Two-Stage Build & Operational Modes

### Stage 1 — Detection Results Only (Low Latency / Low Bandwidth)
* **Pipeline**: `PC webcam -> JPEG -> TCP -> ESP32-S3 -> Decode -> Detect -> Bounding Boxes -> TCP -> PC`
* **Network Overhead**: Minimal (~32 bytes response header + ~14 bytes per detection).
* **Use Case**: Maximum streaming performance and minimal Wi-Fi bandwidth utilization.

### Stage 2 — Echo Frame Back
* **Pipeline**: `PC webcam -> JPEG -> TCP -> ESP32-S3 -> Decode -> Detect -> [Bounding Boxes + Frame Echo] -> TCP -> PC`
* **Network Overhead**: Returns full JPEG image back to PC over Wi-Fi.
* **Use Case**: Sanity check return path bandwidth, verify frame integrity on ESP32-S3 before deploying embedded camera sensors.

> **Dynamic Stage Switch**: Press key **`1`** or **`2`** on the PC Python display window to switch between Stage 1 and Stage 2 in real time!

---

## Firmware Setup & Build Instructions

### Prerequisites
* **Framework**: **ESP-IDF v5.1 / v5.2 / v5.3** (Targeting ESP-IDF v5.x)
* **Hardware**: ESP32-S3 DevKitC-1 N16R8 module (or equivalent ESP32-S3 with Octal PSRAM)

### Step 1: Set Target to ESP32-S3
Open the ESP-IDF Command Prompt or terminal and run:
```bash
idf.py set-target esp32s3
```

### Step 2: Configure Wi-Fi Credentials & Port
Launch menuconfig:
```bash
idf.py menuconfig
```
Navigate to:
`Wi-Fi & Object Detection Rig Configuration`
* Set **Wi-Fi SSID** to your local network SSID.
* Set **Wi-Fi Password** to your network password.
* Set **TCP Server Port** (default: `8888`).

### Step 3: Build Firmware
```bash
idf.py build
```

### Step 4: Flash and Monitor Serial Log
Connect your ESP32-S3 board via USB and run:
```bash
idf.py -p COMx flash monitor
```
*(Replace `COMx` with your actual serial port, e.g. `COM3` on Windows or `/dev/ttyUSB0` on Linux).*

Upon booting, the serial console will output:
```text
I (1200) WIFI_STA: Connected successfully! Station IP Address: 192.168.1.105
I (1210) MAIN: [4/4] Starting TCP Streaming Server...
I (1220) NET_SERVER: TCP Server listening on port 8888...
```
Note the **Station IP Address** printed in the serial log.

---

## PC Client Setup & Run Instructions

### Step 1: Install Python Dependencies
```bash
cd pc_client
pip install -r requirements.txt
```

### Step 2: Configure & Run Script
Open `pc_client/pc_test_rig.py` and update the `ESP32_IP` constant with the IP address printed in your serial monitor:
```python
ESP32_IP = "192.168.1.105"  # Match your ESP32 IP
```

Run the PC test script:
```bash
python pc_test_rig.py
```

---

## What "Working" Looks Like

1. **PC Display Window**:
   * A live webcam window titled **"ESP32-S3 Wi-Fi Object Detection Monitor"** opens.
   * A top-left HUD panel displays:
     * Current Stage Mode (`Stage 1` or `Stage 2`)
     * Real-time FPS and frame resolution (`320x240`)
     * Round-Trip Latency (RTT) in milliseconds
     * Detailed ESP32 execution timings (`Recv`, `Decode`, `Inference`, `Send`)
   * Green bounding boxes with labels (e.g. `Target/Person: 88.5%`) surround detected objects/faces.

2. **ESP32 Serial Log Output**:
   * For every processed frame, the ESP32 serial console logs performance breakdown:
     ```text
     I (4530) NET_SERVER: Frame #142 [Stage 1] | Recv: 9.20ms | Dec: 14.10ms | Infer: 28.50ms | Send: 1.80ms | Total: 53.60ms | Detections: 1
     ```

---

## Next Steps & Roadmap
1. **Custom Model Deployment**: Replace the default feature detector with a custom trained 8-bit quantized ESP-DL / TFLite Micro model.
2. **Physical Camera Hardware Integration**: Wire an **OV3660** or **OV2640** DVP camera directly to the ESP32-S3 GPIO pins using `esp_camera`, swapping the TCP network input with the camera frame buffer pipeline.
