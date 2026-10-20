#!/usr/bin/env python3
"""
ESP32-S3 Wi-Fi Object Detection Test Rig - PC Client
===================================================
Captures PC webcam, streams JPEG frames over TCP to ESP32-S3,
receives object detection bounding boxes (Stage 1) or bounding boxes + echoed frame (Stage 2),
and renders live HUD with latency breakdown and FPS statistics.
"""

import socket
import struct
import time
import sys
import cv2
import argparse
if sys.platform.startswith('win'):
    import msvcrt
import threading

# ==============================================================================
# CONFIGURATION SECTION (Defaults can be overridden via CLI args)
# ==============================================================================
ESP32_IP = "192.168.1.65"    # Match your ESP32-S3 assigned IP (e.g. 192.168.1.65)
ESP32_PORT = 8888            # TCP Server Port (configured in ESP32 Kconfig)
STAGE_MODE = 1               # 1 = Stage 1 (results only), 2 = Stage 2 (results + echoed frame)
CAMERA_INDEX = 0             # PC Webcam Index
FRAME_WIDTH = 320            # Frame width sent to ESP32-S3
FRAME_HEIGHT = 240           # Frame height sent to ESP32-S3
JPEG_QUALITY = 50            # Optimized compression quality for low Wi-Fi latency
TARGET_FPS = 10              # Streaming rate limit (FPS)
RECONNECT_DELAY_SEC = 2.0   # Retry delay when connecting
# ==============================================================================

# Protocol constants matching protocol.h
FRAME_HEADER_MAGIC = 0x53334644      # ASCII "S3FD"
RESPONSE_HEADER_MAGIC = 0x53335245   # ASCII "S3RE"

# Header struct packing strings (Little-Endian)
# frame_header_t: magic(I), frame_id(H), stage_mode(H), width(I), height(I), payload_len(I)
FRAME_HEADER_FMT = "<IHHIII"
FRAME_HEADER_SIZE = struct.calcsize(FRAME_HEADER_FMT)

# latency_report_t: recv_us(I), dec_us(I), infer_us(I), send_us(I), total_us(I)
# response_header_t: magic(I), frame_id(H), stage_mode(H), num_det(H), echoed_len(I), latency_report_t (5 * I)
RESP_HEADER_FMT = "<IHHIIIIIII"
RESP_HEADER_SIZE = struct.calcsize(RESP_HEADER_FMT)

# detection_box_t: class_id(H), score(f), x(H), y(H), w(H), h(H)
DETECTION_BOX_FMT = "<HfHHHH"
DETECTION_BOX_SIZE = struct.calcsize(DETECTION_BOX_FMT)

CLASS_LABELS = {
    0: "Target/Person",
    1: "Face",
    2: "Object"
}

# Global State Variables for Thread Synchronization
g_stage_mode = STAGE_MODE
g_exit_requested = False

def terminal_key_listener():
    """Background thread listening for keyboard input in terminal console."""
    global g_stage_mode, g_exit_requested
    if not sys.platform.startswith('win'):
        return
    while not g_exit_requested:
        try:
            if msvcrt.kbhit():
                ch = msvcrt.getch()
                if ch == b'1':
                    g_stage_mode = 1
                    print("\n[+] Switched to Stage 1 (Results Only)", flush=True)
                elif ch == b'2':
                    g_stage_mode = 2
                    print("\n[+] Switched to Stage 2 (Results + Frame Echo)", flush=True)
                elif ch in (b'q', b'Q', b'\x1b'):
                    print("\n[+] Exit requested from terminal...", flush=True)
                    g_exit_requested = True
        except Exception:
            pass
        time.sleep(0.05)

def recv_exact(sock, length):
    """Utility to receive exactly `length` bytes from TCP socket."""
    buf = bytearray()
    while len(buf) < length:
        chunk = sock.recv(length - len(buf))
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)

def connect_to_esp32(ip, port):
    """Establishes TCP connection to ESP32-S3 server."""
    while True:
        try:
            print(f"[+] Connecting to ESP32-S3 at {ip}:{port}...")
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.settimeout(10.0)
            sock.connect((ip, port))
            print(f"[+] Successfully connected to ESP32-S3 server!")
            return sock
        except Exception as e:
            print(f"[-] Connection failed ({e}). Retrying in {RECONNECT_DELAY_SEC}s...")
            time.sleep(RECONNECT_DELAY_SEC)

def draw_hud(frame, detections, timing_info, stage_mode, fps, rtt_ms):
    """Renders bounding boxes, latency statistics, and FPS overlay on frame."""
    h, w, _ = frame.shape

    # Draw Detection Bounding Boxes
    for box in detections:
        cid, score, bx, by, bw, bh = box
        label = CLASS_LABELS.get(cid, f"Class {cid}")
        
        # Color coding: Green for target
        color = (0, 255, 0)
        cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), color, 2)
        
        # Label overlay with confidence
        text = f"{label}: {score*100:.1f}%"
        txt_size, _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        cv2.rectangle(frame, (bx, max(0, by - 18)), (bx + txt_size[0] + 4, max(0, by)), color, -1)
        cv2.putText(frame, text, (bx + 2, max(12, by - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1, cv2.LINE_AA)

    # Draw Statistics Panel (Top-Left HUD)
    hud_bg = np.zeros((140, 280, 3), dtype=np.uint8)
    cv2.rectangle(hud_bg, (0, 0), (280, 140), (20, 20, 20), -1)
    
    cv2.putText(hud_bg, f"ESP32-S3 Rig [Stage {stage_mode}]", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    cv2.putText(hud_bg, f"FPS: {fps:.1f} | Resolution: {w}x{h}", (10, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
    cv2.putText(hud_bg, f"RTT Latency: {rtt_ms:.1f} ms", (10, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
    
    if timing_info:
        r_us, d_us, i_us, s_us, tot_us = timing_info
        cv2.putText(hud_bg, f"ESP Recv:   {r_us/1000.0:.1f} ms", (10, 74), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)
        cv2.putText(hud_bg, f"ESP Decode: {d_us/1000.0:.1f} ms", (10, 92), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)
        cv2.putText(hud_bg, f"ESP Infer:  {i_us/1000.0:.1f} ms", (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)
        cv2.putText(hud_bg, f"ESP Send:   {s_us/1000.0:.1f} ms", (10, 128), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)

    # Overlay HUD onto main frame
    frame[10:150, 10:290] = cv2.addWeighted(frame[10:150, 10:290], 0.2, hud_bg, 0.8, 0)
    return frame

def main():
    global g_stage_mode, g_exit_requested

    parser = argparse.ArgumentParser(description="ESP32-S3 Object Detection PC Client")
    parser.add_argument("--ip", type=str, default=ESP32_IP, help=f"ESP32-S3 IP Address (default: {ESP32_IP})")
    parser.add_argument("--port", type=int, default=ESP32_PORT, help=f"ESP32-S3 Port (default: {ESP32_PORT})")
    parser.add_argument("--stage", type=int, choices=[1, 2], default=STAGE_MODE, help="Stage Mode (1 or 2)")
    args = parser.parse_args()

    target_ip = args.ip
    target_port = args.port
    g_stage_mode = args.stage

    print("=====================================================")
    print(" PC Webcam Streaming & Detection Monitor")
    print(f" Target: {target_ip}:{target_port} | Initial Stage: {g_stage_mode}")
    print(" Press 'q' or ESC to exit. Press '1' or '2' to switch Stage Mode.")
    print("=====================================================")

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"[-] Error: Unable to open webcam index {CAMERA_INDEX}")
        sys.exit(1)

    # Start background keyboard listener thread for terminal
    listener_thread = threading.Thread(target=terminal_key_listener, daemon=True)
    listener_thread.start()

    sock = connect_to_esp32(target_ip, target_port)

    frame_id = 0
    prev_time = time.time()
    fps = 0.0
    timing_info = None
    resp_stage = g_stage_mode
    rtt_ms = 0.0
    detections = []

    try:
        while not g_exit_requested:
            ret, frame = cap.read()
            if not ret:
                print("[-] Error reading frame from webcam")
                break

            # Resize webcam frame to target resolution (320x240)
            resized = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT), interpolation=cv2.INTER_AREA)

            # Encode as JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
            result, enc_img = cv2.imencode('.jpg', resized, encode_param)
            if not result:
                continue

            jpeg_bytes = enc_img.tobytes()
            payload_len = len(jpeg_bytes)

            # Pack Frame Header
            frame_id = (frame_id + 1) % 65536
            current_stage = g_stage_mode
            header_bytes = struct.pack(
                FRAME_HEADER_FMT,
                FRAME_HEADER_MAGIC,
                frame_id,
                current_stage,
                FRAME_WIDTH,
                FRAME_HEIGHT,
                payload_len
            )

            # Send Header + JPEG Payload to ESP32-S3 and receive response
            t_send_start = time.time()
            try:
                # 1. Send frame to ESP32
                sock.sendall(header_bytes + jpeg_bytes)

                # 2. Receive Response Header from ESP32-S3
                resp_header_raw = recv_exact(sock, RESP_HEADER_SIZE)
                if not resp_header_raw:
                    print("[-] Server closed connection", flush=True)
                    sock.close()
                    sock = connect_to_esp32(target_ip, target_port)
                    continue

                t_recv_end = time.time()
                rtt_ms = (t_recv_end - t_send_start) * 1000.0

                # Unpack Response Header
                resp_fields = struct.unpack(RESP_HEADER_FMT, resp_header_raw)
                magic = resp_fields[0]
                resp_frame_id = resp_fields[1]
                resp_stage = resp_fields[2]
                num_detections = resp_fields[3]
                echoed_len = resp_fields[4]
                timing_info = resp_fields[5:10] # recv, dec, infer, send, total

                if magic != RESPONSE_HEADER_MAGIC:
                    print(f"[-] Invalid response magic: {hex(magic)}", flush=True)
                    continue

                # 3. Receive Bounding Box Detections
                detections = []
                if num_detections > 0:
                    det_bytes = recv_exact(sock, num_detections * DETECTION_BOX_SIZE)
                    if det_bytes:
                        for i in range(num_detections):
                            offset = i * DETECTION_BOX_SIZE
                            box_data = struct.unpack(DETECTION_BOX_FMT, det_bytes[offset:offset+DETECTION_BOX_SIZE])
                            detections.append(box_data)

                # 4. If Stage 2: Receive Echoed Frame
                display_frame = resized.copy()
                if resp_stage == 2 and echoed_len > 0:
                    echoed_bytes = recv_exact(sock, echoed_len)
                    if echoed_bytes:
                        nparr = np.frombuffer(echoed_bytes, np.uint8)
                        decoded_echo = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                        if decoded_echo is not None:
                            display_frame = decoded_echo

            except (socket.timeout, socket.error) as err:
                print(f"[-] Socket timeout/error ({err}). Reconnecting...", flush=True)
                sock.close()
                sock = connect_to_esp32(target_ip, target_port)
                continue

            # Calculate FPS
            curr_time = time.time()
            fps = 1.0 / max(0.001, curr_time - prev_time)
            prev_time = curr_time

            # Render HUD and Display Window
            display_frame = draw_hud(display_frame, detections, timing_info, resp_stage, fps, rtt_ms)
            cv2.imshow("ESP32-S3 Wi-Fi Object Detection Monitor", display_frame)

            # Check OpenCV Window Key Press
            cv_key = cv2.waitKey(1) & 0xFF
            if cv_key in (ord('q'), ord('Q'), 27):
                print("[+] Exit key pressed on window.", flush=True)
                break
            elif cv_key == ord('1'):
                g_stage_mode = 1
                print("[+] Switched to Stage 1 (Results Only)", flush=True)
            elif cv_key == ord('2'):
                g_stage_mode = 2
                print("[+] Switched to Stage 2 (Results + Frame Echo)", flush=True)

            # Rate Limiting
            elapsed = time.time() - t_send_start
            target_delay = 1.0 / TARGET_FPS
            if elapsed < target_delay:
                time.sleep(target_delay - elapsed)

    except KeyboardInterrupt:
        print("\n[+] Exiting on User Interrupt...")
    finally:
        g_exit_requested = True
        sock.close()
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
