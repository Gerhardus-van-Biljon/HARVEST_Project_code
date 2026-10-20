#!/usr/bin/env python3
"""
OTA Wireless Firmware Flashing Utility for ESP32-S3 TMC2209 Robot Controller.

Usage:
    python scripts/ota_flash.py [path_to_binary] [--ip IP_ADDRESS]

Examples:
    python scripts/ota_flash.py build/2209_control.bin
    python scripts/ota_flash.py build/2209_control.bin --ip 192.168.4.1
    python scripts/ota_flash.py build/2209_control.bin --ip 192.168.1.150
"""

import sys
import os
import argparse
import time
import urllib.request
import urllib.error

def flash_ota(bin_path, ip_address, port=80, timeout=60):
    if not os.path.exists(bin_path):
        print(f"[-] Error: Firmware binary not found at '{bin_path}'")
        return False

    file_size = os.path.getsize(bin_path)
    url = f"http://{ip_address}:{port}/update"

    print("=" * 65)
    print("  ESP32-S3 WIRELESS OTA FIRMWARE FLASHER")
    print("=" * 65)
    print(f"[*] Target Device  : http://{ip_address}:{port}")
    print(f"[*] Firmware Binary: {bin_path} ({file_size / 1024:.1f} KB)")
    print(f"[*] Endpoint       : {url}")
    print("[!] Safety Note    : Motors will be stopped during upload")
    print("-" * 65)

    try:
        # Check if device is reachable
        status_url = f"http://{ip_address}:{port}/api/status"
        print(f"[*] Probing device status at {status_url}...")
        req = urllib.request.Request(status_url)
        with urllib.request.urlopen(req, timeout=5) as response:
            status_code = response.getcode()
            content = response.read().decode('utf-8')
            print(f"[+] Device online! Status: {content}")
    except Exception as e:
        print(f"[!] Warning: Device status check returned: {e}")
        print("[*] Proceeding with firmware upload attempt...")

    print("\n[*] Uploading firmware binary...")
    start_time = time.time()

    try:
        with open(bin_path, 'rb') as f:
            data = f.read()

        req = urllib.request.Request(url, data=data, method='POST')
        req.add_header('Content-Type', 'application/octet-stream')
        req.add_header('Content-Length', str(len(data)))

        with urllib.request.urlopen(req, timeout=timeout) as response:
            elapsed = time.time() - start_time
            speed_kb = (file_size / 1024) / elapsed if elapsed > 0 else 0
            res_text = response.read().decode('utf-8')
            print(f"[+] Transfer completed in {elapsed:.2f}s ({speed_kb:.1f} KB/s)")
            print(f"[+] Response: {res_text}")
            print("\n" + "=" * 65)
            print("  SUCCESS! ESP32-S3 rebooting into new partition...")
            print("=" * 65)
            return True

    except urllib.error.HTTPError as e:
        print(f"[-] HTTP Error {e.code}: {e.read().decode('utf-8', errors='ignore')}")
        return False
    except urllib.error.URLError as e:
        print(f"[-] Connection Error: {e.reason}")
        return False
    except Exception as e:
        print(f"[-] Unexpected Error: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description="Flash ESP32-S3 Over-The-Air (Wi-Fi)")
    parser.add_argument("binary", nargs="?", default="build/2209_control.bin",
                        help="Path to .bin firmware file (default: build/2209_control.bin)")
    parser.add_argument("--ip", default="192.168.4.1",
                        help="ESP32 IP address (default: 192.168.4.1 for SoftAP)")
    parser.add_argument("--port", type=int, default=80,
                        help="HTTP port (default: 80)")
    args = parser.parse_args()

    success = flash_ota(args.binary, args.ip, args.port)
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()

