# Renault Clio IV CAN Data Logger & OBD‑II Scanner

## Overview
Dual‑mode embedded system: active OBD‑II polling + passive CAN sniffing.
Designed for driver‑behaviour analysis and vehicle telematics.

## Features
- Real‑time OBD‑II data (RPM, speed, temperature, throttle, load)
- Passive capture of all HS‑CAN traffic (listen‑only)
- Reverse‑engineered proprietary signals (steering, wheels, brake)
- Python live sniffer with DBC decoding
- Custom 4‑layer PCB with ESP32‑S3, MCP2515, GPS, IMU, microSD

## Repository Structure
- `firmware/` – PlatformIO project (ESP‑IDF, FreeRTOS)
- `software/` – Python tools for log analysis
- `hardware/` – Altium design files, BOM, Gerbers

## Getting Started
### Hardware
Order the PCB from JLCPCB using the Gerbers in `hardware/Gerber/`.
Assemble according to the BOM and schematics.

### Firmware
1. Install PlatformIO.
2. Open the `firmware/` folder.
3. Select the build environment (`node_a` for real‑car mode).
4. Build and upload to ESP32‑S3.

### Software
```bash
pip install -r software/requirements.txt
python software/can_sniffer.py --port COM12