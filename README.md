# Automotive CAN Data Logger & OBD-II Scanner

## Overview
Open-source, dual-mode embedded platform for vehicle data acquisition.
Designed for driver-behaviour analysis, fleet telematics, and CAN bus reverse-engineering.
Works with **any** OBD-II compliant vehicle (active polling) and **any** CAN bus (passive sniffing).
Tested and validated on a production Renault Clio IV.

## Features
- **Active OBD-II polling** – retrieves RPM, speed, coolant temperature, throttle position,
  engine load, and intake air temperature in real time
- **Passive CAN sniffing** – captures every frame on the bus in listen-only mode
- **Proprietary signal reverse-engineering** – mapping of steering angle, wheel speeds,
  and brake pressure (verified against vehicle dynamics)
- **Python live sniffer** – terminal dashboard with per-ID frame count, Hz, and raw bytes;
  optional DBC decoding for immediate signal display
- **SD card logging** – raw frames (binary) and decoded CSV for offline analysis
- **Serial-command marker** – insert timestamped markers during a drive for precise
  event correlation
- **Custom 4-layer PCB** – integrates ESP32-S3, MCP2515 CAN controller, GPS, IMU,
  microSD, USB-C, and automotive power protection

## Repository Structure
```text
├── firmware/   # PlatformIO project (ESP-IDF, FreeRTOS)
├── software/   # Python tools: sniffer, decoder, log extractor, validation plots
├── hardware/   # Altium designer files, BOM, Gerber manufacturing files, schematics PDF
├── docs/       # Additional documentation, signal database, application notes
└── README.md
```

## Getting Started

### Hardware
1. Order the PCB from your favourite manufacturer using the Gerber files in `hardware/Gerber/`.
2. Source the components listed in `hardware/BOM.csv`.
3. Assemble the board (all SMD parts are on the top side; through-hole connectors are hand-soldered).

### Firmware
1. Install [PlatformIO](https://platformio.org/).
2. Open the `firmware/` folder as a PlatformIO project.
3. Select the build environment:
   - `node_a` – **real-car OBD-II polling** (normal mode)
   - `node_a` with `PASSIVE_SNIFF_MODE` – **listen-only CAN capture**
   - `node_b` – **bench simulator** (for testing without a vehicle)
4. Build and upload to the ESP32-S3.

### Software
```bash
pip install -r software/requirements.txt
python software/can_sniffer.py --port COM12
```

Additional Python tools:

- `decode_log.py` – process candump logs and decode frames with a DBC file
- `extract_can.py` – extract pure CAN frames from serial console logs
- `plot_validation.py` – generate validation plots from OBD-II JSON logs

## Validation
The system has been validated on a Renault Clio IV:

- Active OBD-II polling matches dashboard gauges within their resolution
- Passive sniffing captures all broadcast CAN identifiers (44 unique IDs)
- Proprietary signals (steering, wheel speeds, brake pressure) are decoded and physically verified
- Continuous logging over a 30-minute drive with zero firmware crashes

## Customisation
- Add your own signal definitions by editing `renault_clio4.json` (or a DBC file)
- Modify the OBD-II schedule in `decoder_app.c` to poll different PIDs
- Implement on-device inference by deploying a trained model using TensorFlow Lite Micro

## License
This project is released under the MIT License.

## Acknowledgements
- comma-ai/opendbc for open CAN databases
- JLCPCB for PCB fabrication
- The open-source automotive community for reverse-engineering knowledge
