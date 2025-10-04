# Thesis-Research-Project---IoT-Monitoring-System-for-Microalgae-Cultivation-.
## Overview

This project is an IoT-based monitoring system designed for microalgae cultivation. It utilizes ESP32 microcontrollers and a Raspberry Pi to collect, process, and visualize environmental data, enabling efficient and automated monitoring of cultivation conditions.

## Features

- Real-time data acquisition (temperature, pH, light intensity, etc.)
- Wireless communication between ESP32 nodes and Raspberry Pi
- Data logging and visualization via web dashboard
- Alert system for abnormal conditions

## Hardware Requirements

- ESP32 development boards
- Raspberry Pi (any recent model)
- Sensors: temperature, pH, light, etc.
- Power supply and wiring

## Software Requirements

- Arduino IDE (for ESP32 firmware)
- Python 3 (for Raspberry Pi scripts)
- Node.js (for web dashboard)
- MQTT broker (e.g., Mosquitto)

## Installation

1. **ESP32 Setup**
    - Flash the provided firmware to each ESP32.
    - Connect sensors as described in the `/hardware` folder.

2. **Raspberry Pi Setup**
    - Install required Python packages:  
      ```
      pip install -r requirements.txt
      ```
    - Set up the MQTT broker and web dashboard.

3. **Configuration**
    - Edit configuration files to match your network and sensor setup.

## Usage

- Power on all ESP32 nodes and the Raspberry Pi.
- Access the web dashboard via your browser to monitor real-time data.
- Set up alerts and data logging as needed.

## Folder Structure

```
/firmware      # ESP32 source code
/hardware      # Schematics and wiring diagrams
/scripts       # Raspberry Pi scripts
/web-dashboard # Web interface code
```

## License

This project is licensed under the MIT License.

## Acknowledgements

- University research team
- Open-source hardware and software communities
