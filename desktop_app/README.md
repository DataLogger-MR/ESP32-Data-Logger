# BMS Data Logger Desktop Application

GUI companion application for communicating with and managing data from the ESP32 BMS Data Logger via USB Serial.

## Features
- Real-time live data monitoring
- File management and log file downloading
- Date-based log filtering and session history
- Configuration manager (WiFi, MQTT, logging settings)

## Installation & Setup

1. Install Python 3.10+
2. Install required dependencies:
   `ash
   pip install -r requirements.txt
   `

## Running the Application
`ash
python UIMODE.py
`

## Building Standalone Executable (.exe)
`ash
python build_complete.py
`
The resulting .exe will be located in the dist/ directory.
