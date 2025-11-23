# IoT-based Water Monitoring and Control System for Fish Farm

A real-time water quality monitoring and automated control system for aquaculture, built with Arduino and Blynk IoT platform.

## Authors
- **Khalid Hossain** - Hardware integration, IoT implementation
- **Soumik Saha** - Sensor calibration, control algorithms

## Features
- **Real-time Monitoring**: Temperature, pH, and turbidity measurements
- **Automated Control**: 
  - Heater/cooler activation for temperature regulation (24.5°C - 26.5°C)
  - Acid/base dispensing for pH correction (6.5 - 8.5)
- **Cloud Integration**: Live data visualization via Blynk mobile app
- **Alert System**: Instant notifications for parameter violations

## Hardware Components
- Arduino Uno/Mega
- ESP8266 WiFi Module (ESP-01)
- DS18B20 waterproof temperature sensor
- Analog pH sensor (with probe)
- Analog turbidity sensor
- 4-channel relay module
- DC motors for acid/base dispensing
- 12V power supply

## Sensor Specifications
| Parameter | Range | Alert Threshold |
|-----------|-------|-----------------|
| Temperature | 0-100°C | >26.5°C |
| pH | 0-14 | <6.0 or >8.0 |
| Turbidity | 0-100% | >30% |

## Software Requirements
- Arduino IDE (v1.8.x or later)
- Libraries:
  - `OneWire`
  - `DallasTemperature`
  - `ESP8266_Lib`
  - `BlynkSimpleShieldEsp8266`
- MATLAB R2019b or later (for calibration scripts)

## Installation
1. Clone this repository
2. Install required Arduino libraries via Library Manager
3. Update WiFi credentials in `water_monitoring_system.ino`:
   ```cpp
   char ssid[] = "YOUR_WIFI_SSID";
   char pass[] = "YOUR_WIFI_PASSWORD";
   ```
4. Update Blynk authentication token (get from Blynk Console)
5. Upload code to Arduino

## Calibration
Run `sensor_calibration.m` in MATLAB to visualize sensor response curves:
- **pH Sensor**: 3-point calibration (pH 4.0, 6.86, 9.18)
- **Turbidity Sensor**: Linear mapping (0-800 ADC → 100-0%)

## Pin Configuration
| Component | Pin |
|-----------|-----|
| DS18B20 (Data) | D5 |
| pH Sensor | A1 |
| Turbidity Sensor | A0 |
| Heater Control | D8 |
| Cooler Control | D9 |
| Heater Relay | D10 |
| Cooler Relay | D11 |
| Acid Motor | D6 |
| Base Motor | D7 |
| ESP8266 RX | D2 |
| ESP8266 TX | D3 |

## Usage
1. Power on the system
2. Monitor serial output (38400 baud) for sensor readings
3. View real-time data on Blynk mobile app
4. System automatically maintains optimal water conditions

## Project Structure
```
├── water_monitoring_system.ino    # Main Arduino code
├── sensor_calibration.m            # MATLAB calibration scripts
└── README.md                       # This file
```

## License
This project is open-source and available for educational purposes.

## Acknowledgments
Developed as part of the Microcontroller course project at Bangladesh University of Engineering and Technology (BUET).
