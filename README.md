# SensorLink

SensorLink is a project built on ESP32S3 Baremetal, designed to collect data from 12 different sensors and transmit it to a remote server.

## Dependencies

To run SensorLink, you will need the following dependencies:

- Arduino IDE with ESP32S3 board installation
- Libraries and drivers for the 12 sensors used (specific dependencies may vary depending on sensor types)
- Network connectivity libraries for transmitting data to a remote server

## Getting Started

### Prerequisites

- ESP-IDF framework installed and configured for ESP32S3 development
- Drivers and libraries for the specific sensors used in the project

### Installation

1. Clone the repository:
   ```sh
   git clone https://github.com/shayan-sohail/SensorLink.git
   ```
2. Set up Arduino IDE and install ESP32S3 board from Boards Manager.
3. Configure and build the project using ESP-IDF tools.
4. Flash the compiled firmware to the ESP32S3 device.

### Usage

1. Ensure the ESP32S3 device is connected to the sensors.
2. Power on the device and monitor the data collection process.
3. Data will be transmitted automatically to the configured remote server.
