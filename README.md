# HLK-LD245X

## Description

The HLK-LD245X library provides an easy-to-use Arduino interface for the Hi-Link LD2450 and LD2451 mmWave radar sensors. These sensors are designed for human presence detection, motion tracking, and zone-based occupancy monitoring. The library handles serial communication (UART) to configure the sensors and parse their output data, including target positions, speeds, and detection zones.

Key features:
- Support for both LD2450 (3-target human detection) and LD2451 (3-target car detection) variants.
- Automatic baud rate detection and configuration.
- Real-time parsing of radar data packets.
- Configurable detection zones and sensitivity.
- Low-power mode support for battery-operated applications.
- Compatible with Arduino IDE and PlatformIO.

This library is ideal for IoT projects involving smart home automation, security systems, and gesture recognition.

## Hardware Compatibility

- **Sensors**: HLK-LD2450, HLK-LD2451
- **Microcontrollers**: Any Arduino board with hardware UART support (e.g., Uno, Mega, ESP32, ESP8266, Raspberry Pi Pico)
- **Connections**:
  - VCC: 5V or 3.3V (sensor-dependent)
  - GND: Ground
  - TX (sensor) → RX (Arduino)
  - RX (sensor) → TX (Arduino)
- Requires a USB-to-UART adapter if using sensors without built-in USB.

## Dependencies

- None (uses standard Arduino `Serial` class)

## Installation

### Arduino Library Manager (Recommended)
1. Open the Arduino IDE.
2. Go to **Sketch > Include Library > Manage Libraries**.
3. Search for "HLK-LD245X".
4. Click **Install** on the library by Fiooodooor.

### Manual Installation
1. Clone or download this repository from [GitHub](https://github.com/Fiooodooor/HLK-LD245X).
2. Copy the `HLK-LD245X` folder to your Arduino libraries directory (e.g., `Documents/Arduino/libraries/`).
3. Restart the Arduino IDE.

### PlatformIO
Add to your `platformio.ini`:
```
lib_deps = https://github.com/Fiooodooor/HLK-LD245X.git
```

## Usage

### Basic Example
Include the library and initialize the sensor on a specific UART port.

```cpp
#if defined(ARDUINO) && ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
  #include <HardwareSerial.h>
#else
  #include <SoftwareSerial.h>
#endif

#include <LD2450.hpp>
#define RXP2 16
#define TXP2 17
#include <LD2451.hpp>
#define RXP1 26
#define TXP1 27

using namespace esphome::ld245x;

HardwareSerial ld2451Serial(1);
HardwareSerial ld2450Serial(2);
LD2451 ld2451;
LD2450 ld2450;

void setup() {
  Serial.begin(115200);
  ld2450Serial.begin(LD2450_SERIAL_SPEED, SERIAL_8N1, RXP1, TXP1);
  ld2451Serial.begin(LD2451_SERIAL_SPEED, SERIAL_8N1, RXP1, TXP1);
  ld2450Serial.setTimeout(1000);
  ld2451Serial.setTimeout(1000);

  LOG_INFO_FTS("LD2450, HardwareSerial(1) waiting for sensor data...\n");
  ld2450.begin(ld2450Serial);  
  LOG_INFO_FTS("LD2451, HardwareSerial(1) waiting for sensor data...\n");
  ld2451.begin(ld2451Serial);

  ld2450.beginConfigurationSession();
  ld2450.setMultiTargetTracking();
  ld2450.queryTargetTrackingMode();
  ld2450.queryFirmwareVersion();
  ld2450.queryMacAddress();
  ld2450.queryZoneFilter();
  ld2450.endConfigurationSession();

  LOG_INFO_FTS("Sensor name: '%s'\n", ld2450.getNameString());
  LOG_INFO_FTS("Zone filter: '%s'\n", ld2450.getZoneFilter());
  LOG_INFO_FTS("Firmware value: '%s'\n", ld2450.getFirmwareString());
  LOG_INFO_FTS("MacAddress value: '%s'\n", ld2450.getMacAddressString());

  ld2451.beginConfigurationSession();
  ld2451.queryFirmwareVersion();
  ld2451.queryMacAddress();
  ld2451.endConfigurationSession();

  LOG_INFO_FTS("Sensor name: '%s'\n", ld2451.getNameString());
  LOG_INFO_FTS("Firmware value: '%s'\n", ld2451.getFirmwareString());
  LOG_INFO_FTS("MacAddress value: '%s'\n", ld2451.getMacAddressString());
}

void loop()
{
  int sensor_got_valid_targets;
  if(ld2450.update()) {
    sensor_got_valid_targets = ld2450.getNrValidTargets();
    while(0<sensor_got_valid_targets--) {
      auto target = ld2450.getTarget(sensor_got_valid_targets);
      Serial.println(target.format().c_str());
    }
  }
  if(ld2451.update()) {
    sensor_got_valid_targets = ld2451.getNrValidTargets();
    while(0<sensor_got_valid_targets--) {
      auto target = ld2451.getTarget(sensor_got_valid_targets);
      Serial.println(target.format().c_str());
    }
  }
}
```

For full examples, see the `examples/` folder in the library.


For detailed parameters and return values, refer to the header file `HLK_LD245X.h`.

## Troubleshooting

- **No data received**: Check wiring, baud rate, and power supply. Ensure TX/RX are not swapped.
- **Garbled output**: Verify baud rate matches sensor default (256000).
- **Compilation errors**: Ensure Arduino IDE version >= 1.8.0.
- **Sensor not responding**: Reset the sensor by powering it off/on.

## Contributing

Contributions are welcome! Please fork the repository, make changes, and submit a pull request. For major changes, open an issue first.

1. Fork the project.
2. Create a feature branch (`git checkout -b feature/AmazingFeature`).
3. Commit changes (`git commit -m 'Add some AmazingFeature'`).
4. Push to the branch (`git push origin feature/AmazingFeature`).
5. Open a Pull Request.

## License

This library is distributed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Credits

- Original implementation by Fiooodooor.
- Based on Hi-Link LD245X sensor documentation.

---

*Library compliant with Arduino Library Manager requirements: Valid ZIP structure, examples, proper keywords in `library.properties`, and no external dependencies.*
