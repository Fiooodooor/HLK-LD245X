#if defined(ARDUINO) && ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

#include <LD2451.hpp>
#define RXP1 26
#define TXP1 27

using namespace esphome::ld245x;

#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
HardwareSerial ld2451Serial(1);
#define RADAR_SERIAL ld2451Serial
#else
// AVR uses predefined Serial1, Serial2, etc.
#define RADAR_SERIAL Serial1
#endif

LD2451 ld2451;

void setup() {
  Serial.begin(115200);
#if defined(ARDUINO_ARCH_ESP32)
  RADAR_SERIAL.begin(LD2451_SERIAL_SPEED, SERIAL_8N1, RXP1, TXP1);
#elif defined(ARDUINO_ARCH_ESP8266)
  RADAR_SERIAL.begin(LD2451_SERIAL_SPEED, SERIAL_8N1);
#else
  // AVR
  RADAR_SERIAL.begin(LD2451_SERIAL_SPEED);
#endif
  RADAR_SERIAL.setTimeout(1000);
  LOG_INFO_FTS("LD2451 waiting for sensor data...\n");
  ld2451.begin(RADAR_SERIAL);

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
  if(ld2451.update()) {
    sensor_got_valid_targets = ld2451.getNrValidTargets();
    while(0<sensor_got_valid_targets--) {
      auto target = ld2451.getTarget(sensor_got_valid_targets);
      Serial.println(target.format().c_str());
    }
  }
}
