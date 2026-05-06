#if defined(ARDUINO) && ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

#include <LD2450.hpp>
#define RXP2 16
#define TXP2 17

using namespace esphome::ld245x;

#if defined(ARDUINO_ARCH_ESP32)
HardwareSerial ld2450Serial(2);
#define RADAR_SERIAL ld2450Serial
#elif defined(ARDUINO_ARCH_ESP8266)
HardwareSerial ld2450Serial(1);
#define RADAR_SERIAL ld2450Serial
#else
// AVR uses predefined Serial1, Serial2, etc.
#define RADAR_SERIAL Serial1
#endif

LD2450 ld2450;

void setup() {
  Serial.begin(115200);
#if defined(ARDUINO_ARCH_ESP32)
  RADAR_SERIAL.begin(LD2450_SERIAL_SPEED, SERIAL_8N1, RXP2, TXP2);
#elif defined(ARDUINO_ARCH_ESP8266)
  RADAR_SERIAL.begin(LD2450_SERIAL_SPEED, SERIAL_8N1);
#else
  // AVR
  RADAR_SERIAL.begin(LD2450_SERIAL_SPEED);
#endif
  RADAR_SERIAL.setTimeout(1000);
  LOG_INFO_FTS("LD2450 waiting for sensor data...\n");
  ld2450.begin(RADAR_SERIAL);

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
}
