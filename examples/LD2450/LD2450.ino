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

using namespace esphome::ld245x;

HardwareSerial ld2450Serial(2);
LD2450 ld2450;

void setup() {
  Serial.begin(115200);
  ld2450Serial.begin(LD2450_SERIAL_SPEED, SERIAL_8N1, RXP2, TXP2);
  ld2450Serial.setTimeout(1000);
  LOG_INFO_FTS("LD2450, HardwareSerial(1) waiting for sensor data...\n");
  ld2450.begin(ld2450Serial);

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
