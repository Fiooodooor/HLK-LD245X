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

#include <LD2451.hpp>
#define RXP1 26
#define TXP1 27

using namespace esphome::ld245x;

HardwareSerial ld2451Serial(1);
LD2451 ld2451;

void setup() {
  Serial.begin(115200);
  ld2451Serial.begin(LD2451_SERIAL_SPEED, SERIAL_8N1, RXP1, TXP1);
  ld2451Serial.setTimeout(1000);
  LOG_INFO_FTS("LD2451, HardwareSerial(1) waiting for sensor data...\n");
  ld2451.begin(ld2451Serial);

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
