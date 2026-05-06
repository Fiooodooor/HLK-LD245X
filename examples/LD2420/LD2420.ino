#if defined(ARDUINO) && ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

#include <LD2420.hpp>
#define RXP2 16
#define TXP2 17

using namespace esphome::ld245x;

#if defined(ARDUINO_ARCH_ESP32)
HardwareSerial ld2420Serial(2);
#define RADAR_SERIAL ld2420Serial
#elif defined(ARDUINO_ARCH_ESP8266)
HardwareSerial ld2420Serial(1);
#define RADAR_SERIAL ld2420Serial
#else
// AVR uses predefined Serial1, Serial2, etc.
#define RADAR_SERIAL Serial1
#endif

LD2420 ld2420;

void setup() {
  Serial.begin(115200);
#if defined(ARDUINO_ARCH_ESP32)
  RADAR_SERIAL.begin(LD2420_SERIAL_SPEED, SERIAL_8N1, RXP2, TXP2);
#elif defined(ARDUINO_ARCH_ESP8266)
  RADAR_SERIAL.begin(LD2420_SERIAL_SPEED, SERIAL_8N1);
#else
  // AVR
  RADAR_SERIAL.begin(LD2420_SERIAL_SPEED);
#endif
  RADAR_SERIAL.setTimeout(1000);
  LOG_INFO_FTS("LD2420 waiting for sensor data...\n");
  ld2420.begin(RADAR_SERIAL);

  ld2420.beginConfigurationSession();
  ld2420.queryFirmwareVersion();
  ld2420.querySerialNumber();
  ld2420.endConfigurationSession();

  LOG_INFO_FTS("Sensor name: '%s'\n", ld2420.getNameString());
  LOG_INFO_FTS("Firmware value: '%s'\n", ld2420.getLD2420FirmwareString());
}

void loop()
{
  int sensor_got_valid_targets;
  if(ld2420.update()) {
    sensor_got_valid_targets = ld2420.getNrValidTargets();
    while(0<sensor_got_valid_targets--) {
      auto target = ld2420.getTarget(sensor_got_valid_targets);
      Serial.println(target.format().c_str());
    }
  }
}
