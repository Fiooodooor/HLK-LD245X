// LD2420.hpp  (header – place in src/ alongside LD2450.hpp / LD2451.hpp)
#pragma once

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

namespace esphome::ld245x {

class LD2420 {
public:
    LD2420();

    void begin(SERIAL_TYPE &radarUartStream, bool waitReady = true);

    // === ALL command-mode functions from the official spec (Yes + NO commands fully implemented) ===
    bool beginConfigurationSession();           // Open command mode (3-step procedure)
    bool endConfigurationSession();             // Disable command mode

    bool queryFirmwareVersion();                // 0x0000 Read version

    // Register access (0x0001 / 0x0002)
    bool writeRegister(uint16_t chipAddr, const uint16_t* addrDataPairs, uint8_t numPairs);
    bool readRegister(uint16_t chipAddr, const uint16_t* regAddrs, uint8_t numRegs, uint16_t* outData);

    // ABD parameters (0x0007 / 0x0008) – implemented even though marked NO for completeness
    bool configureABDParameters(const uint8_t* payload, size_t len);
    bool readABDParameters(const uint8_t* paramNames, uint8_t numParams, uint8_t* outValues, size_t outLen);

    // Serial number (0x0011)
    bool querySerialNumber();

    // System parameters (0x0012 / 0x0013)
    bool configureSystemParameters(const uint8_t* payload, size_t len);
    bool readSystemParameters(const uint8_t* paramNames, uint8_t numParams, uint8_t* outValues, size_t outLen);

    // Factory test mode (0x0024 / 0x0025 / 0x0026)
    bool enterFactoryTestMode();
    bool exitFactoryTestMode();
    bool sendFactoryTestResults(const uint8_t* payload, size_t len);

    // Custom command range (0x0060–0x00A0) – generic access
    bool sendCustomCommand(uint16_t cmd, const uint8_t* payload = nullptr, size_t payload_len = 0);

    const char* getNameString() const;
    const char* getFirmwareString() const;

private:
    bool sendCommand(uint16_t cmd, const uint8_t* payload = nullptr, size_t payload_len = 0);
    int  readCommandResponse();
    void clearSerialBuffer();

    SERIAL_TYPE* rs = nullptr;
    uint8_t frameBuffer[256];
    size_t frameBufferBytesRead = 0;

    char name_string[32];
    char firmware_string[32];

    static const uint8_t FRAME_HEADER[4];
    static const uint8_t FRAME_TAIL[4];
};

}  // namespace esphome::ld245x
