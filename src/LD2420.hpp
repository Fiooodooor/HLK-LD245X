#ifndef __LD2420_hpp
#define __LD2420_hpp

#include "LD245X.hpp"

namespace esphome::ld245x {

// LD2420 specific constants
#define LD2420_MAX_SENSOR_TARGETS 1
#define LD2420_TARGET_SIZE 8
#define LD2420_SERIAL_BUFFER 256
#define LD2420_SERIAL_SPEED 115200

/* --------------------------------------------------------------------- */
/*  Concrete sensor implementation for LD2420                            */
/* --------------------------------------------------------------------- */
class LD2420 : public LD245X {
public:
    LD2420();
    void setFactorySetting() override;
    int parseRadarFrame() override;

    // === LD2420-specific command-mode functions (official spec) ===
    bool beginConfigurationSession();           // Open command mode (3-step procedure)
    bool endConfigurationSession();             // Disable command mode

    bool queryFirmwareVersion();                // 0x0000 Read version

    // Register access (0x0001 / 0x0002)
    bool writeRegister(uint16_t chipAddr, const uint16_t* addrDataPairs, uint8_t numPairs);
    bool readRegister(uint16_t chipAddr, const uint16_t* regAddrs, uint8_t numRegs, uint16_t* outData);

    // ABD parameters (0x0007 / 0x0008)
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

    // Custom command range (0x0060–0x00A0)
    bool sendCustomCommand(uint16_t cmd, const uint8_t* payload = nullptr, size_t payload_len = 0);

    const char* getFirmwareString() const;

private:
    bool sendLD2420Command(uint16_t cmd, const uint8_t* payload = nullptr, size_t payload_len = 0);
    int readLD2420Response();
    void clearSerialBuffer();

    char ld2420_firmware_string[32] = {'\0'};
};

}  // namespace esphome::ld245x

#endif
