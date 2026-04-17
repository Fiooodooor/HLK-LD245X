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

#include "LD2420.hpp"

namespace esphome::ld245x {  // keep the same namespace as the existing library for seamless integration

/* --------------------------------------------------------------------- */
/* LD2420 – motion detector implementation (HLK-LD2420)                  */
/* --------------------------------------------------------------------- */
LD2420::LD2420()
    : _sensorModel(SensorModel::LD2420)
{
    TRACE_FUNC();
    setNameString();
}

/* --------------------------------------------------------------------- */
void LD2420::begin(SERIAL_TYPE &radarUartStream, bool waitReady)
{
    TRACE_FUNC();
    frameBufferBytesRead = 0;
    rs = &radarUartStream;
    if (waitReady && waitForSensorMessage()) {
        LOG_INFO_TS("LD2420, stream available.\n");
    } else if (waitReady) {
        LOG_WARN_TS("LD2420, no stream detected.\n");
    }
}

/* --------------------------------------------------------------------- */
bool LD2420::waitForSensorMessage(bool waitForever)
{
    TRACE_FUNC();
    if (!rs) return false;
    unsigned long start = millis();
    while (waitForever || (millis() - start < 1000)) {
        if (rs->available()) return true;
    }
    return false;
}

/* --------------------------------------------------------------------- */
/* Configuration session – follows the exact 3-step procedure described  */
/* in the LD2420 protocol (open → delay + clear → open + parse).        */
/* --------------------------------------------------------------------- */
bool LD2420::beginConfigurationSession()
{
    LOG_INFO_TS("LD2420: Entering command mode (3-step procedure)...\n");

    const uint8_t openPayload[2] = {0x01, 0x00};  // upper-computer version (as in official example)

    // Step a) First OPEN – ignore any returned data (waveform may still be streaming)
    if (!sendCommand(0x00FF, openPayload, sizeof(openPayload))) {
        LOG_ERROR_TS("LD2420: First OPEN command failed\n");
        return false;
    }

    delay(100);                     // protocol recommendation
    clearSerialBuffer();            // Step b) purge any remaining waveform data

    // Step c) Second OPEN – now parse the response
    if (!sendCommand(0x00FF, openPayload, sizeof(openPayload))) {
        LOG_ERROR_TS("LD2420: Second OPEN command failed\n");
        return false;
    }

    if (readCommandResponse() == 0) {
        LOG_INFO_TS("LD2420: Command mode entered successfully (protocol v2, buffer 1024 bytes)\n");
        return true;
    }

    LOG_ERROR_TS("LD2420: Command mode ACK failed\n");
    return false;
}

/* --------------------------------------------------------------------- */
bool LD2420::endConfigurationSession()
{
    LOG_INFO_TS("LD2420: Exiting command mode...\n");
    if (!sendCommand(0x00FE)) return false;
    return (readCommandResponse() == 0);
}

/* --------------------------------------------------------------------- */
bool LD2420::queryFirmwareVersion()
{
    LOG_INFO_TS("LD2420: Querying firmware version...\n");
    if (!sendCommand(0x0000)) return false;

    if (readCommandResponse() != 0) return false;

    if (frameBufferBytesRead < 12) return false;

    uint16_t retCmd = word(frameBuffer[1], frameBuffer[0]);
    uint16_t retVal = word(frameBuffer[3], frameBuffer[2]);
    if (retCmd != 0x0100 || retVal != 0) return false;

    uint16_t strLen = word(frameBuffer[5], frameBuffer[4]);
    if (frameBufferBytesRead < 6 + strLen) return false;

    snprintf(firmware_string, sizeof(firmware_string), "%.*s", strLen, (const char*)(frameBuffer + 6));
    LOG_INFO_FTS("LD2420 firmware: %s\n", firmware_string);
    return true;
}

/* --------------------------------------------------------------------- */
/* Low-level register commands (both single and multi supported)         */
/* --------------------------------------------------------------------- */
bool LD2420::writeRegister(uint16_t chipAddr, const uint16_t* addrDataPairs, uint8_t numPairs)
{
    if (numPairs == 0 || numPairs > 15) return false;  // safety (max ~64-byte frame)

    uint8_t payload[2 + 4 * 15];
    size_t pos = 0;

    payload[pos++] = chipAddr & 0xFF;
    payload[pos++] = (chipAddr >> 8) & 0xFF;

    for (uint8_t i = 0; i < numPairs; ++i) {
        payload[pos++] = addrDataPairs[2 * i] & 0xFF;
        payload[pos++] = (addrDataPairs[2 * i] >> 8) & 0xFF;
        payload[pos++] = addrDataPairs[2 * i + 1] & 0xFF;
        payload[pos++] = (addrDataPairs[2 * i + 1] >> 8) & 0xFF;
    }

    if (!sendCommand(0x0001, payload, pos)) return false;
    return (readCommandResponse() == 0);
}

/* --------------------------------------------------------------------- */
bool LD2420::readRegister(uint16_t chipAddr, const uint16_t* regAddrs, uint8_t numRegs, uint16_t* outData)
{
    if (numRegs == 0 || numRegs > 30) return false;

    uint8_t payload[2 + 2 * 30];
    size_t pos = 0;

    payload[pos++] = chipAddr & 0xFF;
    payload[pos++] = (chipAddr >> 8) & 0xFF;

    for (uint8_t i = 0; i < numRegs; ++i) {
        payload[pos++] = regAddrs[i] & 0xFF;
        payload[pos++] = (regAddrs[i] >> 8) & 0xFF;
    }

    if (!sendCommand(0x0002, payload, pos)) return false;

    if (readCommandResponse() != 0) return false;

    // response data = (2 bytes data) * N
    if (frameBufferBytesRead < 4 + 2 * numRegs) return false;

    for (uint8_t i = 0; i < numRegs; ++i) {
        outData[i] = word(frameBuffer[5 + 2 * i], frameBuffer[4 + 2 * i]);
    }
    return true;
}

/* --------------------------------------------------------------------- */
/* Generic command sender – builds the exact FA FB FC FD frame format    */
/* --------------------------------------------------------------------- */
bool LD2420::sendCommand(uint16_t cmd, const uint8_t* payload, size_t payload_len)
{
    if (!rs) {
        LOG_ERROR_TS("LD2420: UART stream is null\n");
        return false;
    }

    uint8_t frame[256];
    size_t pos = 0;

    // Header
    memcpy(&frame[pos], FRAME_HEADER, 4);
    pos += 4;

    // Intra-frame length (little-endian)
    uint16_t intraLen = 2 + payload_len;
    frame[pos++] = intraLen & 0xFF;
    frame[pos++] = (intraLen >> 8) & 0xFF;

    // Command (little-endian)
    frame[pos++] = cmd & 0xFF;
    frame[pos++] = (cmd >> 8) & 0xFF;

    // Payload
    if (payload_len > 0 && payload) {
        memcpy(&frame[pos], payload, payload_len);
        pos += payload_len;
    }

    // Tail
    memcpy(&frame[pos], FRAME_TAIL, 4);
    pos += 4;

    LOG_DEBUG_FTS("LD2420 send cmd 0x%04X (%zu bytes)\n", cmd, pos);
    size_t written = rs->write(frame, pos);
    rs->flush();
    return written == pos;
}

/* --------------------------------------------------------------------- */
/* Response reader – expects the same frame format, validates header/tail */
/* and returns 0 on success (retVal == 0)                               */
/* --------------------------------------------------------------------- */
int LD2420::readCommandResponse()
{
    if (!rs) return -2;

    unsigned long start = millis();
    while (millis() - start < 1000) {
        if (rs->available() < 4) continue;

        uint8_t hdr[4];
        if (rs->readBytes(hdr, 4) != 4 || memcmp(hdr, FRAME_HEADER, 4) != 0) continue;

        // Length
        uint8_t lenBuf[2];
        if (rs->readBytes(lenBuf, 2) != 2) return -3;
        uint16_t intraLen = word(lenBuf[1], lenBuf[0]);  // little-endian

        if (intraLen > sizeof(frameBuffer)) return -4;

        int read = rs->readBytes(frameBuffer, intraLen);
        if (read != static_cast<int>(intraLen)) return -5;
        frameBufferBytesRead = read;

        // Tail
        uint8_t tail[4];
        if (rs->readBytes(tail, 4) != 4 || memcmp(tail, FRAME_TAIL, 4) != 0) return -6;

        LOG_DEBUG_PRINT_BYTES(frameBuffer, frameBufferBytesRead);

        // Validate response header
        if (frameBufferBytesRead >= 4) {
            uint16_t retVal = word(frameBuffer[3], frameBuffer[2]);
            return (retVal == 0) ? 0 : -retVal;
        }
        return -7;
    }
    return -1;
}

/* --------------------------------------------------------------------- */
void LD2420::clearSerialBuffer()
{
    while (rs && rs->available()) rs->read();
}

/* --------------------------------------------------------------------- */
/* Name / firmware helpers (mirrors LD245X style)                        */
/* --------------------------------------------------------------------- */
void LD2420::setNameString()
{
    snprintf(name_string, sizeof(name_string), "ld2420_%d", ObjectCounter::count());
}

const char* LD2420::getNameString() const
{
    return name_string;
}

const char* LD2420::getFirmwareString() const
{
    return firmware_string;
}

/* --------------------------------------------------------------------- */
/* Static frame markers (exact byte order from protocol examples)        */
/* --------------------------------------------------------------------- */
const uint8_t LD2420::FRAME_HEADER[4] = {0xFD, 0xFC, 0xFB, 0xFA};
const uint8_t LD2420::FRAME_TAIL[4]   = {0x04, 0x03, 0x02, 0x01};

}  // namespace esphome::ld245x
