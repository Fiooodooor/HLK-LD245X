// LD2420.cpp
#include "LD2420.hpp"

namespace esphome::ld245x {

/* --------------------------------------------------------------------- */
/* LD2420 – motion detector implementation (HLK-LD2420)                  */
/* --------------------------------------------------------------------- */
LD2420::LD2420()
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
/* Configuration session – exact 3-step procedure from spec             */
/* --------------------------------------------------------------------- */
bool LD2420::beginConfigurationSession()
{
    LOG_INFO_TS("LD2420: Entering command mode (3-step procedure)...\n");
    const uint8_t openPayload[2] = {0x01, 0x00};

    // Step a) First OPEN (ignore any waveform data)
    if (!sendCommand(0x00FF, openPayload, sizeof(openPayload))) {
        LOG_ERROR_TS("LD2420: First OPEN command failed\n");
        return false;
    }
    delay(100);
    clearSerialBuffer();

    // Step c) Second OPEN + parse response
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
    LOG_INFO_TS("LD2420: Querying firmware version (0x0000)...\n");
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
/* Register commands (0x0001 / 0x0002) – fully support multi-register   */
/* --------------------------------------------------------------------- */
bool LD2420::writeRegister(uint16_t chipAddr, const uint16_t* addrDataPairs, uint8_t numPairs)
{
    if (numPairs == 0 || numPairs > 15) return false;
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
    if (frameBufferBytesRead < 4 + 2 * numRegs) return false;
    for (uint8_t i = 0; i < numRegs; ++i) {
        outData[i] = word(frameBuffer[5 + 2 * i], frameBuffer[4 + 2 * i]);
    }
    return true;
}

/* --------------------------------------------------------------------- */
/* ABD parameters (0x0007 / 0x0008)                                     */
/* --------------------------------------------------------------------- */
bool LD2420::configureABDParameters(const uint8_t* payload, size_t len)
{
    if (!payload || len == 0) return false;
    if (!sendCommand(0x0007, payload, len)) return false;
    return (readCommandResponse() == 0);
}

/* --------------------------------------------------------------------- */
bool LD2420::readABDParameters(const uint8_t* paramNames, uint8_t numParams, uint8_t* outValues, size_t outLen)
{
    if (!paramNames || numParams == 0) return false;
    uint8_t payload[2 * 32];
    size_t pos = 0;
    for (uint8_t i = 0; i < numParams && pos + 2 <= sizeof(payload); ++i) {
        payload[pos++] = paramNames[2 * i];
        payload[pos++] = paramNames[2 * i + 1];
    }
    if (!sendCommand(0x0008, payload, pos)) return false;
    if (readCommandResponse() != 0) return false;
    if (frameBufferBytesRead < 4 + 4 * numParams || outLen < 4 * numParams) return false;
    memcpy(outValues, frameBuffer + 4, 4 * numParams);
    return true;
}

/* --------------------------------------------------------------------- */
/* Serial number (0x0011)                                                */
/* --------------------------------------------------------------------- */
bool LD2420::querySerialNumber()
{
    LOG_INFO_TS("LD2420: Reading serial number (0x0011)...\n");
    if (!sendCommand(0x0011)) return false;
    if (readCommandResponse() != 0) return false;
    // Response parsed by caller if needed (frameBuffer contains data)
    LOG_INFO_TS("LD2420: Serial number command succeeded\n");
    return true;
}

/* --------------------------------------------------------------------- */
/* System parameters (0x0012 / 0x0013)                                   */
/* --------------------------------------------------------------------- */
bool LD2420::configureSystemParameters(const uint8_t* payload, size_t len)
{
    if (!payload || len == 0) return false;
    if (!sendCommand(0x0012, payload, len)) return false;
    return (readCommandResponse() == 0);
}

/* --------------------------------------------------------------------- */
bool LD2420::readSystemParameters(const uint8_t* paramNames, uint8_t numParams, uint8_t* outValues, size_t outLen)
{
    if (!paramNames || numParams == 0) return false;
    uint8_t payload[2 * 32];
    size_t pos = 0;
    for (uint8_t i = 0; i < numParams && pos + 2 <= sizeof(payload); ++i) {
        payload[pos++] = paramNames[2 * i];
        payload[pos++] = paramNames[2 * i + 1];
    }
    if (!sendCommand(0x0013, payload, pos)) return false;
    if (readCommandResponse() != 0) return false;
    if (frameBufferBytesRead < 4 + 4 * numParams || outLen < 4 * numParams) return false;
    memcpy(outValues, frameBuffer + 4, 4 * numParams);
    return true;
}

/* --------------------------------------------------------------------- */
/* Factory test mode commands                                            */
/* --------------------------------------------------------------------- */
bool LD2420::enterFactoryTestMode()
{
    LOG_INFO_TS("LD2420: Entering factory test mode (0x0024)...\n");
    if (!sendCommand(0x0024)) return false;
    return (readCommandResponse() == 0);
}

/* --------------------------------------------------------------------- */
bool LD2420::exitFactoryTestMode()
{
    LOG_INFO_TS("LD2420: Exiting factory test mode (0x0025)...\n");
    if (!sendCommand(0x0025)) return false;
    return (readCommandResponse() == 0);
}

/* --------------------------------------------------------------------- */
bool LD2420::sendFactoryTestResults(const uint8_t* payload, size_t len)
{
    if (!payload || len == 0) return false;
    if (!sendCommand(0x0026, payload, len)) return false;
    return (readCommandResponse() == 0);
}

/* --------------------------------------------------------------------- */
/* Custom commands (0x0060–0x00A0)                                       */
/* --------------------------------------------------------------------- */
bool LD2420::sendCustomCommand(uint16_t cmd, const uint8_t* payload, size_t payload_len)
{
    if (cmd < 0x0060 || cmd > 0x00A0) {
        LOG_ERROR_FTS("LD2420: Custom command 0x%04X out of allowed range\n", cmd);
        return false;
    }
    return sendCommand(cmd, payload, payload_len) && (readCommandResponse() == 0);
}

/* --------------------------------------------------------------------- */
/* Generic low-level frame builder (exact FA FB FC FD … 04 03 02 01)     */
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

    // Intra-frame length (little-endian) = 2 (cmd) + payload
    uint16_t intraLen = 2 + static_cast<uint16_t>(payload_len);
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
/* Response reader – validates header/tail and return value              */
/* --------------------------------------------------------------------- */
int LD2420::readCommandResponse()
{
    if (!rs) return -2;
    unsigned long start = millis();
    while (millis() - start < 1000) {
        if (rs->available() < 4) continue;

        uint8_t hdr[4];
        if (rs->readBytes(hdr, 4) != 4 || memcmp(hdr, FRAME_HEADER, 4) != 0) continue;

        uint8_t lenBuf[2];
        if (rs->readBytes(lenBuf, 2) != 2) return -3;
        uint16_t intraLen = word(lenBuf[1], lenBuf[0]);

        if (intraLen > sizeof(frameBuffer)) return -4;

        int read = rs->readBytes(frameBuffer, intraLen);
        if (read != static_cast<int>(intraLen)) return -5;
        frameBufferBytesRead = read;

        uint8_t tail[4];
        if (rs->readBytes(tail, 4) != 4 || memcmp(tail, FRAME_TAIL, 4) != 0) return -6;

        LOG_DEBUG_PRINT_BYTES(frameBuffer, frameBufferBytesRead);

        if (frameBufferBytesRead >= 4) {
            uint16_t retVal = word(frameBuffer[3], frameBuffer[2]);
            return (retVal == 0) ? 0 : -static_cast<int>(retVal);
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
const uint8_t LD2420::FRAME_HEADER[4] = {0xFD, 0xFC, 0xFB, 0xFA};
const uint8_t LD2420::FRAME_TAIL[4]   = {0x04, 0x03, 0x02, 0x01};

}  // namespace esphome::ld245x
