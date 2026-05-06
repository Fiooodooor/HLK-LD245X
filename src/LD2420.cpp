
#include "LD2420.hpp"

namespace esphome::ld245x {

/* --------------------------------------------------------------------- */
/*  Concrete LD2420 implementation                                       */
/* --------------------------------------------------------------------- */
LD2420::LD2420()
    : LD245X(
        SensorModel::LD2420, BaudRate::BAUD_115200,
        LD2420_MAX_SENSOR_TARGETS, LD2420_TARGET_SIZE,
        reinterpret_cast<const uint8_t*>("\xFD\xFC\xFB\xFA"),
        reinterpret_cast<const uint8_t*>("\x04\x03\x02\x01"),
        reinterpret_cast<const uint8_t*>("\xAA\xFF"),
        reinterpret_cast<const uint8_t*>("\x55\xCC"))
{
    setFactorySetting();
}

/* --------------------------------------------------------------------- */
void LD2420::setFactorySetting()
{
    _baudRate = BaudRate::BAUD_115200;
    _bluetoothEnabled = false;
    ld2420_firmware_string[0] = '\0';
}

/* --------------------------------------------------------------------- */
int LD2420::parseRadarFrame()
{
#if LD245X_USE_CIRCULAR_BUFFER
    // Ensure we have length bytes
    unsigned long start = millis();
    while (rxBuffer.available() < 2 && (millis() - start < 100)) {
        fillRxBuffer();
        delay(1);
    }

    if (rxBuffer.available() < 2) return -4;

    uint8_t lenBytes[2];
    rxBuffer.read(lenBytes, 2);
    uint16_t objectCount = word(lenBytes[1], lenBytes[0]);

    if (objectCount > dataTargetsCount) objectCount = dataTargetsCount;
    uint16_t bytesExpected = objectCount * dataTargetSize;
    if (bytesExpected > sizeof(frameBuffer)) return -3;

    // Wait for complete frame
    start = millis();
    size_t totalExpected = bytesExpected + frameIndicatorsLen[3];
    while (rxBuffer.available() < totalExpected) {
        if (millis() - start > 100) return -3;
        fillRxBuffer();
        delay(1);
    }

    // Read payload
    frameBufferBytesRead = rxBuffer.read(frameBuffer, bytesExpected);
    if (frameBufferBytesRead != static_cast<int>(bytesExpected)) return -3;

    frameBuffer[frameBufferBytesRead] = '\0';

    // Read and verify footer
    uint8_t endBuf[5] = {};
    int seqRead = rxBuffer.read(endBuf, frameIndicatorsLen[3]);
    if (seqRead != static_cast<int>(frameIndicatorsLen[3])) return -2;
    if (!matchSequence(endBuf, frameIndicatorsSeq[3], frameIndicatorsLen[3])) return -1;

#else
    if (!rs || rs->available() < 2) return -4;

    uint8_t lenBytes[2], endBuf[5] = {};
    lenBytes[0] = rs->read();               // LSB
    lenBytes[1] = rs->read();               // MSB
    uint16_t objectCount = word(lenBytes[1], lenBytes[0]);

    if (objectCount > dataTargetsCount) objectCount = dataTargetsCount;
    uint16_t bytesExpected = objectCount * dataTargetSize;
    if (bytesExpected > sizeof(frameBuffer)) return -3;

    frameBufferBytesRead = rs->readBytes(frameBuffer, bytesExpected);
    if (frameBufferBytesRead != static_cast<int>(bytesExpected)) return -3;

    frameBuffer[frameBufferBytesRead] = '\0';

    int seqRead = rs->readBytes(endBuf, frameIndicatorsLen[3]);
    if (seqRead != static_cast<int>(frameIndicatorsLen[3])) return -2;
    if (!matchSequence(endBuf, frameIndicatorsSeq[3], frameIndicatorsLen[3])) return -1;
#endif

    LOG_DEBUG_TS("LD2420:");
    LOG_DEBUG(frameBufferBytesRead);
    LOG_DEBUG(":");

    uint8_t id = 0;
    for (size_t i = 0; i < static_cast<size_t>(frameBufferBytesRead) && id < dataTargetsCount; i += dataTargetSize, ++id) {
        rt[id].setValid(false);
        if (i + dataTargetSize <= static_cast<size_t>(frameBufferBytesRead)) {
            rt[id].setFromRawBytes(frameBuffer + i,
                                   frameBufferBytesRead - static_cast<int>(i),
                                   id);
            LOG_DEBUG(rt[id].format().c_str());
            LOG_DEBUG(" ");
        }
    }
    LOG_DEBUG("\n");

    return id;
}

/* --------------------------------------------------------------------- */
/* LD2420-specific command functions                                     */
/* --------------------------------------------------------------------- */
bool LD2420::beginConfigurationSession()
{
    LOG_INFO_TS("LD2420: Entering command mode (3-step procedure)...\n");
    const uint8_t openPayload[2] = {0x01, 0x00};

    // Step a) First OPEN (ignore any waveform data)
    if (!sendLD2420Command(0x00FF, openPayload, sizeof(openPayload))) {
        LOG_ERROR_TS("LD2420: First OPEN command failed\n");
        return false;
    }
    delay(100);
    clearSerialBuffer();

    // Step c) Second OPEN + parse response
    if (!sendLD2420Command(0x00FF, openPayload, sizeof(openPayload))) {
        LOG_ERROR_TS("LD2420: Second OPEN command failed\n");
        return false;
    }
    if (readLD2420Response() == 0) {
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
    if (!sendLD2420Command(0x00FE)) return false;
    return (readLD2420Response() == 0);
}

/* --------------------------------------------------------------------- */
bool LD2420::queryFirmwareVersion()
{
    LOG_INFO_TS("LD2420: Querying firmware version (0x0000)...\n");
    if (!sendLD2420Command(0x0000)) return false;
    if (readLD2420Response() != 0) return false;

    if (frameBufferBytesRead < 12) return false;

    uint16_t retCmd = word(frameBuffer[1], frameBuffer[0]);
    uint16_t retVal = word(frameBuffer[3], frameBuffer[2]);
    if (retCmd != 0x0100 || retVal != 0) return false;

    uint16_t strLen = word(frameBuffer[5], frameBuffer[4]);
    if (frameBufferBytesRead < 6 + strLen) return false;

    snprintf(ld2420_firmware_string, sizeof(ld2420_firmware_string),
             "%.*s", strLen, reinterpret_cast<const char*>(frameBuffer + 6));
    LOG_INFO_FTS("LD2420 firmware: %s\n", ld2420_firmware_string);

    return true;
}

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
    if (!sendLD2420Command(0x0001, payload, pos)) return false;
    return (readLD2420Response() == 0);
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
    if (!sendLD2420Command(0x0002, payload, pos)) return false;
    if (readLD2420Response() != 0) return false;
    if (frameBufferBytesRead < 4 + 2 * numRegs) return false;
    for (uint8_t i = 0; i < numRegs; ++i) {
        outData[i] = word(frameBuffer[5 + 2 * i], frameBuffer[4 + 2 * i]);
    }
    return true;
}

/* --------------------------------------------------------------------- */
bool LD2420::configureABDParameters(const uint8_t* payload, size_t len)
{
    if (!payload || len == 0) return false;
    if (!sendLD2420Command(0x0007, payload, len)) return false;
    return (readLD2420Response() == 0);
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
    if (!sendLD2420Command(0x0008, payload, pos)) return false;
    if (readLD2420Response() != 0) return false;
    if (frameBufferBytesRead < 4 + 4 * numParams || outLen < 4 * numParams) return false;
    memcpy(outValues, frameBuffer + 4, 4 * numParams);
    return true;
}

/* --------------------------------------------------------------------- */
bool LD2420::querySerialNumber()
{
    LOG_INFO_TS("LD2420: Reading serial number (0x0011)...\n");
    if (!sendLD2420Command(0x0011)) return false;
    if (readLD2420Response() != 0) return false;
    LOG_INFO_TS("LD2420: Serial number command succeeded\n");
    return true;
}

/* --------------------------------------------------------------------- */
bool LD2420::configureSystemParameters(const uint8_t* payload, size_t len)
{
    if (!payload || len == 0) return false;
    if (!sendLD2420Command(0x0012, payload, len)) return false;
    return (readLD2420Response() == 0);
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
    if (!sendLD2420Command(0x0013, payload, pos)) return false;
    if (readLD2420Response() != 0) return false;
    if (frameBufferBytesRead < 4 + 4 * numParams || outLen < 4 * numParams) return false;
    memcpy(outValues, frameBuffer + 4, 4 * numParams);
    return true;
}

/* --------------------------------------------------------------------- */
bool LD2420::enterFactoryTestMode()
{
    LOG_INFO_TS("LD2420: Entering factory test mode (0x0024)...\n");
    if (!sendLD2420Command(0x0024)) return false;
    return (readLD2420Response() == 0);
}

/* --------------------------------------------------------------------- */
bool LD2420::exitFactoryTestMode()
{
    LOG_INFO_TS("LD2420: Exiting factory test mode (0x0025)...\n");
    if (!sendLD2420Command(0x0025)) return false;
    return (readLD2420Response() == 0);
}

/* --------------------------------------------------------------------- */
bool LD2420::sendFactoryTestResults(const uint8_t* payload, size_t len)
{
    if (!payload || len == 0) return false;
    if (!sendLD2420Command(0x0026, payload, len)) return false;
    return (readLD2420Response() == 0);
}

/* --------------------------------------------------------------------- */
bool LD2420::sendCustomCommand(uint16_t cmd, const uint8_t* payload, size_t payload_len)
{
    if (cmd < 0x0060 || cmd > 0x00A0) {
        LOG_ERROR_FTS("LD2420: Custom command 0x%04X out of allowed range\n", cmd);
        return false;
    }
    return sendLD2420Command(cmd, payload, payload_len) && (readLD2420Response() == 0);
}

/* --------------------------------------------------------------------- */
const char* LD2420::getFirmwareString() const
{
    return ld2420_firmware_string;
}

/* --------------------------------------------------------------------- */
bool LD2420::sendLD2420Command(uint16_t cmd, const uint8_t* payload, size_t payload_len)
{
    if (!rs) {
        LOG_ERROR_TS("LD2420: UART stream is null\n");
        return false;
    }

    uint8_t frame[256];
    size_t pos = 0;

    // Header: FD FC FB FA
    memcpy(&frame[pos], frameIndicatorsSeq[0], frameIndicatorsLen[0]);
    pos += frameIndicatorsLen[0];

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

    // Tail: 04 03 02 01
    memcpy(&frame[pos], frameIndicatorsSeq[1], frameIndicatorsLen[1]);
    pos += frameIndicatorsLen[1];

    LOG_DEBUG_FTS("LD2420 send cmd 0x%04X (%zu bytes)\n", cmd, pos);
    size_t written = rs->write(frame, pos);
    rs->flush();
    return written == pos;
}

/* --------------------------------------------------------------------- */
int LD2420::readLD2420Response()
{
    if (!rs) return -2;

#if LD245X_USE_CIRCULAR_BUFFER
    unsigned long start = millis();
    while (millis() - start < 1000) {
        fillRxBuffer();

        int hdrPos = rxBuffer.find(frameIndicatorsSeq[0], frameIndicatorsLen[0]);
        if (hdrPos < 0) {
            delay(1);
            continue;
        }

        if (hdrPos > 0)
            rxBuffer.discard(hdrPos);

        if (rxBuffer.available() < frameIndicatorsLen[0] + 2) {
            delay(1);
            continue;
        }

        rxBuffer.discard(frameIndicatorsLen[0]);

        uint8_t lenBuf[2];
        rxBuffer.read(lenBuf, 2);
        uint16_t intraLen = word(lenBuf[1], lenBuf[0]);

        if (intraLen > sizeof(frameBuffer)) return -4;

        unsigned long payloadStart = millis();
        while (rxBuffer.available() < intraLen + frameIndicatorsLen[1]) {
            if (millis() - payloadStart > 500) return -5;
            fillRxBuffer();
            delay(1);
        }

        size_t read = rxBuffer.read(frameBuffer, intraLen);
        if (read != intraLen) return -5;
        frameBufferBytesRead = static_cast<int>(read);

        uint8_t tail[8] = {};
        int tailRead = rxBuffer.read(tail, frameIndicatorsLen[1]);
        if (tailRead != static_cast<int>(frameIndicatorsLen[1])) return -6;
        if (!matchSequence(tail, frameIndicatorsSeq[1], frameIndicatorsLen[1])) return -6;

        if (frameBufferBytesRead >= 4) {
            uint16_t retVal = word(frameBuffer[3], frameBuffer[2]);
            return (retVal == 0) ? 0 : -static_cast<int>(retVal);
        }
        return -7;
    }
    return -1;
#else
    unsigned long start = millis();
    while (millis() - start < 1000) {
        if (rs->available() < 4) continue;

        uint8_t hdr[4];
        if (rs->readBytes(hdr, 4) != 4 || memcmp(hdr, frameIndicatorsSeq[0], 4) != 0)
            continue;

        uint8_t lenBuf[2];
        if (rs->readBytes(lenBuf, 2) != 2) return -3;
        uint16_t intraLen = word(lenBuf[1], lenBuf[0]);

        if (intraLen > sizeof(frameBuffer)) return -4;

        int read = rs->readBytes(frameBuffer, intraLen);
        if (read != static_cast<int>(intraLen)) return -5;
        frameBufferBytesRead = read;

        uint8_t tail[4];
        if (rs->readBytes(tail, 4) != 4 || memcmp(tail, frameIndicatorsSeq[1], 4) != 0) return -6;

        if (frameBufferBytesRead >= 4) {
            uint16_t retVal = word(frameBuffer[3], frameBuffer[2]);
            return (retVal == 0) ? 0 : -static_cast<int>(retVal);
        }
        return -7;
    }
    return -1;
#endif
}

/* --------------------------------------------------------------------- */
void LD2420::clearSerialBuffer()
{
#if LD245X_USE_CIRCULAR_BUFFER
    rxBuffer.clear();
#endif
    while (rs && rs->available()) rs->read();
}

}  // namespace esphome::ld245x

