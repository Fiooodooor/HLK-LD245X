
#include "LD2420.hpp"

namespace esphome::ld245x {

/* --------------------------------------------------------------------- */
/*  Concrete LD2420 implementation                                       */
/* --------------------------------------------------------------------- */
LD2420::LD2420()
    : LD245X(
        SensorModel::LD2450,  // Using LD2450 as model type since LD2420 isn't in the enum
        BaudRate::BAUD_115200,
        LD2420_MAX_SENSOR_TARGETS, LD2420_TARGET_SIZE,
        reinterpret_cast<const uint8_t*>("\xFD\xFC\xFB\xFA"),
        reinterpret_cast<const uint8_t*>("\x04\x03\x02\x01"),
        reinterpret_cast<const uint8_t*>("\xAA\xFF"),  // LD2420 might have different data frame markers
        reinterpret_cast<const uint8_t*>("\x55\xCC"))
{
    setFactorySetting();
    setNameString("ld2420");
}

/* --------------------------------------------------------------------- */
void LD2420::setFactorySetting()
{
    _baudRate = BaudRate::BAUD_115200;
    _bluetoothEnabled = false;  // LD2420 typically doesn't have Bluetooth
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
    uint16_t bytesExpected = word(lenBytes[1], lenBytes[0]);

    if (bytesExpected > sizeof(frameBuffer)) return -4;

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
    // Original implementation without circular buffer
    if (rs->available() < 2) return -4;

    uint8_t lenBytes[2], endBuf[5] = {};
    lenBytes[0] = rs->read();               // LSB
    lenBytes[1] = rs->read();               // MSB
    uint16_t bytesExpected = word(lenBytes[1], lenBytes[0]);

    if (bytesExpected > sizeof(frameBuffer)) return -4;

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

    // Parse LD2420 specific data format
    // LD2420 typically reports motion detection data
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

    unsigned long start = millis();
    while (millis() - start < 1000) {
        if (rs->available() < 4) continue;

        uint8_t hdr[4];
        if (rs->readBytes(hdr, 4) != 4 || memcmp(hdr, frameIndicatorsSeq[0], 4) != 0) continue;

        uint8_t lenBuf[2];
        if (rs->readBytes(lenBuf, 2) != 2) return -3;
        uint16_t intraLen = word(lenBuf[1], lenBuf[0]);

        if (intraLen > sizeof(frameBuffer)) return -4;

        int read = rs->readBytes(frameBuffer, intraLen);
        if (read != static_cast<int>(intraLen)) return -5;
        frameBufferBytesRead = read;

        uint8_t tail[4];
        if (rs->readBytes(tail, 4) != 4 || memcmp(tail, frameIndicatorsSeq[1], 4) != 0) return -6;

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
/* Register commands (0x0001 / 0x0002)                                  */
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
/* ABD parameters (0x0007 / 0x0008)                                     */
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
/* Serial number (0x0011)                                                */
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
/* System parameters (0x0012 / 0x0013)                                   */
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
/* Factory test mode commands                                            */
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
/* Custom commands (0x0060–0x00A0)                                       */
/* --------------------------------------------------------------------- */
bool LD2420::sendCustomCommand(uint16_t cmd, const uint8_t* payload, size_t payload_len)
{
    if (cmd < 0x0060 || cmd > 0x00A0) {
        LOG_ERROR_FTS("LD2420: Custom command 0x%04X out of allowed range\n", cmd);
        return false;
    }
    return sendLD2420Command(cmd, payload, payload_len) && (readLD2420Response() == 0);
}

}  // namespace esphome::ld245x
