
#include "LD2451.hpp"

namespace esphome::ld245x {

/* --------------------------------------------------------------------- */
/*  Concrete LD2451 implementation                                       */
/* --------------------------------------------------------------------- */
LD2451::LD2451()
    : LD245X(
        SensorModel::LD2451, BaudRate::BAUD_115200,
        LD2451_MAX_SENSOR_TARGETS, LD2451_TARGET_SIZE,
        reinterpret_cast<const uint8_t*>("\xFD\xFC\xFB\xFA"),
        reinterpret_cast<const uint8_t*>("\x04\x03\x02\x01"),
        reinterpret_cast<const uint8_t*>("\xF4\xF3\xF2\xF1"),
        reinterpret_cast<const uint8_t*>("\xF8\xF7\xF6\xF5"))
{
    setFactorySetting();
}

/* --------------------------------------------------------------------- */
void LD2451::setFactorySetting()
{
    _baudRate = BaudRate::BAUD_115200;
    _bluetoothEnabled = true;
}

int LD2451::parseRadarFrame()
{
    if (rs->available() < 2) return -4;

    uint8_t objectCount, objectApproaching, lenBytes[2], id = 0, endBuf[5] = {};
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

    if (frameBufferBytesRead>=2) {
      objectCount = frameBuffer[0];
      objectApproaching = frameBuffer[1];
    }
    LOG_DEBUG_TS("LD2451:");
    LOG_DEBUG(frameBufferBytesRead);
    LOG_DEBUG(":\n");

    for (size_t i = 2; i < static_cast<size_t>(frameBufferBytesRead) && id < dataTargetsCount; i += dataTargetSize, ++id) {
        rt[id].setValid(false);
        if (i + dataTargetSize <= static_cast<size_t>(frameBufferBytesRead)) {
            rt[id].setFromRaw5Bytes(frameBuffer + i,
                                   frameBufferBytesRead - static_cast<int>(i),
                                   id);
            LOG_DEBUG(rt[id].format());
            LOG_DEBUG(" ");
        }
    }
    LOG_DEBUG("\n");
    return id;
}

}
