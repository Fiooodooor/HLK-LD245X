
#include "LD2450.hpp"

namespace esphome::ld245x {

/* --------------------------------------------------------------------- */
/*  Concrete LD2450 implementation                                       */
/* --------------------------------------------------------------------- */
LD2450::LD2450()
    : LD245X(
        HLK_LD2450, LD2450_MAX_SENSOR_TARGETS, LD2450_TARGET_SIZE,
        reinterpret_cast<const uint8_t*>("\xFD\xFC\xFB\xFA"),
        reinterpret_cast<const uint8_t*>("\x04\x03\x02\x01"),
        reinterpret_cast<const uint8_t*>("\xAA\xFF"),
        reinterpret_cast<const uint8_t*>("\x55\xCC"))
{
}

/* --------------------------------------------------------------------- */
int LD2450::readDataRadarOutput()
{
    if (rs->available() < 2) return -4;

    int read;
    uint8_t lenBytes[2], endBuf[5] = {}, id = 0;
    lenBytes[0] = rs->read();               // LSB
    lenBytes[1] = rs->read();               // MSB
    uint16_t objectCount = word(lenBytes[1], lenBytes[0]);

    uint16_t bytesExpected = objectCount * dataTargetSize;
    if (bytesExpected > sizeof(frameBuffer)) return -3;

    frameBufferBytesRead = rs->readBytes(frameBuffer, bytesExpected);
    if (frameBufferBytesRead != static_cast<int>(bytesExpected)) return -3;

    frameBuffer[frameBufferBytesRead] = '\0';

    int seqRead = rs->readBytes(endBuf, frameIndicatorsLen[3]);
    if (seqRead != static_cast<int>(frameIndicatorsLen[3])) return -2;
    if (!matchSequence(endBuf, frameIndicatorsSeq[3], frameIndicatorsLen[3])) return -1;

    LOG_DEBUG_TS("LD2450:");
    LOG_DEBUG(frameBufferBytesRead);
    LOG_DEBUG(":");

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

}
