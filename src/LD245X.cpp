
#include "LD245X.hpp"

namespace esphome::ld245x
{

/* --------------------------------------------------------------------- */
/*  LD245X – base class implementation                                   */
/* --------------------------------------------------------------------- */
LD245X::LD245X(DataSequenceType dataSequenceType,
               uint8_t maxTargetsCount, uint8_t singleTargetSizeInBytes,
               const uint8_t cmdSeqStart[], const uint8_t cmdSeqEnd[],
               const uint8_t dataSeqStart[], const uint8_t dataSeqEnd[])
    : dataTargetsCount(maxTargetsCount),
      dataTargetSize(singleTargetSizeInBytes),
      type(dataSequenceType),
      frameIndicatorsSeq{
        cmdSeqStart,
        cmdSeqEnd,
        dataSeqStart,
        dataSeqEnd
      },
      frameIndicatorsLen{
        static_cast<const uint8_t>(strlen(reinterpret_cast<const char*>(cmdSeqStart))),
        static_cast<const uint8_t>(strlen(reinterpret_cast<const char*>(cmdSeqEnd))),
        static_cast<const uint8_t>(strlen(reinterpret_cast<const char*>(dataSeqStart))),
        static_cast<const uint8_t>(strlen(reinterpret_cast<const char*>(dataSeqEnd)))
      }
{
    TRACE_FUNC();
    rt.reserve(maxTargetsCount);
    for (uint8_t i = 0; i < maxTargetsCount; ++i)
        rt.emplace_back();
}

/* --------------------------------------------------------------------- */
void LD245X::begin(SERIAL_TYPE &radarUartStream,
                   bool waitReady)
{
    TRACE_FUNC();
    frameBufferBytesRead = 0;
    rs = &radarUartStream;

    if (waitReady && waitForSensorMessage()) {
        LOG_INFO_TS("ld245X, stream available.\n");
    } else if (waitReady) {
        LOG_WARN_TS("ld245X, no stream detected.\n");
    }
}

/* --------------------------------------------------------------------- */
bool LD245X::waitForSensorMessage(bool waitForever)
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
int LD245X::read(bool waitAvailable)
{
    if (!rs) return -2;

    unsigned long start = millis();
    while (millis() - start < 1000) {
        if (!rs->available()) {
          if(waitAvailable) continue;
          return -1;
        }

        /* Try to locate a command start sequence first */
        if (rs->findUntil(reinterpret_cast<const char*>(frameIndicatorsSeq[0]),
                          frameIndicatorsLen[0],
                          reinterpret_cast<const char*>(frameIndicatorsSeq[2]),
                          frameIndicatorsLen[2])) {
            return readDataCommandAck();
        }

        int refreshed = readDataRadarOutput();
        if (refreshed > 0) return refreshed;
    }
    return -1;
}

/* --------------------------------------------------------------------- */
int LD245X::readDataCommandAck()
{
    TRACE_FUNC();
    if (rs->available() < 2) return -4;

    uint8_t tmp[2];
    tmp[0] = rs->read();               // LSB
    tmp[1] = rs->read();               // MSB
    uint16_t payloadLen = word(tmp[1], tmp[0]);

    if (payloadLen > sizeof(frameBuffer)) return -3;

    int read = rs->readBytes(frameBuffer, payloadLen);
    if (read != static_cast<int>(payloadLen)) return -3;

    frameBuffer[read] = '\0';
    frameBufferBytesRead = read;

    uint8_t endBuf[8] = {};                     // enough for any seq
    int seqRead = rs->readBytes(endBuf, frameIndicatorsLen[1]);
    if (seqRead != static_cast<int>(frameIndicatorsLen[1])) return -2;

    if (!matchSequence(endBuf, frameIndicatorsSeq[1], frameIndicatorsLen[1])) return -1;

    /* ---- ACK interpretation ---- */
    // LOG_DEBUG_TS("readCommand: ");
    LOG_DEBUG_PRINT_BYTES(frameBuffer, frameBufferBytesRead);
    if (frameBufferLastCmd < LD245X_CommandsSet::_CommandsCount &&
        CMD_BODY[static_cast<uint8_t>(frameBufferLastCmd)][0] == frameBuffer[0]) {
        if(frameBuffer[1] == 0x01) {
          LOG_DEBUG_TS("readCommand: ack, success\n");
          return 0;
        }
        LOG_ERROR_TS("readCommand: ack, failed\n");
        return -6;
    }
    LOG_ERROR_TS("readCommand: ack error\n");
    return -5;
}

/* --------------------------------------------------------------------- */
bool LD245X::send(LD245X_CommandsSet cmd, const uint8_t* value, uint8_t valueLen)
{
    TRACE_FUNC();
    if (!rs) return false;

    uint8_t idx = static_cast<uint8_t>(cmd);
    if (idx >= static_cast<uint8_t>(LD245X_CommandsSet::_CommandsCount))
        return false;

    if (CMD_BODY[idx][1] - 0x02 != valueLen) {
        LOG_ERROR_TS("LD245X::send: payload length mismatch\n");
        return false;
    }

    if (!sendCommand(cmd, value, valueLen)) return false;

    frameBufferLastCmd = cmd;
    unsigned long start = millis();
    while (millis() - start < 1000) {
        if (read() == 0) return true;
    }
    return false;
}

/* --------------------------------------------------------------------- */
bool LD245X::sendCommand(LD245X_CommandsSet cmd,
                         const uint8_t* value,
                         uint8_t valueLen)
{
    TRACE_FUNC();
    uint8_t idx = static_cast<uint8_t>(cmd);
    uint8_t payload[128] = {};

    payload[0] = CMD_BODY[cmd][1];   // length LSB
    payload[1] = 0x00;              // length MSB (always 0 for now)
    payload[2] = CMD_BODY[cmd][0];   // command byte
    payload[3] = 0x00;              // reserved

    if (valueLen) memcpy(payload + 4, value, valueLen);

    uint8_t total = 4 + valueLen;
    payload[total] = '\0';
    uint8_t toSend = frameIndicatorsLen[0] + total + frameIndicatorsLen[1];
    
    LOG_DEBUG_FTS("Sending command (%d bytes): ", toSend);
    LOG_DEBUG_PRINT_BYTES(payload, total);
    LOG_DEBUG("\n");

    uint8_t sent = 0;
    sent += rs->write(frameIndicatorsSeq[0], frameIndicatorsLen[0]);
    sent += rs->write(payload, total);
    sent += rs->write(frameIndicatorsSeq[1],   frameIndicatorsLen[1]);

    return sent == toSend;
}

/* --------------------------------------------------------------------- */
void LD245X::printTargets() const
{
    LOG_INFO_TS("");
    for (uint8_t i = 0; i < dataTargetsCount; ++i) {
        LOG_INFO(rt[i].format());
        LOG_INFO(" ");
    }
    LOG_INFO("\n");
}

bool LD245X::getFirmwareVersion()
{
    if(strlen(firmware) < 9) {
        if(queryFirmwareVersion()) {
            return true;
        }
    }
    return false;
}
/* --------------------------------------------------------------------- */
/*  Configuration helpers                                                */
/* --------------------------------------------------------------------- */
bool LD245X::beginConfigurationSession()
{
    TRACE_FUNC();
    LOG_INFO_TS("Entering configuration mode... \n");
    const uint8_t payload[] = {0x01, 0x00};
    return send(LD245X_CommandsSet::beginConfigurationSession,
                payload, sizeof(payload));
}

bool LD245X::endConfigurationSession()
{
    TRACE_FUNC();
    LOG_INFO_TS("Exiting configuration mode... \n");
    return send(LD245X_CommandsSet::endConfigurationSession);
}

bool LD245X::setSingleTargetTracking()
{
    TRACE_FUNC();
    LOG_INFO_TS("Setting single-target tracking... \n");
    return send(LD245X_CommandsSet::setSingleTargetTracking);
}

bool LD245X::setMultiTargetTracking()
{
    TRACE_FUNC();
    LOG_INFO_TS("Setting multi-target tracking... \n");
    return send(LD245X_CommandsSet::setMultiTargetTracking);
}

bool LD245X::queryFirmwareVersion()
{
    TRACE_FUNC();
    LOG_INFO_TS("Getting firmware version... \n");
    if (!send(LD245X_CommandsSet::getFirmwareVersion))
        return false;

    if (frameBufferBytesRead < 12) return false;

    snprintf(firmware, sizeof(firmware),
             "V%X.%02X.%02X%02X%02X%02X",
             frameBuffer[7], frameBuffer[6],
             frameBuffer[11], frameBuffer[10],
             frameBuffer[9], frameBuffer[8]);

    LOG_INFO_FTS("Firmware: %s\n", firmware);

    return true;
}

}
