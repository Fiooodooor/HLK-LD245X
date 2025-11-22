
#include "LD245X.hpp"

namespace esphome::ld245x
{

/* --------------------------------------------------------------------- */
/*  LD245X – base class implementation                                   */
/* --------------------------------------------------------------------- */
LD245X::LD245X(SensorModel sensorModel, BaudRate baudRate,
               uint8_t maxTargetsCount, uint8_t singleTargetSizeInBytes,
               const uint8_t cmdSeqStart[], const uint8_t cmdSeqEnd[],
               const uint8_t dataSeqStart[], const uint8_t dataSeqEnd[])
    : dataTargetsCount(maxTargetsCount),
      dataTargetSize(singleTargetSizeInBytes),
      _sensorModel(sensorModel),
      _baudRate(baudRate),
      _lastCmd(LD245X_Commands::_Count),
      _bluetoothEnabled(true),
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
    setNameString();
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
bool LD245X::update()
{
    if(read() >= 0)
        return true;

    return false;
}

/* --------------------------------------------------------------------- */
uint8_t LD245X::getNrValidTargets() const
{
    return nrValidTargets;
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

        int8_t result = parseRadarFrame();
        if (result > 0) {
            nrValidTargets = result;
            return nrValidTargets;
        }
    }
    return -1;
}

/* --------------------------------------------------------------------- */
int LD245X::readDataCommandAck()
{
    TRACE_FUNC();
    if (rs->available() < 4) return -4;

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
    LOG_DEBUG_PRINT_BYTES(frameBuffer, frameBufferBytesRead);
    if (_lastCmd < LD245X_Commands::_Count) {
        if(COMMAND_TABLE[static_cast<uint8_t>(_lastCmd)].cmd_byte == frameBuffer[0] &&
           frameBuffer[1] == 0x01) 
        {
            if(frameBuffer[3] == 0x00 && frameBuffer[2] == 0x00) {
              LOG_DEBUG_TS("readCommand: ack=true, result=success\n");
              return 0;
            } else {
              LOG_DEBUG_TS("readCommand: ack=true, result=failed\n");
              return -1;
            }
        }
    }
    LOG_DEBUG_TS("readCommand: _lastCmd was not set or response is unexpected.\n");
    return -2;
}

/* --------------------------------------------------------------------- */
bool LD245X::send(LD245X_Commands cmd, const uint8_t* payload, size_t payload_len)
{
    TRACE_FUNC();
    if (!rs) {
        LOG_ERROR_TS("Stream UART is a nullpointer.\n");
        return false;
    }

    if (cmd >= LD245X_Commands::_Count) {
        LOG_ERROR_FTS("Invalid command enum: %d\n", static_cast<int>(cmd));
        return false;
    }

    const auto& def = COMMAND_TABLE[static_cast<uint8_t>(cmd)];
    if (def.payload_bytes != payload_len) {
        LOG_ERROR_FTS("Command %s payload length mismatch: expected %u, got %zu\n",
                 def.name, def.payload_bytes, payload_len);
        return false;
    }

    if (!sendRawCommand(cmd, payload, payload_len)) {
        return false;
    }

    _lastCmd = cmd;
    unsigned long start = millis();
    while (millis() - start < 1000) {
        if (read() == 0) return true;
    }
    return false;
}

/* --------------------------------------------------------------------- */
bool LD245X::sendRawCommand(LD245X_Commands cmd, const uint8_t* payload, size_t payload_len)
{
    TRACE_FUNC();

    const auto& def = COMMAND_TABLE[static_cast<uint8_t>(cmd)];
    uint8_t frame[256];
    size_t pos = 0;

    // Header
    memcpy(&frame[pos], frameIndicatorsSeq[0], frameIndicatorsLen[0]);
    pos += frameIndicatorsLen[0];

    // Length (little-endian)
    frame[pos++] = payload_len + 2;  // +2 for cmd byte + reserved
    frame[pos++] = 0x00;

    // Command byte + reserved
    frame[pos++] = def.cmd_byte;
    frame[pos++] = 0x00;

    // Payload
    if (payload_len > 0) {
        memcpy(&frame[pos], payload, payload_len);
        pos += payload_len;
    }

    // Footer
    memcpy(&frame[pos], frameIndicatorsSeq[1], frameIndicatorsLen[1]);
    pos += frameIndicatorsLen[1];

    LOG_DEBUG_FTS("Sending command %s (%zu bytes), raw bytes: ", def.name, pos);
    LOG_DEBUG_PRINT_BYTES(frame, pos);
    LOG_DEBUG("\n");

    size_t written = rs->write(frame, pos);
    rs->flush();

    return written == pos;
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
/* --------------------------------------------------------------------- */
void LD245X::setNameString(const char* name)
{
    if(name != nullptr) {
        snprintf(name_string, sizeof(name_string), "%s", name);
    }
    else if(_sensorModel == SensorModel::LD2450) {
        snprintf(name_string, sizeof(name_string),
                 "ld2450_%d", ObjectCounter::count());
    }
    else if(_sensorModel == SensorModel::LD2451) {
        snprintf(name_string, sizeof(name_string),
                 "ld2451_%d", ObjectCounter::count());
    } else {
        snprintf(name_string, sizeof(name_string),
                 "ld245x_%d", ObjectCounter::count());
    }
}

/* --------------------------------------------------------------------- */
const char* LD245X::getNameString() const
{
    LOG_DEBUG_FTS("Name string is %s\n", name_string);
    return name_string;
}

/* --------------------------------------------------------------------- */
const char* LD245X::getFirmwareString() const
{
    LOG_DEBUG_FTS("Firmware string is %s\n", firmware_string);
    return firmware_string;
}

/* --------------------------------------------------------------------- */
const char* LD245X::getMacAddressString() const
{
    LOG_DEBUG_FTS("Mac address string is %s\n", mac_string);
    return mac_string;
}

/* --------------------------------------------------------------------- */
/*  Configuration helpers                                                */
/* --------------------------------------------------------------------- */
bool LD245X::beginConfigurationSession()
{
    LOG_INFO_TS("Entering configuration mode... \n");
    const uint8_t payload[] = {0x01, 0x00};
    return send(LD245X_Commands::EnterConfig, payload, sizeof(payload));
}

/* --------------------------------------------------------------------- */
bool LD245X::endConfigurationSession()
{
    LOG_INFO_TS("Exiting configuration mode... \n");
    return send(LD245X_Commands::ExitConfig);
}

/* --------------------------------------------------------------------- */
bool LD245X::queryFirmwareVersion()
{
    LOG_INFO_TS("Getting firmware version... \n");
    if (!send(LD245X_Commands::QueryFirmwareVersion))
        return false;

    if (frameBufferBytesRead < 12) return false;

    firmwareType = word(frameBuffer[5], frameBuffer[4]);
    snprintf(firmware_string, sizeof(firmware_string),
             "V%X.%02X.%02X%02X%02X%02X",
             frameBuffer[7], frameBuffer[6],
             frameBuffer[11], frameBuffer[10],
             frameBuffer[9], frameBuffer[8]);

    LOG_DEBUG_FTS("Firmware: %s\n", firmware_string);

    return true;
}

/* --------------------------------------------------------------------- */
bool LD245X::setSerialPortBaudRate(BaudRate baudRate)
{
    LOG_INFO_TS("Setting serial port baud rate...\n");
    if (baudRate >= BaudRate::Unknown) {
        LOG_ERROR_FTS("Invalid serial port baud rate value %u.\n", baudRate);
        return false;
    }
    const uint8_t payload[] = {static_cast<uint8_t>(baudRate), 0x00};
    if (send(LD245X_Commands::SetBaudRate, payload, sizeof(payload))) {
        _baudRate = baudRate;
        LOG_INFO_TS("New serial port baud rate require device reset.\n");
        return true;
    }
    return false;
}

/* --------------------------------------------------------------------- */
bool LD245X::restoreFactorySetting()
{
    LOG_INFO_TS("Restoring factory settings...\n");
    if (send(LD245X_Commands::FactoryReset)) {
        setFactorySetting();
        return true;
    }
    return false;
}

/* --------------------------------------------------------------------- */
bool LD245X::rebootModule()
{
    LOG_INFO_TS("Restarting module...\n");
    return send(LD245X_Commands::Reboot);
}

/* --------------------------------------------------------------------- */
bool LD245X::setBluetoothEnabled(bool enabled)
{
    LOG_INFO_TS("Setting bluetooth state...\n");
    const uint8_t payload[] = {static_cast<uint8_t>(enabled==true?0x01:0x00), 0x00};
    return send(LD245X_Commands::SetBluetooth, payload, sizeof(payload));
}

/* --------------------------------------------------------------------- */
bool LD245X::queryMacAddress()
{
    LOG_INFO_TS("Getting mac address...\n");
    const uint8_t payload[] = {0x01, 0x00};
    if (!send(LD245X_Commands::QueryMAC, payload, sizeof(payload)))
        return false;

    for (int i = 0; i < 6; ++i) {
        mac_bytes[i] = frameBuffer[4 + i];
    }

    snprintf(mac_string, sizeof(mac_string),
             "%02X:%02X:%02X:%02X:%02X:%02X",
             mac_bytes[0], mac_bytes[1],
             mac_bytes[2], mac_bytes[3],
             mac_bytes[4], mac_bytes[5]);

    LOG_DEBUG_FTS("Mac address %s\n", mac_string);
    return true;
}

}
