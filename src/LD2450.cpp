
#include "LD2450.hpp"

namespace esphome::ld245x {

/* --------------------------------------------------------------------- */
/*  Concrete LD2450 implementation                                       */
/* --------------------------------------------------------------------- */
LD2450::LD2450()
    : LD245X(
        SensorModel::LD2450, BaudRate::BAUD_256000,
        LD2450_MAX_SENSOR_TARGETS, LD2450_TARGET_SIZE,
        reinterpret_cast<const uint8_t*>("\xFD\xFC\xFB\xFA"),
        reinterpret_cast<const uint8_t*>("\x04\x03\x02\x01"),
        reinterpret_cast<const uint8_t*>("\xAA\xFF"),
        reinterpret_cast<const uint8_t*>("\x55\xCC")),
      _trackingMode(TrackingMode::MultiTargetTracking),
      _zoneFiltering(ZoneFiltering::Disabled)
{
    setFactorySetting();

    _zones.reserve(3);
    for (uint8_t i = 0; i < 3; ++i)
        _zones.emplace_back();
}

/* --------------------------------------------------------------------- */
void LD2450::setFactorySetting()
{
    _baudRate = BaudRate::BAUD_256000;
    _bluetoothEnabled = true;
    _trackingMode = TrackingMode::MultiTargetTracking;
    _zoneFiltering = ZoneFiltering::Disabled;
}

int LD2450::parseRadarFrame()
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

bool LD2450::setTargetTracking(TrackingMode mode)
{
    LOG_INFO_TS("Setting target tracking... \n");
    if (mode == TrackingMode::SingleTargetTracking) {
        return setSingleTargetTracking();
    } else if (mode == TrackingMode::MultiTargetTracking) {
        return setMultiTargetTracking();
    }
    return false;
}

bool LD2450::setSingleTargetTracking()
{
    LOG_INFO_TS("Setting single-target tracking... \n");
    if (send(LD245X_Commands::SetSingleTarget)) {
        _trackingMode = TrackingMode::SingleTargetTracking;
        return true;
    }
    return false;
}

bool LD2450::setMultiTargetTracking()
{
    LOG_INFO_TS("Setting multi-target tracking... \n");
    if (send(LD245X_Commands::SetMultiTarget)) {
        _trackingMode = TrackingMode::MultiTargetTracking;
        return true;
    }
    return false;
}

bool LD2450::queryTargetTrackingMode()
{
    LOG_INFO_TS("Getting target tracking mode... \n");
    if (send(LD245X_Commands::QueryTrackingMode)) {
        if (frameBuffer[4] == 0x01) {
            _trackingMode = TrackingMode::SingleTargetTracking;
        } else if (frameBuffer[4] == 0x02) {
            _trackingMode = TrackingMode::MultiTargetTracking;
        } else {
            _trackingMode = TrackingMode::Unknown;
        }        
        return true;
    }
    return false;
}

bool LD2450::queryZoneFilter()
{
    LOG_INFO_TS("Getting zone filtering... \n");
    if (!send(LD245X_Commands::QueryZoneFilter))
        return false;

    LOG_DEBUG_TS("ZoneFiltering ");
    if (frameBuffer[4] == 0x00) {
        _zoneFiltering = ZoneFiltering::Disabled;
        LOG_DEBUG("Disabled, ");
    }
    else if (frameBuffer[4] == 0x01) {
        _zoneFiltering = ZoneFiltering::AreaOnly;
        LOG_DEBUG("AreaOnly, ");
    }
    else if (frameBuffer[4] == 0x02) {
        _zoneFiltering = ZoneFiltering::AreaExcluded;
        LOG_DEBUG("AreaExcluded, ");
    }
    for(uint8_t i = 0; i < 3; ++i) {
        _zones[i].x1 = decodeSignedWord(frameBuffer[6+i*8], frameBuffer[7+i*8]);
        _zones[i].y1 = decodeSignedWord(frameBuffer[8+i*8], frameBuffer[9+i*8]);
        _zones[i].x2 = decodeSignedWord(frameBuffer[10+i*8], frameBuffer[11+i*8]);
        _zones[i].y2 = decodeSignedWord(frameBuffer[12+i*8], frameBuffer[13+i*8]);
        LOG_DEBUG_F("z%d={x1=%d,y1=%d,x2=%d,y2=%d} ", i+1, _zones[i].x1, _zones[i].y1, _zones[i].x2, _zones[i].y2);
    }
    LOG_DEBUG("\n");
    return true;
}

bool LD2450::setZoneFilter()
{
    return setZoneFilter(_zoneFiltering, _zones[0], _zones[1], _zones[2]);
}

bool LD2450::setZoneFilter(ZoneFiltering zf)
{
    LOG_INFO_TS("Setting zone filtering from internal _zones...\n");
    return setZoneFilter(zf, _zones[0], _zones[1], _zones[2]);
}

bool LD2450::setZoneFilter(ZoneFiltering zf, ZoneCoordinates z1, ZoneCoordinates z2, ZoneCoordinates z3)
{
    LOG_INFO_TS("Setting zone filtering... \n");
    uint8_t payload[26] = {0};
    payload[0] = static_cast<uint8_t>(zf);
    ZoneCoordinates tmp[3] = {z1, z2, z3};

    for(uint8_t i = 2, j = 0; i < sizeof(payload) && j < 3; i+=8, ++j) {
        payload[i]   = lowByte(encodeSignedWord(tmp[j].x1));
        payload[i+1] = highByte(encodeSignedWord(tmp[j].x1));
        payload[i+2] = lowByte(encodeSignedWord(tmp[j].y1));
        payload[i+3] = highByte(encodeSignedWord(tmp[j].y1));
        payload[i+4] = lowByte(encodeSignedWord(tmp[j].x2));
        payload[i+5] = highByte(encodeSignedWord(tmp[j].x2));
        payload[i+6] = lowByte(encodeSignedWord(tmp[j].y2));
        payload[i+7] = highByte(encodeSignedWord(tmp[j].y2));
    }

    if (send(LD245X_Commands::SetZoneFilter, payload, sizeof(payload))) {
        _zoneFiltering = zf;
        for(uint8_t i = 0; i < 3; ++i) {
            _zones[i] = tmp[i];
        }
        return true;
    }
    return false;
}

const char* LD2450::getZoneFilter()
{
    size_t copied = 0;
    if (static_cast<uint8_t>(_zoneFiltering) == 0x00) {
        copied = snprintf(zone_string, sizeof(zone_string), "[\"ZoneFiltering\":\"Disabled\"");
    }
    else if (static_cast<uint8_t>(_zoneFiltering) == 0x01) {
        copied = snprintf(zone_string, sizeof(zone_string), "[\"ZoneFiltering\":\"AreaOnly\"");
    }
    else if (static_cast<uint8_t>(_zoneFiltering) == 0x02) {
        copied = snprintf(zone_string, sizeof(zone_string), "[\"ZoneFiltering\":\"AreaExcluded\"");
    } else {
        copied = snprintf(zone_string, sizeof(zone_string), "[\"ZoneFiltering\":\"AreaExcluded\"");
    }
    for(uint8_t i = 0; i < 3; ++i) {
        zone_string[copied++] = ',';
        copied += snprintf(zone_string+copied, sizeof(zone_string)-copied,
                    "\"z%d\":{\"x1\":%d,\"y1\":%d,\"x2\":%d,\"y2\":%d}",
                    i, _zones[i].x1, _zones[i].y1, _zones[i].x2, _zones[i].y2);
    }
    zone_string[copied++] = ']';
    zone_string[copied++] = '\0';
    LOG_DEBUG_FTS("ZoneFilteringString: %s \n", zone_string);
    return zone_string;
}

}
