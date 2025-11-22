
#ifndef __LD2450_hpp
#define __LD2450_hpp

#include "LD245X.hpp"

namespace esphome::ld245x {

enum class TrackingMode : uint8_t {
    SingleTargetTracking = 0x01,
    MultiTargetTracking = 0x02,
    Unknown
};

enum class ZoneFiltering : uint8_t {
    Disabled = 0x00,
    AreaOnly = 0x01,
    AreaExcluded = 0x02,
    Unknown
};

struct ZoneCoordinates {
    int16_t     x1 = {0};
    int16_t     y1 = {0};
    int16_t     x2 = {0};
    int16_t     y2 = {0};
};

/* --------------------------------------------------------------------- */
/*  Concrete sensor implementations                                      */
/* --------------------------------------------------------------------- */
class LD2450 : public LD245X {
public:
    LD2450();
    void setFactorySetting() override;
    int parseRadarFrame() override;
    bool setTargetTracking(TrackingMode mode);
    bool setSingleTargetTracking();
    bool setMultiTargetTracking();
    bool queryTargetTrackingMode();
    bool queryZoneFilter();
    bool setZoneFilter();
    bool setZoneFilter(ZoneFiltering zf);
    bool setZoneFilter(ZoneFiltering zf, ZoneCoordinates z1, ZoneCoordinates z2, ZoneCoordinates z3);
    const char* getZoneFilter();

protected:
    TrackingMode _trackingMode = TrackingMode::MultiTargetTracking;
    std::vector<ZoneCoordinates> _zones;
    ZoneFiltering _zoneFiltering = ZoneFiltering::Disabled;
    char zone_string[512] = {'\0'};
};

}

#endif
