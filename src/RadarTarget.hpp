#ifndef __LD245X_RadarTarget
#define __LD245X_RadarTarget

#if defined(ARDUINO) && ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

namespace esphome::ld245x
{

class RadarTarget {
public:
  RadarTarget() : x(0), y(0), v(0), res(0), d(0), angle(0), id(0), snr(0), valid(false) { };
  RadarTarget(const uint8_t* bytes, uint8_t len, uint8_t id = 0) : RadarTarget() { this->setFromRawBytes(bytes, len, id); };

  int setFromRawBytes(const uint8_t* bytes, uint8_t len, uint8_t id = 0);
  int setFromRaw5Bytes(const uint8_t* bytes, uint8_t len, uint8_t id = 0);

  String format(bool ignoreInvalid=true) const;
  int16_t format(char* buffer, const uint16_t bufferLen, bool ignoreInvalid=true) const;

  String toJson(bool ignoreInvalid=true) const;
  int16_t toJson(char* buffer, const uint16_t bufferLen, bool ignoreInvalid=true) const;
  int16_t toJsonArray(const RadarTarget* targets, uint8_t targetsCount, char* buffer, const int16_t bufferLen, bool ignoreInvalid=true);

  void setValid(bool isValid = true) { valid = isValid; };
  bool isValid() const { return valid; };

  uint8_t  id;    // 0, 1, 2
  int16_t  x, y, v, d; // Position x,y; velocity, distance
  int16_t  angle; // angle [degrees]
  uint16_t res;   // resolution
  uint8_t  snr;   // signal-to-noise ratio
  bool     valid; // is valid target

protected:
  static inline int16_t decodeSignedMag(uint8_t low, uint8_t high) {
    uint16_t raw = word(high, low);
    if (high & 0x80) {
      return (int16_t)(raw - 0x8000);
    } else {
      return (int16_t)(-raw);
    }
  }
};

}

#endif
