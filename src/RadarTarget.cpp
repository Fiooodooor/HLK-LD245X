#include "RadarTarget.hpp"
#include "LD245X_Config.h"
#include <math.h>

namespace esphome::ld245x
{

int RadarTarget::setFromRawBytes(const uint8_t* bytes, uint8_t len, uint8_t id)
{
  if (!bytes || len < 8) return -2;

  this->id = id;
  this->x = decodeSignedMag(bytes[0], bytes[1]);
  this->y = decodeSignedMag(bytes[2], bytes[3]);
  this->v = decodeSignedMag(bytes[4], bytes[5]);
  this->res = word(bytes[7], bytes[6]);

#if LD245X_USE_FAST_MATH
  // Optimized distance calculation using hypot (faster than sqrt(pow))
  this->d = (uint16_t)roundf(hypotf((float)this->x, (float)this->y));

  // Optimized angle calculation
  #if LD245X_USE_INTEGER_MATH
    // Fast integer approximation for platforms without FPU
    if (this->x == 0) {
      this->angle = (this->y > 0) ? 0 : 180;
    } else if (this->y == 0) {
      this->angle = (this->x > 0) ? 90 : -90;
    } else {
      // Integer arctangent approximation
      int32_t abs_x = abs(this->x);
      int32_t abs_y = abs(this->y);
      int16_t angle_deg;

      if (abs_x > abs_y) {
        angle_deg = 45 - (45 * (abs_x - abs_y)) / (abs_x + abs_y);
      } else {
        angle_deg = 45 + (45 * (abs_y - abs_x)) / (abs_x + abs_y);
      }

      // Adjust for quadrant
      if (this->x >= 0 && this->y >= 0) this->angle = 90 - angle_deg;
      else if (this->x < 0 && this->y >= 0) this->angle = angle_deg - 90;
      else if (this->x < 0 && this->y < 0) this->angle = 270 - angle_deg;
      else this->angle = 90 + angle_deg;
    }
  #else
    // Fast path for cardinal directions
    if (this->x == 0) {
      this->angle = (this->y > 0) ? 0 : 180;
    } else if (this->y == 0) {
      this->angle = (this->x > 0) ? 90 : -90;
    } else {
      // Use atan2f (single precision is faster and sufficient)
      this->angle = (int16_t)roundf(-(atan2f((float)this->y, (float)this->x) * (180.0f / M_PI) - 90.0f));
    }
  #endif
#else
  // Original implementation
  this->d = (uint16_t)roundf(sqrtf(powf(this->x, 2.0f) + powf(this->y, 2.0f)));
  this->angle = (int16_t)roundf(-(atan2(y, x) * (180 / M_PI) - 90));
#endif

  this->valid = (this->res > 0) ? true : false;

  return 0;
}

int RadarTarget::setFromRaw5Bytes(const uint8_t* bytes, uint8_t len, uint8_t id)
{
  if (!bytes || len < 5) return -2;

  this->id = id;
  this->angle = (int16_t)(bytes[0]-0x80);
  this->d = bytes[1];
  this->v = bytes[3]*(bytes[2]==0x00?1:-1);
  this->x = (int16_t)roundf(this->d*cos(this->angle*(180.0f / M_PI)));
  this->y = (int16_t)roundf(this->d*sin(this->angle*(180.0f / M_PI)));
  this->snr = bytes[4];
  this->valid = true;
  return 0;
}

int16_t RadarTarget::format(char* buffer, const uint16_t bufferLen, bool ignoreInvalid) const
{
  if (!ignoreInvalid && !this->isValid())
    return 0;
  if (!buffer || bufferLen < 30)
    return -1;

  return static_cast<int16_t>(snprintf(buffer, bufferLen,
            "T:%d:x=%+d:y=%+d:v=%+d:d=%d:l=%d:valid=%d",
            this->id, this->x, this->y, this->v, this->d, this->angle, this->valid));
}

String RadarTarget::format(bool ignoreInvalid) const
{
  char buffer[256] = {'\0'};
  format(buffer, sizeof(buffer), ignoreInvalid);
  return String(buffer);
}

String RadarTarget::toJson(bool ignoreInvalid) const
{
  char buffer[256] = {'\0'};
  int16_t charsCopied = toJson(buffer, sizeof(buffer), ignoreInvalid);
  if(charsCopied > 0) {
    return String(buffer);
  }
  return String("");
}

int16_t RadarTarget::toJson(char* buffer, const uint16_t bufferLen, bool ignoreInvalid) const
{
  if (!ignoreInvalid && !this->isValid())
    return 0;
  if (!buffer || bufferLen < 30)
    return -1;

  return snprintf(buffer, bufferLen,
     "{\"id\":%d,\"x\":%d,\"y\":%d,\"velocity\":%d,\"distance\":%d,"
      "\"angle\":%d,\"resolution\":%d,\"snr\":%d,\"valid\":%s}",
      this->id, this->x, this->y, this->v, this->d, this->angle,
      this->res, this->snr, this->valid?"true":"false");
}

int16_t RadarTarget::toJsonArray(const RadarTarget* targets, uint8_t targetsCount, char* buffer, const int16_t bufferLen, bool ignoreInvalid)
{
  int16_t i, tmp, bytes_read = 1, bytes_left = bufferLen-1;

  if (!buffer || bufferLen <= 0)
    return -1;

  buffer[0] = '[';

  for (i = 0; i < targetsCount && bytes_read < bytes_left; ++i) {
    if (ignoreInvalid && !targets[i].isValid()) continue;
    if(bytes_read>2) buffer[bytes_read++] = ',';

    if((tmp = targets[i].toJson(buffer+bytes_read,bytes_left-bytes_read, ignoreInvalid)) >= 0)
      bytes_read += tmp;
    else
      return -1;
  }

  buffer[min((int16_t)(bufferLen-2), bytes_read)] = ']';
  ++bytes_read;
  buffer[min((int16_t)(bufferLen-1), bytes_read)] = '\0';
  ++bytes_read;

  if(bytes_read <= bufferLen)
    return bytes_read;

  return bufferLen - bytes_read;
}

}
