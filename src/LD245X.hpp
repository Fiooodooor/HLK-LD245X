#ifndef __LD245X_hpp
#define __LD245X_hpp

#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266) || defined(FORCE_HARDWARE_SERIAL)
  #include <HardwareSerial.h>
  #define SERIAL_TYPE HardwareSerial
#else
  #include <SoftwareSerial.h>
  #define SERIAL_TYPE SoftwareSerial
#endif

#include "RadarTarget.hpp"
#include "Debug.hpp"

#include <vector>
#include <map>

namespace esphome::ld245x {

#define LD2450_RADAR_MAX_RANGE 5000  // mm (5 meters)
#define LD2450_MAX_SENSOR_TARGETS 3
#define LD2450_TARGET_SIZE 8
#define LD2450_SERIAL_BUFFER 256
#define LD2450_SERIAL_SPEED 256000
#define LD2450_DEFAULT_RETRY_COUNT_FOR_WAIT_FOR_MSG 1000

#define LD2451_RADAR_MAX_RANGE 50  // m (50 meters)
#define LD2451_MAX_SENSOR_TARGETS 3
#define LD2451_TARGET_SIZE 5
#define LD2451_SERIAL_BUFFER 256
#define LD2451_SERIAL_SPEED 115200
#define LD2451_DEFAULT_RETRY_COUNT_FOR_WAIT_FOR_MSG 1000

enum class LD245X_Commands : uint8_t {
    EnterConfig           = 0,
    ExitConfig,
    SetSingleTarget,
    SetMultiTarget,
    QueryTrackingMode,
    QueryFirmwareVersion,
    SetBaudRate,
    FactoryReset,
    Reboot,
    SetBluetooth,
    QueryMAC,
    QueryZoneFilter,
    SetZoneFilter,
    _Count
};

struct CommandDef {
    const char* name;
    uint8_t     cmd_byte;
    uint8_t     payload_bytes;  // excluding cmd byte & reserved
};

inline constexpr CommandDef COMMAND_TABLE[] = {
    { "EnterConfig",         0xFF, 2 },
    { "ExitConfig",          0xFE, 0 },
    { "SetSingleTarget",     0x80, 0 },
    { "SetMultiTarget",      0x90, 0 },
    { "QueryTrackingMode",   0x91, 0 },
    { "QueryFirmwareVersion",0xA0, 0 },
    { "SetBaudRate",         0xA1, 2 },
    { "FactoryReset",        0xA2, 0 },
    { "Reboot",              0xA3, 0 },
    { "SetBluetooth",        0xA4, 2 },
    { "QueryMAC",            0xA5, 2 },
    { "QueryZoneFilter",     0xC1, 0 },
    { "SetZoneFilter",       0xC2, 26 },
};

enum class SensorModel : uint8_t {
    LD2450,
    LD2451,
    Unknown
};

enum class BaudRate : uint8_t
{
    BAUD_9600 = 0x01,
    BAUD_19200 = 0x02,
    BAUD_38400 = 0x03,
    BAUD_57600 = 0x04,
    BAUD_115200 = 0x05,
    BAUD_230400 = 0x06,
    BAUD_256000 = 0x07,
    BAUD_460800 = 0x08,
    Unknown
};

/* --------------------------------------------------------------------- */
class LD245X : public ObjectCounter<LD245X>
{
public:
    /* ----- ctor ------------------------------------------------------ */
    LD245X() : LD245X(SensorModel::LD2450) { };
    LD245X(SensorModel sensorModel, BaudRate baudRate=BaudRate::BAUD_256000,
           uint8_t maxTargetsCount=LD2450_MAX_SENSOR_TARGETS,
           uint8_t singleTargetSizeInBytes=LD2450_TARGET_SIZE,
           const uint8_t cmdSeqStart[]=reinterpret_cast<const uint8_t*>("\xFD\xFC\xFB\xFA"),
           const uint8_t cmdSeqEnd[]=reinterpret_cast<const uint8_t*>("\x04\x03\x02\x01"),
           const uint8_t dataSeqStart[]=reinterpret_cast<const uint8_t*>("\xAA\xFF"),
           const uint8_t dataSeqEnd[]=reinterpret_cast<const uint8_t*>("\x55\xCC"));

    virtual ~LD245X() = default;

    /* ----- public API ------------------------------------------------ */
    void  begin(SERIAL_TYPE &radarStream, bool waitReady = true);
    bool  waitForSensorMessage(bool waitForever = false);

    bool  update();
    uint8_t getNrValidTargets() const;

    RadarTarget getTarget(uint8_t target_id) const { return (target_id < getSensorSupportedTargetCount()) ? rt[target_id] : RadarTarget(); }
    uint8_t    getSensorSupportedTargetCount() const { return dataTargetsCount; }
    void  printTargets() const;

    const char* getNameString() const;
    const char* getFirmwareString() const;
    const char* getMacAddressString() const;

    /* ----- configuration helpers ------------------------------------- */
    bool beginConfigurationSession();
    bool endConfigurationSession();
    bool queryFirmwareVersion();
    bool setSerialPortBaudRate(BaudRate baudRate);
    bool restoreFactorySetting();
    bool rebootModule();
    bool setBluetoothEnabled(bool enabled=true);
    bool queryMacAddress();

    static inline uint16_t encodeSignedWord(int16_t value)
    {
        if (value < 0) {
            return static_cast<uint16_t>(-value) | 0x8000u;
        }
        return static_cast<uint16_t>(value) & 0x7FFFu;
    }

    static inline int16_t decodeSignedWord(uint8_t low, uint8_t high)
    {
      uint16_t raw = word(high, low);
      if (high & 0x80) {
        return (int16_t)(raw - 0x8000);
      }
      return (int16_t)(-raw);
    }

    /* ----- virtual data parser --------------------------------------- */
    virtual void setFactorySetting() = 0;
    virtual int parseRadarFrame() = 0;

protected:
    int   read(bool waitAvailable=true);
    bool  send(LD245X_Commands cmd, const uint8_t* payload = nullptr, size_t payload_len = 0);
    bool  sendRawCommand(LD245X_Commands cmd, const uint8_t* payload = nullptr, size_t payload_len = 0);
    void  setNameString(const char* name = nullptr);

    /* ----- data ------------------------------------------------------ */
    Stream* rs = nullptr;
    std::vector<RadarTarget> rt;
    uint8_t nrValidTargets = 0;
    SensorModel _sensorModel = SensorModel::Unknown;
    BaudRate _baudRate = BaudRate::BAUD_256000;
    LD245X_Commands _lastCmd = LD245X_Commands::_Count;
    bool _bluetoothEnabled = true;

    const uint8_t dataTargetsCount;
    const uint8_t dataTargetSize;

    int frameBufferBytesRead = 0;

    char name_string[20]     = {'\0'};
    char firmware_string[20] = {'\0'};
    int16_t firmwareType     = {0};
    uint8_t mac_bytes[6]     = {0};
    char    mac_string[18]   = {'\0'};
    
    uint8_t frameBuffer[512] = {'\0'};

    /* ----- sequence buffers (copied in ctor) -------------------------- */
    const uint8_t* frameIndicatorsSeq[4];
    const uint8_t  frameIndicatorsLen[4];

    /* ----- helpers --------------------------------------------------- */
    int  readDataCommandAck();
    inline bool matchSequence(const uint8_t* data, const uint8_t* seq, size_t len) const {
        return memcmp(data, seq, len) == 0;
    }

};

}

#endif
