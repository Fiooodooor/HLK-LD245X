#ifndef __LD245X_hpp
#define __LD245X_hpp

#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
  #include <HardwareSerial>
  #define SERIAL_TYPE HardwareSerial
#else
  #include <SoftwareSerial>
  #define SERIAL_TYPE SoftwareSerial
#endif

#include "RadarTarget.hpp"
#include "debug.hpp"

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


enum LD245X_CommandsSet {
  beginConfigurationSession,
  endConfigurationSession,
  setSingleTargetTracking,
  setMultiTargetTracking,
  getCurrentTargetTracking,
  getFirmwareVersion,
  setSerialPortBaudRate,
  restoreFactorySettings,
  restartModule,
  setBluetoothSetup,
  getMacAddress,
  getCurrentZoneFiltering,
  setCurrentZoneFiltering,
  _CommandsCount
};

enum DataSequenceType {
  HLK_LD2450,
  HLK_LD2451,
  HLK_LDUnknown,
  HLK_DataSequenceTypeCount
};

enum BaudRate
{
    BAUD_9600 = 0x01,
    BAUD_19200 = 0x02,
    BAUD_38400 = 0x03,
    BAUD_57600 = 0x04,
    BAUD_115200 = 0x05,
    BAUD_230400 = 0x06,
    BAUD_256000 = 0x07,
    BAUD_460800 = 0x08,
};

static constexpr uint8_t CMD_BODY[static_cast<uint8_t>(LD245X_CommandsSet::_CommandsCount)][2] = {
    {0xFF, 0x04}, // beginConfigurationSession
    {0xFE, 0x02}, // endConfigurationSession
    {0x80, 0x02}, // setSingleTargetTracking
    {0x90, 0x02}, // setMultiTargetTracking
    {0x91, 0x02}, // getCurrentTargetTracking
    {0xA0, 0x02}, // getFirmwareVersion
    {0xA1, 0x04}, // setSerialPortBaudRate,
    {0xA2, 0x02}, // restoreFactorySettings,
    {0xA3, 0x02}, // restartModule,
    {0xA4, 0x04}, // setBluetoothSetup,
    {0xA5, 0x04}, // getMacAddress,
    {0xC1, 0x02}, // getCurrentZoneFiltering,
    {0xC2, 0x1C}  // setCurrentZoneFiltering,
};

/* --------------------------------------------------------------------- */
class LD245X : public ObjectCounter<LD245X>
{
public:
    /* ----- ctor ------------------------------------------------------ */
    LD245X() : LD245X(HLK_LD2450) { };
    LD245X(DataSequenceType dataSequenceType,
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
    int   processSerialDataIntoRadarData(uint8_t rec_buf[], int len);

    int   read(bool waitAvailable=true);

    bool  send(LD245X_CommandsSet cmd,
               const uint8_t* value = nullptr,
               uint8_t valueLen = 0);

    bool  sendCommand(LD245X_CommandsSet cmd,
                      const uint8_t* value = nullptr,
                      uint8_t valueLen = 0);

    RadarTarget getTarget(uint8_t target_id) const { return (target_id < dataTargetsCount) ? rt[target_id] : RadarTarget(); }
    uint8_t    getSensorSupportedTargetCount() const { return dataTargetsCount; }
    void  printTargets() const;
    bool getFirmwareVersion();

    /* ----- configuration helpers ------------------------------------- */
    bool beginConfigurationSession();
    bool endConfigurationSession();
    bool setSingleTargetTracking();
    bool setMultiTargetTracking();
    bool queryFirmwareVersion();

    /* ----- virtual data parser --------------------------------------- */
    virtual int readDataRadarOutput() = 0;

protected:
    /* ----- data ------------------------------------------------------ */
    Stream* rs = nullptr;
    std::vector<RadarTarget> rt;
    DataSequenceType type = HLK_LDUnknown;
    LD245X_CommandsSet frameBufferLastCmd = LD245X_CommandsSet::_CommandsCount;

    const uint8_t dataTargetsCount;
    const uint8_t dataTargetSize;

    int frameBufferBytesRead = 0;

    char name[20]            = {'\0'};
    char firmware[20]        = {'\0'};
    uint8_t frameBuffer[512] = {'\0'};

    /* ----- sequence buffers (copied in ctor) -------------------------- */
    const uint8_t* frameIndicatorsSeq[4];
    const uint8_t  frameIndicatorsLen[4];

    /* ----- helpers --------------------------------------------------- */
    int  readDataCommandAck();
    inline bool matchSequence(const uint8_t* buf, const uint8_t* seq, uint8_t len) const {
        for (uint8_t i = 0; i < len; ++i)
            if (buf[i] != seq[i]) return false;
        return true;
    }
};

}

#endif
