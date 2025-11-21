#ifndef __LD245X_DebugTools
#define __LD245X_DebugTools

namespace esphome::ld245x
{
// ============================================================================
// CONFIGURE YOUR DEBUG LEVEL HERE (0 = off, higher = more verbose)
// ============================================================================
// 0 = No debug output at all
// 1 = ERROR only
// 2 = + WARNING
// 3 = + INFO
// 4 = + DEBUG (very verbose)
// 5 = + TRACE (everything, including function entry/exit)
// Optional: define DEBUG_DISABLE_ALL to completely strip all debug code
// #define DEBUG_DISABLE_ALL

#ifndef DEBUG_LEVEL
  #define DEBUG_LEVEL 3
#endif

#if !defined(DEBUG_DISABLE_ALL)

#ifdef DEBUG_COLOR
  #define ANSI_RED     "\x1b[31m"
  #define ANSI_YELLOW  "\x1b[33m"
  #define ANSI_GREEN   "\x1b[32m"
  #define ANSI_BLUE    "\x1b[34m"
  #define ANSI_MAGENTA "\x1b[35m"
  #define ANSI_CYAN    "\x1b[36m"
  #define ANSI_RESET   "\x1b[0m"
#else
  #define ANSI_RED     ""
  #define ANSI_YELLOW  ""
  #define ANSI_GREEN   ""
  #define ANSI_CYAN    ""
  #define ANSI_MAGENTA ""
  #define ANSI_RESET   ""
#endif

// ============================================================================
// Core Debug class - all methods are stripped when level is too low
// ============================================================================
  class Debug {
  public:
    enum Level { ERROR = 1, WARN = 2, INFO = 3, DEBUG = 4, TRACE = 5 };

    template<typename... Args> static void error(const char* format, Args&&... args) {
        println(ERROR, true, format, std::forward<Args>(args)...); }

    template<typename... Args> static void warn(const char* format, Args&&... args) {
        println(WARN, true, format, std::forward<Args>(args)...); }

    template<typename... Args> static void info(const char* format, Args&&... args) {
        println(INFO, true, format, std::forward<Args>(args)...); }

    template<typename... Args> static void debug(const char* format, Args&&... args) {
        println(DEBUG, true, format, std::forward<Args>(args)...); }

    template<typename... Args> static void trace(const char* format, Args&&... args) {
        printf(TRACE, true, format, std::forward<Args>(args)...); Serial.println(); }

    template<typename... Args> static void raw(const char* format, Args&&... args) {
        Serial.printf(format, std::forward<Args>(args)...); }

    static void print_ts(Level level=INFO, bool timeStamp=true) {
        if (level > DEBUG_LEVEL) return; if(timeStamp) {
          Serial.printf("[%010lu] ", millis());
          switch(level) {
              case ERROR: Serial.print(ANSI_RED    "[ERROR] " ANSI_RESET); break;
              case WARN:  Serial.print(ANSI_YELLOW "[WARN] " ANSI_RESET); break;
              case INFO:  Serial.print(ANSI_GREEN  "[INFO] " ANSI_RESET); break;
              case DEBUG: Serial.print(ANSI_CYAN   "[DEBUG] " ANSI_RESET); break;
              case TRACE: Serial.print(ANSI_MAGENTA"[TRACE] " ANSI_RESET); break; }}}

    template<typename... Args> static void printf(Level level, bool timeStamp, const char* format, Args&&... args) {
        if (level > DEBUG_LEVEL) return; if (timeStamp) { print_ts(level, timeStamp); }
        Serial.printf(format, std::forward<Args>(args)...); }

    template<typename... Args> static void print(Level level, bool timeStamp, Args&&... args) {
        if (level > DEBUG_LEVEL) return; if (timeStamp) { print_ts(level, timeStamp); }
        Serial.print(std::forward<Args>(args)...); }

    template<typename... Args> static void println(Level level, bool timeStamp, Args&&... args) {
        print(level, timeStamp, std::forward<Args>(args)...); Serial.println(); }
  };
#endif

  template<typename T> class ObjectCounter {
    public:
      ObjectCounter() { ++instanceCount; }
      ObjectCounter(const ObjectCounter&) : ObjectCounter() { }
      ObjectCounter(ObjectCounter&&) noexcept : ObjectCounter() { }
      ~ObjectCounter() { --instanceCount; }
      static uint32_t count() { return instanceCount; }
    private:
      static uint32_t instanceCount;
  };

  template<typename T> uint32_t ObjectCounter<T>::instanceCount = 0;

  class Trace {
    public:
      Trace(const char* f) : func(f) { Debug::trace("→ %s", func); }
      ~Trace()                       { Debug::trace("← %s", func); }
    private:
      const char* func;
  };

// ============================================================================
// User-friendly macros
// ============================================================================
#if !defined(DEBUG_DISABLE_ALL)
    #define LOG_NEWLINE(...) Debug::println(Debug::INFO, false, __VA_ARGS__)
    #define LOG_ERROR(...)   Debug::print(Debug::ERROR, false, __VA_ARGS__)
    #define LOG_WARN(...)    if (DEBUG_LEVEL >= Debug::WARN)  Debug::print(Debug::WARN,  false,  __VA_ARGS__)
    #define LOG_INFO(...)    if (DEBUG_LEVEL >= Debug::INFO)  Debug::print(Debug::INFO,  false, __VA_ARGS__)
    #define LOG_DEBUG(...)   if (DEBUG_LEVEL >= Debug::DEBUG) Debug::print(Debug::DEBUG, false, __VA_ARGS__)
    #define LOG_TRACE(...)   if (DEBUG_LEVEL >= Debug::TRACE) Debug::print(Debug::TRACE, false, __VA_ARGS__)
    #define LOG_ERROR_TS(...)   Debug::print(Debug::ERROR, true, __VA_ARGS__)
    #define LOG_WARN_TS(...)    if (DEBUG_LEVEL >= Debug::WARN)  Debug::print(Debug::WARN,  true,  __VA_ARGS__)
    #define LOG_INFO_TS(...)    if (DEBUG_LEVEL >= Debug::INFO)  Debug::print(Debug::INFO,  true, __VA_ARGS__)
    #define LOG_DEBUG_TS(...)   if (DEBUG_LEVEL >= Debug::DEBUG) Debug::print(Debug::DEBUG, true, __VA_ARGS__)
    #define LOG_TRACE_TS(...)   if (DEBUG_LEVEL >= Debug::TRACE) Debug::print(Debug::TRACE, true, __VA_ARGS__)
    #define LOG_ERROR_FTS(...)   Debug::printf(Debug::ERROR, true, __VA_ARGS__)
    #define LOG_WARN_FTS(...)    if (DEBUG_LEVEL >= Debug::WARN)  Debug::printf(Debug::WARN,  true,  __VA_ARGS__)
    #define LOG_INFO_FTS(...)    if (DEBUG_LEVEL >= Debug::INFO)  Debug::printf(Debug::INFO,  true, __VA_ARGS__)
    #define LOG_DEBUG_FTS(...)   if (DEBUG_LEVEL >= Debug::DEBUG) Debug::printf(Debug::DEBUG, true, __VA_ARGS__)
    #define LOG_TRACE_FTS(...)   if (DEBUG_LEVEL >= Debug::TRACE) Debug::printf(Debug::TRACE, true, __VA_ARGS__)
    #define LOG_DEBUG_PRINT_BYTES(data, len) \
      do { for(int j = 0; j < len; j++) { if (data[j] < 0x10) LOG_DEBUG('0'); LOG_DEBUG(data[j], HEX); } LOG_DEBUG(""); } while(0);
    #define LOG_DEBUG_PRINTLN_BYTES(data, len) \
      do { for(int j = 0; j < len; j++) { if (data[j] < 0x10) LOG_DEBUG('0'); LOG_DEBUG(data[j], HEX); } LOG_NEWLINE(""); } while(0);
    #define TRACE_FUNC()        if (DEBUG_LEVEL >= Debug::TRACE) Trace __trace(__PRETTY_FUNCTION__)
#else
    #define LOG_NEWLINE(...)    do {} while(0)
    #define LOG_ERROR(...)      do {} while(0)
    #define LOG_WARN(...)       do {} while(0)
    #define LOG_INFO(...)       do {} while(0)
    #define LOG_DEBUG(...)      do {} while(0)
    #define LOG_TRACE(...)      do {} while(0)
    #define LOG_ERROR_TS(...)   do {} while(0)
    #define LOG_WARN_TS(...)    do {} while(0)
    #define LOG_INFO_TS(...)    do {} while(0)
    #define LOG_DEBUG_TS(...)   do {} while(0)
    #define LOG_TRACE_TS(...)   do {} while(0)
    #define LOG_ERROR_FTS(...)   do {} while(0)
    #define LOG_WARN_FTS(...)    do {} while(0)
    #define LOG_INFO_FTS(...)    do {} while(0)
    #define LOG_DEBUG_FTS(...)   do {} while(0)
    #define LOG_TRACE_FTS(...)   do {} while(0)
    #define LOG_DEBUG_PRINT_BYTES(data, len)    do {} while(0)
    #define LOG_DEBUG_PRINTLN_BYTES(data, len)  do {} while(0)
    #define TRACE_FUNC()         do {} while(0)
#endif

}

#endif
