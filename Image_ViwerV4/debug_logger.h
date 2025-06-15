/**
 * Advanced Debug Logger for ESP32P4
 * 
 * Provides comprehensive logging capabilities with multiple output options
 * and configurable log levels.
 * 
 * Author: Chamil1983
 * Date: 2025-06-14 11:38:45 UTC
 */

#ifndef DEBUG_LOGGER_H
#define DEBUG_LOGGER_H

#include <Arduino.h>
#include <FS.h>
#include <vector>
#include <string>

// Log levels - higher value = less verbose
#define LOG_LEVEL_VERBOSE   0
#define LOG_LEVEL_DEBUG     1
#define LOG_LEVEL_INFO      2
#define LOG_LEVEL_WARN      3
#define LOG_LEVEL_ERROR     4
#define LOG_LEVEL_CRITICAL  5
#define LOG_LEVEL_NONE      6

// Log destinations
#define LOG_DEST_SERIAL     0x01
#define LOG_DEST_FLASH      0x02
#define LOG_DEST_SD         0x04
#define LOG_DEST_CALLBACK   0x08

// Flash log settings
#define LOG_FLASH_FILE      "/syslog.txt"
#define LOG_FLASH_MAX_SIZE  (64*1024)    // 64KB max log file
#define LOG_BUFFER_SIZE     256          // Internal buffer size

// Crash handler settings
#define CRASH_HANDLER_ENABLED 1

class Logger_Class {
private:
    bool _initialized;
    bool _serialEnabled;
    bool _flashEnabled;
    bool _sdEnabled;
    bool _callbackEnabled;
    int _logLevel;
    fs::FS* _fs;
    char _logFilePath[64];
    size_t _maxLogFileSize;
    std::vector<String> _circularBuffer;
    size_t _circularBufferSize;
    uint32_t _logSeqNum;
    bool _crashHandlersSetup;
    
    typedef void (*LogCallback)(const char* msg);
    LogCallback _logCallback;
    
    // Internal methods
    void outputMessage(int level, const char* levelStr, const char* message);
    bool writeToFlash(const char* message);
    bool writeToSD(const char* message);
    void manageLimits();
    void timestampToStr(char* buffer, size_t bufSize);
    
public:
    Logger_Class();
    
    // Basic initialization
    void init(bool enableSerial = true, bool enableFlash = false, int logLevel = LOG_LEVEL_INFO);
    
    // Advanced initialization
    void init(uint8_t destinations, int logLevel = LOG_LEVEL_INFO, fs::FS* fileSystem = nullptr);
    
    // Configure output destinations
    void enableSerialOutput(bool enable);
    void enableFlashOutput(bool enable, fs::FS* fs = nullptr, const char* logPath = LOG_FLASH_FILE);
    void enableSDOutput(bool enable, fs::FS* fs = nullptr, const char* logPath = "/log.txt");
    void setCallbackOutput(LogCallback callback, bool enable = true);
    
    // Configure behavior
    void setLogLevel(int level);
    void setMaxLogFileSize(size_t maxSize);
    void setCircularBufferSize(size_t numEntries);
    
    // Crash handlers
    void setupCrashHandlers();
    void logCrashData(const char* reason);
    
    // Main logging methods
    void verbose(const char* format, ...);
    void debug(const char* format, ...);
    void info(const char* format, ...);
    void warn(const char* format, ...);
    void error(const char* format, ...);
    void critical(const char* format, ...);
    
    // Log with explicit level
    void log(int level, const char* format, ...);
    
    // Special logging
    void raw(const char* message);
    void hexdump(const uint8_t* data, size_t length, const char* prefix = "");
    void memoryStats(const char* tag = "MEM");
    
    // Get circular buffer contents
    std::vector<String> getRecentLogs();
    
    // Flush any buffered logs
    void flush();
};

// Global instance
extern Logger_Class Logger;

// Convenience debug macros
#define DEBUG_PRINT(tag, format, ...) Logger.debug("[%s] " format, tag, ##__VA_ARGS__)
#define DEBUG_INIT(tag) Logger.debug("[%s] Initializing...", tag)
#define DEBUG_SUCCESS(tag) Logger.debug("[%s] Initialization successful", tag)
#define DEBUG_FAIL(tag, reason) Logger.debug("[%s] Initialization failed: %s", tag, reason)

// Assert macro
#define LOG_ASSERT(condition, format, ...) \
    if (!(condition)) { \
        Logger.error("ASSERT FAILED at %s:%d - " format, __FILE__, __LINE__, ##__VA_ARGS__); \
    }

#endif // DEBUG_LOGGER_H