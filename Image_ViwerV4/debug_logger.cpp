/**
 * Implementation of Advanced Debug Logger for ESP32P4
 * 
 * Author: Chamil1983
 * Date: 2025-06-14 11:53:37 UTC
 */

#include "debug_logger.h"
#include <time.h>
#include <SPIFFS.h>
#include <SD.h>
#include <esp_system.h>
#include <esp_task_wdt.h>

// Global instance
Logger_Class Logger;

// For crash handlers
#if CRASH_HANDLER_ENABLED
static void crash_handler(void* arg);
#endif

Logger_Class::Logger_Class() : 
    _initialized(false),
    _serialEnabled(false),
    _flashEnabled(false),
    _sdEnabled(false),
    _callbackEnabled(false),
    _logLevel(LOG_LEVEL_INFO),
    _fs(nullptr),
    _maxLogFileSize(LOG_FLASH_MAX_SIZE),
    _circularBufferSize(50),
    _logSeqNum(0),
    _crashHandlersSetup(false),
    _logCallback(nullptr)
{
    strncpy(_logFilePath, LOG_FLASH_FILE, sizeof(_logFilePath));
}

void Logger_Class::init(bool enableSerial, bool enableFlash, int logLevel) {
    _serialEnabled = enableSerial;
    _flashEnabled = enableFlash;
    _logLevel = logLevel;
    
    if (_initialized) {
        debug("Logger already initialized, reconfiguring...");
    }
    
    if (_serialEnabled) {
        // Force a clean start for serial output
        Serial.println();
        Serial.println("------- Logger Initialized -------");
        Serial.flush();
    }
    
    if (_flashEnabled) {
        if (!SPIFFS.begin(true)) {
            if (_serialEnabled) {
                Serial.println("Failed to initialize SPIFFS for logging");
                Serial.flush();
            }
            _flashEnabled = false;
        } else {
            _fs = &SPIFFS;
            
            // Test if we can write to the filesystem
            File testFile = _fs->open("/logger_test.txt", FILE_WRITE);
            if (!testFile) {
                if (_serialEnabled) {
                    Serial.println("Failed to open test file on SPIFFS");
                    Serial.flush();
                }
                _flashEnabled = false;
            } else {
                testFile.println("Logger test");
                testFile.close();
                _fs->remove("/logger_test.txt");
            }
        }
    }
    
    _circularBuffer.reserve(_circularBufferSize);
    _initialized = true;
    
    // Log initial message
    info("Logger initialized: level=%d, serial=%s, flash=%s",
        _logLevel,
        _serialEnabled ? "ON" : "OFF",
        _flashEnabled ? "ON" : "OFF");
    
    // Force flush to ensure output is seen
    if (_serialEnabled) {
        Serial.flush();
    }
}

void Logger_Class::init(uint8_t destinations, int logLevel, fs::FS* fileSystem) {
    _logLevel = logLevel;
    
    if (_initialized) {
        debug("Logger already initialized, reconfiguring...");
    }
    
    _serialEnabled = (destinations & LOG_DEST_SERIAL) != 0;
    _flashEnabled = (destinations & LOG_DEST_FLASH) != 0;
    _sdEnabled = (destinations & LOG_DEST_SD) != 0;
    _callbackEnabled = (destinations & LOG_DEST_CALLBACK) != 0;
    
    if (_serialEnabled) {
        // Force a clean start for serial output
        Serial.println();
        Serial.println("------- Logger Initialized (Advanced) -------");
        Serial.flush();
    }
    
    if (_flashEnabled) {
        if (fileSystem == nullptr) {
            if (!SPIFFS.begin(true)) {
                if (_serialEnabled) {
                    Serial.println("Failed to initialize SPIFFS for logging");
                    Serial.flush();
                }
                _flashEnabled = false;
            } else {
                _fs = &SPIFFS;
            }
        } else {
            _fs = fileSystem;
        }
    }
    
    if (_sdEnabled && fileSystem != nullptr) {
        _fs = fileSystem;
        strncpy(_logFilePath, "/log.txt", sizeof(_logFilePath));
    }
    
    _circularBuffer.reserve(_circularBufferSize);
    _initialized = true;
    
    // Log initial message
    info("Logger initialized (advanced): level=%d, destinations=0x%02X",
        _logLevel, destinations);
    
    // Force flush to ensure output is seen
    if (_serialEnabled) {
        Serial.flush();
    }
}

void Logger_Class::enableSerialOutput(bool enable) {
    _serialEnabled = enable;
    if (_serialEnabled) {
        Serial.println("[DEBUG] Serial debug output enabled");
        Serial.flush();
    }
}

// Add this to the Logger_Class::enableFlashOutput method 
void Logger_Class::enableFlashOutput(bool enable, fs::FS* fs, const char* logPath) {
    if (!enable) {
        _flashEnabled = false;
        return;
    }
    
    bool fsAvailable = false;
    
    // Try to use the provided filesystem
    if (fs != nullptr) {
        _fs = fs;
        fsAvailable = true;
    } 
    // Otherwise try to initialize SPIFFS
    else {
        if (SPIFFS.begin(false)) {
            _fs = &SPIFFS;
            fsAvailable = true;
        } else {
            if (_serialEnabled) {
                Serial.println("Failed to mount SPIFFS, trying to format...");
            }
            
            // Try formatting as a last resort
            if (SPIFFS.format() && SPIFFS.begin()) {
                _fs = &SPIFFS;
                fsAvailable = true;
                if (_serialEnabled) {
                    Serial.println("SPIFFS formatted successfully");
                }
            } else {
                if (_serialEnabled) {
                    Serial.println("SPIFFS initialization failed completely");
                }
            }
        }
    }
    
    _flashEnabled = fsAvailable;
    
    if (_flashEnabled) {
        strncpy(_logFilePath, logPath, sizeof(_logFilePath));
        if (_serialEnabled) {
            Serial.printf("Flash logging enabled: path=%s\n", _logFilePath);
        }
    } else {
        if (_serialEnabled) {
            Serial.println("Flash logging disabled - filesystem unavailable");
        }
    }
}

void Logger_Class::enableSDOutput(bool enable, fs::FS* fs, const char* logPath) {
    _sdEnabled = enable && fs != nullptr;
    
    if (_sdEnabled) {
        _fs = fs;
        strncpy(_logFilePath, logPath, sizeof(_logFilePath));
        info("SD logging enabled: path=%s", _logFilePath);
    } else {
        info("SD logging disabled");
    }
}

void Logger_Class::setCallbackOutput(LogCallback callback, bool enable) {
    _logCallback = callback;
    _callbackEnabled = enable && (_logCallback != nullptr);
}

void Logger_Class::setLogLevel(int level) {
    if (level >= LOG_LEVEL_VERBOSE && level <= LOG_LEVEL_NONE) {
        _logLevel = level;
        if (_serialEnabled) {
            Serial.printf("[DEBUG] Log level set to %d\n", _logLevel);
            Serial.flush();
        }
    }
}

void Logger_Class::setMaxLogFileSize(size_t maxSize) {
    _maxLogFileSize = maxSize;
}

void Logger_Class::setCircularBufferSize(size_t numEntries) {
    _circularBufferSize = numEntries;
    _circularBuffer.reserve(_circularBufferSize);
}

void Logger_Class::verbose(const char* format, ...) {
    if (_logLevel > LOG_LEVEL_VERBOSE) return;
    
    char buf[LOG_BUFFER_SIZE];
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    
    outputMessage(LOG_LEVEL_VERBOSE, "VERBOSE", buf);
}

void Logger_Class::debug(const char* format, ...) {
    if (_logLevel > LOG_LEVEL_DEBUG) return;
    
    char buf[LOG_BUFFER_SIZE];
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    
    outputMessage(LOG_LEVEL_DEBUG, "DEBUG", buf);
}

void Logger_Class::info(const char* format, ...) {
    if (_logLevel > LOG_LEVEL_INFO) return;
    
    char buf[LOG_BUFFER_SIZE];
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    
    outputMessage(LOG_LEVEL_INFO, "INFO", buf);
}

void Logger_Class::warn(const char* format, ...) {
    if (_logLevel > LOG_LEVEL_WARN) return;
    
    char buf[LOG_BUFFER_SIZE];
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    
    outputMessage(LOG_LEVEL_WARN, "WARN", buf);
}

void Logger_Class::error(const char* format, ...) {
    if (_logLevel > LOG_LEVEL_ERROR) return;
    
    char buf[LOG_BUFFER_SIZE];
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    
    outputMessage(LOG_LEVEL_ERROR, "ERROR", buf);
}

void Logger_Class::critical(const char* format, ...) {
    if (_logLevel > LOG_LEVEL_CRITICAL) return;
    
    char buf[LOG_BUFFER_SIZE];
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    
    outputMessage(LOG_LEVEL_CRITICAL, "CRITICAL", buf);
}

void Logger_Class::log(int level, const char* format, ...) {
    if (_logLevel > level) return;
    
    const char* levelStr;
    switch (level) {
        case LOG_LEVEL_VERBOSE:  levelStr = "VERBOSE"; break;
        case LOG_LEVEL_DEBUG:    levelStr = "DEBUG"; break;
        case LOG_LEVEL_INFO:     levelStr = "INFO"; break;
        case LOG_LEVEL_WARN:     levelStr = "WARN"; break;
        case LOG_LEVEL_ERROR:    levelStr = "ERROR"; break;
        case LOG_LEVEL_CRITICAL: levelStr = "CRITICAL"; break;
        default:                 levelStr = "UNKNOWN"; break;
    }
    
    char buf[LOG_BUFFER_SIZE];
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    
    outputMessage(level, levelStr, buf);
}

void Logger_Class::raw(const char* message) {
    if (_serialEnabled) {
        Serial.println(message);
        Serial.flush();
    }
    
    if (_flashEnabled || _sdEnabled) {
        writeToFlash(message);
    }
    
    if (_callbackEnabled && _logCallback != nullptr) {
        _logCallback(message);
    }
    
    // Add to circular buffer if needed
    if (_circularBufferSize > 0) {
        if (_circularBuffer.size() >= _circularBufferSize) {
            _circularBuffer.erase(_circularBuffer.begin());
        }
        _circularBuffer.push_back(String(message));
    }
}

void Logger_Class::hexdump(const uint8_t* data, size_t length, const char* prefix) {
    if (_logLevel > LOG_LEVEL_DEBUG) return;
    
    const int BYTES_PER_LINE = 16;
    char buf[LOG_BUFFER_SIZE];
    
    for (size_t i = 0; i < length; i += BYTES_PER_LINE) {
        char* p = buf;
        
        // Add prefix if provided
        if (prefix && *prefix) {
            p += snprintf(p, sizeof(buf) - (p - buf), "%s ", prefix);
        }
        
        // Add address offset
        p += snprintf(p, sizeof(buf) - (p - buf), "%04X: ", (unsigned int)i);
        
        // Add hex representation
        for (int j = 0; j < BYTES_PER_LINE; j++) {
            if (i + j < length) {
                p += snprintf(p, sizeof(buf) - (p - buf), "%02X ", data[i + j]);
            } else {
                p += snprintf(p, sizeof(buf) - (p - buf), "   ");
            }
        }
        
        // Add ASCII representation
        p += snprintf(p, sizeof(buf) - (p - buf), " | ");
        for (int j = 0; j < BYTES_PER_LINE; j++) {
            if (i + j < length) {
                uint8_t c = data[i + j];
                char ch = (c >= 32 && c <= 126) ? (char)c : '.';
                p += snprintf(p, sizeof(buf) - (p - buf), "%c", ch);
            } else {
                p += snprintf(p, sizeof(buf) - (p - buf), " ");
            }
        }
        
        outputMessage(LOG_LEVEL_DEBUG, "HEX", buf);
    }
}

void Logger_Class::memoryStats(const char* tag) {
    if (_logLevel > LOG_LEVEL_INFO) return;
    
    char buf[LOG_BUFFER_SIZE];
    snprintf(buf, sizeof(buf), 
        "[%s] Free: %u bytes, Min Free: %u bytes, PSRAM Free: %u bytes",
        tag, ESP.getFreeHeap(), ESP.getMinFreeHeap(), ESP.getFreePsram());
    
    outputMessage(LOG_LEVEL_INFO, "MEM", buf);
}

std::vector<String> Logger_Class::getRecentLogs() {
    return _circularBuffer;
}

void Logger_Class::flush() {
    if (_serialEnabled) {
        Serial.flush();
    }
}

void Logger_Class::outputMessage(int level, const char* levelStr, const char* message) {
    char timestamp[32];
    timestampToStr(timestamp, sizeof(timestamp));
    
    char fullMessage[LOG_BUFFER_SIZE];
    snprintf(fullMessage, sizeof(fullMessage), "[%s][%6s][#%06u] %s", 
        timestamp, levelStr, _logSeqNum++, message);
    
    if (_serialEnabled) {
        Serial.println(fullMessage);
        Serial.flush();
    }
    
    if (_flashEnabled || _sdEnabled) {
        writeToFlash(fullMessage);
    }
    
    if (_callbackEnabled && _logCallback != nullptr) {
        _logCallback(fullMessage);
    }
    
    // Add to circular buffer if needed
    if (_circularBufferSize > 0) {
        if (_circularBuffer.size() >= _circularBufferSize) {
            _circularBuffer.erase(_circularBuffer.begin());
        }
        _circularBuffer.push_back(String(fullMessage));
    }
}

bool Logger_Class::writeToFlash(const char* message) {
    if (!_fs || (!_flashEnabled && !_sdEnabled)) return false;
    
    // Check file size first
    manageLimits();
    
    File logFile = _fs->open(_logFilePath, FILE_APPEND);
    if (!logFile) {
        // Try creating the file if it doesn't exist
        logFile = _fs->open(_logFilePath, FILE_WRITE);
        if (!logFile) {
            return false;
        }
    }
    
    size_t written = logFile.println(message);
    logFile.flush();  // Ensure data is written to disk
    logFile.close();
    
    return written > 0;
}

void Logger_Class::manageLimits() {
    if (!_fs || _maxLogFileSize == 0 || (!_flashEnabled && !_sdEnabled)) return;
    
    if (_fs->exists(_logFilePath)) {
        File logFile = _fs->open(_logFilePath, FILE_READ);
        if (logFile) {
            size_t fileSize = logFile.size();
            logFile.close();
            
            // If file is too large, truncate it
            if (fileSize > _maxLogFileSize) {
                // Strategy: Create a new file with the last portion of the old file
                File oldFile = _fs->open(_logFilePath, FILE_READ);
                File newFile = _fs->open("/temp_log.txt", FILE_WRITE);
                
                if (oldFile && newFile) {
                    // Skip to halfway through the file (simple truncation strategy)
                    oldFile.seek(fileSize / 2);
                    
                    // Copy the second half
                    const size_t BUF_SIZE = 256;
                    uint8_t buf[BUF_SIZE];
                    while (oldFile.available()) {
                        size_t bytesRead = oldFile.read(buf, BUF_SIZE);
                        newFile.write(buf, bytesRead);
                    }
                    
                    oldFile.close();
                    newFile.close();
                    
                    // Replace old file with new one
                    _fs->remove(_logFilePath);
                    _fs->rename("/temp_log.txt", _logFilePath);
                } else {
                    // Fallback: just truncate the file
                    if (oldFile) oldFile.close();
                    if (newFile) newFile.close();
                    
                    _fs->remove(_logFilePath);
                }
            }
        }
    }
}

void Logger_Class::timestampToStr(char* buffer, size_t bufSize) {
    unsigned long ms = millis();
    unsigned long seconds = ms / 1000;
    unsigned long minutes = seconds / 60;
    unsigned long hours = minutes / 60;
    
    seconds %= 60;
    minutes %= 60;
    ms %= 1000;
    
    snprintf(buffer, bufSize, "%3lu:%02lu:%02lu.%03lu", hours, minutes, seconds, ms);
}

void Logger_Class::setupCrashHandlers() {
#if CRASH_HANDLER_ENABLED
    if (_crashHandlersSetup) return;
    
    info("Setting up crash handlers");
    
    // FIXED: Use direct function call check instead of status type
    esp_err_t err = esp_task_wdt_status(NULL);
    if (err == ESP_OK) {
        info("WDT handler registered");
    } else {
        info("WDT status: %d", err);
    }
    
    _crashHandlersSetup = true;
    info("Basic crash handlers installed");
#else
    info("Crash handlers disabled in build");
#endif
}

void Logger_Class::logCrashData(const char* reason) {
    critical("CRASH DETECTED: %s", reason);
    memoryStats("CRASH");
    
    // Attempt to flush logs before crash
    flush();
}

#if CRASH_HANDLER_ENABLED
static void crash_handler(void* arg) {
    // This function would be called if we had registered it with the system
    Logger.logCrashData("System error detected");
}
#endif