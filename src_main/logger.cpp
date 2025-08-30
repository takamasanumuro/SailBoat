#include "logger.hpp"

namespace Logger {
    
    // Current minimum log level
    static LogLevel current_log_level = INFO;
    static bool logging_initialized = false;
    
    // Log level string representations
    const char* getLogLevelString(LogLevel level) {
        switch (level) {
            case DEBUG:    return "[DEBUG]";
            case INFO:     return "[INFO] ";
            case WARNING:  return "[WARN] ";
            case ERROR:    return "[ERROR]";
            case CRITICAL: return "[CRIT] ";
            default:       return "[UNKN] ";
        }
    }
    
    // Check if message should be logged based on current log level
    bool shouldLog(LogLevel level) {
        return logging_initialized && (level >= current_log_level);
    }
    
    // Get timestamp string for log messages
    void printTimestamp() {
        uint32_t time_ms = millis();
        uint32_t seconds = time_ms / 1000;
        uint32_t minutes = seconds / 60;
        uint32_t hours = minutes / 60;
        
        Serial.print("[");
        if (hours < 10) Serial.print("0");
        Serial.print(hours % 24);
        Serial.print(":");
        if ((minutes % 60) < 10) Serial.print("0");
        Serial.print(minutes % 60);
        Serial.print(":");
        if ((seconds % 60) < 10) Serial.print("0");
        Serial.print(seconds % 60);
        Serial.print(".");
        if ((time_ms % 1000) < 100) Serial.print("0");
        if ((time_ms % 1000) < 10) Serial.print("0");
        Serial.print(time_ms % 1000);
        Serial.print("] ");
    }
    
    void init() {
        // Logger initialization is handled by Serial.begin() in main setup
        logging_initialized = true;
        
        #ifdef ENVIRONMENT_MAIN
            current_log_level = INFO;  // Production logging
        #else
            current_log_level = DEBUG; // Development logging
        #endif
        
        log(INFO, "Logger initialized");
    }
    
    void setLogLevel(LogLevel level) {
        current_log_level = level;
        log(INFO, "Log level changed");
    }
    
    LogLevel getLogLevel() {
        return current_log_level;
    }
    
    void log(LogLevel level, const char* message) {
        if (!shouldLog(level)) return;
        
        printTimestamp();
        Serial.print(getLogLevelString(level));
        Serial.print(" ");
        Serial.println(message);
    }
    
    void log(LogLevel level, const char* format, int value) {
        if (!shouldLog(level)) return;
        
        printTimestamp();
        Serial.print(getLogLevelString(level));
        Serial.print(" ");
        Serial.print(format);
        Serial.println(value);
    }
    
    void log(LogLevel level, const char* format, float value) {
        if (!shouldLog(level)) return;
        
        printTimestamp();
        Serial.print(getLogLevelString(level));
        Serial.print(" ");
        Serial.print(format);
        Serial.println(value);
    }
    
    void log(LogLevel level, const char* format, const char* str_value) {
        if (!shouldLog(level)) return;
        
        printTimestamp();
        Serial.print(getLogLevelString(level));
        Serial.print(" ");
        Serial.print(format);
        Serial.println(str_value);
    }
    
    void debugPrint(const char* message) {
        #ifdef SERIAL_DEBUG
            log(DEBUG, message);
        #endif
    }
    
    void debugValue(const char* label, int value) {
        #ifdef SERIAL_DEBUG
            if (!shouldLog(DEBUG)) return;
            
            printTimestamp();
            Serial.print("[DEBUG] ");
            Serial.print(label);
            Serial.print(": ");
            Serial.println(value);
        #endif
    }
    
    void debugValue(const char* label, float value) {
        #ifdef SERIAL_DEBUG
            if (!shouldLog(DEBUG)) return;
            
            printTimestamp();
            Serial.print("[DEBUG] ");
            Serial.print(label);
            Serial.print(": ");
            Serial.println(value);
        #endif
    }
    
    void logError(const char* error_message) {
        log(ERROR, "Error: ", error_message);
    }
    
    void logCritical(const char* critical_message) {
        log(CRITICAL, "CRITICAL: ", critical_message);
    }
    
    void logStatus(const char* subsystem, const char* status) {
        if (!shouldLog(INFO)) return;
        
        printTimestamp();
        Serial.print("[INFO]  ");
        Serial.print(subsystem);
        Serial.print(" status: ");
        Serial.println(status);
    }
    
    void logTiming(const char* operation, uint32_t duration_ms) {
        if (!shouldLog(DEBUG)) return;
        
        printTimestamp();
        Serial.print("[DEBUG] ");
        Serial.print(operation);
        Serial.print(" took ");
        Serial.print(duration_ms);
        Serial.println(" ms");
    }
    
}