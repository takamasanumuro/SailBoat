#pragma once
#include <Arduino.h>
#include "config.hpp"

namespace Logger {
    
    // Log levels for different types of messages
    enum LogLevel {
        DEBUG = 0,    // Detailed debug information
        INFO = 1,     // General information
        WARNING = 2,  // Warning conditions
        ERROR = 3,    // Error conditions  
        CRITICAL = 4  // Critical system errors
    };
    
    // Initialize the logging system
    void init();
    
    // Set minimum log level (messages below this level are filtered out)
    void setLogLevel(LogLevel level);
    
    // Get current log level
    LogLevel getLogLevel();
    
    // Core logging functions
    void log(LogLevel level, const char* message);
    void log(LogLevel level, const char* format, int value);
    void log(LogLevel level, const char* format, float value);
    void log(LogLevel level, const char* format, const char* str_value);
    
    // Debug helpers - only active in debug builds
    void debugPrint(const char* message);
    void debugValue(const char* label, int value);
    void debugValue(const char* label, float value);
    
    // Error logging with automatic severity detection
    void logError(const char* error_message);
    void logCritical(const char* critical_message);
    
    // System status logging
    void logStatus(const char* subsystem, const char* status);
    
    // Performance monitoring
    void logTiming(const char* operation, uint32_t duration_ms);
    
}