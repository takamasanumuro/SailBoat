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
    
    // Specialized logging functions for common sailboat operations
    void logSystemStart(const char* system_name, const char* version);
    void logActuatorCommand(const char* actuator, int command_value);
    void logSensorReading(const char* sensor_name, int raw_value, float converted_value);
    void logPIDOutput(const char* controller, int desired, int current, int error, int pwm_output);
    void logMAVLinkMessage(const char* message_type, float data);
    void logSafetyEvent(const char* event_description);
    void logThrottleControl(int pwm_value);
    void logCalibrationData(const char* actuator, int min_val, int max_val, float min_angle, float max_angle);
    
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