#pragma once
#include <Arduino.h>

namespace AnalogVoltageGenerator {
    
    // Configuration structure for voltage generation
    struct VoltageConfig {
        float min_voltage_mv = 800.0f;   // Minimum output voltage in mV
        float max_voltage_mv = 4000.0f;  // Maximum output voltage in mV
        uint8_t pwm_pin = 9;             // PWM output pin (Timer1 for high resolution)
        uint16_t pwm_frequency_hz = 1000; // PWM frequency in Hz
        uint8_t pwm_resolution_bits = 10;  // PWM resolution (8, 9, or 10 bits)
    };
    
    // Voltage generator class
    class VoltageGenerator {
    private:
        VoltageConfig config;
        bool initialized;
        float current_percentage;
        uint16_t current_pwm_value;
        
        uint16_t percentageToPWM(float percentage);
        
    public:
        VoltageGenerator();
        
        // Initialization and configuration
        bool init(const VoltageConfig& cfg = VoltageConfig());
        bool isInitialized() const { return initialized; }
        
        // Voltage control
        bool setVoltagePercentage(float percentage);
        bool setVoltageMV(float voltage_mv);
        
        // Get current values
        float getCurrentPercentage() const { return current_percentage; }
        float getCurrentVoltage() const;
        uint16_t getCurrentPWM() const { return current_pwm_value; }
        
        // Configuration management
        void setVoltageRange(float min_mv, float max_mv);
        void setPWMPin(uint8_t pin);
        void setPWMFrequency(uint16_t frequency_hz);
        VoltageConfig getConfig() const { return config; }
        
        // Utility functions
        void disable();
        void enable();
        bool isEnabled() const;
        
        // Calibration and testing
        void runCalibrationSequence();
        void outputTestPattern();
    };
    
    // Global instance
    extern VoltageGenerator generator;
    
    // Command interface for UART control
    namespace CommandInterface {
        void init();
        void processSerialInput();
        void parseCommand(const char* command);
        
        // Command handlers
        void handleSetPercentage(float percentage);
        void handleSetVoltage(float voltage_mv);
        void handleSetRange(float min_mv, float max_mv);
        void handleGetStatus();
        void handleCalibration();
        void handleHelp();
        void handleConfig();
        
        // Response functions
        void sendResponse(const char* message);
        void sendError(const char* error);
        void sendStatus();
    }
    
    // Utility functions
    namespace Utils {
        float mapFloat(float value, float in_min, float in_max, float out_min, float out_max);
        uint16_t calculatePWMValue(float voltage_mv, float min_voltage, float max_voltage, uint16_t max_pwm);
        float calculateVoltage(uint16_t pwm_value, float min_voltage, float max_voltage, uint16_t max_pwm);
        bool isValidPercentage(float percentage);
        bool isValidVoltage(float voltage_mv, float min_mv, float max_mv);
    }
    
}