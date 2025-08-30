#pragma once
#include <Arduino.h>
#include "config.hpp"
#include "rudder_control.hpp"
#include "analog_voltage_generator.hpp"

namespace CommandInterface {
    
    class CommandProcessor {
    private:
        char input_buffer[64];
        int buffer_index;
        bool analog_monitoring_enabled;
        unsigned long last_analog_print;
        
        // Command handlers
        void handleRudderCommand(const char* arg);
        void handleMotorCommand(const char* arg);
        void handleAngleCommand(const char* arg);
        void handlePIDCommand(const char* arg);
        void handleTuneCommand(const char* arg1, const char* arg2, const char* arg3);
        void handleAnalogCommand(const char* arg);
        void handleStatusCommand();
        void handleMapCommand(const char* arg1, const char* arg2, const char* arg3, const char* arg4);
        void handleTestCommand(const char* arg);
        void handleStopCommand();
        void handleHelpCommand();
        
        // Helper functions
        void parseCommand(const char* cmd);
        void showCurrentPIDValues();
        
    public:
        CommandProcessor();
        
        void init();
        void processInput();
        void updateAnalogMonitoring();
    };
    
    // Global instance
    extern CommandProcessor processor;
}