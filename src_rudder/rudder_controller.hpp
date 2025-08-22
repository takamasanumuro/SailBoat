#pragma once
#include <Arduino.h>
#include "HBridgeDriver.hpp"

namespace RudderController {
    
    // Simple configuration
    struct Config {
        uint8_t direction_pin_a = 22;  // INA pin
        uint8_t direction_pin_b = 23;  // INB pin  
        uint8_t pwm_pin = 6;           // PWM pin
        int16_t max_pwm = 240;         // Maximum PWM value
    };
    
    // Main controller class
    class Controller {
    private:
        Config config;
        HBridgeDriver* hbridge;
        bool initialized;
        int16_t current_pwm;
        int current_percentage;
        int current_pwm_input;
        
    public:
        Controller();
        ~Controller();
        
        bool init(const Config& cfg = Config());
        
        // Easy control methods
        bool setPercentage(int percentage);  // -100 to +100
        bool setPWMInput(int pwm_input);     // 1000 to 2000 (like RC input)
        bool stop();
        
        // Status
        int getCurrentPercentage() const { return current_percentage; }
        int getCurrentPWMInput() const { return current_pwm_input; }
        int16_t getCurrentPWM() const { return current_pwm; }
        
        void printStatus();
    };
    
    // Global instance
    extern Controller rudder;
    
    // Command interface
    namespace Commands {
        void init();
        void processInput();
        void parseCommand(const char* cmd);
        
        void handlePercentage(int percentage);
        void handlePWMInput(int pwm_input);
        void handleStop();
        void handleStatus();
        void handleHelp();
    }
}