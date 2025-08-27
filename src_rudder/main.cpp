#include <Arduino.h>
#include "HBridgeDriver_v2.hpp"

#ifdef RUDDER_TEST_MODE
    #define TEST_VERSION "2.0.0"
    #define TEST_NAME "Direct H-Bridge Controller"
#endif

// Global H-bridge driver instance
HBridgeDriverV2 rudder_hbridge;

// Simple utility functions
namespace RudderUtils {
    int pwmInputToPercentage(int pwm_input) {
        if (pwm_input < 1000 || pwm_input > 2000) return 0;
        return (pwm_input - 1500) / 5;
    }
    
    int percentageToPWMInput(int percentage) {
        if (percentage < -100 || percentage > 100) return 1500;
        return 1500 + (percentage * 5);
    }
}

// Command interface
namespace Commands {
    static char input_buffer[32];
    static int buffer_index = 0;
    static bool analog_monitoring_enabled = false;
    static unsigned long last_analog_print = 0;
    static const unsigned long ANALOG_PRINT_INTERVAL = 1000; // 1 second
    
    void parseCommand(const char* cmd); // Forward declaration
    
    void init() {
        Serial.println("=======================================");
        Serial.println("     Direct H-Bridge Rudder Control");
        Serial.println("=======================================");
        Serial.println("Commands: p <-100-100>, pwm <1000-2000>, stop, help");
        Serial.print("> ");
    }
    
    void processInput() {
        while (Serial.available()) {
            char c = Serial.read();
            if (c == '\r') continue;
            
            if (c == '\n') {
                input_buffer[buffer_index] = '\0';
                buffer_index = 0;
                if (strlen(input_buffer) > 0) {
                    parseCommand(input_buffer);
                }
                Serial.print("> ");
            } else if (c == '\b' || c == '\x7f') { // Backspace or Delete
                if (buffer_index > 0) {
                    buffer_index--;
                    Serial.print("\b \b"); // Move cursor back, print space, move back again
                }
            } else if (buffer_index < (int)sizeof(input_buffer) - 1) {
                input_buffer[buffer_index++] = c;
                Serial.print(c);
            }
        }
    }
    
    void parseCommand(const char* cmd) {
        Serial.println();
        
        char cmd_copy[32];
        strncpy(cmd_copy, cmd, sizeof(cmd_copy) - 1);
        cmd_copy[sizeof(cmd_copy) - 1] = '\0';
        
        char* command = strtok(cmd_copy, " ");
        char* arg = strtok(NULL, " ");
        
        if (command == NULL) return;
        
        if (strcmp(command, "p") == 0 && arg != NULL) {
            int percentage = atoi(arg);
            auto result = rudder_hbridge.setPercentage((float)percentage, HBridgeDriverV2::Channel::M1);
            if (result == HBridgeDriverV2::ErrorCode::NONE) {
                Serial.print("Set to "); Serial.print(percentage); Serial.println("%");
            } else {
                Serial.println("Error setting percentage");
            }
        }
        else if (strcmp(command, "pwm") == 0 && arg != NULL) {
            int pwm_input = atoi(arg);
            int percentage = RudderUtils::pwmInputToPercentage(pwm_input);
            auto result = rudder_hbridge.setPercentage((float)percentage, HBridgeDriverV2::Channel::M1);
            if (result == HBridgeDriverV2::ErrorCode::NONE) {
                Serial.print("Set PWM "); Serial.print(pwm_input); Serial.print(" ("); 
                Serial.print(percentage); Serial.println("%)");
            } else {
                Serial.println("Error setting PWM input");
            }
        }
        else if (strcmp(command, "stop") == 0) {
            rudder_hbridge.stop(HBridgeDriverV2::Channel::M1);
            Serial.println("Stopped");
        }
        else if (strcmp(command, "status") == 0) {
            int16_t pwm = rudder_hbridge.getCurrentPWM(HBridgeDriverV2::Channel::M1);
            float pct = rudder_hbridge.getCurrentPercentage(HBridgeDriverV2::Channel::M1);
            Serial.print("PWM: "); Serial.print(pwm);
            Serial.print(", Percentage: "); Serial.print(pct); Serial.println("%");
        }
        else if (strcmp(command, "analog") == 0) {
            if (arg != NULL) {
                if (strcmp(arg, "on") == 0) {
                    analog_monitoring_enabled = true;
                    Serial.println("Analog monitoring enabled");
                } else if (strcmp(arg, "off") == 0) {
                    analog_monitoring_enabled = false;
                    Serial.println("Analog monitoring disabled");
                } else {
                    Serial.println("Usage: analog <on|off>");
                }
            } else {
                // Single analog read when no argument provided
                int analog_value = analogRead(A9);
                float voltage = analog_value * 5.0f / 1023.0f;
                Serial.print("A9: "); Serial.print(analog_value);
                Serial.print(" ("); Serial.print(voltage, 2); Serial.println("V)");
            }
        }
        else if (strcmp(command, "help") == 0) {
            Serial.println("Commands:");
            Serial.println("p <-100-100>   - Set percentage");
            Serial.println("pwm <1000-2000> - Set PWM input");
            Serial.println("stop           - Stop motor");
            Serial.println("status         - Show status");
            Serial.println("analog         - Read A9 analog value once");
            Serial.println("analog on      - Enable continuous analog monitoring");
            Serial.println("analog off     - Disable continuous analog monitoring");
        }
        else {
            Serial.println("Unknown command. Type 'help'.");
        }
    }
    
    void updateAnalogMonitoring() {
        if (!analog_monitoring_enabled) return;
        
        unsigned long current_time = millis();
        if (current_time - last_analog_print < ANALOG_PRINT_INTERVAL) return;
        
        int analog_value = analogRead(A9);
        float voltage = analog_value * 5.0f / 1023.0f;
        Serial.print("A9: "); Serial.print(analog_value);
        Serial.print(" ("); Serial.print(voltage, 2); Serial.println("V)");
        last_analog_print = current_time;
    }
}

void setup() {
    Serial.begin(9600);
    delay(1000);
    
    Serial.println("Initializing H-bridge...");
    
    // Configure H-bridge using explicit member initialization
    HBridgeDriverV2::Config rudder_cfg;
    rudder_cfg.pwm_pin = 10;
    rudder_cfg.inb_pin = 9;
    rudder_cfg.ina_pin = 8;
    rudder_cfg.max_pwm = 240;
    rudder_cfg.min_pwm = 60;  // 25% of 240 = 60 (configurable deadband)
    
    auto config_result = rudder_hbridge.setChannelConfig(HBridgeDriverV2::Channel::M1, rudder_cfg);
    if (config_result != HBridgeDriverV2::ErrorCode::NONE) {
        Serial.println("FATAL: Configuration failed");
        Serial.println(rudder_hbridge.getErrorString(config_result));
        while (1) delay(1000);
    }
    
    auto init_result = rudder_hbridge.init();
    if (init_result != HBridgeDriverV2::ErrorCode::NONE) {
        Serial.println("FATAL: Initialization failed");
        Serial.println(rudder_hbridge.getErrorString(init_result));
        while (1) delay(1000);
    }

    pinMode(11, OUTPUT); digitalWrite(11, LOW);
    pinMode(12, OUTPUT); digitalWrite(12, HIGH);
    
    // Configure additional pins
    pinMode(A8, OUTPUT); digitalWrite(A8, LOW);   // A8 as digital LOW
    pinMode(A10, OUTPUT); digitalWrite(A10, HIGH); // A10 as digital HIGH
    pinMode(A9, INPUT);                           // A9 for analog monitoring
    
    Commands::init();
    Serial.println("Ready!");
}

void loop() {
    Commands::processInput();
    Commands::updateAnalogMonitoring();
    delay(10);
}