#include "command_interface.hpp"
#include <avr/pgmspace.h>

// Store help strings in Flash memory (PROGMEM)
const char PROGMEM help_header[] = "=== Available Commands ===";
const char PROGMEM help_rudder[] = "rudder <-100-100>   - Set rudder percentage";
const char PROGMEM help_motor[] = "motor <0-100>       - Set motor voltage percentage";
const char PROGMEM help_angle[] = "angle <degrees>     - Set target angle for PID";
const char PROGMEM help_pid[] = "pid [on|off]        - Enable/disable PID";
const char PROGMEM help_tune[] = "tune <kp> <ki> <kd> - Adjust PID gains";
const char PROGMEM help_analog[] = "analog [on|off]     - Enable/disable monitoring";
const char PROGMEM help_status[] = "status              - Show system status";
const char PROGMEM help_test[] = "test <-20-20>       - Safe motor test";
const char PROGMEM help_stop[] = "stop                - Stop all systems";
const char PROGMEM help_mavlink[] = "mavlink [status|channels] - Mavlink status";
const char PROGMEM help_help[] = "help                - Show this help";

namespace CommandInterface {
    
    // Global instance
    CommandProcessor processor;
    
    CommandProcessor::CommandProcessor() 
        : buffer_index(0), analog_monitoring_enabled(false), last_analog_print(0), mavlink_instance(nullptr) {
    }
    
    void CommandProcessor::init() {
        Serial.println("=======================================");
        Serial.println("Integrated Sailboat Autopilot System");
        Serial.println("=======================================");
        Serial.println("Type 'help' for available commands");
        Serial.print("> ");
    }
    
    void CommandProcessor::processInput() {
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
                    Serial.print("\b \b");
                }
            } else if (buffer_index < (int)sizeof(input_buffer) - 1) {
                input_buffer[buffer_index++] = c;
                Serial.print(c);
            }
        }
    }
    
    void CommandProcessor::parseCommand(const char* cmd) {
        Serial.println();
        
        char cmd_copy[64];
        strncpy(cmd_copy, cmd, sizeof(cmd_copy) - 1);
        cmd_copy[sizeof(cmd_copy) - 1] = '\0';
        
        char* command = strtok(cmd_copy, " ");
        char* arg1 = strtok(NULL, " ");
        char* arg2 = strtok(NULL, " ");
        char* arg3 = strtok(NULL, " ");
        char* arg4 = strtok(NULL, " ");
        
        if (command == NULL) return;
        
        // Route commands to appropriate handlers
        if (strcmp(command, "rudder") == 0) {
            handleRudderCommand(arg1);
        }
        else if (strcmp(command, "motor") == 0) {
            handleMotorCommand(arg1);
        }
        else if (strcmp(command, "angle") == 0) {
            handleAngleCommand(arg1);
        }
        else if (strcmp(command, "pid") == 0) {
            handlePIDCommand(arg1);
        }
        else if (strcmp(command, "tune") == 0) {
            handleTuneCommand(arg1, arg2, arg3);
        }
        else if (strcmp(command, "analog") == 0) {
            handleAnalogCommand(arg1);
        }
        else if (strcmp(command, "status") == 0) {
            handleStatusCommand();
        }
        else if (strcmp(command, "map") == 0) {
            handleMapCommand(arg1, arg2, arg3, arg4);
        }
        else if (strcmp(command, "test") == 0) {
            handleTestCommand(arg1);
        }
        else if (strcmp(command, "stop") == 0) {
            handleStopCommand();
        }
        else if (strcmp(command, "help") == 0) {
            handleHelpCommand();
        }
        else if (strcmp(command, "mavlink") == 0) {
            handleMavlinkCommand(arg1);
        }
        else {
            Serial.print("Unknown command: "); Serial.print(command); Serial.println(". Type 'help' for available commands.");
        }
    }
    
    void CommandProcessor::handleRudderCommand(const char* arg) {
        if (arg == NULL) {
            Serial.println("Usage: rudder <-100-100>");
            return;
        }
        
        int percentage = atoi(arg);
        if (RudderControl::controller.setPercentage((float)percentage)) {
            Serial.print("Rudder set to "); Serial.print(percentage); Serial.println("%");
        }
    }
    
    void CommandProcessor::handleMotorCommand(const char* arg) {
        if (arg == NULL) {
            Serial.println("Usage: motor <0-100>");
            return;
        }
        
        float percentage = atof(arg);
        if (AnalogVoltageGenerator::generator.setVoltagePercentage(percentage)) {
            Serial.print("Motor set to "); Serial.print(percentage); 
            Serial.print("% ("); Serial.print(AnalogVoltageGenerator::generator.getCurrentVoltage());
            Serial.println(" mV)");
        }
    }
    
    void CommandProcessor::handleAngleCommand(const char* arg) {
        if (arg == NULL) {
            Serial.println("Usage: angle <degrees>");
            return;
        }
        
        float target = atof(arg);
        RudderControl::controller.setAngleTarget(target);
    }
    
    void CommandProcessor::handlePIDCommand(const char* arg) {
        if (arg != NULL) {
            if (strcmp(arg, "on") == 0) {
                RudderControl::controller.enablePID(true);
            } else if (strcmp(arg, "off") == 0) {
                RudderControl::controller.enablePID(false);
            } else {
                Serial.println("Usage: pid [on|off]");
            }
        } else {
            // Show PID status
            float current = RudderControl::controller.getCurrentAngle();
            Serial.print("Current angle: "); Serial.print(current); 
            Serial.print("°, Target: "); Serial.print(RudderControl::controller.getTargetAngle());
            Serial.print("°, PID: "); Serial.println(RudderControl::controller.isPIDEnabled() ? "ON" : "OFF");
        }
    }
    
    void CommandProcessor::handleTuneCommand(const char* arg1, const char* arg2, const char* arg3) {
        if (arg1 != NULL && arg2 != NULL && arg3 != NULL) {
            // Set new PID values
            double kp = atof(arg1);
            double ki = atof(arg2);
            double kd = atof(arg3);
            
            RudderControl::controller.setPIDTunings(kp, ki, kd);
        } else if (arg1 == NULL) {
            // Show current PID values
            showCurrentPIDValues();
        } else {
            Serial.println("Usage: tune <kp> <ki> <kd> or tune (to show current values)");
        }
    }
    
    void CommandProcessor::handleAnalogCommand(const char* arg) {
        if (arg != NULL) {
            if (strcmp(arg, "on") == 0) {
                analog_monitoring_enabled = true;
                Serial.println("Analog monitoring enabled");
            } else if (strcmp(arg, "off") == 0) {
                analog_monitoring_enabled = false;
                Serial.println("Analog monitoring disabled");
            } else {
                Serial.println("Usage: analog [on|off]");
            }
        } else {
            // Single analog read
            int analog_value = analogRead(Config::Pins::Potentiometers::Rudder::SIGNAL);
            float voltage = analog_value * 5.0f / 1023.0f;
            float angle = RudderControl::controller.adcToAngle(analog_value);
            Serial.print("A9: "); Serial.print(analog_value); 
            Serial.print(" ("); Serial.print(voltage, 2); 
            Serial.print("V, "); Serial.print(angle, 1); 
            Serial.println("°)");
        }
    }
    
    void CommandProcessor::handleStatusCommand() {
        Serial.println("=== System Status ===");
        Serial.print("Rudder: "); Serial.print(RudderControl::controller.getCurrentPercentage());
        Serial.print("%, PWM: "); Serial.println(RudderControl::controller.getCurrentPWM());
        Serial.print("Motor: "); Serial.print(AnalogVoltageGenerator::generator.getCurrentPercentage());
        Serial.print("% ("); Serial.print(AnalogVoltageGenerator::generator.getCurrentVoltage());
        Serial.println(" mV)");
        Serial.print("Current angle: "); Serial.println(RudderControl::controller.getCurrentAngle());
        Serial.print("PID: "); Serial.print(RudderControl::controller.isPIDEnabled() ? "ON" : "OFF");
        Serial.print(", Target: "); Serial.print(RudderControl::controller.getTargetAngle());
        Serial.println("°");
        
        // Show current PID gains
        showCurrentPIDValues();
    }
    
    void CommandProcessor::handleMapCommand(const char* arg1, const char* arg2, const char* arg3, const char* arg4) {
        if (arg1 && arg2 && arg3 && arg4) {
            int adc_min = atoi(arg1);
            int adc_max = atoi(arg2);
            float angle_min = atof(arg3);
            float angle_max = atof(arg4);
            
            RudderControl::controller.setAngleMapping(adc_min, adc_max, angle_min, angle_max);
        } else {
            Serial.println("Usage: map <adc_min> <adc_max> <angle_min> <angle_max>");
        }
    }
    
    void CommandProcessor::handleTestCommand(const char* arg) {
        if (arg == NULL) {
            Serial.println("Usage: test <-20-20>");
            return;
        }
        
        float percentage = atof(arg);
        percentage = constrain(percentage, -20.0f, 20.0f); // Limit for safety
        
        // Disable PID first for safety
        RudderControl::controller.enablePID(false);
        
        if (RudderControl::controller.setPercentage(percentage)) {
            Serial.print("Direct motor test: "); Serial.print(percentage);
            Serial.println("% (PID disabled for safety)");
        }
    }
    
    void CommandProcessor::handleStopCommand() {
        // Stop everything
        RudderControl::controller.enablePID(false);
        RudderControl::controller.stop();
        AnalogVoltageGenerator::generator.setVoltagePercentage(0.0f);
        Serial.println("All systems stopped");
    }
    
    void CommandProcessor::handleHelpCommand() {
        Serial.println(reinterpret_cast<const __FlashStringHelper*>(help_header));
        Serial.println(reinterpret_cast<const __FlashStringHelper*>(help_rudder));
        Serial.println(reinterpret_cast<const __FlashStringHelper*>(help_motor));
        Serial.println(reinterpret_cast<const __FlashStringHelper*>(help_angle));
        Serial.println(reinterpret_cast<const __FlashStringHelper*>(help_pid));
        Serial.println(reinterpret_cast<const __FlashStringHelper*>(help_tune));
        Serial.println(reinterpret_cast<const __FlashStringHelper*>(help_analog));
        Serial.println(reinterpret_cast<const __FlashStringHelper*>(help_status));
        Serial.println(reinterpret_cast<const __FlashStringHelper*>(help_test));
        Serial.println(reinterpret_cast<const __FlashStringHelper*>(help_stop));
        Serial.println(reinterpret_cast<const __FlashStringHelper*>(help_mavlink));
        Serial.println(reinterpret_cast<const __FlashStringHelper*>(help_help));
        Serial.println(F(""));
        Serial.println(F("Presets: tune 0.1 0 0 | tune 0.2 0.01 0.05 | tune 0.05 0 0.02"));
    }
    
    void CommandProcessor::showCurrentPIDValues() {
        double kp, ki, kd;
        RudderControl::controller.getPIDTunings(kp, ki, kd);
        if (kp != 0.0 || ki != 0.0 || kd != 0.0) {
            Serial.print("Current PID gains: P="); Serial.print(kp, 3);
            Serial.print(", I="); Serial.print(ki, 3);
            Serial.print(", D="); Serial.println(kd, 3);
        } else {
            Serial.println("PID controller not initialized");
        }
    }
    
    void CommandProcessor::updateAnalogMonitoring() {
        if (!analog_monitoring_enabled) return;
        
        unsigned long current_time = millis();
        if (current_time - last_analog_print < Config::Timing::ANALOG_PRINT_INTERVAL_MS) return;
        
        int analog_value = analogRead(Config::Pins::Potentiometers::Rudder::SIGNAL);
        float voltage = analog_value * 5.0f / 1023.0f;
        float angle = RudderControl::controller.adcToAngle(analog_value);
        Serial.print("A9: "); Serial.print(analog_value);
        Serial.print(" ("); Serial.print(voltage, 2);
        Serial.print("V, "); Serial.print(angle, 1);
        Serial.println("°)");
        last_analog_print = current_time;
    }
    
    void CommandProcessor::setMavlinkInstance(MavlinkCommunication* mavlink_ptr) {
        mavlink_instance = mavlink_ptr;
    }
    
    void CommandProcessor::handleMavlinkCommand(const char* arg) {
        if (!mavlink_instance) {
            Serial.println("Mavlink instance not set!");
            return;
        }
        
        if (!arg) {
            // No argument - show status
            mavlink_instance->print_status();
        }
        else if (strcmp(arg, "status") == 0) {
            mavlink_instance->print_status();
        }
        else if (strcmp(arg, "channels") == 0) {
            mavlink_instance->print_rc_channels();
        }
        else if (strcmp(arg, "help") == 0) {
            Serial.println("=== Mavlink Commands ===");
            Serial.println("mavlink           - Show Mavlink status");
            Serial.println("mavlink status    - Show Mavlink status");
            Serial.println("mavlink channels  - Show RC channel values");
            Serial.println("mavlink help      - Show this help");
        }
        else {
            Serial.print("Unknown mavlink command: "); Serial.println(arg);
            Serial.println("Use 'mavlink help' for available commands.");
        }
    }
}