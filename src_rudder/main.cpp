#include <Arduino.h>
#include "HBridgeDriver_v2.hpp"

extern "C" {
#include "c/PID.h"
}

#ifdef RUDDER_TEST_MODE
    #define TEST_NAME "Rudder H-Bridge Controller"
#endif

// Global H-bridge driver instance
HBridgeDriverV2 rudder_hbridge;

// Rudder angle control system
namespace RudderAngle {
    struct Config {
        int adc_min = 285;        // ADC value at minimum angle
        int adc_max = 611;     // ADC value at maximum angle  
        float angle_min = -45.0f; // Minimum rudder angle (degrees)
        float angle_max = 45.0f;  // Maximum rudder angle (degrees)
    };
    
    static Config angle_config;
    static PIDController* pid_controller = nullptr;
    static bool pid_enabled = false;
    static float target_angle = 0.0f;
    static float current_angle = 0.0f;
    
    float adcToAngle(int adc_value) {
        if (angle_config.adc_max == angle_config.adc_min) return 0.0f;
        
        float normalized = (float)(adc_value - angle_config.adc_min) / 
                          (angle_config.adc_max - angle_config.adc_min);
        normalized = constrain(normalized, 0.0f, 1.0f);
        
        return angle_config.angle_min + normalized * (angle_config.angle_max - angle_config.angle_min);
    }
    
    int angleToAdc(float angle) {
        if (angle_config.angle_max == angle_config.angle_min) return angle_config.adc_min;
        
        float normalized = (angle - angle_config.angle_min) / 
                          (angle_config.angle_max - angle_config.angle_min);
        normalized = constrain(normalized, 0.0f, 1.0f);
        
        return angle_config.adc_min + (int)(normalized * (angle_config.adc_max - angle_config.adc_min));
    }
    
    // PID callback functions
    int pidSource() {
        // Return current angle, scaled by 10 for integer precision
        return (int)(adcToAngle(analogRead(A9)) * 10.0f);
    }

    void pidOutput(int output) {
        // PID output is now the desired motor percentage (-100 to 100)
        float percentage = (float)output;

        // Apply reasonable limits (don't use full 100% motor power)
        percentage = constrain(percentage, -80.0f, 80.0f);

        // DEBUG: Print PID output values
        int current_angle_scaled = pidSource();
        int error = pid_controller->target - current_angle_scaled; // Error is in scaled degrees
        Serial.print("PID Raw Output: "); Serial.print(output);
        Serial.print(", Error (deg*10): "); Serial.print(error);
        Serial.print(", Motor %: "); Serial.print(percentage, 1);
        Serial.print(", Current Angle: "); Serial.print((float)current_angle_scaled / 10.0f, 1);
        Serial.print(", Target Angle: "); Serial.print((float)pid_controller->target / 10.0f, 1);

        // Show PID components for debugging
        if (pid_controller != nullptr) {
            Serial.print(", P="); Serial.print(pid_controller->p * error, 2);
            Serial.print(", I="); Serial.print(pid_controller->i * pid_controller->integralCumulation, 2);
            Serial.print(", D="); Serial.print(pid_controller->d * pid_controller->cycleDerivative, 2);
        }
        Serial.println();

        // Dead zone to prevent jitter and unnecessary motor movement
        if (abs(percentage) > 1.0f) {
            // FLIP DIRECTION: PID output may be backwards depending on tuning/wiring
            rudder_hbridge.setPercentage(-percentage, HBridgeDriverV2::Channel::M1);
        } else {
            rudder_hbridge.stop(HBridgeDriverV2::Channel::M1);
        }
    }

    unsigned long getTime() {
        return millis() / 1000;
    }

    void initPID(double kp = 2.0, double ki = 0.05, double kd = 0.25) {
        if (pid_controller != nullptr) {
            free(pid_controller);
        }

        pid_controller = createPIDController(kp, ki, kd, pidSource, pidOutput);
        if (pid_controller != nullptr) {
            registerTimeFunction(pid_controller, getTime);
            // Set output bounds to motor percentage (-100 to 100)
            setOutputBounds(pid_controller, -100, 100);
            // Set a reasonable integral limit to prevent windup
            setMaxIntegralCumulation(pid_controller, 100);
            setEnabled(pid_controller, 0);
            pid_enabled = false;
        }
    }

    void setAngleTarget(float angle) {
        if (pid_controller == nullptr) return;

        target_angle = constrain(angle, angle_config.angle_min, angle_config.angle_max);
        // Set target in scaled degrees
        pid_controller->target = (int)(target_angle * 10.0f);

        Serial.print("Set angle: "); Serial.print(target_angle, 1);
        Serial.print("° -> Scaled Target: "); Serial.print(pid_controller->target);
        Serial.println();
    }
    
    void enablePID(bool enable) {
        if (pid_controller == nullptr) return;
        
        pid_enabled = enable;
        setEnabled(pid_controller, enable ? 1 : 0);
        
        if (enable) {
            // Auto-set target to center (0 degrees) when PID is enabled
            setAngleTarget(0.0f);
            Serial.println("PID enabled - auto-targeting center (0°)");
        } else {
            // Stop motor when PID is disabled
            rudder_hbridge.stop(HBridgeDriverV2::Channel::M1);
        }
    }
    
    void updatePID() {
        if (pid_controller == nullptr || !pid_enabled) return;
        
        current_angle = adcToAngle(analogRead(A9));
        tick(pid_controller);
    }
    
    void setAngleMapping(int adc_min, int adc_max, float angle_min, float angle_max) {
        angle_config.adc_min = adc_min;
        angle_config.adc_max = adc_max;
        angle_config.angle_min = angle_min;
        angle_config.angle_max = angle_max;
    }
    
    // Access functions for external use
    float getCurrentAngle() { return current_angle; }
    float getTargetAngle() { return target_angle; }
    bool isPIDEnabled() { return pid_enabled; }
    void setPIDTunings(double kp, double ki, double kd) { 
        if (pid_controller != nullptr) {
            pid_controller->p = kp;
            pid_controller->i = ki;
            pid_controller->d = kd;
        }
    }
    void getPIDTunings(double& kp, double& ki, double& kd) {
        if (pid_controller != nullptr) {
            kp = pid_controller->p;
            ki = pid_controller->i;
            kd = pid_controller->d;
        } else {
            kp = ki = kd = 0.0;
        }
    }
}

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
        else if (strcmp(command, "angle") == 0 && arg != NULL) {
            float target = atof(arg);
            RudderAngle::setAngleTarget(target);
            Serial.print("Target angle set to "); Serial.print(target); Serial.println(" degrees");
        }
        else if (strcmp(command, "pid") == 0) {
            if (arg != NULL) {
                if (strcmp(arg, "on") == 0) {
                    RudderAngle::enablePID(true);
                    Serial.println("PID control enabled");
                } else if (strcmp(arg, "off") == 0) {
                    RudderAngle::enablePID(false);
                    Serial.println("PID control disabled");
                } else {
                    Serial.println("Usage: pid <on|off>");
                }
            } else {
                // Show PID status and current angle
                float current = RudderAngle::adcToAngle(analogRead(A9));
                Serial.print("Current angle: "); Serial.print(current, 1); Serial.println(" degrees");
                Serial.print("Target angle: "); Serial.print(RudderAngle::getTargetAngle(), 1); Serial.println(" degrees");
                Serial.print("PID enabled: "); Serial.println(RudderAngle::isPIDEnabled() ? "Yes" : "No");
            }
        }
        else if (strcmp(command, "map") == 0) {
            // arg is already the first parameter (adc_min)
            char* adc_max_str = strtok(NULL, " ");
            char* angle_min_str = strtok(NULL, " ");
            char* angle_max_str = strtok(NULL, " ");
            
            if (arg && adc_max_str && angle_min_str && angle_max_str) {
                int adc_min = atoi(arg);        // Use arg for first parameter
                int adc_max = atoi(adc_max_str);
                float angle_min = atof(angle_min_str);
                float angle_max = atof(angle_max_str);
                
                RudderAngle::setAngleMapping(adc_min, adc_max, angle_min, angle_max);
                Serial.print("Angle mapping set: ADC["); Serial.print(adc_min); Serial.print(","); Serial.print(adc_max);
                Serial.print("] -> Angle["); Serial.print(angle_min, 1); Serial.print(","); Serial.print(angle_max, 1); Serial.println("]");
            } else {
                Serial.println("Usage: map <adc_min> <adc_max> <angle_min> <angle_max>");
            }
        }
        else if (strcmp(command, "tune") == 0) {
            if (arg != NULL) {
                // Set new PID values
                char* ki_str = strtok(NULL, " ");
                char* kd_str = strtok(NULL, " ");
                
                if (ki_str && kd_str) {
                    double kp = atof(arg);      // Use arg for first parameter
                    double ki = atof(ki_str);
                    double kd = atof(kd_str);
                    
                    RudderAngle::setPIDTunings(kp, ki, kd);
                    Serial.print("PID gains set: P="); Serial.print(kp, 3);
                    Serial.print(", I="); Serial.print(ki, 3);
                    Serial.print(", D="); Serial.println(kd, 3);
                } else {
                    Serial.println("Usage: tune <kp> <ki> <kd>");
                }
            } else {
                // Show current PID values
                double kp, ki, kd;
                RudderAngle::getPIDTunings(kp, ki, kd);
                if (kp != 0.0 || ki != 0.0 || kd != 0.0) {
                    Serial.print("Current PID gains: P="); Serial.print(kp, 3);
                    Serial.print(", I="); Serial.print(ki, 3);
                    Serial.print(", D="); Serial.println(kd, 3);
                } else {
                    Serial.println("PID controller not initialized");
                }
            }
        }
        else if (strcmp(command, "test") == 0 && arg != NULL) {
            // Direct motor test bypassing PID
            float percentage = atof(arg);
            percentage = constrain(percentage, -20.0f, 20.0f); // Limit for safety
            
            RudderAngle::enablePID(false); // Disable PID first
            auto result = rudder_hbridge.setPercentage(percentage, HBridgeDriverV2::Channel::M1);
            
            Serial.print("Direct motor test: "); Serial.print(percentage, 1); Serial.print("% -> ");
            if (result == HBridgeDriverV2::ErrorCode::NONE) {
                Serial.println("OK");
            } else {
                Serial.println("ERROR");
            }
        }
        else if (strcmp(command, "help") == 0) {
            Serial.println("Commands:");
            Serial.println("p <-100-100>   - Set percentage (manual control)");
            Serial.println("pwm <1000-2000> - Set PWM input (manual control)");
            Serial.println("test <-20-20>  - Safe motor test (auto-disables PID)");
            Serial.println("stop           - Stop motor");
            Serial.println("status         - Show status");
            Serial.println("analog         - Read A9 analog value once");
            Serial.println("analog on      - Enable continuous analog monitoring");
            Serial.println("analog off     - Disable continuous analog monitoring");
            Serial.println("angle <degrees> - Set target angle for PID control");
            Serial.println("pid            - Show PID status and current angle");
            Serial.println("pid on         - Enable PID angle control");
            Serial.println("pid off        - Disable PID angle control");
            Serial.println("tune <kp> <ki> <kd> - Adjust PID gains (start with small values!)");
            Serial.println("tune               - Show current PID gains");
            Serial.println("map <adc_min> <adc_max> <angle_min> <angle_max> - Set angle mapping");
            Serial.println("");
            Serial.println("Quick PID presets:");
            Serial.println("  tune 0.1 0.0 0.0   - Gentle P-only");
            Serial.println("  tune 0.2 0.01 0.05 - Anti-oscillation (default)");
            Serial.println("  tune 0.05 0.0 0.02 - Ultra-smooth");
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

    Serial.print("Starting: "); Serial.println(TEST_NAME);
    Serial.println("====================================");

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
    
    // Initialize PID controller with anti-oscillation tuning
    Serial.println("Initializing PID controller...");
    RudderAngle::initPID(0.2, 0.01, 0.05);  // Lower P, small I, D for damping
    
    Commands::init();
    Serial.println("Ready! Use 'help' to see available commands.");
}

void loop() {
    Commands::processInput();
    Commands::updateAnalogMonitoring();
    RudderAngle::updatePID();
    delay(10);
}