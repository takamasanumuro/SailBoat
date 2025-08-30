#include "analog_voltage_generator.hpp"

namespace AnalogVoltageGenerator {
    
    // Global instance
    VoltageGenerator generator;
    
    // VoltageGenerator implementation
    VoltageGenerator::VoltageGenerator() 
        : initialized(false), current_percentage(0.0f), current_pwm_value(0) {
    }
    
    bool VoltageGenerator::init(const VoltageConfig& cfg) {
        config = cfg;
        
        // Validate configuration
        if (config.min_voltage_mv >= config.max_voltage_mv) {
            Serial.println("ERROR: Invalid voltage range");
            return false;
        }
        
        if (config.pwm_resolution_bits < 8 || config.pwm_resolution_bits > 10) {
            Serial.println("ERROR: PWM resolution must be 8, 9, or 10 bits");
            return false;
        }
        
        // Configure PWM pin (using simple analogWrite)
        pinMode(config.pwm_pin, OUTPUT);
        
        // Force 8-bit resolution for analogWrite
        config.pwm_resolution_bits = 8;
        
        initialized = true;
        Serial.println("Voltage generator initialized");
        Serial.print("Range: ");
        Serial.print(config.min_voltage_mv);
        Serial.print(" - ");
        Serial.print(config.max_voltage_mv);
        Serial.println(" mV");
        Serial.print("PWM Pin: ");
        Serial.println(config.pwm_pin);
        Serial.print("Resolution: ");
        Serial.print(config.pwm_resolution_bits);
        Serial.println(" bits");

        // Set initial output to 0%
        setVoltagePercentage(0.0f);
        
        return true;
    }
    
    bool VoltageGenerator::setVoltagePercentage(float percentage) {
        if (!initialized) {
            Serial.println("ERROR: Generator not initialized");
            return false;
        }
        
        if (!Utils::isValidPercentage(percentage)) {
            Serial.println("ERROR: Percentage must be 0-100");
            return false;
        }
        
        current_percentage = percentage;
        current_pwm_value = percentageToPWM(percentage);
        
        // Set PWM output using analogWrite (8-bit, 0-255)
        analogWrite(config.pwm_pin, current_pwm_value);
        
        return true;
    }
    
    bool VoltageGenerator::setVoltageMV(float voltage_mv) {
        if (!initialized) {
            Serial.println("ERROR: Generator not initialized");
            return false;
        }
        
        if (!Utils::isValidVoltage(voltage_mv, config.min_voltage_mv, config.max_voltage_mv)) {
            Serial.print("ERROR: Voltage must be ");
            Serial.print(config.min_voltage_mv);
            Serial.print(" - ");
            Serial.print(config.max_voltage_mv);
            Serial.println(" mV");
            return false;
        }
        
        float percentage = Utils::mapFloat(voltage_mv, config.min_voltage_mv, config.max_voltage_mv, 0.0f, 100.0f);
        return setVoltagePercentage(percentage);
    }
    
    float VoltageGenerator::getCurrentVoltage() const {
        return Utils::mapFloat(current_percentage, 0.0f, 100.0f, config.min_voltage_mv, config.max_voltage_mv);
    }
    
    uint16_t VoltageGenerator::percentageToPWM(float percentage) {
        // Map percentage to voltage, then voltage to PWM
        float target_voltage_mv = Utils::mapFloat(percentage, 0.0f, 100.0f, 
                                                config.min_voltage_mv, config.max_voltage_mv);
        
        // Map voltage to PWM (assuming 5000mV = 255 PWM)
        uint16_t max_pwm = (1 << config.pwm_resolution_bits) - 1; // 255 for 8-bit
        float pwm_value = (target_voltage_mv / 5000.0f) * max_pwm;
        
        return (uint16_t)pwm_value;
    }
    
    void VoltageGenerator::setVoltageRange(float min_mv, float max_mv) {
        if (min_mv >= max_mv) {
            Serial.println("ERROR: Invalid voltage range");
            return;
        }
        
        config.min_voltage_mv = min_mv;
        config.max_voltage_mv = max_mv;
        
        Serial.print("Voltage range updated: ");
        Serial.print(min_mv);
        Serial.print(" - ");
        Serial.print(max_mv);
        Serial.println(" mV");
        
        // Recalculate current PWM value
        if (initialized) {
            setVoltagePercentage(current_percentage);
        }
    }
    
    void VoltageGenerator::setPWMPin(uint8_t pin) {
        config.pwm_pin = pin;
        if (initialized) {
            Serial.println("WARNING: Pin changed - reinitialize generator");
        }
    }
    
    void VoltageGenerator::disable() {
        analogWrite(config.pwm_pin, 0);
        current_pwm_value = 0;
    }
    
    void VoltageGenerator::enable() {
        if (initialized) {
            setVoltagePercentage(current_percentage);
        }
    }
    
    bool VoltageGenerator::isEnabled() const {
        return initialized && (current_pwm_value > 0);
    }
    
    void VoltageGenerator::runCalibrationSequence() {
        if (!initialized) {
            Serial.println("ERROR: Generator not initialized");
            return;
        }
        
        Serial.println("Starting calibration sequence...");
        
        float test_percentages[] = {0.0f, 25.0f, 50.0f, 75.0f, 100.0f};
        
        for (int i = 0; i < 5; i++) {
            Serial.print("Setting ");
            Serial.print(test_percentages[i]);
            Serial.print("% (");
            Serial.print(Utils::mapFloat(test_percentages[i], 0.0f, 100.0f, config.min_voltage_mv, config.max_voltage_mv));
            Serial.print(" mV)");
            
            setVoltagePercentage(test_percentages[i]);
            Serial.print(" - PWM: ");
            Serial.println(current_pwm_value);
            
            delay(2000);  // Hold for 2 seconds
        }
        
        Serial.println("Calibration sequence complete");
        setVoltagePercentage(0.0f);  // Return to 0%
    }
    
    void VoltageGenerator::outputTestPattern() {
        if (!initialized) {
            Serial.println("ERROR: Generator not initialized");
            return;
        }
        
        Serial.println("Running test pattern (10 second ramp)...");
        
        for (int i = 0; i <= 100; i += 5) {
            setVoltagePercentage(i);
            delay(500);  // 500ms per step = 10 seconds total
        }
        
        Serial.println("Test pattern complete");
        setVoltagePercentage(0.0f);
    }
    
    // Command Interface implementation
    namespace CommandInterface {
        static char input_buffer[64];
        static int buffer_index = 0;
        
        void init() {
            Serial.println("Analog Voltage Generator Command Interface");
            Serial.println("Type 'help' for available commands");
            Serial.print("> ");
        }
        
        void processSerialInput() {
            while (Serial.available()) {
                char c = Serial.read();
                
                if (c == '\r') continue;  // Ignore carriage return
                
                if (c == '\n') {
                    // Process command
                    input_buffer[buffer_index] = '\0';
                    buffer_index = 0;
                    
                    if (strlen(input_buffer) > 0) {
                        parseCommand(input_buffer);
                    }
                    Serial.print("> ");
                } else if (buffer_index < sizeof(input_buffer) - 1) {
                    input_buffer[buffer_index++] = c;
                    Serial.print(c);  // Echo character
                } else {
                    // Buffer overflow
                    buffer_index = 0;
                    Serial.println("\nERROR: Command too long");
                    Serial.print("> ");
                }
            }
        }
        
        void parseCommand(const char* command) {
            Serial.println();  // New line after command
            
            // Parse command and arguments
            char cmd_copy[64];
            strncpy(cmd_copy, command, sizeof(cmd_copy) - 1);
            cmd_copy[sizeof(cmd_copy) - 1] = '\0';
            
            char* cmd = strtok(cmd_copy, " ");
            char* arg1 = strtok(NULL, " ");
            char* arg2 = strtok(NULL, " ");
            
            if (cmd == NULL) return;
            
            // Convert command to lowercase
            for (char* p = cmd; *p; p++) {
                *p = tolower(*p);
            }
            
            // Handle commands
            if (strcmp(cmd, "set") == 0 && arg1 != NULL) {
                float value = atof(arg1);
                handleSetPercentage(value);
            }
            else if (strcmp(cmd, "setp") == 0 && arg1 != NULL) {
                float value = atof(arg1);
                handleSetPercentage(value);
            }
            else if (strcmp(cmd, "setv") == 0 && arg1 != NULL) {
                float value = atof(arg1);
                handleSetVoltage(value);
            }
            else if (strcmp(cmd, "range") == 0 && arg1 != NULL && arg2 != NULL) {
                float min_mv = atof(arg1);
                float max_mv = atof(arg2);
                handleSetRange(min_mv, max_mv);
            }
            else if (strcmp(cmd, "status") == 0) {
                handleGetStatus();
            }
            else if (strcmp(cmd, "cal") == 0) {
                handleCalibration();
            }
            else if (strcmp(cmd, "config") == 0) {
                handleConfig();
            }
            else if (strcmp(cmd, "help") == 0) {
                handleHelp();
            }
            else if (strcmp(cmd, "test") == 0) {
                generator.outputTestPattern();
            }
            else if (strcmp(cmd, "off") == 0) {
                generator.disable();
                sendResponse("Output disabled");
            }
            else if (strcmp(cmd, "on") == 0) {
                generator.enable();
                sendResponse("Output enabled");
            }
            else {
                sendError("Unknown command. Type 'help' for available commands.");
            }
        }
        
        void handleSetPercentage(float percentage) {
            if (generator.setVoltagePercentage(percentage)) {
                Serial.print("Set to ");
                Serial.print(percentage);
                Serial.print("% (");
                Serial.print(generator.getCurrentVoltage());
                Serial.println(" mV)");
            }
        }
        
        void handleSetVoltage(float voltage_mv) {
            if (generator.setVoltageMV(voltage_mv)) {
                Serial.print("Set to ");
                Serial.print(voltage_mv);
                Serial.print(" mV (");
                Serial.print(generator.getCurrentPercentage());
                Serial.println("%)");
            }
        }
        
        void handleSetRange(float min_mv, float max_mv) {
            generator.setVoltageRange(min_mv, max_mv);
        }
        
        void handleGetStatus() {
            sendStatus();
        }
        
        void handleCalibration() {
            generator.runCalibrationSequence();
        }
        
        void handleConfig() {
            VoltageConfig cfg = generator.getConfig();
            Serial.println("=== Configuration ===");
            Serial.print("Voltage Range: ");
            Serial.print(cfg.min_voltage_mv);
            Serial.print(" - ");
            Serial.print(cfg.max_voltage_mv);
            Serial.println(" mV");
            Serial.print("PWM Pin: ");
            Serial.println(cfg.pwm_pin);
            Serial.print("PWM Frequency: ");
            Serial.print(cfg.pwm_frequency_hz);
            Serial.println(" Hz");
            Serial.print("PWM Resolution: ");
            Serial.print(cfg.pwm_resolution_bits);
            Serial.println(" bits");
            Serial.println("===================");
        }
        
        void handleHelp() {
            Serial.println("=== Available Commands ===");
            Serial.println("set <0-100>       - Set output percentage (0-100%)");
            Serial.println("setp <0-100>      - Set output percentage (alias for 'set')");
            Serial.println("setv <voltage>    - Set output voltage in mV");
            Serial.println("range <min> <max> - Set voltage range in mV");
            Serial.println("status            - Show current status");
            Serial.println("config            - Show configuration");
            Serial.println("cal               - Run calibration sequence");
            Serial.println("test              - Run test pattern");
            Serial.println("on                - Enable output");
            Serial.println("off               - Disable output");
            Serial.println("help              - Show this help");
            Serial.println("=========================");
            Serial.println("Examples:");
            Serial.println("  set 50          - Set to 50%");
            Serial.println("  setv 2400       - Set to 2400 mV");
            Serial.println("  range 500 3300  - Set range 500-3300 mV");
        }
        
        void sendResponse(const char* message) {
            Serial.println(message);
        }
        
        void sendError(const char* error) {
            Serial.print("ERROR: ");
            Serial.println(error);
        }
        
        void sendStatus() {
            Serial.println("=== Status ===");
            Serial.print("Initialized: ");
            Serial.println(generator.isInitialized() ? "Yes" : "No");
            Serial.print("Enabled: ");
            Serial.println(generator.isEnabled() ? "Yes" : "No");
            Serial.print("Current Percentage: ");
            Serial.print(generator.getCurrentPercentage());
            Serial.println("%");
            Serial.print("Current Voltage: ");
            Serial.print(generator.getCurrentVoltage());
            Serial.println(" mV");
            Serial.print("PWM Value: ");
            Serial.println(generator.getCurrentPWM());
            Serial.println("=============");
        }
    }
    
    // Utility functions implementation
    namespace Utils {
        float mapFloat(float value, float in_min, float in_max, float out_min, float out_max) {
            return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        }
        
        uint16_t calculatePWMValue(float voltage_mv, float min_voltage, float max_voltage, uint16_t max_pwm) {
            float ratio = (voltage_mv - min_voltage) / (max_voltage - min_voltage);
            return (uint16_t)(ratio * max_pwm);
        }
        
        float calculateVoltage(uint16_t pwm_value, float min_voltage, float max_voltage, uint16_t max_pwm) {
            float ratio = (float)pwm_value / max_pwm;
            return min_voltage + ratio * (max_voltage - min_voltage);
        }
        
        bool isValidPercentage(float percentage) {
            return (percentage >= 0.0f && percentage <= 100.0f);
        }
        
        bool isValidVoltage(float voltage_mv, float min_mv, float max_mv) {
            return (voltage_mv >= min_mv && voltage_mv <= max_mv);
        }
    }
    
}