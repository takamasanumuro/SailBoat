#include "rudder_controller.hpp"

namespace RudderController {
    
    // Global instance
    Controller rudder;
    
    Controller::Controller() : hbridge(nullptr), initialized(false), 
                              current_pwm(0), current_percentage(0), current_pwm_input(1500) {
    }
    
    Controller::~Controller() {
        if (hbridge) {
            delete hbridge;
        }
    }
    
    bool Controller::init(const Config& cfg) {
        config = cfg;
        
        // Create H-bridge driver
        hbridge = new HBridgeDriver(config.direction_pin_a, config.direction_pin_b, config.pwm_pin);
        if (!hbridge) {
            Serial.println("ERROR: Failed to create H-bridge driver");
            return false;
        }
        
        // Initialize H-bridge
        hbridge->init_channel_A();
        
        // Set initial state
        stop();
        
        initialized = true;
        
        Serial.println("Rudder controller initialized");
        Serial.print("Pins - DIR_A: "); Serial.print(config.direction_pin_a);
        Serial.print(", DIR_B: "); Serial.print(config.direction_pin_b);
        Serial.print(", PWM: "); Serial.println(config.pwm_pin);
        Serial.print("Max PWM: "); Serial.println(config.max_pwm);
        
        return true;
    }
    
    bool Controller::setPercentage(int percentage) {
        if (!initialized) {
            Serial.println("ERROR: Controller not initialized");
            return false;
        }
        
        if (percentage < -100 || percentage > 100) {
            Serial.println("ERROR: Percentage must be -100 to +100");
            return false;
        }
        
        current_percentage = percentage;
        
        // Convert percentage to PWM (-100% = -240, +100% = +240)
        current_pwm = (int16_t)((float)percentage * config.max_pwm / 100.0f);
        
        // Convert percentage to PWM input equivalent (1000-2000)
        current_pwm_input = 1500 + (percentage * 5);  // 1000 at -100%, 2000 at +100%
        
        // Set H-bridge PWM
        hbridge->setPWM(current_pwm, HBridgeDriver::M1);
        
        return true;
    }
    
    bool Controller::setPWMInput(int pwm_input) {
        if (!initialized) {
            Serial.println("ERROR: Controller not initialized");
            return false;
        }
        
        if (pwm_input < 1000 || pwm_input > 2000) {
            Serial.println("ERROR: PWM input must be 1000 to 2000");
            return false;
        }
        
        current_pwm_input = pwm_input;
        
        // Convert PWM input to percentage (1500 = 0%, 1000 = -100%, 2000 = +100%)
        current_percentage = (pwm_input - 1500) / 5;
        
        // Convert to H-bridge PWM
        current_pwm = (int16_t)((float)current_percentage * config.max_pwm / 100.0f);
        
        // Set H-bridge PWM
        hbridge->setPWM(current_pwm, HBridgeDriver::M1);
        
        return true;
    }
    
    bool Controller::stop() {
        if (!initialized) {
            return false;
        }
        
        current_pwm = 0;
        current_percentage = 0;
        current_pwm_input = 1500;
        
        hbridge->setPWM(0, HBridgeDriver::M1);
        
        return true;
    }
    
    void Controller::printStatus() {
        Serial.println("=== Rudder Status ===");
        Serial.print("Percentage: "); Serial.print(current_percentage); Serial.println("%");
        Serial.print("PWM Input: "); Serial.println(current_pwm_input);
        Serial.print("H-Bridge PWM: "); Serial.println(current_pwm);
        
        // Direction indicator
        if (current_pwm > 0) {
            Serial.println("Direction: STARBOARD (Right)");
        } else if (current_pwm < 0) {
            Serial.println("Direction: PORT (Left)");
        } else {
            Serial.println("Direction: CENTER (Stopped)");
        }
        
        // PWM bar
        Serial.print("PWM Bar: [");
        int bars = abs(current_pwm) * 20 / config.max_pwm;
        for (int i = 0; i < 10; i++) {
            if (current_pwm < 0 && i < (10 - bars/2)) Serial.print("<");
            else if (current_pwm > 0 && i >= (10 - bars/2)) Serial.print(">");
            else Serial.print("-");
        }
        Serial.println("]");
        Serial.println("==================");
    }
    
    // Command interface implementation
    namespace Commands {
        static char input_buffer[32];
        static int buffer_index = 0;
        
        void init() {
            Serial.println("===================================");
            Serial.println("    Rudder Controller Interface");
            Serial.println("===================================");
            Serial.println("Commands:");
            Serial.println("  p <-100 to 100>  - Set percentage");
            Serial.println("  pwm <1000-2000>  - Set PWM input");
            Serial.println("  stop              - Stop rudder");
            Serial.println("  status            - Show status");
            Serial.println("  help              - Show commands");
            Serial.println("===================================");
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
                } else if (buffer_index < sizeof(input_buffer) - 1) {
                    input_buffer[buffer_index++] = c;
                    Serial.print(c);  // Echo
                } else {
                    buffer_index = 0;
                    Serial.println("\nERROR: Command too long");
                    Serial.print("> ");
                }
            }
        }
        
        void parseCommand(const char* cmd) {
            Serial.println();  // New line
            
            char cmd_copy[32];
            strncpy(cmd_copy, cmd, sizeof(cmd_copy) - 1);
            cmd_copy[sizeof(cmd_copy) - 1] = '\0';
            
            char* command = strtok(cmd_copy, " ");
            char* arg = strtok(NULL, " ");
            
            if (command == NULL) return;
            
            // Convert to lowercase
            for (char* p = command; *p; p++) {
                *p = tolower(*p);
            }
            
            if (strcmp(command, "p") == 0 && arg != NULL) {
                int percentage = atoi(arg);
                handlePercentage(percentage);
            }
            else if (strcmp(command, "pwm") == 0 && arg != NULL) {
                int pwm_input = atoi(arg);
                handlePWMInput(pwm_input);
            }
            else if (strcmp(command, "stop") == 0) {
                handleStop();
            }
            else if (strcmp(command, "status") == 0) {
                handleStatus();
            }
            else if (strcmp(command, "help") == 0) {
                handleHelp();
            }
            else {
                Serial.println("ERROR: Unknown command. Type 'help' for available commands.");
            }
        }
        
        void handlePercentage(int percentage) {
            if (rudder.setPercentage(percentage)) {
                Serial.print("Set rudder to "); Serial.print(percentage); Serial.println("%");
                rudder.printStatus();
            }
        }
        
        void handlePWMInput(int pwm_input) {
            if (rudder.setPWMInput(pwm_input)) {
                Serial.print("Set rudder PWM input to "); Serial.println(pwm_input);
                rudder.printStatus();
            }
        }
        
        void handleStop() {
            if (rudder.stop()) {
                Serial.println("Rudder stopped");
            } else {
                Serial.println("ERROR: Failed to stop rudder");
            }
        }
        
        void handleStatus() {
            rudder.printStatus();
        }
        
        void handleHelp() {
            Serial.println("=== Available Commands ===");
            Serial.println("p <-100 to 100>   - Set rudder percentage");
            Serial.println("                     -100 = full port (left)");
            Serial.println("                        0 = center");
            Serial.println("                     +100 = full starboard (right)");
            Serial.println();
            Serial.println("pwm <1000-2000>   - Set PWM input (like RC)");
            Serial.println("                     1000 = full port (left)");
            Serial.println("                     1500 = center");
            Serial.println("                     2000 = full starboard (right)");
            Serial.println();
            Serial.println("stop               - Stop rudder (center position)");
            Serial.println("status             - Show current rudder status");
            Serial.println("help               - Show this help");
            Serial.println("=========================");
            Serial.println();
            Serial.println("Examples:");
            Serial.println("  p -50       - 50% to port (left)");
            Serial.println("  p 75        - 75% to starboard (right)");
            Serial.println("  pwm 1200    - Port position");
            Serial.println("  pwm 1800    - Starboard position");
            Serial.println("  stop        - Center rudder");
        }
    }
    
}