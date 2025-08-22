#include <Arduino.h>
#include "rudder_controller.hpp"

#ifdef RUDDER_TEST_MODE
    #define TEST_VERSION "1.0.0"
    #define TEST_NAME "Rudder H-Bridge Tester"
#endif

void setup() {
    Serial.begin(9600);
    delay(1000);
    
    #ifdef RUDDER_TEST_MODE
        Serial.println("===================================");
        Serial.print("Starting: "); Serial.println(TEST_NAME);
        Serial.print("Version: "); Serial.println(TEST_VERSION);
        Serial.println("Environment: RUDDER TEST");
        Serial.println("===================================");
    #endif
    
    Serial.println("Initializing rudder controller...");
    
    // Configure rudder controller
    RudderController::Config config;
    config.direction_pin_a = 43;  // INA pin - adjust as needed
    config.direction_pin_b = 42;  // INB pin - adjust as needed  
    config.pwm_pin = 44;           // PWM pin - adjust as needed
    config.max_pwm = 240;         // Max PWM (matches HBridgeDriver)
    
    // Initialize controller
    if (!RudderController::rudder.init(config)) {
        Serial.println("FATAL: Failed to initialize rudder controller!");
        while (1) {
            delay(1000);
        }
    }
    
    // Initialize command interface
    RudderController::Commands::init();
    
    Serial.println();
    Serial.println("Rudder controller ready!");
    Serial.println("Try: 'p 50' for 50% starboard or 'pwm 1750' for starboard");
    Serial.println();
}

void loop() {
    // Process serial commands
    RudderController::Commands::processInput();
    
    delay(10);  // Small delay
}