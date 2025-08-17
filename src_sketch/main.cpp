#include <Arduino.h>
#include "analog_voltage_generator.hpp"

#ifdef SKETCH_MODE
    #define SKETCH_VERSION "1.1.0"
    #define SKETCH_NAME "Analog Voltage Generator"
#endif

void setup() {
    Serial.begin(9600);
    delay(1000);
    
    #ifdef SKETCH_MODE
        Serial.println("====================================");
        Serial.print("Starting: "); Serial.println(SKETCH_NAME);
        Serial.print("Version: "); Serial.println(SKETCH_VERSION);
        Serial.println("Environment: SKETCH");
        Serial.println("====================================");
    #endif
    
    Serial.println("   Analog Voltage Generator Sketch");
    Serial.println("====================================");
    
    // Configure voltage generator
    AnalogVoltageGenerator::VoltageConfig config;
    config.min_voltage_mv = 800.0f;   // 0.8V minimum
    config.max_voltage_mv = 4000.0f;  // 4.0V maximum
    config.pwm_pin = 9;               // PWM output pin
    config.pwm_frequency_hz = 490;    // Arduino default PWM frequency
    config.pwm_resolution_bits = 8;   // 8-bit resolution (0-255)
    
    // Initialize the voltage generator
    if (!AnalogVoltageGenerator::generator.init(config)) {
        Serial.println("FATAL: Failed to initialize voltage generator!");
        while (1) {
            delay(1000);
        }
    }
    
    // Initialize command interface
    AnalogVoltageGenerator::CommandInterface::init();
    Serial.println();
    Serial.println("Ready! Use commands to control voltage output.");
    Serial.println("Output will be on pin 9 (requires low-pass filter).");
    Serial.println("Example: 'set 50' for 50% output (2.4V)");
    Serial.println();
}

void loop() {
    // Process serial commands
    AnalogVoltageGenerator::CommandInterface::processSerialInput();
    
    // Add any other periodic tasks here
    delay(10);  // Small delay to prevent overwhelming the serial buffer
}