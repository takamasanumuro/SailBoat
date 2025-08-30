#include <Arduino.h>
#include "analog_voltage_generator.hpp"

#ifdef MOTOR_TEST_MODE
    #define TEST_NAME "Analog Voltage Generator"
#endif

void setup() {
    Serial.begin(9600);
    delay(1000);
    
    #ifdef MOTOR_TEST_MODE
        Serial.println("====================================");
        Serial.print("Starting: "); Serial.println(TEST_NAME);
        Serial.println("ENVIRONMENT: MOTOR");
        Serial.println("====================================");
    #endif
    
    // Configure voltage generator
    AnalogVoltageGenerator::VoltageConfig config;
    config.min_voltage_mv = 800.0f;   // 0.8V minimum
    config.max_voltage_mv = 4000.0f;  // 4.0V maximum
    config.pwm_pin = 2;               // PWM output pin
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
    Serial.println("Output will be on pin " + String(config.pwm_pin));
    Serial.println("Example: 'set 50' for 50% output (2.4V)");
    Serial.println();

    constexpr int motor_gnd_pin = A1;
    pinMode(motor_gnd_pin, OUTPUT); digitalWrite(motor_gnd_pin, LOW);

    constexpr int motor_signal_pin = A0;
    pinMode(motor_signal_pin, INPUT);
}

void loop() {
    // Process serial commands
    AnalogVoltageGenerator::CommandInterface::processSerialInput();
    
    // Add any other periodic tasks here
    delay(10);  // Small delay to prevent overwhelming the serial buffer
}