#include <Arduino.h>
#include "config.hpp"
#include "timing_manager.hpp"
#include "rudder_control.hpp"
#include "command_interface.hpp"
#include "analog_voltage_generator.hpp"
#include "mavlink_communication.hpp"

#ifdef ENVIRONMENT_MAIN
    #define TEST_NAME "Integrated Sailboat Autopilot System"
#endif

// Global Mavlink communication instance
MavlinkCommunication mavlink;

void setup() {
    Serial.begin(Config::System::SERIAL_BAUD_RATE);
    
    #ifdef ENVIRONMENT_MAIN
		Serial.print("Starting: "); Serial.println(TEST_NAME);
        Serial.println("ENVIRONMENT: MAIN");
    #endif
    
	delay(1000);

    // Initialize timing manager
    TimingManager::init();

    // Initialize rudder controller
    if (!RudderControl::controller.init()) {
        Serial.println("FATAL: Failed to initialize rudder controller!");
        while (1) delay(1000);
    }
    
    // Initialize PID controller with default settings
    RudderControl::controller.initPID();
    
    // Initialize analog voltage generator
    AnalogVoltageGenerator::VoltageConfig voltage_config;
    voltage_config.min_voltage_mv = Config::Control::MotorVoltage::MIN_VOLTAGE_MV;
    voltage_config.max_voltage_mv = Config::Control::MotorVoltage::MAX_VOLTAGE_MV;
    voltage_config.pwm_pin = Config::Pins::MotorVoltage::PWM_PIN;
    voltage_config.pwm_frequency_hz = Config::Control::MotorVoltage::PWM_FREQUENCY_HZ;
    voltage_config.pwm_resolution_bits = Config::Control::MotorVoltage::PWM_RESOLUTION_BITS;
    
    if (!AnalogVoltageGenerator::generator.init(voltage_config)) {
        Serial.println("FATAL: Failed to initialize voltage generator!");
        while (1) delay(1000);
    }
    
    // Configure motor control pins
    pinMode(Config::Pins::MotorVoltage::GND_PIN, OUTPUT);
    digitalWrite(Config::Pins::MotorVoltage::GND_PIN, LOW);
    pinMode(Config::Pins::MotorVoltage::SIGNAL_PIN, INPUT);
    
    // Initialize command interface
    CommandInterface::processor.init();
    
    // Initialize Mavlink communication
    mavlink.initialize();
    
    // Set mavlink instance in command processor
    CommandInterface::processor.setMavlinkInstance(&mavlink);

	Serial.println("System initialization complete");
    Serial.println();
    Serial.println("=== PIN CONNECTIONS ===");
    
    // Rudder H-Bridge connections
    Serial.println("Rudder H-Bridge Motor Control:");
    Serial.print("  INA (Direction A): Pin "); Serial.println(Config::Pins::Rudder::INA_PIN);
    Serial.print("  INB (Direction B): Pin "); Serial.println(Config::Pins::Rudder::INB_PIN);
    Serial.print("  PWM (Speed):       Pin "); Serial.println(Config::Pins::Rudder::PWM_PIN);
    Serial.println();
    
    // Rudder potentiometer connections
    Serial.println("Rudder Angle Potentiometer:");
    Serial.print("  Power (+5V):  Pin A"); Serial.print(Config::Pins::Potentiometers::Rudder::POWER - A0); 
    Serial.print(" (Pin "); Serial.print(Config::Pins::Potentiometers::Rudder::POWER); Serial.println(")");
    Serial.print("  Signal (ADC): Pin A"); Serial.print(Config::Pins::Potentiometers::Rudder::SIGNAL - A0);
    Serial.print(" (Pin "); Serial.print(Config::Pins::Potentiometers::Rudder::SIGNAL); Serial.println(")");
    Serial.print("  Ground (GND): Pin A"); Serial.print(Config::Pins::Potentiometers::Rudder::GND - A0);
    Serial.print(" (Pin "); Serial.print(Config::Pins::Potentiometers::Rudder::GND); Serial.println(")");
    Serial.println();
    
    // Motor voltage generator connections
    Serial.println("Motor Voltage Generator:");
    Serial.print("  PWM Output:   Pin "); Serial.println(Config::Pins::MotorVoltage::PWM_PIN);
    Serial.print("  Ground (GND): Pin A"); Serial.print(Config::Pins::MotorVoltage::GND_PIN - A0);
    Serial.print(" (Pin "); Serial.print(Config::Pins::MotorVoltage::GND_PIN); Serial.println(")");
    Serial.print("  Signal (ADC): Pin A"); Serial.print(Config::Pins::MotorVoltage::SIGNAL_PIN - A0);
    Serial.print(" (Pin "); Serial.print(Config::Pins::MotorVoltage::SIGNAL_PIN); Serial.println(")");
    Serial.println();
    
    // Configuration summary
    Serial.println("=== CONFIGURATION ===");
    Serial.print("Rudder angle range: "); Serial.print(Config::RudderAngle::ANGLE_MIN);
    Serial.print("\u00b0 to "); Serial.print(Config::RudderAngle::ANGLE_MAX); Serial.println("\u00b0");
    Serial.print("ADC range: "); Serial.print(Config::RudderAngle::ADC_MIN);
    Serial.print(" to "); Serial.println(Config::RudderAngle::ADC_MAX);
    Serial.print("Motor voltage range: "); Serial.print(Config::Control::MotorVoltage::MIN_VOLTAGE_MV);
    Serial.print(" to "); Serial.print(Config::Control::MotorVoltage::MAX_VOLTAGE_MV); Serial.println(" mV");
    Serial.println("======================");
}

void loop() {
    // Update Mavlink communication (highest priority)
    mavlink.update();
    
    // Check if Mavlink has control priority
    if (mavlink.has_valid_data()) {
        // Mavlink takes priority - apply RC commands
        int rudder_cmd = mavlink.get_rudder_command();
        int throttle_cmd = mavlink.get_throttle_command();
        MavlinkCommunication::ControlMode mode = mavlink.get_control_mode();
        
        // Apply rudder control based on mode
        if (mode == MavlinkCommunication::ControlMode::ANGLE_CONTROL) {
            // Convert rudder command to angle and use PID control
            //Enable PID first
            RudderControl::controller.enablePID(true);
            float target_angle = map(rudder_cmd, -100, 100, 
                                   Config::RudderAngle::ANGLE_MIN, 
                                   Config::RudderAngle::ANGLE_MAX);
            RudderControl::controller.setAngleTarget(target_angle);
        } else {
            // Direct speed control
            RudderControl::controller.enablePID(false);
            RudderControl::controller.setPercentage(rudder_cmd);
            Serial.print(F("Rudder (CH0): ")); Serial.print(rudder_cmd); Serial.println(F("%"));
        }
        
        // Apply throttle control
        AnalogVoltageGenerator::generator.setVoltagePercentage(throttle_cmd);
        
        // Update PID control if in angle mode
        if (mode == MavlinkCommunication::ControlMode::ANGLE_CONTROL) {
            RudderControl::controller.updatePID();
        }
    } else {
        // Manual/GUI control mode - process user commands
        CommandInterface::processor.processInput();
        
        // Update analog monitoring if enabled
        CommandInterface::processor.updateAnalogMonitoring();
        
        // Update rudder PID control
        RudderControl::controller.updatePID();
    }
}