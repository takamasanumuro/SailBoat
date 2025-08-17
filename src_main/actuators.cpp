#include "actuators.hpp"
#include "HBridgeDriver.hpp"
#include "logger.hpp"
#include "timing_manager.hpp"

extern HBridgeDriver winchActuator;
extern HBridgeDriver rudderActuator;
extern HBridgeDriver throttleMotor;
extern const uint8_t pixHawkReadingPins[];
extern int16_t pixHawkReadingsPWM[];

namespace Actuators {
    
    void initializeHardware() {
        initializePowerPins();
        initializePotentiometers();
        initializePixhawkPins();
        initializeMotorDrivers();
    }
    
    void initializePowerPins() {
        for (auto& pin : Config::Pins::Power::PROPULSION_POWER) {
            pinMode(pin, OUTPUT);
            digitalWrite(pin, HIGH);
        }
        
        for (auto& pin : Config::Pins::Power::SAIL_GND) {
            pinMode(pin, OUTPUT);
            digitalWrite(pin, LOW);
        }
    }
    
    void initializePotentiometers() {
        // Rudder potentiometer setup
        pinMode(Config::Pins::Potentiometers::Rudder::GND, OUTPUT);
        digitalWrite(Config::Pins::Potentiometers::Rudder::GND, LOW);
        pinMode(Config::Pins::Potentiometers::Rudder::SIGNAL, INPUT);
        pinMode(Config::Pins::Potentiometers::Rudder::POWER, OUTPUT);
        digitalWrite(Config::Pins::Potentiometers::Rudder::POWER, HIGH);
        
        // Sail potentiometer setup
        pinMode(Config::Pins::Potentiometers::Sail::SIGNAL, INPUT);
    }
    
    void initializePixhawkPins() {
        for (auto& pin : pixHawkReadingPins) {
            pinMode(pin, INPUT);
        }
    }
    
    void initializeMotorDrivers() {
        rudderActuator.init_channel_A();
        winchActuator.init_channel_A();
        throttleMotor.init_channel_A();
    }
    
    int readRudderAngle() {
        int pot_rudder_ADC = analogRead(Config::Pins::Potentiometers::Rudder::SIGNAL);
        int pot_rudder_angle = map(pot_rudder_ADC, 
                                  Config::Calibration::Rudder::ADC_MIN_THRESHOLD, 
                                  Config::Calibration::Rudder::ADC_MAX_THRESHOLD, 
                                  Config::Calibration::Rudder::MIN_ANGLE, 
                                  Config::Calibration::Rudder::MAX_ANGLE);
        pot_rudder_angle += Config::Calibration::Rudder::ANGLE_OFFSET;
        
        #ifdef PRINT_RUDDER_READINGS
        if (TimingManager::getRudderReadTimer().isReady()) {
            Logger::logSensorReading("Rudder", pot_rudder_ADC, pot_rudder_angle);
        }
        #endif
        
        return pot_rudder_angle;
    }
    
    int readSailAngle() {
        int pot_sail_ADC = analogRead(Config::Pins::Potentiometers::Sail::SIGNAL);
        int pot_sail_angle = map(pot_sail_ADC, 
                                Config::Calibration::Sail::ADC_MIN_THRESHOLD, 
                                Config::Calibration::Sail::ADC_MAX_THRESHOLD, 
                                Config::Calibration::Sail::MAX_ANGLE, 
                                Config::Calibration::Sail::MIN_ANGLE);
        pot_sail_angle += Config::Calibration::Sail::ANGLE_OFFSET;
        
        #ifdef PRINT_SAIL_READINGS
        if (TimingManager::getSailReadTimer().isReady()) {
            Logger::logSensorReading("Sail", pot_sail_ADC, pot_sail_angle);
        }
        #endif
        
        return pot_sail_angle;
    }
    
    bool isRudderPotentiometerValid(int adc_reading) {
        return (adc_reading >= Config::Safety::Potentiometer::CRITICAL_LOW && 
                adc_reading <= Config::Safety::Potentiometer::CRITICAL_HIGH);
    }
    
    bool isSailPotentiometerValid(int adc_reading) {
        return (adc_reading >= Config::Safety::Potentiometer::CRITICAL_LOW && 
                adc_reading <= Config::Safety::Potentiometer::CRITICAL_HIGH);
    }
    
    void checkRudderBoundaries(int current_angle) {
        if (current_angle < Config::Calibration::Rudder::MIN_ANGLE) {
            while (readRudderAngle() < Config::Calibration::Rudder::MIN_ANGLE) {
                rudderActuator.setPWM(-HBridgeDriver::maxPWM, HBridgeDriver::M1);
            }
        } else if (current_angle > Config::Calibration::Rudder::MAX_ANGLE) {
            while (readRudderAngle() > Config::Calibration::Rudder::MAX_ANGLE) {
                rudderActuator.setPWM(HBridgeDriver::maxPWM, HBridgeDriver::M1);
            }
        }
    }
    
    bool isAngleWithinSafetyLimits(int angle, float min_limit, float max_limit) {
        return (angle >= min_limit && angle <= max_limit);
    }
    
    void capturePixhawkPulses() {
        for (int i = 0; i < Config::Buffers::PIXHAWK_CHANNELS; i++) {
            pixHawkReadingsPWM[i] = pulseIn(pixHawkReadingPins[i], HIGH);
        }
    }
    
    int16_t getPixhawkReading(uint8_t channel) {
        if (channel < Config::Buffers::PIXHAWK_CHANNELS) {
            return pixHawkReadingsPWM[channel];
        }
        return Config::Calibration::Pixhawk::TRIM_PWM; // Default safe value
    }
    
    int16_t convertPixhawkReadingToAngle(uint8_t channel) {
        int16_t angle = 0;
        switch (channel) {
            case 1: // rudder channel
                angle = map(pixHawkReadingsPWM[channel], 
                           Config::Calibration::Pixhawk::MIN_PWM, 
                           Config::Calibration::Pixhawk::MAX_PWM, 
                           Config::Calibration::Rudder::MIN_ANGLE, 
                           Config::Calibration::Rudder::MAX_ANGLE);
                break;
            case 0: // sail channel
                angle = map(pixHawkReadingsPWM[channel], 
                           Config::Calibration::Pixhawk::MIN_PWM, 
                           Config::Calibration::Pixhawk::MAX_PWM, 
                           Config::Calibration::Sail::MIN_ANGLE, 
                           Config::Calibration::Sail::MAX_ANGLE);
                break;
            default:
                Logger::logError("Invalid channel for angle conversion");
                // Return safe default instead of infinite loop
                return 0;
        }
        return angle;
    }
    
    bool isPixhawkSignalValid(int16_t pwm_reading) {
        return (pwm_reading >= Config::Calibration::Pixhawk::MIN_PWM && 
                pwm_reading <= Config::Calibration::Pixhawk::MAX_PWM);
    }
    
}