#pragma once
#include <Arduino.h>
#include "config.hpp"

namespace Actuators {
    
    // Function declarations for actuator management
    void initializeHardware();
    void initializePowerPins();
    void initializePotentiometers();
    void initializePixhawkPins();
    void initializeMotorDrivers();
    
    // Sensor reading functions
    int readRudderAngle();
    int readSailAngle();
    bool isRudderPotentiometerValid(int adc_reading);
    bool isSailPotentiometerValid(int adc_reading);
    
    // Safety boundary checking
    void checkRudderBoundaries(int current_angle);
    bool isAngleWithinSafetyLimits(int angle, float min_limit, float max_limit);
    
    // Pixhawk interface functions
    void capturePixhawkPulses();
    int16_t getPixhawkReading(uint8_t channel);
    int16_t convertPixhawkReadingToAngle(uint8_t channel);
    bool isPixhawkSignalValid(int16_t pwm_reading);
    
}