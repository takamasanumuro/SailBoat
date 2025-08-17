#pragma once
#include "mavlink/mavlink.h"

// MAVLink communication options
enum MAVLink_options {
    rudder_pwm = 0,
    rudder_angle = 1,
    sail_pwm = 2,
    sail_angle = 3,
};

// Pixhawk channel enumeration
enum pixHawkChannels {
    sail = 0, 
    rudder = 1, 
    throttle = 2,
    sailInputPWMPin = A11,
    rudderInputPWMPin = A12,
    throttleInputPWMPin = A13
};

// Type definitions
typedef uint32_t timer;

// Function declarations - only for functions that remain in main.cpp
void ParseActuatorCommands(const char* message);
void MAVLinkToPixhawk(MAVLink_options option, float data);
void GetSerialInput();
