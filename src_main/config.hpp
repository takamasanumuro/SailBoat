#pragma once
#include <Arduino.h>

namespace Config {
    
    // System Information
    namespace System {
        constexpr const char* NAME = "Actuator/ADC interface for Pixhawk autopilot";
        constexpr uint32_t SERIAL_BAUD_RATE = 9600;
    }

    // Hardware Pin Definitions
    namespace Pins {
        // Rudder Actuator
        namespace Rudder {
            constexpr uint8_t DIRECTION_A = 43;
            constexpr uint8_t DIRECTION_B = 42;
            constexpr uint8_t PWM = 44;
        }

        // Throttle Motor
        namespace Throttle {
            constexpr uint8_t DIRECTION_A = 41;
            constexpr uint8_t DIRECTION_B = 40;
            constexpr uint8_t PWM = 45;
        }

        // Potentiometer Feedback Pins
        namespace Potentiometers {
            namespace Rudder {
                constexpr uint8_t GND = A8;
                constexpr uint8_t SIGNAL = A9;
                constexpr uint8_t POWER = A10;
            }
        }
    }

    // Actuator Calibration Constants
    namespace Calibration {
        // Rudder Configuration
        namespace Rudder {
            constexpr int16_t ANGLE_OFFSET = 0;
            constexpr int16_t ADC_MIN_THRESHOLD = 308;  // Starboard
            constexpr int16_t ADC_MAX_THRESHOLD = 661;  // Port
            constexpr float MIN_ANGLE = -47.69f;        // Starboard (degrees)
            constexpr float MAX_ANGLE = 53.64f;         // Port (degrees)
        }

        // Throttle Control
        namespace Throttle {
            constexpr int16_t MIN_PWM = 975;            // microseconds
            constexpr int16_t MAX_PWM = 2000;           // microseconds
            constexpr int16_t TRIM_PWM = 1500;          // neutral position
            constexpr int16_t DEAD_ZONE_PWM = 200;      // dead zone around trim
        }
    }

    // Control System Parameters
    namespace Control {
        // PID Constants
        namespace PID {
            namespace Rudder {
                constexpr float PROPORTIONAL_GAIN = 5.0f;
                constexpr float INTEGRAL_GAIN = 8.0f;
            }
        }
    }

    // Timing Constants
    namespace Timing {
        constexpr uint32_t PID_LOG_INTERVAL_MS = 1500;
        constexpr uint32_t THROTTLE_LOG_INTERVAL_MS = 1000;
        constexpr uint32_t RUDDER_READ_INTERVAL_MS = 1000;
    }
}