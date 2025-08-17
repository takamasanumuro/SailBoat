#pragma once
#include <Arduino.h>

namespace Config {
    
    // System Information
    namespace System {
        constexpr const char* VERSION = "2.0.0";
        constexpr const char* NAME = "SailBoat Autopilot Main";
        constexpr uint32_t SERIAL_BAUD_RATE = 9600;
    }

    // Hardware Pin Definitions
    namespace Pins {
        // Winch Actuator (Sail Control)
        namespace Winch {
            constexpr uint8_t DIRECTION_A = 39;
            constexpr uint8_t DIRECTION_B = 38;
            constexpr uint8_t PWM = 46;
        }

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

        // Power Control Pins
        namespace Power {
            constexpr uint8_t PROPULSION_POWER[] = {52, 53};
            constexpr uint8_t SAIL_GND[] = {22, 23};
        }

        // Potentiometer Feedback Pins
        namespace Potentiometers {
            namespace Rudder {
                constexpr uint8_t GND = A8;
                constexpr uint8_t SIGNAL = A9;
                constexpr uint8_t POWER = A10;
            }
            
            namespace Sail {
                constexpr uint8_t SIGNAL = A7;
            }
        }

        // Pixhawk PWM Input Pins (from main.hpp)
        namespace PixhawkInput {
            constexpr uint8_t SAIL = A11;
            constexpr uint8_t RUDDER = A12;
            constexpr uint8_t THROTTLE = A13;
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

        // Sail Configuration
        namespace Sail {
            constexpr int16_t ANGLE_OFFSET = 0;
            constexpr int16_t ADC_MIN_THRESHOLD = 400;  // 2V = 90 degrees
            constexpr int16_t ADC_MAX_THRESHOLD = 800;  // 4V = 0 degrees
            constexpr int16_t MIN_ANGLE = 0;            // degrees
            constexpr int16_t MAX_ANGLE = 90;           // degrees
        }

        // Pixhawk PWM Signal Range
        namespace Pixhawk {
            constexpr uint16_t MIN_PWM = 993;           // microseconds
            constexpr uint16_t MAX_PWM = 1986;          // microseconds
            constexpr uint16_t TRIM_PWM = 1500;         // neutral position
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

            namespace Sail {
                constexpr float PROPORTIONAL_GAIN = 8.0f;
                constexpr float INTEGRAL_GAIN = 15.0f;
            }

            // Dead zones to prevent micro-adjustments
            namespace DeadZone {
                constexpr int16_t RUDDER = 2;           // +/- degrees
                constexpr int16_t SAIL = 3;             // +/- degrees
            }
        }
    }

    // Safety Limits
    namespace Safety {
        namespace Potentiometer {
            constexpr int16_t CRITICAL_LOW = 100;      // ADC reading
            constexpr int16_t CRITICAL_HIGH = 900;     // ADC reading
        }
    }

    // Communication Settings
    namespace Communication {
        namespace MAVLink {
            constexpr uint8_t ARDUINO_SYS_ID = 2;
            constexpr uint8_t ARDUINO_COMP_ID = 1;
            constexpr uint8_t PIXHAWK_SYS_ID = 1;
            constexpr uint8_t PIXHAWK_COMP_ID = 1;
            constexpr uint32_t PUBLISH_DELAY_MS = 1000; // 1 Hz
        }
    }

    // Timing Constants
    namespace Timing {
        constexpr uint32_t PID_LOG_INTERVAL_MS = 1500;
        constexpr uint32_t THROTTLE_LOG_INTERVAL_MS = 1000;
        constexpr uint32_t RUDDER_READ_INTERVAL_MS = 1000;
        constexpr uint32_t SAIL_READ_INTERVAL_MS = 3000;
        constexpr uint32_t HEARTBEAT_INTERVAL_MS = 5000;
    }

    // Buffer Sizes
    namespace Buffers {
        constexpr int SERIAL_INPUT_LENGTH = 256;
        constexpr uint8_t PIXHAWK_CHANNELS = 3;
    }
}