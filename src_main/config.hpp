#pragma once
#include <Arduino.h>

namespace Config {
    
    // System Information
    namespace System {
        constexpr const char* NAME = "Actuator/ADC interface for Pixhawk autopilot";
        constexpr uint32_t SERIAL_BAUD_RATE = 115200;
    }

    // Hardware Pin Definitions
    namespace Pins {
        // Rudder Actuator
        namespace Rudder {
            constexpr uint8_t DIRECTION_A = 43;
            constexpr uint8_t DIRECTION_B = 42;
            constexpr uint8_t PWM = 44;
            // Integrated system rudder pins
            constexpr uint8_t INA_PIN = 8;
            constexpr uint8_t INB_PIN = 9;
            constexpr uint8_t PWM_PIN = 10;
        }

        // Throttle Motor
        namespace Throttle {
            constexpr uint8_t DIRECTION_A = 41;
            constexpr uint8_t DIRECTION_B = 40;
            constexpr uint8_t PWM = 45;
        }

        // Motor Voltage Generator
        namespace MotorVoltage {
            constexpr uint8_t PWM_PIN = 2;
            constexpr uint8_t GND_PIN = A1;
            constexpr uint8_t SIGNAL_PIN = A0;
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
                // Modern PID controller settings
                constexpr double DEFAULT_KP = 0.2;
                constexpr double DEFAULT_KI = 0.01;
                constexpr double DEFAULT_KD = 0.05;
            }
        }

        // H-Bridge Configuration
        namespace HBridge {
            namespace Rudder {
                constexpr int16_t MAX_PWM = 240;
                constexpr int16_t MIN_PWM = 60;  // 25% of MAX_PWM for deadband
            }
        }

        // Motor Voltage Generator Configuration
        namespace MotorVoltage {
            constexpr float MIN_VOLTAGE_MV = 800.0f;   // 0.8V
            constexpr float MAX_VOLTAGE_MV = 4000.0f;  // 4.0V
            constexpr uint16_t PWM_FREQUENCY_HZ = 490;
            constexpr uint8_t PWM_RESOLUTION_BITS = 8;
        }
    }

    // Timing Constants
    namespace Timing {
        constexpr uint32_t PID_LOG_INTERVAL_MS = 1500;
        constexpr uint32_t THROTTLE_LOG_INTERVAL_MS = 1000;
        constexpr uint32_t RUDDER_READ_INTERVAL_MS = 1000;
        constexpr uint32_t ANALOG_PRINT_INTERVAL_MS = 1000;
    }

    // Rudder Angle Control Constants
    namespace RudderAngle {
        constexpr int ADC_MIN = 285;
        constexpr int ADC_MAX = 611;
        constexpr float ANGLE_MIN = -45.0f;
        constexpr float ANGLE_MAX = 45.0f;
        
        // PID output limits
        constexpr int PID_OUTPUT_MIN = -100;
        constexpr int PID_OUTPUT_MAX = 100;
        constexpr int MAX_INTEGRAL_CUMULATION = 100;
        constexpr float MOTOR_POWER_LIMIT = 80.0f;  // Don't use full 100% power
        constexpr float DEAD_ZONE_THRESHOLD = 1.0f; // Prevent jitter
    }
}