#pragma once
#include <Arduino.h>
#include "config.hpp"
#include "HBridgeDriver.hpp"

extern "C" {
#include "c/PID.h"
}

namespace RudderControl {
    
    struct AngleConfig {
        int adc_min = Config::RudderAngle::ADC_MIN;
        int adc_max = Config::RudderAngle::ADC_MAX;
        float angle_min = Config::RudderAngle::ANGLE_MIN;
        float angle_max = Config::RudderAngle::ANGLE_MAX;
    };
    
    class RudderController {
    private:
        HBridgeDriverV2 hbridge;
        AngleConfig angle_config;
        PIDController* pid_controller;
        bool pid_enabled;
        float target_angle;
        float current_angle;
        
        // PID callback functions (static to be used as C callbacks)
        static int pidSourceCallback();
        static void pidOutputCallback(int output);
        static unsigned long getTimeCallback();
        
        // Static instance pointer for callbacks
        static RudderController* instance;
        
    public:
        RudderController();
        ~RudderController();
        
        // Initialization
        bool init();
        
        // Manual control
        bool setPercentage(float percentage);
        bool stop();
        
        // Angle control
        float adcToAngle(int adc_value) const;
        int angleToAdc(float angle) const;
        float getCurrentAngle();
        void setAngleTarget(float angle);
        float getTargetAngle() const { return target_angle; }
        
        // PID control
        void initPID(double kp = Config::Control::PID::Rudder::DEFAULT_KP,
                     double ki = Config::Control::PID::Rudder::DEFAULT_KI,
                     double kd = Config::Control::PID::Rudder::DEFAULT_KD);
        void enablePID(bool enable);
        bool isPIDEnabled() const { return pid_enabled; }
        void updatePID();
        
        // PID tuning
        void setPIDTunings(double kp, double ki, double kd);
        void getPIDTunings(double& kp, double& ki, double& kd) const;
        
        // Configuration
        void setAngleMapping(int adc_min, int adc_max, float angle_min, float angle_max);
        
        // Status
        float getCurrentPercentage() const;
        int getCurrentPWM() const;
        HBridgeDriverV2::ErrorCode getLastError() const;
        const char* getErrorString(HBridgeDriverV2::ErrorCode error) const;
    };
    
    // Global instance
    extern RudderController controller;
}