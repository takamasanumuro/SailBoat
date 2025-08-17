#pragma once
#include <Arduino.h>
#include "config.hpp"
#include "timing_manager.hpp"

namespace PIDController {
    
    // Individual PID controller class for better state management
    class PID {
    private:
        float proportional_gain;
        float integral_gain;
        float derivative_gain;
        
        float integral_sum;
        float previous_error;
        uint32_t last_time;
        
        float output_min;
        float output_max;
        bool initialized;
        
    public:
        PID(float p_gain = 1.0f, float i_gain = 0.0f, float d_gain = 0.0f);
        
        // Set PID gains
        void setGains(float p_gain, float i_gain, float d_gain = 0.0f);
        void setProportionalGain(float p_gain);
        void setIntegralGain(float i_gain);
        void setDerivativeGain(float d_gain);
        
        // Set output limits
        void setOutputLimits(float min_output, float max_output);
        
        // Calculate PID output
        float calculate(float setpoint, float current_value);
        float calculate(float error);  // Direct error input
        
        // Reset controller state
        void reset();
        
        // Get current gains
        float getProportionalGain() const { return proportional_gain; }
        float getIntegralGain() const { return integral_gain; }
        float getDerivativeGain() const { return derivative_gain; }
        
        // Get internal state (for debugging)
        float getIntegralSum() const { return integral_sum; }
        float getPreviousError() const { return previous_error; }
        
        // Check if controller is initialized
        bool isInitialized() const { return initialized; }
    };
    
    // System PID controllers
    namespace Controllers {
        // Get singleton instances of PID controllers
        PID& getRudderPID();
        PID& getSailPID();
        PID& getThrottlePID();  // For future use
        
        // Initialize all PID controllers with config values
        void initializeControllers();
        
        // Reset all controllers
        void resetAllControllers();
    }
    
    // Enhanced control functions using new PID controllers
    void controlRudder(int desired_angle);
    void controlSail(int desired_angle);
    void controlThrottle(int16_t pixhawk_pwm);
    
    // Throttle control variants (unchanged)
    void controlThrottleHBridge(int16_t pixhawk_pwm, int16_t min_pwm, int16_t max_pwm, int16_t trim_pwm, int16_t dead_zone);
    void controlThrottleBLDC(int16_t pixhawk_pwm);
    
    // Utility functions
    bool isErrorWithinDeadZone(int error, int dead_zone);
    int constrainPWMOutput(int pwm, int max_pwm);
}