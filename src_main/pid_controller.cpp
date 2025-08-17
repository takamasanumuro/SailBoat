#include "pid_controller.hpp"
#include "actuators.hpp"
#include "HBridgeDriver.hpp"
#include "logger.hpp"

extern HBridgeDriver winchActuator;
extern HBridgeDriver rudderActuator;
extern HBridgeDriver throttleMotor;

namespace PIDController {
    
    // PID class implementation
    PID::PID(float p_gain, float i_gain, float d_gain) 
        : proportional_gain(p_gain), integral_gain(i_gain), derivative_gain(d_gain),
          integral_sum(0.0f), previous_error(0.0f), last_time(0),
          output_min(-HBridgeDriver::maxPWM), output_max(HBridgeDriver::maxPWM),
          initialized(false) {
    }
    
    void PID::setGains(float p_gain, float i_gain, float d_gain) {
        proportional_gain = p_gain;
        integral_gain = i_gain;
        derivative_gain = d_gain;
        Logger::debugPrint("PID gains updated");
    }
    
    void PID::setProportionalGain(float p_gain) {
        proportional_gain = p_gain;
    }
    
    void PID::setIntegralGain(float i_gain) {
        integral_gain = i_gain;
    }
    
    void PID::setDerivativeGain(float d_gain) {
        derivative_gain = d_gain;
    }
    
    void PID::setOutputLimits(float min_output, float max_output) {
        output_min = min_output;
        output_max = max_output;
        
        // Constrain integral sum to new limits
        integral_sum = constrain(integral_sum, output_min, output_max);
    }
    
    float PID::calculate(float setpoint, float current_value) {
        float error = setpoint - current_value;
        return calculate(error);
    }
    
    float PID::calculate(float error) {
        uint32_t current_time = TimingManager::getTimeMS();
        
        if (!initialized) {
            last_time = current_time;
            previous_error = error;
            initialized = true;
            return 0.0f;  // Return zero on first call
        }
        
        // Calculate time delta in seconds
        float dt = static_cast<float>(TimingManager::getTimeDifference(last_time, current_time)) / 1000.0f;
        
        // Avoid division by zero and handle very small time steps
        if (dt <= 0.001f) {
            return 0.0f;
        }
        
        // Proportional term
        float proportional = proportional_gain * error;
        
        // Integral term with windup protection
        integral_sum += integral_gain * error * dt;
        integral_sum = constrain(integral_sum, output_min, output_max);
        
        // Derivative term
        float derivative = 0.0f;
        if (dt > 0) {
            derivative = derivative_gain * (error - previous_error) / dt;
        }
        
        // Calculate total output
        float output = proportional + integral_sum + derivative;
        output = constrain(output, output_min, output_max);
        
        // Update state for next iteration
        previous_error = error;
        last_time = current_time;
        
        return output;
    }
    
    void PID::reset() {
        integral_sum = 0.0f;
        previous_error = 0.0f;
        last_time = TimingManager::getTimeMS();
        initialized = false;
        Logger::debugPrint("PID controller reset");
    }
    
    // System PID Controllers
    namespace Controllers {
        static PID rudder_pid;
        static PID sail_pid;
        static PID throttle_pid;
        static bool controllers_initialized = false;
        
        PID& getRudderPID() {
            return rudder_pid;
        }
        
        PID& getSailPID() {
            return sail_pid;
        }
        
        PID& getThrottlePID() {
            return throttle_pid;
        }
        
        void initializeControllers() {
            // Initialize rudder PID controller
            rudder_pid.setGains(
                Config::Control::PID::Rudder::PROPORTIONAL_GAIN,
                Config::Control::PID::Rudder::INTEGRAL_GAIN,
                0.0f  // No derivative term configured
            );
            rudder_pid.setOutputLimits(-HBridgeDriver::maxPWM, HBridgeDriver::maxPWM);
            
            // Initialize sail PID controller  
            sail_pid.setGains(
                Config::Control::PID::Sail::PROPORTIONAL_GAIN,
                Config::Control::PID::Sail::INTEGRAL_GAIN,
                0.0f  // No derivative term configured
            );
            sail_pid.setOutputLimits(-HBridgeDriver::maxPWM, HBridgeDriver::maxPWM);
            
            // Initialize throttle PID (for future use)
            throttle_pid.setGains(1.0f, 0.1f, 0.0f);
            throttle_pid.setOutputLimits(-HBridgeDriver::maxPWM, HBridgeDriver::maxPWM);
            
            controllers_initialized = true;
            Logger::log(Logger::INFO, "PID controllers initialized");
        }
        
        void resetAllControllers() {
            rudder_pid.reset();
            sail_pid.reset();
            throttle_pid.reset();
            Logger::log(Logger::INFO, "All PID controllers reset");
        }
    }
    
    // Enhanced control functions
    void controlRudder(int desired_angle) {
        int current_angle = Actuators::readRudderAngle();
        Actuators::checkRudderBoundaries(current_angle);
        
        // Calculate PID output
        float pid_output = Controllers::getRudderPID().calculate(desired_angle, current_angle);
        
        // Apply sign flip and dead zone
        int output_pwm = static_cast<int>(-pid_output);  // Flip sign for actuator direction
        int error = desired_angle - current_angle;
        
        if (isErrorWithinDeadZone(error, Config::Control::PID::DeadZone::RUDDER)) {
            output_pwm = 0;
        }
        
        // Log PID debug information
        if (TimingManager::getPIDLogTimer().isReady()) {
            Logger::logPIDOutput("Rudder", desired_angle, current_angle, error, output_pwm);
        }
        
        rudderActuator.setPWM(output_pwm, HBridgeDriver::M1);
    }
    
    void controlSail(int desired_angle) {
        int sail_adc_reading = analogRead(Config::Pins::Potentiometers::Sail::SIGNAL);
        
        if (!Actuators::isSailPotentiometerValid(sail_adc_reading)) {
            Logger::logSafetyEvent("Sail Potentiometer critical!");
            winchActuator.setPWM(0, HBridgeDriver::M1);
            return;
        }
        
        int current_angle = map(sail_adc_reading, 
                               Config::Calibration::Sail::ADC_MIN_THRESHOLD, 
                               Config::Calibration::Sail::ADC_MAX_THRESHOLD, 
                               Config::Calibration::Sail::MIN_ANGLE, 
                               Config::Calibration::Sail::MAX_ANGLE);
        
        // Calculate PID output
        float pid_output = Controllers::getSailPID().calculate(desired_angle, current_angle);
        
        // Apply sign flip and dead zone
        int output_pwm = static_cast<int>(-pid_output);  // Flip sign for actuator direction
        int error = desired_angle - current_angle;
        
        if (isErrorWithinDeadZone(error, Config::Control::PID::DeadZone::SAIL)) {
            output_pwm = 0;
        }
        
        // Log PID debug information
        if (TimingManager::getPIDLogTimer().isReady()) {
            Logger::logPIDOutput("Sail", desired_angle, current_angle, error, output_pwm);
        }
        
        winchActuator.setPWM(output_pwm, HBridgeDriver::M1);
    }
    
    void controlThrottle(int16_t pixhawk_pwm) {
        static int16_t previous_valid_signal = Config::Calibration::Pixhawk::TRIM_PWM;
        
        if (!Actuators::isPixhawkSignalValid(pixhawk_pwm)) {
            pixhawk_pwm = previous_valid_signal;
        }
        
        // Choose control method (currently using BLDC passthrough)
        controlThrottleBLDC(pixhawk_pwm);
        
        if (TimingManager::getThrottleLogTimer().isReady()) {
            Logger::logThrottleControl(pixhawk_pwm);
        }
        
        previous_valid_signal = pixhawk_pwm;
    }
    
    void controlThrottleHBridge(int16_t pixhawk_pwm, int16_t min_pwm, int16_t max_pwm, int16_t trim_pwm, int16_t dead_zone) {
        int h_bridge_pwm;
        
        if (pixhawk_pwm > trim_pwm + dead_zone) {
            h_bridge_pwm = map(pixhawk_pwm, trim_pwm + dead_zone, max_pwm, 0, HBridgeDriver::maxPWM);
            throttleMotor.setPWM(h_bridge_pwm, HBridgeDriver::M1);
        } else if (pixhawk_pwm < (trim_pwm - dead_zone)) {
            h_bridge_pwm = map(pixhawk_pwm, min_pwm, trim_pwm - dead_zone, HBridgeDriver::maxPWM, 0);
            throttleMotor.setPWM(-h_bridge_pwm, HBridgeDriver::M1);
        } else {
            throttleMotor.setPWM(0, HBridgeDriver::M1);
        }
    }
    
    void controlThrottleBLDC(int16_t pixhawk_pwm) {
        // BLDC servo passthrough - would use Servo library if enabled
        // bldcThrottleServo.writeMicroseconds(pixhawk_pwm);
        
        // For now, map to H-bridge as fallback
        int mapped_pwm = map(pixhawk_pwm, 
                           Config::Calibration::Throttle::MIN_PWM, 
                           Config::Calibration::Throttle::MAX_PWM, 
                           -HBridgeDriver::maxPWM, 
                           HBridgeDriver::maxPWM);
        throttleMotor.setPWM(mapped_pwm, HBridgeDriver::M1);
    }
    
    bool isErrorWithinDeadZone(int error, int dead_zone) {
        return (error > -dead_zone && error < dead_zone);
    }
    
    int constrainPWMOutput(int pwm, int max_pwm) {
        return constrain(pwm, -max_pwm, max_pwm);
    }    
}