#include "rudder_control.hpp"

namespace RudderControl {
    
    // Static member initialization
    RudderController* RudderController::instance = nullptr;
    
    // Global instance
    RudderController controller;
    
    RudderController::RudderController() 
        : pid_controller(nullptr), pid_enabled(false), target_angle(0.0f), current_angle(0.0f) {
        instance = this;  // Set static instance for callbacks
    }
    
    RudderController::~RudderController() {
        if (pid_controller != nullptr) {
            free(pid_controller);
        }
    }
    
    bool RudderController::init() {
        // Configure H-bridge
        HBridgeDriverV2::Config rudder_cfg;
        rudder_cfg.pwm_pin = Config::Pins::Rudder::PWM_PIN;
        rudder_cfg.inb_pin = Config::Pins::Rudder::INB_PIN;
        rudder_cfg.ina_pin = Config::Pins::Rudder::INA_PIN;
        rudder_cfg.max_pwm = Config::Control::HBridge::Rudder::MAX_PWM;
        rudder_cfg.min_pwm = Config::Control::HBridge::Rudder::MIN_PWM;
        
        auto config_result = hbridge.setChannelConfig(HBridgeDriverV2::Channel::M1, rudder_cfg);
        if (config_result != HBridgeDriverV2::ErrorCode::NONE) {
            Serial.print("Rudder configuration failed: ");
            Serial.println(hbridge.getErrorString(config_result));
            return false;
        }
        
        auto init_result = hbridge.init();
        if (init_result != HBridgeDriverV2::ErrorCode::NONE) {
            Serial.print("Rudder initialization failed: ");
            Serial.println(hbridge.getErrorString(init_result));
            return false;
        }
        
        // Configure feedback pins
        pinMode(Config::Pins::Potentiometers::Rudder::GND, OUTPUT);
        digitalWrite(Config::Pins::Potentiometers::Rudder::GND, LOW);
        pinMode(Config::Pins::Potentiometers::Rudder::POWER, OUTPUT);
        digitalWrite(Config::Pins::Potentiometers::Rudder::POWER, HIGH);
        pinMode(Config::Pins::Potentiometers::Rudder::SIGNAL, INPUT);
        
        Serial.println("Rudder controller initialized");
        return true;
    }
    
    bool RudderController::setPercentage(float percentage) {
        auto result = hbridge.setPercentage(percentage, HBridgeDriverV2::Channel::M1);
        if (result != HBridgeDriverV2::ErrorCode::NONE) {
            Serial.print("Failed to set rudder percentage: ");
            Serial.println(hbridge.getErrorString(result));
            return false;
        }
        return true;
    }
    
    bool RudderController::stop() {
        auto result = hbridge.stop(HBridgeDriverV2::Channel::M1);
        return (result == HBridgeDriverV2::ErrorCode::NONE);
    }
    
    float RudderController::adcToAngle(int adc_value) const {
        if (angle_config.adc_max == angle_config.adc_min) return 0.0f;
        
        float normalized = (float)(adc_value - angle_config.adc_min) / 
                          (angle_config.adc_max - angle_config.adc_min);
        normalized = constrain(normalized, 0.0f, 1.0f);
        
        return angle_config.angle_min + normalized * (angle_config.angle_max - angle_config.angle_min);
    }
    
    int RudderController::angleToAdc(float angle) const {
        if (angle_config.angle_max == angle_config.angle_min) return angle_config.adc_min;
        
        float normalized = (angle - angle_config.angle_min) / 
                          (angle_config.angle_max - angle_config.angle_min);
        normalized = constrain(normalized, 0.0f, 1.0f);
        
        return angle_config.adc_min + (int)(normalized * (angle_config.adc_max - angle_config.adc_min));
    }
    
    float RudderController::getCurrentAngle() {
        int adc_value = analogRead(Config::Pins::Potentiometers::Rudder::SIGNAL);
        current_angle = adcToAngle(adc_value);
        return current_angle;
    }
    
    // Static PID callback functions
    int RudderController::pidSourceCallback() {
        if (instance == nullptr) return 0;
        // Return current angle, scaled by 10 for integer precision
        return (int)(instance->getCurrentAngle() * 10.0f);
    }
    
    void RudderController::pidOutputCallback(int output) {
        if (instance == nullptr) return;
        
        // PID output is the desired motor percentage (-100 to 100)
        float percentage = (float)output;
        
        // Apply reasonable limits
        percentage = constrain(percentage, -Config::RudderAngle::MOTOR_POWER_LIMIT, 
                               Config::RudderAngle::MOTOR_POWER_LIMIT);
        
        // DEBUG: Print PID output values every 2 seconds (0.5Hz)
        static unsigned long last_debug_time = 0;
        unsigned long now = millis();
        if (now - last_debug_time >= 500) {
            last_debug_time = now;
            int current_angle_scaled = pidSourceCallback();
            int error = instance->pid_controller->target - current_angle_scaled;
            Serial.print("PID Raw Output: "); Serial.print(output);
            Serial.print(", Error (deg*10): "); Serial.print(error);
            Serial.print(", Motor %: "); Serial.print(percentage, 1);
            Serial.print(", Current Angle: "); Serial.print((float)current_angle_scaled / 10.0f, 1);
            Serial.print(", Target Angle: "); Serial.println((float)instance->pid_controller->target / 10.0f, 1);
            // Show PID components for debugging
            if (instance->pid_controller != nullptr) {
                Serial.print("P="); Serial.print(instance->pid_controller->p * error, 2);
                Serial.print(", I="); Serial.print(instance->pid_controller->i * instance->pid_controller->integralCumulation, 2);
                Serial.print(", D="); Serial.println(instance->pid_controller->d * instance->pid_controller->cycleDerivative, 2);
            }
        }
        
        // Dead zone to prevent jitter
        if (abs(percentage) > Config::RudderAngle::DEAD_ZONE_THRESHOLD) {
            // FLIP DIRECTION: PID output may be backwards depending on tuning/wiring
            instance->hbridge.setPercentage(-percentage, HBridgeDriverV2::Channel::M1);
        } else {
            instance->hbridge.stop(HBridgeDriverV2::Channel::M1);
        }
    }
    
    unsigned long RudderController::getTimeCallback() {
        return millis() / 1000;
    }
    
    void RudderController::initPID(double kp, double ki, double kd) {
        if (pid_controller != nullptr) {
            free(pid_controller);
        }
        
        pid_controller = createPIDController(kp, ki, kd, pidSourceCallback, pidOutputCallback);
        if (pid_controller != nullptr) {
            registerTimeFunction(pid_controller, getTimeCallback);
            setOutputBounds(pid_controller, Config::RudderAngle::PID_OUTPUT_MIN, 
                            Config::RudderAngle::PID_OUTPUT_MAX);
            setMaxIntegralCumulation(pid_controller, Config::RudderAngle::MAX_INTEGRAL_CUMULATION);
            setEnabled(pid_controller, 0);
            pid_enabled = false;
            Serial.print("PID controller initialized with gains: P="); Serial.print(kp, 3);
            Serial.print(", I="); Serial.print(ki, 3);
            Serial.print(", D="); Serial.println(kd, 3);
        } else {
            Serial.println("Failed to initialize PID controller");
        }
    }
    
    void RudderController::setAngleTarget(float angle) {
        if (pid_controller == nullptr) return;
        
        target_angle = constrain(angle, angle_config.angle_min, angle_config.angle_max);
        pid_controller->target = (int)(target_angle * 10.0f);
        
        static unsigned long last_print_time = 0;
        if (millis() - last_print_time > 1000) {  // Print every second
            last_print_time = millis();
            Serial.print("Set angle target: "); Serial.print(target_angle, 1);
            Serial.print("° -> Scaled Target: "); Serial.println(pid_controller->target);
        }
    }
    
    void RudderController::enablePID(bool enable) {
        if (pid_controller == nullptr) return;
        if (pid_controller->enabled) return;

        pid_enabled = enable;
        setEnabled(pid_controller, enable ? 1 : 0);
        
        if (enable) {
            // Auto-set target to center when PID is enabled
            setAngleTarget(0.0f);
            Serial.println("PID enabled - auto-targeting center (0°)");
        } else {
            // Stop motor when PID is disabled
            hbridge.stop(HBridgeDriverV2::Channel::M1);
            Serial.println("PID disabled - motor stopped");
        }
    }
    
    void RudderController::updatePID() {
        if (pid_controller == nullptr || !pid_enabled) return;
        tick(pid_controller);
    }
    
    void RudderController::setPIDTunings(double kp, double ki, double kd) {
        if (pid_controller != nullptr) {
            pid_controller->p = kp;
            pid_controller->i = ki;
            pid_controller->d = kd;
            Serial.print("PID gains updated: P="); Serial.print(kp, 3);
            Serial.print(", I="); Serial.print(ki, 3);
            Serial.print(", D="); Serial.println(kd, 3);
        }
    }
    
    void RudderController::getPIDTunings(double& kp, double& ki, double& kd) const {
        if (pid_controller != nullptr) {
            kp = pid_controller->p;
            ki = pid_controller->i;
            kd = pid_controller->d;
        } else {
            kp = ki = kd = 0.0;
        }
    }
    
    void RudderController::setAngleMapping(int adc_min, int adc_max, float angle_min, float angle_max) {
        angle_config.adc_min = adc_min;
        angle_config.adc_max = adc_max;
        angle_config.angle_min = angle_min;
        angle_config.angle_max = angle_max;
        Serial.print("Angle mapping updated: ADC["); Serial.print(adc_min);
        Serial.print(","); Serial.print(adc_max);
        Serial.print("] -> Angle["); Serial.print(angle_min, 1);
        Serial.print(","); Serial.print(angle_max, 1); Serial.println("]");
    }
    
    float RudderController::getCurrentPercentage() const {
        return hbridge.getCurrentPercentage(HBridgeDriverV2::Channel::M1);
    }
    
    int RudderController::getCurrentPWM() const {
        return hbridge.getCurrentPWM(HBridgeDriverV2::Channel::M1);
    }
    
    HBridgeDriverV2::ErrorCode RudderController::getLastError() const {
        return hbridge.getLastError();
    }
    
    const char* RudderController::getErrorString(HBridgeDriverV2::ErrorCode error) const {
        return hbridge.getErrorString(error);
    }
}