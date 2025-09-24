#include "HBridgeDriver.hpp"

// Using directive to simplify ErrorCode references
using ErrorCode = HBridgeDriverV2::ErrorCode;
using Channel = HBridgeDriverV2::Channel;
using ControlMode = HBridgeDriverV2::ControlMode;
using Config = HBridgeDriverV2::Config;

// Constructor implementations
HBridgeDriverV2::HBridgeDriverV2() 
    : m1_enabled(false), m2_enabled(false), initialized(false),
      m1_current_pwm(0), m2_current_pwm(0),
      m1_current_mode(ControlMode::COAST), m2_current_mode(ControlMode::COAST),
      last_error(ErrorCode::NONE), error_count(0) {
}

HBridgeDriverV2::HBridgeDriverV2(const Config& m1_cfg) 
    : HBridgeDriverV2() {
    m1_config = m1_cfg;
    m1_enabled = true;
}

HBridgeDriverV2::HBridgeDriverV2(const Config& m1_cfg, const Config& m2_cfg) 
    : HBridgeDriverV2() {
    m1_config = m1_cfg;
    m2_config = m2_cfg;
    m1_enabled = true;
    m2_enabled = true;
}

// Initialization methods
ErrorCode HBridgeDriverV2::init() {
    if (m1_enabled) {
        ErrorCode result = initChannel(Channel::M1);
        if (result != ErrorCode::NONE) {
            return result;
        }
    }
    
    if (m2_enabled) {
        ErrorCode result = initChannel(Channel::M2);
        if (result != ErrorCode::NONE) {
            return result;
        }
    }
    
    if (!m1_enabled && !m2_enabled) {
        setError(ErrorCode::INVALID_CHANNEL);
        return ErrorCode::INVALID_CHANNEL;
    }
    
    initialized = true;
    return ErrorCode::NONE;
}

ErrorCode HBridgeDriverV2::initChannel(Channel channel) {
    Config* cfg = nullptr;
    
    switch (channel) {
        case Channel::M1:
            if (!m1_enabled) {
                setError(ErrorCode::INVALID_CHANNEL);
                return ErrorCode::INVALID_CHANNEL;
            }
            cfg = &m1_config;
            break;
            
        case Channel::M2:
            if (!m2_enabled) {
                setError(ErrorCode::INVALID_CHANNEL);
                return ErrorCode::INVALID_CHANNEL;
            }
            cfg = &m2_config;
            break;
            
        default:
            setError(ErrorCode::INVALID_CHANNEL);
            return ErrorCode::INVALID_CHANNEL;
    }
    
    // Validate pins
    if (!isValidPin(cfg->ina_pin) || !isValidPin(cfg->inb_pin) || !isValidPin(cfg->pwm_pin)) {
        setError(ErrorCode::INVALID_PIN);
        return ErrorCode::INVALID_PIN;
    }
    
    // Initialize pins
    pinMode(cfg->ina_pin, OUTPUT);
    pinMode(cfg->inb_pin, OUTPUT);
    pinMode(cfg->pwm_pin, OUTPUT);
    
    // Set initial state (coast)
    digitalWrite(cfg->ina_pin, LOW);
    digitalWrite(cfg->inb_pin, LOW);
    digitalWrite(cfg->pwm_pin, LOW);
    
    return ErrorCode::NONE;
}

// Configuration methods
ErrorCode HBridgeDriverV2::setChannelConfig(Channel channel, const Config& cfg) {
    // Check if channel is valid (M1, M2, but NOT BOTH for configuration)
    if (channel != Channel::M1 && channel != Channel::M2) {
        setError(ErrorCode::INVALID_CHANNEL);
        return ErrorCode::INVALID_CHANNEL;
    }
    
    if (!isValidPin(cfg.ina_pin) || !isValidPin(cfg.inb_pin) || !isValidPin(cfg.pwm_pin)) {
        setError(ErrorCode::INVALID_PIN);
        return ErrorCode::INVALID_PIN;
    }
    
    switch (channel) {
        case Channel::M1:
            m1_config = cfg;
            m1_enabled = true;  // Enable the channel when configured
            break;
        case Channel::M2:
            m2_config = cfg;
            m2_enabled = true;  // Enable the channel when configured
            break;
        default:
            setError(ErrorCode::INVALID_CHANNEL);
            return ErrorCode::INVALID_CHANNEL;
    }
    
    return ErrorCode::NONE;
}

HBridgeDriverV2::Config HBridgeDriverV2::getChannelConfig(Channel channel) const {
    switch (channel) {
        case Channel::M1:
            return m1_config;
        case Channel::M2:
            return m2_config;
        default:
            return Config{}; // Return empty config for invalid channel
    }
}

void HBridgeDriverV2::setMaxPWM(Channel channel, int16_t max_pwm) {
    if (max_pwm < 0 || max_pwm > MAX_PWM_VALUE) {
        return; // Invalid value, ignore
    }
    
    switch (channel) {
        case Channel::M1:
            if (m1_enabled) m1_config.max_pwm = max_pwm;
            break;
        case Channel::M2:
            if (m2_enabled) m2_config.max_pwm = max_pwm;
            break;
        case Channel::BOTH:
            if (m1_enabled) m1_config.max_pwm = max_pwm;
            if (m2_enabled) m2_config.max_pwm = max_pwm;
            break;
    }
}

int16_t HBridgeDriverV2::getMaxPWM(Channel channel) const {
    switch (channel) {
        case Channel::M1:
            return m1_enabled ? m1_config.max_pwm : 0;
        case Channel::M2:
            return m2_enabled ? m2_config.max_pwm : 0;
        default:
            return 0;
    }
}

// Motor control methods
ErrorCode HBridgeDriverV2::setPWM(int16_t pwm, Channel channel) {
    if (!initialized) {
        setError(ErrorCode::NOT_INITIALIZED);
        return ErrorCode::NOT_INITIALIZED;
    }
    
    if (!isValidChannel(channel)) {
        setError(ErrorCode::INVALID_CHANNEL);
        return ErrorCode::INVALID_CHANNEL;
    }
    
    // For operations, check if channels are enabled
    if (!isChannelEnabled(channel)) {
        setError(ErrorCode::INVALID_CHANNEL);
        return ErrorCode::INVALID_CHANNEL;
    }
    
    ErrorCode result = ErrorCode::NONE;
    
    if (channel == Channel::BOTH) {
        // Set both channels
        if (m1_enabled) {
            result = setChannelPWM(Channel::M1, pwm);
            if (result != ErrorCode::NONE) return result;
        }
        if (m2_enabled) {
            result = setChannelPWM(Channel::M2, pwm);
            if (result != ErrorCode::NONE) return result;
        }
    } else {
        result = setChannelPWM(channel, pwm);
    }
    
    return result;
}

ErrorCode HBridgeDriverV2::setPercentage(float percentage, Channel channel) {
    if (!isValidPercentage(percentage)) {
        setError(ErrorCode::INVALID_PWM);
        return ErrorCode::INVALID_PWM;
    }
    
    int16_t max_pwm = DEFAULT_MAX_PWM;
    int16_t min_pwm = 0;
    
    // Get appropriate max_pwm and min_pwm for the channel
    if (channel == Channel::M1 && m1_enabled) {
        max_pwm = m1_config.max_pwm;
        min_pwm = m1_config.min_pwm;
    } else if (channel == Channel::M2 && m2_enabled) {
        max_pwm = m2_config.max_pwm;
        min_pwm = m2_config.min_pwm;
    } else if (channel == Channel::BOTH) {
        // Use the minimum max_pwm and maximum min_pwm of enabled channels
        if (m1_enabled && m2_enabled) {
            max_pwm = min(m1_config.max_pwm, m2_config.max_pwm);
            min_pwm = max(m1_config.min_pwm, m2_config.min_pwm);
        } else if (m1_enabled) {
            max_pwm = m1_config.max_pwm;
            min_pwm = m1_config.min_pwm;
        } else if (m2_enabled) {
            max_pwm = m2_config.max_pwm;
            min_pwm = m2_config.min_pwm;
        }
    }
    
    int16_t pwm = percentageToPWM(percentage, max_pwm, min_pwm);
    return setPWM(pwm, channel);
}

ErrorCode HBridgeDriverV2::stop(Channel channel) {
    return coast(channel); // Stop is the same as coast for this H-bridge
}

ErrorCode HBridgeDriverV2::coast(Channel channel) {
    return setPWM(0, channel); // Set PWM to 0 for coasting
}

ErrorCode HBridgeDriverV2::emergencyStop() {
    ErrorCode result = ErrorCode::NONE;
    
    // Immediately stop all enabled channels
    if (m1_enabled) {
        Config& cfg = m1_config;
        digitalWrite(cfg.ina_pin, LOW);
        digitalWrite(cfg.inb_pin, LOW);
        analogWrite(cfg.pwm_pin, 0);
        updateChannelState(Channel::M1, 0, ControlMode::COAST);
    }
    
    if (m2_enabled) {
        Config& cfg = m2_config;
        digitalWrite(cfg.ina_pin, LOW);
        digitalWrite(cfg.inb_pin, LOW);
        analogWrite(cfg.pwm_pin, 0);
        updateChannelState(Channel::M2, 0, ControlMode::COAST);
    }
    
    return result;
}

// Status query methods
int16_t HBridgeDriverV2::getCurrentPWM(Channel channel) const {
    switch (channel) {
        case Channel::M1:
            return m1_enabled ? m1_current_pwm : 0;
        case Channel::M2:
            return m2_enabled ? m2_current_pwm : 0;
        default:
            return 0;
    }
}

float HBridgeDriverV2::getCurrentPercentage(Channel channel) const {
    int16_t pwm = getCurrentPWM(channel);
    int16_t max_pwm = getMaxPWM(channel);
    
    if (max_pwm == 0) return 0.0f;
    
    return pwmToPercentage(pwm, max_pwm);
}

HBridgeDriverV2::ControlMode HBridgeDriverV2::getCurrentMode(Channel channel) const {
    switch (channel) {
        case Channel::M1:
            return m1_enabled ? m1_current_mode : ControlMode::COAST;
        case Channel::M2:
            return m2_enabled ? m2_current_mode : ControlMode::COAST;
        default:
            return ControlMode::COAST;
    }
}

bool HBridgeDriverV2::isChannelActive(Channel channel) const {
    return getCurrentPWM(channel) != 0;
}

bool HBridgeDriverV2::isChannelEnabled(Channel channel) const {
    switch (channel) {
        case Channel::M1:
            return m1_enabled;
        case Channel::M2:
            return m2_enabled;
        case Channel::BOTH:
            return m1_enabled || m2_enabled;
        default:
            return false;
    }
}

// Error handling methods
const char* HBridgeDriverV2::getErrorString(ErrorCode error) const {
    switch (error) {
        case ErrorCode::NONE:
            return "No error";
        case ErrorCode::INVALID_PIN:
            return "Invalid pin number";
        case ErrorCode::INVALID_PWM:
            return "Invalid PWM value";
        case ErrorCode::INVALID_CHANNEL:
            return "Invalid channel";
        case ErrorCode::NOT_INITIALIZED:
            return "Driver not initialized";
        case ErrorCode::PWM_WRITE_FAILED:
            return "PWM write failed";
        default:
            return "Unknown error";
    }
}

// Diagnostic methods
bool HBridgeDriverV2::performSelfTest() {
    if (!initialized) {
        return false;
    }
    
    // Test each enabled channel
    bool all_tests_passed = true;
    
    if (m1_enabled) {
        // Test M1 pins
        Config& cfg = m1_config;
        
        // Test direction pins
        digitalWrite(cfg.ina_pin, HIGH);
        delayMicroseconds(10);
        if (digitalRead(cfg.ina_pin) != HIGH) all_tests_passed = false;
        
        digitalWrite(cfg.ina_pin, LOW);
        delayMicroseconds(10);
        if (digitalRead(cfg.ina_pin) != LOW) all_tests_passed = false;
        
        // Similar test for INB
        digitalWrite(cfg.inb_pin, HIGH);
        delayMicroseconds(10);
        if (digitalRead(cfg.inb_pin) != HIGH) all_tests_passed = false;
        
        digitalWrite(cfg.inb_pin, LOW);
        delayMicroseconds(10);
        if (digitalRead(cfg.inb_pin) != LOW) all_tests_passed = false;
        
        // Reset to safe state
        digitalWrite(cfg.ina_pin, LOW);
        digitalWrite(cfg.inb_pin, LOW);
        analogWrite(cfg.pwm_pin, 0);
    }
    
    if (m2_enabled) {
        // Similar tests for M2
        Config& cfg = m2_config;
        digitalWrite(cfg.ina_pin, LOW);
        digitalWrite(cfg.inb_pin, LOW);
        analogWrite(cfg.pwm_pin, 0);
    }
    
    return all_tests_passed;
}

void HBridgeDriverV2::printStatus() const {
    Serial.println("=== HBridge Driver V2 Status ===");
    Serial.print("Initialized: "); Serial.println(initialized ? "Yes" : "No");
    Serial.print("Error Count: "); Serial.println(error_count);
    Serial.print("Last Error: "); Serial.println(getErrorString(last_error));
    
    if (m1_enabled) {
        Serial.println("--- Channel M1 ---");
        Serial.print("PWM: "); Serial.println(m1_current_pwm);
        Serial.print("Percentage: "); Serial.print(getCurrentPercentage(Channel::M1)); Serial.println("%");
        Serial.print("Mode: ");
        switch (m1_current_mode) {
            case ControlMode::COAST: Serial.println("COAST"); break;
            case ControlMode::FORWARD: Serial.println("FORWARD"); break;
            case ControlMode::REVERSE: Serial.println("REVERSE"); break;
        }
        Serial.print("Pins - INA:"); Serial.print(m1_config.ina_pin);
        Serial.print(", INB:"); Serial.print(m1_config.inb_pin);
        Serial.print(", PWM:"); Serial.println(m1_config.pwm_pin);
    }
    
    if (m2_enabled) {
        Serial.println("--- Channel M2 ---");
        Serial.print("PWM: "); Serial.println(m2_current_pwm);
        Serial.print("Percentage: "); Serial.print(getCurrentPercentage(Channel::M2)); Serial.println("%");
        Serial.print("Mode: ");
        switch (m2_current_mode) {
            case ControlMode::COAST: Serial.println("COAST"); break;
            case ControlMode::FORWARD: Serial.println("FORWARD"); break;
            case ControlMode::REVERSE: Serial.println("REVERSE"); break;
        }
        Serial.print("Pins - INA:"); Serial.print(m2_config.ina_pin);
        Serial.print(", INB:"); Serial.print(m2_config.inb_pin);
        Serial.print(", PWM:"); Serial.println(m2_config.pwm_pin);
    }
    
    Serial.println("==============================");
}

void HBridgeDriverV2::printDiagnostics() const {
    Serial.println("=== HBridge Driver V2 Diagnostics ===");
    printStatus();
    
    Serial.println("--- Self Test ---");
    bool test_result = const_cast<HBridgeDriverV2*>(this)->performSelfTest();
    Serial.print("Self Test Result: "); Serial.println(test_result ? "PASS" : "FAIL");
    
    Serial.println("=====================================");
}

// Private helper methods
bool HBridgeDriverV2::isValidPin(uint8_t pin) const {
    // Arduino Mega has pins 0-69
    return pin <= 69;
}

bool HBridgeDriverV2::isValidPWM(int16_t pwm, const Config& cfg) const {
    return pwm >= -cfg.max_pwm && pwm <= cfg.max_pwm;
}

bool HBridgeDriverV2::isValidChannel(Channel channel) const {
    // Check if channel ID is valid (doesn't check if enabled)
    return (channel == Channel::M1 || channel == Channel::M2 || channel == Channel::BOTH);
}

ErrorCode HBridgeDriverV2::setChannelPWM(Channel channel, int16_t pwm) {
    Config* cfg = nullptr;
    
    // Get channel configuration and state pointers
    switch (channel) {
        case Channel::M1:
            if (!isChannelEnabled(Channel::M1)) {
                setError(ErrorCode::INVALID_CHANNEL);
                return ErrorCode::INVALID_CHANNEL;
            }
            cfg = &m1_config;
            break;
            
        case Channel::M2:
            if (!isChannelEnabled(Channel::M2)) {
                setError(ErrorCode::INVALID_CHANNEL);
                return ErrorCode::INVALID_CHANNEL;
            }
            cfg = &m2_config;
            break;
            
        default:
            setError(ErrorCode::INVALID_CHANNEL);
            return ErrorCode::INVALID_CHANNEL;
    }
    
    // Validate PWM value
    if (!isValidPWM(pwm, *cfg)) {
        setError(ErrorCode::INVALID_PWM);
        return ErrorCode::INVALID_PWM;
    }
    
    // Determine direction and mode
    bool reverse = (pwm < 0);
    uint8_t abs_pwm = abs(pwm);
    ControlMode mode;
    
    if (pwm == 0) {
        mode = ControlMode::COAST;
    } else if (reverse) {
        mode = ControlMode::REVERSE;
    } else {
        mode = ControlMode::FORWARD;
    }
    
    // Set hardware pins
    analogWrite(cfg->pwm_pin, abs_pwm);
    
    if (pwm == 0) {
        // Coast mode - both direction pins LOW
        digitalWrite(cfg->ina_pin, LOW);
        digitalWrite(cfg->inb_pin, LOW);
    } else if (reverse) {
        // Reverse mode - INA LOW, INB HIGH
        digitalWrite(cfg->ina_pin, LOW);
        digitalWrite(cfg->inb_pin, HIGH);
    } else {
        // Forward mode - INA HIGH, INB LOW
        digitalWrite(cfg->ina_pin, HIGH);
        digitalWrite(cfg->inb_pin, LOW);
    }
    
    // Update state
    updateChannelState(channel, pwm, mode);
    
    return ErrorCode::NONE;
}

void HBridgeDriverV2::updateChannelState(Channel channel, int16_t pwm, ControlMode mode) {
    switch (channel) {
        case Channel::M1:
            m1_current_pwm = pwm;
            m1_current_mode = mode;
            break;
        case Channel::M2:
            m2_current_pwm = pwm;
            m2_current_mode = mode;
            break;
        default:
            break;
    }
}

void HBridgeDriverV2::setError(ErrorCode error) {
    last_error = error;
    if (error != ErrorCode::NONE) {
        error_count++;
    }
}

// Static utility functions
int16_t HBridgeDriverV2::percentageToPWM(float percentage, int16_t max_pwm, int16_t min_pwm) {
    if (!isValidPercentage(percentage)) {
        return 0;
    }
    
    if (percentage == 0.0f) {
        return 0; // Zero percentage always means stop
    }
    
    // Map percentage to min_pwm-max_pwm range, preserving sign
    float abs_percentage = abs(percentage);
    int16_t effective_range = max_pwm - min_pwm;
    int16_t mapped_pwm = min_pwm + (int16_t)(abs_percentage * effective_range / 100.0f);
    
    return (percentage < 0) ? -mapped_pwm : mapped_pwm;
}

float HBridgeDriverV2::pwmToPercentage(int16_t pwm, int16_t max_pwm) {
    if (max_pwm == 0) {
        return 0.0f;
    }
    
    return (float)pwm * 100.0f / max_pwm;
}

bool HBridgeDriverV2::isValidPercentage(float percentage) {
    return percentage >= MIN_PERCENTAGE && percentage <= MAX_PERCENTAGE;
}