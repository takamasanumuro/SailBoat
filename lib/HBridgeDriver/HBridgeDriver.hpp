#pragma once
#include <Arduino.h>

/**
 * @brief Improved H-Bridge Driver for UN178-based motor controllers
 * @version 2.0
 * 
 * Features:
 * - Type safety and input validation
 * - Error handling and status reporting  
 * - Emergency stop functionality
 * - Current state tracking
 * - Consistent API design
 * 
 * Note: Brake mode (both pins HIGH) is NOT supported as it can damage the H-bridge
 */

class HBridgeDriverV2 {
public:
    // Channel enumeration
    enum class Channel : uint8_t {
        M1 = 0,
        M2 = 1,
        BOTH = 2
    };
    
    // Motor control modes
    enum class ControlMode : uint8_t {
        COAST = 0,  // Both direction pins LOW (free spinning)
        FORWARD = 1,
        REVERSE = 2
    };
    
    // Error codes
    enum class ErrorCode : uint8_t {
        NONE = 0,
        INVALID_PIN = 1,
        INVALID_PWM = 2,
        INVALID_CHANNEL = 3,
        NOT_INITIALIZED = 4,
        PWM_WRITE_FAILED = 5
    };
    
    // Configuration structure
    struct Config {
        uint8_t ina_pin;
        uint8_t inb_pin;
        uint8_t pwm_pin;
        int16_t max_pwm = 240;          // Maximum PWM value (94% duty cycle)
        int16_t min_pwm = 0;            // Minimum PWM value (deadband threshold)
    };
    
private:
    Config m1_config;
    Config m2_config;
    bool m1_enabled;
    bool m2_enabled;
    bool initialized;
    
    // Current state tracking
    int16_t m1_current_pwm;
    int16_t m2_current_pwm;
    ControlMode m1_current_mode;
    ControlMode m2_current_mode;
    
    // Error tracking
    ErrorCode last_error;
    uint32_t error_count;
    
    // Private helper methods
    bool isValidPin(uint8_t pin) const;
    bool isValidPWM(int16_t pwm, const Config& cfg) const;
    bool isValidChannel(Channel channel) const; // Checks if channel ID is valid (M1/M2)
    bool isChannelEnabled(Channel channel) const; // Checks if channel is enabled
    ErrorCode setChannelPWM(Channel channel, int16_t pwm);
    void updateChannelState(Channel channel, int16_t pwm, ControlMode mode);
    void setError(ErrorCode error);
    
public:
    // Constructors
    HBridgeDriverV2();
    HBridgeDriverV2(const Config& m1_cfg);
    HBridgeDriverV2(const Config& m1_cfg, const Config& m2_cfg);
    
    // Initialization
    ErrorCode init();
    ErrorCode initChannel(Channel channel);
    bool isInitialized() const { return initialized; }
    
    // Configuration
    ErrorCode setChannelConfig(Channel channel, const Config& cfg);
    Config getChannelConfig(Channel channel) const;
    void setMaxPWM(Channel channel, int16_t max_pwm);
    int16_t getMaxPWM(Channel channel) const;
    
    // Motor control
    ErrorCode setPWM(int16_t pwm, Channel channel);
    ErrorCode setPercentage(float percentage, Channel channel); // -100.0 to +100.0
    ErrorCode stop(Channel channel = Channel::BOTH);
    ErrorCode coast(Channel channel = Channel::BOTH);
    ErrorCode emergencyStop(); // Immediate stop all channels
    
    // Status queries  
    int16_t getCurrentPWM(Channel channel) const;
    float getCurrentPercentage(Channel channel) const;
    ControlMode getCurrentMode(Channel channel) const;
    bool isChannelActive(Channel channel) const;
    
    // Error handling
    ErrorCode getLastError() const { return last_error; }
    uint32_t getErrorCount() const { return error_count; }
    const char* getErrorString(ErrorCode error) const;
    void clearError() { last_error = ErrorCode::NONE; }
    
    // Diagnostics
    bool performSelfTest();
    void printStatus() const;
    void printDiagnostics() const;
    
    // Static constants
    static constexpr int16_t MAX_PWM_VALUE = 255;
    static constexpr int16_t DEFAULT_MAX_PWM = 240; // 94% duty cycle
    static constexpr int16_t MIN_PWM_VALUE = 0;
    static constexpr float MAX_PERCENTAGE = 100.0f;
    static constexpr float MIN_PERCENTAGE = -100.0f;
    
    // Static utility functions
    static int16_t percentageToPWM(float percentage, int16_t max_pwm = DEFAULT_MAX_PWM, int16_t min_pwm = 0);
    static float pwmToPercentage(int16_t pwm, int16_t max_pwm = DEFAULT_MAX_PWM);
    static bool isValidPercentage(float percentage);
};