#pragma once
#include <Arduino.h>
#include "../include/mavlink/v2.0/common/mavlink.h"

/**
 * @brief Mavlink communication module for RC channel monitoring and priority control
 * 
 * This module handles communication with Pixhawk via Mavlink protocol over Serial3.
 * RC output channels from Pixhawk take priority over manual/GUI control when valid.
 * 
 * Channel mapping:
 * - RC Output Channel 0: Rudder control value
 * - RC Output Channel 1: Throttle motor speed control
 * - RC Output Channel 6: Control mode (low PWM = PID angle, high PWM = speed control)
 */
class MavlinkCommunication {
private:
    // Mavlink configuration
    static constexpr uint8_t SYSTEM_ID = 255;                    // Arduino system ID
    static constexpr uint8_t COMPONENT_ID = MAV_COMP_ID_PERIPHERAL; // Component ID
    static constexpr uint8_t TARGET_SYSTEM_ID = 1;               // Pixhawk system ID
    static constexpr uint8_t TARGET_COMPONENT_ID = 1;            // Pixhawk component ID
    
    // Communication timing
    static constexpr uint32_t REQUEST_INTERVAL_MS = 3000;       // Data stream request interval
    static constexpr uint32_t TIMEOUT_MS = 5000;                // Mavlink message timeout
    static constexpr uint16_t STREAM_RATE_HZ = 5;               // RC channels stream rate
    
    // PWM thresholds for mode detection
    static constexpr uint16_t PWM_LOW_THRESHOLD = 1400;         // Below this = LOW
    static constexpr uint16_t PWM_HIGH_THRESHOLD = 1600;        // Above this = HIGH
    static constexpr uint16_t PWM_NEUTRAL = 1500;               // Neutral PWM value
    
    // State variables
    uint32_t last_request_time;
    uint32_t last_mavlink_message_time;
    bool mavlink_active;
    
    // RC channel data
    struct RCChannels {
        uint16_t rudder_value;      // Channel 0: Rudder control (1000-2000)
        uint16_t throttle_value;    // Channel 1: Throttle control (1000-2000) 
        uint16_t mode_value;        // Channel 6: Control mode (1000-2000)
        bool valid;                 // True if recent data available
        uint32_t timestamp;         // Last update time
    } rc_channels;
    
    // Private methods
    void request_data_stream(uint8_t stream_id, uint16_t rate_hz);
    void handle_mavlink_message(mavlink_message_t* msg);
    void handle_rc_channels(const mavlink_rc_channels_t& channels);
    void handle_servo_output(const mavlink_servo_output_raw_t& servo_output);
    bool is_mavlink_timeout() const;
    
public:
    // Control mode enumeration
    enum class ControlMode {
        SPEED_CONTROL,    // Direct speed control
        ANGLE_CONTROL     // PID angle control
    };
    
    // Constructor
    MavlinkCommunication();
    
    // Core methods
    void initialize();
    void update();
    
    // Status methods
    bool is_active() const { return mavlink_active && !is_mavlink_timeout(); }
    bool has_valid_data() const { return rc_channels.valid && is_active(); }
    uint32_t get_last_message_time() const { return last_mavlink_message_time; }
    
    // RC channel access methods
    int get_rudder_command() const;          // Returns -100 to +100
    int get_throttle_command() const;        // Returns 0 to 100
    ControlMode get_control_mode() const;    // Returns current control mode
    
    // Debug methods
    void print_status() const;
    void print_rc_channels() const;
};