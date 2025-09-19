#include "mavlink_communication.hpp"
#include <avr/pgmspace.h>

// Store status strings in Flash memory
const char PROGMEM mavlink_status_header[] = "=== MAVLINK STATUS ===";
const char PROGMEM mavlink_status_footer[] = "======================";
const char PROGMEM mavlink_channels_header[] = "--- MAVLINK RC OUTPUT ---";
const char PROGMEM mavlink_channels_footer[] = "-------------------------";

MavlinkCommunication::MavlinkCommunication() 
    : last_request_time(0)
    , last_mavlink_message_time(0)
    , mavlink_active(false)
    , rc_channels{0, 0, 0, false, 0}
{
}

void MavlinkCommunication::initialize() {
    Serial.println(F("Initializing Mavlink on Serial3..."));
    
    // Initialize Serial3 for Mavlink communication
    Serial3.begin(57600);
    while (!Serial3) {
        ; // Wait for serial port to connect
    }
    
    // Setup communication ground pin (from original mavlink code)
    constexpr int COMM_GND_PIN = 19;
    pinMode(COMM_GND_PIN, OUTPUT);
    digitalWrite(COMM_GND_PIN, LOW);
    
    Serial.println(F("Mavlink ready: CH0=Rudder, CH1=Throttle, CH6=Mode"));
}

void MavlinkCommunication::update() {
    uint32_t current_time = millis();
    
    // Periodically request data streams
    if (current_time - last_request_time > REQUEST_INTERVAL_MS) {
        last_request_time = current_time;
        request_data_stream(MAV_DATA_STREAM_RC_CHANNELS, STREAM_RATE_HZ);
    }
    
    // Process incoming Mavlink messages
    mavlink_message_t msg;
    mavlink_status_t status;
    
    while (Serial3.available()) {
        uint8_t c = Serial3.read();
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
            handle_mavlink_message(&msg);
        }
    }
    
    // Update active status based on recent messages
    if (is_mavlink_timeout() && mavlink_active) {
        mavlink_active = false;
        rc_channels.valid = false;
        Serial.println("[MAVLINK] Communication timeout - switching to manual control");
    }
}

void MavlinkCommunication::request_data_stream(uint8_t stream_id, uint16_t rate_hz) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    // Pack the request message
    mavlink_msg_request_data_stream_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &msg,
        TARGET_SYSTEM_ID,
        TARGET_COMPONENT_ID,
        stream_id,
        rate_hz,
        1  // Start sending
    );
    
    // Send the message
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial3.write(buf, len);
}

void MavlinkCommunication::handle_mavlink_message(mavlink_message_t* msg) {
    last_mavlink_message_time = millis();
    
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_RC_CHANNELS: {
            mavlink_rc_channels_t rc_channels_msg;
            mavlink_msg_rc_channels_decode(msg, &rc_channels_msg);
            handle_rc_channels(rc_channels_msg);
            break;
        }
        
        case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: {
            mavlink_servo_output_raw_t servo_output;
            mavlink_msg_servo_output_raw_decode(msg, &servo_output);
            handle_servo_output(servo_output);
            break;
        }
        
        default:
            // Ignore other message types
            break;
    }
}

void MavlinkCommunication::handle_rc_channels(const mavlink_rc_channels_t& channels) {
    // Note: RC_CHANNELS contains input channels, not output channels
    // We'll primarily use SERVO_OUTPUT_RAW for control commands
    if (!mavlink_active) {
        mavlink_active = true;
        Serial.println("[MAVLINK] RC input channels received - Mavlink active");
    }
}

void MavlinkCommunication::handle_servo_output(const mavlink_servo_output_raw_t& servo_output) {
    // Update RC channel data from servo output (these are the commands we need)
    rc_channels.throttle_value = servo_output.servo1_raw; 
    rc_channels.rudder_value = servo_output.servo2_raw;   
    rc_channels.mode_value = servo_output.servo6_raw;     
    rc_channels.valid = true;
    rc_channels.timestamp = millis();
    
    if (!mavlink_active) {
        mavlink_active = true;
        Serial.println("[MAVLINK] Servo output received - Mavlink control active");
    }
}

bool MavlinkCommunication::is_mavlink_timeout() const {
    return (millis() - last_mavlink_message_time) > TIMEOUT_MS;
}

int MavlinkCommunication::get_rudder_command_percentage() const {
    if (!has_valid_data()) {
        return 0;  // No valid data, return neutral
    }

    // Convert PWM (1000-2000) to command range (-100 to +100)
    int pwm = rc_channels.rudder_value;
    if (pwm < 1000) pwm = 1000;
    if (pwm > 2000) pwm = 2000;

    // Add a deadzone of +/- 20 around the neutral 1500 to avoid jitter
    const int deadzone = 20;
    if (abs(pwm - 1500) < deadzone) {
        return 0; // Input is inside the deadzone, return neutral
    }

    // Map 1000-2000 to -100 to +100
    return map(pwm, 1000, 2000, -100, 100);
}

int MavlinkCommunication::get_throttle_command() const {
    if (!has_valid_data()) {
        return 0;  // No valid data, return zero throttle
    }
    
    // Convert PWM (1000-2000) to throttle range (0 to 100)
    int pwm = rc_channels.throttle_value;
    if (pwm < 1000) pwm = 1000;
    if (pwm > 2000) pwm = 2000;
    
    // Map 1000-2000 to 0 to 100
    return map(pwm, 1000, 2000, 0, 100);
}

MavlinkCommunication::ControlMode MavlinkCommunication::get_control_mode() const {
    if (!has_valid_data()) {
        return ControlMode::ANGLE_CONTROL;  // Default to angle control
    }
    
    // Check mode channel (Channel 6)
    uint16_t mode_pwm = rc_channels.mode_value;
    
    if (mode_pwm < PWM_LOW_THRESHOLD) {
        return ControlMode::ANGLE_CONTROL;  // Low PWM = PID angle control
    } else if (mode_pwm > PWM_HIGH_THRESHOLD) {
        return ControlMode::SPEED_CONTROL;  // High PWM = speed control
    } else {
        return ControlMode::ANGLE_CONTROL;  // Default for neutral
    }
}

void MavlinkCommunication::print_status() const {
    Serial.println(reinterpret_cast<const __FlashStringHelper*>(mavlink_status_header));
    Serial.print(F("Active: ")); Serial.println(is_active() ? F("YES") : F("NO"));
    Serial.print(F("Valid Data: ")); Serial.println(has_valid_data() ? F("YES") : F("NO"));
    Serial.print(F("Last Message: ")); Serial.print(millis() - last_mavlink_message_time); Serial.println(F("ms ago"));
    
    if (has_valid_data()) {
        Serial.print(F("Control Mode: "));
        Serial.println(get_control_mode() == ControlMode::ANGLE_CONTROL ? F("PID ANGLE") : F("SPEED"));
        Serial.print(F("Rudder: ")); Serial.print(get_rudder_command_percentage()); Serial.println(F("%"));
        Serial.print(F("Throttle: ")); Serial.print(get_throttle_command()); Serial.println(F("%"));
    }
    Serial.println(reinterpret_cast<const __FlashStringHelper*>(mavlink_status_footer));
}

void MavlinkCommunication::print_rc_channels() const {
    if (!has_valid_data()) {
        Serial.println(F("[MAVLINK] No valid RC data"));
        return;
    }
    
    Serial.println(reinterpret_cast<const __FlashStringHelper*>(mavlink_channels_header));
    Serial.print(F("Rudder: ")); Serial.print(rc_channels.rudder_value); 
    Serial.print(F(" -> ")); Serial.print(get_rudder_command_percentage()); Serial.println(F("%"));
    
    Serial.print(F("Throttle: ")); Serial.print(rc_channels.throttle_value); 
    Serial.print(F(" -> ")); Serial.print(get_throttle_command()); Serial.println(F("%"));
    
    Serial.print(F("Mode: ")); Serial.print(rc_channels.mode_value); 
    Serial.print(F(" -> ")); Serial.println(get_control_mode() == ControlMode::ANGLE_CONTROL ? F("PID") : F("SPEED"));
    Serial.println(reinterpret_cast<const __FlashStringHelper*>(mavlink_channels_footer));
}