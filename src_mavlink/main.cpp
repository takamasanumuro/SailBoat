#include <Arduino.h>
#include "mavlink/v2.0/common/mavlink.h"

// Define the serial port for MAVLink communication
#define MAVLINK_SERIAL Serial3

// Define system and component IDs for this device (the Arduino)
uint8_t system_id = 255; // 255 is a common ID for GCS or companion computers
uint8_t component_id = MAV_COMP_ID_PERIPHERAL; // A generic component ID

// Timer for requesting streams
unsigned long last_request_time = 0;
const unsigned long request_interval = 3000; // Request every 3 seconds

// Function prototypes
void handle_mavlink_message(mavlink_message_t* msg);
void request_data_stream(uint8_t stream_id, uint16_t rate_hz);

void setup() {
    // Initialize the primary serial port for logging
    Serial.begin(9600);
    while (!Serial) {
        ; // wait for serial port to connect.
    }
    Serial.println("MAVLink RC Channel Extractor");

    // Initialize the MAVLink serial port
    MAVLINK_SERIAL.begin(57600);
    while (!MAVLINK_SERIAL) {
        ; // wait for serial port to connect
    }

    constexpr int COMM_GND_PIN = 19;
    pinMode(COMM_GND_PIN, OUTPUT); digitalWrite(COMM_GND_PIN, LOW);

    Serial.println("Listening for MAVLink messages on Serial3...");
}

void loop() {
    // Periodically request the data streams to ensure they are active
    unsigned long current_time = millis();
    if (current_time - last_request_time > request_interval) {
        last_request_time = current_time;

        // Request RC_CHANNELS stream at 5 Hz
        // This stream includes both RC_CHANNELS and SERVO_OUTPUT_RAW
        request_data_stream(MAV_DATA_STREAM_RC_CHANNELS, 5);
    }

    // Read and parse incoming MAVLink messages
    mavlink_message_t msg;
    mavlink_status_t status;
    while (MAVLINK_SERIAL.available()) {
        uint8_t c = MAVLINK_SERIAL.read();
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
            handle_mavlink_message(&msg);
        }
    }
}

void request_data_stream(uint8_t stream_id, uint16_t rate_hz) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack the message
    mavlink_msg_request_data_stream_pack(
        system_id,
        component_id,
        &msg,
        1, // Target system ID (1 for the Pixhawk/autopilot)
        1, // Target component ID (1 for the autopilot)
        stream_id,
        rate_hz,
        1 // 1 to start sending, 0 to stop
    );

    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    // Send the message over the MAVLink serial port
    MAVLINK_SERIAL.write(buf, len);

    // Log the request to the main serial port
    Serial.print("Requested data stream #");
    Serial.print(stream_id);
    Serial.print(" at ");
    Serial.print(rate_hz);
    Serial.println(" Hz");
}

void handle_mavlink_message(mavlink_message_t* msg) {
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_RC_CHANNELS: {
            mavlink_rc_channels_t rc_channels;
            mavlink_msg_rc_channels_decode(msg, &rc_channels);

            Serial.println("--- RC_CHANNELS (Input) ---");
            Serial.print("  Time: "); Serial.println(rc_channels.time_boot_ms);
            Serial.print("  CH 1: "); Serial.print(rc_channels.chan1_raw);
            Serial.print(" | CH 2: "); Serial.print(rc_channels.chan2_raw);
            Serial.print(" | CH 3: "); Serial.print(rc_channels.chan3_raw);
            Serial.print(" | CH 4: "); Serial.println(rc_channels.chan4_raw);
            Serial.print("  CH 5: "); Serial.print(rc_channels.chan5_raw);
            Serial.print(" | CH 6: "); Serial.print(rc_channels.chan6_raw);
            Serial.print(" | CH 7: "); Serial.print(rc_channels.chan7_raw);
            Serial.print(" | CH 8: "); Serial.println(rc_channels.chan8_raw);
            Serial.print("  RSSI: "); Serial.println(rc_channels.rssi);
            Serial.println("---------------------------");
            break;
        }

        case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: {
            mavlink_servo_output_raw_t servo_output;
            mavlink_msg_servo_output_raw_decode(msg, &servo_output);

            Serial.println("--- SERVO_OUTPUT_RAW (Output) ---");
            Serial.print("  Time: "); Serial.println(servo_output.time_usec);
            Serial.print("  Servo 1: "); Serial.print(servo_output.servo1_raw);
            Serial.print(" | Servo 2: "); Serial.print(servo_output.servo2_raw);
            Serial.print(" | Servo 3: "); Serial.print(servo_output.servo3_raw);
            Serial.print(" | Servo 4: "); Serial.println(servo_output.servo4_raw);
            Serial.print("  Servo 5: "); Serial.print(servo_output.servo5_raw);
            Serial.print(" | Servo 6: "); Serial.print(servo_output.servo6_raw);
            Serial.print(" | Servo 7: "); Serial.print(servo_output.servo7_raw);
            Serial.print(" | Servo 8: "); Serial.println(servo_output.servo8_raw);
            Serial.println("---------------------------------");
            break;
        }

        default:
            // You can add handling for other messages here if needed
            break;
    }
}
