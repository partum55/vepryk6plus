#include "mavlink_sender.h"
#include "mavlink_constants.h"
#include "common/mavlink.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>

MAVLinkSender::MAVLinkSender() 
    : source_system(MAVLINK_SYSTEM_ID)
    , target_system(MAVLINK_TARGET_SYSTEM)
    , gear_command_active(false)
    , gear_command_channel(0)
    , gear_command_value(0)
    , gear_command_end_time(0.0)
    , current_gear(0.0)
    , gear_known(false)
    , last_message_check_time(0.0)
{
    // Default RC values
    rc_values.min = 1000;
    rc_values.neutral = 1500;
    rc_values.max = 2000;
    
    // Default gear bits
    gear_bits.gear_up = 11;
    gear_bits.gear_down = 12;
    
    control_state_init(&prev_state);
}

MAVLinkSender::MAVLinkSender(const RCValues& rc_vals, const GearBits& gear_bits_cfg)
    : source_system(MAVLINK_SYSTEM_ID)
    , target_system(MAVLINK_TARGET_SYSTEM)
    , gear_command_active(false)
    , gear_command_channel(0)
    , gear_command_value(0)
    , gear_command_end_time(0.0)
    , current_gear(0.0)
    , gear_known(false)
    , last_message_check_time(0.0)
    , rc_values(rc_vals)
    , gear_bits(gear_bits_cfg)
{
    control_state_init(&prev_state);
}

MAVLinkSender::~MAVLinkSender() {
    disconnect();
}

// Get current time in seconds (with decimal precision)
double MAVLinkSender::get_time() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec + tv.tv_usec / 1000000.0;
}

bool MAVLinkSender::connect(const char* host, int port) {
    if (!bridge.connect(host, port)) {
        return false;
    }
    
    printf("MAVLink sender connected. Waiting for vehicle heartbeat...\n");
    
    // Wait for vehicle heartbeat to detect target system/component
    if (!wait_for_heartbeat()) {
        bridge.disconnect();
        return false;
    }
    
    last_message_check_time = get_time();
    
    return true;
}

bool MAVLinkSender::wait_for_heartbeat() {
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    double start_time = get_time();
    double last_heartbeat_sent = 0;
    double timeout = HEARTBEAT_TIMEOUT;
    
    printf("Waiting for vehicle heartbeat (ignoring system %d)...\n", source_system);
    
    while (get_time() - start_time < timeout) {
        // Send heartbeat every second to announce presence
        double now = get_time();
        if (now - last_heartbeat_sent >= 1.0) {
            sendHeartbeat();
            last_heartbeat_sent = now;
        }

        int bytes_read = bridge.receive(buf, sizeof(buf));
        
        if (bytes_read > 0) {
            for (int i = 0; i < bytes_read; i++) {
                if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                    if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                        // Ignore heartbeats from ourselves or other GCS (MAVProxy is 255)
                        if (msg.sysid != source_system && msg.sysid != 255) {
                            target_system = msg.sysid;
                            target_component = msg.compid;
                            printf("Vehicle connected! System ID: %d, Component ID: %d\n", 
                                   target_system, target_component);
                            return true;
                        }
                    }
                }
            }
        }
        
        usleep(10000); // 10ms delay
    }
    
    printf("Timeout waiting for vehicle heartbeat.\n");
    return false;
}

bool MAVLinkSender::send_mavlink_message(mavlink_message_t* msg) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    
    int bytes_sent = bridge.send(buf, len);
    if (bytes_sent < 0) {
        fprintf(stderr, "Failed to send MAVLink message\n");
        return false;
    }
    
    return true;
}

bool MAVLinkSender::sendControlState(const ControlState& state) {
    // Check for arm/disarm commands
    if (state.arm_requested && !prev_state.arm_requested) {
        send_arm_command(true);
    }
    if (state.disarm_requested && !prev_state.disarm_requested) {
        send_arm_command(false);
    }
    
    // Check for gear changes based on buttons bitmask (from config)
    uint16_t button_gear_up_mask = (1 << gear_bits.gear_up);
    uint16_t button_gear_down_mask = (1 << gear_bits.gear_down);
    
    bool gear_up_pressed = (state.buttons & button_gear_up_mask) != 0;
    bool gear_down_pressed = (state.buttons & button_gear_down_mask) != 0;
    bool prev_gear_up = (prev_state.buttons & button_gear_up_mask) != 0;
    bool prev_gear_down = (prev_state.buttons & button_gear_down_mask) != 0;
    
    // Detect gear up button press (edge detection)
    if (gear_up_pressed && !prev_gear_up) {
        printf("Shifting GEAR UP\n");
        start_gear_change(RC_CHANNEL_GEAR, rc_values.max, GEAR_COMMAND_DURATION);
    }
    
    // Detect gear down button press (edge detection)
    if (gear_down_pressed && !prev_gear_down) {
        printf("Shifting GEAR DOWN\n");
        start_gear_change(RC_CHANNEL_GEAR, rc_values.min, GEAR_COMMAND_DURATION);
    }
    
    // Check messages from vehicle
    check_messages();
    
    // Send RC channels override with steering and throttle
    send_rc_channels_override(state.steering, state.throttle);
    
    // Store previous state
    prev_state = state;
    
    return true;
}

void MAVLinkSender::start_gear_change(uint8_t channel, uint16_t value, double duration) {
    printf("Starting gear change: Channel %d, PWM %d, Duration %.2fs\n", 
           channel, value, duration);
    gear_command_active = true;
    gear_command_channel = channel;
    gear_command_value = value;
    gear_command_end_time = get_time() + duration;
}

void MAVLinkSender::check_messages() {
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    int bytes_read = bridge.receive(buf, sizeof(buf));
    
    if (bytes_read > 0) {
        for (int i = 0; i < bytes_read; i++) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                if (msg.msgid == MAVLINK_MSG_ID_ESTIMATOR_STATUS) {
                    mavlink_estimator_status_t estimator;
                    mavlink_msg_estimator_status_decode(&msg, &estimator);
                    
                    if (!gear_known || estimator.vel_ratio != current_gear) {
                        if (gear_known) {
                            printf("GEAR UPDATE: %.2f -> %.2f\n", 
                                   current_gear, estimator.vel_ratio);
                        }
                        current_gear = estimator.vel_ratio;
                        gear_known = true;
                    }
                }
            }
        }
    }
    
    last_message_check_time = get_time();
}

bool MAVLinkSender::send_rc_channels_override(uint16_t steering, uint16_t throttle) {
    uint16_t overrides[18];
    
    // Initialize all channels to NO_OVERRIDE
    for (int i = 0; i < 18; i++) {
        overrides[i] = NO_OVERRIDE;
    }
    
    // Set steering channel (channel 1, index 0)
    overrides[RC_CHANNEL_STEERING - 1] = steering;
    
    // Set movement channel (channel 3, index 2)
    overrides[RC_CHANNEL_THROTTLE - 1] = throttle;
    
    // Handle gear command if active
    if (gear_command_active) {
        if (get_time() < gear_command_end_time) {
            overrides[gear_command_channel - 1] = gear_command_value;
        } else {
            // Time expired, send neutral and complete
            overrides[gear_command_channel - 1] = rc_values.neutral;
            printf("Gear command complete. Channel %d reset to %d\n", 
                   gear_command_channel, rc_values.neutral);
            gear_command_active = false;
        }
    }
    
    // Send RC_CHANNELS_OVERRIDE message - exactly as in main.c
    mavlink_message_t msg;
    mavlink_msg_rc_channels_override_pack(
        source_system,
        MAVLINK_COMPONENT_ID,
        &msg,
        target_system,
        target_component,
        overrides[0], overrides[1], overrides[2], overrides[3],
        overrides[4], overrides[5], overrides[6], overrides[7],
        overrides[8], overrides[9], overrides[10], overrides[11],
        overrides[12], overrides[13], overrides[14], overrides[15],
        overrides[16], overrides[17]
    );
    
    return send_mavlink_message(&msg);
}

bool MAVLinkSender::send_arm_command(bool arm) {
    printf("Attempting to %s...\n", arm ? "ARM" : "DISARM");
    
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        source_system,
        MAVLINK_COMPONENT_ID,
        &msg,
        target_system,
        target_component,
        MAV_CMD_COMPONENT_ARM_DISARM,
        0,              // confirmation
        arm ? 1.0 : 0.0, // param1: 1 to arm, 0 to disarm
        0, 0, 0, 0, 0, 0
    );
    
    if (!send_mavlink_message(&msg)) {
        return false;
    }
    
    // Wait for ACK
    double start_time = get_time();
    double timeout = ARM_TIMEOUT;
    
    mavlink_message_t ack_msg;
    mavlink_status_t status;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    while (get_time() - start_time < timeout) {
        int bytes_read = bridge.receive(buf, sizeof(buf));
        
        if (bytes_read > 0) {
            for (int i = 0; i < bytes_read; i++) {
                if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &ack_msg, &status)) {
                    if (ack_msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
                        mavlink_command_ack_t ack;
                        mavlink_msg_command_ack_decode(&ack_msg, &ack);
                        
                        if (ack.command == MAV_CMD_COMPONENT_ARM_DISARM) {
                            if (ack.result == MAV_RESULT_ACCEPTED) {
                                printf("%s successfully!\n", arm ? "ARMED" : "DISARMED");
                                return true;
                            } else {
                                printf("Failed to %s! Result: %d\n", 
                                       arm ? "ARM" : "DISARM", ack.result);
                                return false;
                            }
                        }
                    }
                }
            }
        }
        
        usleep(10000); // 10ms delay
    }
    
    printf("Timeout waiting for %s acknowledgment.\n", arm ? "ARM" : "DISARM");
    return false;
}

bool MAVLinkSender::sendHeartbeat() {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(
        source_system,
        MAVLINK_COMPONENT_ID,
        &msg,
        MAV_TYPE_GCS,
        MAV_AUTOPILOT_INVALID,
        0, 0, 0
    );
    
    return send_mavlink_message(&msg);
}

bool MAVLinkSender::isConnected() const {
    return bridge.isConnected();
}

void MAVLinkSender::disconnect() {
    bridge.disconnect();
}
