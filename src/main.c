#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <time.h>
#include <sys/select.h>

#include "common/mavlink.h"

#define SERIAL_PORT "/dev/ttyS0"
#define BAUD_RATE B57600
#define SYSTEM_ID 255
#define COMPONENT_ID 1

void handle_mavlink_message(mavlink_message_t *msg);
void handle_user_command(const char *command, int serial_fd);
void send_mavlink_arm_command(int serial_fd, int arming_state);


int main() {
    int serial_fd;

    serial_fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY);
    if (serial_fd < 0) {
        perror("Error opening serial port");
        return 1;
    }

    struct termios tty;
    tcgetattr(serial_fd, &tty);
    cfsetospeed(&tty, BAUD_RATE);
    cfsetispeed(&tty, BAUD_RATE);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    tcsetattr(serial_fd, TCSANOW, &tty);

    printf("Serial port configured. Type commands like 'arm' or 'disarm' and press Enter.\n");

    time_t last_heartbeat_time = 0;

    while (1) {
        time_t current_time = time(NULL);
        if (current_time - last_heartbeat_time >= 1) {
            last_heartbeat_time = current_time;
            mavlink_message_t msg;
            uint8_t buf[MAVLINK_MAX_PACKET_LEN];
            mavlink_msg_heartbeat_pack(SYSTEM_ID, COMPONENT_ID, &msg, MAV_TYPE_GCS, MAV_AUTOPILOT_GENERIC, MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, 0, MAV_STATE_ACTIVE);
            uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
            write(serial_fd, buf, len);
        }

        // SETUP I/O MULTIPLEXING WITH select()
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(serial_fd, &read_fds);         // Monitor the serial port
        FD_SET(STDIN_FILENO, &read_fds);      // Monitor the keyboard (stdin)

        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 50000; // 50ms timeout

        int max_fd = (serial_fd > STDIN_FILENO) ? serial_fd : STDIN_FILENO;

        // select() blocks until data is available or the timeout occurs
        int activity = select(max_fd + 1, &read_fds, NULL, NULL, &timeout);

        if (activity < 0) {
            perror("select error");
            break;
        }

        if (activity > 0) {
            // Check for incoming MAVLink messages
            if (FD_ISSET(serial_fd, &read_fds)) {
                uint8_t read_buf[MAVLINK_MAX_PACKET_LEN];
                mavlink_message_t received_msg;
                mavlink_status_t status;
                ssize_t bytes_read = read(serial_fd, read_buf, sizeof(read_buf));
                if (bytes_read > 0) {
                    for (int i = 0; i < bytes_read; ++i) {
                        if (mavlink_parse_char(MAVLINK_COMM_0, read_buf[i], &received_msg, &status)) {
                            handle_mavlink_message(&received_msg);
                        }
                    }
                }
            }

            // Check for user keyboard input
            if (FD_ISSET(STDIN_FILENO, &read_fds)) {
                char user_buf[100];
                if (fgets(user_buf, sizeof(user_buf), stdin)) {
                    user_buf[strcspn(user_buf, "\n")] = 0;
                    handle_user_command(user_buf, serial_fd);
                }
            }
        }
    }

    close(serial_fd);
    return 0;
}

void handle_user_command(const char *command, int serial_fd) {
    if (strcmp(command, "arm") == 0) {
        printf("Sending ARM command...\n");
        send_mavlink_arm_command(serial_fd, 1); // 1 to arm
    } else if (strcmp(command, "disarm") == 0) {
        printf("Sending DISARM command...\n");
        send_mavlink_arm_command(serial_fd, 0); // 0 to disarm
    } else {
        printf("Unknown command: '%s'\n", command);
    }
}

// Function to send an arm/disarm command
void send_mavlink_arm_command(int serial_fd, int arming_state) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // MAV_CMD_COMPONENT_ARM_DISARM
    mavlink_msg_command_long_pack(
        SYSTEM_ID,                // This system's ID
        COMPONENT_ID,             // This system's component ID
        &msg,
        1,                        // Target System ID (1 is typically the vehicle)
        0,                        // Target Component ID (0 means broadcast to all components)
        MAV_CMD_COMPONENT_ARM_DISARM,
        0,                        // Confirmation
        (float)arming_state,      // param1: 1 to arm, 0 to disarm
        0.0f,                     // param2: Not used
        0.0f,                     // param3: Not used
        0.0f,                     // param4: Not used
        0.0f,                     // param5: Not used
        0.0f,                     // param6: Not used
        0.0f                      // param7: Not used
    );

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    write(serial_fd, buf, len);
}

// Function to handle incoming MAVLink messages
void handle_mavlink_message(mavlink_message_t *msg) {
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
            printf("Received HEARTBEAT from vehicle.\n");
            break;
        case MAVLINK_MSG_ID_STATUSTEXT:
            mavlink_statustext_t status;
            mavlink_msg_statustext_decode(msg, &status);
            printf("VEHICLE STATUS: %s\n", status.text);
            break;
        case MAVLINK_MSG_ID_COMMAND_ACK:
            mavlink_command_ack_t ack;
            mavlink_msg_command_ack_decode(msg, &ack);
            printf("Received ACK for command %d with result %d\n", ack.command, ack.result);
            break;
        default:
            break;
    }
}