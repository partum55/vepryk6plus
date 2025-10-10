#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <time.h>

#include "common/mavlink.h"

#define SERIAL_PORT "/dev/ttyS0"
#define BAUD_RATE B57600
#define SYSTEM_ID 255
#define COMPONENT_ID 1

void handle_mavlink_message(mavlink_message_t *msg) {
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t heartbeat;
            mavlink_msg_heartbeat_decode(msg, &heartbeat);
            printf("Received HEARTBEAT from system %d, component %d. Vehicle is in state %d.\n",
                   msg->sysid, msg->compid, heartbeat.system_status);
            break;
        }
        case MAVLINK_MSG_ID_STATUSTEXT: {
            mavlink_statustext_t status;
            mavlink_msg_statustext_decode(msg, &status);
            printf("Received STATUSTEXT: %s\n", status.text);
            break;
        }
        case MAVLINK_MSG_ID_COMMAND_ACK: {
            mavlink_command_ack_t ack;
            mavlink_msg_command_ack_decode(msg, &ack);
            printf("Received COMMAND_ACK: Command %d, Result %d\n", ack.command, ack.result);
            break;
        }
        default:
            break;
    }
}

int main() {
    int serial_fd;

    serial_fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY);
    if (serial_fd < 0) {
        perror("Error opening serial port");
        return 1;
    }

    struct termios tty;
    if (tcgetattr(serial_fd, &tty) != 0) {
        perror("Error from tcgetattr");
        close(serial_fd);
        return 1;
    }
    cfsetospeed(&tty, BAUD_RATE);
    cfsetispeed(&tty, BAUD_RATE);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;
    // Set VMIN = 0, VTIME = 0 for non-blocking read
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        perror("Error from tcsetattr");
        close(serial_fd);
        return 1;
    }

    printf("Serial port configured. Starting MAVLink communication...\n");

    time_t last_heartbeat_time = 0;

    while (1) {
        time_t current_time = time(NULL);
        if (current_time - last_heartbeat_time >= 1) {
            last_heartbeat_time = current_time;

            mavlink_message_t msg;
            uint8_t buf[MAVLINK_MAX_PACKET_LEN];

            mavlink_msg_heartbeat_pack(
                SYSTEM_ID,
                COMPONENT_ID,
                &msg,
                MAV_TYPE_GCS,
                MAV_AUTOPILOT_GENERIC,
                MAV_MODE_FLAG_MANUAL_INPUT_ENABLED,
                0,
                MAV_STATE_ACTIVE
            );

            uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

            ssize_t bytes_written = write(serial_fd, buf, len);
            if (bytes_written > 0) {
                printf("Heartbeat sent (%zd bytes).\n", bytes_written); // NOISE
            } else {
                perror("Failed to write to serial port");
            }
        }

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

        usleep(10000); // 1ms sleep
    }

    close(serial_fd);
    return 0;
}