// A small nano-ROS board for the emulator's tests: osu-uwrt/nano_ros's libnanoros over CAN (CANmore messages, as
// titan_firmware's micro-ROS CAN transport sends them) on the Talos LED board, with the topics of ros.toml:
//   * bit_sub (std_msgs/UInt8) in: printed, and published back on bit_echo as data + 1;
//   * imu_cmd (sensor_msgs/Imu) in: printed; its orientation and frame_id go into the next imu_data;
//   * firmware_status (riptide_msgs2/FirmwareStatus) out every second, imu_data (sensor_msgs/Imu) out every 100 ms.
// libnanoros's CAN transport is still micro-ROS's, so the transport below is this program's own.
#include "driver/canbus.h"
#include "nano_ros.h"
#include "nano_ros_codegen.h"
#include "pico/stdlib.h"

#include <stdio.h>
#include <string.h>

#define CANMORE_MSG_MAX 1024

// ---- Transport: one nano-ROS write is one CANmore message; reads hand out a received message piece by piece
static uint8_t rx_msg[CANMORE_MSG_MAX];
static size_t rx_len, rx_pos;

static bool can_t_init(void *ctx) {
    (void) ctx;
    return true;
}
static bool can_t_open(void) {
    return canbus_msg_open();
}
static bool can_t_close(void) {
    canbus_msg_close();
    return true;
}
static size_t can_t_write(const uint8_t *payload, uint16_t len, void *user_data) {
    (void) user_data;
    if (!canbus_msg_write_available()) {
        return 0;
    }
    return canbus_msg_write(payload, len);
}
static size_t can_t_read(uint8_t *buffer, uint16_t len, void *user_data) {
    (void) user_data;
    if (rx_pos == rx_len) {
        if (!canbus_msg_read_available()) {
            return 0;
        }
        rx_len = canbus_msg_read(rx_msg, sizeof(rx_msg));
        rx_pos = 0;
    }
    size_t n = rx_len - rx_pos;
    if (n > len) {
        n = len;
    }
    memcpy(buffer, rx_msg + rx_pos, n);
    rx_pos += n;
    return n;
}

static nros_transport_t can_transport = {
    .init = can_t_init,
    .open = can_t_open,
    .close = can_t_close,
    .write = can_t_write,
    .read = can_t_read,
    .user_data = NULL,
};

// ---- Topics
static std_msgs__msg__UInt8_t echo;
static bool echo_pending;
static sensor_msgs__msg__Imu_t imu;

static void on_bit_sub(const std_msgs__msg__UInt8_t *msg) {
    printf("bit_sub %u\n", msg->data);
    echo.data = (uint8_t) (msg->data + 1);
    echo_pending = true;
}

static void on_imu_cmd(const sensor_msgs__msg__Imu_t *msg) {
    printf("imu_cmd frame_id %s stamp %ld.%09lu orientation %.3f %.3f %.3f %.3f\n", msg->header.frame_id,
           (long) msg->header.stamp.sec, (unsigned long) msg->header.stamp.nanosec, msg->orientation.x,
           msg->orientation.y, msg->orientation.z, msg->orientation.w);
    imu.orientation = msg->orientation;
    strncpy(imu.header.frame_id, msg->header.frame_id, sizeof(imu.header.frame_id) - 1);
}

int main(void) {
    stdio_init_all();
    printf("nano-ROS test board, client %d\n", CAN_BUS_CLIENT_ID);
    if (!canbus_init(CAN_BUS_CLIENT_ID)) {
        panic("Failed to initialize CAN bus hardware!");
    }
    nros_set_bit_sub_subscriber_cb(on_bit_sub);
    nros_set_imu_cmd_subscriber_cb(on_imu_cmd);
    while (!nros_init(&can_transport, true)) {
        canbus_tick();
    }
    printf("nano-ROS up\n");

    strcpy(imu.header.frame_id, "imu_link");
    imu.orientation.w = 1.0;
    imu.linear_acceleration.z = 9.81;
    for (int i = 0; i < 9; i += 4) {
        imu.orientation_covariance[i] = 0.01;
    }

    absolute_time_t next_status = make_timeout_time_ms(1000);
    absolute_time_t next_imu = make_timeout_time_ms(100);
    while (true) {
        canbus_tick();
        nros_spin();
        if (echo_pending && nros_publish_bit_echo(&echo)) {
            echo_pending = false;
        }
        uint64_t now_us = to_us_since_boot(get_absolute_time());
        if (time_reached(next_status)) {
            riptide_msgs2__msg__FirmwareStatus_t st = { 0 };
            strcpy(st.board_name, "nanoros_led");
            st.bus_id = 1;
            st.client_id = CAN_BUS_CLIENT_ID;
            st.version_major = 1;
            st.uptime_ms = (uint32_t) (now_us / 1000);
            if (nros_publish_firmware_status(&st)) {
                next_status = delayed_by_ms(next_status, 1000);
            }
        }
        if (time_reached(next_imu)) {
            imu.header.stamp.sec = (int32_t) (now_us / 1000000);
            imu.header.stamp.nanosec = (uint32_t) (now_us % 1000000) * 1000;
            if (nros_publish_imu_data(&imu)) {
                next_imu = delayed_by_ms(next_imu, 100);
            }
        }
    }
}
