// IVCBoard2's main loop without micro-ROS (see CMakeLists.txt): UART0 bytes are enqueued for transmission, received
// packets are printed.
#include "ivc.h"
#include "ros.h"
#include "tx.h"

#include "pico/stdlib.h"
#include "titan/logger.h"

#include <stdio.h>

rcl_ret_t ros_publish_rx(uint8_t rx) {
    printf("ROS ivc/rx_new %d\n", (int8_t) rx);
    return RCL_RET_OK;
}

// Pinger, storage and debug publishers: unused in comms mode
rcl_ret_t ros_publish_adc_sample(uint8_t sample) {
    (void) sample;
    return RCL_RET_OK;
}
rcl_ret_t ros_publish_amplitude(float amp) {
    (void) amp;
    return RCL_RET_OK;
}
rcl_ret_t ros_publish_pinger_amp(float amp) {
    (void) amp;
    return RCL_RET_OK;
}
rcl_ret_t ros_publish_set_pinger_mode(int32_t khz) {
    (void) khz;
    return RCL_RET_OK;
}
rcl_ret_t ros_publish_sample(int8_t sample) {
    (void) sample;
    return RCL_RET_OK;
}
rcl_ret_t ros_publish_sample_snr(float amp) {
    (void) amp;
    return RCL_RET_OK;
}

int main() {
    stdio_init_all();
    LOG_INFO("C IVC modem (IVCBoard2 without micro-ROS)");
    ivc_init();
    while (true) {
        ivc_tick();
        int c = getchar_timeout_us(0);
        if (c >= 0) {
            tx_enqueue_data((uint8_t) c);
        }
    }
}
