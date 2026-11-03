// IVCBoard2's main loop in pinger mode without micro-ROS (see CMakeLists.txt): the modem listens for a 25 kHz
// pinger, and every amplitude it would publish on `ivc/pinger/selected_freq_amp_stream` is printed on UART0.
#include "ivc.h"
#include "ros.h"

#include "pico/stdlib.h"
#include "titan/logger.h"

#include <stdio.h>


rcl_ret_t ros_publish_rx(uint8_t rx) {
    (void) rx;
    return RCL_RET_OK;
}
rcl_ret_t ros_publish_adc_sample(uint8_t sample) {
    (void) sample;
    return RCL_RET_OK;
}
rcl_ret_t ros_publish_amplitude(float amp) {
    (void) amp;
    return RCL_RET_OK;
}
rcl_ret_t ros_publish_pinger_amp(float amp) {
    printf("ROS pinger_amp %.3f\n", amp);
    return RCL_RET_OK;
}
rcl_ret_t ros_publish_set_pinger_mode(int32_t khz) {
    printf("ROS pinger_khz %d\n", (int) khz);
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
    LOG_INFO("C IVC pinger");
    ivc_init();
    set_pinger_mode(25);
    pinger_enable(true);
    while (true) {
        ivc_tick();
    }
}
