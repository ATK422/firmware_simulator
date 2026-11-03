// The part of IVCBoard2's ros.h that ivc.c and rx.c use, without micro-ROS.
#ifndef ROS_H
#define ROS_H

#include "ivc.h"

#include <stdint.h>

typedef int rcl_ret_t;
#define RCL_RET_OK 0

rcl_ret_t ros_publish_rx(uint8_t rx);
rcl_ret_t ros_publish_adc_sample(uint8_t sample);
rcl_ret_t ros_publish_amplitude(float amp);
rcl_ret_t ros_publish_pinger_amp(float amp);
rcl_ret_t ros_publish_set_pinger_mode(int32_t khz);
rcl_ret_t ros_publish_sample(int8_t sample);
rcl_ret_t ros_publish_sample_snr(float amp);

#endif
