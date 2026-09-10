#include "proto_data.h"
#include "esp_log.h"
#include <stddef.h>

#define MEMBER(frame, member) ((frame)[offsetof(data_frame_t, member)])

bool ros_send_data_format_check(const uint8_t *frame) {
    typedef ros_send_data_frame_t data_frame_t;
    if (MEMBER(frame, head) != ROS_HEAD || MEMBER(frame, tail) != ROS_TAIL) {
        return false;
    }

    uint8_t checksum = 0;
    const uint8_t *sp = frame;
    const uint8_t *ep = sp + offsetof(data_frame_t, checksum);
    for (const uint8_t *p = sp; p < ep; ++p) {
        checksum ^= *p;
    }

    return checksum == MEMBER(frame, checksum);
}

bool ros_recv_data_format_check(const uint8_t *frame) {
    typedef ros_recv_data_frame_t data_frame_t;
    if (MEMBER(frame, head) != ROS_HEAD || MEMBER(frame, tail) != ROS_TAIL) {
        return false;
    }

    uint8_t checksum = 0;
    const uint8_t *sp = frame;
    const uint8_t *ep = sp + offsetof(data_frame_t, checksum);
    for (const uint8_t *p = sp; p < ep; ++p) {
        checksum ^= *p;
    }

    return checksum == MEMBER(frame, checksum);
}

bool rc_data_format_check(const uint8_t *frame) {
    typedef remote_data_frame_t data_frame_t;
    static const uint8_t head0 = 0xBB;
    static const uint8_t head1 = 0xBB;
    static const uint8_t function = 1;
    static const uint8_t len = 4;

    if (MEMBER(frame, head0) != head0 || MEMBER(frame, head1) != head1
        || MEMBER(frame, function) != function || MEMBER(frame, len) != len) {
        return false;
    }

    uint8_t checksum = 0;
    const uint8_t *sp = frame;
    const uint8_t *ep = frame + offsetof(data_frame_t, checksum);
    for (const uint8_t *p = sp; p < ep; ++p) {
        checksum ^= *p;
    }

    return checksum == MEMBER(frame, checksum);
}


