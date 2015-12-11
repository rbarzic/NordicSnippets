#pragma once

typedef struct {
    uint32_t magic1;   // 4 bytes
    uint16_t throttle; // 2
    uint16_t yaw;      // 2
    uint16_t pitch;    // 2
    uint16_t roll;     // 2
    uint32_t magic2;   // 4
} rc_packet_t;         // total : 16


#define MAGIC1 0xDEADBEEF
#define MAGIC2 0xCAFEBABE
