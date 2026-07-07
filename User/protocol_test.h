#ifndef PROTOCOL_TEST_H__
#define PROTOCOL_TEST_H__

#include <stdint.h>

#define VISION_PAYLOAD_LEN      14
#define VISION_FLAG_FOUND       0x01
#define VISION_FLAG_STABLE      0x02
#define VISION_PRINT_EVERY_N_VALID_PACKETS 0

typedef struct {
    uint16_t seq;
    uint8_t flags;
    int16_t cx;
    int16_t cy;
    int16_t dx;
    int16_t dy;
    uint8_t confidence;
    uint8_t fps;
    uint32_t valid_count;
} vision_target_t;

extern volatile vision_target_t g_vision_target;

void protocol_test_init(void);
void protocol_test_proc(void);

#endif


