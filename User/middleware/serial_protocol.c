#include "serial_protocol.h"
#include <stdio.h>
#include <string.h>

/*
 * MaixCAM2 binary_v1 fixed 19-byte packet:
 * A5 5A 01 seq_lo seq_hi flags cx_lo cx_hi cy_lo cy_hi
 * dx_lo dx_hi dy_lo dy_hi confidence fps bcc 0D 0A
 *
 * bcc is XOR of bytes 2..15, from type through fps.
 */

#define HEAD0               0xA5
#define HEAD1               0x5A
#define TAIL0               0x0D
#define TAIL1               0x0A
#define FRAME_TYPE_TARGET   0x01
#define PACKET_LEN          19
#define PAYLOAD_OFFSET      2
#define PAYLOAD_LEN         14
#define BCC_INDEX           16

static uint8_t bcc_xor(uint8_t* data, uint32_t len)
{
    uint8_t bcc = 0x00;
    uint32_t i;

    for(i = 0; i < len; i++){
        bcc ^= data[i];
    }

    return bcc;
}

/*
 * Check whether data contains a complete valid packet.
 * return:  0 valid
 *         -1 invalid argument
 *         -2 not enough bytes yet
 *         -3 packet format/checksum error at current header
 * redundant: bytes before the candidate header, safe to discard
 */
int32_t packet_is_valid(uint8_t* data, uint32_t len, uint32_t* redundant)
{
    if((data == NULL) || (len == 0) || (redundant == NULL)){
        return -1;
    }

    uint32_t index = 0;
    for(index = 0; index < len; index++){
        if(data[index] != HEAD0){
            continue;
        }
        if((index + 1) >= len){
            break;
        }
        if(data[index + 1] == HEAD1){
            break;
        }
    }
    *redundant = index;

    if((len - index) < PACKET_LEN){
        return -2;
    }

    if((data[index] != HEAD0) ||
       (data[index + 1] != HEAD1) ||
       (data[index + 2] != FRAME_TYPE_TARGET) ||
       (data[index + 17] != TAIL0) ||
       (data[index + 18] != TAIL1) ||
       (bcc_xor(&data[index + PAYLOAD_OFFSET], PAYLOAD_LEN) != data[index + BCC_INDEX])){
        return -3;
    }

    return 0;
}

uint32_t packet_length(uint8_t* data, uint32_t len)
{
    if((data == NULL) || (len < PACKET_LEN) || (data[0] != HEAD0) || (data[1] != HEAD1)){
        return 0;
    }

    return PACKET_LEN;
}

/*
 * Encode binary_v1 payload. Payload must be 14 bytes: type through fps.
 */
int32_t packet_encode(uint8_t* payload, uint32_t len, uint8_t* packet_buff, uint32_t buff_len)
{
    if((payload == NULL) || (packet_buff == NULL) || (len != PAYLOAD_LEN) || (buff_len < PACKET_LEN)){
        return -1;
    }

    uint32_t index = 0;

    packet_buff[index++] = HEAD0;
    packet_buff[index++] = HEAD1;

    memcpy(&packet_buff[index], payload, len);
    index += len;

    packet_buff[index++] = bcc_xor(payload, len);
    packet_buff[index++] = TAIL0;
    packet_buff[index++] = TAIL1;

    return index;
}

/*
 * Decode binary_v1 payload, returns 14 bytes: type through fps.
 */
int32_t packet_decode(uint8_t* data, uint32_t len, uint8_t* payload_buff, uint32_t buff_len)
{
    if((data == NULL) || (payload_buff == NULL) || (buff_len < PAYLOAD_LEN)){
        return -1;
    }
    if((len < PACKET_LEN) || (data[0] != HEAD0) || (data[1] != HEAD1)){
        return -1;
    }

    memcpy(payload_buff, &data[PAYLOAD_OFFSET], PAYLOAD_LEN);
    return PAYLOAD_LEN;
}



