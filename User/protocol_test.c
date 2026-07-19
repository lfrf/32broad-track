#include "stm32f1xx.h"
#include "ringbuffer.h"
#include "serial_protocol.h"
#include "protocol_test.h"

#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart2;
rbuffer_t rbProtocol;

static uint8_t _u8Data = 0x00;
static uint8_t _aBuffer[256] = {0x00};

volatile vision_target_t g_vision_target = {0};

static uint16_t read_u16_le(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static int16_t read_i16_le(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void parse_vision_payload(const uint8_t *payload)
{
    if(payload[0] != 0x01){
        return;
    }

    g_vision_target.seq = read_u16_le(&payload[1]);
    g_vision_target.flags = payload[3];
    g_vision_target.cx = read_i16_le(&payload[4]);
    g_vision_target.cy = read_i16_le(&payload[6]);
    g_vision_target.dx = read_i16_le(&payload[8]);
    g_vision_target.dy = read_i16_le(&payload[10]);
    g_vision_target.confidence = payload[12];
    g_vision_target.fps = payload[13];
    g_vision_target.valid_count++;
}

/*
 * USART2: PA2 TX, PA3 RX, 115200 8N1.
 * Connect MaixCAM2 A21/UART4_TX to STM32 PA3/USART2_RX, and connect GND.
 */
void protocol_test_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin   = GPIO_PIN_2;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin  = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    huart2.Instance        = USART2;
    huart2.Init.BaudRate   = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits   = UART_STOPBITS_1;
    huart2.Init.Parity     = UART_PARITY_NONE;
    huart2.Init.Mode       = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    HAL_UART_Init(&huart2);
    HAL_UART_Receive_IT(&huart2, &_u8Data, 1);

    rbuffer_init(&rbProtocol, _aBuffer, sizeof(_aBuffer));
}

void protocol_test_proc(void)
{
    int32_t rc = -1;
    int32_t payload_len = 0;
    uint32_t redundant = 0x00, packet_len = 0;
    uint8_t buffer[128] = {0x00};
    uint8_t payload[32] = {0x00};
    uint32_t len = rbuffer_data_len(&rbProtocol);

    if(len == 0){
        return;
    }
    len = (len <= sizeof(buffer)) ? len : sizeof(buffer);

    rbuffer_peek(&rbProtocol, buffer, len);

    rc = packet_is_valid(buffer, len, &redundant);
    if(redundant > 0){
        rbuffer_del(&rbProtocol, redundant);
        return;
    }

    if(rc == -2){
        return;
    }

    if(rc == -3){
        rbuffer_del(&rbProtocol, 1);
        return;
    }

    if(rc >= 0){
        payload_len = packet_decode(buffer, len, payload, sizeof(payload));
        if(payload_len == VISION_PAYLOAD_LEN){
            parse_vision_payload(payload);

            if((VISION_PRINT_EVERY_N_VALID_PACKETS > 0) && ((g_vision_target.valid_count % VISION_PRINT_EVERY_N_VALID_PACKETS) == 0)){
                printf("seq=%u found=%u stable=%u cx=%d cy=%d dx_ex10=%d dy_ey10=%d conf=%u fps=%u count=%lu\r\n",
                       (unsigned int)g_vision_target.seq,
                       (unsigned int)((g_vision_target.flags & VISION_FLAG_FOUND) ? 1 : 0),
                       (unsigned int)((g_vision_target.flags & VISION_FLAG_STABLE) ? 1 : 0),
                       (int)g_vision_target.cx,
                       (int)g_vision_target.cy,
                       (int)g_vision_target.dx,
                       (int)g_vision_target.dy,
                       (unsigned int)g_vision_target.confidence,
                       (unsigned int)g_vision_target.fps,
                       (unsigned long)g_vision_target.valid_count);
            }
        }

        packet_len = packet_length(buffer, len);
        if(packet_len > 0){
            rbuffer_del(&rbProtocol, packet_len);
        }else{
            rbuffer_del(&rbProtocol, 1);
        }
    }
    else if(rbuffer_status(&rbProtocol) == RB_FULL){
        rbuffer_del(&rbProtocol, len);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2){
        rbuffer_putchar(&rbProtocol, _u8Data);
        HAL_UART_Receive_IT(&huart2, &_u8Data, 1);
    }
}




