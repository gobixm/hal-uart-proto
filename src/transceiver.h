//
// Created by gobi on 2/2/2021.
//

#ifndef STM32_PROTO_TRANSCEIVER_H
#define STM32_PROTO_TRANSCEIVER_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <pb.h>
#include <pb_decode.h>
#include <pb_encode.h>

#define FRAME_START_FLAG 0x7C
#define FRAME_ESCAPE_FLAG 0x7D
#define FRAME_END_FLAG 0x7E
#define FRAME_ESCAPE_XOR 0x20

//#define RX_DMA_BUFFER_SIZE 100
//#define TX_BUFFER_SIZE 100

typedef struct transceiver_t {
    UART_HandleTypeDef *huart;
    DMA_HandleTypeDef *hdma;
    const pb_msgdesc_t *fields;
    uint16_t tx_buffer_size;
    uint8_t *rx_buffer;
    uint16_t rx_buffer_size;
    uint8_t *tx_buffer;
    void* rx_proto_message;
} transceiver_t;

void transceiver_message_receive_handler(transceiver_t *transceiver);
void transceiver_transmit_message(transceiver_t *transceiver, void* proto_message);

#endif //STM32_PROTO_TRANSCEIVER_H
