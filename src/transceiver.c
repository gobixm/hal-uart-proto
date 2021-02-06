#include "transceiver.h"

// receive state
int16_t rx_dma_frame_start;
int16_t rx_dma_frame_end;
int16_t rx_dma_cursor = 0;
bool rx_dma_waiting_end = false;

// transmit state
uint8_t tx_cursor = 0;
uint8_t fletcher_sum1 = 0;
uint8_t fletcher_sum2 = 0;

static int16_t circular_find_symbol(transceiver_t* transceiver, int16_t to, uint8_t symbol);

extern void transceiver_on_rx(void *message);

static void handle_frame(transceiver_t *transceiver);

void transceiver_message_receive_handler(transceiver_t *transceiver) {
    uint32_t tmp_flag = 0;

    tmp_flag = __HAL_UART_GET_FLAG(transceiver->huart, UART_FLAG_IDLE);
    if ((tmp_flag != RESET)) {
        __HAL_UART_DISABLE_IT(transceiver->huart, UART_IT_IDLE);

        int16_t cursor = transceiver->rx_buffer_size - transceiver->hdma->Instance->CNDTR;

        while (rx_dma_cursor != cursor) {
            if (!rx_dma_waiting_end) {
                rx_dma_frame_start = circular_find_symbol(transceiver, cursor, FRAME_START_FLAG);
                if (rx_dma_frame_start != -1) {
                    rx_dma_waiting_end = rx_dma_frame_start != -1;
                    rx_dma_cursor = rx_dma_frame_start + 1;
                } else {
                    rx_dma_cursor = cursor;
                }
            }

            if (rx_dma_waiting_end) {
                rx_dma_frame_end = circular_find_symbol(transceiver, cursor, FRAME_END_FLAG);
                if (rx_dma_frame_end != -1) {
                    handle_frame(transceiver);
                    rx_dma_cursor = rx_dma_frame_end + 1;
                    rx_dma_waiting_end = false;
                } else {
                    rx_dma_cursor = cursor;
                }
            }
        }

        __HAL_UART_CLEAR_IDLEFLAG(transceiver->huart);
        __HAL_UART_ENABLE_IT(transceiver->huart, UART_IT_IDLE);
    }
}

static int16_t circular_find_symbol(transceiver_t* transceiver, int16_t to, uint8_t symbol) {
    if (to == rx_dma_cursor + 1) {
        return -1;
    }

    if (to > rx_dma_cursor + 1) {
        for (uint16_t i = rx_dma_cursor; i < to; i++) {
            if (transceiver->rx_buffer[i] == symbol) {
                return i;
            }
        }
    }

    if (to < rx_dma_cursor + 1) {
        for (uint16_t i = rx_dma_cursor; i < transceiver->rx_buffer_size; i++) {
            if (transceiver->rx_buffer[i] == symbol) {
                return i;
            }
        }
        for (uint16_t i = 0; i < to; i++) {
            if (transceiver->rx_buffer[i] == symbol) {
                return i;
            }
        }
    }
    return -1;
}

struct read_state {
    bool escape;
    uint16_t cursor;
    transceiver_t *transceiver;
};

bool read_callback(pb_istream_t *stream, uint8_t *buf, size_t count) {
    struct read_state *state = stream->state;
    uint16_t written = 0;

    for (; stream->bytes_left > 2; stream->bytes_left--) {  //skip crc
        uint8_t byte = state->transceiver->rx_buffer[state->cursor];
        if (byte == FRAME_ESCAPE_FLAG) {
            state->escape = true;
        } else {
            if (state->escape) {
                byte = byte ^ FRAME_ESCAPE_XOR;
                state->escape = false;
            }
            buf[written] = byte;
            written += 1;
        }

        if (state->cursor == state->transceiver->rx_buffer_size - 1) {
            state->cursor = 0;
        } else {
            state->cursor++;
        }
        if (written >= count) {
            return true;
        }
    }

    stream->bytes_left = 0;
    return false;
}

void handle_frame(transceiver_t *transceiver) {
    struct read_state state;
    state.cursor = rx_dma_frame_start + 1;
    state.escape = false;
    state.transceiver = transceiver;
    pb_istream_t stream = {read_callback, &state, SIZE_MAX};
    if (rx_dma_frame_start < rx_dma_frame_end) {
        stream.bytes_left = rx_dma_frame_end - rx_dma_frame_start - 1;
    } else {
        stream.bytes_left = transceiver->rx_buffer_size - rx_dma_frame_start - 1 + rx_dma_frame_end;
    }

    pb_decode(&stream, transceiver->fields, transceiver->rx_proto_message);
    transceiver_on_rx(transceiver->rx_proto_message);
}

void fletcher_16(const uint8_t byte) {
    fletcher_sum1 = ((uint16_t) fletcher_sum1 + byte) % 255;
    fletcher_sum2 = ((uint16_t) fletcher_sum2 + fletcher_sum1) % 255;
}

void write_tx(transceiver_t *transceiver, uint8_t byte, bool checksum) {
    transceiver->tx_buffer[tx_cursor++] = byte;
    if (checksum) {
        fletcher_16(byte);
    }
}

bool transmit_flush(transceiver_t *transceiver) {
    if (tx_cursor > 0) {
        HAL_UART_Transmit((void*)transceiver->huart, transceiver->tx_buffer, tx_cursor, 1000);
    }
    tx_cursor = 0;
    fletcher_sum1 = 0;
    fletcher_sum2 = 0;
    return true;
}

bool transmit(transceiver_t *transceiver, const uint8_t *buf, size_t count, bool escape, bool checksum) {
    for (size_t i = 0; i < count; i++) {
        if (escape && (buf[i] == FRAME_START_FLAG || buf[i] == FRAME_END_FLAG || buf[i] == FRAME_ESCAPE_FLAG)) {
            write_tx(transceiver, FRAME_ESCAPE_FLAG, checksum);
            write_tx(transceiver,buf[i] ^ FRAME_ESCAPE_XOR, checksum);
        } else {
            write_tx(transceiver, buf[i], checksum);
        }
        if (tx_cursor > transceiver->tx_buffer_size - 2) {
            HAL_UART_Transmit(transceiver->huart, transceiver->tx_buffer, tx_cursor, 1000);
            tx_cursor = 0;
        }
    }

    return true;
}

bool transmit_start_frame(transceiver_t *transceiver) {
    uint8_t b = FRAME_START_FLAG;
    return transmit(transceiver, &b, 1, false, false);
}

bool transmit_end_frame(transceiver_t *transceiver) {
    uint8_t b = FRAME_END_FLAG;
    transmit(transceiver, &fletcher_sum2, 1, true, false);
    transmit(transceiver, &fletcher_sum1, 1, true, false);
    return transmit(transceiver, &b, 1, false, false);
}

bool transmit_callback(pb_ostream_t *stream, const pb_byte_t *buf, size_t count) {
    transceiver_t *transceiver = stream->state;
    return transmit(transceiver, buf, count, true, true);
}

void transceiver_transmit_message(transceiver_t *transceiver, void* proto_message) {
    transmit_start_frame(transceiver);

    pb_ostream_t stream = pb_ostream_from_buffer( &transceiver->tx_buffer[tx_cursor], transceiver->tx_buffer_size - tx_cursor);
    stream.state = transceiver;
    stream.callback = transmit_callback;

    pb_encode(&stream, transceiver->fields, proto_message);
    transmit_end_frame(transceiver);
    transmit_flush(transceiver);
}