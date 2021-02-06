# hal-uart-proto

## Usage
stm32 serial communication library via nanopb

```c

// dma irq handler
#include <transceiver.h>
extern transceiver_t transceiver;
void USART1_IRQHandler(void)
{  
  HAL_UART_IRQHandler(&huart1);
 
  transceiver_message_receive_handler(&transceiver);    // call handler
}
```

```c
// main.c
#include <transceiver.h>

transceiver_t transceiver; // extern variable
MessageBase rx_message;
uint8_t rx_buffer[100];
uint8_t tx_buffer[100];

int main(void) {
    transceiver.huart = &huart1;
    transceiver.hdma = &hdma_usart1_rx;
    transceiver.fields = MessageBase_fields;
    transceiver.rx_proto_message = &rx_message;
    transceiver.tx_buffer = tx_buffer;
    transceiver.tx_buffer_size = sizeof(tx_buffer);
    transceiver.rx_buffer = tx_buffer;
    transceiver.rx_buffer_size = sizeof(rx_buffer);

    uint32_t pc = 0;
    HAL_UART_Receive_DMA(&huart1, transceiver.rx_buffer, transceiver.rx_buffer_size);
}

// receive decoded message handler
void transceiver_on_rx(void *message) {     // extern callback
    MessageBase *msg = message;
    if (msg->which_type == MessageBase_about_request_tag) {
        MessageBase out = MessageBase_init_zero;
        out.which_type = MessageBase_about_response_tag;
        out.type.about_response.app_id = 42;

        transceiver_transmit_message(&transceiver, &out);
    }
}

```