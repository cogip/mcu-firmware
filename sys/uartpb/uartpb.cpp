#include "uartpb/UartProtobuf.hpp"

namespace cogip {

namespace uartpb {

void uart_rx_cb_wrapper(void *arg, uint8_t data)
{
    UartProtobufBase *uartpb = (UartProtobufBase *)arg;
    uartpb->uart_rx_cb(data);
}

void *message_reader_wrapper(void *arg)
{
    UartProtobufBase *uartpb = (UartProtobufBase *)arg;
    uartpb->message_reader();

    return NULL;
}

} // namespace uartpb

} // namespace cogip
