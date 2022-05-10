#include "platform.hpp"

#include "uartpb_config.hpp"

#include "app_samples.hpp"

#define APP_SAMPLES_MAX_DETECTED 10

PB_Samples<APP_SAMPLES_MAX_DETECTED> samples;
static PB_OutputMessage output_message;

void app_samples_request(void)
{
    cogip::uartpb::UartProtobuf *uartpb = pf_get_uartpb();
    output_message.clear();
    output_message.set_req_samples(true);
    uartpb->send_message(output_message);
}

void app_samples_process(const PB_Samples<APP_SAMPLES_MAX_DETECTED> &samples)
{
    (void)samples;
    puts("Samples received.");
}
