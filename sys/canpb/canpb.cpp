#include "canpb/CanProtobuf.hpp"

namespace cogip {

namespace canpb {

void *message_reader_wrapper(void *arg)
{
    CanProtobuf *canpb = static_cast<CanProtobuf *>(arg);
    canpb->message_reader();

    return NULL;
}

} // namespace canpb

} // namespace cogip
