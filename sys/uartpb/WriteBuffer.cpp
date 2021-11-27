#include <string.h>
#include "uartpb/WriteBuffer.hpp"

namespace cogip {

namespace uartpb {

void WriteBuffer::clear()
{
  write_index_ = 0;
}

uint32_t WriteBuffer::get_size() const
{
  return write_index_;
}

uint32_t WriteBuffer::get_max_size() const
{
  return UARTPB_OUTPUT_MESSAGE_LENGTH_MAX;
}

uint32_t WriteBuffer::get_available_size() const
{
  return UARTPB_OUTPUT_MESSAGE_LENGTH_MAX - write_index_;
}

bool WriteBuffer::push(const uint8_t byte)
{
  bool return_value = UARTPB_OUTPUT_MESSAGE_LENGTH_MAX > write_index_;
  if (return_value)
  {
    data_[write_index_] = byte;
    ++write_index_;
  }
  return return_value;
}

bool WriteBuffer::push(const uint8_t *bytes, const uint32_t length)
{
  bool return_value = UARTPB_OUTPUT_MESSAGE_LENGTH_MAX > (write_index_ + length);
  if (return_value)
  {
    memcpy(data_ + write_index_, bytes, length);
    write_index_ += length;
  }
  return return_value;
}

uint8_t * WriteBuffer::get_data()
{
  return data_;
}

} // namespace uartpb

} // namespace cogip
