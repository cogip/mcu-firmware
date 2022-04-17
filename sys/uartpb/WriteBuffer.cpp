// System includes
#include <cstdio>
#include <string.h>

// RIOT includes
#include "base64.h"

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

size_t WriteBuffer::base64_encode()
{
    size_t base64_buffer_size = 0;
    memset(base64_data_, '\n', UARTPB_BASE64_ENCODE_BUFFER_SIZE);
    int ret = ::base64_encode(data_, write_index_, NULL, &base64_buffer_size);
    if (ret != BASE64_ERROR_BUFFER_OUT_SIZE) {
        printf("1 ret = %d (success = %d)\n", ret, BASE64_SUCCESS);
        return 0;
    }
    if (base64_buffer_size > UARTPB_BASE64_ENCODE_BUFFER_SIZE) {
        printf("Failed to base64 encode, buffer too small (%u > %u).\n", base64_buffer_size, UARTPB_BASE64_ENCODE_BUFFER_SIZE);
        return 0;
    }

    ret = ::base64_encode(data_, write_index_, base64_data_, &base64_buffer_size);
    if (ret != BASE64_SUCCESS) {
        printf("2 ret = %d (success = %d)\n", ret, BASE64_SUCCESS);
        return 0;
    }

    return base64_buffer_size;
}

uint8_t * WriteBuffer::get_base64_data()
{
    return base64_data_;
}

} // namespace uartpb

} // namespace cogip
