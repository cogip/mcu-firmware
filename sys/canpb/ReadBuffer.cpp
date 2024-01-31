// System includes
#include <cstdio>
#include <string.h>

// RIOT includes
#include "base64.h"

#include "canpb/ReadBuffer.hpp"

namespace cogip {

namespace canpb {

ReadBuffer::ReadBuffer()
  : data_{0},
    write_index_(0),
    read_index_(0)
{
}

uint32_t ReadBuffer::get_size() const
{
  return write_index_;
}

uint32_t ReadBuffer::get_max_size() const
{
  return CANPB_INPUT_MESSAGE_LENGTH_MAX;
}

bool ReadBuffer::peek(uint8_t& byte) const
{
  bool return_value = write_index_ > read_index_;
  if (return_value) {
    byte = data_[read_index_];
  }
  return return_value;
}

bool ReadBuffer::advance()
{
  const bool return_value = write_index_ > read_index_;
  if(return_value) {
    ++read_index_;
  }
  return return_value;
}

bool ReadBuffer::advance(const uint32_t n)
{
  const uint32_t new_read_index = read_index_ + n;
  const bool return_value = write_index_ > new_read_index;
  if(return_value) {
    read_index_ = new_read_index;
  }
  return return_value;
}

bool ReadBuffer::pop(uint8_t &byte)
{
  bool return_value = write_index_ > read_index_;
  if(return_value) {
    byte = data_[read_index_];
    ++read_index_;
  }
  return return_value;
}

uint8_t * ReadBuffer::get_data_array()
{
  return data_;
}

uint32_t & ReadBuffer::get_bytes_written()
{
  return write_index_;
}

void ReadBuffer::clear()
{
  read_index_ = 0;
  write_index_ = 0;
  memset(base64_data_, 0, CANPB_BASE64_DECODE_BUFFER_SIZE);
}

bool ReadBuffer::push(uint8_t &byte)
{
  bool return_value = CANPB_INPUT_MESSAGE_LENGTH_MAX > write_index_;
  if (return_value) {
    data_[write_index_] = byte;
    ++write_index_;
  }
  return return_value;
}

uint8_t * ReadBuffer::get_base64_data()
{
    return base64_data_;
}

size_t ReadBuffer::base64_decode()
{
    size_t base64_message_length = strlen((const char *)base64_data_);
    size_t pb_buffer_size = 0;
    int ret = ::base64_decode(base64_data_, base64_message_length, NULL, &pb_buffer_size);
    if (ret != BASE64_ERROR_BUFFER_OUT_SIZE) {
        return 0;
    }
    if (pb_buffer_size > CANPB_INPUT_MESSAGE_LENGTH_MAX) {
        printf("Failed to base64 decode, buffer too small (%zu > %u).\n", pb_buffer_size, CANPB_INPUT_MESSAGE_LENGTH_MAX);
        return 0;
    }
    ret = ::base64_decode(base64_data_, base64_message_length, data_, &pb_buffer_size);
    if (ret != BASE64_SUCCESS) {
        printf("Failed to base64 decode (ret = %d)\n", ret);
        return 0;
    }
    write_index_ = pb_buffer_size;
    return pb_buffer_size;
}

} // namespace canpb

} // namespace cogip
