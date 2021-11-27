#include "uartpb/ReadBuffer.hpp"

namespace cogip {

namespace uartpb {

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
  return UARTPB_INPUT_MESSAGE_LENGTH_MAX;
}

bool ReadBuffer::peek(uint8_t& byte) const
{
  bool return_value = write_index_ > read_index_;
  if (return_value)
  {
    byte = data_[read_index_];
  }
  return return_value;
}

void ReadBuffer::advance()
{
  ++read_index_;
}

void ReadBuffer::advance(const uint32_t n)
{
  read_index_ += n;
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
}

bool ReadBuffer::push(uint8_t &byte)
{
  bool return_value = UARTPB_INPUT_MESSAGE_LENGTH_MAX > write_index_;
  if (return_value)
  {
    data_[write_index_] = byte;
    ++write_index_;
  }
  return return_value;
}

} // namespace uartpb

} // namespace cogip
