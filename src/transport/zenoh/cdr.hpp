#ifndef TELEOP_AUTOWARE_CDR_CDR_HPP
#define TELEOP_AUTOWARE_CDR_CDR_HPP

#include <fastcdr/Cdr.h>
#include <fastcdr/FastBuffer.h>

#include <cstdint>
#include <stdexcept>
#include <vector>

namespace autoware::manual_control::cdr
{

inline constexpr size_t kWriteBufferBytes = 64 * 1024;

class CdrWriter
{
public:
  CdrWriter()
  : store_(kWriteBufferBytes, 0),
    buffer_(store_.data(), store_.size()),
    cdr_(buffer_, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
      eprosima::fastcdr::Cdr::DDS_CDR)
  {
    cdr_.serialize_encapsulation();
  }

  eprosima::fastcdr::Cdr & cdr() {return cdr_;}

  std::vector<uint8_t> bytes() const
  {
    const auto * p = reinterpret_cast<const uint8_t *>(buffer_.getBuffer());
    return std::vector<uint8_t>(p, p + cdr_.getSerializedDataLength());
  }

private:
  std::vector<char> store_;
  eprosima::fastcdr::FastBuffer buffer_;
  eprosima::fastcdr::Cdr cdr_;
};

class CdrReader
{
public:
  CdrReader(const uint8_t * data, size_t size)
  : store_(data, data + size),
    buffer_(reinterpret_cast<char *>(store_.data()), store_.size()),
    cdr_(buffer_, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
      eprosima::fastcdr::Cdr::DDS_CDR)
  {
    if (size < 4) {
      throw std::runtime_error("cdr: short encapsulation");
    }
    cdr_.read_encapsulation();
  }
  explicit CdrReader(const std::vector<uint8_t> & v)
  : CdrReader(v.data(), v.size()) {}

  eprosima::fastcdr::Cdr & cdr() {return cdr_;}

private:
  std::vector<uint8_t> store_;
  eprosima::fastcdr::FastBuffer buffer_;
  eprosima::fastcdr::Cdr cdr_;
};

} // namespace autoware::manual_control::cdr

#endif // TELEOP_AUTOWARE_CDR_CDR_HPP
