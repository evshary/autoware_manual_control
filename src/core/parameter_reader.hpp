#ifndef TELEOP_PARAMETER_READER_HPP
#define TELEOP_PARAMETER_READER_HPP

#include <memory>
#include <string>
#include <vector>

namespace autoware::manual_control
{

// Transport-blind parameter reader. The Pimpl hides each transport's read
// machinery (rclcpp::Node / a parsed param map); the read<T> definition lives in
// the selected transport .cpp and is reached via explicit instantiation, so this
// header pulls in neither rclcpp nor zenoh. A transport builds the Impl and binds
// it through AutowareGateway::make_parameter_reader().
class ParameterReader
{
public:
  struct Impl;

  explicit ParameterReader(std::unique_ptr<Impl> impl);
  ~ParameterReader();

  template<typename T>
  T read(const std::string & name, const T & default_val) const;

private:
  std::unique_ptr<Impl> impl_;
};

// read<T> is instantiated once in the linked transport .cpp; extern stops every
// includer from instantiating (and thus needing) the definition. A new type
// needs a line here plus one explicit instantiation per transport.
extern template float ParameterReader::read<float>(const std::string &, const float &) const;
extern template double ParameterReader::read<double>(const std::string &, const double &) const;
extern template std::string ParameterReader::read<std::string>(
  const std::string &,
  const std::string &) const;
extern template std::vector<std::string>
ParameterReader::read<std::vector<std::string>>(
  const std::string &,
  const std::vector<std::string> &) const;

} // namespace autoware::manual_control

#endif // TELEOP_PARAMETER_READER_HPP
