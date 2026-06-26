#include "core/parameter_reader.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

namespace autoware::manual_control
{

struct ParameterReader::Impl
{
  rclcpp::Node * node;
};

ParameterReader::ParameterReader(std::unique_ptr<Impl> impl)
: impl_(std::move(impl)) {}
ParameterReader::~ParameterReader() = default;

// Built here, where Impl is complete, so the gateway factory need not see it.
std::unique_ptr<ParameterReader> make_rclcpp_parameter_reader(rclcpp::Node & node)
{
  return std::make_unique<ParameterReader>(
    std::make_unique<ParameterReader::Impl>(
      ParameterReader::
      Impl{&node}));
}

template<typename T>
T ParameterReader::read(const std::string & name, const T & default_val) const
{
  auto * node = impl_->node;
  if (!node->has_parameter(name)) {
    node->declare_parameter(name, default_val);
  }
  if constexpr (std::is_floating_point_v<T>) {
    // A YAML integer is declared/read as PARAMETER_INTEGER; coerce it to the
    // requested float/double rather than throwing in as_double().
    const auto p = node->get_parameter(name);
    return (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) ?
           static_cast<T>(p.as_int()) :
           static_cast<T>(p.as_double());
  } else {
    T out{};
    node->get_parameter(name, out);
    return out;
  }
}

template float ParameterReader::read<float>(const std::string &, const float &) const;
template double ParameterReader::read<double>(const std::string &, const double &) const;
template std::string ParameterReader::read<std::string>(
  const std::string &,
  const std::string &) const;
template std::vector<std::string>
ParameterReader::read<std::vector<std::string>>(
  const std::string &,
  const std::vector<std::string> &) const;
template std::vector<double>
ParameterReader::read<std::vector<double>>(
  const std::string &,
  const std::vector<double> &) const;

} // namespace autoware::manual_control
