#include "core/autoware_gateway.hpp"
#include "core/parameter_reader.hpp"
#include "transport/rclcpp/manual_control_node.hpp"
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace autoware::manual_control
{

// Defined in this transport's parameter_reader.cpp, where Impl is complete.
std::unique_ptr<ParameterReader> make_rclcpp_parameter_reader(rclcpp::Node & node);

struct AutowareGateway::Impl
{
  std::shared_ptr<ManualControlNode> node;
  Impl()
  {
    node = std::make_shared<ManualControlNode>();
  }
};

AutowareGateway::AutowareGateway()
: impl_(std::make_unique<Impl>()) {}
AutowareGateway::~AutowareGateway() = default;

void AutowareGateway::spin_some()
{
  rclcpp::spin_some(impl_->node);
}

std::string AutowareGateway::operationModeName() const
{
  return impl_->node->operationModeName();
}

void AutowareGateway::toggle_operation_mode()
{
  impl_->node->toggle_operation_mode();
}

void AutowareGateway::reset_initial_pose()
{
  impl_->node->reset_initial_pose();
}

VehicleState AutowareGateway::get_vehicle_state() const
{
  return impl_->node->get_vehicle_state();
}

bool AutowareGateway::is_localized() const
{
  return impl_->node->is_localized();
}

void AutowareGateway::set_target_gear(Gear gear)
{
  impl_->node->set_target_gear(gear);
}

void AutowareGateway::publish_command(const ControlCommand & cmd)
{
  impl_->node->publish_command(cmd);
}

std::string AutowareGateway::get_info_message() const
{
  return impl_->node->get_info_message();
}

bool AutowareGateway::ok() const
{
  return rclcpp::ok();
}

void AutowareGateway::init_system(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
}

void AutowareGateway::shutdown_system()
{
  rclcpp::shutdown();
}

bool AutowareGateway::ok_system()
{
  return rclcpp::ok();
}

std::unique_ptr<ParameterReader> AutowareGateway::make_parameter_reader() const
{
  return make_rclcpp_parameter_reader(*impl_->node);
}

} // namespace autoware::manual_control
