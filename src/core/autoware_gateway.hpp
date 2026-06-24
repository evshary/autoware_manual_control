#ifndef TELEOP_AUTOWARE_GATEWAY_HPP
#define TELEOP_AUTOWARE_GATEWAY_HPP

#include "common/types.hpp"
#include "core/parameter_reader.hpp"
#include <string>
#include <memory>

namespace autoware::manual_control
{

// Concrete Pimpl-based gateway to completely hide ROS/Zenoh transport implementations.
class AutowareGateway
{
public:
  AutowareGateway();
  ~AutowareGateway();

  void spin_some();
  std::string operationModeName() const;
  void toggle_operation_mode();
  void reset_initial_pose();
  VehicleState get_vehicle_state() const;
  bool is_localized() const;
  void set_target_gear(Gear gear);
  void publish_command(const ControlCommand & cmd);
  std::string get_info_message() const;
  bool ok() const;

  // Link-time system abstraction
  static void init_system(int argc, char * argv[]);
  static void shutdown_system();
  static bool ok_system();

  // Builds a reader bound to this transport's parameter source (rclcpp: this
  // node; zenoh: the startup-parsed param map). Defined in the linked transport
  // .cpp, so core never sees rclcpp/zenoh.
  std::unique_ptr<ParameterReader> make_parameter_reader() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace autoware::manual_control

#endif // TELEOP_AUTOWARE_GATEWAY_HPP
