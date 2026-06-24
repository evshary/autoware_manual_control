#include "core/autoware_gateway.hpp"
#include "core/parameter_reader.hpp"
#include "transport/zenoh/cdr.hpp"
#include "transport/zenoh/cli_params.hpp"

// Message types and their fastrtps codec, regenerated from the installed .idl
// under this project's namespace so the binary links no message .so.
#include "autoware_manual_control/msg/control.hpp"
#include "autoware_manual_control/msg/gear_command.hpp"
#include "autoware_manual_control/msg/gear_report.hpp"
#include "autoware_manual_control/msg/localization_initialization_state.hpp"
#include "autoware_manual_control/msg/manual_operator_heartbeat.hpp"
#include "autoware_manual_control/msg/operation_mode_state.hpp"
#include "autoware_manual_control/msg/steering_report.hpp"
#include "autoware_manual_control/msg/velocity_report.hpp"
#include "autoware_manual_control/srv/change_operation_mode.hpp"

#include "autoware_manual_control/msg/detail/control__rosidl_typesupport_fastrtps_cpp.hpp"
#include "autoware_manual_control/msg/detail/gear_command__rosidl_typesupport_fastrtps_cpp.hpp"
#include "autoware_manual_control/msg/detail/gear_report__rosidl_typesupport_fastrtps_cpp.hpp"
#include \
  "autoware_manual_control/msg/detail/localization_initialization_state__rosidl_typesupport_fastrtps_cpp.hpp"
#include \
  "autoware_manual_control/msg/detail/manual_operator_heartbeat__rosidl_typesupport_fastrtps_cpp.hpp"
#include \
  "autoware_manual_control/msg/detail/operation_mode_state__rosidl_typesupport_fastrtps_cpp.hpp"
#include "autoware_manual_control/msg/detail/steering_report__rosidl_typesupport_fastrtps_cpp.hpp"
#include "autoware_manual_control/msg/detail/velocity_report__rosidl_typesupport_fastrtps_cpp.hpp"
#include \
  "autoware_manual_control/srv/detail/change_operation_mode__rosidl_typesupport_fastrtps_cpp.hpp"

#include <zenoh.hxx>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace autoware::manual_control
{

// Defined in this transport's parameter_reader.cpp, where Impl is complete.
std::unique_ptr<ParameterReader> make_zenoh_parameter_reader();

namespace msgs = autoware_manual_control::msg;
namespace srvs = autoware_manual_control::srv;

// The codec lives in a nested typesupport namespace (not reachable by ADL on the
// message type), so bring both msg and srv overload sets into scope.
using autoware_manual_control::msg::typesupport_fastrtps_cpp::cdr_deserialize;
using autoware_manual_control::msg::typesupport_fastrtps_cpp::cdr_serialize;
using autoware_manual_control::srv::typesupport_fastrtps_cpp::cdr_deserialize;
using autoware_manual_control::srv::typesupport_fastrtps_cpp::cdr_serialize;

// Logs `msg` the first time a given call site trips, then stays quiet so a
// persistent fault doesn't spam the control-rate loop.
inline void warn_once(std::atomic_flag & flag, const char * msg)
{
  if (!flag.test_and_set()) {
    std::fprintf(stderr, "[ZenohGateway] %s; will not warn again\n", msg);
  }
}

struct AutowareGateway::Impl
{
  std::shared_ptr<zenoh::Session> session;
  std::string scope;
  uint8_t operator_mode = msgs::OperationModeState::REMOTE;
  uint64_t service_timeout_ms = 500;

  std::unique_ptr<zenoh::Publisher> pub_control;
  std::unique_ptr<zenoh::Publisher> pub_gear;
  std::unique_ptr<zenoh::Publisher> pub_heartbeat;

  std::unique_ptr<zenoh::Subscriber<void>> sub_velocity;
  std::unique_ptr<zenoh::Subscriber<void>> sub_gear;
  std::unique_ptr<zenoh::Subscriber<void>> sub_steering;
  std::unique_ptr<zenoh::Subscriber<void>> sub_op_mode;
  std::unique_ptr<zenoh::Subscriber<void>> sub_loc_state;

  std::atomic<float> current_velocity{0.0f};
  std::atomic<float> current_steer{0.0f};
  std::atomic<uint8_t> current_gear{msgs::GearReport::PARK};
  std::atomic<uint8_t> current_op_mode{msgs::OperationModeState::UNKNOWN};
  std::atomic<bool> avail_drive{false};
  std::atomic<bool> control_enabled{false};
  std::atomic<bool> loc_initialized{false};

  std::mutex enable_mutex;
  std::chrono::steady_clock::time_point last_enable_request;
  std::atomic<bool> enable_in_flight{false};
  std::atomic<Gear> target_gear{Gear::PARK};

  mutable std::mutex info_mutex;
  std::string info_message;

  std::mutex service_mutex;
  std::condition_variable service_cv;
  std::deque<std::function<void()>> service_jobs;
  std::thread service_thread;

  std::atomic<bool> stop{false};
  std::thread heartbeat_thread;

  Impl() = default;
};

AutowareGateway::AutowareGateway()
: impl_(std::make_unique<Impl>())
{
  // The ctor reads its own params, before any external make_parameter_reader()
  // call, from the same startup-parsed map.
  auto reader = make_parameter_reader();
  impl_->scope = reader->read<std::string>("scope", std::string{"v1"});
  const std::string operator_mode_str = reader->read<std::string>(
    "operator_mode",
    std::string{"remote"});
  impl_->operator_mode = (operator_mode_str == "local") ?
    msgs::OperationModeState::LOCAL :
    msgs::OperationModeState::REMOTE;
  const float arrival_timeout_ms = reader->read<float>("arrival_timeout_ms", 500.0f);
  impl_->service_timeout_ms = static_cast<uint64_t>(arrival_timeout_ms);

  const std::string zenoh_cfg = reader->read<std::string>("zenoh_config", std::string{});
  zenoh::Config zconf = zenoh_cfg.empty() ?
    zenoh::Config::create_default() :
    zenoh::Config::from_file(zenoh_cfg.c_str());
  impl_->session = std::make_shared<zenoh::Session>(zenoh::Session::open(std::move(zconf)));

  auto key = [this](const std::string & ros_name) {
      return impl_->scope + ros_name;
    };

  impl_->pub_control = std::make_unique<zenoh::Publisher>(
    impl_->session->declare_publisher(zenoh::KeyExpr(key("/external/selected/control_cmd"))));
  impl_->pub_gear = std::make_unique<zenoh::Publisher>(
    impl_->session->declare_publisher(zenoh::KeyExpr(key("/external/selected/gear_cmd"))));
  impl_->pub_heartbeat = std::make_unique<zenoh::Publisher>(
    impl_->session->declare_publisher(
      zenoh::KeyExpr(
        key(
          impl_->operator_mode == msgs::OperationModeState::LOCAL ?
          "/external/local/heartbeat" :
          "/external/remote/heartbeat"))));

  auto put = [](zenoh::Publisher & pub, std::vector<uint8_t> bytes) {
      try {
        pub.put(std::move(bytes));
      } catch (const zenoh::ZException & e) {
        static std::atomic_flag warned = ATOMIC_FLAG_INIT;
        warn_once(warned, e.what());
      }
    };

  impl_->sub_velocity = std::make_unique<zenoh::Subscriber<void>>(
    impl_->session->declare_subscriber(
      zenoh::KeyExpr(key("/vehicle/status/velocity_status")),
      [this](zenoh::Sample & s) {
        try {
          auto bytes = s.get_payload().as_vector();
          cdr::CdrReader r(bytes);
          msgs::VelocityReport m;
          cdr_deserialize(r.cdr(), m);
          impl_->current_velocity.store(m.longitudinal_velocity);
        } catch (const std::exception & e) {
          static std::atomic_flag warned = ATOMIC_FLAG_INIT;
          warn_once(warned, e.what());
        }
      },
      []() {}));

  impl_->sub_gear = std::make_unique<zenoh::Subscriber<void>>(
    impl_->session->declare_subscriber(
      zenoh::KeyExpr(key("/vehicle/status/gear_status")),
      [this](zenoh::Sample & s) {
        try {
          auto bytes = s.get_payload().as_vector();
          cdr::CdrReader r(bytes);
          msgs::GearReport m;
          cdr_deserialize(r.cdr(), m);
          impl_->current_gear.store(m.report);
        } catch (const std::exception & e) {
          static std::atomic_flag warned = ATOMIC_FLAG_INIT;
          warn_once(warned, e.what());
        }
      },
      []() {}));

  impl_->sub_steering = std::make_unique<zenoh::Subscriber<void>>(
    impl_->session->declare_subscriber(
      zenoh::KeyExpr(key("/vehicle/status/steering_status")),
      [this](zenoh::Sample & s) {
        try {
          auto bytes = s.get_payload().as_vector();
          cdr::CdrReader r(bytes);
          msgs::SteeringReport m;
          cdr_deserialize(r.cdr(), m);
          impl_->current_steer.store(m.steering_tire_angle);
        } catch (const std::exception & e) {
          static std::atomic_flag warned = ATOMIC_FLAG_INIT;
          warn_once(warned, e.what());
        }
      },
      []() {}));

  impl_->sub_op_mode = std::make_unique<zenoh::Subscriber<void>>(
    impl_->session->declare_subscriber(
      zenoh::KeyExpr(key("/api/operation_mode/state")),
      [this](zenoh::Sample & s) {
        try {
          auto bytes = s.get_payload().as_vector();
          cdr::CdrReader r(bytes);
          msgs::OperationModeState m;
          cdr_deserialize(r.cdr(), m);

          impl_->current_op_mode.store(m.mode);
          impl_->avail_drive.store(
            impl_->operator_mode == msgs::OperationModeState::LOCAL ?
            m.is_local_mode_available :
            m.is_remote_mode_available);
          impl_->control_enabled.store(m.is_autoware_control_enabled);

          if (m.mode == impl_->operator_mode && !m.is_in_transition &&
          !m.is_autoware_control_enabled)
          {
            // Request autoware control enable
            auto t = std::chrono::steady_clock::now();
            bool should_trigger = false;
            {
              std::lock_guard<std::mutex> lock(impl_->enable_mutex);
              if (t - impl_->last_enable_request > std::chrono::seconds(1)) {
                impl_->last_enable_request = t;
                should_trigger = true;
              }
            }
            if (should_trigger && !impl_->enable_in_flight.exchange(true)) {
              {
                std::lock_guard<std::mutex> lock(impl_->service_mutex);
                impl_->service_jobs.push_back(
                  [this]() {
                    try {
                      srvs::ChangeOperationMode_Request req;
                      cdr::CdrWriter w;
                      cdr_serialize(req, w.cdr());

                      zenoh::Session::GetOptions opts;
                      opts.payload = zenoh::Bytes(w.bytes());
                      opts.timeout_ms = impl_->service_timeout_ms;
                      auto replies = impl_->session->get(
                        zenoh::KeyExpr(
                          impl_->scope +
                          "/api/operation_mode/enable_autoware_control"), "",
                        zenoh::channels::FifoChannel(16), std::move(opts));
                      for (auto res = replies.recv(); std::holds_alternative<zenoh::Reply>(res);
                      res = replies.recv())
                      {
                        // we just need one OK response
                        if (std::get<zenoh::Reply>(res).is_ok()) {break;}
                      }
                    } catch (const std::exception & e) {
                      static std::atomic_flag warned = ATOMIC_FLAG_INIT;
                      warn_once(warned, e.what());
                    }
                    impl_->enable_in_flight.store(false);
                  });
              }
              impl_->service_cv.notify_one();
            }
          }
        } catch (const std::exception & e) {
          static std::atomic_flag warned = ATOMIC_FLAG_INIT;
          warn_once(warned, e.what());
        }
      },
      []() {}));

  impl_->sub_loc_state = std::make_unique<zenoh::Subscriber<void>>(
    impl_->session->declare_subscriber(
      zenoh::KeyExpr(key("/api/localization/initialization_state")),
      [this](zenoh::Sample & s) {
        try {
          auto bytes = s.get_payload().as_vector();
          cdr::CdrReader r(bytes);
          msgs::LocalizationInitializationState m;
          cdr_deserialize(r.cdr(), m);
          impl_->loc_initialized.store(
            m.state == msgs::LocalizationInitializationState::INITIALIZED);
        } catch (const std::exception & e) {
          static std::atomic_flag warned = ATOMIC_FLAG_INIT;
          warn_once(warned, e.what());
        }
      },
      []() {}));

  impl_->service_thread = std::thread(
    [this]() {
      std::unique_lock<std::mutex> lock(impl_->service_mutex);
      while (!impl_->stop.load()) {
        impl_->service_cv.wait(
          lock, [this] {
            return impl_->stop.load() || !impl_->service_jobs.empty();
          });
        while (!impl_->service_jobs.empty()) {
          auto job = std::move(impl_->service_jobs.front());
          impl_->service_jobs.pop_front();
          lock.unlock();
          job();
          lock.lock();
        }
      }
    });

  impl_->heartbeat_thread = std::thread(
    [this, put, key]() {
      while (!impl_->stop.load()) {
        msgs::ManualOperatorHeartbeat msg;
        auto d = std::chrono::system_clock::now().time_since_epoch();
        auto sec = std::chrono::duration_cast<std::chrono::seconds>(d);
        auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(d - sec);
        msg.stamp.sec = static_cast<int32_t>(sec.count());
        msg.stamp.nanosec = static_cast<uint32_t>(ns.count());
        msg.ready = true;

        cdr::CdrWriter w;
        cdr_serialize(msg, w.cdr());
        put(*(impl_->pub_heartbeat), w.bytes());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    });
}

AutowareGateway::~AutowareGateway()
{
  impl_->stop.store(true);
  impl_->service_cv.notify_all();
  if (impl_->service_thread.joinable()) {impl_->service_thread.join();}
  if (impl_->heartbeat_thread.joinable()) {impl_->heartbeat_thread.join();}
}

void AutowareGateway::spin_some() {}

std::string AutowareGateway::operationModeName() const
{
  uint8_t mode = impl_->current_op_mode.load();
  switch (mode) {
    case msgs::OperationModeState::STOP:
      return "STOP";
    case msgs::OperationModeState::AUTONOMOUS:
      return "AUTONOMOUS";
    case msgs::OperationModeState::LOCAL:
      return "LOCAL";
    case msgs::OperationModeState::REMOTE:
      return "REMOTE";
    default:
      return "UNKNOWN";
  }
}

void AutowareGateway::toggle_operation_mode()
{
  uint8_t op_mode = impl_->operator_mode;
  uint8_t cur_mode = impl_->current_op_mode.load();

  uint8_t target_mode = (cur_mode == op_mode) ?
    static_cast<uint8_t>(msgs::OperationModeState::STOP) :
    op_mode;
  std::string name =
    (target_mode == msgs::OperationModeState::STOP) ? "STOP" : (op_mode ==
    msgs::OperationModeState::LOCAL ? "LOCAL" : "REMOTE");

  if (target_mode != msgs::OperationModeState::STOP && !impl_->avail_drive.load()) {
    std::lock_guard<std::mutex> lock(impl_->info_mutex);
    impl_->info_message = name + " mode unavailable";
    return;
  }

  std::lock_guard<std::mutex> lock(impl_->service_mutex);
  impl_->service_jobs.push_back(
    [this, target_mode, name]() {
      try {
        srvs::ChangeOperationMode_Request req;
        cdr::CdrWriter w;
        cdr_serialize(req, w.cdr());

        zenoh::Session::GetOptions opts;
        opts.payload = zenoh::Bytes(w.bytes());
        opts.timeout_ms = impl_->service_timeout_ms;

        std::string srv_suffix = (target_mode == msgs::OperationModeState::STOP) ?
        "/api/operation_mode/change_to_stop" :
        (impl_->operator_mode == msgs::OperationModeState::LOCAL ?
        "/api/operation_mode/change_to_local" :
        "/api/operation_mode/change_to_remote");

        auto replies = impl_->session->get(
          zenoh::KeyExpr(impl_->scope + srv_suffix), "",
          zenoh::channels::FifoChannel(16), std::move(opts));
        bool replied = false;
        for (auto res = replies.recv(); std::holds_alternative<zenoh::Reply>(res);
        res = replies.recv())
        {
          auto & rep = std::get<zenoh::Reply>(res);
          if (!rep.is_ok()) {continue;}
          try {
            auto bytes = rep.get_ok().get_payload().as_vector();
            cdr::CdrReader r(bytes);
            srvs::ChangeOperationMode_Response resp;
            cdr_deserialize(r.cdr(), resp);
            replied = true;
            std::lock_guard<std::mutex> info_lock(impl_->info_mutex);
            if (resp.status.success) {
              impl_->info_message = "Operation mode -> " + name;
            } else {
              impl_->info_message = "Operation mode " + name + " rejected: " + resp.status.message;
            }
            break;
          } catch (const std::exception & e) {
            static std::atomic_flag warned = ATOMIC_FLAG_INIT;
            warn_once(warned, e.what());
          }
        }
        if (!replied) {
          std::lock_guard<std::mutex> info_lock(impl_->info_mutex);
          impl_->info_message = "Operation mode " + name + " call timeout";
        }
      } catch (const std::exception & e) {
        static std::atomic_flag warned = ATOMIC_FLAG_INIT;
        warn_once(warned, e.what());
        std::lock_guard<std::mutex> info_lock(impl_->info_mutex);
        impl_->info_message = "Operation mode " + name + " call failed";
      }
    });
  impl_->service_cv.notify_one();
}

void AutowareGateway::reset_initial_pose()
{
  std::lock_guard<std::mutex> lock(impl_->info_mutex);
  impl_->info_message = "[ManualControl]: reset_initial_pose unavailable (native)";
}

VehicleState AutowareGateway::get_vehicle_state() const
{
  VehicleState s;
  s.velocity = impl_->current_velocity.load();
  s.steer_angle = impl_->current_steer.load();
  uint8_t report = impl_->current_gear.load();
  switch (report) {
    case msgs::GearReport::PARK:
      s.gear = Gear::PARK;
      break;
    case msgs::GearReport::REVERSE:
      s.gear = Gear::REVERSE;
      break;
    case msgs::GearReport::DRIVE:
      s.gear = Gear::DRIVE;
      break;
    case msgs::GearReport::LOW:
      s.gear = Gear::LOW;
      break;
    default:
      s.gear = Gear::NONE;
      break;
  }
  s.is_engaged = impl_->control_enabled.load();
  return s;
}

bool AutowareGateway::is_localized() const
{
  return impl_->loc_initialized.load();
}

void AutowareGateway::set_target_gear(Gear gear)
{
  impl_->target_gear.store(gear);
}

void AutowareGateway::publish_command(const ControlCommand & cmd)
{
  msgs::Control msg;
  auto d = std::chrono::system_clock::now().time_since_epoch();
  auto sec = std::chrono::duration_cast<std::chrono::seconds>(d);
  auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(d - sec);
  msg.stamp.sec = static_cast<int32_t>(sec.count());
  msg.stamp.nanosec = static_cast<uint32_t>(ns.count());
  msg.longitudinal.velocity = cmd.velocity;
  msg.longitudinal.acceleration = cmd.acceleration;
  msg.lateral.steering_tire_angle = cmd.steer_angle;

  cdr::CdrWriter w_ctrl;
  cdr_serialize(msg, w_ctrl.cdr());
  try {
    impl_->pub_control->put(w_ctrl.bytes());
  } catch (const zenoh::ZException & e) {
    static std::atomic_flag warned = ATOMIC_FLAG_INIT;
    warn_once(warned, e.what());
  }

  msgs::GearCommand gcmd;
  gcmd.stamp = msg.stamp;
  Gear target = impl_->target_gear.load();
  switch (target) {
    case Gear::PARK:
      gcmd.command = msgs::GearCommand::PARK;
      break;
    case Gear::REVERSE:
      gcmd.command = msgs::GearCommand::REVERSE;
      break;
    case Gear::DRIVE:
      gcmd.command = msgs::GearCommand::DRIVE;
      break;
    case Gear::LOW:
      gcmd.command = msgs::GearCommand::LOW;
      break;
    default:
      gcmd.command = msgs::GearCommand::NONE;
      break;
  }

  cdr::CdrWriter w_gear;
  cdr_serialize(gcmd, w_gear.cdr());
  try {
    impl_->pub_gear->put(w_gear.bytes());
  } catch (const zenoh::ZException & e) {
    static std::atomic_flag warned = ATOMIC_FLAG_INIT;
    warn_once(warned, e.what());
  }
}

std::string AutowareGateway::get_info_message() const
{
  std::lock_guard<std::mutex> lock(impl_->info_mutex);
  return impl_->info_message;
}

bool AutowareGateway::ok() const
{
  return !impl_->stop.load();
}

void AutowareGateway::init_system(int argc, char * argv[])
{
  zenoh_params::parse(argc, argv);
}
void AutowareGateway::shutdown_system() {}
bool AutowareGateway::ok_system() {return true;}

std::unique_ptr<ParameterReader> AutowareGateway::make_parameter_reader() const
{
  return make_zenoh_parameter_reader();
}

} // namespace autoware::manual_control
