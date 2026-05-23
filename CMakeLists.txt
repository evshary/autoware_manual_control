cmake_minimum_required(VERSION 3.8)
project(autoware_manual_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tier4_control_msgs)
find_package(tier4_external_api_msgs)
find_package(autoware_control_msgs)
find_package(autoware_vehicle_msgs)
find_package(autoware_adapi_v1_msgs)
find_package(geometry_msgs)

set(COMMON_ROS_DEPS rclcpp
                    tier4_control_msgs
                    tier4_external_api_msgs
                    autoware_control_msgs
                    autoware_vehicle_msgs
                    autoware_adapi_v1_msgs
                    geometry_msgs)

# keyboard_control: standalone, always built.
add_executable(keyboard_control src/keyboard_control.cpp src/core/drive_mode_factory.cpp)
target_include_directories(keyboard_control PRIVATE src)
ament_target_dependencies(keyboard_control ${COMMON_ROS_DEPS})

install(TARGETS keyboard_control DESTINATION lib/${PROJECT_NAME})

# remote_control: optional, built only when zenoh-cpp + nlohmann_json are found.
find_package(nlohmann_json QUIET)

set(ZENOH_VENDOR_PREFIX
    "$ENV{ZENOH_VENDOR_PREFIX}"
    CACHE PATH "Path to zenoh vendor from rmw_zenoh build")
if(ZENOH_VENDOR_PREFIX)
    find_package(zenohc   QUIET PATHS "${ZENOH_VENDOR_PREFIX}/lib/cmake" NO_DEFAULT_PATH)
    find_package(zenohcxx QUIET PATHS "${ZENOH_VENDOR_PREFIX}/lib/cmake" NO_DEFAULT_PATH)
endif()
if(NOT zenohc_FOUND)
    find_package(zenohc QUIET)
endif()
if(NOT zenohcxx_FOUND)
    find_package(zenohcxx QUIET)
endif()

if(zenohcxx_FOUND AND nlohmann_json_FOUND)
    message(STATUS "[autoware_manual_control] Building remote_control (zenoh-cpp found)")
    add_executable(remote_control src/remote_control.cpp src/core/drive_mode_factory.cpp)
    target_include_directories(remote_control PRIVATE src)
    ament_target_dependencies(remote_control ${COMMON_ROS_DEPS})
    target_link_libraries(remote_control
        nlohmann_json::nlohmann_json
        zenohcxx::zenohc)
    install(TARGETS remote_control DESTINATION lib/${PROJECT_NAME})
else()
    message(WARNING "[autoware_manual_control] Skipping remote_control — zenoh-cpp or nlohmann_json not found. "
                    "Set ZENOH_VENDOR_PREFIX env var or install zenohcxx system-wide.")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
