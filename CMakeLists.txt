cmake_minimum_required(VERSION 3.8)
project(autoware_manual_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(TELEOP_WITH_KEYBOARD "Build keyboard_control" ON)
option(TELEOP_WITH_ZENOH "Build zenoh_control" OFF)
set(TELEOP_AUTOWARE_TRANSPORT "rclcpp" CACHE STRING "Autoware transport choice: rclcpp | native_zenoh")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_control_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(COMMON_ROS_DEPS rclcpp autoware_control_msgs autoware_vehicle_msgs
                    autoware_adapi_v1_msgs geometry_msgs)

# Handle the transport layer source selection
if(TELEOP_AUTOWARE_TRANSPORT STREQUAL "rclcpp")
  set(TELEOP_GATEWAY_SRC
    src/transport/rclcpp/autoware_gateway.cpp
    src/transport/rclcpp/parameter_reader.cpp
  )
elseif(TELEOP_AUTOWARE_TRANSPORT STREQUAL "native_zenoh")
  set(TELEOP_GATEWAY_SRC
    src/transport/zenoh/autoware_gateway.cpp
    src/transport/zenoh/parameter_reader.cpp
    src/transport/zenoh/typesupport_linker_shim.cpp
  )
else()
  message(FATAL_ERROR "Unknown TELEOP_AUTOWARE_TRANSPORT: ${TELEOP_AUTOWARE_TRANSPORT}")
endif()

# We always need zenoh find_package if building zenoh_control OR using native_zenoh transport
if(TELEOP_WITH_ZENOH OR TELEOP_AUTOWARE_TRANSPORT STREQUAL "native_zenoh")
  find_package(nlohmann_json REQUIRED)
  set(ZENOH_VENDOR_PREFIX "$ENV{ZENOH_VENDOR_PREFIX}"
      CACHE PATH "Path to zenoh vendor from rmw_zenoh build")
  if(ZENOH_VENDOR_PREFIX)
    find_package(zenohc REQUIRED PATHS "${ZENOH_VENDOR_PREFIX}/lib/cmake" NO_DEFAULT_PATH)
    find_package(zenohcxx REQUIRED PATHS "${ZENOH_VENDOR_PREFIX}/lib/cmake" NO_DEFAULT_PATH)
  else()
    find_package(zenohc REQUIRED)
    find_package(zenohcxx REQUIRED)
  endif()
endif()

if(TELEOP_WITH_KEYBOARD)
  add_executable(keyboard_control
    src/keyboard_control.cpp
    src/core/drive_mode_factory.cpp
    ${TELEOP_GATEWAY_SRC}
  )
  target_include_directories(keyboard_control PRIVATE src)
  ament_target_dependencies(keyboard_control ${COMMON_ROS_DEPS})
  if(TELEOP_AUTOWARE_TRANSPORT STREQUAL "native_zenoh")
    target_link_libraries(keyboard_control zenohcxx::zenohc)
  endif()
  install(TARGETS keyboard_control DESTINATION lib/${PROJECT_NAME})
endif()

if(TELEOP_WITH_ZENOH)
  add_executable(zenoh_control
    src/zenoh_control.cpp
    src/core/drive_mode_factory.cpp
    ${TELEOP_GATEWAY_SRC}
  )
  target_include_directories(zenoh_control PRIVATE src)
  ament_target_dependencies(zenoh_control ${COMMON_ROS_DEPS})
  target_link_libraries(zenoh_control nlohmann_json::nlohmann_json zenohcxx::zenohc)
  install(TARGETS zenoh_control DESTINATION lib/${PROJECT_NAME})
endif()

if(NOT TELEOP_WITH_KEYBOARD AND NOT TELEOP_WITH_ZENOH)
  message(WARNING "No entry point selected; set TELEOP_WITH_KEYBOARD or TELEOP_WITH_ZENOH")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
