cmake_minimum_required(VERSION 3.8)
project(autoware_manual_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(TELEOP_WITH_KEYBOARD "Build keyboard_control" ON)
option(TELEOP_WITH_ZENOH "Build zenoh_control" OFF)
set(TELEOP_AUTOWARE_TRANSPORT "rclcpp" CACHE STRING "Autoware transport choice: rclcpp | native_zenoh")
set_property(CACHE TELEOP_AUTOWARE_TRANSPORT PROPERTY STRINGS rclcpp native_zenoh)

# ament_cmake is the colcon/ROS packaging layer. The native_zenoh transport
# builds as a plain CMake project (CMake + a C++ compiler + Fast-CDR, no ROS), so
# ament is optional -- and skipped when absent -- for it; every other path needs
# it.
find_package(ament_cmake QUIET)
if(NOT ament_cmake_FOUND AND NOT TELEOP_AUTOWARE_TRANSPORT STREQUAL "native_zenoh")
  message(FATAL_ERROR "ament_cmake is required for the ${TELEOP_AUTOWARE_TRANSPORT} transport")
endif()
if(NOT ament_cmake_FOUND)
  option(BUILD_TESTING "Build the tests" ON)
  enable_testing()
endif()

if(NOT TELEOP_WITH_KEYBOARD AND NOT TELEOP_WITH_ZENOH)
  message(WARNING "No entry point selected; set TELEOP_WITH_KEYBOARD or TELEOP_WITH_ZENOH")
endif()

if(TELEOP_AUTOWARE_TRANSPORT STREQUAL "rclcpp")
  find_package(rclcpp REQUIRED)
  find_package(autoware_control_msgs REQUIRED)
  find_package(autoware_vehicle_msgs REQUIRED)
  find_package(autoware_adapi_v1_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)

  set(COMMON_ROS_DEPS rclcpp autoware_control_msgs autoware_vehicle_msgs
                      autoware_adapi_v1_msgs geometry_msgs)

  set(TELEOP_GATEWAY_SRC
    src/transport/rclcpp/autoware_gateway.cpp
    src/transport/rclcpp/parameter_reader.cpp)

  if(TELEOP_WITH_KEYBOARD)
    add_executable(keyboard_control
      src/keyboard_control.cpp src/core/drive_mode_factory.cpp ${TELEOP_GATEWAY_SRC})
    target_include_directories(keyboard_control PRIVATE src)
    ament_target_dependencies(keyboard_control ${COMMON_ROS_DEPS})
    install(TARGETS keyboard_control DESTINATION lib/${PROJECT_NAME})
  endif()

  if(TELEOP_WITH_ZENOH)
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
    add_executable(zenoh_control
      src/zenoh_control.cpp src/core/drive_mode_factory.cpp ${TELEOP_GATEWAY_SRC})
    target_include_directories(zenoh_control PRIVATE src)
    ament_target_dependencies(zenoh_control ${COMMON_ROS_DEPS})
    target_link_libraries(zenoh_control nlohmann_json::nlohmann_json zenohcxx::zenohc)
    install(TARGETS zenoh_control DESTINATION lib/${PROJECT_NAME})
  endif()
endif()

# native_zenoh transport: compile the committed rosidl_typesupport_fastrtps_cpp codec
# under src/transport/zenoh/generated straight into the binary (no ROS, no message
# packages -- only zenoh + static Fast-CDR). Regenerate with tools/vendor_idl.sh after
# an Autoware message bump; the CI golden-drift check flags when that is due.
if(TELEOP_AUTOWARE_TRANSPORT STREQUAL "native_zenoh")
  set(_gen_dir ${CMAKE_CURRENT_SOURCE_DIR}/src/transport/zenoh/generated)

  # Fast-CDR: pinned, static, PIC, no tests; folds into the target (no
  # libfastcdr.so). BUILD_TESTING is forced off across makeavailable so Fast-CDR
  # skips its own tests, then restored for this project's tests.
  include(FetchContent)
  set(BUILD_SHARED_LIBS OFF)
  set(BUILD_TESTING OFF)
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)
  fetchcontent_declare(fastcdr
    GIT_REPOSITORY https://github.com/eProsima/Fast-CDR
    GIT_TAG v1.1.1)
  fetchcontent_makeavailable(fastcdr)
  unset(BUILD_SHARED_LIBS)
  unset(BUILD_TESTING)
  unset(CMAKE_POSITION_INDEPENDENT_CODE)

  # The shipped binary carries exactly these production codecs -- the wire contract,
  # kept in step with TELEOP_MSG_IDL in tools/regen/CMakeLists.txt. An allow-list, not
  # a denylist: a stem missing here routes its codec to test-only and the binary fails
  # to link, so a drop is never silent; any other vendored codec (the test_msgs shape
  # corpus) goes to test_cdr.
  set(TELEOP_PROD_MSG_STEMS
      time header point quaternion pose pose_with_covariance
      pose_with_covariance_stamped lateral longitudinal control gear_command
      gear_report steering_report velocity_report
      localization_initialization_state manual_operator_heartbeat
      operation_mode_state response_status change_operation_mode)
  file(GLOB_RECURSE _codec_all CONFIGURE_DEPENDS
       ${_gen_dir}/autoware_manual_control/*/detail/dds_fastrtps/*__type_support.cpp)
  set(TELEOP_CODEC_SRC "")
  set(TELEOP_TEST_CODEC_SRC "")
  foreach(_c ${_codec_all})
    get_filename_component(_stem ${_c} NAME)
    string(REGEX REPLACE "__type_support\\.cpp$" "" _stem "${_stem}")
    if(_stem IN_LIST TELEOP_PROD_MSG_STEMS)
      list(APPEND TELEOP_CODEC_SRC ${_c})
    else()
      list(APPEND TELEOP_TEST_CODEC_SRC ${_c})
    endif()
  endforeach()

  # The codec is machine-generated and compiled into our targets; keep the strict
  # first-party warnings off it at the source level.
  set_source_files_properties(${TELEOP_CODEC_SRC} ${TELEOP_TEST_CODEC_SRC}
      PROPERTIES COMPILE_OPTIONS "-w")

  set(TELEOP_CODEC_INCLUDE ${_gen_dir})

  set(TELEOP_GATEWAY_SRC
    src/transport/zenoh/autoware_gateway.cpp
    src/transport/zenoh/parameter_reader.cpp
    src/transport/zenoh/typesupport_stub.cpp
    ${TELEOP_CODEC_SRC})

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

  # The native ParameterReader parses params-files with libyaml (the engine ROS
  # itself uses). Link the static archive so no libyaml.so joins the runtime
  # dependencies -- the binary still needs only zenoh + static Fast-CDR.
  find_library(YAML_STATIC_LIB NAMES libyaml.a REQUIRED)
  find_path(YAML_INCLUDE_DIR NAMES yaml.h REQUIRED)

  if(TELEOP_WITH_KEYBOARD)
    add_executable(keyboard_control
      src/keyboard_control.cpp src/core/drive_mode_factory.cpp ${TELEOP_GATEWAY_SRC})
    target_include_directories(keyboard_control PRIVATE src ${TELEOP_CODEC_INCLUDE} ${YAML_INCLUDE_DIR})
    target_link_libraries(keyboard_control zenohcxx::zenohc fastcdr ${YAML_STATIC_LIB})
    install(TARGETS keyboard_control DESTINATION lib/${PROJECT_NAME})
  endif()

  if(TELEOP_WITH_ZENOH)
    add_executable(zenoh_control
      src/zenoh_control.cpp src/core/drive_mode_factory.cpp ${TELEOP_GATEWAY_SRC})
    target_include_directories(zenoh_control PRIVATE src ${TELEOP_CODEC_INCLUDE} ${YAML_INCLUDE_DIR})
    target_link_libraries(zenoh_control nlohmann_json::nlohmann_json zenohcxx::zenohc fastcdr ${YAML_STATIC_LIB})
    install(TARGETS zenoh_control DESTINATION lib/${PROJECT_NAME})
  endif()
endif()

# Package the config templates so the release image (which copies only install/)
# can read them from run_teleop.sh.
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  if(ament_cmake_FOUND)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
  endif()

  # Byte-perfect CDR gate for the native_zenoh transport. Links the production
  # codec plus the test_msgs shape-coverage codec; the shipped binary carries
  # neither test corpus. ament_add_gtest under colcon, plain gtest standalone.
  if(TELEOP_AUTOWARE_TRANSPORT STREQUAL "native_zenoh")
    set(_test_cdr_src test/test_cdr.cpp
      src/transport/zenoh/typesupport_stub.cpp
      ${TELEOP_CODEC_SRC} ${TELEOP_TEST_CODEC_SRC})
    if(ament_cmake_FOUND)
      find_package(ament_cmake_gtest REQUIRED)
      ament_add_gtest(test_cdr ${_test_cdr_src})
      ament_add_gtest(test_param_reader test/test_param_reader.cpp
        src/transport/zenoh/parameter_reader.cpp)
    else()
      find_package(GTest REQUIRED)
      add_executable(test_cdr ${_test_cdr_src})
      add_test(NAME test_cdr COMMAND test_cdr)
      target_link_libraries(test_cdr GTest::gtest GTest::gtest_main)
      add_executable(test_param_reader test/test_param_reader.cpp
        src/transport/zenoh/parameter_reader.cpp)
      add_test(NAME test_param_reader COMMAND test_param_reader)
      target_link_libraries(test_param_reader GTest::gtest GTest::gtest_main)
    endif()
    target_include_directories(test_cdr PRIVATE src ${TELEOP_CODEC_INCLUDE})
    target_link_libraries(test_cdr fastcdr)
    target_include_directories(test_param_reader PRIVATE src ${YAML_INCLUDE_DIR})
    target_link_libraries(test_param_reader ${YAML_STATIC_LIB})
  endif()
endif()

if(ament_cmake_FOUND)
  ament_package()
endif()
