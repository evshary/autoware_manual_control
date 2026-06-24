cmake_minimum_required(VERSION 3.8)
project(autoware_manual_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(TELEOP_WITH_KEYBOARD "Build keyboard_control" ON)
option(TELEOP_WITH_ZENOH "Build zenoh_control" OFF)
set(TELEOP_AUTOWARE_TRANSPORT "rclcpp" CACHE STRING "Autoware transport choice: rclcpp | native_zenoh")
set_property(CACHE TELEOP_AUTOWARE_TRANSPORT PROPERTY STRINGS rclcpp native_zenoh)

find_package(ament_cmake REQUIRED)

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

# native_zenoh transport: the codec is rosidl's own typesupport_fastrtps_cpp,
# regenerated from the installed message .idl under this project's namespace and
# compiled into the executable. The wire bytes are byte-perfect by construction
# (the same functions rmw_fastrtps calls); the prebuilt rosidl/message .so are
# never linked, so the binary needs only zenoh + Fast-CDR (static).
if(TELEOP_AUTOWARE_TRANSPORT STREQUAL "native_zenoh")
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  # find_package needs the ament prefixes on CMAKE_PREFIX_PATH; some setup.bash
  # (Autoware's) export AMENT_PREFIX_PATH only, so mirror it here.
  if(DEFINED ENV{AMENT_PREFIX_PATH})
    string(REPLACE ":" ";" _ament_prefixes "$ENV{AMENT_PREFIX_PATH}")
    list(APPEND CMAKE_PREFIX_PATH ${_ament_prefixes})
  endif()

  # Fast-CDR: pinned, static, PIC, no tests; folds into the target (no
  # libfastcdr.so). Declared BEFORE the rosidl find_package calls, which import a
  # SHARED fastcdr target of the same name -- the static one must win first. The
  # overrides are unset afterwards so a later option() does not warn.
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

  find_package(rosidl_generator_cpp REQUIRED)
  find_package(rosidl_typesupport_fastrtps_cpp REQUIRED)
  find_package(rosidl_runtime_c REQUIRED)
  find_package(rosidl_runtime_cpp REQUIRED)
  find_package(rosidl_typesupport_interface REQUIRED)

  # The msg packages whose .idl are this transport's wire contract. WHICH files
  # (intent) lives here; the CONTENT comes from the installed package share dirs.
  set(TELEOP_MSG_IDL
    builtin_interfaces/msg/Time.idl
    std_msgs/msg/Header.idl
    geometry_msgs/msg/Point.idl
    geometry_msgs/msg/Quaternion.idl
    geometry_msgs/msg/Pose.idl
    geometry_msgs/msg/PoseWithCovariance.idl
    geometry_msgs/msg/PoseWithCovarianceStamped.idl
    autoware_control_msgs/msg/Lateral.idl
    autoware_control_msgs/msg/Longitudinal.idl
    autoware_control_msgs/msg/Control.idl
    autoware_vehicle_msgs/msg/GearCommand.idl
    autoware_vehicle_msgs/msg/GearReport.idl
    autoware_vehicle_msgs/msg/SteeringReport.idl
    autoware_vehicle_msgs/msg/VelocityReport.idl
    autoware_adapi_v1_msgs/msg/LocalizationInitializationState.idl
    autoware_adapi_v1_msgs/msg/ManualOperatorHeartbeat.idl
    autoware_adapi_v1_msgs/msg/OperationModeState.idl
    autoware_adapi_v1_msgs/msg/ResponseStatus.idl
    autoware_adapi_v1_msgs/srv/ChangeOperationMode.idl)

  set(TELEOP_ALL_IDL ${TELEOP_MSG_IDL})

  # rosidl namespaces the generated types after the consuming project, so the
  # module declared inside each .idl must match it. Stage the installed .idl
  # (content = the deployed source of truth, no vendored copy) into the build
  # tree under this project's namespace, then feed those to the generator.
  set(_share_args "")
  foreach(rel ${TELEOP_ALL_IDL})
    string(REGEX REPLACE "/.*" "" pkg "${rel}")
    if(NOT ${pkg}_FOUND)
      find_package(${pkg} REQUIRED)
    endif()
    get_filename_component(_share "${${pkg}_DIR}/.." ABSOLUTE)
    list(APPEND _share_args "${pkg}=${_share}")
  endforeach()
  list(REMOVE_DUPLICATES _share_args)

  set(_stage_dir "${CMAKE_CURRENT_BINARY_DIR}/idl")
  execute_process(
    COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/tools/stage_idl.py"
            ${PROJECT_NAME} "${_stage_dir}" ${_share_args} -- ${TELEOP_ALL_IDL}
    OUTPUT_VARIABLE _staged_idl RESULT_VARIABLE _stage_rc)
  if(NOT _stage_rc EQUAL 0)
    message(FATAL_ERROR "stage_idl.py failed")
  endif()

  # Map each staged .idl to its interface tuple and to the codec .cpp the
  # generator emits (deterministic: snake_case of the stem under detail/).
  set(_idl_tuples "")
  set(TELEOP_CODEC_SRC "")
  set(_codec_dir "${CMAKE_CURRENT_BINARY_DIR}/rosidl_typesupport_fastrtps_cpp/${PROJECT_NAME}")
  foreach(s ${_staged_idl})
    list(APPEND _idl_tuples "${_stage_dir}:${s}")
    get_filename_component(_kind "${s}" DIRECTORY)
    get_filename_component(_stem "${s}" NAME_WE)
    # Mirror rosidl's convert_camel_case_to_lower_case_underscore (two steps),
    # so acronym names (e.g. HTTPResponse -> http_response) match the emitted file.
    string(REGEX REPLACE "(.)([A-Z][a-z]+)" "\\1_\\2" _snake "${_stem}")
    string(REGEX REPLACE "([a-z0-9])([A-Z])" "\\1_\\2" _snake "${_snake}")
    string(TOLOWER "${_snake}" _snake)
    list(APPEND TELEOP_CODEC_SRC
      "${_codec_dir}/${_kind}/detail/dds_fastrtps/${_snake}__type_support.cpp")
  endforeach()

  rosidl_generate_interfaces(${PROJECT_NAME} ${_idl_tuples}
    SKIP_INSTALL SKIP_GROUP_MEMBERSHIP_CHECK)

  # The codec .cpp are compiled into the executable directly; the rosidl-built
  # library (which PUBLIC-links rmw/rosidl_runtime) stays out of the build.
  set_target_properties(${PROJECT_NAME}__rosidl_typesupport_fastrtps_cpp
                        PROPERTIES EXCLUDE_FROM_ALL TRUE)

  set(TELEOP_CODEC_INCLUDE
    "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp"
    "${CMAKE_CURRENT_BINARY_DIR}/rosidl_typesupport_fastrtps_cpp"
    ${rosidl_runtime_c_INCLUDE_DIRS}
    ${rosidl_runtime_cpp_INCLUDE_DIRS}
    ${rosidl_typesupport_interface_INCLUDE_DIRS}
    ${rosidl_typesupport_fastrtps_cpp_INCLUDE_DIRS})

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
    add_dependencies(keyboard_control ${PROJECT_NAME})
    target_include_directories(keyboard_control PRIVATE src ${TELEOP_CODEC_INCLUDE} ${YAML_INCLUDE_DIR})
    target_link_libraries(keyboard_control zenohcxx::zenohc fastcdr ${YAML_STATIC_LIB})
    install(TARGETS keyboard_control DESTINATION lib/${PROJECT_NAME})
  endif()

  if(TELEOP_WITH_ZENOH)
    add_executable(zenoh_control
      src/zenoh_control.cpp src/core/drive_mode_factory.cpp ${TELEOP_GATEWAY_SRC})
    add_dependencies(zenoh_control ${PROJECT_NAME})
    target_include_directories(zenoh_control PRIVATE src ${TELEOP_CODEC_INCLUDE} ${YAML_INCLUDE_DIR})
    target_link_libraries(zenoh_control nlohmann_json::nlohmann_json zenohcxx::zenohc fastcdr ${YAML_STATIC_LIB})
    install(TARGETS zenoh_control DESTINATION lib/${PROJECT_NAME})
  endif()
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
