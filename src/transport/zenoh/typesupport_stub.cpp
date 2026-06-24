// Inert definitions for the registration-tail symbols the rosidl fastrtps
// generator always emits (the rmw type-discovery path). The native_zenoh
// transport calls cdr_serialize / cdr_deserialize directly and never the
// registration path, so these only need to satisfy the linker -- providing
// them here keeps the binary free of librosidl_runtime / librmw.

#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rosidl_runtime_c/service_type_support_struct.h>

namespace rosidl_typesupport_fastrtps_cpp
{
const char * typesupport_identifier = "rosidl_typesupport_fastrtps_cpp";
} // namespace rosidl_typesupport_fastrtps_cpp

extern "C" {

const rosidl_message_type_support_t *
get_message_typesupport_handle_function(
  const rosidl_message_type_support_t * handle, const char *)
{
  return handle;
}

const rosidl_service_type_support_t *
get_service_typesupport_handle_function(
  const rosidl_service_type_support_t * handle, const char *)
{
  return handle;
}

} // extern "C"
