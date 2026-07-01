#!/usr/bin/env bash
# Regenerate the vendored native_zenoh codec under src/transport/zenoh/generated.
#
# Run this in a ROS 2 + Autoware environment whenever the upstream message
# definitions change (the golden drift guard in CI flags when that happens).
# It builds the rosidl generators from tools/regen, copies their C++ output, and
# vendors the rosidl runtime headers those sources include -- so the normal build
# needs only CMake + a C++ compiler + Fast-CDR, no ROS.
#
#   source /opt/ros/humble/setup.bash   # or /opt/autoware/setup.bash
#   tools/vendor_idl.sh
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
GEN="${REPO}/src/transport/zenoh/generated"
WORK="$(mktemp -d)"
trap 'rm -rf "${WORK}"' EXIT

if [ -z "${AMENT_PREFIX_PATH:-}" ]; then
  echo "error: source a ROS 2 + Autoware environment first" >&2
  exit 1
fi

echo "[1/4] generating codec with rosidl"
colcon --log-base "${WORK}/log" build --paths "${REPO}/tools/regen" \
  --build-base "${WORK}/build" --install-base "${WORK}/install" >/dev/null

BD="${WORK}/build/autoware_manual_control"
GC="${BD}/rosidl_generator_cpp/autoware_manual_control"
FR="${BD}/rosidl_typesupport_fastrtps_cpp/autoware_manual_control"

echo "[2/4] copying generated package tree"
rm -rf "${GEN}"
mkdir -p "${GEN}/autoware_manual_control"
cp -a "${GC}/." "${GEN}/autoware_manual_control/"
cp -a "${FR}/." "${GEN}/autoware_manual_control/"

echo "[3/4] vendoring the rosidl runtime headers the codec includes"
INCS="-I ${BD}/rosidl_generator_cpp -I ${BD}/rosidl_typesupport_fastrtps_cpp"
for p in rosidl_runtime_c rosidl_runtime_cpp rosidl_typesupport_interface \
         rosidl_typesupport_fastrtps_cpp rosidl_typesupport_cpp rmw rcutils; do
  d="$(ros2 pkg prefix "${p}" 2>/dev/null || echo /opt/ros/humble)"
  INCS="${INCS} -I ${d}/include -I ${d}/include/${p}"
done
FCDR="$(dirname "$(find /opt /usr -path '*/fastcdr/Cdr.h' 2>/dev/null | head -1)")"
if [ "${FCDR}" = "." ]; then
  echo "error: Fast-CDR headers (fastcdr/Cdr.h) not found under /opt or /usr" >&2
  exit 1
fi
INCS="${INCS} -I $(dirname "${FCDR}")"

# A synthetic translation unit including every generated codec source and message
# header; -M reports the full include closure, from which the external rosidl
# runtime headers are vendored.
SYN="${WORK}/syn.cpp"
: > "${SYN}"
find "${FR}" -name '*__type_support.cpp' -printf '#include "%p"\n' >> "${SYN}"
( cd "${BD}/rosidl_generator_cpp" && \
  find autoware_manual_control/msg autoware_manual_control/srv -maxdepth 1 -name '*.hpp' \
       -printf '#include "%p"\n' ) >> "${SYN}"

g++ -std=c++17 -M ${INCS} "${SYN}" \
  | tr ' \\' '\n' | grep '^/' | sort -u \
  | grep -E '/include/(rosidl_runtime_c|rosidl_runtime_cpp|rosidl_typesupport_cpp|rosidl_typesupport_interface|rosidl_typesupport_fastrtps_cpp|rmw|rcutils)/' \
  > "${WORK}/runtime_headers.txt"

while read -r h; do
  rel="$(echo "${h}" | sed -E 's#.*/include/[^/]+/##')"
  mkdir -p "${GEN}/$(dirname "${rel}")"
  cp "${h}" "${GEN}/${rel}"
done < "${WORK}/runtime_headers.txt"

echo "[4/4] done"
echo "  package sources : $(find "${GEN}/autoware_manual_control" -type f | wc -l)"
echo "  runtime headers : $(wc -l < "${WORK}/runtime_headers.txt")"
echo "  -> ${GEN}"
