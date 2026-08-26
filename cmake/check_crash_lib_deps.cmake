# [#217] Boundary guard for libcamp_crash: no ROS, no Qt, no GDAL.
#
# The crash handler is a library so that camp_map's and camp_map_ros's worker
# threads can call install_thread_alt_stack() — which means camp_map links it,
# and camp_map is ROS-free by design (ADR-0002: "the src/camp_map core has zero
# ROS includes"). Anything this library links, camp_map links too. Resolving the
# ROS logging directory is kept out of it for exactly this reason
# (src/camp/crash_log_path.cpp, in the executable).
#
# That is a property of the linked artifact, not of anyone's intent, and the way
# it erodes is quiet: one #include and one ament_target_dependencies line added
# for a plausible reason, no test anywhere the wiser. So assert it against the
# built .so.
#
# Invoked as: cmake -DLIBRARY=<path> -DREADELF=<path> -P <this file>

if(NOT DEFINED LIBRARY OR NOT EXISTS "${LIBRARY}")
  message(FATAL_ERROR "check_crash_lib_deps: no library at '${LIBRARY}'")
endif()

if(NOT DEFINED READELF OR READELF STREQUAL "" OR READELF MATCHES "NOTFOUND$")
  message(FATAL_ERROR
    "check_crash_lib_deps: neither readelf nor llvm-readelf was found, so the "
    "#217 dependency-boundary guard cannot run. Install binutils (or llvm) on "
    "this host; do not silence this by dropping the test.")
endif()

# The DT_NEEDED entries — what this .so itself declares, not the full transitive
# closure ldd would print.
execute_process(
  COMMAND "${READELF}" -W -d "${LIBRARY}"
  OUTPUT_VARIABLE _dynamic
  ERROR_VARIABLE _err
  RESULT_VARIABLE _rc
)

if(NOT _rc EQUAL 0)
  message(FATAL_ERROR "check_crash_lib_deps: readelf failed (${_rc}): ${_err}")
endif()

set(_forbidden
  "libQt"        # Qt — camp_crash is called FROM Qt worker threads, not with Qt
  "librclcpp"    # ROS client library
  "librcl"       # rcl / rcutils / rcpputils
  "librmw"       # middleware
  "libgdal"      # GDAL
  "libmarine_"   # the workspace's own marine_* libraries
)

# One entry per line so each DT_NEEDED soname can be reported on its own.
string(REPLACE "\n" ";" _dynamic_lines "${_dynamic}")

set(_violations "")
foreach(_line IN LISTS _dynamic_lines)
  if(NOT _line MATCHES "NEEDED")
    continue()
  endif()
  string(REGEX MATCH "\\[([^]]*)\\]" _bracketed "${_line}")
  set(_soname "${CMAKE_MATCH_1}")
  if(_soname STREQUAL "")
    continue()
  endif()
  foreach(_lib IN LISTS _forbidden)
    if(_soname MATCHES "^${_lib}")
      list(APPEND _violations "  ${_soname}")
      break()
    endif()
  endforeach()
endforeach()

if(NOT _violations STREQUAL "")
  string(REPLACE ";" "\n" _report "${_violations}")
  message(FATAL_ERROR
    "[#217] libcamp_crash has acquired a dependency it must not have:\n"
    "${_report}\n\n"
    "  ${LIBRARY}\n\n"
    "camp_map links this library and is ROS-free by design (ADR-0002), so this "
    "dependency is now camp_map's too. Keep the mechanism (signals, alternate "
    "stacks, async-signal-safe emission) in camp_crash and put anything needing "
    "ROS, Qt or GDAL in the caller — the way src/camp/crash_log_path.cpp holds "
    "the one piece of #217 that needs rclcpp.")
endif()

message(STATUS "check_crash_lib_deps: OK — libcamp_crash links libc/libstdc++ only")
