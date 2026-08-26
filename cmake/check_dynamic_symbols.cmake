# [#217] Regression guard for ENABLE_EXPORTS on the CAMP executable.
#
# backtrace_symbols_fd() resolves a frame to a function name only if that
# symbol is in the executable's DYNAMIC symbol table, which on ELF requires
# -rdynamic / --export-dynamic (CMake's ENABLE_EXPORTS target property).
# Without it every crash dump degrades to bare addresses — output that still
# looks plausible, which is exactly why this needs a test rather than a
# reviewer's memory.
#
# The gtest death tests in test/test_crash_handler.cpp cannot cover this: they
# assert against test_crash_handler, which sets ENABLE_EXPORTS on its own
# target. Only a check against the shipped binary catches the flag being
# dropped from CCOMAutonomousMissionPlanner.
#
# Invoked as: cmake -DEXECUTABLE=<path> -DREADELF=<path> -P <this file>

if(NOT DEFINED EXECUTABLE OR NOT EXISTS "${EXECUTABLE}")
  message(FATAL_ERROR "check_dynamic_symbols: no executable at '${EXECUTABLE}'")
endif()

if(NOT DEFINED READELF OR READELF STREQUAL "")
  message(FATAL_ERROR "check_dynamic_symbols: no readelf")
endif()

execute_process(
  COMMAND "${READELF}" --dyn-syms "${EXECUTABLE}"
  OUTPUT_VARIABLE _dynsyms
  ERROR_VARIABLE _err
  RESULT_VARIABLE _rc
)

if(NOT _rc EQUAL 0)
  message(FATAL_ERROR "check_dynamic_symbols: readelf failed (${_rc}): ${_err}")
endif()

# A symbol from CAMP's own sources, not from a linked library: proof that the
# executable's own symbols were exported.
if(NOT _dynsyms MATCHES "camp_crash")
  message(FATAL_ERROR
    "No camp_crash* symbol in the dynamic symbol table of\n"
    "  ${EXECUTABLE}\n"
    "ENABLE_EXPORTS has been dropped from the CCOMAutonomousMissionPlanner "
    "target (CMakeLists.txt). Crash backtraces (#217) will render as bare "
    "addresses with no function names.")
endif()

message(STATUS "check_dynamic_symbols: OK — CAMP symbols are exported")
