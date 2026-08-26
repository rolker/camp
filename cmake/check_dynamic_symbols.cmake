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

# find_program() leaves <VAR>-NOTFOUND when it finds nothing, and that string is
# what reaches us. Fail here rather than let CMakeLists.txt skip registering the
# test: a guard that silently does not run is worse than one that fails.
if(NOT DEFINED READELF OR READELF STREQUAL "" OR READELF MATCHES "NOTFOUND$")
  message(FATAL_ERROR
    "check_dynamic_symbols: neither readelf nor llvm-readelf was found, so the "
    "#217 ENABLE_EXPORTS guard cannot run. Install binutils (or llvm) on this "
    "host; do not silence this by dropping the test.")
endif()

# -W (wide): without it readelf elides long symbol names to
# `_ZN10camp_crash1[...]`, which is short enough to still match a substring
# check and long enough to hide which symbol it is.
execute_process(
  COMMAND "${READELF}" -W --dyn-syms "${EXECUTABLE}"
  OUTPUT_VARIABLE _dynsyms
  ERROR_VARIABLE _err
  RESULT_VARIABLE _rc
)

if(NOT _rc EQUAL 0)
  message(FATAL_ERROR "check_dynamic_symbols: readelf failed (${_rc}): ${_err}")
endif()

# A symbol DEFINED by CAMP's own sources, not merely referenced: proof that the
# executable's own symbols were exported.
#
# The "defined" half is load-bearing and was not always so. `crash_handler.cpp`
# used to be compiled into this executable, so grepping the dynamic symbol table
# for `camp_crash` proved something. Once the handler moved into libcamp_crash
# (#217 follow-up), that same grep matched the UNDEFINED import every dynamically
# linked executable carries — the check would have passed with ENABLE_EXPORTS
# deleted. So: match on `crash_log_path`, which is `src/camp/crash_log_path.cpp`
# and stays in the executable precisely because it needs rclcpp, and require at
# least one match whose section index is not UND.
#
# readelf --dyn-syms columns: Num: Value Size Type Bind Vis Ndx Name
set(_defined_match FALSE)
string(REPLACE "\n" ";" _dynsym_lines "${_dynsyms}")
foreach(_line IN LISTS _dynsym_lines)
  if(_line MATCHES "crash_log_path" AND NOT _line MATCHES "[ \t]UND[ \t]")
    set(_defined_match TRUE)
    break()
  endif()
endforeach()

if(NOT _defined_match)
  message(FATAL_ERROR
    "No DEFINED crash_log_path symbol in the dynamic symbol table of\n"
    "  ${EXECUTABLE}\n"
    "ENABLE_EXPORTS has been dropped from the CCOMAutonomousMissionPlanner "
    "target (CMakeLists.txt), or src/camp/crash_log_path.cpp is no longer "
    "linked into it. Crash backtraces (#217) will render as bare addresses "
    "with no function names.")
endif()

message(STATUS "check_dynamic_symbols: OK — CAMP symbols are exported")
