# [#217] Coverage guard for the per-thread alternate signal stacks.
#
# A thread without an alternate signal stack cannot report a stack-overflow
# SIGSEGV: the kernel has no room left on the faulting stack to push a handler
# frame, so that crash class dies as silently as every crash did before #217.
# `sigaltstack(2)` is a per-thread attribute that neither `pthread_create(3)`
# nor `QThread` inherits, so every thread CAMP starts has to call
# `camp_crash::install_thread_alt_stack()` on entry itself.
#
# Nothing about writing a new `QtConcurrent::run(...)` call, or a new
# `QThread::run()` override, announces that requirement — and the failure is
# invisible: everything still builds, every test still passes, and the only
# symptom is a crash that is never reported, on a boat, months later. Hence a
# test rather than a convention.
#
# The rule, deliberately coarse and fail-closed: in any source file that starts
# worker threads, the number of non-comment
# `camp_crash::install_thread_alt_stack()` calls must be at least the number of
# thread entry points that file creates. It does not attempt to resolve which
# function each `QtConcurrent::run()` dispatches to — CMake script is the wrong
# tool for that, and a check that guesses wrong in the permissive direction is
# worse than useless. A worker whose body lives in another file trips this; the
# fix is to add the call and, if the arithmetic still does not hold, to say so
# in a comment on the site rather than to loosen the check.
#
# Invoked as: cmake -DSOURCE_DIR=<repo root> -P <this file>

if(NOT DEFINED SOURCE_DIR OR NOT IS_DIRECTORY "${SOURCE_DIR}/src")
  message(FATAL_ERROR "check_worker_alt_stacks: no source tree at '${SOURCE_DIR}/src'")
endif()

file(GLOB_RECURSE _sources "${SOURCE_DIR}/src/*.cpp")
list(LENGTH _sources _source_count)
if(_source_count EQUAL 0)
  message(FATAL_ERROR
    "check_worker_alt_stacks: found no sources under '${SOURCE_DIR}/src'. The "
    "guard cannot run; do not silence this by deleting the test.")
endif()

set(_failures "")
set(_guarded_files 0)

foreach(_file IN LISTS _sources)
  # The handler's own translation unit defines the function; it is not a caller.
  if(_file MATCHES "/camp_crash/")
    continue()
  endif()

  file(STRINGS "${_file}" _lines)

  set(_entry_points 0)
  set(_installs 0)
  foreach(_line IN LISTS _lines)
    # Skip comments: this file's own explanatory prose names both symbols, and a
    # commented-out call must not count as coverage.
    string(REGEX MATCH "^[ \t]*(//|\\*|/\\*)" _is_comment "${_line}")
    if(NOT _is_comment STREQUAL "")
      continue()
    endif()
    if(_line MATCHES "QtConcurrent::run[ \t]*\\(")
      math(EXPR _entry_points "${_entry_points} + 1")
    endif()
    # A QThread subclass's entry point. Matches the definition, not the
    # declaration in the header.
    #
    # Known blind spot, stated rather than left to be discovered: only .cpp
    # files are globbed, so a QThread subclass whose run() is defined INLINE in
    # its header is not seen by this check. Nothing in CAMP does that today.
    # Widening the glob to headers would also match every declaration, so the
    # fix if one ever appears is to define run() in a .cpp — which is the house
    # style anyway — not to loosen the pattern.
    if(_line MATCHES "^[A-Za-z_].*::run[ \t]*\\([ \t]*\\)")
      math(EXPR _entry_points "${_entry_points} + 1")
    endif()
    if(_line MATCHES "install_thread_alt_stack[ \t]*\\([ \t]*\\)")
      math(EXPR _installs "${_installs} + 1")
    endif()
  endforeach()

  if(_entry_points GREATER 0)
    if(_installs LESS _entry_points)
      file(RELATIVE_PATH _rel "${SOURCE_DIR}" "${_file}")
      list(APPEND _failures
        "  ${_rel}: ${_entry_points} thread entry point(s), ${_installs} install_thread_alt_stack() call(s)")
    else()
      math(EXPR _guarded_files "${_guarded_files} + 1")
    endif()
  endif()
endforeach()

if(NOT _failures STREQUAL "")
  string(REPLACE ";" "\n" _report "${_failures}")
  message(FATAL_ERROR
    "[#217] Worker thread(s) started without an alternate signal stack:\n"
    "${_report}\n\n"
    "Call camp_crash::install_thread_alt_stack() as the FIRST statement of each "
    "worker entry point (include \"crash_handler.h\"). It is idempotent and "
    "costs a thread_local pointer test. Without it, a stack-overflow SIGSEGV on "
    "that thread is not reported at all — the crash class #215 is suspected of.")
endif()

message(STATUS
  "check_worker_alt_stacks: OK — ${_guarded_files} file(s) starting worker "
  "threads, all guarded")
