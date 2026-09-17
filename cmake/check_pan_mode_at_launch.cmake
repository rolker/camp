# [camp#22 round-11 should-fix] ADR-0016 D16's pan-mode arrow cursor must be in
# force at LAUNCH.
#
# CAMP idles in pan mode, so the launch state is the state the operator spends
# most of a session in. `ProjectView` starts in pan mode by its own bookkeeping
# (mouseMode = pan, the status bar reads "Mode: pan"), but the ARROW cursor is
# installed only by `ProjectView::setPanMode()`. Nothing called it at startup:
# `MainWindow` runs `setupUi()`, which applies the .ui's
# `dragMode = ScrollHandDrag` property, and Qt's `setDragMode()` puts the OPEN
# HAND on the viewport with it. The view therefore claimed pan mode while showing
# the cursor the 2026-09-15 operator GUI test rejected — ADR-0016 D15 records that
# nobody could land it inside 5 pixels with that hand, which is what hover-to-
# inspect and every mission-item placement are aimed with.
#
# WHY A SOURCE CHECK rather than a unit test: `ProjectView` is a widget of the
# CCOMAutonomousMissionPlanner executable and pulls in roslink / platform_manager /
# mission_manager / helm_manager, so constructing one in a gtest means standing up
# the ROS-linked half of the application. What can regress here is not behaviour
# inside a function but the ORDER of two calls in a constructor — exactly what a
# source check reads, and the same reasoning as check_worker_alt_stacks.
#
# The rule: mainwindow.cpp must contain a non-comment `setPanMode()` call, and it
# must come AFTER `setupUi(`. Before it, the .ui property is applied later and
# reinstalls the hand over the arrow — which is the bug, differently spelled.
#
# Invoked as: cmake -DSOURCE_DIR=<repo root> -P <this file>

if(NOT DEFINED SOURCE_DIR)
  message(FATAL_ERROR "check_pan_mode_at_launch: SOURCE_DIR is not set")
endif()

set(_main_window "${SOURCE_DIR}/src/camp/mainwindow.cpp")
if(NOT EXISTS "${_main_window}")
  message(FATAL_ERROR
    "check_pan_mode_at_launch: no '${_main_window}'. The guard cannot run; if the "
    "file moved, point this check at its new home rather than deleting the test.")
endif()

# Read as ONE string and split on newlines by hand. `file(STRINGS)` returns a
# CMake LIST, so every semicolon in the source — which is most lines of C++ —
# splits that line into further elements and drops empty lines, and the element
# index then bears no relation to the line number this check reports and orders
# by. Escaping the semicolons first keeps one element per line.
file(READ "${_main_window}" _content)
string(REPLACE ";" "\\;" _content "${_content}")
string(REPLACE "\n" ";" _lines "${_content}")

set(_line_number 0)
set(_setup_ui_line 0)
set(_set_pan_mode_line 0)
foreach(_line IN LISTS _lines)
  math(EXPR _line_number "${_line_number} + 1")
  # Skip comments: the explanatory prose at the call site names both symbols.
  string(REGEX MATCH "^[ \t]*(//|\\*|/\\*)" _is_comment "${_line}")
  if(NOT _is_comment STREQUAL "")
    continue()
  endif()
  if(_setup_ui_line EQUAL 0 AND _line MATCHES "setupUi[ \t]*\\(")
    set(_setup_ui_line ${_line_number})
  endif()
  if(_set_pan_mode_line EQUAL 0 AND _line MATCHES "setPanMode[ \t]*\\([ \t]*\\)")
    set(_set_pan_mode_line ${_line_number})
  endif()
endforeach()

if(_setup_ui_line EQUAL 0)
  message(FATAL_ERROR
    "check_pan_mode_at_launch: no setupUi() call found in mainwindow.cpp. The "
    "guard's premise no longer holds; re-derive it rather than silencing it.")
endif()

if(_set_pan_mode_line EQUAL 0)
  message(FATAL_ERROR
    "[camp#22 / ADR-0016 D16] MainWindow never calls ProjectView::setPanMode(), so "
    "CAMP launches showing ScrollHandDrag's OPEN HAND while reporting \"Mode: pan\". "
    "The hand has no visible hotspot — ADR-0016 D15 records the operator GUI test in "
    "which nobody could land it within 5 pixels — and pan mode is what CAMP idles in. "
    "Call m_ui->projectView->setPanMode() after m_ui->setupUi(this).")
endif()

if(_set_pan_mode_line LESS _setup_ui_line)
  message(FATAL_ERROR
    "[camp#22 / ADR-0016 D16] setPanMode() is called at mainwindow.cpp:"
    "${_set_pan_mode_line}, BEFORE setupUi() at line ${_setup_ui_line}. setupUi() "
    "applies the .ui's dragMode = ScrollHandDrag property, and Qt's setDragMode() "
    "reinstalls the open hand over the arrow — so the call has no effect where it "
    "stands. Move it after setupUi().")
endif()

message(STATUS
  "check_pan_mode_at_launch: OK — setPanMode() at mainwindow.cpp:"
  "${_set_pan_mode_line}, after setupUi() at line ${_setup_ui_line}")
