# [camp#242] ProjectView's constructor must call camp::keepCenterOnResize(this),
# which sets QGraphicsView::AnchorViewCenter. Without it a resize after the startup
# view restore moves the map centre by half the size change. ProjectView cannot be
# constructed in a gtest (see check_pan_mode_at_launch), so this is a source check;
# the behaviour itself is covered by test_view_resize_anchor.
#
# Invoked as: cmake -DSOURCE_DIR=<repo root> -P <this file>

if(NOT DEFINED SOURCE_DIR)
  message(FATAL_ERROR "check_project_view_resize_anchor: SOURCE_DIR is not set")
endif()

set(_src "${SOURCE_DIR}/src/camp/projectview.cpp")
set(_hdr "${SOURCE_DIR}/src/camp/view_resize_anchor.h")
foreach(_f "${_src}" "${_hdr}")
  if(NOT EXISTS "${_f}")
    message(FATAL_ERROR
      "check_project_view_resize_anchor: no '${_f}'. If the file moved, point this "
      "check at its new home rather than deleting the test.")
  endif()
endforeach()

file(STRINGS "${_src}" _calls REGEX "^[ \t]*camp::keepCenterOnResize[ \t]*\\([ \t]*this[ \t]*\\)")
if(NOT _calls)
  message(FATAL_ERROR
    "[camp#242] ProjectView's constructor no longer calls camp::keepCenterOnResize(this). "
    "A window resize after the startup restore will shift the map centre.")
endif()

file(STRINGS "${_hdr}" _anchor REGEX "^[ \t]*view->setResizeAnchor[ \t]*\\([ \t]*QGraphicsView::AnchorViewCenter")
if(NOT _anchor)
  message(FATAL_ERROR
    "[camp#242] keepCenterOnResize() no longer sets QGraphicsView::AnchorViewCenter.")
endif()

message(STATUS "check_project_view_resize_anchor: OK")
