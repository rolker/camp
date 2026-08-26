#ifndef CAMP_CRASH_LOG_PATH_H
#define CAMP_CRASH_LOG_PATH_H

/// Where CAMP's crash log goes (#217).
///
/// **Deliberately NOT part of `libcamp_crash`.** The handler library is linked
/// by `camp_map`, which is ROS-free by design (ADR-0002: "the src/camp_map core
/// has zero ROS includes"), and this is the one piece of #217 that needs
/// `rclcpp` — it asks rcl for the logging directory rather than reimplementing
/// rcl's `ROS_LOG_DIR` / `ROS_HOME` / `~/.ros` precedence, which would be a
/// second copy of upstream policy free to drift out of agreement with the
/// directory `ros2 launch` actually writes to.
///
/// So the *policy* (which directory, under which environment) stays here in the
/// application, which already depends on rclcpp, and the *mechanism* (signal
/// handlers, alternate stacks, async-signal-safe emission) lives in the
/// library. `main()` resolves the path with this and hands the string to
/// `camp_crash::install_crash_handlers()`.
///
/// Kept in `namespace camp_crash` alongside the library's own API: it is one
/// feature with one namespace, split across two targets for the dependency
/// reason above, not two features.

#include <string>

namespace camp_crash
{

/// Resolve the crash-log path. Does **not** create the file.
///
/// Returns an empty string if the log directory cannot be resolved. **That is
/// not an error to act on**: the caller still installs the handlers, which then
/// write to stderr only. A missing log directory must never keep CAMP from
/// starting.
///
/// The file is created by the handler, at crash time, not here. Pre-creating it
/// left a zero-byte file behind after every clean run, which in a log directory
/// holding thousands of entries makes a real crash report impossible to spot —
/// and let a recycled pid truncate an earlier genuine report.
///
/// The path is `<base ROS logging dir>/camp_crash_<pid>.log` — where the base
/// dir is `$ROS_LOG_DIR`, else `$ROS_HOME/log`, else `~/.ros/log`. Note this
/// is the *base* directory, NOT the per-run `~/.ros/log/<timestamp>/` that
/// `ros2 launch` creates: launch never exports that path to child processes
/// unless the launch file uses `SetROSLogDir`, and camp_launch.py does not.
/// So crash files land flat and accumulate across runs; the `<pid>` in the
/// name is what correlates a file to the launch log's
/// `process has died [pid N, ...]` line.
///
/// Does **not** require `rclcpp::init()`: `rcl_logging_get_logging_directory()`
/// reads `$ROS_LOG_DIR` / `$ROS_HOME` / `$HOME` directly, which is why a test
/// can call it standalone. `main()` still calls it after `rclcpp::init()`, so
/// the resolved path reflects the environment CAMP is actually running under.
std::string crash_log_path();

} // namespace camp_crash

#endif
