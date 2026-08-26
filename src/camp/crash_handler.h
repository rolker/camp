#ifndef CAMP_CRASH_HANDLER_H
#define CAMP_CRASH_HANDLER_H

/// Crash diagnostics for CAMP (#217).
///
/// CAMP is built by colcon, so it is not a packaged binary, and apport
/// **discards the crash report** for unpackaged binaries
/// (`/usr/share/apport/apport:1135-1142`). That branch still writes a core if
/// the user has configured one — but `ulimit -c` is 0 by default, so in
/// practice no core is written either, and the `ros2 launch` log records only
/// `process has died [pid N, exit code -11]`.
/// Fixing that means root-level administration of a field host, which is not
/// available mid-deployment — so CAMP explains its own death instead.
///
/// On a fatal signal or an uncaught exception these handlers write a
/// backtrace to stderr (which the `ros2 launch` session log captures, right
/// next to the "process has died" line) and to a pre-opened file, then
/// re-raise so the exit status is unchanged and the supervisor's respawn
/// behavior is identical.
///
/// **This header is deliberately free of ROS and Qt types.** The fd is passed
/// in rather than resolved internally, so the handlers can be installed and
/// exercised by a test without `rclcpp::init()` or a `QApplication`.

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
/// Must be called after `rclcpp::init()`.
std::string crash_log_path();

/// Install handlers for fatal signals and for `std::terminate`, writing to
/// stderr and to `crash_log_path` (from `crash_log_path()`, or any path a test
/// picks). The file is created on the first crash, never before.
///
/// An empty path installs the handlers stderr-only. Safe to call more than
/// once; calling it again simply reinstalls.
void install_crash_handlers(const std::string& crash_log_path);

/// Overload writing to an already-open fd instead of a path — for callers that
/// have no ROS logging directory, and for tests.
///
/// `backtrace_fd` may be -1, in which case output goes to stderr only. Calling
/// this clears any path set by the overload above.
///
/// The signal handlers themselves are **process-wide**: a SIGSEGV on any
/// thread — including the ROS node thread and the executor's workers — is
/// caught and dumped. The one exception is the *stack-overflow* SIGSEGV, which
/// needs a per-thread alternate signal stack; see
/// `install_thread_alt_stack()`.
void install_crash_handlers(int backtrace_fd);

/// Give the **calling thread** an alternate signal stack.
///
/// `install_crash_handlers()` does this for the thread that calls it (the main
/// thread). `sigaltstack(2)` is a per-thread attribute that `pthread_create(3)`
/// does not inherit, so every other thread starts without one — and a thread
/// without an alternate stack cannot report a *stack-overflow* SIGSEGV at all,
/// because the kernel has no room left on the faulting stack to push a handler
/// frame. Ordinary faults on such a thread are still reported normally.
///
/// Call this as the first statement of any thread CAMP starts itself
/// (`camp_ros::NodeThread::start()` does).
///
/// **Known gap, deliberately not papered over:** threads created *inside*
/// rclcpp — the `MultiThreadedExecutor` workers and the
/// `tf2_ros::TransformListener` thread — offer no entry hook, so they have no
/// alternate stack. An unbounded recursion inside a subscription callback
/// running on an executor worker therefore still dies silently. Every other
/// crash class on those threads is covered.
void install_thread_alt_stack();

} // namespace camp_crash

#endif
