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
/// On a fatal signal or an uncaught exception these handlers write a backtrace
/// to a crash file — **created at crash time, never pre-opened** — and then to
/// stderr, which the `ros2 launch` session log captures right next to the
/// "process has died" line. The file goes FIRST, deliberately; the reason is on
/// `emit()` in crash_handler.cpp. The handler then re-raises, so the exit status
/// is unchanged and the supervisor's respawn behavior is identical — unless the
/// anti-hang watchdog fires first, in which case the process dies of SIGALRM.
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
/// Does **not** require `rclcpp::init()`: it reads `$ROS_LOG_DIR` / `$ROS_HOME`
/// / `$HOME` directly, which is why a test can call it standalone. `main()`
/// still calls it after `rclcpp::init()`, so the resolved path reflects the
/// environment CAMP is actually running under.
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
/// thread — the ROS node thread, the executor's workers, a QtConcurrent worker
/// — is caught and dumped. The one exception is the *stack-overflow* SIGSEGV,
/// which needs a per-thread alternate signal stack, and which several of CAMP's
/// threads do not have; see `install_thread_alt_stack()` for the enumerated
/// gap.
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
/// Call this as the first statement of any thread started by the
/// `CCOMAutonomousMissionPlanner` executable's own sources. Today that is
/// exactly one thread — `camp_ros::NodeThread::start()`
/// (`src/camp/ros/node_thread.cpp`) — which is also where
/// `MultiThreadedExecutor::spin()` runs one worker inline, so that worker is
/// covered too.
///
/// **Known gap, deliberately not papered over and wider than the executable.**
/// Threads with no alternate signal stack, and therefore no report for a
/// *stack-overflow* SIGSEGV:
///
///  - rclcpp-internal threads: the `MultiThreadedExecutor`'s *spawned* workers
///    (all but the inline one above) and the `tf2_ros::TransformListener`
///    thread. Neither offers an entry hook. An unbounded recursion inside a
///    subscription callback **may** die silently, depending on which worker
///    picks the callback up.
///  - `camp::ros::GraphThread` (`src/camp_map/ros/graph_thread.cpp`), live in
///    the shipped app via `MainWindow`. It *does* have an entry hook —
///    `run()` — and does not use it.
///  - Qt's global thread pool: every `QtConcurrent::run()` worker, which is
///    where the GDAL / raster / tile work implicated in #215 executes.
///
/// The last two are in the `camp_map` / `camp_map_ros` libraries, and this
/// translation unit is compiled only into the executable and the test target
/// (CMakeLists.txt:72,458) — not into those installed, exported libraries. So
/// closing them means promoting the crash handler out of the executable and
/// into a library, which changes that library's public surface: a design
/// decision beyond #217's scope, recorded as a follow-up in the work plan
/// rather than made in passing.
///
/// **Every other crash class on all of these threads is reported normally** —
/// the `sigaction()` handlers are process-wide. What is missing is only the one
/// class that cannot push a handler frame on its own stack.
void install_thread_alt_stack();

} // namespace camp_crash

#endif
