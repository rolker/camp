#ifndef CAMP_CRASH_HANDLER_H
#define CAMP_CRASH_HANDLER_H

/// Crash diagnostics for CAMP (#217).
///
/// CAMP is built by colcon, so it is not a packaged binary, and apport
/// **discards the crash report** for unpackaged binaries: the
/// `likely_packaged()` branch of `/usr/share/apport/apport` logs "executable
/// does not belong to a package, ignoring" and returns without filing one.
/// (Named by branch, not by line: the line number moves between releases. The
/// installed version here is apport 2.28.3-0ubuntu0.1.)
///
/// **A core file may still be written — do not assume otherwise.** That same
/// branch first invokes apport's core-dump callback, which writes a core
/// whenever the crashing process's core ulimit is non-zero. Measured on the dev
/// workstation `deadpool`, 2026-08-26: `ulimit -c` is `unlimited`,
/// `/proc/sys/kernel/core_pattern` pipes to apport, and
/// `/var/lib/apport/coredump/` holds cores written by this feature's own death
/// tests. Check `ulimit -c` on the host in question rather than assuming either
/// way — it has not been measured on the operator station.
///
/// None of that reaches an operator mid-deployment. No report is filed; a core,
/// if one is written at all, lands in `/var/lib/apport/coredump/` under a name
/// keyed by exe path, uid, boot id, pid and start time, and reading it needs
/// matching debug symbols and a gdb session on the host. What the operator
/// actually sees is the `ros2 launch` log, and it records only
/// `process has died [pid N, exit code -11]`. Changing any of that means
/// root-level administration of a field host, which is not available
/// mid-deployment — so CAMP explains its own death instead.
///
/// On a fatal signal or an uncaught exception these handlers write a backtrace
/// to a crash file — **created at crash time, never pre-opened** — and then to
/// stderr, which the `ros2 launch` session log captures right next to the
/// "process has died" line. The file goes FIRST, deliberately; the reason is on
/// `emit()` in crash_handler.cpp. The handler then re-raises, so the exit status
/// is unchanged and the supervisor's respawn behavior is identical — unless the
/// anti-hang watchdog fires first, in which case the process dies of SIGALRM.
///
/// **This is `libcamp_crash`, and it is deliberately free of ROS and Qt — in
/// its headers and in what it links.** The fd (or path) is passed in rather
/// than resolved internally, so the handlers can be installed and exercised by
/// a test without `rclcpp::init()` or a `QApplication` — and, more to the
/// point, so `camp_map` can link this library without acquiring a ROS
/// dependency it does not have (ADR-0002). The one part of #217 that does need
/// rclcpp — resolving the ROS logging directory — is the application's, in
/// `src/camp/crash_log_path.h`.
///
/// Every thread started by CAMP's own sources, in the executable *and* in
/// `camp_map` / `camp_map_ros`, calls `install_thread_alt_stack()` on entry.
/// That is what this library being a library buys: before #217's follow-up
/// pass the handler was compiled into the executable only, so the map
/// libraries' threads had no symbol to call.

#include <string>

namespace camp_crash
{

/// Install handlers for fatal signals and for `std::terminate`, writing to
/// stderr and to `crash_log_path` (from the application's `crash_log_path()` in
/// `src/camp/crash_log_path.h`, or any path a test picks). The file is created
/// on the first crash, never before.
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
/// which needs a per-thread alternate signal stack; see
/// `install_thread_alt_stack()` for which threads have one.
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
/// **Call this as the first statement of every thread CAMP's own sources
/// start**, wherever that source lives. Idempotent and cheap — a `thread_local`
/// pointer test — so calling it on a pooled thread that already has a stack
/// costs a compare and returns.
///
/// The call sites today. The `check_worker_alt_stacks` test (CMakeLists.txt,
/// `cmake/check_worker_alt_stacks.cmake`) fails the build's test run if a
/// `QtConcurrent::run()` site is added without one, so coverage cannot be lost
/// silently the way it was before this list existed:
///
///  - `camp_ros::NodeThread::start()` (`src/camp/ros/node_thread.cpp`), which
///    is also where `MultiThreadedExecutor::spin()` runs one worker inline, so
///    that worker is covered too.
///  - `camp::ros::GraphThread::run()` (`src/camp_map/ros/graph_thread.cpp`).
///  - The entry point of every `QtConcurrent::run()` worker — the GDAL /
///    raster / tile work implicated in #215. The alt stack is installed on the
///    *pool* thread that picks the task up and persists for that thread's
///    life, so a pool thread is covered from its first CAMP task onward.
///
/// **Remaining gap, deliberately not papered over.** rclcpp's own internal
/// threads — the `MultiThreadedExecutor`'s *spawned* workers (all but the
/// inline one above) and the `tf2_ros::TransformListener` thread — have no
/// alternate signal stack, because rclcpp offers no thread-entry hook to hang
/// one on. An unbounded recursion inside a subscription callback **may** die
/// silently, depending on which worker picks the callback up. Closing that
/// needs an upstream hook (or a `pthread_create` interposer, which is a far
/// worse trade in a diagnostics feature); it is recorded as a follow-up rather
/// than papered over.
///
/// **Every other crash class on all of these threads is reported normally** —
/// the `sigaction()` handlers are process-wide. What is missing is only the one
/// class that cannot push a handler frame on its own stack, on the rclcpp
/// threads only.
void install_thread_alt_stack();

} // namespace camp_crash

#endif
