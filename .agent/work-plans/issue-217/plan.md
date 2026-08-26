# Plan: CAMP should log its own crash — install SIGSEGV/SIGABRT/terminate handlers that dump a backtrace

## Issue

https://github.com/rolker/camp/issues/217

## Context

`src/camp/main.cpp` is 43 lines with no signal or terminate handling. On
2026-08-25 CAMP segfaulted five times on the operator station (#215) and left
zero evidence: apport discards crashes from unpackaged (colcon-built) binaries
at `/usr/share/apport/apport:1136`, so no core is ever written, and the
`ros2 launch` log only records `process has died [pid N, exit code -11]`.
Fixing the host (`systemd-coredump`, `/etc/apport/settings`) is root-level
field-host administration and is out of scope by workspace rule
(AGENTS.md § Never). CAMP needs to explain its own death without host help.

Three death classes are in scope, all observed in this codebase per the issue:
`SIGSEGV` (#215), `SIGABRT` (`pure virtual method called` / `terminate called
after throwing rclcpp::exceptions::RCLError`, #215/#207), and an unhandled
exception reaching `std::terminate` (#207's abort-on-close path).

**Operator decisions (settled, not open questions):**

1. **Backtrace destination — both stderr and a dedicated file.** stderr lands
   in the `ros2 launch` session log immediately next to the existing
   `process has died [pid N, exit code -11]` line. The file gives a durable,
   greppable artifact independent of scrollback. Both fds are opened once at
   startup, before any handler can fire.
2. **No ADR.** Considered and declined: this adds diagnostics, it does not
   change CAMP's architecture or constrain future design (camp's ADRs 0002-
   0015 are all map/scene/raster architecture; a signal-handling utility is
   not in that class). The rationale is recorded here and in the PR rather
   than as a separate ADR file.

## Approach

1. **New pure-utility module `src/camp/crash_handler.{h,cpp}`.** Keeps
   `main.cpp` thin and gives the regression test (step 5) something to link
   without pulling in Qt/rclcpp app startup. No ROS or Qt types in the header;
   the `.cpp` may call `rclcpp::get_logging_directory()` at install time
   (rclcpp is already a `main.cpp`/CCOMAutonomousMissionPlanner dependency —
   CMakeLists.txt `ament_target_dependencies(... rclcpp ...)`).

2. **Path convention: `<rclcpp::get_logging_directory()>/camp_crash_<pid>.log`.**
   `rclcpp::get_logging_directory()` (jazzy, `rclcpp/logger.hpp`, backed by
   `rcl_logging_get_logging_directory()`) returns the same per-run directory
   ROS node logs already land in (typically
   `~/.ros/log/<timestamp>/`), so the crash file sits next to the run it
   belongs to instead of a fixed, run-independent path. Call it once, after
   `rclcpp::init()` and before installing the handlers, and pre-open the file
   with `::open(path, O_WRONLY|O_CREAT|O_TRUNC, 0644)`. If `open()` fails,
   fall back to stderr-only (still install handlers; never abort startup over
   a missing log dir).

3. **`install_crash_handlers(int backtrace_fd)`** (called from `main()` right
   after `rclcpp::init()`):
   - `signal(SIGSEGV, &on_fatal_signal)`, `signal(SIGABRT, &on_fatal_signal)`.
   - `on_fatal_signal(int sig)`: async-signal-safe only. Write a fixed
     signal-name preamble with `write()` (not `fprintf`/`iostream`) to stderr
     (fd 2) and to the pre-opened crash-file fd; call
     `backtrace()` + `backtrace_symbols_fd()` (not `backtrace_symbols()`,
     which allocates — the issue specifically calls out heap corruption as
     the suspected #215 mechanism) writing to both fds; then
     `signal(sig, SIG_DFL); raise(sig);` to re-raise with the default handler
     so the exit status (`-11`/`-6`) and any future core behavior are
     preserved unchanged.
   - `std::set_terminate(&on_terminate)`: if a `std::exception` is the active
     exception (`std::current_exception()` + `try { rethrow } catch`), write
     its `what()` to both fds — this path runs on the main thread during
     unwind, before `abort()`, so it is not required to be async-signal-safe,
     but MUST NOT allocate/throw itself if the reason for termination is
     already heap corruption; keep it to `write()` + `strlen()` for
     symmetry with the signal path and to avoid a second-failure mode. Then
     call `backtrace()`/`backtrace_symbols_fd()` the same way as the signal
     handler (a `terminate` reached via `abort()`, e.g.
     "pure virtual method called", still benefits from a stack), and end
     with `std::abort()`.
   - All handler-side output goes through `write()` exclusively — extends
     the issue's `backtrace_symbols_fd` requirement to the signal-name/what()
     preambles too (Issue Review action item).

4. **Build flag.** Add
   `set_target_properties(CCOMAutonomousMissionPlanner PROPERTIES ENABLE_EXPORTS ON)`
   in `CMakeLists.txt` next to the `add_executable(CCOMAutonomousMissionPlanner ...)`
   call (line 143) — the CMake-portable equivalent of `-rdynamic`, needed so
   `backtrace_symbols_fd()` resolves function names instead of bare addresses.

5. **Regression test `test/test_crash_handler.cpp`** (new `ament_add_gtest`
   target, following the existing pattern at CMakeLists.txt:403+): a
   death-test style test using `ASSERT_EXIT`/`EXPECT_EXIT` (gtest's
   subprocess-based death test, which itself relies on `fork()` — the
   process-under-test crashes in the forked child, so the parent test process
   is unaffected) that:
   - installs the handlers against a temp file (via the same
     `install_crash_handlers()` entry point, no `rclcpp::init()` needed since
     the fd is passed in directly — this is why step 1 keeps the header
     ROS-free and fd-based rather than resolving the path internally),
   - raises `SIGSEGV`, asserts the child's exit description matches signal 11
     and that the temp file contains a non-empty backtrace,
   - repeats for `SIGABRT`,
   - throws an uncaught `std::runtime_error("boom")` in a child process,
     asserts `abort()`'s exit status (signal 6) and that the temp file
     contains `"boom"` and a backtrace.

6. **Documentation.** Add a `.agents/README.md` § Common Pitfalls entry (style
   matching the existing "Shutdown ordering" / "QSettings store name" bullets)
   describing: where the crash file lands
   (`<ROS logging dir>/camp_crash_<pid>.log`), that stderr also gets the same
   output (so `ros2 launch` scrollback has it too), and which three death
   classes are covered.

## Files to Change

| File | Change |
|------|--------|
| `src/camp/crash_handler.h` | New. Declares `install_crash_handlers(int backtrace_fd)` (fd-based, no ROS/Qt types) and `open_crash_log_fd()` (wraps `rclcpp::get_logging_directory()` + `open()`, returns `-1` on failure). |
| `src/camp/crash_handler.cpp` | New. Signal handler, `set_terminate` handler, `write()`-only output, `backtrace()`/`backtrace_symbols_fd()`. |
| `src/camp/main.cpp` | Call `open_crash_log_fd()` and `install_crash_handlers()` immediately after `rclcpp::init()`, before `QApplication a(...)`. |
| `CMakeLists.txt` | Add `crash_handler.cpp` to `SOURCES`; add `ENABLE_EXPORTS ON` target property; add `ament_add_gtest(test_crash_handler ...)` block in the `if(BUILD_TESTING)` section. |
| `test/test_crash_handler.cpp` | New. Death-test coverage for SIGSEGV, SIGABRT, uncaught exception. |
| `.agents/README.md` | New Common Pitfalls bullet documenting the crash-diagnostics behavior. |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Output is stderr (existing `ros2 launch` capture) + a file under the existing ROS log directory; no upload, no dialog, no new host dependency. |
| Enforcement over documentation | Replaces the documentation-only workaround (read `/var/log/kern.log`, needs `adm` group) with a mechanism CAMP carries itself. |
| Capture decisions, not just implementations | No ADR — deliberately declined (see Context); the rationale is recorded here and will be restated in the PR description so the skip is traceable, not silent. |
| Only what's needed | Scope stays to handlers + one build flag + the fd-passing indirection needed to test them; no crash-reporting service, no upload. |
| Test what breaks | Step 5 death-test regression covers all three death classes end-to-end (exit status + backtrace content), not just a manual field-verify. |
| A change includes its consequences | `.agents/README.md` updated in this PR (step 6); no parameters/topics/services change, so no other doc surface is affected. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| camp ADR-0001 (adopt ADRs, precedent set by 0002-0015) | No | Deliberately declined — see Context "No ADR" and the Issue Review's "Capture decisions" row. This is diagnostic tooling, not an architecture decision; the issue body and this plan already carry the durable rationale (why apport can't help, why `backtrace_symbols_fd` not `backtrace_symbols`, why re-raise with `SIG_DFL`, why no core-dump/host fix). |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `main.cpp` startup sequence | `.agents/README.md` Common Pitfalls (crash file location, trigger conditions) | Yes — step 6 |
| `CMakeLists.txt` SOURCES / test list | Nothing else generates from this file (no docs auto-derived from CMakeLists) | N/A |
| Executable link flags (`ENABLE_EXPORTS`) | Binary size/symbol visibility — negligible; no packaging step depends on stripped symbols today (unpackaged colcon build per the issue's own apport analysis) | Yes — noted, no follow-up needed |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): `.agents/README.md` gains a Common
  Pitfalls bullet for the new crash-diagnostics behavior — no existing doc
  becomes inaccurate, this is new operational knowledge.
- **Agent-instruction candidates** (proposals only — operator decides): none.
  The pattern (fd-based handler injection for testability, `write()`-only
  handler output) is CAMP-local and not yet seen enough elsewhere in the
  workspace to warrant a `.agent/knowledge/` entry; revisit if a second
  project repo needs the same crash-handler shape.

## Open Questions

- [ ] No open questions — both design questions raised in the Issue Review
      (backtrace destination, ADR-or-not) were settled by the operator before
      planning; see Context.

## Estimated Scope

Single PR. Six files (2 new source, 1 new test, 3 edited), no cross-repo or
cross-layer coordination.
