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
   `rcl_logging_get_logging_directory()`) returns the **base** logging
   directory — `$ROS_LOG_DIR`, else `$ROS_HOME/log`, else `~/.ros/log`
   (`rcl_logging_interface.h:102-112`). It is **not** the per-run
   `~/.ros/log/<timestamp>/` directory that `ros2 launch` creates: launch
   derives that itself and never exports it to child processes unless the
   launch file uses the explicit `SetROSLogDir` action, which
   `camp_launch.py` does not. [Plan Review must-fix 1 — the original plan
   claimed the per-run directory and was wrong.]

   Consequences, accepted deliberately:
   - Crash files land **flat** in the base log dir and **accumulate across
     runs**. That is acceptable and arguably desirable here: a crash file
     that outlives its run is easier to find after the fact, and `<pid>` in
     the name keeps the five-crashes-in-a-day case (#215) from colliding.
   - There is **no automatic correlation** to the per-run launch directory.
     The `<pid>` provides it manually: the launch log's
     `process has died [pid N, exit code -11]` line names the same pid as
     `camp_crash_<N>.log`. Say so explicitly in the `.agents/README.md`
     bullet, so the correlation step is documented rather than rediscovered.
   - The `.agents/README.md` bullet **must document the base path**
     (`~/.ros/log/camp_crash_<pid>.log` in the default case), not a
     timestamped subdirectory. Documenting the wrong directory would send a
     field agent looking where the file never is — the concrete harm behind
     must-fix 1.

   Resolve the directory once, after `rclcpp::init()` and before installing
   the handlers, and pre-open the file with
   `::open(path, O_WRONLY|O_CREAT|O_TRUNC|O_CLOEXEC, 0644)` (`O_CLOEXEC` so
   the fd does not leak into any child process camp spawns). If `open()`
   fails, fall back to stderr-only — still install the handlers; never abort
   startup over a missing log dir.

2b. **Log-dir resolution must never fail startup.** `get_logging_directory()`
   **throws** `rclcpp::exceptions::RCLError` (`rclcpp/logger.hpp:80-90`).
   Wrap the call in `try { ... } catch (const std::exception&) { fd = -1; }`
   so an unresolvable log directory degrades to stderr-only rather than
   terminating CAMP before `QApplication` is even constructed. A diagnostics
   feature must not become a startup failure on the field hosts it exists to
   serve. [Plan Review must-fix 2.]

3. **`install_crash_handlers(int backtrace_fd)`** (called from `main()` right
   after `rclcpp::init()`):
   - Install with **`sigaction`**, not `signal()`, using `SA_ONSTACK` plus a
     `sigaltstack()` alternate signal stack allocated at install time. Without
     an alternate stack a **stack-overflow SIGSEGV cannot be handled at all** —
     the kernel has no room to push the handler frame — so infinite-recursion
     crashes stay as silent as they are today, which is precisely the failure
     mode this issue exists to end. [Plan Review should-fix.]
   - Cover `SIGSEGV` and `SIGABRT`; also install for **`SIGBUS`, `SIGFPE` and
     `SIGILL`**. They are the same class of fatal, silent death, the handler
     is identical, and adding them costs three lines. [Plan Review suggestion.]
   - **Re-entry guard: restore `SIG_DFL` on entry, not on exit.** If the
     handler itself faults — entirely possible when the heap is already
     corrupt, which is the suspected #215 mechanism — the default action must
     take over immediately rather than recursing into a second fault. Set
     `signal(sig, SIG_DFL)` as the *first* statement of the handler, then
     emit, then `raise(sig)`. [Plan Review should-fix.]
   - **Warm up `backtrace()` at install time.** Its first call may `dlopen`
     libgcc and allocate; doing that lazily inside a handler entered from a
     corrupt heap is the exact hazard the `backtrace_symbols_fd` rule exists
     to avoid. Call `backtrace()` once into a throwaway buffer during
     `install_crash_handlers()` so the handler path is already resolved.
     [Plan Review should-fix.]
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
   - **Do not emit a second stack on the terminate path.** `std::abort()`
     raises `SIGABRT`, which re-enters `on_fatal_signal` and prints a second,
     useless backtrace rooted in `abort()` itself. Guard with a
     `static volatile sig_atomic_t already_dumped` set by whichever handler
     runs first; the signal handler still re-raises to preserve exit status,
     it simply skips the duplicate output. [Plan Review should-fix.]
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
   - throws an uncaught `std::runtime_error("boom")` **across a `noexcept`
     boundary**, asserts `abort()`'s exit status (signal 6) and that the temp
     file contains `"boom"` and a backtrace.

     **This detail is load-bearing, not stylistic.** gtest wraps death-test
     statements in `try`/`catch` when exceptions are enabled
     (`gtest-death-test-internal.h:197-213`), so a plain `throw` inside
     `ASSERT_EXIT` is caught by gtest and reported as
     `TEST_THREW_EXCEPTION` — `std::terminate` is never reached and the
     `set_terminate` handler ships **untested**. That handler is the half
     that targets #207, so an untested one defeats a stated purpose of this
     issue. Throwing from inside a `noexcept` function escapes gtest's catch
     and forces the real `std::terminate` path. [Plan Review must-fix 3.]

   - **Assert a resolved symbol name, not merely a non-empty backtrace.** The
     test exists partly to protect the `ENABLE_EXPORTS` build flag; without
     it `backtrace_symbols_fd()` still emits output, just bare addresses with
     no function names. A non-empty assertion therefore passes with the flag
     removed and covers nothing. Assert that the dump contains a symbol from
     the test binary (e.g. the crashing helper's mangled name, or `main`), so
     dropping `ENABLE_EXPORTS` actually fails the test. [Plan Review
     should-fix.]

6. **Documentation.** Add a `.agents/README.md` § Common Pitfalls entry (style
   matching the existing "Shutdown ordering" / "QSettings store name" bullets)
   describing:
   - where the crash file lands — the **base** ROS logging directory,
     `~/.ros/log/camp_crash_<pid>.log` by default, **not** a per-run
     timestamped subdirectory (see step 2);
   - that the `<pid>` is how you correlate it to the launch log's
     `process has died [pid N, exit code -11]` line;
   - that stderr gets the same output, so `ros2 launch` scrollback has it too;
   - which death classes are covered (fatal signals + uncaught exception);
   - that `backtrace_symbols_fd()` output is **mangled** — pipe it through
     `c++filt` to read it. [Plan Review suggestion; without this the first
     reader assumes the dump is corrupt.]

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

## Revisions

**Amended after Plan Review (verdict: changes-requested), before implementation
started.** The reviewer verified three of this plan's claims against the
installed Jazzy headers and gtest source; all three were wrong, and each would
have shipped a defective artifact. Corrected in place per `plan-task`'s
"During implementation" rules:

| Plan Review finding | Where corrected |
|---|---|
| must-fix 1 — `get_logging_directory()` returns the **base** log dir, not the per-run timestamped one | Step 2, rewritten with the real semantics, the accepted consequences (flat, accumulating, pid-correlated), and the corrected path for the docs bullet |
| must-fix 2 — `get_logging_directory()` **throws** `RCLError`; unhandled it aborts startup | New step 2b |
| must-fix 3 — gtest catches a plain `throw` in a death test, so `std::terminate` is never reached and the `set_terminate` handler ships untested | Step 5, now requires throwing across a `noexcept` boundary |
| should-fix — handler re-entry + uncatchable stack-overflow SIGSEGV | Step 3: `sigaction` + `sigaltstack`/`SA_ONSTACK`, `SIG_DFL` restored on entry |
| should-fix — `backtrace()`'s first call may `dlopen`/allocate | Step 3: warm-up call at install time |
| should-fix — terminate path emits a second useless stack via `abort()` | Step 3: `already_dumped` guard |
| should-fix — "non-empty backtrace" assertion does not cover the `ENABLE_EXPORTS` regression it exists to protect | Step 5: assert a resolved symbol name |
| suggestion — `O_CLOEXEC`; mangled output; `SIGBUS`/`SIGFPE`/`SIGILL` | Steps 2, 6 and 3 respectively |

The one suggestion **not** adopted needs no change: the reviewer confirmed the
planned `main.cpp` ordering (after `rclcpp::init()`, before `QApplication`) is
correct — `rclcpp::init()` claims only SIGINT/SIGTERM and Qt5 installs no fatal
handlers — and asked only that it stay explicit, which step 3 already makes it.

## Estimated Scope

Single PR. Six files (2 new source, 1 new test, 3 edited), no cross-repo or
cross-layer coordination.
