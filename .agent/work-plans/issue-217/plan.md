# Plan: CAMP should log its own crash — install SIGSEGV/SIGABRT/terminate handlers that dump a backtrace

## Issue

https://github.com/rolker/camp/issues/217

## Context

`src/camp/main.cpp` is 43 lines with no signal or terminate handling. On
2026-08-25 CAMP segfaulted five times on the operator station (#215) and left
zero evidence: apport's `likely_packaged()` branch discards the crash *report*
for unpackaged (colcon-built) binaries, and the `ros2 launch` log only records
`process has died [pid N, exit code -11]`. (Cited by branch, not line: the line
number moves between apport releases; installed here is 2.28.3-0ubuntu0.1.) A
**core** is a separate matter — apport still runs its core-dump callback, so one
is written wherever the crashing process's core ulimit is non-zero; measured
`unlimited` on the dev workstation `deadpool`, 2026-08-26, with cores landing in
`/var/lib/apport/coredump/`. That does not help an operator: no report is filed,
and reading a core needs matching debug symbols and a gdb session on the host.
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
   greppable artifact independent of scrollback. The crash file's **path** is
   resolved at install time; the file itself is created by the handler at crash
   time (see step 2), and the durable write goes out before stderr (step 3).
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
   the handlers. **Do not pre-open the file** — resolve the *path* at install
   time and `::open()` it inside the handler, on the first crash
   (`open(2)` is async-signal-safe). [Local Review Round 1 suggestion:
   pre-opening with `O_CREAT|O_TRUNC` left a zero-byte
   `camp_crash_<pid>.log` after every clean run, making "no crash"
   indistinguishable from "crashed before the first write" in a log directory
   holding 10k+ entries, and let a recycled pid truncate an earlier genuine
   report.] Open flags: `O_WRONLY|O_CREAT|O_APPEND|O_CLOEXEC|O_NOFOLLOW`, mode
   `0600` — `O_APPEND` so a recycled pid appends rather than destroying;
   `O_CLOEXEC` so the fd does not leak into a child process; `O_NOFOLLOW`
   because `$ROS_LOG_DIR`/`$ROS_HOME` are environment-controlled and a
   pre-planted symlink must not redirect the write; `0600` because the dump
   embeds full install paths. If `open()` fails, the handlers still report to
   stderr — never abort startup over a log dir.

2b. **Log-dir resolution must never fail startup.** `get_logging_directory()`
   **throws** `rclcpp::exceptions::RCLError` (`rclcpp/logger.hpp:80-90`).
   Wrap the call in `try { ... } catch (const std::exception&) { return {}; }`
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
     **The alternate stack is per-thread** (`sigaltstack(2)` is a per-thread
     attribute that `pthread_create(3)` does not inherit), so it is
     `thread_local` behind an exported `install_thread_alt_stack()` that
     `camp_ros::NodeThread::start()` also calls — the only thread the
     executable's own sources start. Three groups have **no** alternate stack:
     rclcpp's *spawned* executor workers and the `tf2_ros::TransformListener`
     thread (no entry hook); `camp::ros::GraphThread` in `camp_map`; and every
     `QtConcurrent` worker. The last two have entry hooks but live in the
     `camp_map`/`camp_map_ros` libraries, which do not compile
     `crash_handler.cpp` — closing them means promoting the crash handler into
     an exported library, a design change beyond this issue (see Follow-ups).
     The gap is enumerated in the header and in the `.agents/README.md` bullet
     rather than left implied as covered. [Local Review Round 1 must-fix: the
     original single global stack covered only the main thread while the docs
     claimed the class outright. Round 2 must-fix: the enumeration named only
     the rclcpp-internal threads.]
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
     signal-name preamble with `write()` (not `fprintf`/`iostream`) to the
     crash-file fd — opened by the handler on first use — and then to stderr
     (fd 2); call
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
     symmetry with the signal path and to avoid a second-failure mode.
     **Order: backtrace FIRST, `what()` extraction second.**
     `current_exception()`/`rethrow_exception()` allocate and run the
     unwinder; if that faults under the very heap corruption being diagnosed,
     the SIGSEGV handler finds `already_dumped` set and suppresses its output,
     so an exception-first order yields a header line and no stack at all.
     Backtrace-first degrades instead to "stack without a reason line".
     [Plan Review should-fix "order the terminate handler's output", carried
     into implementation only in its `already_dumped` half — re-raised as a
     Local Review Round 1 must-fix.] End with `std::abort()`.
   - **Do not emit a second stack on the terminate path.** `std::abort()`
     raises `SIGABRT`, which re-enters `on_fatal_signal` and prints a second,
     useless backtrace rooted in `abort()` itself. Guard with a
     `static volatile sig_atomic_t already_dumped` set by whichever handler
     runs first; the signal handler still re-raises to preserve exit status,
     it simply skips the duplicate output. [Plan Review should-fix.]
   - Handler hardening carried in from Local Review Round 1: the
     `already_dumped` guard is an atomic `__atomic_test_and_set` (a plain
     test-and-set races between two faulting threads); `SA_SIGINFO` so
     `si_code`/`si_addr` are reported — `si_code` through a **signed** formatter
     and `si_addr` only when `si_code > 0`, i.e. only when the kernel generated
     the signal (Round 2 must-fix); the faulting thread's `gettid()` in the
     header line; an `alarm()` watchdog on **both** the signal and the terminate
     paths, because `backtrace()` takes glibc's loader locks and a fault while
     another thread holds one in a GDAL/Qt-plugin `dlopen()` would turn a silent
     death into a silent hang — armed by a shared `arm_watchdog()` that restores
     SIGALRM to `SIG_DFL` and unblocks it first, since restoring the *faulting*
     signal's disposition says nothing about SIGALRM (Round 2 must-fix);
     `SA_RESETHAND` in place of the inert `SA_RESTART`; `extern "C"` linkage on
     the handler; every `sigaction()`/`sigaltstack()` return value checked and
     reported, with `SIGSTKSZ` treated as failable and floored at 64 KB; the
     crash fd validated with `fstat()` after `open()` and closed on reinstall if
     the handler opened it; a release/acquire pair on the crash-path flag.
     `::raise(sig)` is kept deliberately — see the decline recorded under
     Revisions.
   - `main.cpp` installs stderr-only handlers **before** `rclcpp::init()` as
     well: `rclcpp::init()` is itself a documented thrower, and an exception
     escaping it would otherwise reach `std::terminate` with the default
     handler and die as silently as before this issue. [Local Review Round 1
     suggestion.]
   - All handler-side output goes through `write()` exclusively — extends
     the issue's `backtrace_symbols_fd` requirement to the signal-name/what()
     preambles too (Issue Review action item).

4. **Build flag.** Add
   `set_target_properties(CCOMAutonomousMissionPlanner PROPERTIES ENABLE_EXPORTS ON)`
   in `CMakeLists.txt` next to the `add_executable(CCOMAutonomousMissionPlanner ...)`
   call (line 143) — the CMake-portable equivalent of `-rdynamic`, needed so
   `backtrace_symbols_fd()` resolves function names instead of bare addresses.
   **Guard it with a check against the shipped executable**, not against the
   test target: `test_crash_handler` sets `ENABLE_EXPORTS` on itself, so its
   symbol assertion stays green if the flag is dropped from
   `CCOMAutonomousMissionPlanner`. A `check_camp_exports` CTest runs
   `readelf --dyn-syms` over `$<TARGET_FILE:CCOMAutonomousMissionPlanner>` and
   requires a `camp_crash*` symbol in `.dynsym`. Record the accepted
   tradeoff — exporting ~100 TUs' globals invites symbol interposition from a
   later-loaded plugin — next to the flag. [Local Review Round 1 must-fix +
   suggestion.]

5. **Regression test `test/test_crash_handler.cpp`** (new `ament_add_gtest`
   target, following the existing pattern at CMakeLists.txt:403+): a
   death-test style test using `ASSERT_EXIT`/`EXPECT_EXIT` (gtest's
   subprocess-based death test, which itself relies on `fork()` — the
   process-under-test crashes in the forked child, so the parent test process
   is unaffected) that:
   - installs the handlers against a temp file (via the same
     `install_crash_handlers()` entry point, no `rclcpp::init()` needed since
     the path/fd is passed in directly — this is why step 1 keeps the header
     ROS-free and fd-based rather than resolving the path internally),
   - raises `SIGSEGV`, asserts the child's exit description matches signal 11
     and that the temp file contains a backtrace **with a resolved symbol
     name** (see the `ENABLE_EXPORTS` bullet below — a non-empty assertion
     would pass with the flag removed and cover nothing),
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

   - Also covered, per Local Review Round 1: the **stderr** half (all three
     `ASSERT_EXIT` matchers assert the real preamble instead of `""`, so
     deleting the `write(STDERR_FILENO, ...)` calls fails the suite); a crash
     on a **non-main thread**, which is where CAMP actually crashes; that
     installing creates **no file** when there is no crash; and the
     `crash_log_path()` / unopenable-path / `fd == -1` degradation paths.
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
| `src/camp_crash/crash_handler.h` | New. Declares `install_crash_handlers(const std::string& crash_log_path)` and an `install_crash_handlers(int backtrace_fd)` overload (no ROS/Qt types), and `install_thread_alt_stack()`. Header of the `camp_crash` library. |
| `src/camp_crash/crash_handler.cpp` | New. Signal handler, `set_terminate` handler, `write()`-only output, `backtrace()`/`backtrace_symbols_fd()`, per-thread alternate stacks. No ROS, no Qt. |
| `src/camp/crash_log_path.h` / `.cpp` | New. `crash_log_path()` — wraps `rclcpp::get_logging_directory()`, returns `""` on failure; creates nothing. Kept in the **executable**, not the library: it is the only rclcpp-dependent piece of #217, and `camp_map` links the library (see Revisions round 3). |
| `src/camp/main.cpp` | `install_crash_handlers(-1)` before `rclcpp::init()`, then `install_crash_handlers(crash_log_path())` immediately after it and before `QApplication a(...)`. |
| `CMakeLists.txt` | Add the `camp_crash` shared library + install rule; link it from `CCOMAutonomousMissionPlanner` and PUBLIC from `camp_map`; add `crash_log_path.cpp` to `SOURCES`; add `ENABLE_EXPORTS ON` target property; add `ament_add_gtest(test_crash_handler ...)` and the `check_camp_exports`, `check_worker_alt_stacks` and `check_crash_lib_deps` CTests in the `if(BUILD_TESTING)` section. |
| `cmake/check_dynamic_symbols.cmake` | New. `readelf`-based `ENABLE_EXPORTS` regression guard for the shipped executable. |
| `cmake/check_worker_alt_stacks.cmake` | New. Fails the test run if a thread entry point (`QtConcurrent::run()` site or a `::run()` override) exists without an `install_thread_alt_stack()` call in the same file. |
| `cmake/check_crash_lib_deps.cmake` | New. Asserts `libcamp_crash`'s `DT_NEEDED` list carries no Qt / ROS / GDAL / `marine_*` entry — the boundary that lets `camp_map` link it (ADR-0002). |
| `src/camp/ros/node_thread.cpp` | Call `install_thread_alt_stack()` on entry to `NodeThread::start()`. |
| `src/camp_map/ros/graph_thread.cpp` | Call `install_thread_alt_stack()` on entry to `GraphThread::run()`. |
| `src/camp_map/**` (6 files) | Call `install_thread_alt_stack()` as the first statement of each `QtConcurrent` worker entry point: `Polygon::processPolygon`, `OccupancyGrid::processOccupancyGrid`, `GridMap::processGridMap`, `RasterLayer::loadAndReprojectFile`, `GggsTileLayer::loadTilesWorker`, `writeTileToCache`, `reloadTilesFromCache`. |
| `test/test_crash_handler.cpp` | New. Death-test coverage for SIGSEGV, SIGABRT, uncaught exception. Links the shipped `camp_crash` library rather than recompiling its source. |
| `.agents/README.md` | New Common Pitfalls bullets documenting the crash-diagnostics behavior and the `libcamp_crash` dependency boundary; `camp_crash` added to the target inventory and repository layout. |

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
| workspace ADR-0001 (record architecture decisions as ADRs; camp's own ADR-0001 is the TopicBridge/executor contract, and camp has no meta-ADR) | No | Deliberately declined — see Context "No ADR" and the Issue Review's "Capture decisions" row. This is diagnostic tooling, not an architecture decision; the issue body and this plan already carry the durable rationale (why apport can't help, why `backtrace_symbols_fd` not `backtrace_symbols`, why re-raise with `SIG_DFL`, why no core-dump/host fix). |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `main.cpp` startup sequence | `.agents/README.md` Common Pitfalls (crash file location, trigger conditions) | Yes — step 6 |
| `CMakeLists.txt` SOURCES / test list | Nothing else generates from this file (no docs auto-derived from CMakeLists) | N/A |
| Crash-log file lifecycle (created lazily, `O_APPEND`, `0600`) | Nothing generates from it; the `.agents/README.md` bullet describes where it lands and that its *presence* means a crash | Yes — step 6 |
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

**Amended again after Local Review (Pre-Push) Round 1** (verdict:
changes-requested), during the address-findings pass. Corrected in place per
`plan-task`'s "During implementation" rules:

| Local Review finding | Where corrected |
|---|---|
| must-fix 1 — stderr written before the durable fd, so a blocked or broken stderr costs the crash file | Step 3: durable fd first, stderr second, `SIGPIPE` ignored at install |
| must-fix 2 — terminate handler ran the allocating exception introspection before emitting any stack | Step 3 `set_terminate` bullet, rewritten with the ordering and why it is load-bearing. This is the Plan Review should-fix that the first Revisions table recorded as addressed but which shipped only in its `already_dumped` half |
| must-fix 3 — `sigaltstack()` is per-thread, so the stack-overflow claim was false off the main thread | Step 3 `sigaltstack` bullet: `thread_local` + `install_thread_alt_stack()`, called from `NodeThread::start()`; the rclcpp-internal threads recorded as a known gap in the header and the README bullet |
| must-fix 4 — the advertised `ENABLE_EXPORTS` regression guard did not exist | Step 4: `check_camp_exports` CTest over the shipped binary; Files to Change gains `cmake/check_dynamic_symbols.cmake` |
| suggestions — non-atomic guard, `SA_SIGINFO`, tid in the header, `alarm()` bound on the loader-lock deadlock, `SIGSTKSZ` evaluated once, unchecked `sigaction`/`sigaltstack` returns, `SA_RESETHAND` over the inert `SA_RESTART`, `extern "C"` handler linkage, `raise()` comment | New Step 3 "Handler hardening" bullet |
| suggestions — `O_TRUNC` littering a zero-byte file every run and truncating a real report on pid reuse; `O_NOFOLLOW`/mode | Step 2, rewritten around lazy open-at-crash-time and the hardened flags; `open_crash_log_fd()` becomes `crash_log_path()` throughout |
| suggestion — install stderr-only handlers before `rclcpp::init()`, itself a thrower | Step 3, new bullet; Files to Change `main.cpp` row |
| suggestions — `ASSERT_EXIT` matchers all `""`; no non-main-thread crash test; no `open_crash_log_fd()`/`fd == -1` coverage; `open_temp()` asserting inside the forked child | Step 5, new coverage bullet (the `open_temp()` helper is gone: the handler creates the file itself) |
| suggestion — plan step 5 still said "non-empty backtrace" | Step 5, first sub-bullet |
| suggestion — ADR Compliance cited camp ADR-0001 for the adopt-ADRs decision | ADR Compliance table now cites workspace ADR-0001 and names what camp's own ADR-0001 actually is |
| suggestion — apport "discards outright" overstated | `.agents/README.md` bullet and `crash_handler.h` header comment (not a plan claim) |

One Round-1 suggestion was **declined**: replacing `::raise(sig)` with a return
from the handler so a core would be taken at the faulting instruction. The
decline stands, but its recorded basis has been corrected (Round 2 must-fix 6 —
the "no core is written" premise is false where it can be checked). The reasons
that actually hold, and do not depend on any host's `ulimit -c`:

- Returning only works for a **hardware fault**, where the faulting instruction
  re-executes and traps again against the restored `SIG_DFL`. For a signal that
  was *delivered* rather than faulted — `kill(1)`, or the `raise()` inside
  `abort()` on the #207 path — returning resumes the interrupted code, and CAMP
  survives a SIGSEGV or SIGABRT in an undefined state. That is strictly worse
  than a core whose top frame is this handler.
- Doing it correctly therefore means branching on `si_code` in the death path of
  a diagnostics-only feature.
- The backtrace is what this feature delivers, it names the real fault site, and
  it is identical either way.

The misleading half of the comment it flagged was corrected in Round 1, and the
whole comment was re-argued on these grounds in Round 2.

**Amended again after Local Review (Pre-Push) Round 2** (verdict:
changes-requested; 6 must-fix, 18 suggestions), during the second
address-findings pass. Round 2's stated reason for continuing was *claims
outrunning code* — two Round-1 fixes recorded as closing a finding that did not,
and a test advertised as a guard that could not fail. Every claim below was
verified empirically before being written down, and every new or changed test
was watched to fail with its fix removed.

| Local Review Round 2 finding | Where corrected |
|---|---|
| must-fix 1 — `si_code` through the unsigned formatter (`18446744073709551610`); `si_addr` gated on the signal number, printing `uid<<32\|pid` as an address | Step 3 hardening bullet; `emit_long()` plus an `si_code > 0` gate; asserted in the SIGSEGV, SIGABRT and stack-overflow tests |
| must-fix 2 — `on_terminate()` had no `alarm()` bound, and the alarm's `SIG_DFL` reasoning was wrong | Step 3 hardening bullet; shared `arm_watchdog()` on both paths, restoring and unblocking SIGALRM itself |
| must-fix 3 — the alt-stack gap is wider than documented (`camp::ros::GraphThread` has an entry hook and does not call it; QtConcurrent workers uncovered) | Step 3 `sigaltstack` bullet, rewritten with the full enumeration and why the `camp_map` half is a follow-up rather than a #217 change |
| must-fix 4 — the non-main-thread test raises on a healthy stack, so it does not cover the alt stack | Step 5 coverage bullet; new `StackOverflowOnANonMainThreadIsReported`, verified to fail (empty stderr) with the alt stack removed |
| must-fix 5 — `crash_handler.h`'s summary still described the pre-opened, stderr-first design | Context decision 1 and step 3's preamble bullet; the header was rewritten |
| must-fix 6 — the "`ulimit -c` is 0" premise is false where checkable | Context, and the `::raise(sig)` decline below; measured values attributed to host and date per AGENTS.md § Documentation Accuracy |
| suggestions — unreported `install_thread_alt_stack()` failures; `SIGSTKSZ` -1 becoming `SIZE_MAX`; unchecked `signal(SIGPIPE)` and its process-wide scope; `g_crash_fd` overwritten without closing; missing release barrier; `O_NOFOLLOW` not sufficient alone; the deliberate `thread_local` leak | Step 3 hardening bullet |
| suggestion — `check_camp_exports` silently unregistered when `readelf` is absent | Step 4; the test is now always registered and the script FATAL_ERRORs on a missing readelf |
| suggestions — predictable `/tmp` test paths; `NoCrashFileMeansNoFile` installing handlers in the gtest parent; `crash_log_path()`'s catch branch uncovered while claimed | Step 5 coverage bullet |
| suggestions — "must be called after `rclcpp::init()`" contradicted by its own test; "before anything at all can fault" overclaimed; the `alarm()`/exit-status clause; the README's "still dies silently" | Corrected at each site (header, `main.cpp`, `crash_handler.cpp`, `.agents/README.md`) |
| suggestion — stale plan prose ("Both fds are opened once at startup", "the pre-opened crash-file fd") | Context decision 1 and step 3 |
| suggestion — apport line citations disagreed with each other and with the installed version | Context, `crash_handler.h`, `.agents/README.md`: cited by branch name plus version |
| suggestion — the decline's "nothing can collect the benefit" clause | Re-argued above on grounds independent of `ulimit -c` |

### Round 3 — the worker-thread gap, closed rather than carried

Round 2 closed the alternate-signal-stack must-fix by **correcting the
documentation**: the gap on `camp::ros::GraphThread` and the QtConcurrent pool
workers was enumerated honestly and recorded as a follow-up, because
`crash_handler.cpp` was compiled only into the executable and the test target,
and code in `camp_map` cannot call into the executable that links it. Roland's
instruction was to close it properly instead, by promoting the handler into a
library. That is what this round does.

**The design decision the round-2 note said should not be made in passing.**
`camp_map` is ROS-free by design (ADR-0002), and it now links the crash handler
— so whatever the handler links, `camp_map` links. `crash_log_path()` was the
one piece of #217 that needs rclcpp. Three options were weighed:

| Option | Rejected because |
|---|---|
| Put the whole handler, rclcpp and all, in the library | Hands `camp_map` a ROS dependency to buy a diagnostics feature. ADR-0002's boundary is the point of that library. |
| Keep it whole and ROS-free by reimplementing rcl's `ROS_LOG_DIR` / `ROS_HOME` / `~/.ros` precedence | A second copy of upstream policy, free to drift out of agreement with the directory `ros2 launch` actually writes to — a diagnostics feature failing quietly, which is the exact failure mode #217 exists to end. |
| **Chosen:** split mechanism from policy | `libcamp_crash` = signals, alternate stacks, async-signal-safe emission (libc + libstdc++ only). `src/camp/crash_log_path.cpp` = the rclcpp call, in the executable, which already depends on rclcpp. One namespace, two targets, for a stated dependency reason. |

**What is now covered.** Every thread CAMP's own sources start installs an
alternate signal stack on entry: the main thread, `NodeThread::start()` (whose
inline `MultiThreadedExecutor` worker comes with it), `GraphThread::run()`, and
all seven QtConcurrent worker entry points — including the GDAL/raster/tile work
implicated in #215, which is where a deep recursion is most plausible. The
QtConcurrent call is per-*pool-thread*, not per-task: it is idempotent (a
`thread_local` pointer test), so a pool thread is covered from its first CAMP
task onward and pays nothing after that.

**What is still not covered, stated as narrowly as it is true.** rclcpp's
*spawned* executor workers and the `tf2_ros::TransformListener` thread, because
rclcpp offers no thread-entry hook. Only the stack-overflow SIGSEGV class, only
on those threads; the `sigaction()` handlers remain process-wide for everything
else. Recorded under Follow-ups with the two mechanisms that could close it and
why the available one (a `pthread_create` interposer) is a worse trade than the
gap.

**Three guards, each verified non-vacuous by making it fail.**

- `check_worker_alt_stacks` (new) — fails the test run when a thread entry point
  has no `install_thread_alt_stack()` call in its file. Nothing about writing a
  new `QtConcurrent::run(...)` announces the requirement, and the failure is
  invisible: it builds, it tests green, and the only symptom is a crash that is
  never reported, on a boat, months later. Verified by deleting the call from
  `grid_map.cpp` and watching it fail. Deliberately coarse (a per-file count,
  not call-graph resolution) and fail-closed; the reasoning is on the script.
- `check_crash_lib_deps` (new) — asserts `libcamp_crash`'s `DT_NEEDED` list has
  no Qt / ROS / GDAL / `marine_*` entry, i.e. the boundary this whole split
  exists to hold. That boundary erodes one plausible `#include` at a time, so it
  is asserted against the built `.so` rather than trusted. Verified by pointing
  it at `libcamp_map.so`, which fails with seven entries.
- `check_camp_exports` (existing, **repaired**) — it looked for any `camp_crash`
  symbol in the executable's dynamic symbol table as proof that
  `ENABLE_EXPORTS` was still on. Once the handler moved to a library, those
  symbols became *undefined imports*, which every dynamically linked executable
  carries: the guard would have passed with `ENABLE_EXPORTS` deleted. It now
  requires a **defined** (non-`UND`) `crash_log_path` symbol — the function that
  stays in the executable precisely because it needs rclcpp. Two related fixes
  fell out: `readelf` needs `-W`, because without it the symbol name is elided
  to `_ZN10camp_crash1[...]` — short enough to still satisfy a substring match,
  long enough to hide which symbol matched. Verified in both directions.

### Undocumented behavior now recorded

- `install_crash_handlers(const std::string&)` silently drops a crash path
  longer than `PATH_MAX`; the handlers stay installed and degrade to
  stderr-only. Deliberate — an over-long path must not keep CAMP from starting —
  but it is silent, and a `$ROS_LOG_DIR` that long would be pathological.
- `check_camp_exports` needs `readelf` or `llvm-readelf` on the build host. As
  of Round 2 a host without either **fails** that test rather than skipping it.

### Considered and declined: raw addresses plus `/proc/self/maps`

Raised after Round 2's entry was written: since `backtrace_symbols_fd()` can
only ever emit *mangled* names, dumping raw addresses plus `/proc/self/maps`
would let an engineer resolve frames offline with `addr2line`, without
`ENABLE_EXPORTS` and without exposing CAMP to symbol interposition.

Declined, on a verified premise: `backtrace_symbols_fd()` **already** prints the
object-relative offset for every frame it cannot name — `binary(+0x11bb)
[0x5586...]` — and that `+0x11bb` is exactly what `addr2line -e <binary>`
consumes. Confirmed by direct experiment, with and without `-rdynamic`. So the
offline path this suggestion wants is available today and needs no maps dump,
while dropping `ENABLE_EXPORTS` would cost the inline, readable names in the
field log — which is the operator-facing point of the feature. The
symbol-interposition tradeoff is recorded next to the flag in `CMakeLists.txt`
and stays revisitable if a plugin ever misbehaves.

## Follow-ups (not #217)

- ~~**Alternate signal stacks for `camp_map`'s threads.**~~ **Done in this PR**,
  round 3 — the handler was promoted into its own `camp_crash` library, which
  `camp_map` links, so `GraphThread` and every QtConcurrent worker entry point
  now calls `install_thread_alt_stack()`. See Revisions round 3.
- **Alternate signal stacks for rclcpp's own threads.** The
  `MultiThreadedExecutor`'s *spawned* workers and the
  `tf2_ros::TransformListener` thread still have none, because rclcpp exposes no
  thread-entry hook. The only mechanisms that would close it are an upstream
  hook or a `pthread_create` interposer; the latter is a bad trade in a
  diagnostics feature (it changes thread creation for every library in the
  process to buy one crash class). Residual exposure: *stack-overflow* SIGSEGV
  on those threads only; every other crash class is reported, on every thread.
- **Operator manual.** `docs/camp_user_manual.md` has no troubleshooting
  section; where to find `camp_crash_<pid>.log` and how to read it is operator
  knowledge that belongs there.

## Estimated Scope

Single PR. Nineteen files (7 new source/cmake, 1 new test, 11 edited), no
cross-repo or cross-layer coordination. Grew from nine at round 3, when the
worker-thread gap was closed by promoting the handler into a library instead of
being carried as a follow-up.
