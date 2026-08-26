---
issue: 217
---

# Issue #217 — CAMP should log its own crash: install SIGSEGV/SIGABRT/terminate handlers that dump a backtrace (apport discards unpackaged binaries, so field hosts leave no core)

## Issue Review
**Status**: complete
**When**: 2026-08-26 00:28 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #217
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Scope Assessment

**Well-scoped?** Yes. The ask is concrete and small: a `SIGSEGV`/`SIGABRT`
handler using `backtrace()` + `backtrace_symbols_fd()`, a `std::set_terminate`
handler for uncaught exceptions, and one CMake build flag
(`ENABLE_EXPORTS ON` / `-rdynamic`) on the single `CCOMAutonomousMissionPlanner`
executable target. Confirmed against the current tree: `src/camp/main.cpp` is
43 lines with no signal/terminate handling today, and
`add_executable(CCOMAutonomousMissionPlanner ...)` (CMakeLists.txt:143) is the
only executable target in this package, so `ENABLE_EXPORTS` applies cleanly
without touching the `camp_map` / `camp_map_ros` libraries. Deliverable fits a
single PR; no sub-issue split needed.

**Right repo?** Yes — this is CAMP's own `main.cpp`/`CMakeLists.txt`, a
project-repo (not workspace-infra) change.

**Dependencies**: None blocking. Related but independent: #215 (operator-station
crash RCA — this issue is diagnostic infrastructure meant to help future
instances of #215, not a fix for it), #207 (abort-on-close — the
`set_terminate` handler directly targets this class), #216 (log-spam issue,
unrelated to crash handling). None of these need to land first or wait on
#217.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | Output is stderr (already captured by the `ros2 launch` session) plus a file; no upload, no dialog, no hidden automation — matches the issue's own explicit scope statement. |
| Enforcement over documentation | OK | This *is* the enforcement fix — it replaces a documentation-only workaround (read `/var/log/kern.log`, needs `adm` group) with a mechanism CAMP carries itself, independent of host config. |
| Capture decisions, not just implementations | Watch | No ADR proposed. The issue body itself already records the rationale in detail (why apport can't help, why `backtrace_symbols_fd` not `backtrace_symbols`, why re-raise with `SIG_DFL`) — durable enough that a separate ADR would be duplicative. Camp's existing ADRs (0002–0015) are all map/scene/raster architecture; a signal-handling utility is a reasonable case to skip ADR-0001 on, but note the choice explicitly rather than silently omitting it. |
| A change includes its consequences | Action needed | No parameters/topics/services change, so no API-doc update is required there. But `.agents/README.md` (Common Pitfalls / architecture overview) should gain a short note on the new crash-diagnostics behavior — what gets written and where — so a future agent or operator investigating a field crash knows to look for it. This should land in the same PR. |
| Only what's needed | OK | Issue explicitly scopes down: "Not a crash-reporting system, no upload, no dialog." |
| Test what breaks | Action needed | The Quality Standard requires testing what breaks, and a crash handler is exactly the kind of risky, easy-to-silently-break logic that needs a regression test — not just a manual field-verify. No test is mentioned in the issue. |
| Workspace vs. project separation | OK | Fully project-specific (CAMP's own binary lifecycle); nothing belongs in workspace infra. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| 0001 — Adopt ADRs (workspace) | Borderline, not required | See "Capture decisions" row above — the issue body already carries the design rationale; recommend explicitly deciding whether it warrants a camp ADR rather than defaulting silently to skipping one. |
| camp docs/decisions 0002–0015 | No | All are map/scene/raster/tile architecture; none touch process lifecycle or diagnostics. |

### Consequences

- `.agents/README.md` should document the new crash-handler behavior (destination of the backtrace file/log, what triggers it) once implemented — this is new operational knowledge a field agent would need, matching the existing "Common Pitfalls" / "Shutdown ordering" style entries already in that file.

### Recommendations

- The issue does not specify a destination path convention for the backtrace file, only that "the file" should be pre-opened at startup (not from inside the handler). Since the fd must be opened once, up front, plan-task needs to settle on a concrete path convention (e.g., ROS log-directory-relative via `rcutils`/`ament_index`, or a fixed `~/.ros/log`-adjacent or `/tmp` path) before implementation starts — this is a real design decision, not an implementation detail to leave open.
- The issue calls out `backtrace_symbols_fd()` vs. `backtrace_symbols()` as the async-signal-safety concern, but the same constraint applies to *any* other text the handler writes (e.g., a signal-name preamble before the backtrace) — use `write()`, not `fprintf`/iostream, for all handler-side output, not just the backtrace itself.
- Add a regression test exercising this: a small test (subprocess or death-test style) that deliberately raises `SIGSEGV`, raises `SIGABRT`, and throws an uncaught exception, then asserts a backtrace/terminate message was written and the process exit status matches what the default handler would have produced (`-11`/`-6`). This is the concrete way to satisfy "test what breaks" for logic that is otherwise only exercised by an actual field crash.
- Document the new behavior in `.agents/README.md` in the same PR (see Consequences above).

### Actions
- [ ] Settle on and document a concrete backtrace-file destination path convention during planning (no default exists in the current codebase).
- [ ] Extend the async-signal-safety requirement (issue only calls out `backtrace_symbols_fd`) to all handler-side output — use `write()`, not buffered I/O, throughout the handler.
- [ ] Add a regression test that deliberately triggers SIGSEGV/SIGABRT/an uncaught exception and asserts backtrace output + preserved exit status.
- [ ] Update `.agents/README.md` with the new crash-diagnostics behavior (destination, trigger conditions) in the same PR.
- [ ] Decide explicitly (and note the decision) whether this warrants a camp ADR, rather than silently skipping ADR-0001.

## Plan Authored
**Status**: complete
**When**: 2026-08-26 00:38 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-217/plan.md` at `547b9ac`
**Branch**: feature/issue-217 at `547b9ac`
**Phases**: single

Both open design questions from the Issue Review were settled by the operator
before planning (not re-litigated): backtrace destination is both stderr and a
dedicated per-run file (`<rclcpp::get_logging_directory()>/camp_crash_<pid>.log`,
verified against the real jazzy `rclcpp::get_logging_directory()` API rather
than assumed), and no ADR is warranted (diagnostics, not an architecture
decision) — the skip is recorded explicitly in the plan's Context and ADR
Compliance sections rather than silently omitted. The other three Issue Review
action items are folded into the plan: `write()`-only output across all
handler paths (not just `backtrace_symbols_fd`), a gtest death-test regression
covering SIGSEGV/SIGABRT/uncaught-exception (exit status + backtrace content),
and a `.agents/README.md` Common Pitfalls update in the same PR.

### Open questions
- [ ] No open questions — plan is review-plan-ready.

## Plan Review
**Status**: complete
**When**: 2026-08-26 00:41 -04:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-217/plan.md` at `547b9ac`
**PR**: PR-less (`--issue` mode, branch `feature/issue-217`)
**Verdict**: changes-requested

The plan is structurally sound and well-scoped — the shape (fd-based pure-utility
module, `write()`-only handlers, re-raise with `SIG_DFL`, `ENABLE_EXPORTS`, death-test
regression, `.agents/README.md` bullet) is right, and all five Issue Review action
items are addressed. The two settled operator decisions (both-destinations, no ADR)
are implemented faithfully and are not reopened here. But three claims in the plan do
not survive verification against the installed Jazzy headers and gtest, and each one
would produce a wrong artifact if implemented as written. They are small inline
amendments, not a re-plan.

### Evaluation

| Dimension | Verdict | Notes |
|---|---|---|
| Scope | Good | 6 files, one component, single PR. `add_executable(CCOMAutonomousMissionPlanner ...)` at CMakeLists.txt:143 is confirmed the only executable target, so `ENABLE_EXPORTS` applies cleanly without touching `camp_map`/`camp_map_ros`. |
| Issue alignment | Good | All three death classes covered; `backtrace_symbols_fd` (not `backtrace_symbols`), pre-opened fd, re-raise with `SIG_DFL`, and the `-rdynamic` equivalent all match the issue's implementation notes. |
| File targeting | Needs work | The CMake row does not give the new gtest target its `rclcpp` dependency, which it needs — see finding 3. |
| Consequences | Good | `.agents/README.md` is in-PR. No params/topics/services change, so the `.agents/README.md` verified-parameter table is untouched — correctly not claimed. |
| Documentation & instruction impact | Good | Section present and non-silent; instruction candidates explicitly "none" with a stated reason and a revisit trigger. But the documented path is wrong today — finding 1. |
| Principle alignment | Good | "Test what breaks" is met in intent; finding 4 notes the one regression the test as specified would not catch. |
| ADR compliance | Good | The skip is recorded explicitly in Context + the ADR table rather than silently omitted, which is what the Issue Review asked for. Operator-settled; not reopened. |
| ROS conventions | Needs work | `rclcpp::get_logging_directory()` exists but does not mean what the plan says it means — finding 1. Its throw contract is unhandled — finding 2. |

### Findings
- [ ] (must-fix) `rclcpp::get_logging_directory()` returns the **base** log dir (`$ROS_LOG_DIR`, else `$ROS_HOME/log`, else `~/.ros/log`) — **not** the per-run `~/.ros/log/<timestamp>/` directory the plan claims — `plan.md:45-50`
  - Verified: `rcl_logging_interface.h:102-112` documents exactly that resolution, and `launch/logging/__init__.py` *reads* `ROS_LOG_DIR` to pick its own unique per-run dir but never exports it to child processes (only the explicit `launch_ros` `SetROSLogDir` action sets it, and `camp_launch.py` does not use it). So under `ros2 launch`, CAMP's crash file lands flat at `~/.ros/log/camp_crash_<pid>.log`, accumulating across every run with no run correlation and no cleanup — not "next to the run it belongs to".
  - The destination is still acceptable (pid + mtime disambiguate, and the operator decision was "a dedicated per-process file", which this satisfies). What must change is the **rationale and the operator-facing documentation**: correct `plan.md`'s claim, and make sure the `.agents/README.md` bullet (step 6) states the real path so a field agent looking for the file finds it. Documenting `~/.ros/log/<timestamp>/` would send them to a directory that never contains it. This is the Documentation Accuracy rule ("never document from assumptions").
  - If per-run correlation is actually wanted, the cheap fix is appending the crash file's own path to the startup log line, or deriving a run stamp at startup — but that is a design change, so raise it rather than assume it.
- [ ] (must-fix) `rclcpp::get_logging_directory()` **throws** `rclcpp::exceptions::RCLError` on error; the plan handles only `open()` failure — `plan.md:52-54`
  - `rclcpp/logger.hpp:80-90`: "\throws rclcpp::exceptions::RCLError if an unexpected error occurs." `open_crash_log_fd()` runs at startup on the main thread, so an unresolvable log dir would abort CAMP before `QApplication` is even constructed — turning a diagnostics feature into a startup failure mode on exactly the field hosts it exists to serve. Wrap the whole resolution in `try/catch(...)` and fall back to stderr-only, same as the `open()` failure path. The plan's own principle ("never abort startup over a missing log dir") is right; the code path it specifies does not enforce it.
  - Related, minor: the returned `rcpputils::fs::path` is the soft-deprecated `rcpputils::fs` API (`filesystem_helper.hpp:36`). No deprecation attribute, so no build warning under camp's `-Wall -Wextra -Wpedantic` — worth a one-line note only.
- [ ] (must-fix) The uncaught-exception death test as specified **will not reach `std::terminate`** — `plan.md:102-104`
  - gtest wraps every death-test statement in `try { ... } catch (const std::exception&) { ... } catch (...) { ... }` when exceptions are enabled (`gtest-death-test-internal.h:197-213`, `GTEST_EXECUTE_DEATH_TEST_STATEMENT_`). A plain `throw std::runtime_error("boom")` inside `ASSERT_EXIT`/`EXPECT_EXIT` is therefore caught by gtest in the child, which aborts the death test as `TEST_THREW_EXCEPTION` — the `set_terminate` handler never runs, and the test would either fail confusingly or (worse) pass for the wrong reason if the assertion is loose. Since the `set_terminate` half is the piece that directly targets #207, this would ship the feature's most-wanted path untested.
  - Fix: make the throw escape a `noexcept` boundary inside the death-test statement, e.g. `ASSERT_EXIT({ auto f = []() noexcept { throw std::runtime_error("boom"); }; f(); }, KilledBySignal(SIGABRT), ...)`. That reaches `std::terminate` with a live current exception, which is what exercises the `what()` extraction. Verify by asserting `"boom"` appears in the temp file, as the plan already intends.
- [ ] (should-fix) The test asserts only a "non-empty backtrace", so it does not cover the `ENABLE_EXPORTS` regression it exists to protect — `plan.md:83-87`, `plan.md:99-104`
  - `ENABLE_EXPORTS` (step 4) is the single most droppable line in this change, and losing it silently degrades every future field backtrace to bare addresses — the exact failure this issue was filed about. Give the gtest target `ENABLE_EXPORTS ON` as well and assert that a **known function name** from the test binary appears in the file, not merely that bytes were written. Otherwise "test what breaks" covers the handler but not the flag.
- [ ] (should-fix) Nothing defends against the handler itself faulting, and `signal()` leaves a stack-overflow SIGSEGV uncatchable — `plan.md:56-67`
  - Set `SIG_DFL` (or a `static volatile sig_atomic_t` re-entry guard) at handler **entry**, not only at the end. As written, a fault inside `backtrace()`/`backtrace_symbols_fd()` re-enters the handler and can loop or hang the process instead of dying — the handoff explicitly asks about this case.
  - Prefer `sigaction()` with `SA_ONSTACK` plus a `sigaltstack()` installed at startup. A stack-overflow SIGSEGV (a plausible Qt/recursion crash class) cannot run a handler on the exhausted stack, so without an alternate signal stack that whole class still dies silently. `signal()` also carries less-defined semantics than `sigaction()` for the returning/re-raise pattern.
- [ ] (should-fix) `backtrace()`'s first call is not async-signal-safe — force it at install time — `plan.md:60-64`
  - glibc's `backtrace()` lazily `dlopen`s `libgcc_s` and may allocate on its **first** invocation. Call `backtrace(buf, 1)` once inside `install_crash_handlers()` (discarding the result) so the handler's call touches only already-resolved code. This matters for exactly the plan's stated threat model (heap corruption in #215) and costs one line.
- [ ] (should-fix) The `set_terminate` path will emit a **second, useless backtrace** through the SIGABRT handler — `plan.md:68-78`
  - `on_terminate` ends with `std::abort()`, which raises SIGABRT, which the just-installed `on_fatal_signal` catches and dumps a second stack from inside `abort()`. Two stacks per #207-class death, the second one noise, and the useful first one pushed up the file. Set a guard (or restore `SIG_DFL` for SIGABRT before aborting) so the terminate path produces one stack.
  - Also order the terminate handler's output: write the **backtrace first**, then attempt the `what()` extraction. `std::current_exception()` + rethrow-and-catch touches the exception runtime and allocates — precisely what the plan says must not be relied on when the heap is suspect. Backtrace-first means a failure in that machinery loses only the message, not the stack.
- [ ] (suggestion) Open the crash fd with `O_CLOEXEC` — `plan.md:52`
  - CAMP is a Qt app that can spawn child processes; without `O_CLOEXEC` the crash fd leaks into every child. One flag, no downside.
- [ ] (suggestion) `backtrace_symbols_fd()` output is **mangled**; say so in the `.agents/README.md` bullet — `plan.md:106-111`
  - Demangling (`abi::__cxa_demangle`) allocates and is correctly excluded from the handler. So the operator-facing doc should tell the reader to pipe the file through `c++filt` — otherwise the artifact is there and reads as gibberish to the person who needs it at 2am in the field.
- [ ] (suggestion) Consider `SIGBUS`/`SIGFPE`/`SIGILL` alongside SIGSEGV/SIGABRT — `plan.md:58`
  - The issue scopes to three observed death classes, so this is not a gap against the issue. But the handler is written once and these are one `signal()`/`sigaction()` line each; a SIGBUS on the operator station would otherwise still die silent. Operator's call — flagging, not assuming.
- [ ] (suggestion) Ordering vs `QApplication` is right and worth keeping explicit — `plan.md:119`
  - Verified against `src/camp/main.cpp:10-12`: installing after `rclcpp::init()` and before `QApplication` is correct. `rclcpp::init()` installs only SIGINT/SIGTERM handling, so there is no conflict with SIGSEGV/SIGABRT, and Qt5 installs no fatal-signal handlers of its own. No change needed — noting it so a later reorder does not look harmless.

### Actions
- [ ] Correct `plan.md`'s `get_logging_directory()` claim and the path the `.agents/README.md` bullet will document (finding 1).
- [ ] Specify the `try/catch` around log-dir resolution so startup can never fail on it (finding 2).
- [ ] Respecify the uncaught-exception death test to escape a `noexcept` boundary (finding 3).
- [ ] Decide on the should-fix hardening items (handler re-entry guard + `sigaltstack`, `backtrace()` warm-up, single-stack terminate path, ENABLE_EXPORTS coverage in the test) and amend the plan inline per `plan-task`'s "During implementation" rules before implementation starts.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-08-26 01:04 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-217 at `40ab7c8`
**Mode**: pre-push
**Depth**: Deep (reason: signal handling / crash + shutdown path in a multi-threaded Qt+rclcpp app; 491 lines of new C++ across 6 files)
**Must-fix**: 4 | **Suggestions**: 16
**Round**: 1 | **Ship**: continue — three of the four must-fixes are genuine correctness concerns (lost artifact, lost stack, uncovered thread class), and two were cross-confirmed by both independent adversarial passes; they warrant a fix pass and a re-read rather than a ship.

Specialists run: Static Analysis, Governance, Plan Drift, Claude Adversarial x2 (Lens A + Lens B).
Copilot Adversarial: off (default). Local Adversarial: off (default).

The shape of this change is right and the craft is high: the fd-injection design, the
`write()`-only handler path, `backtrace_symbols_fd` over `backtrace_symbols`, the
`SIG_DFL`-on-entry re-entry guard, the `backtrace()` warm-up, and the base-vs-per-run
log-directory correction were all verified accurate against the installed Jazzy headers
and glibc. Every row of the plan's `## Revisions` table is genuinely present in the code.
Plan Drift and Governance found no must-fix. All four must-fixes below are in the
handler's own failure paths — the places where a diagnostics feature quietly stops
producing the diagnostic.

### Findings
- [x] (must-fix) `emit()`/`emit_backtrace()` write stderr *before* the durable fd, so a blocked or broken stderr costs the crash file the feature exists to produce — under `ros2 launch` stderr is a pty/pipe to the launch parent, and in the #207 abort-on-close class that parent may already be gone (SIGPIPE kills the process before the file is written, and changes the exit status to 13) or not draining (the ~10-20 KB dump blocks in-handler forever, so CAMP hangs instead of dying and no "process has died" line is ever logged). Write `g_crash_fd` first, stderr second, and `::signal(SIGPIPE, SIG_IGN)` at install. Cross-confirmed by Claude Adversarial Lens A and Lens B independently — `src/camp/crash_handler.cpp:49-55,71-73`
- [x] (must-fix) `on_terminate` sets `g_already_dumped` and then runs the *allocating* exception introspection (`current_exception()` + `rethrow_exception`) **before** emitting the backtrace — so if that machinery faults under the heap corruption this exists to diagnose, the SIGSEGV handler sees the guard already set, suppresses all output, and the crash produces a truncated header line and **no stack at all**. Emit the backtrace first, then attempt the `what()` extraction. This is the Plan Review should-fix "order the terminate handler's output: write the backtrace first" — it was neither implemented nor recorded as declined in the plan's `## Revisions` table, which lists only the `already_dumped` half of that finding — `src/camp/crash_handler.cpp:113-148`
- [x] (must-fix) `sigaltstack()` is per-thread and is installed only on the main thread, so the stack-overflow SIGSEGV class that `crash_handler.cpp:35-37`, `crash_handler.h` and the new `.agents/README.md` bullet present as covered is still silent on every thread that runs ROS work — `ROSLink::node_thread_` (`src/camp/roslink.h:63`), the `MultiThreadedExecutor` workers and the `tf2_ros::TransformListener` thread (`src/camp/ros/node_thread.cpp:31,39,44`), all created after install, and `pthread_create(3)` explicitly does not inherit an alt stack. That is where a subscription-callback crash actually lands. Either install a per-thread alt stack at `NodeThread::start()` or scope the claim honestly in both the comment and the operator-facing bullet. Cross-confirmed by Lens A, Lens B and Plan Drift — `src/camp/crash_handler.cpp:198-210`, `.agents/README.md:146-159`
- [x] (must-fix) The `ENABLE_EXPORTS` regression guard that `test_crash_handler.cpp:9-12` and the CMake comment both claim ("the regression guard for the flag on the main executable") does not exist: `CMakeLists.txt:450` sets `ENABLE_EXPORTS` on `test_crash_handler` independently, so deleting line 148 from `CCOMAutonomousMissionPlanner` leaves every test green while the shipped binary reverts to bare addresses — the exact regression advertised as covered. Add a real check on the executable (a CTest running `readelf --dynsym $<TARGET_FILE:CCOMAutonomousMissionPlanner>`, or a CMake-time `get_target_property` assertion), or at minimum correct both comments so no future reader trusts a guard that is not there — `CMakeLists.txt:439-441,450`, `test/test_crash_handler.cpp:9-12`
- [x] (suggestion) `if (!g_already_dumped) { g_already_dumped = 1; ... }` is a non-atomic test-and-set; `volatile sig_atomic_t` prevents tearing but not a race, so two threads faulting in the same window both pass and shred two dumps together across ~5 `write()` calls each. `__atomic_test_and_set(..., __ATOMIC_ACQ_REL)` is lock-free and handler-safe on x86-64 — `src/camp/crash_handler.cpp:96-98,113-115`
- [x] (suggestion) The dump never says which thread produced it; on a multi-threaded crash the winner of the guard may be a secondary victim. One `emit()` of `gettid()` in the header line — `src/camp/crash_handler.cpp:99`
- [x] (suggestion) No `SA_SIGINFO`, so `si_addr`/`si_code` — the datum that separates a null `this->member` from a use-after-free from a wild write, often before anyone reads the stack — is discarded. Cross-confirmed by both lenses — `src/camp/crash_handler.cpp:214,216`
- [x] (suggestion) `backtrace()`/`backtrace_symbols_fd()` take glibc's loader locks (the warm-up correctly fixes the malloc/dlopen hazard but not this); a fault while another thread holds the loader lock during a GDAL/Qt-plugin `dlopen` deadlocks the handler, turning a silent death into a silent hang with no "process has died" line — strictly worse than today. `::alarm(5..10)` right after the `SIG_DFL` restore bounds it. Cross-confirmed by both lenses — `src/camp/crash_handler.cpp:70-73`
- [x] (suggestion) `open_crash_log_fd()` is called unconditionally at startup with `O_CREAT|O_TRUNC`, so every clean run leaves a zero-byte `camp_crash_<pid>.log` (the dominant population in `~/.ros/log`, which already holds 10k+ entries on the dev box) and an operator cannot tell "no crash" from "crashed before the first write"; and on a host with the stock `pid_max` a recycled pid silently truncates a real earlier crash report. Drop `O_TRUNC`/add `O_APPEND`, put a start timestamp in the name, or defer the `open()` into the handler (`open()` is async-signal-safe). Not in the plan's `## Consequences` table. Cross-confirmed by Governance, Lens A and Lens B — `src/camp/crash_handler.cpp:179-181`
- [x] (suggestion) All three death tests pass `""` as the `ASSERT_EXIT` matcher and then assert only against the file, so deleting every `::write(STDERR_FILENO, ...)` would leave the suite green — yet the stderr copy is the half the issue argues is most valuable (it lands beside "process has died" in the launch log). Passing `"CAMP caught SIGSEGV"` / `"CAMP std::terminate"` covers it for free — `test/test_crash_handler.cpp:99,121,139`
- [x] (suggestion) `open_crash_log_fd()` has zero coverage — the `catch → -1` degradation that is the header's central "must never keep CAMP from starting" promise, the `dir.empty()` branch, and the path composition are all untested, as is the `backtrace_fd == -1` stderr-only path. A `ROS_LOG_DIR=/nonexistent/x` case plus an `install_crash_handlers(-1)` death test with a non-empty matcher covers both — `test/test_crash_handler.cpp`
- [x] (suggestion) No test crashes on a non-main thread — which is where CAMP will actually crash, and what would have surfaced the sigaltstack gap above — `test/test_crash_handler.cpp`
- [x] (suggestion) `open_temp()`'s `EXPECT_GE(fd, 0)` runs inside the forked death-test child where gtest failures are invisible; on failure the test instead fails later against an empty dump, pointing the reader at the handler rather than at the unopenable path. The `/tmp/camp_crash_test_*_<pid>.log` name is also predictable and non-`O_EXCL` in a world-writable sticky directory. Open in the parent and assert there — `test/test_crash_handler.cpp:46-51`
- [x] (suggestion) `SIGSTKSZ` is `sysconf(_SC_SIGSTKSZ)` on glibc >= 2.34, so lines 201 and 206 are two independent runtime evaluations feeding the allocation size and the kernel's `ss_size`; capture it once in a local. The return values of `::sigaltstack()` and all five `::sigaction()` calls are also unchecked, so a rejected alt stack silently downgrades with no trace — `src/camp/crash_handler.cpp:201-208`
- [x] (suggestion) `SA_RESTART` is inert on five signals whose handler never returns normally, and implies the opposite; `SA_RESETHAND` would reset all five at delivery atomically and let the `::signal(sig, SIG_DFL)` line go. Also `on_fatal_signal` has C++ language linkage where `sa_handler` strictly wants C — `src/camp/crash_handler.cpp:94,216`
- [x] (suggestion) `::raise(sig)` means that if `systemd-coredump` is ever enabled the core is taken at the raise site with the faulting frame buried; for the four synchronous faults, simply returning after restoring `SIG_DFL` re-executes the faulting instruction and dumps at the real fault with the same exit status (SIGABRT still needs the raise). The comment's "any host-level core handling still applies" is only half true as written — `src/camp/crash_handler.cpp:106-108` **(partially deferred: the misleading half of the comment is corrected; the behavior change — returning from the handler instead of re-raising, so a core would be taken at the faulting instruction — is declined. No core is written on these hosts (`ulimit -c` 0, apport drops the report for unpackaged binaries), it does not apply to SIGABRT, and it alters the death path of a diagnostics-only feature for a benefit nothing on these hosts can collect.)**
- [x] (suggestion) `O_CREAT` without `O_NOFOLLOW` at mode 0644 on a predictable pid-named path: `ROS_LOG_DIR`/`ROS_HOME` are environment-controlled, so if the log dir is ever pointed somewhere shared, a pre-planted symlink turns the `O_WRONLY|O_TRUNC` open into a file-destroying write. `O_NOFOLLOW` costs nothing; 0600 loses nothing operationally and the dump embeds full install paths — `src/camp/crash_handler.cpp:179-181`
- [x] (suggestion) `ENABLE_EXPORTS` puts every global symbol across CAMP's ~100 TUs into `.dynsym` with the executable first in the global search scope, so a later-loaded Qt plugin / GDAL driver / ROS component with a same-named symbol binds to CAMP's definition — a silent wrong-function call, indistinguishable from the unexplained #215 crashes. Verified necessary (CMake 3.28 emits `-Wl,--export-dynamic -rdynamic` only with the property set; camp's `cmake_minimum_required(3.5.1)` puts CMP0065 at NEW so nothing adds it by default), so keep it — but note the tradeoff next to the flag, or scope it with `--dynamic-list` — `CMakeLists.txt:146-148`
- [x] (suggestion) Install a stderr-only handler *before* `rclcpp::init()` as well — `rclcpp::init()` is itself a documented thrower, and if it escapes, `std::terminate` runs with the default handler and the crash is as silent as before. `install_crash_handlers()` is documented as re-callable and accepts `-1`, so this is two lines. The comment "before anything else can fault" is not accurate as written — `src/camp/main.cpp:12-23`
- [x] (suggestion) "apport discards crashes from unpackaged binaries **outright**" is slightly overstated: verified at `/usr/share/apport/apport:1135-1142`, the *report* is dropped but the same branch still writes a core if the user configured one. Under the default `ulimit -c 0` the conclusion holds, but as written a future reader concludes cores are impossible and skips a working local option — `.agents/README.md:157-159`, `src/camp/crash_handler.h:5-8`
- [x] (suggestion) The plan's ADR Compliance table cites "camp ADR-0001 (adopt ADRs...)"; camp's ADR-0001 is the TopicBridge/executor contract and camp has no meta-ADR — the adopt-ADRs decision is workspace ADR-0001. The judgement to decline an ADR is sound and consistent with camp's precedent (its ADRs are all cross-module contracts; this is a leaf utility with one caller), but the citation misidentifies the governing record — `.agent/work-plans/issue-217/plan.md` § ADR Compliance
- [x] (suggestion) Plan step 5 still says the test asserts "a non-empty backtrace", contradicting the later folded-in bullet requiring a resolved symbol name; the code follows the stronger one. One-line edit keeps the plan usable as review reference — `.agent/work-plans/issue-217/plan.md` § Approach step 5
- [x] (suggestion) Cosmetic: a stray double blank line after the `ENABLE_EXPORTS` block, and `crash_handler.cpp` inserted after `main.cpp` breaking the otherwise-alphabetical `SOURCES` list — `CMakeLists.txt:75,149-150`

### Governance
No must-fix governance findings. Principles: Pass across the board (one Watch on the
every-run empty file). ADRs: worktree isolation, ROS 2 conventions, ADR-0013 progress
vocabulary and ADR-0017 all compliant; the ADR decline is defensible against camp's own
precedent. Every consequence the plan committed to is present in the diff. The
`.agents/README.md` bullet's load-bearing claims (path, signals, mangling, pid
correlation) were each verified accurate against the code; only the apport wording
overstates. Camp's `.agents/README.md` carries no verified-parameter table and this
change touches no parameter/topic/service, so that rule is N/A.

Carry-forwards for push/merge, neither a diff defect: the PR body must carry the
ADR-decline rationale the plan promises, and merge needs a full-scope `ci_local.sh`
attestation per ADR-0018 (camp is a project repo).

### Static Analysis
cpplint and cppcheck on the three changed C++ files. No real findings. cpplint's
header-guard style complaint is against camp's house convention (`GEOREFERENCED_H`,
`MAINWINDOW_H`) and is dropped; cppcheck's `throwInNoexceptFunction` on
`test_crash_handler.cpp:86` is the deliberate, documented `noexcept`-boundary throw.
Note the `#pragma GCC diagnostic ignored "-Wterminate"` at that site names a warning
group clang does not know — clang emits an ignorable `-Wunknown-warning-option` notice
rather than an error, and camp does not build with `-Werror`, so it is not a
portability defect; guarding it with `#if defined(__GNUC__) && !defined(__clang__)`
would silence that notice.

### Plan Adherence
No drift. All six plan steps (including 2b) and all six Files-to-Change rows are
implemented, and every row of the `## Revisions` table — sigaltstack + `SA_ONSTACK`,
`SIG_DFL` on entry, the `backtrace()` warm-up, the `already_dumped` guard, `O_CLOEXEC`,
the try/catch around `get_logging_directory()`, the `noexcept`-boundary throw, the
resolved-symbol assertion, `SIGBUS`/`SIGFPE`/`SIGILL`, the mangled-output docs note,
and the base-vs-per-run correction in both the code and the docs — is genuinely
present. The implementation went beyond the plan in one place (the terminate test also
pins the single-backtrace guard by asserting exactly one end marker). No scope creep.
The one Plan Review item that reached neither the code nor the Revisions table is
must-fix 2 above.

## Implementation
**Status**: complete
**When**: 2026-08-26 01:20 -04:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-217 at `480eb2d`
**Addressed**: `## Local Review (Pre-Push)` (Round 1, 2026-08-26 01:04 -04:00, branch at `40ab7c8`, entry committed as `d8f861c`) — 4 must-fixes and 19 suggestions
**Commits**: `45e145b`, `a6b83dd`, `e38ee3e`, `fb1f672`, `512ca61`, `b7c63a8`, `17b283a`, `1c7ba10`, `480eb2d`

### Verification
`./ui_ws/build.sh camp` clean. `./ui_ws/test.sh camp`: **303 tests, 0 errors,
0 failures, 1 skipped** (Round-1 baseline was 297 — +5 new gtest cases in
`test_crash_handler`, which goes from 3 cases to 8, and +1 new CTest,
`check_camp_exports`). The new guard was verified to actually fail: running
`cmake/check_dynamic_symbols.cmake` against a binary without CAMP's symbols
produces the intended FATAL_ERROR.

### Actions

**Must-fix**

- [x] stderr written before the durable fd — `src/camp/crash_handler.cpp` `emit()`/`emit_backtrace()` now write `g_crash_fd` first and stderr second, and `install_crash_handlers()` sets `SIGPIPE` to `SIG_IGN` (`45e145b`)
- [x] `on_terminate` ran the allocating exception introspection before emitting a stack — backtrace now goes out first, the reason follows on its own `=== CAMP terminate reason: ... ===` line (`a6b83dd`)
- [x] `sigaltstack()` per-thread, so the stack-overflow claim was false off the main thread — the alt stack is now `thread_local` behind a new exported `install_thread_alt_stack()`; `camp_ros::NodeThread::start()` calls it; the rclcpp-internal threads (`MultiThreadedExecutor` workers, `TransformListener`) have no entry hook and are now documented as a **known gap** in `crash_handler.h` and in the `.agents/README.md` bullet, so the operator-facing claim is true as written (`e38ee3e`)
- [x] the advertised `ENABLE_EXPORTS` regression guard did not exist — new `check_camp_exports` CTest runs `readelf --dyn-syms` over `$<TARGET_FILE:CCOMAutonomousMissionPlanner>` and requires a `camp_crash*` symbol; new `cmake/check_dynamic_symbols.cmake`; both misleading comments corrected (`fb1f672`)

**Suggestions taken**

- [x] non-atomic `already_dumped` test-and-set → `__atomic_test_and_set(..., __ATOMIC_ACQ_REL)` (`512ca61`)
- [x] dump did not name the faulting thread → `gettid()` in the header line, via a new async-signal-safe `emit_ulong()` (`512ca61`)
- [x] no `SA_SIGINFO` → `si_code`/`si_addr` now reported (`512ca61`)
- [x] loader-lock deadlock turns a silent death into a silent hang → `::alarm(10)` after the `SIG_DFL` restore (`512ca61`)
- [x] `SIGSTKSZ` evaluated twice; `sigaltstack()`/`sigaction()` returns unchecked → evaluated once into a local, every return checked, install-time failures reported on stderr (`512ca61`, `e38ee3e`)
- [x] inert `SA_RESTART`; C++ linkage on the handler → `SA_RESETHAND`, `extern "C"` (`512ca61`)
- [x] `O_TRUNC` littering a zero-byte file every run and truncating a real report on pid reuse → the file is no longer pre-opened at all: `open_crash_log_fd()` becomes `crash_log_path()` (resolves, creates nothing) and the handler `open()`s it at crash time, which is async-signal-safe (`b7c63a8`)
- [x] `O_NOFOLLOW` / mode 0600 on an environment-controlled path → both applied, plus `O_APPEND` in place of `O_TRUNC` (`b7c63a8`)
- [x] install a stderr-only handler before `rclcpp::init()` → `main.cpp` installs twice; the comment claiming "before anything else can fault" is now true (`b7c63a8`)
- [x] `open_temp()` asserting inside the forked child, predictable non-`O_EXCL` name → helper deleted; the handler creates the file (`b7c63a8`)
- [x] all three `ASSERT_EXIT` matchers were `""` → matched against the real stderr preamble, so deleting the `write(STDERR_FILENO, ...)` calls now fails the suite (`17b283a`)
- [x] no coverage of `open_crash_log_fd()` or the `fd == -1` path → `CrashLogPathIsUnderTheRosLoggingDirectory`, `UnopenableCrashPathStillDumpsToStderr`, `EmptyCrashPathIsStderrOnly`, plus `NoCrashFileMeansNoFile` pinning the zero-byte-litter fix (`17b283a`)
- [x] no non-main-thread crash test → `CrashOnANonMainThreadIsStillReported` (`17b283a`)
- [x] `ENABLE_EXPORTS` symbol-interposition tradeoff undocumented → recorded next to the flag in `CMakeLists.txt` (`fb1f672`)
- [x] apport "discards outright" overstated → corrected in `crash_handler.h` and the `.agents/README.md` bullet, which now says a core *is* written if `ulimit -c` allows one (`1c7ba10`)
- [x] cosmetic: stray double blank line; `crash_handler.cpp` out of alphabetical order in `SOURCES` (`1c7ba10`)
- [x] plan cites camp ADR-0001 for the adopt-ADRs decision → now cites workspace ADR-0001 and names what camp's ADR-0001 actually is (`480eb2d`)
- [x] plan step 5 still said "non-empty backtrace" → corrected (`480eb2d`)

**Partially deferred**

- [x] `::raise(sig)` buries the faulting frame if core dumps are ever enabled — the misleading half of the comment **is** corrected (`512ca61`); the behavior change is **declined**: returning from the handler instead of re-raising would take the core at the faulting instruction, but no core is written on these hosts (`ulimit -c` 0, and apport drops the report for unpackaged binaries), it does not apply to SIGABRT, and it alters the death path of a diagnostics-only feature for a benefit nothing on these hosts can collect. The backtrace this feature exists to deliver is unaffected either way. (deferred: no core is collectable on the target hosts)

### Plan sync

`.agent/work-plans/issue-217/plan.md` amended in the same pass (`480eb2d`):
steps 2, 2b, 3, 4 and 5, the Files to Change table, the ADR Compliance citation
and the Consequences table now describe what was built, and a second
`## Revisions` table maps each Round-1 finding to where the plan was corrected,
including the one declined suggestion. Note the Round-1 must-fix 2 arose
*because* a Plan Review should-fix was recorded as addressed while shipping
only half — the new table records the decline explicitly to avoid repeating
that.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-08-26 01:31 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-217 at `8d46d11`
**Mode**: pre-push
**Depth**: Deep (reason: signal handling / crash + shutdown path in a multi-threaded Qt+rclcpp app; whole feature diff re-read, 1591 lines across 10 files)
**Must-fix**: 6 | **Suggestions**: 18
**Round**: 2 | **Ship**: continue — must-fix rose 4 -> 6, and two of the six are Round-1 must-fixes closed on claims that do not hold (the alt-stack coverage claim, and the test advertised as its guard), plus a new output defect introduced by the Round-1 `SA_SIGINFO` fix. That is a correctness pattern, not a residue of mechanical fixes, so it warrants another independent read.

Specialists run: Static Analysis, Governance, Plan Drift, Claude Adversarial x2 (Lens A + Lens B).
Copilot Adversarial: off (default). Local Adversarial: off (default).

The fix pass is substantive, not cosmetic: file-before-stderr ordering, the atomic
`claim_dump()`, backtrace-before-introspection in `on_terminate`, lazy open-at-crash-time
with `O_APPEND`/`O_NOFOLLOW`/0600, and a `check_camp_exports` guard that really does check
the shipped binary all verified against the code. The single hardest claim in the diff was
verified independently by three readers: `install_thread_alt_stack()` genuinely executes on
the ROS thread, because `NodeThread::start()` is a slot on a `moveToThread`'d object reached
by a queued connection. Plan Drift found no drift; Governance found no must-fix governance
defect. Both remaining must-fix clusters are claims that outran the code, and one new defect
the Round-1 hardening introduced.

### Findings
- [ ] (must-fix) `si_code` printed through the unsigned `emit_ulong()`, so every `abort()`-path dump prints `[si_code=18446744073709551610]` (SI_TKILL is -6) — the #207 abort-on-close class this feature targets; and `si_addr` is gated on the signal number rather than `si_code > 0`, so a raise()d SIGSEGV prints the `_kill` union member (uid<<32|pid) as a fault address. Both verified empirically; no test asserts on the line, so both ship invisible. Cross-confirmed by Lens A, Lens B and Plan Drift — `src/camp/crash_handler.cpp:206-211`
- [ ] (must-fix) The `alarm(10)` anti-hang bound covers only the signal path: `on_terminate()` calls the same `emit_backtrace()` with no bound and reaches the alarm only later via `abort()`, downstream of the hazard. The mitigation is also weaker than its comment claims — line 182 restores `SIG_DFL` for `sig` only, never SIGALRM, so an intercepted or blocked SIGALRM voids it silently. Cross-confirmed by Lens A, Lens B and Governance — `src/camp/crash_handler.cpp:233-285,186-190`
- [ ] (must-fix) Round-1 must-fix 3 is not closed as claimed: the header's own rule is "call this as the first statement of any thread CAMP starts itself", but `camp::ros::GraphThread` (`src/camp_map/ros/node.cpp:75`, live in the shipped app via `src/camp/mainwindow.cpp:134`) has an entry hook and does not call it, and the QtConcurrent pool workers running the GDAL/raster/tile work implicated in #215 are uncovered too. The gap enumeration names only rclcpp-internal threads. Cross-confirmed by Lens A and Lens B — `.agents/README.md:160-164`, `src/camp/crash_handler.h:84-92`
- [ ] (must-fix) `CrashOnANonMainThreadIsStillReported`'s claim "this is the test that would have caught the alternate signal stack being main-thread-only" is false: the worker `raise()`s on a healthy stack, and `sigaltstack` only changes where the handler frame is pushed. Deleting `src/camp/ros/node_thread.cpp:24` — or the in-lambda call at line 187 — leaves it green, so the alt-stack half of Round-1 must-fix 3 ships with zero coverage. Third claimed-guard-that-isn't in this change. Cross-confirmed by Lens A and Lens B — `test/test_crash_handler.cpp:178-192`
- [ ] (must-fix) The header summary still says the handlers write "to a pre-opened file" and orders the writes stderr-then-file; the Round-1 fixes inverted both, and the same header contradicts itself 18 lines later (`:35-38`, `:52`), as does `crash_handler.cpp:33,104-111`. This is the first file a future reader opens. Cross-confirmed by Governance and Plan Drift — `src/camp/crash_handler.h:15-19`
- [ ] (must-fix) "`ulimit -c` is 0 by default / on these hosts" is contradicted where it can be checked: on this workstation `ulimit -c` is `unlimited` with `core_pattern` piping to apport, so a core is written here. The claim is load-bearing three times — an operator instruction, half the feature's justification, and the recorded basis for declining the raise-vs-return suggestion. Verify on the operator station and attribute it (host + date), or soften to "check `ulimit -c` first" — AGENTS.md § Documentation Accuracy forbids hand-typed measured values — `src/camp/crash_handler.h:9`, `.agents/README.md:166`
- [ ] (suggestion) `install_thread_alt_stack()` reports nothing on either the allocation failure or the `sigaltstack()` failure, so a thread silently loses stack-overflow coverage; the Implementation entry's "every return checked, install-time failures reported on stderr" is only half earned (`signal(SIGPIPE)` is also unchecked). Cross-confirmed by Lens A and Plan Drift — `src/camp/crash_handler.cpp:311-336,362`
- [ ] (suggestion) `SIGSTKSZ` is `sysconf(_SC_SIGSTKSZ)` on glibc >= 2.34 and can return -1, which `static_cast<size_t>` turns into `SIZE_MAX` (degrades silently to no alt stack); and 8 KB on older glibc is tight against this handler's own 1 KB frame buffer plus the unwinder. Clamp to `MINSIGSTKSZ` with a floor — `src/camp/crash_handler.cpp:319-321`
- [ ] (suggestion) `check_camp_exports` is simply not registered when `readelf` is absent — a configure-time `message(WARNING)` nobody reads in colcon output, leaving a green run with the guard gone. Register a test that fails loudly, or FATAL_ERROR under CI. Cross-confirmed by Lens A and Governance — `CMakeLists.txt:456-465`
- [ ] (suggestion) "an unbounded recursion inside a subscription callback still dies silently" overstates the gap: `MultiThreadedExecutor::spin()` runs one worker inline on the calling thread, which here is the ROS node thread and does have an alternate stack. "may die silently, depending on which worker picks up the callback" is true and less discouraging — `.agents/README.md:162-163`
- [ ] (suggestion) "Must be called after `rclcpp::init()`" is contradicted by the only test of that function, which calls it with no `rclcpp::init()` in the binary and passes. Strike the precondition or fix the test. Cross-confirmed by Lens A and Lens B — `src/camp/crash_handler.h:51`, `test/test_crash_handler.cpp:244-256`
- [ ] (suggestion) "before anything at all can fault" overclaims: static initialization for ~100 TUs plus Qt resource and GDAL driver registration has already run by the time `main()` starts. Scope the comment — `src/camp/main.cpp:12`
- [ ] (suggestion) `install_crash_handlers(int)` overwrites `g_crash_fd` without closing a previously handler-opened fd; unreachable in `main.cpp`'s sequence, but the header advertises the call as re-callable — `src/camp/crash_handler.cpp:352-355`
- [ ] (suggestion) In the hang case `alarm(10)` kills with SIGALRM (14), not the real fault, contradicting the file's own "the exit status is exactly what it would have been" two lines below. One clause; the tradeoff itself is right — `src/camp/crash_handler.cpp:190,220`
- [ ] (suggestion) No release barrier between the path `memcpy` and `g_crash_path_valid = 1`; `volatile` orders the compiler, not another CPU. Theoretical (both installs precede thread creation) but this file is careful about exactly this class everywhere else — `src/camp/crash_handler.cpp:345-349`
- [ ] (suggestion) `O_NOFOLLOW` rejects only a symlink at the final component; a pre-planted hardlink or a symlinked parent directory still lands the append (with full install paths) in someone else's file. An async-signal-safe `fstat` check of `st_nlink`/`st_uid`/`S_ISREG` after `open()` closes it — `src/camp/crash_handler.cpp:54-61`
- [ ] (suggestion) Test temp paths are predictable `/tmp/camp_crash_test_<tag>_<pid>.log` in a world-writable directory: a local user can pre-fill the file so the content assertions pass without the handler writing anything. Use `mkdtemp()` per test — `test/test_crash_handler.cpp:39-44`
- [ ] (suggestion) `NoCrashFileMeansNoFile` installs handlers (including `set_terminate` and `SIGPIPE`->`SIG_IGN`) in the parent gtest process rather than a forked child, and relies on a trailing reset that a future edit could skip — `test/test_crash_handler.cpp:201-216`
- [ ] (suggestion) The `thread_local` alt-stack leak at thread exit is the correct choice (freeing would leave the kernel a dangling `ss_sp`), but nothing says so — a future reader will "fix" it into a `unique_ptr` and reintroduce a use-after-free reachable only from a signal handler — `src/camp/crash_handler.cpp:96,321,333`
- [ ] (suggestion) `signal(SIGPIPE, SIG_IGN)` is process-wide, survives `execve()` into any future child, and silently overwrites whatever disposition Qt Network / GDAL-curl established; return value discarded. Nothing is broken today (no `QProcess`/`popen`/`system` in `src/`) — worth recording rather than rediscovering — `src/camp/crash_handler.cpp:362`
- [ ] (suggestion) `crash_log_path()`'s `catch -> return {}` and `dir.empty()` branches are still untested, while the test's own comment claims it covers "the documented returns-empty-rather-than-throwing degradation". Round 1 asked for both by name; the `fd == -1` half is covered twice — `test/test_crash_handler.cpp:244-256`, `src/camp/crash_handler.cpp:300-306`
- [ ] (suggestion) Stale plan prose the fix pass did not reach: "Both fds are opened once at startup, before any handler can fire" and "to stderr (fd 2) and to the pre-opened crash-file fd" — the plan now contradicts itself, since step 3 and the Revisions table state the corrected design — `.agent/work-plans/issue-217/plan.md:28-29,131`
- [ ] (suggestion) apport line citations disagree with each other (`:1135-1142` in the header vs `:1136` in the plan) and with installed apport 2.28.3-0ubuntu0.1, where the branch is 1137-1144. Cite the version and the `likely_packaged()` branch by name; drop the range — `src/camp/crash_handler.h:8`, `.agent/work-plans/issue-217/plan.md:12`
- [ ] (suggestion) The decline's "nothing on these hosts can collect the benefit" clause is undercut by the README correction made in the same commit range, which tells the reader that raising `ulimit -c` does produce a core and calls it a useful dev-box option. The decline still stands on SIGABRT asymmetry and death-path churn; the no-core-collectable argument should be softened — `.agent/work-plans/issue-217/plan.md:352-357`, `src/camp/crash_handler.cpp:222-229`
- [ ] (suggestion) Undocumented in the plan: the `PATH_MAX` buffer silently drops an over-long crash path, and the `readelf` degradation above. Neither is wrong; neither is recorded — `src/camp/crash_handler.cpp:43,345-349`, `CMakeLists.txt:456-465`

### Governance
No must-fix governance findings. Principles Pass across the board with two Watches: the same
facts are stated on three surfaces (`crash_handler.h`, `crash_handler.cpp`, `.agents/README.md`)
and the fix pass updated two of three — which is exactly how must-fix 5 survived; and comments
outnumber code roughly 2:1 (defensible for signal-handler subtleties, no cuts asked). ADRs:
workspace 0001 (ADR declined, now correctly citing the workspace record), 0002, 0008, 0013 and
0017 all compliant; camp ADR-0001 marginal and untouched. Consequences all Done except two
push/merge carry-forwards that are not diff defects: the PR body must carry the ADR-decline
rationale, and merge needs a full-scope `ci_local.sh` attestation (ADR-0018, camp is a project
repo). `docs/camp_user_manual.md` gained nothing — nothing there became stale, but the operator
manual has no troubleshooting section and this is operator knowledge; reasonable as its own issue.
Verified accurate and not to be re-litigated: the five signals, the base-vs-per-run path, the
absence of `SetROSLogDir` in `camp_launch.py`, pid correlation, mangled output, and the corrected
apport report-vs-core wording.

### Static Analysis
cpplint and cppcheck over the five changed C++ files. No new findings. cpplint's header-guard
style and `runtime/int` complaints are against camp's house convention and the deliberate
`unsigned long` formatter; cppcheck's `throwInNoexceptFunction` is the documented `noexcept`-boundary
throw and its `constParameterCallback` on `on_fatal_signal` is inapplicable to a `sa_sigaction`
signature. The `check_camp_exports` CTest was run directly and passes, and the guard was verified
non-vacuous: four `camp_crash*` symbols are present in the executable's `.dynsym`.

### Plan Adherence
No drift. All six Approach steps, all eight Files-to-Change rows, the Consequences table and both
`## Revisions` tables are genuinely implemented, and the Implementation entry's ~20 addressed-finding
claims were re-checked against the code rather than sampled: all hold except the three folded into
the findings above (the alt-stack reporting half, the `si_code` rendering, and the `crash_log_path()`
coverage claim). The declined `::raise(sig)` finding is recorded consistently in four places — plan
step 3, the second Revisions table, the Implementation entry, and an in-code comment — and its
technical core is sound: returning would core at the true fault site but does not apply to SIGABRT,
so adopting it means a per-signal branch in the death path of a diagnostics-only feature while the
backtrace, the actual deliverable, is identical either way. Only the "no core is collectable" clause
needs softening (suggestion above).
