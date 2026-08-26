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
