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
