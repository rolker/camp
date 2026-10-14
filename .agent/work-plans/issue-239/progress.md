---
issue: 239
---

# Issue #239 — Live coverage: depth colormap with uncertainty-driven opacity, so mapping uncovers the bathymetry

## Issue Review
**Status**: complete
**When**: 2026-10-14 10:01 -04:00
**By**: Claude Code Agent (Claude Sonnet 5.5)

**Issue**: #239
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Actions
- [ ] Renderer gap: RasterFieldItem carries ONE R32F texture and the unified shader samples one value; the two-band mode needs a second texture (or paired texture) per item plus new shader uniforms. This is a change to the shared ADR-0007 render contract (also used by GggsTileLayer and RasterLayer), so it must be additive and leave the Scalar path byte-identical for existing users.
- [ ] ADR-0008 consequence: TransferParams must stay identity and alpha must not be baked into the LUT. The opacity ramp therefore has to be computed in the shader from the uncertainty texel (u_opaque/u_transparent uniforms), not through bake_lut's alpha-ramp. Record this as an ADR (new, or amendment note on ADR-0008/0010) in the same PR.
- [ ] Overview pyramid (ADR-0010 D3): foldChild() already folds every band independently with a MEAN over finite samples, so both bands exist at every level. But the folded uncertainty is a mean of child uncertainties, not a propagated one, and a parent cell can have finite depth with no finite uncertainty counterpart. Decide and document the missing-uncertainty policy (suggest: treat as fully transparent) and the fidelity statement for overviews.
- [ ] Cells where depth is NoData must stay discarded regardless of uncertainty; cells where uncertainty is NoData need an explicit rule (issue is silent).
- [ ] Premultiplied output: the shader emits premultiplied-opaque alpha 1 today; the partial-alpha path must premultiply and the blend state in RasterGlRenderer must be checked.
- [ ] Persistence: add render mode and [u_opaque, u_transparent] under settingsKey() (layer_id); unknown/absent keys must restore today's behaviour. Add to the .agents/README.md verified-parameter table if applicable and docs/camp_user_manual.md.
- [ ] Default [u_opaque, u_transparent]: issue says "sensible"; needs a concrete rule (e.g. derived from the configured uncertainty range / auto-range of the uncertainty band, or IHO order thresholds). Open question for owner.
- [ ] Interaction with camp#145 (quality-ramp auto-default for the uncertainty band) and the existing single-band range override (camp#142 dialog): the colour range applies to the depth band in the new mode; the dialog needs a second control for the opacity range.
- [ ] Design note in docs/ citing Colin Ware (uncertainty visualisation; value-by-alpha) - verify citations against sources before writing; do not cite from memory.
- [ ] Out of scope: GggsTileLayer two-band mode (the issue is the live layer); note as follow-up.
- [ ] Tests: shader-side alpha mapping cannot run headless unless the mapping is extracted into a pure function; plan a CPU reference function (shared by a unit test and a documented GLSL mirror) plus a persistence round-trip in test_range_persist style.

## Plan Authored
**Status**: complete
**When**: 2026-10-14 10:03 -04:00
**By**: Claude Code Agent (Claude Sonnet 5.5)

**Plan**: `.agent/work-plans/issue-239/plan.md` at `68f1e23`
**Branch**: feature/issue-239 at `68f1e23`
**Phases**: single

### Open questions
- [ ] Default opacity ramp: auto-range snapshot, survey standard, or app-wide default
- [ ] Missing-uncertainty policy (cell transparent, tile-without-band opaque)
- [ ] Basemap to blend against for the demo
- [ ] Fixed band name "uncertainty" vs user-pickable
- [ ] Opacity range on the colorbar legend in this PR?
- [ ] Which Ware works to anchor on
- [ ] GGGS layer follow-up issue?

## Plan Review
**Status**: complete
**When**: 2026-10-14 10:03 -04:00
**By**: Claude Code Agent (Claude Sonnet 5.5) (in-context — author self-review)

**Plan**: `.agent/work-plans/issue-239/plan.md` at `a24ea26`
**PR**: PR-less
Independence uncertain: this review ran in the same context that authored the plan (no fresh dispatch), so findings are self-review; an independent re-review is advisable.
**Verdict**: approve-with-suggestions (after must-fix items folded in at `a24ea26`)

### Findings
- [x] (must-fix) Default ramp as an auto-range snapshot makes the lowest uncertainty present at enable time fully opaque and defeats firming-up; use a fixed/owner-chosen default, never data-derived — folded into Approach 4f
- [x] (must-fix) Unspecified when colour band equals opacity band; define disabled menu entry and restore-with-mode-off — folded into Approach 4g
- [x] (must-fix) Plan said 8 texture.reset() sites; actual is 6 (lines 301, 303, 694, 1055, 1236, 1263); also opacity-band change must dirty overview entries — folded into Approach 4c
- [ ] (suggestion) Make the "alpha mode off is byte-identical" renderer test compare against a golden captured BEFORE the renderer change, not against the new code's own nullptr path — plan.md Approach 8
- [ ] (suggestion) The modal opacity dialog blocks the UI; consider modeless/live for demo use (ADR-0009 already accepts modal)
- [ ] (suggestion) Size is large (~15 files); renderer commit should be independently reviewable, consider splitting docs/dialog into a second PR if review drags
- [ ] (suggestion) Rename shader uniforms u_u_opaque/u_u_transparent to u_alpha_lo/u_alpha_hi

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-10-14 10:45 -04:00
**By**: Claude Code Agent (Claude Sonnet 5.5)
**Verdict**: changes-requested

**Branch**: feature/issue-239 at `73459ce`
**Mode**: pre-push
**Depth**: Standard (reason: ~1500 changed lines, shared GL renderer, new ADR)
**Must-fix**: 4 | **Suggestions**: 5
**Round**: 1 | **Ship**: continue — the must-fixes are mechanical but touch correctness of the displayed quantity
Specialists: static analysis skipped (pre-commit hooks passed on every commit); governance + plan drift by the lead (plan sync'd, no drift beyond recorded deviations); Claude Adversarial Lens A + Lens B (two fresh subagents); Cross-Model: codex ran, gemini FAILED (agy print timeout 420s, partial output discarded).

### Findings
- [x] (must-fix) Standard ramp reads the COLOUR band's value as depth; with a non-depth colour band the S-44 limit is computed from the wrong quantity (codex; lens A+B minor) — raster_gl_renderer.cpp:129, sonar_live_cache_layer.cpp opacityActive
- [x] (must-fix) Shader rejects NaN but not +/-inf, unlike opacity_ramp.h (codex must-fix, lens A) — raster_gl_renderer.cpp:121
- [x] (must-fix) Settings restore: toFloat() without ok flag and Manual accepted without both bounds present, so corrupt/missing values restore [0,0] and hide every cell (codex must-fix, lens B) — sonar_live_cache_layer.cpp:2159
- [x] (must-fix) Mixed tiles (some lack the uncertainty band) draw those tiles opaque with no warning, since the status only warns when NO tile has the band (codex must-fix, lens A) — sonar_live_cache_layer.cpp:1554
- [x] (suggestion) Turning the mode off (or making the bands collide) never frees alpha textures; they stay counted in the ADR-0010 budget (lens A+B) — sonar_live_cache_layer.cpp setOpacityMode
- [x] (suggestion) A runtime-valid mode=on with colour==opacity band is lost on restart (restore forces off, next write persists false) (lens A) — sonar_live_cache_layer.cpp readSettings (deferred: the restore-off behaviour was an explicit review-plan decision (plan 4g); the preference loss needs a separate requested-vs-effective flag, low value for this slice)
- [x] (suggestion) Test-only mutators are public members of the production class (lens B) — sonar_live_cache_layer.h (deferred: matches the file's existing convention (setResidentBudgetForTest and the other *ForTest seams are already public))
- [x] (suggestion) Modal dialog has no parent and on_change captures the layer this; layer removal during the modal would dangle (lens B) — opacity_range_dialog.cpp (deferred: identical pattern to the existing colormap range dialog (nullptr parent); layers cannot be removed while a modal dialog is open in the current UI)
- [x] (suggestion) Documentation of the policy for tiles without the band: consistent with the owner's decision, but tile-level fallback is inconsistent between tiles (lens A) — see the mixed-tiles must-fix (deferred: the tile-level fallback is the owner's decision; the inconsistency is now surfaced by the mixed-tile status line)

## Implementation
**Status**: complete
**When**: 2026-10-14 10:50 -04:00
**By**: Claude Code Agent (Claude Sonnet 5.5)

**Branch**: feature/issue-239 at `b4908de`
**Addressed**: Local Review (Pre-Push) round 1 at b5eca40
**Commits**: ae26128, 8269d9b, c67b6da

### Actions
- [x] Standard ramp read the colour band as depth — applied only with the depth colour band, manual ramp unaffected — `sonar_live_cache_layer.h opacityActive`
- [x] Shader rejected only NaN — now +/-inf uncertainty or depth is transparent, with a parity test — `raster_gl_renderer.cpp`
- [x] Settings restore accepted garbled / incomplete ramps — whole-ramp fallback to the shipped standard, tested — `sonar_live_cache_layer.cpp readSettings`
- [x] Mixed tiles drew opaque with no warning — status counts tiles lacking the band, tested — `sonar_live_cache_layer.cpp updateDisplay`
- [x] Alpha textures stayed resident when the mode was off — released and uncounted, tested with GL — `sonar_live_cache_layer.cpp releaseAlphaTextures`
- [x] mode=on with colour==opacity band lost on restart (deferred: restore-off was a review-plan decision; needs a requested-vs-effective flag)
- [x] Test-only mutators public (deferred: matches existing *ForTest convention in this class)
- [x] Modal dialog unparented (deferred: same pattern as the existing colormap range dialog)
- [x] Inconsistent tile-level fallback policy (deferred: owner decision; now surfaced by the status line)

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-10-14 11:01 -04:00
**By**: Claude Code Agent (Claude Sonnet 5.5)
**Verdict**: changes-requested

**Branch**: feature/issue-239 at `7b93bc2`
**Mode**: pre-push
**Depth**: Standard (reason: shared GL renderer + new ADR; re-review of round-1 fixes)
**Must-fix**: 3 | **Suggestions**: 3
**Round**: 2 | **Ship**: recommended — remaining items are mechanical and were fixed in the same sitting; the host asked for exactly one re-review, so the fixes in 2c2ebf7 are not re-reviewed again
Specialists: Claude Adversarial Lens A + Lens B (fresh subagents); Cross-Model: codex ran, gemini FAILED again (agy 420 s print timeout). Lens A and Lens B found no must-fix; codex found three.

### Findings
- [x] (must-fix) Alpha textures upload at render time and no arrival-driven eviction sees them; footprint can sit over the ADR-0010 budget while the stream is idle (codex; lens B suggestion) — sonar_live_cache_layer.cpp renderImage
- [x] (must-fix) Overview-blind status: opacityAvailable() and the mixed-tile count scanned fine tiles only (codex) — sonar_live_cache_layer.cpp opacityAvailable, updateDisplay
- [x] (must-fix) The dialog applied Manual [0,0] (blank layer) and persisted bounds that readSettings rejects, so a restart silently reverted to Standard (codex; lens A suggestion) — opacity_range_dialog.cpp, sonar_live_cache_layer.cpp setOpacityRamp
- [x] (suggestion) updateDisplay() scans all tiles on every tile message while the mode is active, O(n^2) over a burst (lens A+B) — sonar_live_cache_layer.cpp updateDisplay (deferred: opt-in mode; per-tile cost is one string-keyed map lookup, thousands of tiles is millions of cheap operations; revisit with a cached count if a profile shows it)
- [x] (suggestion) Misleading status before the first band is known (lens A) — sonar_live_cache_layer.cpp updateDisplay
- [x] (suggestion) A colour-band switch re-uploads unchanged opacity textures (lens B) — sonar_live_cache_layer.cpp setBandName
