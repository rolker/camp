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
