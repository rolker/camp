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
