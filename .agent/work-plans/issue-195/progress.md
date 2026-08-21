---
issue: 195
---

# Issue #195 — GggsTileLayer: no viewport-scoped retention or residency budget — panning accumulates tiles without bound (camp#153 precedent)

## Plan Authored
**Status**: complete
**When**: 2026-08-21 15:02 -04:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-195/plan.md` at `b70311f`
**Branch**: feature/issue-195 at `b70311f`
**Phases**: single (one PR, ~5 atomic commits)

### Design decisions taken (host constraints evaluated, not inherited)
- **Fold-to-parent rejected.** GGGS tiles are file-backed; `resetPixels()` +
  `releaseGL()` is already a complete release and the existing demand-driven
  loader re-reads on pan-back, so the reload path is free. `SonarLiveCacheLayer`
  folds only because its data arrives once over ROS. A CAMP-side pyramid would
  also manufacture a derived product `uma-ADR-0010` D9 deliberately withholds
  from `chart`/`reference`. Cost — an evicted area blanks rather than degrades —
  is bought back by never evicting the coarsest available level.
- **Count budget**, adaptive cap `clamp(3 × protected_count, 32, 64)` (MapTiles /
  camp#98 form), which makes "budget ≥ current-frame set" true by construction.
  Fixed 960×960 tiles cost ~7.03 MiB each (CPU buffer retained past GPU upload
  per camp#180), so 64 ≈ 450 MiB.
- **`views().first()` designed around, not inherited.** `paint()` already
  receives the painting viewport widget; `deriveViewportClip()` gains an optional
  `widget` param and recovers the painting view, falling back to `views().first()`.
  Protection accumulates per `paint()`; eviction is deferred + debounced so one
  pass sees the union of all views (camp#150-safe).
- **Structural protection** via a new `TileResidency` splice-partition type
  (two `std::list`s, O(1) per frame/touch) whose eviction API cannot reach the
  protected partition — D4's sentinel-node guarantee without an intrusive list.
- **Hysteresis**: evict to `0.75 × cap`, plus a per-kick loader volume cap.
- **Degrade lever** included: `pressure_bias_` steps the LOD selection coarser
  when the protected set alone exceeds the cap, with a cool-down and a visible
  status; a no-op with a visible status where no coarser level exists.
- **camp#194 hole-coverage rule preserved**: finer tiles covering a
  `loadFailed()` tile are protected from budget eviction too.

### Open questions
- [ ] Confirm cap defaults (`kMultiplier=3`, `kMinCap=32`, `kMaxCap=64` ≈ 450 MiB) and the `GggsTileLayers/max_resident_tiles` settings key.
- [ ] Confirm the degrade lever belongs in this PR rather than a follow-up — it is the part most likely to oscillate in the field.
- [ ] Confirm the `deriveViewportClip()` `widget` parameter (touching `RasterLayer` and `SonarLiveCacheLayer` call sites) is acceptable scope here versus a separate camp#150 PR.

## Plan Review
**Status**: complete
**When**: 2026-08-21 15:04 -04:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-195/plan.md` at `b70311f`
**PR**: PR-less (`--issue` mode; branch `feature/issue-195`)
**Verdict**: changes-requested

Design direction is sound and well-grounded in the code — the D4 mapping,
the `TileResidency` splice-partition, hysteresis, and the fold-to-parent
rejection all hold up. Seven must-fix items are implementation traps or D4
gaps that are cheap now and silent later; the rest are scoping/robustness
suggestions. Answers to the plan's three Open Questions are in findings
6 (degrade lever — defer), 8 (`deriveViewportClip` — drop from this PR),
and 10 (cap defaults — re-derive from a byte target).

### Findings
- [ ] (must-fix) Protection must be driven by the loader predicate (tiles ∩ `clip.scene` at level ≤ selection), NOT the `itemsIntersecting()` draw list — `cached_image_` short-circuits the collect on static frames, so a draw-list-sourced `protect()` would protect nothing and the live visible set becomes evictable — `plan.md:78`
- [ ] (must-fix) `beginFrame()`/`protect()` must sit immediately after `deriveViewportClip()` and before `paint()`'s early returns (`tiles_.empty()`, `data_min_ > data_max_`, `gggs_tile_layer.cpp:942,1002`); an early return between them leaves the whole set spliced into the evictable partition — `plan.md:78`
- [ ] (must-fix) Eviction gated on `!future_watcher_.isRunning()` will almost never fire during a continuous pan (every pan step re-kicks the loader) — the exact scenario the budget exists for. The debounce must re-arm when it finds the worker busy (or abort+join like `loadTiles()` does) — `plan.md:106`
- [ ] (must-fix) camp#194 interaction: an evicted hole-covering finer tile can NEVER reload — `loadTilesWorker()` skips `tile->level() > level` (`gggs_tile_layer.cpp:527`), so the "reload path is free" premise fails for exactly that class. Either protect hole-coverers regardless of viewport, or extend the loader ceiling to admit finer tiles over a failed footprint; state which. Factor the hole predicate into one helper shared by `tilesReady()` and the budget pass so the two rules cannot drift — `plan.md:98`
- [ ] (must-fix) `tilesReady()` unconditionally rewrites `setStatus()` (`:732-739`, clears to `""` when no failures), so any degrade / over-budget status set from `paint()` is silently wiped. Unify all status writes into one composer — `plan.md:94`
- [ ] (must-fix) `clamp(3 × protected, 32, 64)` does not make "budget ≥ current-frame working set" true by construction: `kMaxCap` is a hard ceiling, so `protected > 64` yields budget < working set — D4's "thrashes by construction". Either floor the cap at `protected` (+ headroom), or make the τ lever mandatory; the plan must say which — `plan.md:61-63`
- [ ] (must-fix) Per-kick loader cap creates a settled-but-incomplete picture: the re-kick guard needs `selected_level_`/`load_viewport_` to change (`:996-999`), so a truncated kick with the operator stationary never completes and `tilesReady()` clears the status to `""`. Drop the per-kick cap (the kick set is already viewport-bounded = the protected set) or force a re-kick plus a visible status on truncation — `plan.md:85-87`
- [ ] (suggestion) Open Question 3 — drop the `deriveViewportClip()` `widget` parameter from this PR. The eviction metric needs a view centre, and `clip.scene.center()` is already computed per `paint()`; with CAMP's single MapView it is exactly the painting view's centre. Leave the painting-view recovery to camp#150 — `plan.md:71-77,126-127`
- [ ] (suggestion) Open Question 2 — defer the `pressure_bias_` lever to a follow-up issue. As specified it oscillates deterministically: trigger `protected > kMaxCap` → degrade → `protected' ≈ protected/4` → cool-down relaxes → re-trigger, period = cool-down. A stable relax needs the *predicted* protected count at the finer level (countable from `tiles_ ∩ clip`) under a hysteresis factor, not a frame cool-down. Deferring is safe because the unbounded-pan accumulation (the actual OOM) is fully fixed by the budget; the lever only addresses "one viewport is itself over budget". Pair the deferral with finding 6's cap floor plus a non-silent over-budget status so D4's "never fail silently" still holds in this PR — `plan.md:89-96`
- [ ] (suggestion) Open Question 1 — re-derive the cap from a byte target. `width_`/`height_` come from GDAL (`gggs_tile.cpp:70-71,96-97`); 960×960 is a store convention, not an invariant. Convert a byte budget (`GggsTileLayers/max_resident_bytes`) to a count using observed tile bytes — keeps D4's blessed count mechanics, removes the fixed-size assumption, and makes the number comparable to the co-resident `LiveTileCache/max_vram_bytes` (512 MiB default). 450 MiB on top of that, on the station camp#153 already OOM'd, needs an explicit headroom argument — `plan.md:61-66,178-179`
- [ ] (suggestion) Bound the coarsest-level exemption (its resident set grows with area panned) and note `available_levels_` is dynamic across `rescan()`. Also state and test the real degrade-not-blank mechanism: because every level ≤ selection loads, a panned-away area retains coarse coverage — assert a non-empty render over an evicted area, not merely "coarsest level survives" — `plan.md:66-67,113`
- [ ] (suggestion) The A→B→A ping-pong assertion is unachievable or vacuous as written — with cap < |A ∪ B| and distance-primary ordering, ping-pong is guaranteed. Specify the cap relative to |A ∪ B| and assert a bounded reload count. Relatedly, distance-primary ordering is what D4 criticises ("distance-only discards history along a path being traversed"); add a staleness term or record why not — `plan.md:78-80,113`
- [ ] (suggestion) The `camp-ADR-0013` amendment is broader than one sentence: the Residency bullet (`:81-84`), the Release bullet (`:132-138`), the Residency-bound/camp#195 paragraph (`:147-165`), and the camp#195 pointers at `:97` and `:131` all go stale. The overdraw mitigation the ADR routes to camp#195 must be re-routed to the new follow-up issue, or the ADR will point at a closed issue — `plan.md:132`
- [ ] (suggestion) `camp-ADR-0014` is warranted (not overkill): a new residency policy with deliberate divergences from `camp-ADR-0010` (drop vs fold, count vs bytes) plus the repo-local D4 compliance record. Record there that τ-raising weakens `camp-ADR-0013`'s "`getElevation()` samples finest-covering-tile-first" mitigation (`:124-126`) — `plan.md:131`
- [ ] (suggestion) File targeting: add `test/test_gggs_elevation.cpp` (readout over an evicted area) — the consequences table names `getElevation()` but no test covers it. Cite the in-repo `test/test_map_tiles_eviction.cpp` as the count-budget precedent alongside cube's `test_tile_eviction_rss.cpp`. Note that headless tests exercise only the CPU half of `releaseTile()` — the GL texture free (the larger half of the 7.03 MiB) stays uncovered — `plan.md:113,119-131`

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-08-21 16:00 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested (all findings addressed in `21751c2`)

**Branch**: feature/issue-195 at `6b05355` (reviewed) → `21751c2` (fixed)
**Mode**: pre-push
**Depth**: Deep (reason: new ADR + ~2000-line diff across a render hot path)
**Specialists**: static analysis (cppcheck); Claude Adversarial Lens A (logic) + Lens B (systemic/safety); governance + plan drift. Copilot off (default). Local model off (`--no-local`).
**Must-fix**: 5 | **Suggestions**: 12
**Round**: 1 | **Ship**: recommended — every must-fix is fixed and re-tested; 263 tests green

### Findings
- [x] (must-fix) Zoom-out backdrop drawn but not protected: in-view tiles finer than the selection are the whole visible picture mid-transition and cannot reload, so the budget could undo camp#103/#194's no-blank-frame guarantee — `gggs_tile_layer.cpp:refreshProtection`
- [x] (must-fix) Coarsest exemption (flat 64) can exceed the eviction target (~57 at the 512 MiB/960x960 default), so the victim loop can never reach it and sheds every ordinary candidate every pass — `gggs_tile_layer.cpp:evictIfOverBudget`
- [x] (must-fix) Timer re-arm is a blind resample: the loader-idle window during a sustained pan is ~one event-loop turn, so the budget can be starved for seconds. `tilesReady()` now drains a pending pass — `gggs_tile_layer.cpp:tilesReady`
- [x] (must-fix) A latched GL `makeCurrent()` failure permanently disables eviction with no report — now surfaced as "tile budget NOT enforced" — `gggs_tile_layer.cpp:evictIfOverBudget`
- [x] (must-fix) `hole_coverage_released_` was a one-way latch surviving the recovery it advertised, and named Rescan alone (which does not reload an evicted finer tile) — `gggs_tile_layer.cpp:updateStatus`, `rescan`
- [x] (suggestion) Malformed `GggsTileLayers/max_resident_bytes` parses to 0 = the disable sentinel, silently restoring pre-#195 behaviour — warn + fall back — `gggs_tile_layer.cpp` ctor
- [x] (suggestion) Generation 0 (never seen) classified "recent" for the first 120 paints, inverting the staleness key — `gggs_tile_layer.cpp:evictIfOverBudget`
- [x] (suggestion) `budgetTiles() == 0` early return left `over_budget_` stale — `gggs_tile_layer.cpp:evictIfOverBudget`
- [x] (suggestion) String-based `QMetaObject::invokeMethod` — a rename would silently stop the budget; use the pointer-to-member overload — `gggs_tile_layer.cpp:scheduleEvictionIfNeeded`
- [x] (suggestion) `perTileResidentBytes()` recomputed O(n) every frame — memoized against `tiles_.size()` — `gggs_tile_layer.cpp`
- [x] (suggestion) `updateStatus()`'s `loading_` early return hid the residency reports for the whole duration of a pan — "loading..." is now a part — `gggs_tile_layer.cpp:updateStatus`
- [x] (suggestion) Quotation "never fail silently" misattributed to `uma-ADR-0013` D4; it is camp#195's own ask (D4 says "degrades visibly and predictably rather than churning") — `gggs_tile_layer.cpp`, `0014-*.md`, `plan.md`
- [x] (suggestion) ADR-0014's null-viewport "pure LRU" claim, its stated per-frame cost, and its `uma-ADR-0010` D9 store-class claim were imprecise — corrected against the code and D9's text
- [x] (suggestion) `camp-ADR-0013`'s `getElevation()` "finest-covering-tile-first" fidelity mitigation is now residency-conditioned and was left unqualified — amended
- [x] (suggestion) `camp-ADR-0010` had no back-reference to the sibling ADR-0014 — added
- [x] (suggestion) `gggs_tile.h:resetPixels()` invariant still claimed `data_` is freed after upload; camp#180 retains it, and that retention is the basis of ADR-0014's 2x byte model — corrected
- [x] (suggestion) Test gaps: the cap-relative exemption bound, the zoom-out backdrop, the rescan-clears-report path, and the entire deferred schedule→`processEvents` path were uncovered; `writeStrip()`'s in-helper fatal assertion could let an upper-bound assertion pass spuriously — four tests added, helper returns bool

### Not accepted
- Freeing the CPU half of a tile when GL release is refused (Lens B): correct in principle once the renderer's GL-failed flag is latched, but it depends on a renderer internal the layer cannot observe. Reported instead; exposing the flag is a follow-up, recorded in ADR-0014.
- Skipping the per-frame protection sweep on unmoved frames: the sweep is what keeps the LRU generations honest. Cost is now stated accurately in ADR-0014 and named as the first thing to profile alongside camp#198.

### Clean
Static analysis (cppcheck) reported nothing on the changed lines. Both adversarial passes independently cleared the worker/GUI-thread ordering contract, the destruction-with-pending-invocation path, re-entrancy, the repaint loop, and `TileResidency` itself (epoch arithmetic, iterator stability, the append-only precondition). Plan drift: none — every planned file changed, nothing outside the plan, and all seven plan-review must-fixes verified present in code.
