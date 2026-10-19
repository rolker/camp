---
issue: 242
---

# Issue #242 — Map centre shifts when the window is resized after startup (resize anchor)

## Implementation
**Status**: complete
**When**: 2026-10-19 10:02 -04:00
**By**: Claude Code Agent (Claude Sonnet 5.5)

**Branch**: feature/issue-242

### Actions
- [x] ProjectView constructor calls camp::keepCenterOnResize(this), which sets QGraphicsView::AnchorViewCenter (src/camp/view_resize_anchor.h). The anchor alone fixes the startup case: the restore's centerOn() runs at the pre-maximize size and the later window-manager maximize arrives as a resize, which the anchor now handles. No second re-apply after the first resize was needed.
- [x] Checked interference: updateBackground()/centerMap() use centerOn()/fitInView() (explicit, unaffected by the resize anchor); the constructor's scale(1,-1) and the wheel zoom use scale(), governed by transformationAnchor, which is left unchanged (test SetsOnlyTheResizeAnchor pins this).
- [x] Tests: test_view_resize_anchor (centre and scale survive a grow to 3774x2091 and a shrink; control shows default NoAnchor moves the centre by more than 100 pixels; transformationAnchor untouched). ProjectView itself cannot be constructed in a gtest (roslink and manager dependencies), so the test drives a plain QGraphicsView through the shared helper and check_project_view_resize_anchor (cmake source check) guards the constructor call. Source check confirmed red with the constructor call removed.
- [x] Docs: .agents/README.md pitfall (view restore + resize anchor), docs/camp_user_manual.md note.
- [x] Test counts: baseline 454 tests, 0 failures, 1 skipped; after 459, 0 failures, 1 skipped.

### Not done
- [ ] Not run against a real window manager maximize (no GUI allowed here); the live rehearsal should confirm.
