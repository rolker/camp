// Copyright 2026 Roland Arsenault
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CAMP_MAP__RASTER__SHORELINE_ANCHOR_H_
#define CAMP_MAP__RASTER__SHORELINE_ANCHOR_H_

#include <optional>

#include <QObject>
#include <QString>

namespace camp
{
namespace raster
{

/// The single, source-agnostic seam every shoreline-anchor source writes into
/// (camp#181, camp ADR-0015). A tiny **ROS-free** `QObject`: it holds one
/// `std::optional<double>` per source, a selected mode, and emits `changed()`
/// on the **union** of two conditions: any real **mode** change, and any value
/// change that moves the resolved `(value, activeSource)` pair. A mode change
/// emits even when nothing resolves — selecting an unavailable source is a state
/// the operator must SEE reported, which is why it is not optimised away (see
/// `setMode()`, and `ShorelineAnchorTest.UnresolvableModeChangeStillReportsItself`
/// which pins it). Being ROS-free is load-bearing — it is
/// what keeps `libcamp_map`'s ADR-0002 boundary one-directional even though a
/// later source (Phase B's `map_tide` tracker) lives across the ROS boundary and
/// Phase C's chart-datum provider lives in the `camp` app: both **push** a value
/// in from outside; the layers never pull. It also keeps the headless GL tests
/// hermetic, since a test can set a value directly with no boat and no grids.
///
/// **Thread affinity — GUI thread only.** Every source pushes *in* from
/// somewhere else, so this must be said plainly: the holder is not thread-safe
/// and all of `setChartDatum()` / `setPlatformTide()` / `setManual()` /
/// `setMode()` / `applyManualSelection()` must be called on the thread that owns
/// the object (the GUI thread — the layers read `value()` from `renderImage()`
/// there). The setters read-modify-write and then emit, so a direct call from a
/// worker thread races the render read while *appearing* to work: `changed()`
/// would still be delivered queued to the layer's slot, hiding the torn read.
/// A source living on another thread (Phase B's `map_tide` tracker, Phase C's
/// datum provider) must therefore marshal — `QMetaObject::invokeMethod(anchor,
/// ..., Qt::QueuedConnection)`, or a queued signal/slot connection into a
/// wrapper — never call a setter directly.
///
/// **Why a holder and not a plain optional on the layer.** The manual anchor
/// (Phase A / PR1) is deliberately not a dialog special case: it writes into the
/// same holder that the chart-datum (Phase C) and platform-tide (Phase B) sources
/// will. That is what makes the design source-agnostic rather than
/// retrofitted-around-manual — adding a source later is `set<Source>()`, with no
/// change to the resolution logic or the layers.
class ShorelineAnchor : public QObject
{
  Q_OBJECT

public:
  /// The anchor sources, **in D3 fallback precedence order** (camp ADR-0015 D3).
  /// Declaration order IS the precedence: when the active mode yields nothing,
  /// resolution walks *downward* from the mode to the first source that has a
  /// value. `None` renders unanchored and never carries a value.
  ///
  /// PR1 ships this whole ordering, but only `Manual` and `None` are
  /// runtime-satisfiable: no code pushes a `ChartDatum` value until Phase C (PR2)
  /// and no code pushes a `PlatformTide` value until Phase B (PR3). They are
  /// present and correctly ordered so the seam and the ADR are honest about the
  /// eventual precedence — not so PR1 can pretend to resolve them. The *policy*
  /// default is chart datum; the *runtime* default in PR1 is `None` (unanchored,
  /// byte-identical to pre-camp#181), because chart datum has no source to resolve
  /// against yet. See ADR-0015's policy-vs-runtime-default separation.
  enum class Source
  {
    ChartDatum,    ///< spatial datum surface, static in time (Phase C / PR2)
    PlatformTide,  ///< a platform's `map_tide`, one measured scalar (Phase B / PR3)
    Manual,        ///< operator-typed value, per region (Phase A / PR1)
    None,          ///< render unanchored and say so (D5)
  };

  explicit ShorelineAnchor(QObject * parent = nullptr);

  /// The selected mode. Resolution starts here and falls through the D3 order
  /// beneath it (a `ChartDatum` mode with no datum value falls to platform tide,
  /// then manual, then none). Runtime default is `None` in PR1 — see `Source`.
  Source mode() const { return mode_; }
  /// Emits `changed()` on any real mode change, even one that leaves the resolved
  /// `(value, activeSource)` pair untouched: the layer status names `mode()` when
  /// nothing resolves, so switching between two unresolvable sources still changes
  /// what the operator is shown.
  void setMode(Source mode);

  /// Push a value from each source. `std::nullopt` clears that source (e.g. the
  /// datum provider reporting no coverage, or the tide tracker with no platform
  /// selected). Setters emit `changed()` only when the *resolved* anchor moves,
  /// so a source refreshing an identical value costs no repaint.
  ///
  /// A **non-finite** value clears the source, exactly as `std::nullopt` does. A
  /// NaN would otherwise be corrosive rather than merely wrong: NaN != NaN, so it
  /// defeats the "did the resolved anchor move?" equality (every identical
  /// re-push would emit `changed()`) and the renderer's LUT cache key (a re-bake
  /// plus a texture upload every frame). It is not an anchor either — the bake
  /// falls back to the unanchored ramp — so reporting it as an absence is the
  /// honest outcome (D5/D6).
  void setChartDatum(std::optional<double> value);
  void setPlatformTide(std::optional<double> value);
  void setManual(std::optional<double> value);

  /// Apply a manual value AND a mode as one change, emitting `changed()` at most
  /// once. Calling `setManual()` then `setMode()` can emit against the state
  /// half-applied — a `changed()` carrying the new value under the mode the
  /// operator just left, composing a status that names a mode already abandoned
  /// before the second emission corrects it. Both callers that change the pair
  /// together (the range dialog's apply, and `readSettings()`'s restore) go
  /// through here, so the "single changed() emission" their comments claim is
  /// true rather than aspirational. Emission follows the same rules as the
  /// separate setters: a real mode change always emits; a value-only change emits
  /// only when the resolved `(value, activeSource)` pair moves.
  void applyManualSelection(std::optional<double> manual, Source mode);

  /// The manual value as stored, independent of the active mode — the value the
  /// range dialog edits and the layer persists.
  std::optional<double> manualValue() const { return manual_; }

  /// The resolved anchor: from `mode()`, the first source at or below it with a
  /// value. `std::nullopt` when nothing resolves — which is D5's contract that an
  /// **absent anchor never means 0.0** (a 0.0 in the ellipsoidal frame would put
  /// the shoreline break ~28 m into deep water). Callers must treat `nullopt` as
  /// "render unanchored", never substitute 0.
  std::optional<double> value() const;

  /// Which source actually provided `value()` — `None` when nothing resolved.
  /// This is what the layer status names, so a fall-through is reported honestly
  /// (D5 / S-98's permanent indication) rather than silently applied.
  Source activeSource() const;

  /// The stable token a source persists as in QSettings — deliberately NOT
  /// `sourceLabel()`, whose words are operator-facing and may be reworded, and
  /// NOT the enum's integer, which a reordering (the enum's order IS the D3
  /// precedence) would silently repoint at a different source. `sourceFromKey()`
  /// returns `std::nullopt` for an unrecognized or absent token, and callers must
  /// treat that as `None` (unanchored): an anchor nobody can confirm was chosen
  /// must never be inferred back into existence (D6).
  static QString sourceKey(Source source);
  static std::optional<Source> sourceFromKey(const QString & key);

  /// A short, operator-facing label for a source ("chart datum", "platform tide",
  /// "manual", "none"). Shared by the layer status and the range dialog readout so
  /// the two cannot drift.
  static QString sourceLabel(Source source);

signals:
  /// Emitted on a real mode change, OR when the resolved `(value, activeSource)`
  /// pair changes. NOT only the latter: narrowing this to a strict resolved-pair
  /// move would silently reinstate the stale-indication bug where selecting an
  /// unavailable source left the previous source's label on screen. Sources added
  /// later (chart datum, platform tide) must not assume `changed()` implies the
  /// value moved. The layer's
  /// slot drops its cached image, re-feeds the renderer, recomposes its status, and
  /// requests a repaint. This slot is not a paint-time path — but note that
  /// `updateStatus()` itself is NOT reached exclusively from outside `paint()`:
  /// `GggsTileLayer::paint()` calls it via `scheduleEvictionIfNeeded()` when the
  /// over-budget flag flips. That path predates this work (camp#195) and is not
  /// changed here.
  void changed();

private:
  /// The stored value for one source (`None` never has one).
  std::optional<double> valueFor(Source source) const;

  Source mode_ = Source::None;
  std::optional<double> chart_datum_;
  std::optional<double> platform_tide_;
  std::optional<double> manual_;
};

}  // namespace raster
}  // namespace camp

#endif  // CAMP_MAP__RASTER__SHORELINE_ANCHOR_H_
