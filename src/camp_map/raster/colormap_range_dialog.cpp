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

#include "colormap_range_dialog.h"

#include <optional>
#include <utility>

#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QRadioButton>
#include <QSignalBlocker>
#include <QVBoxLayout>

#include "marine_colormap/colormap.hpp"
#include "marine_colormap/palette.hpp"
#include "marine_colormap/transfer.hpp"
#include "marine_colormap_widgets/colormap_legend_widget.hpp"

namespace camp
{
namespace raster
{

namespace
{

/// [camp#181 / ADR-0015 D6] The anchor spin's minimum, shown as "not set" via
/// QDoubleSpinBox::setSpecialValueText(). It is the *absence* of an anchor made
/// representable in a widget whose natural default is 0.0 — the one value D6
/// forbids. Chosen far outside any plausible ellipsoidal height so no real anchor
/// can collide with it.
constexpr double kAnchorUnset = -1.0e6;

/// The spin's value as an anchor: `std::nullopt` while it sits at "not set".
std::optional<double> spin_value(const QDoubleSpinBox * spin)
{
  if (spin->value() <= kAnchorUnset) {
    return std::nullopt;
  }
  return spin->value();
}

}  // namespace

void showColormapRangeDialog(
  QWidget * parent, const QString & title, const ColormapRangeState & state,
  std::function<void(float, float)> on_range, std::function<void()> on_reset,
  std::function<void(ShorelineAnchor::Source, std::optional<double>)> on_anchor)
{
  QDialog dialog(parent);
  dialog.setWindowTitle(title);

  auto * layout = new QVBoxLayout(&dialog);
  layout->addWidget(new QLabel(
    "Drag the handles or edit the bounds to pin a manual range; "
    "reset to track the data automatically.", &dialog));

  auto * legend = new marine_colormap_widgets::ColormapLegendWidget(&dialog);
  legend->setPalette(state.palette_index);

  // The colorbar domain is the data extent the handles slide within. When no data
  // has been folded yet (data_min > data_max, the layers' crossed sentinel), fall
  // back to the resolved range; widen a degenerate point so the handles have room.
  float domain_min = state.data_min;
  float domain_max = state.data_max;
  if (domain_min > domain_max) {
    domain_min = state.lo;
    domain_max = state.hi;
  }
  // [camp#181 / ADR-0015] Remember that the widening happened. The map renders a
  // degenerate range UNANCHORED (RasterGlRenderer::ensureLut: a zero-width range
  // has no domain for the BreakpointMap to hinge on), but the widened colorbar has
  // a range and would happily bake an anchored ramp over it — the one remaining
  // state where the map paints and the legend describes something else. Narrower
  // than the deferred lo()/hi() divergence as a whole: the crossed-extent path
  // still needs camp#142's separation of the render range from the handle domain,
  // and the map paints nothing there to disagree with.
  bool domain_widened = false;
  if (domain_min >= domain_max) {
    domain_min -= 0.5f;
    domain_max += 0.5f;
    domain_widened = true;
  }
  legend->setDomain(domain_min, domain_max);
  layout->addWidget(legend);

  // Min/max spin boxes give precise entry alongside the drag (the numeric path
  // PR1 offered, now folded into the same dialog and kept in sync with the bar).
  auto * form = new QFormLayout();
  auto * min_spin = new QDoubleSpinBox(&dialog);
  auto * max_spin = new QDoubleSpinBox(&dialog);
  for (QDoubleSpinBox * spin : {min_spin, max_spin}) {
    spin->setRange(-1.0e9, 1.0e9);
    spin->setDecimals(6);
  }
  form->addRow("Minimum:", min_spin);
  form->addRow("Maximum:", max_spin);
  layout->addLayout(form);

  // Seed the widget to the layer's current state BEFORE wiring callbacks, so the
  // seeding emissions don't echo back to the layer or the spin boxes.
  if (state.mode == marine_colormap::RangeMode::Manual) {
    legend->setManual(state.lo, state.hi);
  } else {
    legend->updateAuto(domain_min, domain_max);
  }
  // Initialise the spin boxes from the resolved bar state so the two always agree.
  min_spin->setValue(legend->lo());
  max_spin->setValue(legend->hi());

  // The widget's rangeChanged fires for both a manual pin and a reset/auto fold;
  // mode() disambiguates which callback to run. The spin boxes mirror the bar.
  QObject::connect(
    legend, &marine_colormap_widgets::ColormapLegendWidget::rangeChanged,
    [legend, min_spin, max_spin, on_range, on_reset](float lo, float hi) {
      {
        const QSignalBlocker block_min(min_spin);
        const QSignalBlocker block_max(max_spin);
        min_spin->setValue(lo);
        max_spin->setValue(hi);
      }
      if (legend->mode() == marine_colormap::RangeMode::Manual) {
        if (on_range) {
          on_range(lo, hi);
        }
      } else if (on_reset) {
        on_reset();
      }
    });

  // Editing either spin box pins a manual range on the bar (which then echoes
  // back through rangeChanged to update the layer and re-sync the boxes).
  auto pin_from_spins = [legend, min_spin, max_spin]() {
    legend->setManual(
      static_cast<float>(min_spin->value()), static_cast<float>(max_spin->value()));
  };
  QObject::connect(min_spin, &QDoubleSpinBox::editingFinished, legend, pin_from_spins);
  QObject::connect(max_spin, &QDoubleSpinBox::editingFinished, legend, pin_from_spins);

  // ---- [camp#181 / ADR-0015] Shoreline anchor --------------------------------
  // Offered ONLY when the palette declares a shoreline (oleron / hypsometric).
  // Chart datum and Platform tide are listed in D3 order but disabled and labelled
  // unavailable — present and honestly reported, never faked or silently tried
  // (no source resolves them until PR2 / PR3). Manual + None are operator-driven.
  if (state.supports_anchor) {
    const marine_colormap::Palette * anchor_palette =
      marine_colormap::find_palette(state.palette_name);

    auto * anchor_box = new QGroupBox("Shoreline anchor", &dialog);
    auto * anchor_layout = new QVBoxLayout(anchor_box);
    anchor_layout->addWidget(new QLabel(
      "Pin the land/sea colour transition to a real water level.", anchor_box));

    auto * chart_radio = new QRadioButton(
      "Chart datum — automatic (available in a later update)", anchor_box);
    auto * tide_radio = new QRadioButton(
      "Platform tide — automatic (available in a later update)", anchor_box);
    chart_radio->setEnabled(false);   // no source resolves these yet (PR2 / PR3)
    tide_radio->setEnabled(false);

    auto * manual_radio = new QRadioButton("Manual:", anchor_box);
    auto * anchor_spin = new QDoubleSpinBox(anchor_box);
    // [camp#181 / ADR-0015 D6] The spin has a "not set" state that is STRUCTURALLY
    // distinct from 0.0, and it is what an unconfigured Manual anchor reads. A
    // QDoubleSpinBox's own default value is 0.0, and 0.0 is precisely the value D6
    // forbids: anchor values are ellipsoidal heights, so 0.0 puts the land/sea
    // break ~28 m into deep water at the Isles of Shoals — a plausible-looking
    // display that is wrong in the direction that matters when the question is
    // under-keel clearance among rocks. Making "unset" a separate state (rather
    // than warning about 0.0, which reads as sea level in the readout) is what
    // makes an accidental 0.0 anchor unreachable: 0.0 can only be applied by
    // typing it. Qt renders specialValueText at the minimum in place of the
    // number+suffix, and kAnchorUnset is far outside any plausible anchor.
    anchor_spin->setRange(kAnchorUnset, 1.0e6);
    anchor_spin->setDecimals(3);
    anchor_spin->setSuffix(" m");
    anchor_spin->setSpecialValueText("not set");
    auto * manual_row = new QHBoxLayout();
    manual_row->addWidget(manual_radio);
    manual_row->addWidget(anchor_spin);
    manual_row->addStretch();

    auto * none_radio = new QRadioButton("None (render unanchored)", anchor_box);
    auto * anchor_readout = new QLabel(anchor_box);

    anchor_layout->addWidget(chart_radio);
    anchor_layout->addWidget(tide_radio);
    anchor_layout->addLayout(manual_row);
    anchor_layout->addWidget(none_radio);
    anchor_layout->addWidget(anchor_readout);
    layout->addWidget(anchor_box);

    // Seed BEFORE wiring so the setChecked() calls don't echo into the layer.
    // The radio comes from the layer's STORED anchor_mode, never inferred from
    // whether a manual value happens to exist: opening a dialog must not mutate
    // the thing it is inspecting. Inferring the mode would open a ChartDatum layer
    // on Manual (or on None) and, together with the unconditional fire below, would
    // WRITE that misreading back the moment the dialog appeared. That is inert only
    // while the upper two radios are disabled; it goes live with the chart-datum
    // source (PR2).
    switch (state.anchor_mode) {
      case ShorelineAnchor::Source::ChartDatum:   chart_radio->setChecked(true); break;
      case ShorelineAnchor::Source::PlatformTide: tide_radio->setChecked(true); break;
      case ShorelineAnchor::Source::Manual:       manual_radio->setChecked(true); break;
      case ShorelineAnchor::Source::None:         none_radio->setChecked(true); break;
    }
    // A persisted manual value seeds the spin; its ABSENCE seeds "not set", never
    // 0.0 (ADR-0015 D6) — so selecting Manual on a layer that has never had one
    // applies no anchor at all rather than a silently-wrong sea-level break.
    if (state.manual_anchor) {
      anchor_spin->setValue(*state.manual_anchor);
    } else {
      anchor_spin->setValue(kAnchorUnset);   // reads "not set"
    }

    // The widget state as an (mode, manual value) pair — the same shape the layer's
    // holder stores. The mode is read from ALL FOUR radios, so a stored ChartDatum /
    // PlatformTide selection survives a dialog visit untouched.
    auto current_anchor =
      [chart_radio, tide_radio, manual_radio, anchor_spin]() {
        ShorelineAnchor::Source mode = ShorelineAnchor::Source::None;
        if (chart_radio->isChecked()) {
          mode = ShorelineAnchor::Source::ChartDatum;
        } else if (tide_radio->isChecked()) {
          mode = ShorelineAnchor::Source::PlatformTide;
        } else if (manual_radio->isChecked()) {
          mode = ShorelineAnchor::Source::Manual;
        }
        // [ADR-0015 D6] A spin sitting at the "not set" sentinel yields NO value.
        // Manual-with-nothing-entered therefore resolves to unanchored (and says
        // so), which is the whole point: there is no path from one click to a 0.0
        // anchor.
        return std::pair<ShorelineAnchor::Source, std::optional<double>>(
          mode, spin_value(anchor_spin));
      };

    // Repaint the colorbar exactly as the layer renders it (the anchored bake over
    // the resolved [lo, hi]) and update the readout. Touches the DIALOG only —
    // nothing is pushed to the layer from here, so it is safe to call on open.
    auto repaint_anchor =
      [legend, anchor_palette, anchor_spin, anchor_readout, current_anchor,
       domain_widened]() {
        const auto [mode, typed] = current_anchor();
        anchor_spin->setEnabled(mode == ShorelineAnchor::Source::Manual);
        // PR1 can only resolve Manual; the upper two sources have no provider yet
        // and are reported as unavailable rather than faked (D4 / D5).
        const std::optional<double> value =
          mode == ShorelineAnchor::Source::Manual ? typed : std::nullopt;
        // Bake the anchor into the colorbar only where the MAP can apply it. The
        // map's range is the layer's resolved range: while the bar tracks Auto that
        // is the data extent, which on the widened path is degenerate (see
        // domain_widened above) and renders unanchored; once the operator pins a
        // Manual range the map has a real range again and anchors, so the bar must
        // follow it back. A Manual range typed as a single point is degenerate for
        // both.
        const bool anchorable_range =
          legend->hi() > legend->lo() &&
          !(domain_widened && legend->mode() != marine_colormap::RangeMode::Manual);
        if (anchor_palette && value && anchorable_range) {
          legend->setLut(marine_colormap::bake_shoreline_anchored_lut(
            *anchor_palette, marine_colormap::TransferParams{},
            legend->lo(), legend->hi(),
            static_cast<float>(*value), 256));
        } else {
          legend->setLut({});   // fall back to the plain palette ramp
        }
        // Readout naming the active anchor AND its source (S-98 permanent
        // indication; D5 report-the-degraded-state).
        if (value) {
          QString text = QString("Shoreline at %1 m (%2)")
                           .arg(*value, 0, 'f', 3)
                           .arg(ShorelineAnchor::sourceLabel(mode));
          // Report the degraded states in the order they override each other: an
          // unanchorable range means nothing is anchored at all, so it is said
          // first. Otherwise, an anchor outside the render range is ORDINARY, not
          // an error (a survey line with no land in view has its break above hi) —
          // but BreakpointMap clamps it to an endpoint and the ramp goes
          // single-sided. Say so, rather than let the colorbar imply a land/sea
          // break that is not on it.
          if (!anchorable_range) {
            text += " - range too narrow to anchor; ramp is unanchored";
          } else if (*value < legend->lo() || *value > legend->hi()) {
            text += " - outside the range; ramp is single-sided";
          }
          anchor_readout->setText(text);
        } else if (mode == ShorelineAnchor::Source::Manual) {
          // Never "shoreline 0.00 m" — an unset Manual is reported as what it is.
          anchor_readout->setText("Unanchored - enter a manual value");
        } else if (mode != ShorelineAnchor::Source::None) {
          anchor_readout->setText(QString("Unanchored - %1 unavailable")
                                    .arg(ShorelineAnchor::sourceLabel(mode)));
        } else {
          anchor_readout->setText("Unanchored");
        }
      };

    // Push the operator's change to the layer. Fires live, and ONLY from a real
    // interaction — never from seeding.
    auto apply_anchor = [repaint_anchor, current_anchor, on_anchor]() {
      repaint_anchor();
      if (on_anchor) {
        const auto [mode, typed] = current_anchor();
        on_anchor(mode, typed);
      }
    };

    QObject::connect(manual_radio, &QRadioButton::toggled, &dialog,
                     [apply_anchor](bool) { apply_anchor(); });
    QObject::connect(none_radio, &QRadioButton::toggled, &dialog,
                     [apply_anchor](bool) { apply_anchor(); });
    // editingFinished, not valueChanged: the range spins in this dialog already use
    // it, and per-keystroke firing would repaint the map and rewrite QSettings on
    // every digit typed — and would briefly apply half-typed anchors ("-2" on the
    // way to "-28") as though the operator had chosen them.
    QObject::connect(anchor_spin, &QDoubleSpinBox::editingFinished, &dialog,
                     [apply_anchor]() { apply_anchor(); });
    // A range change (drag / spin / reset) must re-bake the anchored colorbar over
    // the new [lo, hi] — the anchored LUT is range-dependent (ADR-0015). This is a
    // DIALOG repaint only: the range change itself already went to the layer, which
    // re-bakes its own LUT; re-pushing the anchor here would be a write the operator
    // did not ask for.
    QObject::connect(
      legend, &marine_colormap_widgets::ColormapLegendWidget::rangeChanged, &dialog,
      [repaint_anchor](float, float) { repaint_anchor(); });

    // Paint the colorbar + readout to match the SEEDED state. Deliberately
    // repaint-only: opening the dialog must not write anything back to the layer.
    repaint_anchor();
  }

  auto * buttons = new QHBoxLayout();
  auto * reset_btn = new QPushButton("Reset to auto", &dialog);
  QObject::connect(reset_btn, &QPushButton::clicked, legend, [legend, domain_min, domain_max]() {
    legend->reset();                          // model -> Auto (emits rangeChanged -> on_reset)
    legend->updateAuto(domain_min, domain_max);  // move handles back across the domain
  });
  auto * close_btn = new QPushButton("Close", &dialog);
  close_btn->setDefault(true);
  QObject::connect(close_btn, &QPushButton::clicked, &dialog, &QDialog::accept);
  buttons->addWidget(reset_btn);
  buttons->addStretch();
  buttons->addWidget(close_btn);
  layout->addLayout(buttons);

  dialog.exec();
}

}  // namespace raster
}  // namespace camp
