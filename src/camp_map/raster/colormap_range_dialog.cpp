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

#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QVBoxLayout>

#include "marine_colormap_widgets/colormap_legend_widget.hpp"

namespace camp
{
namespace raster
{

void showColormapRangeDialog(
  QWidget * parent, const QString & title, const ColormapRangeState & state,
  std::function<void(float, float)> on_range, std::function<void()> on_reset)
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
  if (domain_min >= domain_max) {
    domain_min -= 0.5f;
    domain_max += 0.5f;
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
