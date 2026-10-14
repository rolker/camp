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

#include "opacity_range_dialog.h"

#include <QButtonGroup>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QPushButton>
#include <QRadioButton>
#include <QSignalBlocker>
#include <QVBoxLayout>

namespace camp
{
namespace raster
{

void showOpacityRangeDialog(
  QWidget * parent, const QString & title, const OpacityRamp & current,
  std::function<void(const OpacityRamp &)> on_change)
{
  QDialog dialog(parent);
  dialog.setWindowTitle(title);
  auto * layout = new QVBoxLayout(&dialog);
  layout->addWidget(new QLabel(
    "Cells are fully opaque where their uncertainty is low and fade to fully "
    "transparent as it rises. Uncertainty is the CUBE 95 % (1.96 sigma) figure, "
    "the same confidence level the IHO S-44 limits are written at.", &dialog));

  auto * standard_radio = new QRadioButton("Survey standard (per cell depth)", &dialog);
  auto * manual_radio = new QRadioButton("Manual thresholds", &dialog);
  auto * group = new QButtonGroup(&dialog);
  group->addButton(standard_radio);
  group->addButton(manual_radio);

  auto spin = [&dialog](double lo, double hi, int decimals, const char * suffix) {
    auto * s = new QDoubleSpinBox(&dialog);
    s->setRange(lo, hi);
    s->setDecimals(decimals);
    s->setSuffix(suffix);
    return s;
  };
  auto * a_spin = spin(0.0, 1.0e3, 4, " m");
  auto * b_spin = spin(0.0, 1.0, 5, "");
  auto * mult_spin = spin(1.0, 1.0e3, 2, " x");
  auto * lo_spin = spin(0.0, 1.0e6, 4, " m");
  auto * hi_spin = spin(0.0, 1.0e6, 4, " m");

  layout->addWidget(standard_radio);
  auto * std_form = new QFormLayout();
  std_form->addRow("a (IHO S-44 Order 1a: 0.5):", a_spin);
  std_form->addRow("b (IHO S-44 Order 1a: 0.013):", b_spin);
  std_form->addRow("Transparent at:", mult_spin);
  layout->addLayout(std_form);
  layout->addWidget(new QLabel(
    "Opaque at sqrt(a^2 + (b*depth)^2); transparent at the multiple of that.", &dialog));
  layout->addWidget(manual_radio);
  auto * man_form = new QFormLayout();
  man_form->addRow("Opaque at or below:", lo_spin);
  man_form->addRow("Transparent at or above:", hi_spin);
  layout->addLayout(man_form);

  auto load = [=](const OpacityRamp & r) {
    const QSignalBlocker b1(a_spin), b2(b_spin), b3(mult_spin), b4(lo_spin), b5(hi_spin),
      b6(standard_radio), b7(manual_radio);
    a_spin->setValue(r.a);
    b_spin->setValue(r.b);
    mult_spin->setValue(r.multiple);
    lo_spin->setValue(r.lo);
    hi_spin->setValue(r.hi);
    standard_radio->setChecked(r.kind == OpacityRamp::Kind::Standard);
    manual_radio->setChecked(r.kind == OpacityRamp::Kind::Manual);
    a_spin->setEnabled(r.kind == OpacityRamp::Kind::Standard);
    b_spin->setEnabled(r.kind == OpacityRamp::Kind::Standard);
    mult_spin->setEnabled(r.kind == OpacityRamp::Kind::Standard);
    lo_spin->setEnabled(r.kind == OpacityRamp::Kind::Manual);
    hi_spin->setEnabled(r.kind == OpacityRamp::Kind::Manual);
  };
  load(current);

  auto read = [=]() {
    OpacityRamp r;
    r.kind = manual_radio->isChecked() ? OpacityRamp::Kind::Manual
                                       : OpacityRamp::Kind::Standard;
    r.a = static_cast<float>(a_spin->value());
    r.b = static_cast<float>(b_spin->value());
    r.multiple = static_cast<float>(mult_spin->value());
    r.lo = static_cast<float>(lo_spin->value());
    r.hi = static_cast<float>(hi_spin->value());
    // Same rule readSettings() applies on restore, so what is applied now is what comes
    // back after a restart: a Manual ramp needs hi > lo.
    if (r.kind == OpacityRamp::Kind::Manual && r.hi <= r.lo) {
      r.hi = r.lo + 0.01f;
    }
    return r;
  };
  auto changed = [=]() {
    const OpacityRamp r = read();
    load(r);
    if (on_change) {
      on_change(r);
    }
  };
  QObject::connect(standard_radio, &QRadioButton::toggled, &dialog, changed);
  QObject::connect(manual_radio, &QRadioButton::toggled, &dialog, changed);
  for (QDoubleSpinBox * s : {a_spin, b_spin, mult_spin, lo_spin, hi_spin}) {
    QObject::connect(s, &QDoubleSpinBox::editingFinished, &dialog, changed);
  }

  auto * buttons = new QDialogButtonBox(QDialogButtonBox::Close, &dialog);
  auto * reset = buttons->addButton("Reset to default", QDialogButtonBox::ResetRole);
  QObject::connect(reset, &QPushButton::clicked, &dialog, [=]() {
    const OpacityRamp def;   // IHO S-44 Order 1a, 3x
    load(def);
    if (on_change) {
      on_change(def);
    }
  });
  QObject::connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::accept);
  layout->addWidget(buttons);

  dialog.exec();
}

}  // namespace raster
}  // namespace camp
