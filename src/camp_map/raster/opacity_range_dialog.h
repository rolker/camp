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

#ifndef CAMP_MAP__RASTER__OPACITY_RANGE_DIALOG_H_
#define CAMP_MAP__RASTER__OPACITY_RANGE_DIALOG_H_

#include <functional>

#include <QString>

#include "opacity_ramp.h"

class QWidget;

namespace camp
{
namespace raster
{

/// [camp#239 / ADR-0017] Modal dialog (ADR-0009 placement) for the uncertainty-opacity
/// ramp of a scalar layer. Two sources of thresholds:
///  - "Survey standard": opaque at the standard's allowed uncertainty for the cell's
///    own depth, sqrt(a^2 + (b*d)^2), transparent at `multiple` times that. Defaults
///    to IHO S-44 Order 1a (a = 0.5 m, b = 0.013) and 3x; a, b and the multiple are
///    editable so another order can be typed in.
///  - "Manual": absolute uncertainty thresholds [opaque at/below, transparent at/above]
///    in the uncertainty band's unit (metres for CUBE).
/// Every edit calls `on_change(ramp)` live while the dialog is open so the map
/// re-renders behind it. "Reset to default" restores the shipped standard and calls
/// on_change with it. Opening the dialog pushes nothing: inspecting must not mutate.
void showOpacityRangeDialog(
  QWidget * parent, const QString & title, const OpacityRamp & current,
  std::function<void(const OpacityRamp &)> on_change);

}  // namespace raster
}  // namespace camp

#endif  // CAMP_MAP__RASTER__OPACITY_RANGE_DIALOG_H_
