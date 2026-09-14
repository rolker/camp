#ifndef CAMP_VECTOR_STYLE_H
#define CAMP_VECTOR_STYLE_H

#include <optional>
#include <vector>

#include <QColor>
#include <QMap>
#include <QString>
#include <QVariant>

namespace marine_colormap { class Palette; }

namespace camp::vector
{

struct ParsedGeometry;

/// [camp#22] Attribute-driven styling for a read-only vector layer, as free
/// functions rather than VectorLayer methods so the mapping is unit-testable
/// without a QApplication, a GL context, or a Map model — the same reason
/// `vector_parse` is its own TU.
///
/// Every function here is total: there is a defined, documented result for a
/// field that is missing on a feature, holds a non-numeric or NaN value, or is
/// identical across every feature. Those are the cases a field-styled layer meets
/// on real data, and the one thing none of them may do is quietly look like a
/// legitimate low value.

/// The numeric extent of one attribute field over a set of features. `valid` is
/// false when NO feature carries a numeric value for the field — a string-only
/// field, a misspelled name, an empty layer.
struct FieldRange
{
  bool valid = false;
  double min = 0.0;
  double max = 0.0;
};

/// The feature's value for @p field as a number, or nullopt when the field is
/// absent from this feature or its value is not a finite number. A numeric string
/// converts (a GeoJSON property quoted by its writer is still a measurement);
/// anything else — text, NaN, infinity — is nullopt, not 0.
std::optional<double> numericAttribute(const QMap<QString, QVariant>& attributes,
                                       const QString& field);

/// Fold one feature's value into a running range. `nullopt` (field missing or
/// non-numeric) is ignored, which is what keeps the extent computed over only the
/// features that actually have a value. The one place the fold is written, shared
/// by `fieldRange()` and by VectorLayer's per-style-change pass over its features.
void accumulateValue(FieldRange& range, const std::optional<double>& value);

/// Extent of @p field over @p geometries, computed ONLY over the features that
/// have a present, finite, numeric value for it. Features without one neither
/// widen the range nor drag it toward zero.
FieldRange fieldRange(const std::vector<ParsedGeometry>& geometries, const QString& field);

/// Normalize @p value onto [0, 1] within @p range. NEVER returns NaN.
///
/// Degenerate range (every feature holding the same value) returns 0.5, the
/// middle of the palette: there is nothing to distinguish, so the honest reading
/// is "uniform" rather than either extreme. This is deliberately NOT the grid
/// renderer's `low = max - 1.0` offset (`ros/grids/grid_map.cpp`), which is a
/// no-op wherever 1.0 is below the value's precision — above 2^53 (OGR int64 ids,
/// nanosecond timestamps) the span stays zero and the division yields NaN, which
/// then reaches `Palette::sample()` and the radius arithmetic. The guard here is
/// total for every finite double.
///
/// Out-of-range values clamp. A range so wide that the arithmetic overflows falls
/// back to the value's ORDER relative to the bounds. An invalid range (no feature
/// has a value) returns 1.0 — nothing to distinguish, and the caller paints
/// no-data anyway.
double normalizedValue(double value, const FieldRange& range);

/// The fixed "no data" colour: a feature whose styling field is missing or
/// non-numeric. Deliberately a neutral grey that is NOT a position on any
/// palette, so "this feature has no value" never reads as "this feature is at
/// the bottom of the ramp".
QColor noDataColor();

/// Resolved fill colour for one feature.
/// - @p value empty (field missing / non-numeric) -> noDataColor()
/// - @p range invalid (no feature has a value)    -> noDataColor()
/// - @p palette null                              -> @p default_color
/// - otherwise                                     -> the palette sampled at the
///   normalized value.
QColor colorForValue(const marine_colormap::Palette* palette,
                     const std::optional<double>& value,
                     const FieldRange& range,
                     const QColor& default_color);

/// Marker radius in device pixels for one point feature. A missing or
/// non-numeric value — or an invalid range — gets @p default_radius, NOT a
/// computed one, so an unstyled feature is not silently drawn as the smallest.
double radiusForValue(const std::optional<double>& value,
                      const FieldRange& range,
                      double min_radius,
                      double max_radius,
                      double default_radius);

/// Point marker geometry, in device pixels (the point items ignore the view
/// transform, so these are screen sizes, not scene metres).
constexpr double kDefaultPointRadius = 5.0;
constexpr double kMinPointRadius = 3.0;
constexpr double kMaxPointRadius = 15.0;

}  // namespace camp::vector

#endif  // CAMP_VECTOR_STYLE_H
