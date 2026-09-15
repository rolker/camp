#include "vector_style.h"

#include <algorithm>
#include <cmath>

#include <marine_colormap/color.hpp>
#include <marine_colormap/palette.hpp>

#include "vector_parse.h"

namespace camp::vector
{

std::optional<double> numericAttribute(const QMap<QString, QVariant>& attributes,
                                       const QString& field)
{
  if(field.isEmpty())
    return std::nullopt;
  const auto it = attributes.find(field);
  if(it == attributes.end())
    return std::nullopt;
  bool ok = false;
  const double value = it.value().toDouble(&ok);
  if(!ok || !std::isfinite(value))
    return std::nullopt;
  return value;
}

void accumulateValue(FieldRange& range, const std::optional<double>& value)
{
  if(!value)
    return;
  if(!range.valid)
  {
    range.valid = true;
    range.min = *value;
    range.max = *value;
    return;
  }
  range.min = std::min(range.min, *value);
  range.max = std::max(range.max, *value);
}

FieldRange fieldRange(const std::vector<ParsedGeometry>& geometries, const QString& field)
{
  FieldRange range;
  for(const auto& geometry : geometries)
    accumulateValue(range, numericAttribute(geometry.attributes, field));
  return range;
}

double normalizedValue(double value, const FieldRange& range)
{
  // [camp#22] No range at all — no feature in the layer held a numeric value for
  // this field. Nothing distinguishes one feature from another, which is the same
  // situation as the degenerate range below, so it gets the same answer: the
  // middle of the ramp, reading as "uniform" rather than as an extreme. (Both
  // callers guard on range.valid before reaching this, so neither case has a
  // reader today; they agree anyway, rather than disagreeing silently.)
  if(!range.valid)
    return 0.5;
  const double span = range.max - range.min;
  // [camp#22] Degenerate range — every feature holds the same value, so there is
  // nothing to distinguish and any position on the ramp is equally (un)true. Take
  // the middle of the palette, which reads as "uniform" rather than as an extreme.
  //
  // The previous guard offset the low bound by one unit (`low = max - 1.0`) and
  // divided by that. It is a NO-OP wherever 1.0 is below the value's precision:
  // above 2^53 — OGR int64 ids, nanosecond timestamps, large accumulated counters
  // — `max - 1.0 == max`, the span stays zero, and 0.0/0.0 puts a NaN into
  // Palette::sample() and into the radius arithmetic. `!(span > 0.0)` is total for
  // every finite double AND for a NaN span, which no arithmetic offset can be.
  if(!(span > 0.0))
    return 0.5;
  const double t = (value - range.min) / span;
  if(!std::isfinite(t))
  {
    // Reachable when the span or the numerator overflows to infinity (a range
    // spanning most of the double line, e.g. -DBL_MAX to DBL_MAX). Fall back to
    // the ORDER, which is still meaningful, rather than emitting a NaN.
    if(value >= range.max)
      return 1.0;
    if(value <= range.min)
      return 0.0;
    return 0.5;
  }
  return std::clamp(t, 0.0, 1.0);
}

QColor noDataColor()
{
  // Mid grey at full opacity: distinct from the default unstyled colour, and from
  // both ends of every shipped palette — but NOT from the MIDDLE of grayscale,
  // which is a palette the operator can select and the fallback for an unknown
  // name. Colour is therefore only half the answer; isNoData() carries the other
  // half into the outline and fill pattern, where no palette reaches.
  return QColor(128, 128, 128);
}

bool isNoData(const std::optional<double>& value, const FieldRange& range)
{
  return !value || !range.valid;
}

QColor colorForValue(const marine_colormap::Palette* palette,
                     const std::optional<double>& value,
                     const FieldRange& range,
                     const QColor& default_color)
{
  if(isNoData(value, range))
    return noDataColor();
  if(!palette)
    return default_color;
  const marine_colormap::Rgba8 c =
    marine_colormap::to_rgba8(palette->sample(static_cast<float>(normalizedValue(*value, range))));
  return QColor(c.r, c.g, c.b, c.a);
}

double radiusForValue(const std::optional<double>& value,
                      const FieldRange& range,
                      double min_radius,
                      double max_radius,
                      double default_radius)
{
  if(!value || !range.valid)
    return default_radius;
  return min_radius + normalizedValue(*value, range) * (max_radius - min_radius);
}

}  // namespace camp::vector
