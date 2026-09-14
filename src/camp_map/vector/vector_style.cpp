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
  if(!range.valid)
    return 1.0;
  double low = range.min;
  // Every value identical: offset the low bound rather than divide by zero, so
  // the field still reads as "present" (top of the ramp) instead of collapsing
  // to a fallback style. Mirrors the grid renderer's guard.
  if(!(low < range.max))
    low = range.max - 1.0;
  const double t = (value - low) / (range.max - low);
  return std::clamp(t, 0.0, 1.0);
}

QColor noDataColor()
{
  // Mid grey at full opacity: distinct from both ends of every shipped palette
  // and from the default unstyled colour below.
  return QColor(128, 128, 128);
}

QColor colorForValue(const marine_colormap::Palette* palette,
                     const std::optional<double>& value,
                     const FieldRange& range,
                     const QColor& default_color)
{
  if(!value || !range.valid)
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
