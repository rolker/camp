#include "color_map.h"

#include <algorithm>   // std::min / std::max in colorNormalized()
#include <array>
#include <cmath>
#include <vector>

namespace camp
{
namespace map
{

namespace
{

// A control point in a colour ramp: normalised position [0,1] and its RGB (0-255).
struct Stop
{
  double t;
  int r, g, b;
};

// Viridis (matplotlib) sampled at 7 stops — perceptually uniform.
const std::vector<Stop>& viridisStops()
{
  static const std::vector<Stop> stops = {
    {0.000,  68,   1,  84},
    {0.167,  65,  68, 135},
    {0.333,  42, 120, 142},
    {0.500,  34, 168, 132},
    {0.667, 122, 209,  81},
    {0.833, 189, 223,  38},
    {1.000, 253, 231,  37},
  };
  return stops;
}

// Turbo (Google) sampled at 8 stops — high-contrast, improved jet.
const std::vector<Stop>& turboStops()
{
  static const std::vector<Stop> stops = {
    {0.000,  48,  18,  59},
    {0.143,  64, 116, 240},
    {0.286,  30, 184, 199},
    {0.429,  76, 220,  99},
    {0.571, 191, 211,  47},
    {0.714, 251, 162,  44},
    {0.857, 230,  79,  21},
    {1.000, 122,   4,   3},
  };
  return stops;
}

int lerp(int a, int b, double f)
{
  return static_cast<int>(std::lround(a + (b - a) * f));
}

QColor sampleStops(const std::vector<Stop>& stops, double t)
{
  if(t <= stops.front().t)
    return QColor(stops.front().r, stops.front().g, stops.front().b);
  if(t >= stops.back().t)
    return QColor(stops.back().r, stops.back().g, stops.back().b);
  for(size_t i = 1; i < stops.size(); ++i)
  {
    if(t <= stops[i].t)
    {
      const Stop& lo = stops[i - 1];
      const Stop& hi = stops[i];
      const double span = hi.t - lo.t;
      const double f = span > 0.0 ? (t - lo.t) / span : 0.0;
      return QColor(lerp(lo.r, hi.r, f), lerp(lo.g, hi.g, f), lerp(lo.b, hi.b, f));
    }
  }
  return QColor(stops.back().r, stops.back().g, stops.back().b);
}

} // namespace

ColorMap::ColorMap(Type type): type_(type)
{
}

ColorMap::Type ColorMap::type() const
{
  return type_;
}

void ColorMap::setType(Type type)
{
  type_ = type;
}

QColor ColorMap::color(double value, double min, double max) const
{
  if(std::isnan(value) || !(max > min))
    return QColor(0, 0, 0, 0);
  return colorNormalized((value - min) / (max - min));
}

QColor ColorMap::colorNormalized(double t) const
{
  if(std::isnan(t))
    return QColor(0, 0, 0, 0);
  t = std::min(1.0, std::max(0.0, t));
  switch(type_)
  {
    case Viridis:
      return sampleStops(viridisStops(), t);
    case Turbo:
      return sampleStops(turboStops(), t);
    case Grayscale:
    default:
    {
      const int v = static_cast<int>(std::lround(t * 255.0));
      return QColor(v, v, v);
    }
  }
}

QString ColorMap::name(Type type)
{
  switch(type)
  {
    case Viridis: return QStringLiteral("viridis");
    case Turbo:   return QStringLiteral("turbo");
    case Grayscale:
    default:      return QStringLiteral("grayscale");
  }
}

ColorMap::Type ColorMap::typeFromName(const QString& name, bool* ok)
{
  if(ok)
    *ok = true;
  if(name == QStringLiteral("viridis")) return Viridis;
  if(name == QStringLiteral("turbo"))   return Turbo;
  if(name == QStringLiteral("grayscale")) return Grayscale;
  if(ok)
    *ok = false;
  return Grayscale;
}

QList<ColorMap::Type> ColorMap::allTypes()
{
  return {Grayscale, Viridis, Turbo};
}

} // namespace map
} // namespace camp
