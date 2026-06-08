#ifndef MAP_COLOR_MAP_H
#define MAP_COLOR_MAP_H

#include <QColor>
#include <QString>
#include <QList>

namespace camp
{
namespace map
{

/// Maps a scalar value to a QColor through a named, perceptually-reasonable
/// colour ramp. A single reusable facility for camp's scalar-field renderers
/// (grid_map, depth shading) so each subsystem stops reimplementing its own
/// value->colour ramp. See camp#63.
class ColorMap
{
public:
  enum Type
  {
    Grayscale,   ///< linear black -> white
    Viridis,     ///< perceptually-uniform purple -> yellow
    Turbo        ///< high-contrast blue -> red rainbow (improved jet)
  };

  explicit ColorMap(Type type = Grayscale);

  Type type() const;
  void setType(Type type);

  /// Map @p value within [@p min, @p max] to a colour. The value is clamped to
  /// the range. A NaN value, or a degenerate range (max <= min), yields a fully
  /// transparent colour so callers can treat "no data" uniformly.
  QColor color(double value, double min, double max) const;

  /// Map an already-normalised value in [0,1] (clamped) to a colour.
  /// NaN yields a fully transparent colour.
  QColor colorNormalized(double t) const;

  /// Display name <-> Type, for UI population and settings persistence.
  static QString name(Type type);
  static Type typeFromName(const QString& name, bool* ok = nullptr);
  static QList<Type> allTypes();

private:
  Type type_;
};

} // namespace map
} // namespace camp

#endif
