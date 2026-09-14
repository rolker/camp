// [camp#22] Attribute-driven styling for the read-only vector layer.
//
// Colour-by-field and size-by-field are the part of this feature that meets real
// data and can be quietly wrong: a field that is missing on some features, a
// field whose values are all identical (divide by zero), a field holding text or
// NaN. The rule that matters is that none of those may LOOK like a legitimate
// low value — an operator reading a magnetic-anomaly layer must not mistake "no
// measurement here" for "the smallest measurement here".
//
// The mapping lives in free functions (vector_style.h) precisely so it can be
// exercised like this: no QApplication, no GL, no Map model.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include <marine_colormap/color.hpp>
#include <marine_colormap/palette.hpp>

#include "vector/vector_parse.h"
#include "vector/vector_style.h"

using camp::vector::FieldRange;
using camp::vector::ParsedGeometry;
using camp::vector::accumulateValue;
using camp::vector::colorForValue;
using camp::vector::fieldRange;
using camp::vector::kDefaultPointRadius;
using camp::vector::kMaxPointRadius;
using camp::vector::kMinPointRadius;
using camp::vector::noDataColor;
using camp::vector::normalizedValue;
using camp::vector::numericAttribute;
using camp::vector::radiusForValue;

namespace
{

ParsedGeometry pointWith(const QMap<QString, QVariant>& attributes)
{
  ParsedGeometry g;
  g.type = ParsedGeometry::Point;
  g.exterior.push_back(QGeoCoordinate(43.0, -70.0));
  g.attributes = attributes;
  return g;
}

QColor paletteColorAt(const char* palette_name, double t)
{
  const marine_colormap::Palette* palette = marine_colormap::find_palette(palette_name);
  EXPECT_NE(palette, nullptr);
  const marine_colormap::Rgba8 c = marine_colormap::to_rgba8(palette->sample(static_cast<float>(t)));
  return QColor(c.r, c.g, c.b, c.a);
}

}  // namespace

// A number is a number whether OGR gave it as int, real, or a quoted string; a
// non-number (text, NaN) is absent, NOT zero.
TEST(VectorLayerStyling, NumericAttributeReadsOnlyRealNumbers)
{
  QMap<QString, QVariant> attributes;
  attributes["real"] = 58.0;
  attributes["int"] = 7;
  attributes["quoted"] = QString("12.5");
  attributes["text"] = QString("candidate C");
  attributes["nan"] = std::numeric_limits<double>::quiet_NaN();

  EXPECT_DOUBLE_EQ(*numericAttribute(attributes, "real"), 58.0);
  EXPECT_DOUBLE_EQ(*numericAttribute(attributes, "int"), 7.0);
  EXPECT_DOUBLE_EQ(*numericAttribute(attributes, "quoted"), 12.5);
  EXPECT_FALSE(numericAttribute(attributes, "text").has_value());
  EXPECT_FALSE(numericAttribute(attributes, "nan").has_value());
  EXPECT_FALSE(numericAttribute(attributes, "absent").has_value());
  EXPECT_FALSE(numericAttribute(attributes, QString()).has_value());
}

// The extent spans only the features that HAVE a value: a feature missing the
// field must not drag the range toward zero.
TEST(VectorLayerStyling, RangeIgnoresFeaturesWithoutAValue)
{
  std::vector<ParsedGeometry> geometries;
  geometries.push_back(pointWith({{"signal", 10.0}}));
  geometries.push_back(pointWith({{"signal", 58.0}}));
  geometries.push_back(pointWith({{"other", 0.0}}));                      // no signal
  geometries.push_back(pointWith({{"signal", QString("not a number")}})); // unusable

  const FieldRange range = fieldRange(geometries, "signal");
  ASSERT_TRUE(range.valid);
  EXPECT_DOUBLE_EQ(range.min, 10.0);
  EXPECT_DOUBLE_EQ(range.max, 58.0);

  const FieldRange none = fieldRange(geometries, "missing");
  EXPECT_FALSE(none.valid);
}

// min -> 0, max -> 1, midpoint -> 0.5, out of range clamps.
TEST(VectorLayerStyling, NormalizationSpansTheRange)
{
  FieldRange range;
  accumulateValue(range, 10.0);
  accumulateValue(range, 50.0);

  EXPECT_DOUBLE_EQ(normalizedValue(10.0, range), 0.0);
  EXPECT_DOUBLE_EQ(normalizedValue(50.0, range), 1.0);
  EXPECT_DOUBLE_EQ(normalizedValue(30.0, range), 0.5);
  EXPECT_DOUBLE_EQ(normalizedValue(-5.0, range), 0.0);
  EXPECT_DOUBLE_EQ(normalizedValue(999.0, range), 1.0);
}

// Every feature holding the same value: the MIDDLE of the palette for all of them
// — not a NaN from dividing by zero, and not a fallback style. There is nothing to
// distinguish, so neither extreme would be honest.
TEST(VectorLayerStyling, EqualValuesMapToTheMiddleOfTheRamp)
{
  std::vector<ParsedGeometry> geometries;
  geometries.push_back(pointWith({{"flat", 4.0}}));
  geometries.push_back(pointWith({{"flat", 4.0}}));

  const FieldRange range = fieldRange(geometries, "flat");
  ASSERT_TRUE(range.valid);
  const double t = normalizedValue(4.0, range);
  EXPECT_FALSE(std::isnan(t));
  EXPECT_DOUBLE_EQ(t, 0.5);

  const marine_colormap::Palette* palette = marine_colormap::find_palette("viridis");
  EXPECT_EQ(colorForValue(palette, 4.0, range, Qt::darkCyan), paletteColorAt("viridis", 0.5));
}

// [camp#22 must-fix 8] The degenerate-range guard has to be TOTAL, not merely
// adequate on small numbers.
//
// The previous guard offset the low bound by one unit and divided by that. Above
// 2^53 — OGR int64 feature ids, nanosecond timestamps, large counters, all
// perfectly ordinary attribute fields — `max - 1.0 == max`: the offset is a no-op,
// the span stays zero, and 0.0/0.0 puts a NaN into Palette::sample() and into the
// radius arithmetic. A NaN radius is not a visible failure; it is a marker that
// silently does not draw.
TEST(VectorLayerStyling, DegenerateRangeIsTotalAboveTwoToTheFiftyThree)
{
  const marine_colormap::Palette* palette = marine_colormap::find_palette("viridis");
  ASSERT_NE(palette, nullptr);

  // 2^53 and beyond: adding or subtracting 1.0 changes nothing.
  for(const double v : {9007199254740992.0,            // 2^53
                        1.8e19,                         // beyond int64
                        1.757e18,                       // a nanosecond timestamp
                        -9007199254740992.0})
  {
    FieldRange range;
    accumulateValue(range, v);
    accumulateValue(range, v);
    ASSERT_TRUE(range.valid);
    ASSERT_DOUBLE_EQ(range.max - 1.0, range.max) << "fixture must be past the 1.0 ulp boundary";

    const double t = normalizedValue(v, range);
    EXPECT_FALSE(std::isnan(t)) << "value " << v;
    EXPECT_GE(t, 0.0);
    EXPECT_LE(t, 1.0);

    // Nothing NaN reaches the palette or the radius.
    const QColor color = colorForValue(palette, v, range, Qt::darkCyan);
    EXPECT_TRUE(color.isValid());
    const double radius =
      radiusForValue(v, range, kMinPointRadius, kMaxPointRadius, kDefaultPointRadius);
    EXPECT_FALSE(std::isnan(radius)) << "value " << v;
    EXPECT_GE(radius, kMinPointRadius);
    EXPECT_LE(radius, kMaxPointRadius);
  }

  // A range so wide the arithmetic overflows still yields a usable position.
  FieldRange huge;
  accumulateValue(huge, -std::numeric_limits<double>::max());
  accumulateValue(huge, std::numeric_limits<double>::max());
  for(const double v : {-std::numeric_limits<double>::max(), 0.0,
                        std::numeric_limits<double>::max()})
  {
    const double t = normalizedValue(v, huge);
    EXPECT_FALSE(std::isnan(t)) << "value " << v;
    EXPECT_GE(t, 0.0);
    EXPECT_LE(t, 1.0);
    EXPECT_FALSE(std::isnan(
      radiusForValue(v, huge, kMinPointRadius, kMaxPointRadius, kDefaultPointRadius)));
  }
}

// The ends of the data map to the ends of the palette.
TEST(VectorLayerStyling, ColorSpansThePalette)
{
  FieldRange range;
  accumulateValue(range, 0.0);
  accumulateValue(range, 58.0);
  const marine_colormap::Palette* palette = marine_colormap::find_palette("viridis");
  ASSERT_NE(palette, nullptr);

  EXPECT_EQ(colorForValue(palette, 0.0, range, Qt::darkCyan), paletteColorAt("viridis", 0.0));
  EXPECT_EQ(colorForValue(palette, 58.0, range, Qt::darkCyan), paletteColorAt("viridis", 1.0));
  EXPECT_EQ(colorForValue(palette, 29.0, range, Qt::darkCyan), paletteColorAt("viridis", 0.5));
}

// The rule with teeth: a feature with no usable value gets the documented
// no-data colour, which is NOT the palette's first entry.
TEST(VectorLayerStyling, MissingValueGetsNoDataColorNotPaletteZero)
{
  FieldRange range;
  accumulateValue(range, 10.0);
  accumulateValue(range, 58.0);
  const marine_colormap::Palette* palette = marine_colormap::find_palette("viridis");

  const QColor missing = colorForValue(palette, std::nullopt, range, Qt::darkCyan);
  EXPECT_EQ(missing, noDataColor());
  EXPECT_NE(missing, paletteColorAt("viridis", 0.0));

  // No feature at all has a value -> every feature is no-data, not palette zero.
  const FieldRange invalid;
  EXPECT_EQ(colorForValue(palette, 42.0, invalid, Qt::darkCyan), noDataColor());
}

// Size-by-field spans the marker range, and an unusable value gets the DEFAULT
// radius rather than the smallest one.
TEST(VectorLayerStyling, RadiusSpansTheMarkerRange)
{
  FieldRange range;
  accumulateValue(range, 1.0);
  accumulateValue(range, 5.0);

  EXPECT_DOUBLE_EQ(radiusForValue(1.0, range, kMinPointRadius, kMaxPointRadius, kDefaultPointRadius),
                   kMinPointRadius);
  EXPECT_DOUBLE_EQ(radiusForValue(5.0, range, kMinPointRadius, kMaxPointRadius, kDefaultPointRadius),
                   kMaxPointRadius);
  EXPECT_DOUBLE_EQ(radiusForValue(3.0, range, kMinPointRadius, kMaxPointRadius, kDefaultPointRadius),
                   (kMinPointRadius + kMaxPointRadius) / 2.0);

  EXPECT_DOUBLE_EQ(
    radiusForValue(std::nullopt, range, kMinPointRadius, kMaxPointRadius, kDefaultPointRadius),
    kDefaultPointRadius);
  const FieldRange invalid;
  EXPECT_DOUBLE_EQ(
    radiusForValue(2.0, invalid, kMinPointRadius, kMaxPointRadius, kDefaultPointRadius),
    kDefaultPointRadius);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
