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
using camp::vector::isNoData;
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

  // [camp#22] An INVALID range — no feature in the layer had a value for the
  // field — answers the same 0.5 as a degenerate one below: both are "nothing
  // distinguishes these features", and the two used to disagree (1.0 vs 0.5) with
  // no reader to notice.
  const FieldRange invalid;
  ASSERT_FALSE(invalid.valid);
  EXPECT_DOUBLE_EQ(normalizedValue(42.0, invalid), 0.5);
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

// [camp#22 should-fix, round 2] An overflowing span must still ORDER the data.
//
// With min = -DBL_MAX and max = DBL_MAX the span is +inf, and (value - min)/inf is
// the finite value 0.0 for everything up to about 1e308 — so the !isfinite(t)
// guard never fired and the result was not merely compressed but NON-monotonic:
// 0 -> 0.0, -1e100 -> 0.0, 1e100 -> 0.0, yet 1e307 -> 0.5. A colour ramp and a
// marker radius built on that say the opposite of the data for some pairs of
// features, which is worse than a coarse ramp. The test above only asserts the
// result is finite and in [0, 1], which the broken arithmetic also satisfied;
// ORDER is the property that catches it.
TEST(VectorLayerStyling, OverflowingSpanStaysMonotonic)
{
  FieldRange huge;
  accumulateValue(huge, -std::numeric_limits<double>::max());
  accumulateValue(huge, std::numeric_limits<double>::max());
  ASSERT_TRUE(huge.valid);

  const std::vector<double> ascending = {-std::numeric_limits<double>::max(),
                                         -1e307, -1e100, 0.0, 1e100, 1e307,
                                         std::numeric_limits<double>::max()};
  double previous = -1.0;
  for(const double v : ascending)
  {
    const double t = normalizedValue(v, huge);
    EXPECT_FALSE(std::isnan(t)) << "value " << v;
    EXPECT_GE(t, 0.0) << "value " << v;
    EXPECT_LE(t, 1.0) << "value " << v;
    EXPECT_GE(t, previous) << "value " << v
                           << " normalised BELOW a smaller value — the ramp is"
                              " ordered wrongly, not just compressed";
    previous = t;
  }

  // The ends still reach the ends of the ramp, and the middle is not pinned to
  // either of them: a monotonic-but-constant mapping would pass the loop above.
  EXPECT_DOUBLE_EQ(normalizedValue(-std::numeric_limits<double>::max(), huge), 0.0);
  EXPECT_DOUBLE_EQ(normalizedValue(std::numeric_limits<double>::max(), huge), 1.0);
  EXPECT_NEAR(normalizedValue(0.0, huge), 0.5, 1e-12);
  EXPECT_GT(normalizedValue(1e307, huge), normalizedValue(-1e307, huge));
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

// [camp#22 should-fix] No-data is a STATE, not just a colour.
//
// noDataColor() is mid grey, and the grayscale palette — selectable by the
// operator, and the fallback for an unknown palette name — samples to very nearly
// that same grey at its midpoint. Under grayscale a missing value would therefore
// be indistinguishable from a mid-range measurement, which is two different kinds
// of thing looking alike. isNoData() names the state so VectorFeatureItem can also
// draw it with a dashed outline and a hatched fill, channels no palette touches.
//
// The predicate must agree exactly with the branch colorForValue() answers with
// noDataColor(), or the outline and the colour would disagree on some feature.
TEST(VectorLayerStyling, NoDataIsCarriedAsAStateNotOnlyAColour)
{
  FieldRange range;
  accumulateValue(range, 1.0);
  accumulateValue(range, 5.0);
  ASSERT_TRUE(range.valid);
  const FieldRange invalid;
  const QColor fallback(Qt::darkCyan);

  // The field is missing from this feature, or holds something non-numeric.
  EXPECT_TRUE(isNoData(std::nullopt, range));
  // No feature in the layer has a value: the whole layer is no-data.
  EXPECT_TRUE(isNoData(3.0, invalid));
  EXPECT_TRUE(isNoData(std::nullopt, invalid));
  // A real measurement inside a real range is not.
  EXPECT_FALSE(isNoData(3.0, range));

  // The predicate and the colour answer the same question the same way — with no
  // palette, colorForValue falls back to default_color for a REAL value, so the
  // grey is reached only on the no-data branch.
  EXPECT_EQ(colorForValue(nullptr, std::nullopt, range, fallback), noDataColor());
  EXPECT_EQ(colorForValue(nullptr, 3.0, invalid, fallback), noDataColor());
  EXPECT_EQ(colorForValue(nullptr, 3.0, range, fallback), fallback);
}

// [camp#22 round-12 must-fix] An ENGAGED optional holding NaN or infinity is
// no-data, in every channel.
//
// The contract has always been "empty, or not a finite number", but the predicate
// only checked engagement, so a non-finite value read as a real measurement: it
// reached normalizedValue(), whose own guards answer 0.5, and the feature was
// painted in the MIDDLE of the palette with a solid fill and a solid outline —
// indistinguishable from a real mid-range measurement, which is the exact
// confusion the no-data channels exist to prevent. numericAttribute() refuses a
// non-finite value today, so this is the contract holding for the NEXT producer
// (a computed field, an expression, a value taken straight off a ParsedGeometry),
// which has no reason to know it must pre-filter.
TEST(VectorLayerStyling, NonFiniteValuesAreNoDataNotAMidRangeMeasurement)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  FieldRange range;
  range.valid = true;
  range.min = 0.0;
  range.max = 10.0;

  EXPECT_TRUE(isNoData(nan, range));
  EXPECT_TRUE(isNoData(inf, range));
  EXPECT_TRUE(isNoData(-inf, range));
  EXPECT_FALSE(isNoData(5.0, range)) << "a real measurement is untouched";

  // And the colour follows the predicate, under a palette as well as without one.
  const marine_colormap::Palette* palette = marine_colormap::find_palette("viridis");
  ASSERT_NE(palette, nullptr);
  EXPECT_EQ(colorForValue(palette, nan, range, Qt::darkCyan), noDataColor());
  EXPECT_EQ(colorForValue(palette, inf, range, Qt::darkCyan), noDataColor());
  EXPECT_EQ(colorForValue(palette, -inf, range, Qt::darkCyan), noDataColor());
  EXPECT_NE(colorForValue(palette, 5.0, range, Qt::darkCyan), noDataColor());
}

// [camp#22 round-12 must-fix] The marker SIZE answers no-data exactly as the
// colour does, because it asks the same predicate.
//
// radiusForValue() repeated `!value || !range.valid` instead of calling
// isNoData(), so it carried the same non-finite hole: a NaN took
// normalizedValue()'s 0.5 and was drawn as a mid-sized marker. Two spellings of
// one condition also let size and colour disagree about which features are
// unstyled - a feature drawn grey and hatched at a computed size, or
// palette-coloured at the default size.
TEST(VectorLayerStyling, NonFiniteValuesGetTheDefaultRadiusNotAComputedOne)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  FieldRange range;
  range.valid = true;
  range.min = 0.0;
  range.max = 10.0;

  EXPECT_DOUBLE_EQ(
    radiusForValue(nan, range, kMinPointRadius, kMaxPointRadius, kDefaultPointRadius),
    kDefaultPointRadius);
  EXPECT_DOUBLE_EQ(
    radiusForValue(inf, range, kMinPointRadius, kMaxPointRadius, kDefaultPointRadius),
    kDefaultPointRadius);
  EXPECT_DOUBLE_EQ(
    radiusForValue(-inf, range, kMinPointRadius, kMaxPointRadius, kDefaultPointRadius),
    kDefaultPointRadius);
  // A real measurement still gets its computed size.
  EXPECT_DOUBLE_EQ(
    radiusForValue(10.0, range, kMinPointRadius, kMaxPointRadius, kDefaultPointRadius),
    kMaxPointRadius);

  // Size and colour agree, feature for feature, on what is unstyled.
  for(const double v : {nan, inf, -inf})
  {
    EXPECT_TRUE(isNoData(v, range));
    EXPECT_EQ(colorForValue(nullptr, v, range, Qt::darkCyan), noDataColor());
    EXPECT_DOUBLE_EQ(
      radiusForValue(v, range, kMinPointRadius, kMaxPointRadius, kDefaultPointRadius),
      kDefaultPointRadius);
  }
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
