// Unit tests for bake_anchored_lut() — the shoreline anchor (camp#181, ADR-0015).
//
// What this covers, and why each case is here rather than being obvious:
//
//  - The ANCHOR LANDS where it is asked to. This is the reported symptom in
//    camp#181: selecting the palette that should pin at 0 did not pin at 0.
//  - The UNANCHORED PATH IS BYTE-IDENTICAL to bake_lut(pal, TransferParams{}, n).
//    Anchoring must not silently recolour every existing layer, so this is
//    asserted per-byte, not approximately.
//  - The PALETTE GATE: a general-purpose ramp (grayscale, viridis) declares no
//    shoreline_position, so an anchor on it is a no-op rather than an arbitrary
//    warp. Without this, anchoring a non-topo-bathy layer would produce colours
//    that mean nothing.
//  - DEGENERATE INPUTS clamp and never throw. An anchor outside the range is the
//    ORDINARY case, not an error — a survey line with no land in view has its
//    datum break above hi — so it must produce a usable ramp, not an assert.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include <marine_colormap/colormap.hpp>
#include <marine_colormap/palette.hpp>
#include <marine_colormap/transfer.hpp>

#include "../src/camp_map/raster/anchored_lut.h"

namespace
{

const marine_colormap::Palette& palette(const std::string& name)
{
  const marine_colormap::Palette* p = marine_colormap::find_palette(name);
  EXPECT_NE(p, nullptr) << "built-in palette missing: " << name;
  return *p;
}

// The LUT index the shader would sample for `value`, given its LINEAR normalize.
// Mirroring the shader here (rather than reusing production code) is deliberate:
// if the two ever disagree, that disagreement IS the bug these tests exist to
// catch, and a shared helper would hide it.
std::size_t shader_index(float value, float lo, float hi, std::size_t n)
{
  const float t = std::clamp((value - lo) / (hi - lo), 0.0f, 1.0f);
  return static_cast<std::size_t>(std::lround(t * static_cast<float>(n - 1)));
}

}  // namespace

// --- The anchor lands at the palette's declared shoreline --------------------

TEST(AnchoredLut, AnchorLandsAtShorelinePosition)
{
  const marine_colormap::Palette& oleron = palette("oleron");
  ASSERT_TRUE(camp::raster::palette_supports_anchor(oleron));
  const float shoreline = *oleron.domain()->shoreline_position;

  // Deliberately ASYMMETRIC range: the anchor is nowhere near the midpoint, so a
  // LUT that ignored the anchor would fail this. -28.038 is the measured chart
  // datum at the Isles of Shoals, which is the case this feature exists for.
  const float lo = -60.0f, hi = 10.0f, anchor = -28.038f;
  const std::vector<marine_colormap::Rgba8> lut =
    camp::raster::bake_anchored_lut(oleron, lo, hi, anchor, 256);
  ASSERT_EQ(lut.size(), 256u);

  const marine_colormap::Rgba8 expected =
    marine_colormap::to_rgba8(oleron.sample(shoreline));
  const marine_colormap::Rgba8 got = lut[shader_index(anchor, lo, hi, 256)];

  // One quantization step of tolerance: the index is rounded to a discrete entry.
  EXPECT_NEAR(got.r, expected.r, 2) << "anchor did not land on the shoreline colour";
  EXPECT_NEAR(got.g, expected.g, 2);
  EXPECT_NEAR(got.b, expected.b, 2);
}

TEST(AnchoredLut, AnchorActuallyMovesWithTheRequestedValue)
{
  // Guards against a LUT that is anchored-shaped but ignores the value: two
  // different anchors over the same range must produce different tables.
  const marine_colormap::Palette& oleron = palette("oleron");
  const std::vector<marine_colormap::Rgba8> a =
    camp::raster::bake_anchored_lut(oleron, -60.0f, 10.0f, -40.0f, 256);
  const std::vector<marine_colormap::Rgba8> b =
    camp::raster::bake_anchored_lut(oleron, -60.0f, 10.0f, -10.0f, 256);
  bool differs = false;
  for(std::size_t i = 0; i < a.size() && !differs; ++i)
    differs = (a[i].r != b[i].r) || (a[i].g != b[i].g) || (a[i].b != b[i].b);
  EXPECT_TRUE(differs) << "anchor value had no effect on the baked table";
}

// --- The unanchored path must not change any existing colours ----------------

TEST(AnchoredLut, NoAnchorIsByteIdenticalToBakeLut)
{
  for(const char* name : {"grayscale", "viridis", "turbo", "oleron", "hypsometric"})
  {
    const marine_colormap::Palette& p = palette(name);
    const std::vector<marine_colormap::Rgba8> baseline =
      marine_colormap::bake_lut(p, marine_colormap::TransferParams{}, 256);
    const std::vector<marine_colormap::Rgba8> got =
      camp::raster::bake_anchored_lut(p, -50.0f, 20.0f, std::nullopt, 256);
    ASSERT_EQ(got.size(), baseline.size()) << name;
    for(std::size_t i = 0; i < baseline.size(); ++i)
    {
      ASSERT_EQ(got[i].r, baseline[i].r) << name << " entry " << i;
      ASSERT_EQ(got[i].g, baseline[i].g) << name << " entry " << i;
      ASSERT_EQ(got[i].b, baseline[i].b) << name << " entry " << i;
      ASSERT_EQ(got[i].a, baseline[i].a) << name << " entry " << i;
    }
  }
}

TEST(AnchoredLut, PaletteWithoutShorelineIgnoresTheAnchor)
{
  // The gate. grayscale/viridis declare no shoreline_position, so anchoring them
  // would warp the ramp to no purpose — it must be a no-op instead.
  for(const char* name : {"grayscale", "viridis"})
  {
    const marine_colormap::Palette& p = palette(name);
    EXPECT_FALSE(camp::raster::palette_supports_anchor(p)) << name;
    const std::vector<marine_colormap::Rgba8> baseline =
      marine_colormap::bake_lut(p, marine_colormap::TransferParams{}, 256);
    const std::vector<marine_colormap::Rgba8> got =
      camp::raster::bake_anchored_lut(p, -50.0f, 20.0f, -28.038f, 256);
    for(std::size_t i = 0; i < baseline.size(); ++i)
      ASSERT_EQ(got[i].r, baseline[i].r) << name << " entry " << i;
  }
}

TEST(AnchoredLut, NonFiniteAnchorFallsBackToUnanchored)
{
  const marine_colormap::Palette& oleron = palette("oleron");
  const std::vector<marine_colormap::Rgba8> baseline =
    marine_colormap::bake_lut(oleron, marine_colormap::TransferParams{}, 256);
  for(float bad : {std::numeric_limits<float>::quiet_NaN(),
                   std::numeric_limits<float>::infinity(),
                   -std::numeric_limits<float>::infinity()})
  {
    const std::vector<marine_colormap::Rgba8> got =
      camp::raster::bake_anchored_lut(oleron, -50.0f, 20.0f, bad, 256);
    ASSERT_EQ(got.size(), baseline.size());
    for(std::size_t i = 0; i < baseline.size(); ++i)
      ASSERT_EQ(got[i].r, baseline[i].r) << "entry " << i;
  }
}

// --- Degenerate inputs clamp; they never throw --------------------------------

TEST(AnchoredLut, AnchorOutsideRangeIsOrdinaryNotAnError)
{
  // A survey line with no land in view: the datum break sits ABOVE hi. Must
  // still produce a full, usable table.
  const marine_colormap::Palette& oleron = palette("oleron");
  for(float anchor : {-1000.0f, 1000.0f})
  {
    std::vector<marine_colormap::Rgba8> lut;
    ASSERT_NO_THROW(
      lut = camp::raster::bake_anchored_lut(oleron, -60.0f, -10.0f, anchor, 256));
    EXPECT_EQ(lut.size(), 256u);
  }
}

TEST(AnchoredLut, BoundaryAnchorAndZeroWidthRangeAreSafe)
{
  const marine_colormap::Palette& oleron = palette("oleron");
  std::vector<marine_colormap::Rgba8> lut;
  ASSERT_NO_THROW(lut = camp::raster::bake_anchored_lut(oleron, -60.0f, 10.0f, -60.0f, 256));
  EXPECT_EQ(lut.size(), 256u);
  ASSERT_NO_THROW(lut = camp::raster::bake_anchored_lut(oleron, -60.0f, 10.0f, 10.0f, 256));
  EXPECT_EQ(lut.size(), 256u);
  // Zero-width and inverted ranges: BreakpointMap's contract is clamp-never-throw.
  ASSERT_NO_THROW(lut = camp::raster::bake_anchored_lut(oleron, 5.0f, 5.0f, 5.0f, 256));
  EXPECT_EQ(lut.size(), 256u);
  ASSERT_NO_THROW(lut = camp::raster::bake_anchored_lut(oleron, 10.0f, -60.0f, -28.0f, 256));
  EXPECT_EQ(lut.size(), 256u);
}

TEST(AnchoredLut, DegenerateEntryCountMatchesBakeLut)
{
  const marine_colormap::Palette& oleron = palette("oleron");
  // n < 1 is coerced to 1, matching bake_lut's own guard.
  EXPECT_EQ(camp::raster::bake_anchored_lut(oleron, -60.0f, 10.0f, -28.0f, 0).size(), 1u);
  EXPECT_EQ(camp::raster::bake_anchored_lut(oleron, -60.0f, 10.0f, -28.0f, 1).size(), 1u);
}
