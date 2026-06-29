// Characterization tests for the marine_colormap adoption (camp#141).
//
// camp's internal camp::map::ColorMap is deleted; the colormap is now a
// marine_colormap palette selected by name and baked into the GPU LUT (ADR-0008)
// or sampled on the CPU (GridMap). viridis/turbo are now the CANONICAL 256-entry
// matplotlib/Google tables — an INTENDED colour change from camp's old sparse
// 7/8-stop approximations — so these tests SNAPSHOT the new canonical bake_lut
// output rather than asserting equivalence to the retired ramp. grayscale still
// matches exactly (a plain black->white ramp).
//
// The render-path properties (NaN + finite-NoData discard, Nearest filtering,
// sub-unit range stretch) live with the GPU shader and are covered by
// test_raster_gl_renderer.cpp; here we pin the palette colours, the exposed
// registry, and the name-resolution rules the layers rely on.
#include <gtest/gtest.h>

#include <QString>

#include <string>
#include <vector>

#include <marine_colormap/colormap.hpp>
#include <marine_colormap/palette.hpp>
#include <marine_colormap/transfer.hpp>

using marine_colormap::Rgba8;

namespace
{

// The camp colormap path bakes a 256-entry LUT with IDENTITY TransferParams (the
// per-band range stays in the shader; the LUT carries only the palette ramp — see
// RasterGlRenderer::ensureLut / ADR-0008). Reproduce that exact bake here.
std::vector<Rgba8> bake(const std::string& name)
{
  const marine_colormap::Palette* pal = marine_colormap::find_palette(name);
  EXPECT_NE(pal, nullptr) << "palette not found: " << name;
  return marine_colormap::bake_lut(*pal, marine_colormap::TransferParams{}, 256);
}

// Mirrors the case-insensitive, registry-validated settings read every migrated
// layer performs: lowercase the stored string, validate against the registry, and
// fall back to grayscale for an unknown name.
std::string canonicalName(const QString& stored)
{
  std::string name = stored.toLower().toStdString();
  if(!marine_colormap::palette_index(name))
    name = "grayscale";
  return name;
}

}  // namespace

// The colormap context menus are built from palette_names(): pin the full exposed
// registry so a palette add/rename/reorder is a deliberate, visible change.
TEST(Colormap, RegistryIsTheFullSix)
{
  const std::vector<std::string> expected = {
    "grayscale", "bronze", "thermal", "viridis", "turbo", "quality"};
  EXPECT_EQ(marine_colormap::palette_names(), expected);
}

// grayscale is a plain black->white ramp: it matches camp's old grayscale exactly.
TEST(Colormap, GrayscaleMatchesExactly)
{
  const std::vector<Rgba8> lut = bake("grayscale");
  EXPECT_EQ(lut.front().r, 0);   EXPECT_EQ(lut.front().g, 0);   EXPECT_EQ(lut.front().b, 0);
  EXPECT_EQ(lut.back().r, 255);  EXPECT_EQ(lut.back().g, 255);  EXPECT_EQ(lut.back().b, 255);
  // Mid LUT entry is a neutral grey on the diagonal.
  const Rgba8 mid = lut[128];
  EXPECT_EQ(mid.r, mid.g);
  EXPECT_EQ(mid.g, mid.b);
  EXPECT_NEAR(mid.r, 128, 1);
}

// viridis: snapshot the NEW canonical table (endpoints exact, midpoint ~1 LSB).
TEST(Colormap, ViridisIsCanonical)
{
  const std::vector<Rgba8> lut = bake("viridis");
  EXPECT_EQ(lut.front().r, 68);  EXPECT_EQ(lut.front().g, 1);   EXPECT_EQ(lut.front().b, 84);
  EXPECT_EQ(lut.back().r, 253);  EXPECT_EQ(lut.back().g, 231);  EXPECT_EQ(lut.back().b, 37);
  const Rgba8 mid = lut[128];
  EXPECT_NEAR(mid.r, 33, 1);
  EXPECT_NEAR(mid.g, 145, 1);
  EXPECT_NEAR(mid.b, 140, 1);
  // The canonical midpoint is a teal green (~145), NOT camp's old sparse-ramp green
  // (~168): this guards that the canonical table — not the retired approximation —
  // is in use. An intended change, documented in ADR-0008.
  EXPECT_LT(mid.g, 160);
}

// turbo: snapshot the NEW canonical table.
TEST(Colormap, TurboIsCanonical)
{
  const std::vector<Rgba8> lut = bake("turbo");
  EXPECT_EQ(lut.front().r, 48);  EXPECT_EQ(lut.front().g, 18);  EXPECT_EQ(lut.front().b, 59);
  EXPECT_EQ(lut.back().r, 122);  EXPECT_EQ(lut.back().g, 4);    EXPECT_EQ(lut.back().b, 3);
  const Rgba8 mid = lut[128];
  EXPECT_NEAR(mid.r, 164, 1);
  EXPECT_NEAR(mid.g, 252, 1);
  EXPECT_NEAR(mid.b, 60, 1);
}

// Every palette is fully opaque across its in-range entries (no accidental
// transparency in the LUT; transparency is a render-path NoData concern, not a
// palette one).
TEST(Colormap, AllPalettesOpaqueInRange)
{
  for(const std::string& name : marine_colormap::palette_names())
  {
    const std::vector<Rgba8> lut = bake(name);
    EXPECT_EQ(lut.front().a, 255) << name;
    EXPECT_EQ(lut[128].a, 255) << name;
    EXPECT_EQ(lut.back().a, 255) << name;
  }
}

// palette_names() round-trips through the name<->index<->palette registry, and an
// unknown name resolves to nullptr/nullopt (the layers fall back to grayscale).
TEST(Colormap, NameRoundTrip)
{
  for(const std::string& name : marine_colormap::palette_names())
  {
    const auto index = marine_colormap::palette_index(name);
    ASSERT_TRUE(index.has_value()) << name;
    EXPECT_EQ(marine_colormap::palette(*index).name(), name);
    EXPECT_NE(marine_colormap::find_palette(name), nullptr) << name;
  }
  EXPECT_FALSE(marine_colormap::palette_index("nope").has_value());
  EXPECT_EQ(marine_colormap::find_palette("nope"), nullptr);
}

// The persisted-name migration rule the layers apply: camp stores lowercase, but a
// legacy capitalized "Viridis"/"Turbo" still resolves; an unknown name -> grayscale.
TEST(Colormap, CaseInsensitiveSettingsFallback)
{
  EXPECT_EQ(canonicalName("viridis"), "viridis");
  EXPECT_EQ(canonicalName("Viridis"), "viridis");
  EXPECT_EQ(canonicalName("TURBO"), "turbo");
  EXPECT_EQ(canonicalName("grayscale"), "grayscale");
  EXPECT_EQ(canonicalName("nope"), "grayscale");
  EXPECT_EQ(canonicalName(""), "grayscale");
}
