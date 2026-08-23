// [camp#134] Unit tests for the shared raster::RasterGlRenderer (ADR-0007): the
// one GL raster path GggsTileLayer / SonarLiveCacheLayer / RasterLayer delegate to.
// Exercises the unified shader directly with hand-built textures:
//  - Scalar NaN NoData discard (the bug #134 fixes: `v != v` discards NaN cells),
//  - Scalar finite-sentinel discard (v == u_nodata),
//  - colormap LUT bake (a valid scalar shades to the ramp colour),
//  - Nearest sampling on the value texture (no blend across a NoData boundary),
//  - the Rgba bypass path (RGBA8 sampled directly, LUT bypassed; a==0 discards),
//  - [camp#181 / ADR-0015 D2] the LUT CACHE KEY: an anchored LUT re-bakes when the
//    render range moves (and holds the anchor's colour across the move), while an
//    unanchored one does NOT — plus the non-finite-anchor guard at the seam.
//
// Uses the renderer's own offscreen context (makeCurrent), a non-geographic item
// (1:1 unit-per-pixel quad) so a texture cell maps straight to an output pixel.
// Skips automatically where no offscreen GL context can be created (in-container).

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

#include <marine_colormap/palette.hpp>

#include <QApplication>
#include <QImage>
#include <QOffscreenSurface>
#include <QOpenGLContext>
#include <QOpenGLTexture>

#include "raster/raster_field_source.h"
#include "raster/raster_gl_renderer.h"

using camp::raster::RasterFieldItem;
using camp::raster::RasterGlRenderer;

namespace
{

bool offscreenGLAvailable()
{
  QOffscreenSurface surface;
  surface.create();
  QOpenGLContext ctx;
  return surface.isValid() && ctx.create();
}

// A non-geographic item over the unit-per-pixel quad [0,n]x[0,n], so an n x n
// render maps texture cell (col,row) straight to output pixel (col,row) — texture
// row 0 (v=0) is north = the top of the image.
RasterFieldItem scalarItem(QOpenGLTexture* tex, int n, bool has_nodata, float nodata)
{
  RasterFieldItem item;
  item.texture = tex;
  item.format = RasterFieldItem::Format::Scalar;
  item.geographic = false;
  item.west = 0.0; item.east = double(n);
  item.south = 0.0; item.north = double(n);
  item.has_nodata = has_nodata;
  item.nodata = nodata;
  return item;
}

std::unique_ptr<QOpenGLTexture> makeScalarTexture(int w, int h, const std::vector<float>& v)
{
  auto tex = std::make_unique<QOpenGLTexture>(QOpenGLTexture::Target2D);
  tex->setFormat(QOpenGLTexture::R32F);
  tex->setSize(w, h);
  tex->setMipLevels(1);
  tex->allocateStorage(QOpenGLTexture::Red, QOpenGLTexture::Float32);
  tex->setData(QOpenGLTexture::Red, QOpenGLTexture::Float32, v.data());
  tex->setWrapMode(QOpenGLTexture::ClampToEdge);
  return tex;
}

}  // namespace

// NaN + finite-sentinel discard, plus colormap shading on the valid cells. A 4x4
// R32F tile: cell (row0,col0) = NaN, (row0,col1) = 9999 (finite sentinel), every
// other cell = 10 (the range max). Grayscale ramp, range [0,10]: the valid cells
// shade to white; the two NoData cells discard (transparent).
TEST(RasterGlRendererTest, ScalarNanAndSentinelDiscard)
{
  if(!offscreenGLAvailable())
    GTEST_SKIP() << "no offscreen GL context available";

  RasterGlRenderer renderer;
  ASSERT_TRUE(renderer.makeCurrent());

  const int n = 4;
  std::vector<float> data(n * n, 10.0f);
  data[0 * n + 0] = std::numeric_limits<float>::quiet_NaN();   // north-west cell
  data[0 * n + 1] = 9999.0f;                                   // finite sentinel
  auto tex = makeScalarTexture(n, n, data);

  const RasterFieldItem item = scalarItem(tex.get(), n, /*has_nodata=*/true, 9999.0f);
  const QImage img = renderer.renderToImage({item}, QRectF(0, 0, n, n), 0.0f, 10.0f,
                                            QSize(n, n));
  tex.reset();
  renderer.doneCurrent();

  ASSERT_FALSE(img.isNull());
  EXPECT_EQ(img.width(), n);
  EXPECT_EQ(img.height(), n);

  // Row 0 (north) col 0 = NaN, col 1 = sentinel -> transparent; col 2,3 valid.
  EXPECT_EQ(img.pixelColor(0, 0).alpha(), 0) << "NaN cell must discard";
  EXPECT_EQ(img.pixelColor(1, 0).alpha(), 0) << "finite sentinel must discard";
  EXPECT_GT(img.pixelColor(2, 0).alpha(), 0) << "valid cell must render";

  // Valid cells shade to the top of the Grayscale ramp (white).
  const QColor valid = img.pixelColor(2, 2);
  EXPECT_GT(valid.alpha(), 0);
  EXPECT_GT(valid.red(), 200);
  EXPECT_GT(valid.green(), 200);
  EXPECT_GT(valid.blue(), 200);

  // Nearest sampling: the discard does not bleed onto the neighbouring valid cell.
  EXPECT_GT(img.pixelColor(1, 1).alpha(), 0);
}

// [camp#134 must-fix] Sub-unit value range: the colormap must stretch across a data
// range narrower than 1.0. A 2x1 R32F tile: col0 = data_min (0.0), col1 = data_max
// (0.3), Grayscale ramp, range [0.0, 0.3]. With the TRUE-span denominator the max
// sample reaches t = 1.0 (white, ~255). Under the old max(u_max - u_min, 1.0)
// denominator floor it only reached t = 0.3 -> grey ~76, collapsing the contrast —
// so the EXPECT_GT(hi.red(), 200) below FAILS under the floor and PASSES with the fix.
TEST(RasterGlRendererTest, ScalarSubUnitRangeSpansColormap)
{
  if(!offscreenGLAvailable())
    GTEST_SKIP() << "no offscreen GL context available";

  RasterGlRenderer renderer;
  ASSERT_TRUE(renderer.makeCurrent());
  renderer.setColormap("grayscale");

  const int w = 2, h = 1;
  std::vector<float> data = {0.0f, 0.3f};   // col0 = data_min, col1 = data_max
  auto tex = makeScalarTexture(w, h, data);

  RasterFieldItem item;
  item.texture = tex.get();
  item.format = RasterFieldItem::Format::Scalar;
  item.geographic = false;
  item.west = 0.0; item.east = double(w);
  item.south = 0.0; item.north = double(h);
  item.has_nodata = false;
  item.nodata = 0.0f;

  const QImage img = renderer.renderToImage({item}, QRectF(0, 0, w, h), 0.0f, 0.3f,
                                            QSize(w, h));
  tex.reset();
  renderer.doneCurrent();

  ASSERT_FALSE(img.isNull());
  const QColor lo = img.pixelColor(0, 0);   // data_min -> bottom of ramp (black)
  const QColor hi = img.pixelColor(1, 0);   // data_max -> top of ramp (white)

  EXPECT_GT(hi.red(), 200) << "sub-unit max must span to the top of the colormap "
                              "(fails under the max(span, 1.0) floor)";
  EXPECT_LT(lo.red(), 40) << "data_min stays at the bottom of the colormap";
  // Distinct LUT outputs at min vs max: the ramp spans the range.
  EXPECT_GT(hi.red() - lo.red(), 150) << "sub-unit range must preserve contrast";
}

// The Rgba bypass path: an RGBA8 texture is sampled directly (LUT bypassed). A
// uniform red texture renders red everywhere (NOT a colormap of red.r), proving the
// LUT is skipped; a fully-transparent texture discards everywhere.
TEST(RasterGlRendererTest, RgbaBypassesLut)
{
  if(!offscreenGLAvailable())
    GTEST_SKIP() << "no offscreen GL context available";

  RasterGlRenderer renderer;
  ASSERT_TRUE(renderer.makeCurrent());

  const int n = 4;
  // Uniform opaque red (RGBA bytes).
  std::vector<uchar> red(n * n * 4);
  for(int i = 0; i < n * n; ++i)
  {
    red[i * 4 + 0] = 255; red[i * 4 + 1] = 0;
    red[i * 4 + 2] = 0;   red[i * 4 + 3] = 255;
  }
  auto tex = std::make_unique<QOpenGLTexture>(QOpenGLTexture::Target2D);
  tex->setFormat(QOpenGLTexture::RGBA8_UNorm);
  tex->setSize(n, n);
  tex->setMipLevels(1);
  tex->allocateStorage(QOpenGLTexture::RGBA, QOpenGLTexture::UInt8);
  tex->setData(QOpenGLTexture::RGBA, QOpenGLTexture::UInt8, red.data());
  tex->setWrapMode(QOpenGLTexture::ClampToEdge);

  RasterFieldItem item = scalarItem(tex.get(), n, false, 0.0f);
  item.format = RasterFieldItem::Format::Rgba;
  const QImage img = renderer.renderToImage({item}, QRectF(0, 0, n, n), 0.0f, 10.0f,
                                            QSize(n, n));

  ASSERT_FALSE(img.isNull());
  const QColor c = img.pixelColor(2, 2);
  EXPECT_GT(c.alpha(), 0);
  EXPECT_GT(c.red(), 200);
  EXPECT_LT(c.green(), 60);
  EXPECT_LT(c.blue(), 60);

  // Fully-transparent texture -> every cell discards.
  std::vector<uchar> clear(n * n * 4, 0);
  tex->setData(QOpenGLTexture::RGBA, QOpenGLTexture::UInt8, clear.data());
  RasterFieldItem clear_item = item;
  const QImage clear_img = renderer.renderToImage({clear_item}, QRectF(0, 0, n, n),
                                                  0.0f, 10.0f, QSize(n, n));
  tex.reset();
  renderer.doneCurrent();

  ASSERT_FALSE(clear_img.isNull());
  int opaque = 0;
  for(int y = 0; y < clear_img.height(); ++y)
    for(int x = 0; x < clear_img.width(); ++x)
      if(clear_img.pixelColor(x, y).alpha() > 0)
        ++opaque;
  EXPECT_EQ(opaque, 0) << "a fully-transparent RGBA texture must discard everywhere";
}

// A colormap change re-bakes the LUT: the same valid scalar shades differently
// under Grayscale vs Viridis (the max maps to white vs Viridis' yellow).
TEST(RasterGlRendererTest, ColormapRebakesLut)
{
  if(!offscreenGLAvailable())
    GTEST_SKIP() << "no offscreen GL context available";

  RasterGlRenderer renderer;
  ASSERT_TRUE(renderer.makeCurrent());

  const int n = 2;
  std::vector<float> data(n * n, 10.0f);
  auto tex = makeScalarTexture(n, n, data);
  const RasterFieldItem item = scalarItem(tex.get(), n, false, 0.0f);

  renderer.setColormap("grayscale");
  const QColor gray = renderer.renderToImage({item}, QRectF(0, 0, n, n), 0.0f, 10.0f,
                                             QSize(n, n)).pixelColor(0, 0);
  renderer.setColormap("viridis");
  const QColor viridis = renderer.renderToImage({item}, QRectF(0, 0, n, n), 0.0f, 10.0f,
                                                QSize(n, n)).pixelColor(0, 0);
  tex.reset();
  renderer.doneCurrent();

  // Grayscale max -> white-ish (high, near-equal channels); Viridis max -> yellow
  // (high red+green, low blue). They must differ.
  EXPECT_NE(gray.rgb(), viridis.rgb());
  EXPECT_LT(viridis.blue(), viridis.green());
}

// [camp#181 / ADR-0015 D2] The LUT cache key. ADR-0008 Decision #2 asserted the LUT
// was range-INDEPENDENT; anchoring amends that, because the anchored bake warps the
// ramp through a BreakpointMap over [lo, hi]. Getting the key wrong is silent in
// both directions — a stale LUT renders wrong colours with no crash and no log
// line, and a never-matching key re-bakes and re-uploads a texture every frame —
// so these count bakes rather than eyeballing pixels.
TEST(RasterGlRendererTest, AnchoredLutRebakesOnRangeChange)
{
  if(!offscreenGLAvailable())
    GTEST_SKIP() << "no offscreen GL context available";

  RasterGlRenderer renderer;
  ASSERT_TRUE(renderer.makeCurrent());

  // oleron declares a shoreline_position, so the anchor actually bites.
  ASSERT_NE(marine_colormap::find_palette("oleron"), nullptr);
  const int n = 2;
  const float kAnchor = -28.0f;                 // an ellipsoidal height, not 0.0
  std::vector<float> data(n * n, kAnchor);      // every cell sits ON the shoreline
  auto tex = makeScalarTexture(n, n, data);
  const RasterFieldItem item = scalarItem(tex.get(), n, false, 0.0f);

  renderer.setColormap("oleron");
  renderer.setShorelineAnchor(kAnchor);

  const QColor wide = renderer.renderToImage({item}, QRectF(0, 0, n, n), -60.0f, 20.0f,
                                             QSize(n, n)).pixelColor(0, 0);
  const std::size_t after_first = renderer.lutBakeCount();
  EXPECT_GT(after_first, 0u);

  // Same anchor, same palette, DIFFERENT range: the anchored LUT is a function of
  // [lo, hi], so it must re-bake.
  const QColor narrow = renderer.renderToImage({item}, QRectF(0, 0, n, n), -40.0f, 5.0f,
                                               QSize(n, n)).pixelColor(0, 0);
  EXPECT_GT(renderer.lutBakeCount(), after_first)
      << "an anchored LUT must re-bake when the render range moves";

  // And the whole point of the re-bake: the anchor value keeps its shoreline colour
  // no matter what the range does. A stale LUT would slide it.
  EXPECT_LT(std::abs(wide.red()   - narrow.red()),   12);
  EXPECT_LT(std::abs(wide.green() - narrow.green()), 12);
  EXPECT_LT(std::abs(wide.blue()  - narrow.blue()),  12);

  // Re-rendering with the SAME (palette, anchor, range) is a cache hit.
  const std::size_t before_repeat = renderer.lutBakeCount();
  renderer.renderToImage({item}, QRectF(0, 0, n, n), -40.0f, 5.0f, QSize(n, n));
  EXPECT_EQ(renderer.lutBakeCount(), before_repeat)
      << "an unchanged (palette, anchor, range) must hit the LUT cache";

  tex.reset();
  renderer.doneCurrent();
}

TEST(RasterGlRendererTest, UnanchoredLutDoesNotRebakeOnRangeChange)
{
  if(!offscreenGLAvailable())
    GTEST_SKIP() << "no offscreen GL context available";

  RasterGlRenderer renderer;
  ASSERT_TRUE(renderer.makeCurrent());

  const int n = 2;
  std::vector<float> data(n * n, -28.0f);
  auto tex = makeScalarTexture(n, n, data);
  const RasterFieldItem item = scalarItem(tex.get(), n, false, 0.0f);

  // No anchor: the bake reduces to the plain range-independent palette ramp, so
  // [lo, hi] must stay OUT of the cache key. Auto range ticks on every fold, so a
  // range-keyed cache here would re-bake and re-upload a texture continuously.
  renderer.setColormap("oleron");
  ASSERT_FALSE(renderer.shorelineAnchor().has_value());

  renderer.renderToImage({item}, QRectF(0, 0, n, n), -60.0f, 20.0f, QSize(n, n));
  const std::size_t after_first = renderer.lutBakeCount();
  EXPECT_GT(after_first, 0u);

  renderer.renderToImage({item}, QRectF(0, 0, n, n), -40.0f, 5.0f, QSize(n, n));
  renderer.renderToImage({item}, QRectF(0, 0, n, n), -1.0f, 1.0f, QSize(n, n));
  EXPECT_EQ(renderer.lutBakeCount(), after_first)
      << "an UNANCHORED layer must not re-bake its LUT on a range change";

  // Setting an anchor re-bakes; clearing it re-bakes back.
  renderer.setShorelineAnchor(-28.0f);
  renderer.renderToImage({item}, QRectF(0, 0, n, n), -1.0f, 1.0f, QSize(n, n));
  const std::size_t after_anchor = renderer.lutBakeCount();
  EXPECT_GT(after_anchor, after_first) << "an anchor appearing must re-bake";

  renderer.setShorelineAnchor(std::nullopt);
  renderer.renderToImage({item}, QRectF(0, 0, n, n), -1.0f, 1.0f, QSize(n, n));
  EXPECT_GT(renderer.lutBakeCount(), after_anchor) << "an anchor clearing must re-bake";

  tex.reset();
  renderer.doneCurrent();
}

// [camp#181 / ADR-0015] A non-finite anchor is not an anchor. Stored, it would make
// the cache key never match again (NaN != NaN) — a bake plus a texture upload every
// frame, with no visible symptom because the bake already falls back to the
// unanchored ramp.
TEST(RasterGlRendererTest, NonFiniteAnchorIsDroppedAtTheSeam)
{
  RasterGlRenderer renderer;   // no GL needed: this is the setter's own contract
  renderer.setShorelineAnchor(std::numeric_limits<float>::quiet_NaN());
  EXPECT_FALSE(renderer.shorelineAnchor().has_value());
  renderer.setShorelineAnchor(std::numeric_limits<float>::infinity());
  EXPECT_FALSE(renderer.shorelineAnchor().has_value());
  renderer.setShorelineAnchor(-28.0f);
  ASSERT_TRUE(renderer.shorelineAnchor().has_value());
  EXPECT_FLOAT_EQ(*renderer.shorelineAnchor(), -28.0f);
}

int main(int argc, char** argv)
{
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
