// [camp#90 / I4] Headless offscreen render test for the GGGS tile layer. The
// layer warps tiles to Web-Mercator in its own offscreen GL context and returns
// a QImage (the portable path that does not depend on the QGraphicsView viewport
// being GL). This renders a synthetic tile with a bright "L" along its NORTH and
// WEST edges so the output orientation is unambiguous, asserts the render is
// non-empty, and writes /tmp/gggs_render.png for visual inspection.
//
// Skipped automatically if no offscreen GL context can be created (e.g. a CI
// box with no GL at all).

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdlib>
#include <limits>
#include <vector>

#include <gdal_priv.h>

#include <QApplication>
#include <QImage>
#include <QOffscreenSurface>
#include <QOpenGLContext>
#include <QTemporaryDir>

#include "map/map.h"
#include "map/layer_list.h"
#include "raster/gggs_tile_layer.h"

namespace
{

QString writeTile(const QTemporaryDir& dir, int w, int h, const double geo[6],
                  const std::vector<uint16_t>& samples,
                  const QString& name = "13_0_0.tif")
{
  if(GDALGetDriverCount() == 0)
    GDALAllRegister();
  const QString path = dir.filePath(name);
  GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("GTiff");
  // [camp#122] Guard the GDAL handles so a driver/create failure fails the test
  // cleanly instead of dereferencing null. (These helpers return QString, so a
  // void-returning ASSERT_* can't be used here — EXPECT_NE + early return.)
  EXPECT_NE(driver, nullptr);
  if(!driver)
    return QString();
  GDALDataset* ds = driver->Create(path.toUtf8().constData(), w, h, 1, GDT_UInt16, nullptr);
  EXPECT_NE(ds, nullptr);
  if(!ds)
    return QString();
  ds->SetGeoTransform(const_cast<double*>(geo));
  GDALRasterBand* band = ds->GetRasterBand(1);
  band->SetNoDataValue(0);
  const CPLErr err = band->RasterIO(GF_Write, 0, 0, w, h,
                                    const_cast<uint16_t*>(samples.data()),
                                    w, h, GDT_UInt16, 0, 0);
  GDALClose(ds);
  return err == CE_None ? path : QString();
}

// [camp#122] Float32 single-band tile writer with an arbitrary NoData sentinel —
// the variant needed to exercise valid 0/negative samples and a non-zero NoData
// that the UInt16 floor-to-1 writer above can't express.
QString writeFloatTile(const QTemporaryDir& dir, int w, int h, const double geo[6],
                       double nodata, const std::vector<float>& samples)
{
  if(GDALGetDriverCount() == 0)
    GDALAllRegister();
  const QString path = dir.filePath("13_0_0.tif");
  GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("GTiff");
  // [camp#122] Guard the GDAL handles so a driver/create failure fails the test
  // cleanly instead of dereferencing null. (These helpers return QString, so a
  // void-returning ASSERT_* can't be used here — EXPECT_NE + early return.)
  EXPECT_NE(driver, nullptr);
  if(!driver)
    return QString();
  GDALDataset* ds = driver->Create(path.toUtf8().constData(), w, h, 1, GDT_Float32, nullptr);
  EXPECT_NE(ds, nullptr);
  if(!ds)
    return QString();
  ds->SetGeoTransform(const_cast<double*>(geo));
  GDALRasterBand* band = ds->GetRasterBand(1);
  band->SetNoDataValue(nodata);
  const CPLErr err = band->RasterIO(GF_Write, 0, 0, w, h,
                                    const_cast<float*>(samples.data()),
                                    w, h, GDT_Float32, 0, 0);
  GDALClose(ds);
  return err == CE_None ? path : QString();
}

bool offscreenGLAvailable()
{
  QOffscreenSurface surface;
  surface.create();
  QOpenGLContext ctx;
  return surface.isValid() && ctx.create();
}

}  // namespace

TEST(GggsRenderTest, OffscreenWarpProducesOrientedImage)
{
  if(!offscreenGLAvailable())
    GTEST_SKIP() << "no offscreen GL context available";

  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());

  // North-up tile over a small patch; row 0 = north edge.
  const int w = 100, h = 100;
  const double geo[6] = {-71.40, 0.0001, 0.0, 43.00, 0.0, -0.0001};
  std::vector<uint16_t> samples(w * h, 8000);
  for(int r = 0; r < h; ++r)
    for(int c = 0; c < w; ++c)
      if(r < 10 || c < 10)                 // north band (low row) + west band (low col)
        samples[r * w + c] = 60000;
  const QString path = writeTile(dir, w, h, geo, samples);
  ASSERT_FALSE(path.isEmpty());

  camp::map::Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);
  auto* layer = new camp::raster::GggsTileLayer(layers, dir.path());
  ASSERT_TRUE(layer->valid());
  // [camp#102] Pixel loads are async + lazily kicked from paint(); this headless
  // test calls renderImage() directly, so drive + await the load explicitly.
  layer->waitForLoad();

  const QImage img = layer->renderImage(QSize(200, 200));
  ASSERT_FALSE(img.isNull());
  EXPECT_EQ(img.width(), 200);
  EXPECT_EQ(img.height(), 200);

  // Some pixels are opaque (the warped tile), some transparent (outside it).
  int opaque = 0, bright = 0;
  for(int y = 0; y < img.height(); ++y)
    for(int x = 0; x < img.width(); ++x)
    {
      const QColor px = img.pixelColor(x, y);
      if(px.alpha() > 0)
        ++opaque;
      if(px.red() > 200)
        ++bright;
    }
  EXPECT_GT(opaque, 0);
  EXPECT_GT(bright, 0);

  img.save("/tmp/gggs_render.png");
}

// [camp#122] The shader discards by the band's actual NoData sentinel (via the
// per-tile u_has_nodata/u_nodata uniforms), not the old hardcoded `v <= 0.0`. A
// Float32 tile whose valid samples are mostly 0.0 (plus two positive stripes), with
// a NoData (9999) block punched in the interior, must render: the 0.0 background
// OPAQUE (previously discarded by `v <= 0`) and the NoData block TRANSPARENT.
//
// DISCRIMINATION (the must-fix): asserting only "some opaque + an enclosed
// transparent hole" passes under BOTH the fix and the reverted `v <= 0.0` bug —
// under the bug the positive stripes stay opaque and the discarded 0.0 background
// supplies the transparent pixels, so a column still shows opaque/transparent/
// opaque. The discriminating assertion is the DENSE column: under the fix the
// valid 0.0 background fills a column edge-to-edge (away from the hole), so some
// column is ~fully opaque over its footprint span; under the bug the 0.0
// background is discarded and EVERY column is sparse (only the two thin stripes,
// max fill ~0.44), so the dense-column assertion FAILS. Verified by reverting the
// shader discard to `if(v <= 0.0)`: the EXPECT_GT(max_fill, 0.8) then fails while
// it passes with the per-NoData discard. (Runs only where offscreen GL exists;
// SKIPs in-container by design.)
TEST(GggsRenderTest, NoDataDiscardHonorsUniform)
{
  if(!offscreenGLAvailable())
    GTEST_SKIP() << "no offscreen GL context available";

  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());

  const int w = 100, h = 100;
  const double geo[6] = {-71.40, 0.0001, 0.0, 43.00, 0.0, -0.0001};
  // Background is the valid zero sample everywhere; this is the regression case
  // (the old `v <= 0` discard would have dropped the entire tile).
  std::vector<float> samples(w * h, 0.0f);
  // Two distinct positive values seed a real colormap range.
  for(int c = 0; c < w; ++c)
  {
    samples[10 * w + c] = 5.0f;    // a positive stripe near the north
    samples[20 * w + c] = 10.0f;   // a second, distinct positive stripe
  }
  // A NoData (9999) block punched into the INTERIOR -> discarded -> transparent
  // hole enclosed by the opaque valid background.
  for(int r = 40; r < 60; ++r)
    for(int c = 40; c < 60; ++c)
      samples[r * w + c] = 9999.0f;
  const QString path = writeFloatTile(dir, w, h, geo, 9999.0, samples);
  ASSERT_FALSE(path.isEmpty());

  camp::map::Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);
  auto* layer = new camp::raster::GggsTileLayer(layers, dir.path());
  ASSERT_TRUE(layer->valid());
  layer->waitForLoad();

  const QImage img = layer->renderImage(QSize(200, 200));
  ASSERT_FALSE(img.isNull());

  // Per-column alpha scan collecting three signals:
  //  - `opaque`: any opaque pixel exists at all,
  //  - `max_fill`: the highest opaque fraction of any column's footprint span
  //    (first..last opaque) — DENSE only if the valid 0.0 background renders,
  //  - `enclosed_transparent`: a transparent pixel with opaque pixels both above
  //    and below it in the same column — the interior NoData hole.
  int opaque = 0, enclosed_transparent = 0;
  double max_fill = 0.0;
  for(int x = 0; x < img.width(); ++x)
  {
    int first_opaque = -1, last_opaque = -1, col_opaque = 0;
    for(int y = 0; y < img.height(); ++y)
    {
      if(img.pixelColor(x, y).alpha() > 0)
      {
        ++opaque;
        ++col_opaque;
        if(first_opaque < 0)
          first_opaque = y;
        last_opaque = y;
      }
    }
    if(first_opaque >= 0)
    {
      const int span = last_opaque - first_opaque + 1;
      max_fill = std::max(max_fill, double(col_opaque) / span);
      for(int y = first_opaque + 1; y < last_opaque; ++y)
        if(img.pixelColor(x, y).alpha() == 0)
          ++enclosed_transparent;
    }
  }
  EXPECT_GT(opaque, 0);                  // valid 0.0 samples render opaque
  // Discriminator: a column of valid 0.0 background is densely opaque. Under the
  // reverted `v <= 0.0` bug the background is discarded and no column exceeds
  // ~0.44 fill (stripes + block only), so this fails; with the fix it is ~1.0.
  EXPECT_GT(max_fill, 0.8);
  EXPECT_GT(enclosed_transparent, 0);    // NoData (9999) block discarded -> hole

  img.save("/tmp/gggs_nodata_render.png");
}

// [camp#134] NaN NoData discard. Chart + backscatter stores use NaN as their
// NoData sentinel; the OLD shader tested only `v == u_nodata`, which is FALSE for
// NaN (NaN compares unequal to everything), so those cells rendered OPAQUE. The
// unified renderer's `if(v != v) discard;` fixes it. This writes a Float32 tile
// whose NoData sentinel is NaN: a valid 0.0 background (two positive stripes seed a
// real range) with a NaN block punched into the interior. After the fix the NaN
// block is a transparent hole enclosed by the opaque valid background.
//
// DISCRIMINATION mirrors NoDataDiscardHonorsUniform: the dense-column assertion
// (max_fill > 0.8) holds only because the valid 0.0 background renders; if NaN
// cells rendered opaque (the bug) the "hole" would fill in and enclosed_transparent
// would be 0. (Runs only where offscreen GL exists; SKIPs in-container by design.)
TEST(GggsRenderTest, NanNoDataDiscardsTransparent)
{
  if(!offscreenGLAvailable())
    GTEST_SKIP() << "no offscreen GL context available";

  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());

  const int w = 100, h = 100;
  const double geo[6] = {-71.40, 0.0001, 0.0, 43.00, 0.0, -0.0001};
  std::vector<float> samples(w * h, 0.0f);             // valid zero background
  for(int c = 0; c < w; ++c)
  {
    samples[10 * w + c] = 5.0f;                        // positive stripes -> real range
    samples[20 * w + c] = 10.0f;
  }
  const float nan = std::numeric_limits<float>::quiet_NaN();
  for(int r = 40; r < 60; ++r)
    for(int c = 40; c < 60; ++c)
      samples[r * w + c] = nan;                        // interior NaN block
  const QString path = writeFloatTile(dir, w, h, geo, nan, samples);   // NoData = NaN
  ASSERT_FALSE(path.isEmpty());

  camp::map::Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);
  auto* layer = new camp::raster::GggsTileLayer(layers, dir.path());
  ASSERT_TRUE(layer->valid());
  layer->waitForLoad();

  const QImage img = layer->renderImage(QSize(200, 200));
  ASSERT_FALSE(img.isNull());

  int opaque = 0, enclosed_transparent = 0;
  double max_fill = 0.0;
  for(int x = 0; x < img.width(); ++x)
  {
    int first_opaque = -1, last_opaque = -1, col_opaque = 0;
    for(int y = 0; y < img.height(); ++y)
    {
      if(img.pixelColor(x, y).alpha() > 0)
      {
        ++opaque;
        ++col_opaque;
        if(first_opaque < 0)
          first_opaque = y;
        last_opaque = y;
      }
    }
    if(first_opaque >= 0)
    {
      const int span = last_opaque - first_opaque + 1;
      max_fill = std::max(max_fill, double(col_opaque) / span);
      for(int y = first_opaque + 1; y < last_opaque; ++y)
        if(img.pixelColor(x, y).alpha() == 0)
          ++enclosed_transparent;
    }
  }
  EXPECT_GT(opaque, 0);                  // valid 0.0 background renders opaque
  EXPECT_GT(max_fill, 0.8);              // a column of valid background is dense
  EXPECT_GT(enclosed_transparent, 0);    // NaN block discarded -> enclosed hole

  img.save("/tmp/gggs_nan_render.png");
}

// Optional real-data smoke render: set GGGS_TEST_STORE to a tile directory to
// warp it headlessly to /tmp/gggs_real.png (QA / preview; skipped otherwise).
TEST(GggsRenderTest, RealStoreRendersWhenProvided)
{
  const QByteArray store = qgetenv("GGGS_TEST_STORE");
  if(store.isEmpty())
    GTEST_SKIP() << "set GGGS_TEST_STORE to a tile directory to run";
  if(!offscreenGLAvailable())
    GTEST_SKIP() << "no offscreen GL context available";

  camp::map::Map map;
  auto* layer = new camp::raster::GggsTileLayer(map.topLevelLayers(),
                                                QString::fromLocal8Bit(store));
  ASSERT_TRUE(layer->valid());
  layer->waitForLoad();
  const QImage img = layer->renderImage(QSize(900, 900));
  ASSERT_FALSE(img.isNull());
  img.save("/tmp/gggs_real.png");
}

// [camp#103] The clip-aware renderImage(size, clip) must (a) render ONLY the
// clipped region — a clip covering tile A yields an image of A's pixels alone —
// and (b) keep tiles that PARTIALLY intersect the clip (a straddling clip shows
// both tiles). Two horizontally-adjacent uniform tiles with distinct values are
// distinguishable through the auto-ranged grayscale LUT (A=max -> bright,
// B=min -> dark, both opaque; outside coverage transparent).
TEST(GggsRenderTest, ClipFilterRendersOnlyIntersectingTiles)
{
  if(!offscreenGLAvailable())
    GTEST_SKIP() << "no offscreen GL context available";

  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());

  const int w = 100, h = 100;
  // Tile A spans lon [-71.40, -71.39]; tile B spans [-71.39, -71.38]; both span
  // lat [42.99, 43.00] (north-up, row 0 = north edge).
  const double geo_a[6] = {-71.40, 0.0001, 0.0, 43.00, 0.0, -0.0001};
  const double geo_b[6] = {-71.39, 0.0001, 0.0, 43.00, 0.0, -0.0001};
  const std::vector<uint16_t> samples_a(w * h, 60000);   // -> LUT max: bright
  const std::vector<uint16_t> samples_b(w * h, 20000);   // -> LUT min: dark
  ASSERT_FALSE(writeTile(dir, w, h, geo_a, samples_a, "13_0_0.tif").isEmpty());
  ASSERT_FALSE(writeTile(dir, w, h, geo_b, samples_b, "13_0_1.tif").isEmpty());

  camp::map::Map map;
  auto* layer = new camp::raster::GggsTileLayer(map.topLevelLayers(), dir.path());
  ASSERT_TRUE(layer->valid());
  layer->waitForLoad();

  const QRectF sb = layer->sceneBounds();
  auto countPixels = [](const QImage& img, int& bright, int& dark)
  {
    bright = dark = 0;
    for(int y = 0; y < img.height(); ++y)
      for(int x = 0; x < img.width(); ++x)
      {
        const QColor px = img.pixelColor(x, y);
        if(px.alpha() == 0)
          continue;
        if(px.red() > 200)
          ++bright;
        else if(px.red() < 60)
          ++dark;
      }
  };

  // Full extent: both tiles contribute.
  int bright = 0, dark = 0;
  countPixels(layer->renderImage(QSize(200, 100)), bright, dark);
  EXPECT_GT(bright, 0);
  EXPECT_GT(dark, 0);

  // Clip strictly inside tile A's (western) half: only A's pixels appear.
  const QRectF clip_a(sb.left(), sb.top(), sb.width() * 0.45, sb.height());
  const QImage img_a = layer->renderImage(QSize(100, 100), clip_a);
  ASSERT_FALSE(img_a.isNull());
  countPixels(img_a, bright, dark);
  EXPECT_GT(bright, 0);
  EXPECT_EQ(dark, 0);
  img_a.save("/tmp/gggs_clip_a.png");

  // Clip straddling the A|B boundary: BOTH partially-intersecting tiles are
  // kept (the intersect predicate must not drop a partially-visible tile).
  const QRectF clip_mid(sb.left() + sb.width() * 0.25, sb.top(),
                        sb.width() * 0.5, sb.height());
  countPixels(layer->renderImage(QSize(100, 100), clip_mid), bright, dark);
  EXPECT_GT(bright, 0);
  EXPECT_GT(dark, 0);
}

// [camp#103] The clip render's pixel density scales with the CLIP, not the whole
// extent — the fix for the field-observed blur. The "L" tile's bright west band
// is 10% of the tile width; rendered at the same image size, a west-half clip
// makes that band span ~twice as many image columns as the full-extent render.
TEST(GggsRenderTest, ClipRenderScalesResolutionToClip)
{
  if(!offscreenGLAvailable())
    GTEST_SKIP() << "no offscreen GL context available";

  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());

  const int w = 100, h = 100;
  const double geo[6] = {-71.40, 0.0001, 0.0, 43.00, 0.0, -0.0001};
  std::vector<uint16_t> samples(w * h, 8000);
  for(int r = 0; r < h; ++r)
    for(int c = 0; c < w; ++c)
      if(c < 10)                             // bright west band only
        samples[r * w + c] = 60000;
  ASSERT_FALSE(writeTile(dir, w, h, geo, samples).isEmpty());

  camp::map::Map map;
  auto* layer = new camp::raster::GggsTileLayer(map.topLevelLayers(), dir.path());
  ASSERT_TRUE(layer->valid());
  layer->waitForLoad();

  auto brightColumns = [](const QImage& img)
  {
    int cols = 0;
    for(int x = 0; x < img.width(); ++x)
      for(int y = 0; y < img.height(); ++y)
        if(img.pixelColor(x, y).alpha() > 0 && img.pixelColor(x, y).red() > 200)
        {
          ++cols;
          break;
        }
    return cols;
  };

  const QSize size(200, 200);
  const int full_cols = brightColumns(layer->renderImage(size));
  const QRectF sb = layer->sceneBounds();
  const QRectF west_half(sb.left(), sb.top(), sb.width() * 0.5, sb.height());
  const QImage img_clip = layer->renderImage(size, west_half);
  ASSERT_FALSE(img_clip.isNull());
  const int clip_cols = brightColumns(img_clip);

  EXPECT_GT(full_cols, 0);
  // Same band, half the scene width, same image width -> ~2x the columns.
  EXPECT_GT(clip_cols, full_cols * 3 / 2);
  img_clip.save("/tmp/gggs_clip_res.png");
}

int main(int argc, char** argv)
{
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
