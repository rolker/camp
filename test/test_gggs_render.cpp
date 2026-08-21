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
#include <QDir>
#include <QGeoCoordinate>
#include <QGraphicsView>
#include <QImage>
#include <QOffscreenSurface>
#include <QOpenGLContext>
#include <QTemporaryDir>
#include <QTransform>

#include "map/map.h"
#include "map/layer_list.h"
#include "map_view/web_mercator.h"
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

// [camp#103] Paint-path regression: with a view ATTACHED, paint() must render at
// the view's resolution — the field bug was a whole-extent <=4096px render
// upscaled over the viewport (blurry). This drives the REAL paint() path through
// a QGraphicsView (grab()), not renderImage() directly, so the viewport
// derivation itself is under test: the derivation must come from the attached
// view (painter clip state is unreliable — empty on live full-viewport
// repaints). A 2000x2000 per-pixel checkerboard zoomed to 40 screen-px per data
// cell makes the whole extent 80000 screen px (>> 4096): the fixed path renders
// the viewport crisp (pixels cluster at the two LUT extremes), the broken path
// renders cells at ~2px in the 4096-clamped image and upscales ~20x with
// smoothing, smearing most pixels into mid-tones (verified fails-without-fix).
TEST(GggsRenderTest, ViewAttachedPaintRendersAtViewResolution)
{
  if(!offscreenGLAvailable())
    GTEST_SKIP() << "no offscreen GL context available";

  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());

  const int w = 2000, h = 2000;
  const double geo[6] = {-71.40, 0.00002, 0.0, 43.00, 0.0, -0.00002};
  std::vector<uint16_t> samples(w * h);
  for(int r = 0; r < h; ++r)
    for(int c = 0; c < w; ++c)
      samples[r * w + c] = ((r + c) % 2) ? 60000 : 20000;   // per-pixel checker
  ASSERT_FALSE(writeTile(dir, w, h, geo, samples).isEmpty());

  camp::map::Map map;
  auto* layer = new camp::raster::GggsTileLayer(map.topLevelLayers(), dir.path());
  ASSERT_TRUE(layer->valid());
  layer->waitForLoad();

  const QRectF sb = layer->sceneBounds();
  const double cell_m = sb.width() / w;
  const double scale = 40.0 / cell_m;   // 40 screen px per data cell

  QGraphicsView view(map.scene());
  view.resize(400, 300);
  // Match MapView's convention: scale(s, -s) composes with the layer's
  // fromScale(1,-1) to a net-upright draw.
  view.setTransform(QTransform::fromScale(scale, -scale));
  view.centerOn(QPointF(sb.center().x(), sb.center().y()));
  view.show();
  QApplication::processEvents();
  // AFTER scene-add/settings side effects: tile-set layers default to
  // hidden (camp#102 readSettings false-fallback), which would leave the
  // grab blank white.
  layer->setVisible(true);
  QApplication::processEvents();

  // Grab the viewport widget only (no frame/scrollbars).
  const QImage img = view.viewport()->grab().toImage();
  ASSERT_FALSE(img.isNull());
  img.save("/tmp/gggs_view_paint.png");

  // Sample the central region (well inside the tile). Crisp = the checker's two
  // LUT extremes each cover ~half the pixels; blurred = mid-tones dominate;
  // blank/white (layer didn't paint) = no dark pixels at all. Requiring BOTH
  // extremes discriminates all three.
  int sampled = 0, dark = 0, mid = 0;
  for(int y = img.height() / 4; y < 3 * img.height() / 4; ++y)
    for(int x = img.width() / 4; x < 3 * img.width() / 4; ++x)
    {
      const int red = img.pixelColor(x, y).red();
      ++sampled;
      if(red < 60)
        ++dark;
      else if(red <= 200)
        ++mid;
    }
  ASSERT_GT(sampled, 1000);
  // Presence: a blank grab (layer never painted) has no dark cells at all; a
  // crisp checker is ~half dark. Crispness: the broken whole-extent upscale
  // smears ~half the pixels into mid-tones (measured 0.5); the crisp render has
  // almost none. Together the two discriminate blank, blurred, and crisp.
  EXPECT_GT(double(dark) / sampled, 0.25)
    << "dark checker cells missing — layer blank or washed out (fraction "
    << double(dark) / sampled << ")";
  EXPECT_LT(double(mid) / sampled, 0.15)
    << "mid-tone fraction " << double(mid) / sampled
    << " — paint() rendered blurred; viewport derivation regressed";
}

// [camp#103 / ADR-0013] Overview-sidecar enumeration: a store with fine L13
// tiles plus a `overviews/` sidecar (uma ADR-0011) exposes both levels, and the
// layer extent tracks the FINE footprint (an overview tile is padded to its
// coarse GGGS grid cell — uniting it would balloon fit-to-extent).
TEST(GggsRenderTest, OverviewSidecarLoadsBothLevels)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  ASSERT_TRUE(QDir(dir.path()).mkdir("overviews"));

  const int w = 20, h = 20;
  const double fine_geo[6] = {-71.40, 0.0001, 0.0, 43.00, 0.0, -0.0001};
  const double coarse_geo[6] = {-72.00, 0.1, 0.0, 44.00, 0.0, -0.1};
  const std::vector<uint16_t> samples(w * h, 5000);
  ASSERT_FALSE(writeTile(dir, w, h, fine_geo, samples, "13_0_0.tif").isEmpty());
  ASSERT_FALSE(writeTile(dir, w, h, coarse_geo, samples,
                         "overviews/0_0_0.tif").isEmpty());

  camp::map::Map map;
  auto* layer = new camp::raster::GggsTileLayer(map.topLevelLayers(), dir.path());
  ASSERT_TRUE(layer->valid());
  EXPECT_EQ(layer->availableLevels(), (std::vector<int>{0, 13}));

  // Extent = the fine footprint, not the coarse padded cell: the fine tile spans
  // 0.002° of longitude; the overview spans 2°. A fine-based extent is ~1000×
  // narrower, so a generous factor-10 bound discriminates unambiguously.
  const double fine_width_m = web_mercator::geoToMap(QGeoCoordinate(43.0, -71.398)).x() -
                              web_mercator::geoToMap(QGeoCoordinate(43.0, -71.40)).x();
  EXPECT_LT(layer->sceneBounds().width(), fine_width_m * 10.0);
}

// [camp#103/#194] The demand-driven CEILING filter: with a forced LOD
// selection, every level <= the selection loads (multi-level compositing —
// the coarse level is a permanent part of the composited picture, so a
// region-disjoint native ladder renders all its regions). The exclusion half
// of the ceiling (levels ABOVE the selection stay out) is the mid-ladder case
// below, DemandDrivenCeilingExcludesFinerLevels — selecting the max, as here,
// cannot distinguish a ceiling from no filter at all.
TEST(GggsRenderTest, DemandDrivenLoadsLevelsUpToSelection)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  ASSERT_TRUE(QDir(dir.path()).mkdir("overviews"));
  const int w = 20, h = 20;
  const double fine_geo[6] = {-71.40, 0.0001, 0.0, 43.00, 0.0, -0.0001};
  const double coarse_geo[6] = {-72.00, 0.1, 0.0, 44.00, 0.0, -0.1};
  const std::vector<uint16_t> samples(w * h, 5000);
  ASSERT_FALSE(writeTile(dir, w, h, fine_geo, samples, "13_0_0.tif").isEmpty());
  ASSERT_FALSE(writeTile(dir, w, h, coarse_geo, samples,
                         "overviews/0_0_0.tif").isEmpty());

  camp::map::Map map;
  auto* layer = new camp::raster::GggsTileLayer(map.topLevelLayers(), dir.path());
  ASSERT_TRUE(layer->valid());
  layer->setLodForTest(13, QRectF());   // level ceiling only, no spatial filter
  layer->waitForLoad();
  EXPECT_EQ(layer->pixelsLoadedCount(13), 1);
  EXPECT_EQ(layer->pixelsLoadedCount(0), 1) <<
    "coarse level excluded despite ceiling semantics — compositing must load "
    "every level <= the selection";
}

// [camp#194] The exclusion half of the ceiling filter, selected MID-ladder:
// levels above the selection must NOT load. This is the case that
// distinguishes a correct ceiling from the filter being removed entirely (a
// regression to the pre-camp#103 eager whole-store load) — with the selection
// at the ladder's max, every level trivially satisfies level <= selection.
TEST(GggsRenderTest, DemandDrivenCeilingExcludesFinerLevels)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const int w = 20, h = 20;
  // Three-level NATIVE ladder (all in the main directory, as an ENC chart
  // store lays them out), at three disjoint regions.
  const double geo_l0[6] = {-71.40, 0.0001, 0.0, 43.00, 0.0, -0.0001};
  const double geo_l7[6] = {-71.40, 0.0001, 0.0, 43.10, 0.0, -0.0001};
  const double geo_l13[6] = {-71.40, 0.0001, 0.0, 43.20, 0.0, -0.0001};
  const std::vector<uint16_t> samples(w * h, 5000);
  ASSERT_FALSE(writeTile(dir, w, h, geo_l0, samples, "0_0_0.tif").isEmpty());
  ASSERT_FALSE(writeTile(dir, w, h, geo_l7, samples, "7_0_0.tif").isEmpty());
  ASSERT_FALSE(writeTile(dir, w, h, geo_l13, samples, "13_0_0.tif").isEmpty());

  camp::map::Map map;
  auto* layer = new camp::raster::GggsTileLayer(map.topLevelLayers(), dir.path());
  ASSERT_TRUE(layer->valid());
  EXPECT_EQ(layer->availableLevels(), (std::vector<int>{0, 7, 13}));
  layer->setLodForTest(7, QRectF());    // mid-ladder selection
  layer->waitForLoad();
  EXPECT_EQ(layer->pixelsLoadedCount(0), 1);
  EXPECT_EQ(layer->pixelsLoadedCount(7), 1);
  EXPECT_EQ(layer->pixelsLoadedCount(13), 0) <<
    "level above the selection loaded — the demand-driven ceiling is gone "
    "(eager whole-store load regression)";
}

TEST(GggsRenderTest, HeadlessDefaultsLoadEverything)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  ASSERT_TRUE(QDir(dir.path()).mkdir("overviews"));
  const int w = 20, h = 20;
  const double fine_geo[6] = {-71.40, 0.0001, 0.0, 43.00, 0.0, -0.0001};
  const double coarse_geo[6] = {-72.00, 0.1, 0.0, 44.00, 0.0, -0.1};
  const std::vector<uint16_t> samples(w * h, 5000);
  ASSERT_FALSE(writeTile(dir, w, h, fine_geo, samples, "13_0_0.tif").isEmpty());
  ASSERT_FALSE(writeTile(dir, w, h, coarse_geo, samples,
                         "overviews/0_0_0.tif").isEmpty());

  camp::map::Map map;
  auto* layer = new camp::raster::GggsTileLayer(map.topLevelLayers(), dir.path());
  ASSERT_TRUE(layer->valid());
  EXPECT_EQ(layer->selectedLevel(), -1);   // no paint() ⇒ no selection
  layer->waitForLoad();                    // -1 ⇒ no filter ⇒ everything loads
  EXPECT_EQ(layer->pixelsLoadedCount(13), 1);
  EXPECT_EQ(layer->pixelsLoadedCount(0), 1);
}

// [camp#103] The pan re-kick predicate: unloaded tiles intersecting the
// viewport at the selected level ⇒ re-kick; all loaded (or nothing visible
// unloaded) ⇒ idle. This is the unit seam for the paint() must-fix (a pure pan
// previously never re-kicked the loader → permanently blank panned-in regions).
TEST(GggsRenderTest, UnloadedVisibleTilesTriggerRekick)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const int w = 20, h = 20;
  const double fine_geo[6] = {-71.40, 0.0001, 0.0, 43.00, 0.0, -0.0001};
  const std::vector<uint16_t> samples(w * h, 5000);
  ASSERT_FALSE(writeTile(dir, w, h, fine_geo, samples, "13_0_0.tif").isEmpty());

  camp::map::Map map;
  auto* layer = new camp::raster::GggsTileLayer(map.topLevelLayers(), dir.path());
  ASSERT_TRUE(layer->valid());
  layer->setLodForTest(13, QRectF());

  // Viewport over the tile, pixels not yet loaded ⇒ re-kick needed.
  const QRectF over_tile = layer->sceneBounds();
  EXPECT_TRUE(layer->hasUnloadedVisibleTiles(over_tile));
  // Viewport far from the tile ⇒ nothing visible is unloaded ⇒ idle.
  const QRectF far_away = over_tile.translated(over_tile.width() * 100.0, 0.0);
  EXPECT_FALSE(layer->hasUnloadedVisibleTiles(far_away));
  // After the load completes, even the over-tile viewport is idle.
  layer->waitForLoad();
  EXPECT_FALSE(layer->hasUnloadedVisibleTiles(over_tile));
}

// [camp#103 field verify / camp#194] Residency across level switches under
// multi-level compositing. Zoom-IN (coarse -> fine): the coarse level is <=
// the new selection, so it stays resident PERMANENTLY as part of the
// composited picture — even after the fine level finishes loading (the old
// equality-filter model released it here). Zoom-OUT (fine -> coarse): the
// fine level (now > selection) is the transition backdrop — never released
// eagerly, dropped by tilesReady() once the selection's visible set has
// loaded and the loader is idle — while the coarse level stays loaded
// throughout.
TEST(GggsRenderTest, LevelSwitchResidencyAcrossZoomInAndOut)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  ASSERT_TRUE(QDir(dir.path()).mkdir("overviews"));
  const int w = 20, h = 20;
  const double fine_geo[6] = {-71.40, 0.0001, 0.0, 43.00, 0.0, -0.0001};
  const double coarse_geo[6] = {-72.00, 0.1, 0.0, 44.00, 0.0, -0.1};
  const std::vector<uint16_t> samples(w * h, 5000);
  ASSERT_FALSE(writeTile(dir, w, h, fine_geo, samples, "13_0_0.tif").isEmpty());
  ASSERT_FALSE(writeTile(dir, w, h, coarse_geo, samples,
                         "overviews/0_0_0.tif").isEmpty());

  camp::map::Map map;
  auto* layer = new camp::raster::GggsTileLayer(map.topLevelLayers(), dir.path());
  ASSERT_TRUE(layer->valid());

  // Load at the coarse selection: only levels <= 0 load.
  layer->setLodForTest(0, QRectF());
  layer->waitForLoad();
  ASSERT_EQ(layer->pixelsLoadedCount(0), 1);
  ASSERT_EQ(layer->pixelsLoadedCount(13), 0);

  // Zoom IN: select the fine level. The coarse tile must stay resident both
  // mid-transition (backdrop) and — new under compositing — permanently after
  // the fine level completes (it is part of the composited picture).
  layer->setLodForTest(13, QRectF());
  EXPECT_EQ(layer->pixelsLoadedCount(0), 1) <<
    "outgoing level released eagerly — zoom would flicker blank";
  layer->waitForLoad();
  EXPECT_EQ(layer->pixelsLoadedCount(13), 1);
  EXPECT_EQ(layer->pixelsLoadedCount(0), 1) <<
    "coarse level released after the fine level completed — compositing keeps "
    "every level <= the selection resident permanently";

  // Zoom OUT: select the coarse level again. Everything <= the selection is
  // already loaded, so the now-stale fine level (> selection) releases at the
  // next idle tilesReady(); the coarse level stays loaded throughout.
  layer->setLodForTest(0, QRectF());
  layer->waitForLoad();
  EXPECT_EQ(layer->pixelsLoadedCount(0), 1);
  EXPECT_EQ(layer->pixelsLoadedCount(13), 0) <<
    "finer-than-selection level not released after the selection's visible "
    "set completed";
}

// [camp#194] Zoom-OUT transition backdrop: with the coarse level selected but
// its tiles not yet loaded, the still-resident FINER tiles (level > selection)
// must keep rendering — the render-time path must draw the whole resident set,
// not filter to level <= selection (which would blank the view until the
// coarse load completes, the exact flicker ADR-0013/camp#103 fixed for
// zoom-in). Residency asserts alone cannot catch this; the render must be
// non-blank. (GL-gated, like the zoom-in twin below.)
TEST(GggsRenderTest, ZoomOutRetainsFinerBackdropUntilCoarseLoads)
{
  if(!offscreenGLAvailable())
    GTEST_SKIP() << "no offscreen GL context available";

  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const int w = 20, h = 20;
  // A two-level NATIVE ladder at two disjoint regions (0.002 deg tiles ~0.01
  // deg apart), so the spatial filter can load one without the other while
  // both stay a visible fraction of the union extent.
  const double fine_geo[6] = {-71.400, 0.0001, 0.0, 43.000, 0.0, -0.0001};
  const double coarse_geo[6] = {-71.410, 0.0001, 0.0, 43.010, 0.0, -0.0001};
  const std::vector<uint16_t> samples(w * h, 5000);
  ASSERT_FALSE(writeTile(dir, w, h, fine_geo, samples, "13_0_0.tif").isEmpty());
  ASSERT_FALSE(writeTile(dir, w, h, coarse_geo, samples, "0_0_0.tif").isEmpty());

  camp::map::Map map;
  auto* layer = new camp::raster::GggsTileLayer(map.topLevelLayers(), dir.path());
  ASSERT_TRUE(layer->valid());

  // Load ONLY the fine tile: fine selection + a viewport over the fine
  // region — the disjoint coarse tile fails the spatial filter.
  const QPointF fine_lo = web_mercator::geoToMap(QGeoCoordinate(42.998, -71.400));
  const QPointF fine_hi = web_mercator::geoToMap(QGeoCoordinate(43.000, -71.398));
  const QRectF fine_viewport = QRectF(fine_lo, fine_hi).normalized();
  layer->setLodForTest(13, fine_viewport);
  layer->waitForLoad();
  ASSERT_EQ(layer->pixelsLoadedCount(13), 1);
  ASSERT_EQ(layer->pixelsLoadedCount(0), 0);

  // Zoom OUT: select the coarse level over the whole extent. Mid-transition
  // (coarse not yet loaded) the fine tile must stay resident...
  layer->setLodForTest(0, layer->sceneBounds());
  EXPECT_EQ(layer->pixelsLoadedCount(13), 1) <<
    "zoom-out backdrop released before the coarse level loaded";
  // ...and must still RENDER — the no-blank-frame guarantee on zoom-out.
  const QImage img = layer->renderImage(QSize(200, 200));
  ASSERT_FALSE(img.isNull());
  int opaque = 0;
  for(int y = 0; y < img.height(); ++y)
    for(int x = 0; x < img.width(); ++x)
      if(img.pixelColor(x, y).alpha() > 0)
        ++opaque;
  EXPECT_GT(opaque, 0) <<
    "zoom-out mid-transition render is blank — the finer backdrop is not drawn";

  // Drive the coarse load; once the selection's visible set completes, the
  // finer backdrop releases.
  layer->waitForLoad();
  EXPECT_EQ(layer->pixelsLoadedCount(0), 1);
  EXPECT_EQ(layer->pixelsLoadedCount(13), 0) <<
    "finer backdrop not released after the coarse visible set completed";
}

// [camp#103 field verify / camp#194] The zoom-IN mid-transition render: with
// the fine level selected but not yet loaded, the resident coarse tile draws —
// the render must NOT be blank. (GL-gated; discriminates the old eager-release
// behavior, under which this render was fully transparent.) Under multi-level
// compositing the coarse level, being <= the new selection, is a PERMANENT
// part of the composited picture rather than a transient backdrop — the
// assertions are unchanged, but what they demonstrate shifted (camp#194).
TEST(GggsRenderTest, LevelSwitchBackdropRendersDuringTransition)
{
  if(!offscreenGLAvailable())
    GTEST_SKIP() << "no offscreen GL context available";

  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  ASSERT_TRUE(QDir(dir.path()).mkdir("overviews"));
  const int w = 20, h = 20;
  const double fine_geo[6] = {-71.40, 0.0001, 0.0, 43.00, 0.0, -0.0001};
  const double coarse_geo[6] = {-72.00, 0.1, 0.0, 44.00, 0.0, -0.1};
  const std::vector<uint16_t> samples(w * h, 5000);
  ASSERT_FALSE(writeTile(dir, w, h, fine_geo, samples, "13_0_0.tif").isEmpty());
  ASSERT_FALSE(writeTile(dir, w, h, coarse_geo, samples,
                         "overviews/0_0_0.tif").isEmpty());

  camp::map::Map map;
  auto* layer = new camp::raster::GggsTileLayer(map.topLevelLayers(), dir.path());
  ASSERT_TRUE(layer->valid());
  layer->setLodForTest(0, QRectF());
  layer->waitForLoad();

  // Mid-transition state: fine selected, fine not loaded, coarse resident.
  layer->setLodForTest(13, QRectF());
  ASSERT_EQ(layer->pixelsLoadedCount(13), 0);
  const QImage img = layer->renderImage(QSize(100, 100));
  ASSERT_FALSE(img.isNull());
  int opaque = 0;
  for(int y = 0; y < img.height(); ++y)
    for(int x = 0; x < img.width(); ++x)
      if(img.pixelColor(x, y).alpha() > 0)
        ++opaque;
  EXPECT_GT(opaque, 0) <<
    "mid-transition render is blank — the coarse backdrop is not drawn";
}

int main(int argc, char** argv)
{
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(argc, argv);
  // [camp#117] Map's ctor now writes the BackgroundTileLayers seed into QSettings;
  // a test org/app name keeps that out of the developer's real camp settings.
  QCoreApplication::setOrganizationName("camp_test");
  QCoreApplication::setApplicationName("test_gggs_render");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
