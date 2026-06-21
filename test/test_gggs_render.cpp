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

#include <cstdlib>
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
                  const std::vector<uint16_t>& samples)
{
  if(GDALGetDriverCount() == 0)
    GDALAllRegister();
  const QString path = dir.filePath("13_0_0.tif");
  GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("GTiff");
  GDALDataset* ds = driver->Create(path.toUtf8().constData(), w, h, 1, GDT_UInt16, nullptr);
  ds->SetGeoTransform(const_cast<double*>(geo));
  GDALRasterBand* band = ds->GetRasterBand(1);
  band->SetNoDataValue(0);
  const CPLErr err = band->RasterIO(GF_Write, 0, 0, w, h,
                                    const_cast<uint16_t*>(samples.data()),
                                    w, h, GDT_UInt16, 0, 0);
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

int main(int argc, char** argv)
{
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
