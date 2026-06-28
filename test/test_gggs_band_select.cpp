// [camp#108] Layer-level band selection for GggsTileLayer. A multi-band tile-set
// directory is rendered through the layer's offscreen GL path; switching the
// rendered band via setBand() must abort + reload the async pixel read and shift
// the layer's auto-range to the new band's distinct value range.
//
// Skipped automatically (not failed) if no offscreen GL context can be created,
// using the same offscreenGLAvailable() guard as test_gggs_render.cpp — the
// layer's renderImage()/setBand() texture path needs a real GL context.

#include <gtest/gtest.h>

#include <vector>

#include <gdal_priv.h>

#include <QApplication>
#include <QOffscreenSurface>
#include <QOpenGLContext>
#include <QTemporaryDir>

#include "map/map.h"
#include "map/layer_list.h"
#include "raster/gggs_tile_layer.h"

namespace
{

// Write a 2-band UInt16 north-up GeoTIFF (NoData = 0 on each band) with disjoint
// per-band ranges so a band switch is unambiguous.
QString writeTwoBandTile(const QTemporaryDir& dir, const QString& name,
                         int w, int h, const double geo[6],
                         const std::vector<uint16_t>& band1,
                         const std::vector<uint16_t>& band2)
{
  if(GDALGetDriverCount() == 0)
    GDALAllRegister();
  const QString path = dir.filePath(name);
  GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("GTiff");
  GDALDataset* ds = driver->Create(path.toUtf8().constData(), w, h, 2, GDT_UInt16, nullptr);
  ds->SetGeoTransform(const_cast<double*>(geo));
  GDALRasterBand* b1 = ds->GetRasterBand(1);
  b1->SetNoDataValue(0);
  CPLErr err = b1->RasterIO(GF_Write, 0, 0, w, h,
                            const_cast<uint16_t*>(band1.data()), w, h, GDT_UInt16, 0, 0);
  if(err == CE_None)
  {
    GDALRasterBand* b2 = ds->GetRasterBand(2);
    b2->SetNoDataValue(0);
    err = b2->RasterIO(GF_Write, 0, 0, w, h,
                       const_cast<uint16_t*>(band2.data()), w, h, GDT_UInt16, 0, 0);
  }
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

// A 2-band tile-set: the layer reports bandCount() == 2 and renders band 1 by
// default; setBand(2) reloads and shifts the auto-range to band 2's range.
TEST(GggsBandSelectTest, SetBandShiftsLayerRange)
{
  if(!offscreenGLAvailable())
    GTEST_SKIP() << "no offscreen GL context available";

  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());

  const int w = 8, h = 8;
  const double geo[6] = {-71.40, 0.0001, 0.0, 43.00, 0.0, -0.0001};
  // Disjoint ranges AND distinct spatial patterns so the rendered output differs:
  // the shader maps the auto-range-normalized value, so two bands that normalized
  // to the same pattern would render identically even with disjoint raw ranges.
  // Band 1: column ramp over [10, 40]; band 2: row ramp over [1000, 3800].
  std::vector<uint16_t> band1(w * h), band2(w * h);
  for(int r = 0; r < h; ++r)
    for(int c = 0; c < w; ++c)
    {
      band1[r * w + c] = uint16_t(10 + (c % 4) * 10);   // column ramp, [10, 40]
      band2[r * w + c] = uint16_t(1000 + r * 400);      // row ramp, [1000, 3800]
    }
  const QString path = writeTwoBandTile(dir, "13_0_0.tif", w, h, geo, band1, band2);
  ASSERT_FALSE(path.isEmpty());

  camp::map::Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);
  auto* layer = new camp::raster::GggsTileLayer(layers, dir.path());
  ASSERT_TRUE(layer->valid());
  EXPECT_EQ(layer->bandCount(), 2);
  EXPECT_EQ(layer->band(), 1);

  // Load band 1 and render so the layer auto-range folds band 1's range.
  layer->waitForLoad();
  QImage img1 = layer->renderImage(QSize(64, 64));
  ASSERT_FALSE(img1.isNull());

  // Switch to band 2: aborts + reloads, releases textures, resets the range.
  layer->setBand(2);
  EXPECT_EQ(layer->band(), 2);
  layer->waitForLoad();
  QImage img2 = layer->renderImage(QSize(64, 64));
  ASSERT_FALSE(img2.isNull());

  // The two bands have disjoint ranges, so the auto-ranged grayscale/colormap
  // mapping differs — the renders must not be pixel-identical. (A direct
  // data_min_/data_max_ getter is not exposed; the rendered image is the
  // observable proof the range shifted.)
  EXPECT_NE(img1, img2);
}

int main(int argc, char** argv)
{
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
