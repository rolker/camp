// [camp#96] Regression test for the GDAL dataset leak in
// RasterLayer::loadAndReprojectFile. That function opens a source dataset
// (GDALOpen) and a warped VRT (GDALAutoCreateWarpedVRT) on every invocation
// (ctor, setColormap, readSettings) and historically closed neither — leaking
// two handles per load. The fix wraps both in RAII unique_ptrs; this test
// proves no handle survives a load + colormap re-render cycle.
//
// Strategy (Plan Review resolutions):
//  - #2 baseline-delta: GDALDataset::GetOpenDatasets() counts ALL process-wide
//    open handles, so capture the count BEFORE constructing the layer and
//    assert it returns to that baseline after destruction (delta == 0), rather
//    than assuming zero.
//  - #1 no vacuous pass: assert each load actually SUCCEEDED and reached the
//    warp path (status() == "" — imageReady() only clears the status after the
//    warp produced non-empty mipmaps; a null/unwarpable load leaves
//    "(load failed)"). A silently-failing load can't make the leak check pass
//    vacuously, because a load that never warps also never opens the handles.

#include <gtest/gtest.h>

#include <vector>

#include <gdal_priv.h>
#include <ogr_spatialref.h>

#include <QApplication>
#include <QElapsedTimer>
#include <QTemporaryDir>

#include "map/map.h"
#include "map/layer_list.h"
#include "raster/raster_layer.h"

namespace
{

// Minimal valid WGS84 single-band Float32 GeoTIFF with real georeferencing, so
// GDALAutoCreateWarpedVRT(... web_mercator ...) actually produces a reprojected
// dataset and loadAndReprojectFile runs its full warp + scalar-shade path.
QString writeRaster(const QTemporaryDir& dir)
{
  if(GDALGetDriverCount() == 0)
    GDALAllRegister();
  const QString path = dir.filePath("raster.tif");
  GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("GTiff");
  if(!driver)
    return QString();

  const int w = 8, h = 8;
  GDALDataset* ds = driver->Create(path.toUtf8().constData(), w, h, 1, GDT_Float32, nullptr);
  if(!ds)
    return QString();

  // Small north-up patch near (-71.4, 43.0).
  double geo[6] = {-71.40, 0.0001, 0.0, 43.00, 0.0, -0.0001};
  ds->SetGeoTransform(geo);

  OGRSpatialReference srs;
  srs.SetWellKnownGeogCS("WGS84");
  char* wkt = nullptr;
  srs.exportToWkt(&wkt);
  ds->SetProjection(wkt);
  CPLFree(wkt);

  // Varying values so the colormap spans a real (non-degenerate) range.
  std::vector<float> samples(static_cast<size_t>(w) * h);
  for(size_t i = 0; i < samples.size(); ++i)
    samples[i] = static_cast<float>(i);

  GDALRasterBand* band = ds->GetRasterBand(1);
  const CPLErr err = band->RasterIO(GF_Write, 0, 0, w, h, samples.data(), w, h, GDT_Float32, 0, 0);
  GDALClose(ds);
  return err == CE_None ? path : QString();
}

int openDatasetCount()
{
  int count = 0;
  GDALDataset::GetOpenDatasets(&count);
  return count;
}

// The pixel load is async (QtConcurrent + QFutureWatcher); imageReady() runs on
// the event loop and sets the status to "" (success) or "(load failed)". Spin
// the loop until the status leaves "(loading...)"; returns true on success.
bool waitForLoad(const camp::raster::RasterLayer* layer, int timeout_ms = 5000)
{
  QElapsedTimer timer;
  timer.start();
  while(layer->status() == QStringLiteral("(loading...)") && timer.elapsed() < timeout_ms)
    QCoreApplication::processEvents(QEventLoop::AllEvents, 50);
  return layer->status().isEmpty();
}

}  // namespace

TEST(RasterLayerGdalCleanupTest, LoadAndColormapFlipsLeakNoDatasets)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString path = writeRaster(dir);
  ASSERT_FALSE(path.isEmpty());

  camp::map::Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);

  // [#2] Baseline AFTER writeRaster (which opened+closed its own handle) and
  // BEFORE the layer exists — Map/GDAL internals may hold unrelated handles.
  const int baseline = openDatasetCount();

  auto* layer = new camp::raster::RasterLayer(layers, path);

  // [#1] The synchronous initExtent() warp must have succeeded...
  ASSERT_TRUE(layer->valid());
  EXPECT_FALSE(layer->boundingRect().isEmpty());
  // ...and so must the async load that opens the handles under test.
  ASSERT_TRUE(waitForLoad(layer)) << "initial load did not reach the warp path";

  // Two additional ramp names (default is viridis); each re-invokes
  // loadAndReprojectFile, opening and — with the fix — closing both handles.
  layer->setColormap("grayscale");
  ASSERT_TRUE(waitForLoad(layer)) << "grayscale re-render did not reach the warp path";
  layer->setColormap("turbo");
  ASSERT_TRUE(waitForLoad(layer)) << "turbo re-render did not reach the warp path";

  // Dtor aborts + joins the in-flight load (waitForFinished), so every
  // loadAndReprojectFile invocation has returned and closed its handles.
  // Deliberately a raw `delete` rather than removeFromMap(): removeFromMap()
  // defers destruction (deleteLater) to a later event-loop turn, so the
  // open-dataset count below could run before the dtor's join completes and
  // flake. The direct delete makes the join synchronous and the guarantee firm.
  delete layer;

  // [#2] Every load + colormap flip closed its source dataset and warped VRT.
  EXPECT_EQ(openDatasetCount(), baseline);
}

int main(int argc, char** argv)
{
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
