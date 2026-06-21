// [camp#90 / I4] Tests for the GGGS GPU-warp tile layer's non-GL logic:
//  - GggsTile geographic-extent recovery from the GDAL geotransform,
//  - NoData / data-range handling (the mosaicker reserves 0 for NoData and
//    floors real returns to >= 1), and
//  - geoToMap parity: the vertex shader warps lon/lat -> Web-Mercator with
//    x = R*lambda, y = R*asinh(tan phi), expanded as log(t + sqrt(t^2+1)) to
//    avoid the asinh builtin. This pins that formula against the C++ reference
//    web_mercator::geoToMap so the shader can't silently drift. (Full offscreen
//    GPU/CPU render parity is the rqt#48-style harness deferred to camp#63.)

#include <gtest/gtest.h>

#include <atomic>
#include <cmath>
#include <thread>
#include <vector>

#include <gdal_priv.h>

#include <QGeoCoordinate>
#include <QPointF>
#include <QTemporaryDir>

#include "raster/gggs_tile.h"
#include "map_view/web_mercator.h"

using camp::raster::GggsTile;

namespace
{

// Write a single-band UInt16 north-up WGS84-ish GeoTIFF with the given
// geotransform, NoData = 0, and row-major samples. Returns the file path.
QString writeTile(const QTemporaryDir& dir, const QString& name,
                  int width, int height, const double geo[6],
                  const std::vector<uint16_t>& samples)
{
  if(GDALGetDriverCount() == 0)
    GDALAllRegister();
  const QString path = dir.filePath(name);
  GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("GTiff");
  GDALDataset* ds = driver->Create(path.toUtf8().constData(), width, height, 1,
                                   GDT_UInt16, nullptr);
  ds->SetGeoTransform(const_cast<double*>(geo));
  GDALRasterBand* band = ds->GetRasterBand(1);
  band->SetNoDataValue(0);
  const CPLErr err = band->RasterIO(GF_Write, 0, 0, width, height,
                                    const_cast<uint16_t*>(samples.data()),
                                    width, height, GDT_UInt16, 0, 0);
  GDALClose(ds);
  if(err != CE_None)
    return QString();
  return path;
}

}  // namespace

// Extent corners come straight from the geotransform: north-up tile, row 0 is
// the north edge (maxLat).
TEST(GggsTileTest, ExtentFromGeotransform)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const int w = 4, h = 4;
  const double dlon = 0.001, dlat = 0.001;
  const double min_lon = -71.4, max_lat = 43.0;
  const double geo[6] = {min_lon, dlon, 0.0, max_lat, 0.0, -dlat};
  std::vector<uint16_t> samples(w * h, 100);
  const QString path = writeTile(dir, "13_10_20.tif", w, h, geo, samples);

  GggsTile tile(path);
  ASSERT_TRUE(tile.valid());
  EXPECT_EQ(tile.width(), w);
  EXPECT_EQ(tile.height(), h);
  EXPECT_NEAR(tile.minLon(), min_lon, 1e-12);
  EXPECT_NEAR(tile.maxLon(), min_lon + w * dlon, 1e-12);
  EXPECT_NEAR(tile.maxLat(), max_lat, 1e-12);
  EXPECT_NEAR(tile.minLat(), max_lat - h * dlat, 1e-12);
}

// [camp#102] The constructor reads extent/dimensions ONLY — valid() and the
// geographic extent are known before any pixel read. The data range stays at the
// crossed sentinel until loadPixels() runs, which then populates it.
TEST(GggsTileTest, ExtentKnownBeforePixelsLoad)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const int w = 2, h = 2;
  const double geo[6] = {-71.4, 0.001, 0.0, 43.0, 0.0, -0.001};
  std::vector<uint16_t> samples = {0, 5, 12345, 50000};
  const QString path = writeTile(dir, "13_2_2.tif", w, h, geo, samples);

  GggsTile tile(path);
  // Extent + dimensions are valid synchronously, before any RasterIO.
  ASSERT_TRUE(tile.valid());
  EXPECT_FALSE(tile.pixelsLoaded());
  EXPECT_EQ(tile.width(), w);
  EXPECT_EQ(tile.height(), h);
  EXPECT_NEAR(tile.maxLat(), 43.0, 1e-12);
  // The range is crossed (unknown) until pixels load.
  EXPECT_GT(tile.dataMin(), tile.dataMax());

  // loadPixels() reads the band and populates the range.
  EXPECT_TRUE(tile.loadPixels());
  EXPECT_TRUE(tile.pixelsLoaded());
  EXPECT_DOUBLE_EQ(tile.dataMin(), 5.0);
  EXPECT_DOUBLE_EQ(tile.dataMax(), 50000.0);

  // Idempotent: a second call is a no-op and stays loaded.
  EXPECT_TRUE(tile.loadPixels());
  EXPECT_TRUE(tile.pixelsLoaded());
}

// NoData (0) is excluded from the data range; real samples set min/max
// (post loadPixels()).
TEST(GggsTileTest, NoDataExcludedFromRange)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const int w = 2, h = 2;
  const double geo[6] = {-71.4, 0.001, 0.0, 43.0, 0.0, -0.001};
  // one NoData (0), the rest spanning [5, 50000]
  std::vector<uint16_t> samples = {0, 5, 12345, 50000};
  const QString path = writeTile(dir, "13_0_0.tif", w, h, geo, samples);

  GggsTile tile(path);
  ASSERT_TRUE(tile.valid());
  EXPECT_TRUE(tile.hasNoData());
  EXPECT_DOUBLE_EQ(tile.noData(), 0.0);
  ASSERT_TRUE(tile.loadPixels());
  EXPECT_DOUBLE_EQ(tile.dataMin(), 5.0);
  EXPECT_DOUBLE_EQ(tile.dataMax(), 50000.0);
}

// An all-NoData tile has a crossed range (min > max) so the layer can skip it.
TEST(GggsTileTest, AllNoDataHasCrossedRange)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const int w = 2, h = 2;
  const double geo[6] = {-71.4, 0.001, 0.0, 43.0, 0.0, -0.001};
  std::vector<uint16_t> samples(w * h, 0);
  const QString path = writeTile(dir, "13_1_1.tif", w, h, geo, samples);

  GggsTile tile(path);
  ASSERT_TRUE(tile.valid());           // dimensions known
  ASSERT_TRUE(tile.loadPixels());      // pixels read (all NoData)
  EXPECT_GT(tile.dataMin(), tile.dataMax());   // no valid samples
}

// [camp#102] Publication-contract regression: pixelsLoaded() must stay false
// until loadPixels() has fully populated the buffer + range. The layer's paint
// path gates every data_/texture() read on this flag (an acquire load that pairs
// with the worker's release store), so this flag flipping early — or the range
// being observable before the flag — would reopen the worker-vs-paint race the
// rescan() re-kick exposes. Pins the contract without GL/event-loop scaffolding.
TEST(GggsTileTest, PixelsLoadedFalseUntilLoadCompletes)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const int w = 2, h = 2;
  const double geo[6] = {-71.4, 0.001, 0.0, 43.0, 0.0, -0.001};
  std::vector<uint16_t> samples = {0, 5, 12345, 50000};
  const QString path = writeTile(dir, "13_3_3.tif", w, h, geo, samples);

  GggsTile tile(path);
  ASSERT_TRUE(tile.valid());
  // Before any load: flag false and the range is still the crossed sentinel.
  EXPECT_FALSE(tile.pixelsLoaded());
  EXPECT_GT(tile.dataMin(), tile.dataMax());

  ASSERT_TRUE(tile.loadPixels());
  // After load: flag true AND the range is consistent (the flag must not become
  // observable before the range it advertises).
  EXPECT_TRUE(tile.pixelsLoaded());
  EXPECT_LE(tile.dataMin(), tile.dataMax());
}

// [camp#102] Cross-thread publication: a worker thread loads the pixels while a
// second thread spins on pixelsLoaded() exactly as the paint path does. The
// moment the spinner observes the (acquire) flag true, the released data_ range
// MUST already be visible and consistent — that is the happens-before the
// release/acquire pair provides. A plain (non-atomic, no-barrier) flag could let
// the spinner see true with a torn/stale range; this guards that regression.
TEST(GggsTileTest, PixelsPublishedToObserverThread)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const int w = 4, h = 4;
  const double geo[6] = {-71.4, 0.001, 0.0, 43.0, 0.0, -0.001};
  std::vector<uint16_t> samples(w * h);
  for(int i = 0; i < w * h; ++i)
    samples[i] = uint16_t(i + 1);   // 1..16, no NoData → range [1, 16]
  const QString path = writeTile(dir, "13_4_4.tif", w, h, geo, samples);

  GggsTile tile(path);
  ASSERT_TRUE(tile.valid());

  std::atomic<bool> go{false};
  std::atomic<bool> saw_inconsistent{false};

  // Observer mirrors renderImage()'s gate: only read the range once the flag
  // (acquire) reads true, then assert it is the fully-populated value.
  std::thread observer([&]() {
    while(!go.load(std::memory_order_acquire)) { /* spin to start */ }
    while(!tile.pixelsLoaded()) { /* spin until published */ }
    if(tile.dataMin() != 1.0 || tile.dataMax() != 16.0)
      saw_inconsistent.store(true, std::memory_order_relaxed);
  });

  std::thread worker([&]() {
    while(!go.load(std::memory_order_acquire)) { /* spin to start */ }
    tile.loadPixels();
  });

  go.store(true, std::memory_order_release);
  worker.join();
  observer.join();

  EXPECT_FALSE(saw_inconsistent.load(std::memory_order_relaxed));
  EXPECT_TRUE(tile.pixelsLoaded());
  EXPECT_DOUBLE_EQ(tile.dataMin(), 1.0);
  EXPECT_DOUBLE_EQ(tile.dataMax(), 16.0);
}

// A missing file degrades to invalid (no crash), like RasterLayer.
TEST(GggsTileTest, MissingFileInvalid)
{
  GggsTile tile("/nonexistent/path/13_0_0.tif");
  EXPECT_FALSE(tile.valid());
}

// The layer warps each mesh vertex via web_mercator::geoToMap on the CPU (in
// double precision — the warp is intentionally NOT done in the GPU shader, whose
// transcendentals x R caused a ~50 m latitude error). This pins geoToMap against
// the analytic EPSG:3857 formula: x = R*lambda (linear); y = R*asinh(tan phi).
TEST(GggsTileTest, GeoToMapMatchesWebMercator)
{
  const double R = web_mercator::earth_radius_at_equator;
  const double deg2rad = M_PI / 180.0;
  for(double lat : {-85.0, -43.0, -10.0, 0.0, 10.0, 42.99, 43.0, 85.0})
  {
    for(double lon : {-180.0, -71.39, 0.0, 71.39, 179.0})
    {
      // shader formula (see gggs_tile_layer.cpp kVertexShader)
      const double x = lon * deg2rad * R;
      const double t = std::tan(lat * deg2rad);
      const double y = std::log(t + std::sqrt(t * t + 1.0)) * R;

      const QPointF ref = web_mercator::geoToMap(QGeoCoordinate(lat, lon));
      // relative tolerance over the (large, ~1e7 m) Web-Mercator magnitudes
      const double scale = std::max(1.0, std::abs(ref.x()) + std::abs(ref.y()));
      EXPECT_NEAR(x, ref.x(), 1e-6 * scale) << "lon=" << lon;
      EXPECT_NEAR(y, ref.y(), 1e-6 * scale) << "lat=" << lat;
    }
  }
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
