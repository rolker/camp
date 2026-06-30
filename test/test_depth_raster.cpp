// Safe-degradation tests for DepthRaster (#59 PR6 increment 1 — depth extracted
// from BackgroundRaster). Depth feeds the planner's shoal avoidance, so a
// missing/invalid chart must degrade to "no depth" (NaN) rather than crashing or
// returning a garbage value that could read as deep water. Correctness against a
// real depth chart (the load mirrors BackgroundRaster's exactly) is verified in
// sim: cursor depth readout + planner shoal avoidance.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <gdal_priv.h>
#include <ogr_spatialref.h>

#include <QGeoCoordinate>
#include <QTemporaryDir>

#include "depth_raster.h"

namespace
{

// Minimal valid WGS84 single-band Float32 GeoTIFF so DepthRaster's load runs its
// full GDAL open + georeference path (and so the open-dataset count below moves).
QString writeDepthRaster(const QTemporaryDir& dir)
{
  GDALAllRegister();
  GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("GTiff");
  if(!driver)
    return QString();
  const QString path = dir.filePath("depth.tif");
  const int w = 8, h = 8;
  GDALDataset* ds = driver->Create(path.toUtf8().constData(), w, h, 1, GDT_Float32, nullptr);
  if(!ds)
    return QString();
  double geo[6] = {-70.80, 0.0001, 0.0, 43.20, 0.0, -0.0001};
  ds->SetGeoTransform(geo);
  OGRSpatialReference srs;
  srs.SetWellKnownGeogCS("WGS84");
  char* wkt = nullptr;
  srs.exportToWkt(&wkt);
  ds->SetProjection(wkt);
  CPLFree(wkt);
  std::vector<float> samples(static_cast<size_t>(w) * h, -5.0f);
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

}  // namespace

// A file that does not exist must not crash and must report no depth.
TEST(DepthRasterTest, NonexistentFileHasNoDepth)
{
  DepthRaster dr("/nonexistent/path/no_such_chart.tif");
  EXPECT_FALSE(dr.depthValid());
  EXPECT_EQ(dr.width(), 0);
  EXPECT_EQ(dr.height(), 0);
}

// Queries against an invalid raster return NaN (so AutonomousVehicleProject::
// getDepth surfaces "unknown", which the planner treats as unsafe) — never a
// finite garbage depth, and never a crash via the unset georeference.
TEST(DepthRasterTest, InvalidRasterReturnsNanDepth)
{
  DepthRaster dr("/nonexistent/path/no_such_chart.tif");
  EXPECT_TRUE(std::isnan(dr.getDepth(0, 0)));
  EXPECT_TRUE(std::isnan(dr.getDepth(-1, -1)));
  EXPECT_TRUE(std::isnan(dr.getDepth(QGeoCoordinate(43.07, -70.71))));
}

// [camp#152] DepthRaster's ctor opens a GDAL dataset and must GDALClose it
// (depth_raster.cpp). Loading then destroying the provider must leave the
// process-wide open-dataset count at its baseline. NOTE: this guards the
// pre-existing GDALClose, NOT this PR's new ~Georeferenced() — the dtor frees
// the OGRCoordinateTransformation pipelines, which GetOpenDatasets() cannot see;
// that ~180 KB leak is proven only under valgrind. A valid load is asserted so
// the check is not vacuous (a failed load opens nothing).
TEST(DepthRasterTest, LoadAndDestroyLeavesNoOpenDataset)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString path = writeDepthRaster(dir);
  ASSERT_FALSE(path.isEmpty());

  const int baseline = openDatasetCount();
  {
    DepthRaster dr(path);
    ASSERT_TRUE(dr.depthValid());  // non-vacuous: the load actually opened a dataset
    // The ctor already opened AND closed the source dataset.
    EXPECT_EQ(openDatasetCount(), baseline);
  }
  // After destruction the count is still at baseline.
  EXPECT_EQ(openDatasetCount(), baseline);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
