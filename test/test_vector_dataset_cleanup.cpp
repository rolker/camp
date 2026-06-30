// [camp#152] Regression test for the GDAL/OGR handle leaks in VectorDataset
// chart loading. The leak-prone resource lifecycle — opening the dataset,
// creating one OGRCoordinateTransformation per layer, and creating an
// OGRPointIterator per ring — was historically interleaved with construction of
// the project item graph (Group/Point/LineString/Polygon + AutonomousVehicle-
// Project signal connections), which made it impossible to exercise without
// standing up the whole application. The fix splits the pure parse into
// camp::vector::parseVectorLayers (a MissionItem-free TU), which this test links
// directly and drives headless.
//
// Coverage:
//  - GDAL *dataset* handle: asserted via GDALDataset::GetOpenDatasets() — a
//    baseline-delta (the count returns to what it was before the test opened its
//    own handle), proving parseVectorLayers opens/leaks no dataset of its own.
//  - OGRCoordinateTransformation + OGRPointIterator: these are NOT visible to
//    GetOpenDatasets() and are the primary ~180 KB leak. They are proven closed
//    only under valgrind — run this binary with `valgrind --leak-check=full` and
//    require "definitely lost: 0 bytes". The asserts below also force every
//    leak site to execute (multi-layer, points, linestrings, polygon interior
//    rings) so a valgrind run is non-vacuous.

#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include <gdal_priv.h>
#include <ogr_spatialref.h>
#include <ogrsf_frmts.h>

#include <QTemporaryDir>

#include "vector/vector_parse.h"

namespace
{

int openDatasetCount()
{
  int count = 0;
  GDALDataset::GetOpenDatasets(&count);
  return count;
}

// Add a point, a linestring, and a polygon-with-one-hole to `layer` — one of
// each geometry type so every parse branch (and every OGRPointIterator site)
// runs.
void populateLayer(OGRLayer* layer)
{
  OGRFeatureDefn* defn = layer->GetLayerDefn();

  {
    OGRFeature* f = OGRFeature::CreateFeature(defn);
    OGRPoint pt(-70.71, 43.07);
    f->SetGeometry(&pt);
    EXPECT_EQ(layer->CreateFeature(f), OGRERR_NONE);
    OGRFeature::DestroyFeature(f);
  }
  {
    OGRFeature* f = OGRFeature::CreateFeature(defn);
    OGRLineString ls;
    ls.addPoint(-70.70, 43.06);
    ls.addPoint(-70.69, 43.05);
    ls.addPoint(-70.68, 43.07);
    f->SetGeometry(&ls);
    EXPECT_EQ(layer->CreateFeature(f), OGRERR_NONE);
    OGRFeature::DestroyFeature(f);
  }
  {
    OGRFeature* f = OGRFeature::CreateFeature(defn);
    OGRPolygon poly;
    OGRLinearRing exterior;
    exterior.addPoint(-70.80, 43.00);
    exterior.addPoint(-70.60, 43.00);
    exterior.addPoint(-70.60, 43.20);
    exterior.addPoint(-70.80, 43.20);
    exterior.closeRings();
    poly.addRing(&exterior);
    OGRLinearRing hole;
    hole.addPoint(-70.74, 43.06);
    hole.addPoint(-70.66, 43.06);
    hole.addPoint(-70.66, 43.14);
    hole.addPoint(-70.74, 43.14);
    hole.closeRings();
    poly.addRing(&hole);
    f->SetGeometry(&poly);
    EXPECT_EQ(layer->CreateFeature(f), OGRERR_NONE);
    OGRFeature::DestroyFeature(f);
  }
}

// Write a 2-layer GeoPackage (each layer WGS84, each with the three geometry
// types) so the per-layer transform-destroy path runs more than once.
QString writeGeoPackage(const QTemporaryDir& dir)
{
  GDALAllRegister();
  GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("GPKG");
  if(!driver)
    return QString();

  const QString path = dir.filePath("vectors.gpkg");
  GDALDataset* ds = driver->Create(path.toUtf8().constData(), 0, 0, 0, GDT_Unknown, nullptr);
  if(!ds)
    return QString();

  OGRSpatialReference srs;
  srs.SetWellKnownGeogCS("WGS84");
  srs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

  for(const char* name : {"alpha", "beta"})
  {
    OGRLayer* layer = ds->CreateLayer(name, &srs, wkbUnknown, nullptr);
    if(!layer)
    {
      GDALClose(ds);
      return QString();
    }
    populateLayer(layer);
  }

  GDALClose(ds);
  return path;
}

}  // namespace

TEST(VectorDatasetCleanupTest, ParseLeavesNoOpenDataset)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString path = writeGeoPackage(dir);
  ASSERT_FALSE(path.isEmpty());

  // Baseline AFTER writeGeoPackage closed its writer handle and BEFORE this test
  // opens its own — GDAL internals may hold unrelated handles.
  const int baseline = openDatasetCount();

  {
    const auto gdal_closer = [](GDALDataset* d){ if(d) GDALClose(d); };
    std::unique_ptr<GDALDataset, decltype(gdal_closer)> dataset(
      reinterpret_cast<GDALDataset*>(GDALOpenEx(path.toUtf8().constData(), GDAL_OF_READONLY, nullptr, nullptr, nullptr)),
      gdal_closer);
    ASSERT_TRUE(dataset);

    // This test's own open handle is live now.
    EXPECT_EQ(openDatasetCount(), baseline + 1);

    const std::vector<camp::vector::ParsedLayer> layers =
      camp::vector::parseVectorLayers(dataset.get());

    // Non-vacuous: every parse branch produced what we wrote, so a valgrind run
    // of this binary genuinely exercises all the transform/iterator leak sites.
    ASSERT_EQ(layers.size(), 2u);
    for(const auto& layer : layers)
    {
      ASSERT_EQ(layer.geometries.size(), 3u);
      int points = 0, lines = 0, polygons = 0;
      for(const auto& g : layer.geometries)
      {
        switch(g.type)
        {
        case camp::vector::ParsedGeometry::Point:
          ++points;
          EXPECT_EQ(g.exterior.size(), 1u);
          break;
        case camp::vector::ParsedGeometry::LineString:
          ++lines;
          EXPECT_EQ(g.exterior.size(), 3u);
          break;
        case camp::vector::ParsedGeometry::Polygon:
          ++polygons;
          EXPECT_GE(g.exterior.size(), 4u);
          ASSERT_EQ(g.interiorRings.size(), 1u);  // the hole — interior-ring iterator site
          EXPECT_GE(g.interiorRings.front().size(), 4u);
          break;
        }
      }
      EXPECT_EQ(points, 1);
      EXPECT_EQ(lines, 1);
      EXPECT_EQ(polygons, 1);
    }
  }

  // The RAII dataset closed at scope exit; parseVectorLayers leaked no dataset.
  EXPECT_EQ(openDatasetCount(), baseline);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
