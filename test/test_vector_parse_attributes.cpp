// [camp#22] Feature attributes + geometry coverage in camp::vector::parseVectorLayers.
//
// The parser is the shared front end for both vector-file entry points — the
// editable VectorDataset import and the read-only VectorLayer display — so the
// two things this issue adds to it are pinned here:
//
//  1. ATTRIBUTES. ParsedGeometry::attributes must round-trip a feature's fields
//     with their types (string / int / int64 / real), and must OMIT a field that
//     is unset on a feature rather than reporting it as an empty value: the
//     styling path paints a missing value in its no-data colour, which it can
//     only do if absence is distinguishable.
//  2. GEOMETRY COVERAGE. Before this change the parser switched on the RAW
//     geometry type with `default: break`, so every 25D variant (a GeoJSON point
//     with an elevation is wkbPoint25D) and every multi-part geometry
//     (MultiPolygon is routine in shapefile/KML) was dropped silently, while the
//     issue claimed those formats "come free from the driver".
//
// Written against a GeoPackage built in a temp dir — the same driver-backed
// pattern as test_vector_dataset_cleanup — plus a GeoJSON fixture of the shape
// of the acceptance datasets (numeric peaks + a string assessment field).

#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include <gdal_priv.h>
#include <ogr_spatialref.h>
#include <ogrsf_frmts.h>

#include <QFile>
#include <QTemporaryDir>

#include "vector/vector_parse.h"

using camp::vector::ParsedGeometry;
using camp::vector::ParsedLayer;

namespace
{

const auto gdal_closer = [](GDALDataset* d){ if(d) GDALClose(d); };
using DatasetPtr = std::unique_ptr<GDALDataset, decltype(gdal_closer)>;

DatasetPtr openDataset(const QString& path)
{
  return DatasetPtr(
    static_cast<GDALDataset*>(
      GDALOpenEx(path.toUtf8().constData(), GDAL_OF_READONLY | GDAL_OF_VECTOR, nullptr, nullptr, nullptr)),
    gdal_closer);
}

// A GeoPackage with one WGS84 layer carrying typed fields on every feature:
// a point (with all fields set), a 25D point (elevation — the variant that used
// to be dropped), a MultiPolygon of two parts, a MultiLineString of two parts,
// and a point with the string field deliberately left unset.
QString writeGeoPackage(const QTemporaryDir& dir)
{
  GDALAllRegister();
  GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("GPKG");
  if(!driver)
    return QString();

  const QString path = dir.filePath("attributes.gpkg");
  GDALDataset* ds = driver->Create(path.toUtf8().constData(), 0, 0, 0, GDT_Unknown, nullptr);
  if(!ds)
    return QString();

  OGRSpatialReference srs;
  srs.SetWellKnownGeogCS("WGS84");
  srs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

  OGRLayer* layer = ds->CreateLayer("features", &srs, wkbUnknown, nullptr);
  if(!layer)
  {
    GDALClose(ds);
    return QString();
  }

  {
    OGRFieldDefn name("name", OFTString);
    layer->CreateField(&name);
    OGRFieldDefn count("count", OFTInteger);
    layer->CreateField(&count);
    OGRFieldDefn big("big", OFTInteger64);
    layer->CreateField(&big);
    OGRFieldDefn value("value", OFTReal);
    layer->CreateField(&value);
  }

  OGRFeatureDefn* defn = layer->GetLayerDefn();

  auto setFields = [](OGRFeature* f, const char* name, int count, GIntBig big, double value)
  {
    f->SetField("name", name);
    f->SetField("count", count);
    f->SetField("big", big);
    f->SetField("value", value);
  };

  {  // plain 2D point, all fields set
    OGRFeature* f = OGRFeature::CreateFeature(defn);
    setFields(f, "alpha", 7, 4294967296LL, 1.5);
    OGRPoint pt(-70.71, 43.07);
    f->SetGeometry(&pt);
    layer->CreateFeature(f);
    OGRFeature::DestroyFeature(f);
  }
  {  // 25D point — wkbPoint25D, dropped by the pre-#22 raw-type switch
    OGRFeature* f = OGRFeature::CreateFeature(defn);
    setFields(f, "elevated", 1, 1LL, 2.5);
    OGRPoint pt(-70.70, 43.06, 12.25);
    f->SetGeometry(&pt);
    layer->CreateFeature(f);
    OGRFeature::DestroyFeature(f);
  }
  {  // MultiPolygon of two parts, the first with a hole
    OGRFeature* f = OGRFeature::CreateFeature(defn);
    setFields(f, "multipoly", 2, 2LL, 3.5);
    OGRMultiPolygon multi;
    for(int part = 0; part < 2; ++part)
    {
      OGRPolygon poly;
      OGRLinearRing exterior;
      const double x0 = -70.80 + part * 0.5;
      exterior.addPoint(x0, 43.00);
      exterior.addPoint(x0 + 0.2, 43.00);
      exterior.addPoint(x0 + 0.2, 43.20);
      exterior.addPoint(x0, 43.20);
      exterior.closeRings();
      poly.addRing(&exterior);
      if(part == 0)
      {
        OGRLinearRing hole;
        hole.addPoint(-70.74, 43.06);
        hole.addPoint(-70.72, 43.06);
        hole.addPoint(-70.72, 43.10);
        hole.addPoint(-70.74, 43.10);
        hole.closeRings();
        poly.addRing(&hole);
      }
      multi.addGeometry(&poly);
    }
    f->SetGeometry(&multi);
    layer->CreateFeature(f);
    OGRFeature::DestroyFeature(f);
  }
  {  // MultiLineString of two parts
    OGRFeature* f = OGRFeature::CreateFeature(defn);
    setFields(f, "multiline", 3, 3LL, 4.5);
    OGRMultiLineString multi;
    for(int part = 0; part < 2; ++part)
    {
      OGRLineString ls;
      ls.addPoint(-70.60 + part * 0.1, 43.30);
      ls.addPoint(-70.59 + part * 0.1, 43.31);
      ls.addPoint(-70.58 + part * 0.1, 43.32);
      multi.addGeometry(&ls);
    }
    f->SetGeometry(&multi);
    layer->CreateFeature(f);
    OGRFeature::DestroyFeature(f);
  }
  {  // point with "name" left unset — absence must survive the parse
    OGRFeature* f = OGRFeature::CreateFeature(defn);
    f->SetField("count", 9);
    f->SetField("value", 5.5);
    OGRPoint pt(-70.50, 43.40);
    f->SetGeometry(&pt);
    layer->CreateFeature(f);
    OGRFeature::DestroyFeature(f);
  }

  GDALClose(ds);
  return path;
}

// GeoJSON of the same shape as the issue's acceptance datasets: points with one
// numeric field and one string field, in lon/lat order with an elevation.
QString writeGeoJson(const QTemporaryDir& dir)
{
  const QString path = dir.filePath("peaks.geojson");
  QFile file(path);
  if(!file.open(QIODevice::WriteOnly | QIODevice::Text))
    return QString();
  file.write(R"({
    "type": "FeatureCollection",
    "features": [
      {"type": "Feature",
       "geometry": {"type": "Point", "coordinates": [-71.4361, 42.9925, 3.0]},
       "properties": {"analytic_signal_nT_per_m": 58.0, "assessment": "candidate C"}},
      {"type": "Feature",
       "geometry": {"type": "Point", "coordinates": [-71.4300, 42.9900]},
       "properties": {"analytic_signal_nT_per_m": 12.5, "assessment": "background"}}
    ]
  })");
  file.close();
  return path;
}

const ParsedGeometry* firstNamed(const ParsedLayer& layer, const QString& name)
{
  for(const auto& g : layer.geometries)
    if(g.attributes.value("name").toString() == name)
      return &g;
  return nullptr;
}

int countNamed(const ParsedLayer& layer, const QString& name)
{
  int n = 0;
  for(const auto& g : layer.geometries)
    if(g.attributes.value("name").toString() == name)
      ++n;
  return n;
}

}  // namespace

// Typed fields round-trip onto every parsed geometry, with their OGR types
// preserved (not stringified) so the styling path can read them as numbers.
TEST(VectorParseAttributes, TypedFieldsRoundTrip)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString path = writeGeoPackage(dir);
  ASSERT_FALSE(path.isEmpty());

  DatasetPtr dataset = openDataset(path);
  ASSERT_TRUE(dataset);
  const std::vector<ParsedLayer> layers = camp::vector::parseVectorLayers(dataset.get());
  ASSERT_EQ(layers.size(), 1u);

  const ParsedGeometry* alpha = firstNamed(layers.front(), "alpha");
  ASSERT_NE(alpha, nullptr);
  EXPECT_EQ(alpha->type, ParsedGeometry::Point);
  EXPECT_EQ(alpha->attributes.value("name").toString(), QStringLiteral("alpha"));
  EXPECT_EQ(alpha->attributes.value("count").type(), QVariant::Int);
  EXPECT_EQ(alpha->attributes.value("count").toInt(), 7);
  EXPECT_EQ(alpha->attributes.value("big").toLongLong(), 4294967296LL);
  EXPECT_EQ(alpha->attributes.value("value").type(), QVariant::Double);
  EXPECT_DOUBLE_EQ(alpha->attributes.value("value").toDouble(), 1.5);

  // The coordinate convention: latitude 43.07, longitude -70.71 (and NOT the
  // transposed pair) — the acceptance check for this issue is a coordinate check.
  ASSERT_EQ(alpha->exterior.size(), 1u);
  EXPECT_NEAR(alpha->exterior.front().latitude(), 43.07, 1e-6);
  EXPECT_NEAR(alpha->exterior.front().longitude(), -70.71, 1e-6);
}

// A field that is unset on a feature is ABSENT from the map, not present-and-empty.
TEST(VectorParseAttributes, UnsetFieldIsAbsent)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString path = writeGeoPackage(dir);
  ASSERT_FALSE(path.isEmpty());

  DatasetPtr dataset = openDataset(path);
  ASSERT_TRUE(dataset);
  const std::vector<ParsedLayer> layers = camp::vector::parseVectorLayers(dataset.get());
  ASSERT_EQ(layers.size(), 1u);

  const ParsedGeometry* unset = nullptr;
  for(const auto& g : layers.front().geometries)
    if(g.attributes.value("count").toInt() == 9)
      unset = &g;
  ASSERT_NE(unset, nullptr);
  EXPECT_FALSE(unset->attributes.contains("name"));
  EXPECT_TRUE(unset->attributes.contains("value"));
}

// Multi-part geometry emits one ParsedGeometry PER PART, each carrying the
// feature's attributes — instead of the pre-#22 silent drop.
TEST(VectorParseAttributes, MultiPartGeometryEmitsEveryPart)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString path = writeGeoPackage(dir);
  ASSERT_FALSE(path.isEmpty());

  DatasetPtr dataset = openDataset(path);
  ASSERT_TRUE(dataset);
  const std::vector<ParsedLayer> layers = camp::vector::parseVectorLayers(dataset.get());
  ASSERT_EQ(layers.size(), 1u);
  const ParsedLayer& layer = layers.front();

  EXPECT_EQ(countNamed(layer, "multipoly"), 2);
  EXPECT_EQ(countNamed(layer, "multiline"), 2);

  int polygons_with_hole = 0;
  for(const auto& g : layer.geometries)
    if(g.attributes.value("name").toString() == QStringLiteral("multipoly"))
    {
      EXPECT_EQ(g.type, ParsedGeometry::Polygon);
      EXPECT_GE(g.exterior.size(), 4u);
      EXPECT_DOUBLE_EQ(g.attributes.value("value").toDouble(), 3.5);
      if(!g.interiorRings.empty())
        ++polygons_with_hole;
    }
  EXPECT_EQ(polygons_with_hole, 1);

  for(const auto& g : layer.geometries)
    if(g.attributes.value("name").toString() == QStringLiteral("multiline"))
    {
      EXPECT_EQ(g.type, ParsedGeometry::LineString);
      EXPECT_EQ(g.exterior.size(), 3u);
    }
}

// A 25D point (wkbPoint25D) parses as a Point — the variant a GeoJSON file with
// elevations produces, and the one the raw-type switch used to drop.
TEST(VectorParseAttributes, TwentyFiveDVariantIsNotDropped)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString path = writeGeoPackage(dir);
  ASSERT_FALSE(path.isEmpty());

  DatasetPtr dataset = openDataset(path);
  ASSERT_TRUE(dataset);
  const std::vector<ParsedLayer> layers = camp::vector::parseVectorLayers(dataset.get());
  ASSERT_EQ(layers.size(), 1u);

  const ParsedGeometry* elevated = firstNamed(layers.front(), "elevated");
  ASSERT_NE(elevated, nullptr);
  EXPECT_EQ(elevated->type, ParsedGeometry::Point);
  ASSERT_EQ(elevated->exterior.size(), 1u);
  EXPECT_NEAR(elevated->exterior.front().latitude(), 43.06, 1e-6);
  EXPECT_NEAR(elevated->exterior.front().longitude(), -70.70, 1e-6);
}

// The acceptance-dataset shape: GeoJSON points with a numeric field and a string
// field, one of them carrying an elevation (again wkbPoint25D).
TEST(VectorParseAttributes, GeoJsonPeaksFixture)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString path = writeGeoJson(dir);
  ASSERT_FALSE(path.isEmpty());

  DatasetPtr dataset = openDataset(path);
  ASSERT_TRUE(dataset);
  const std::vector<ParsedLayer> layers = camp::vector::parseVectorLayers(dataset.get());
  ASSERT_EQ(layers.size(), 1u);
  const ParsedLayer& layer = layers.front();
  ASSERT_EQ(layer.geometries.size(), 2u);

  const ParsedGeometry* peak = nullptr;
  for(const auto& g : layer.geometries)
    if(g.attributes.value("assessment").toString() == QStringLiteral("candidate C"))
      peak = &g;
  ASSERT_NE(peak, nullptr);
  EXPECT_EQ(peak->type, ParsedGeometry::Point);
  EXPECT_DOUBLE_EQ(peak->attributes.value("analytic_signal_nT_per_m").toDouble(), 58.0);
  ASSERT_EQ(peak->exterior.size(), 1u);
  EXPECT_NEAR(peak->exterior.front().latitude(), 42.9925, 1e-6);
  EXPECT_NEAR(peak->exterior.front().longitude(), -71.4361, 1e-6);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
