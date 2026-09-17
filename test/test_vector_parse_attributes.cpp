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
#include <QStringList>
#include <QTemporaryDir>
#include <QtGlobal>

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

// [camp#22] A GeoPackage layer whose SRS is a LOCAL_CS — a coordinate system with
// no path to WGS84. Building a transformation from it fails, and the pre-fix
// parser fell silently through to the untransformed branch and read its
// coordinates as degrees.
QString writeUnprojectableSrsPackage(const QTemporaryDir& dir)
{
  GDALAllRegister();
  GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("GPKG");
  if(!driver)
    return QString();
  const QString path = dir.filePath("local_cs.gpkg");
  GDALDataset* ds = driver->Create(path.toUtf8().constData(), 0, 0, 0, GDT_Unknown, nullptr);
  if(!ds)
    return QString();

  OGRSpatialReference srs;
  if(srs.SetFromUserInput("LOCAL_CS[\"shipboard\",UNIT[\"metre\",1.0]]") != OGRERR_NONE)
  {
    GDALClose(ds);
    return QString();
  }
  OGRLayer* layer = ds->CreateLayer("local", &srs, wkbUnknown, nullptr);
  if(!layer)
  {
    GDALClose(ds);
    return QString();
  }
  OGRFeature* f = OGRFeature::CreateFeature(layer->GetLayerDefn());
  OGRPoint pt(350000.0, 4800000.0);   // projected metres, NOT degrees
  f->SetGeometry(&pt);
  layer->CreateFeature(f);
  OGRFeature::DestroyFeature(f);
  GDALClose(ds);
  return path;
}

// A GeoPackage layer with NO spatial reference at all — the untransformed branch,
// where x is longitude and y is latitude. Carries a line and a polygon, which is
// where the lat/lon swap this issue fixed used to live.
QString writeNoSrsPackage(const QTemporaryDir& dir)
{
  GDALAllRegister();
  GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("GPKG");
  if(!driver)
    return QString();
  const QString path = dir.filePath("no_srs.gpkg");
  GDALDataset* ds = driver->Create(path.toUtf8().constData(), 0, 0, 0, GDT_Unknown, nullptr);
  if(!ds)
    return QString();

  OGRLayer* layer = ds->CreateLayer("plain", nullptr, wkbUnknown, nullptr);
  if(!layer)
  {
    GDALClose(ds);
    return QString();
  }
  {
    OGRFieldDefn name("name", OFTString);
    layer->CreateField(&name);
  }
  OGRFeatureDefn* defn = layer->GetLayerDefn();
  {
    OGRFeature* f = OGRFeature::CreateFeature(defn);
    f->SetField("name", "line");
    OGRLineString ls;
    ls.addPoint(-70.80, 43.10);   // (x = longitude, y = latitude)
    ls.addPoint(-70.60, 43.20);
    f->SetGeometry(&ls);
    layer->CreateFeature(f);
    OGRFeature::DestroyFeature(f);
  }
  {
    OGRFeature* f = OGRFeature::CreateFeature(defn);
    f->SetField("name", "poly");
    OGRPolygon poly;
    OGRLinearRing ring;
    ring.addPoint(-70.90, 43.00);
    ring.addPoint(-70.70, 43.00);
    ring.addPoint(-70.70, 43.30);
    ring.addPoint(-70.90, 43.30);
    ring.closeRings();
    poly.addRing(&ring);
    f->SetGeometry(&poly);
    layer->CreateFeature(f);
    OGRFeature::DestroyFeature(f);
  }
  GDALClose(ds);
  return path;
}

// A GeoPackage carrying a heterogeneous wkbGeometryCollection — what KML emits
// for a placemark that mixes a point with its outline, and what the parser used
// to warn-and-drop one line below the recursion that already handled it.
QString writeGeometryCollectionPackage(const QTemporaryDir& dir)
{
  GDALAllRegister();
  GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("GPKG");
  if(!driver)
    return QString();
  const QString path = dir.filePath("collection.gpkg");
  GDALDataset* ds = driver->Create(path.toUtf8().constData(), 0, 0, 0, GDT_Unknown, nullptr);
  if(!ds)
    return QString();

  OGRSpatialReference srs;
  srs.SetWellKnownGeogCS("WGS84");
  srs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
  OGRLayer* layer = ds->CreateLayer("mixed", &srs, wkbUnknown, nullptr);
  if(!layer)
  {
    GDALClose(ds);
    return QString();
  }
  {
    OGRFieldDefn name("name", OFTString);
    layer->CreateField(&name);
  }
  OGRFeature* f = OGRFeature::CreateFeature(layer->GetLayerDefn());
  f->SetField("name", "placemark");

  OGRGeometryCollection collection;
  OGRPoint pt(-70.71, 43.07);
  collection.addGeometry(&pt);
  OGRLineString ls;
  ls.addPoint(-70.70, 43.06);
  ls.addPoint(-70.69, 43.05);
  collection.addGeometry(&ls);
  OGRPolygon poly;
  OGRLinearRing ring;
  ring.addPoint(-70.80, 43.00);
  ring.addPoint(-70.60, 43.00);
  ring.addPoint(-70.60, 43.20);
  ring.addPoint(-70.80, 43.20);
  ring.closeRings();
  poly.addRing(&ring);
  collection.addGeometry(&poly);
  // A nested collection: the recursion has to reach through it too.
  OGRGeometryCollection nested;
  OGRPoint nested_pt(-70.65, 43.02);
  nested.addGeometry(&nested_pt);
  collection.addGeometry(&nested);

  f->SetGeometry(&collection);
  layer->CreateFeature(f);
  OGRFeature::DestroyFeature(f);
  GDALClose(ds);
  return path;
}

// [camp#22 round-4] A GeoPackage holding ONE feature whose single LineString has
// enough vertices to reach readRing()'s in-loop abort poll (kVertexPollInterval is
// 1024). It is the fixture for the interaction between the vertex-level
// truncation and the per-feature geometry cap: the feature both crosses a cap of
// one AND carries a ring the abort cut short, which is the case where the two
// feature-boundary returns disagree about what the result is.
constexpr int kTruncatableRingVertices = 4000;

QString writeTruncatableRingPackage(const QTemporaryDir& dir)
{
  GDALAllRegister();
  GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("GPKG");
  if(!driver)
    return QString();
  const QString path = dir.filePath("long_ring.gpkg");
  GDALDataset* ds = driver->Create(path.toUtf8().constData(), 0, 0, 0, GDT_Unknown, nullptr);
  if(!ds)
    return QString();

  OGRSpatialReference srs;
  srs.SetWellKnownGeogCS("WGS84");
  srs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
  OGRLayer* layer = ds->CreateLayer("long_ring", &srs, wkbUnknown, nullptr);
  if(!layer)
  {
    GDALClose(ds);
    return QString();
  }

  OGRFeature* f = OGRFeature::CreateFeature(layer->GetLayerDefn());
  OGRLineString ls;
  for(int i = 0; i < kTruncatableRingVertices; ++i)
    ls.addPoint(-70.80 + i * 1e-5, 43.00 + i * 1e-5);
  f->SetGeometry(&ls);
  layer->CreateFeature(f);
  OGRFeature::DestroyFeature(f);
  GDALClose(ds);
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

// [camp#22 must-fix 9] The lat/lon convention, asserted on a LINE and a POLYGON
// vertex — the geometries the swap actually affected.
//
// readRing() used to build QGeoCoordinate(getX(), getY()) on BOTH branches while
// the Point path built (getY(), getX()), so line and polygon vertices from a
// source with no spatial reference came out transposed while points from the same
// file did not — a 43N/70W survey line drawn through the Indian Ocean. Points were
// covered by the existing tests; nothing pinned the geometries that were wrong.

// UNTRANSFORMED branch: a layer with no SRS, where x is longitude, y is latitude.
TEST(VectorParseAttributes, LineAndPolygonVerticesAreLatLonWithoutAnSrs)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString path = writeNoSrsPackage(dir);
  ASSERT_FALSE(path.isEmpty());

  DatasetPtr dataset = openDataset(path);
  ASSERT_TRUE(dataset);
  const std::vector<ParsedLayer> layers = camp::vector::parseVectorLayers(dataset.get());
  ASSERT_EQ(layers.size(), 1u);
  const ParsedLayer& layer = layers.front();

  const ParsedGeometry* line = firstNamed(layer, "line");
  ASSERT_NE(line, nullptr);
  ASSERT_EQ(line->type, ParsedGeometry::LineString);
  ASSERT_EQ(line->exterior.size(), 2u);
  EXPECT_NEAR(line->exterior.front().latitude(), 43.10, 1e-6);
  EXPECT_NEAR(line->exterior.front().longitude(), -70.80, 1e-6);
  EXPECT_NEAR(line->exterior.back().latitude(), 43.20, 1e-6);
  EXPECT_NEAR(line->exterior.back().longitude(), -70.60, 1e-6);

  const ParsedGeometry* poly = firstNamed(layer, "poly");
  ASSERT_NE(poly, nullptr);
  ASSERT_EQ(poly->type, ParsedGeometry::Polygon);
  ASSERT_GE(poly->exterior.size(), 4u);
  EXPECT_NEAR(poly->exterior.front().latitude(), 43.00, 1e-6);
  EXPECT_NEAR(poly->exterior.front().longitude(), -70.90, 1e-6);
  for(const auto& vertex : poly->exterior)
  {
    EXPECT_NEAR(vertex.latitude(), 43.15, 0.16);
    EXPECT_NEAR(vertex.longitude(), -70.80, 0.11);
  }
}

// TRANSFORMED branch: the WGS84 GeoPackage, whose target SRS comes back in
// authority (latitude, longitude) order.
TEST(VectorParseAttributes, LineAndPolygonVerticesAreLatLonWhenTransformed)
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

  // The multiline parts run from (-70.60, 43.30) east/north in 0.01 steps.
  const ParsedGeometry* line = firstNamed(layer, "multiline");
  ASSERT_NE(line, nullptr);
  ASSERT_EQ(line->type, ParsedGeometry::LineString);
  ASSERT_EQ(line->exterior.size(), 3u);
  for(const auto& vertex : line->exterior)
  {
    EXPECT_NEAR(vertex.latitude(), 43.31, 0.02);
    EXPECT_NEAR(vertex.longitude(), -70.55, 0.07);
  }

  // The multipoly parts are 0.2-degree boxes anchored near (-70.80, 43.00).
  const ParsedGeometry* poly = firstNamed(layer, "multipoly");
  ASSERT_NE(poly, nullptr);
  ASSERT_EQ(poly->type, ParsedGeometry::Polygon);
  ASSERT_GE(poly->exterior.size(), 4u);
  for(const auto& vertex : poly->exterior)
  {
    EXPECT_NEAR(vertex.latitude(), 43.10, 0.11);
    EXPECT_NEAR(vertex.longitude(), -70.45, 0.36);
  }
  ASSERT_EQ(poly->interiorRings.size(), 1u);
  for(const auto& vertex : poly->interiorRings.front())
  {
    EXPECT_NEAR(vertex.latitude(), 43.08, 0.03);
    EXPECT_NEAR(vertex.longitude(), -70.73, 0.02);
  }
}

// [camp#22 must-fix 5] A heterogeneous wkbGeometryCollection goes through the SAME
// recursion as the Multi* collections — including a nested collection — instead of
// being warn-and-dropped one line below the code that already handled it.
TEST(VectorParseAttributes, GeometryCollectionIsNotDropped)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString path = writeGeometryCollectionPackage(dir);
  ASSERT_FALSE(path.isEmpty());

  DatasetPtr dataset = openDataset(path);
  ASSERT_TRUE(dataset);
  camp::vector::ParseDiagnostics diagnostics;
  const std::vector<ParsedLayer> layers =
    camp::vector::parseVectorLayers(dataset.get(), camp::vector::ParseOptions(), &diagnostics);
  ASSERT_EQ(layers.size(), 1u);
  const ParsedLayer& layer = layers.front();

  // Point, line, polygon, and the point inside the nested collection.
  ASSERT_EQ(layer.geometries.size(), 4u);
  EXPECT_EQ(diagnostics.geometries_unhandled, 0);

  int points = 0, lines = 0, polygons = 0;
  for(const auto& g : layer.geometries)
  {
    // Every part carries the parent feature's attributes.
    EXPECT_EQ(g.attributes.value("name").toString(), QStringLiteral("placemark"));
    switch(g.type)
    {
    case ParsedGeometry::Point: ++points; break;
    case ParsedGeometry::LineString: ++lines; break;
    case ParsedGeometry::Polygon: ++polygons; break;
    }
  }
  EXPECT_EQ(points, 2);
  EXPECT_EQ(lines, 1);
  EXPECT_EQ(polygons, 1);
}

// [camp#22 must-fix 3] A layer that HAS a spatial reference but from which no
// transformation to WGS84 can be built is FAILED and reported — never quietly read
// as if its projected metres were degrees.
TEST(VectorParseAttributes, LayerWithUnprojectableSrsFailsRatherThanFallingThrough)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString path = writeUnprojectableSrsPackage(dir);
  ASSERT_FALSE(path.isEmpty());

  DatasetPtr dataset = openDataset(path);
  ASSERT_TRUE(dataset);
  ASSERT_GE(dataset->GetLayerCount(), 1);
  ASSERT_NE(dataset->GetLayer(0)->GetSpatialRef(), nullptr)
      << "fixture must declare an SRS, or it exercises the untransformed branch";

  camp::vector::ParseDiagnostics diagnostics;
  const std::vector<ParsedLayer> layers =
    camp::vector::parseVectorLayers(dataset.get(), camp::vector::ParseOptions(), &diagnostics);

  EXPECT_EQ(diagnostics.layers_total, 1);
  EXPECT_EQ(diagnostics.layers_failed, 1);
  EXPECT_TRUE(layers.empty()) << "a failed layer must emit no geometry at all";
}

// [camp#22 must-fix 6] The abort predicate is polled INSIDE the parse, so a
// destructor that cancels a load is joined in feature time rather than file time.
TEST(VectorParseAttributes, AbortStopsTheParseAndIsReported)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString path = writeGeoPackage(dir);
  ASSERT_FALSE(path.isEmpty());

  // Baseline: the whole file.
  DatasetPtr dataset = openDataset(path);
  ASSERT_TRUE(dataset);
  camp::vector::ParseDiagnostics full_diag;
  const std::vector<ParsedLayer> full =
    camp::vector::parseVectorLayers(dataset.get(), camp::vector::ParseOptions(), &full_diag);
  ASSERT_EQ(full.size(), 1u);
  const size_t full_count = full.front().geometries.size();
  ASSERT_GT(full_count, 2u);
  EXPECT_FALSE(full_diag.aborted);

  // Aborting before the first layer returns nothing at all.
  DatasetPtr immediate_ds = openDataset(path);
  ASSERT_TRUE(immediate_ds);
  camp::vector::ParseOptions immediate;
  immediate.aborted = []() { return true; };
  camp::vector::ParseDiagnostics immediate_diag;
  const std::vector<ParsedLayer> nothing =
    camp::vector::parseVectorLayers(immediate_ds.get(), immediate, &immediate_diag);
  EXPECT_TRUE(nothing.empty());
  EXPECT_TRUE(immediate_diag.aborted);

  // Aborting after the first feature stops there — it does NOT read the rest of
  // the file, which is the property the destructor's bounded join depends on.
  DatasetPtr partial_ds = openDataset(path);
  ASSERT_TRUE(partial_ds);
  int polls = 0;
  camp::vector::ParseOptions after_one;
  after_one.aborted = [&polls]() { return ++polls > 2; };
  camp::vector::ParseDiagnostics partial_diag;
  const std::vector<ParsedLayer> partial =
    camp::vector::parseVectorLayers(partial_ds.get(), after_one, &partial_diag);
  EXPECT_TRUE(partial_diag.aborted);
  ASSERT_EQ(partial.size(), 1u);
  EXPECT_LT(partial.front().geometries.size(), full_count)
      << "the abort flag must be read inside the feature loop, not only before it";
}

// [camp#22 round-2 must-fix] The feature cap is applied INSIDE the parse, so the
// rest of the file is never read.
//
// The cap started as a bound on GUI-thread item construction, applied to the
// parser's RESULT — which meant the worker had already built every geometry and
// every attribute map of the whole file before anything was capped. For the case
// the cap is documented for (a national coastline shapefile) that is the OOM, not
// the freeze. Stopping the parse is what bounds the memory, and the way to prove
// it from outside is that the returned count is exactly the cap on a file that
// holds more.
TEST(VectorParseAttributes, GeometryCapStopsTheParse)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString path = writeGeoPackage(dir);
  ASSERT_FALSE(path.isEmpty());

  // Baseline: how many geometries the fixture actually holds.
  DatasetPtr dataset = openDataset(path);
  ASSERT_TRUE(dataset);
  camp::vector::ParseDiagnostics full_diag;
  const std::vector<ParsedLayer> full =
    camp::vector::parseVectorLayers(dataset.get(), camp::vector::ParseOptions(), &full_diag);
  ASSERT_EQ(full.size(), 1u);
  const size_t full_count = full.front().geometries.size();
  ASSERT_GT(full_count, 2u) << "fixture must hold more geometries than the cap below";
  EXPECT_FALSE(full_diag.geometry_cap_reached);

  // Capped at two: exactly two come back, and the cap is reported. A multi-part
  // feature can carry the running total past the cap, which is why the parser
  // trims rather than returning "the cap, give or take a feature".
  DatasetPtr capped_ds = openDataset(path);
  ASSERT_TRUE(capped_ds);
  camp::vector::ParseOptions capped;
  capped.max_geometries = 2;
  camp::vector::ParseDiagnostics capped_diag;
  const std::vector<ParsedLayer> layers =
    camp::vector::parseVectorLayers(capped_ds.get(), capped, &capped_diag);
  ASSERT_EQ(layers.size(), 1u);
  EXPECT_EQ(layers.front().geometries.size(), 2u);
  EXPECT_TRUE(capped_diag.geometry_cap_reached);
  EXPECT_FALSE(capped_diag.aborted) << "the cap is not an abort; they are reported apart";

  // A cap at or above the file's size changes nothing and is not reported as hit
  // unless it is actually reached.
  DatasetPtr generous_ds = openDataset(path);
  ASSERT_TRUE(generous_ds);
  camp::vector::ParseOptions generous;
  generous.max_geometries = static_cast<int>(full_count) + 1;
  camp::vector::ParseDiagnostics generous_diag;
  const std::vector<ParsedLayer> all =
    camp::vector::parseVectorLayers(generous_ds.get(), generous, &generous_diag);
  ASSERT_EQ(all.size(), 1u);
  EXPECT_EQ(all.front().geometries.size(), full_count);
  EXPECT_FALSE(generous_diag.geometry_cap_reached);
}

// [camp#22 round-4 should-fix] REPROJECTION from a projected CRS.
//
// Every other fixture with a spatial reference is WGS84 -> WGS84 (the one
// exception, LOCAL_CS, tests transform FAILURE), so the transform path was only
// ever exercised for target-axis order — a broken projected->WGS84 transform
// would have passed the whole suite. Reading UTM survey data is a must-have of
// this layer, so it gets a fixture of its own: EPSG:32619 (UTM zone 19N, the zone
// covering the Gulf of Maine) eastings/northings, asserted back as the lat/lon
// they stand for.
QString writeUtmGeoPackage(const QTemporaryDir& dir)
{
  GDALAllRegister();
  GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("GPKG");
  if(!driver)
    return QString();
  const QString path = dir.filePath("utm19n.gpkg");
  GDALDataset* ds = driver->Create(path.toUtf8().constData(), 0, 0, 0, GDT_Unknown, nullptr);
  if(!ds)
    return QString();

  OGRSpatialReference srs;
  if(srs.importFromEPSG(32619) != OGRERR_NONE)
  {
    GDALClose(ds);
    return QString();
  }
  srs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);   // easting, northing

  OGRLayer* layer = ds->CreateLayer("survey", &srs, wkbUnknown, nullptr);
  if(!layer)
  {
    GDALClose(ds);
    return QString();
  }

  // (350000 E, 4769000 N) in zone 19N is off the New Hampshire coast, near the
  // Isles of Shoals: 43.0589 N, 70.8420 W (computed with the same PROJ this build
  // links). The second point is 1 km east.
  {
    OGRPoint p(350000.0, 4769000.0);
    OGRFeature* f = OGRFeature::CreateFeature(layer->GetLayerDefn());
    f->SetGeometry(&p);
    layer->CreateFeature(f);
    OGRFeature::DestroyFeature(f);
  }
  {
    OGRLineString ls;
    ls.addPoint(350000.0, 4769000.0);
    ls.addPoint(351000.0, 4769000.0);
    OGRFeature* f = OGRFeature::CreateFeature(layer->GetLayerDefn());
    f->SetGeometry(&ls);
    layer->CreateFeature(f);
    OGRFeature::DestroyFeature(f);
  }
  GDALClose(ds);
  return path;
}

// [camp#22 round-4 should-fix] An unhandled geometry type is reported ONCE PER
// LAYER, not once per geometry.
//
// An unhandled geometry does not spend the geometry budget, so max_geometries
// bounds nothing on this path: a large file of curve types could emit unbounded
// log I/O while the worker read all of it. The count already existed in
// ParseDiagnostics; the message now follows points_dropped's pattern and names
// the first type seen.
//
// [camp#22 round-7 must-fix] TWO layers, of two different unhandled types: the
// name each summary line carries must be that LAYER's first type, not the
// parse's. `first_unhandled_geometry_type` accumulates across the whole parse,
// so layer 2 used to be reported with layer 1's type beside its own per-layer
// count. The parse-wide field keeps its documented whole-parse meaning.
TEST(VectorParseAttributes, UnhandledGeometriesAreReportedOncePerLayer)
{
  GDALAllRegister();
  GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("Memory");
  ASSERT_NE(driver, nullptr);
  DatasetPtr ds(driver->Create("unhandled", 0, 0, 0, GDT_Unknown, nullptr), gdal_closer);
  ASSERT_TRUE(ds);

  OGRSpatialReference srs;
  srs.SetWellKnownGeogCS("WGS84");
  srs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

  // Layer 1: three circular strings — a type the parser documents as unhandled.
  OGRLayer* layer = ds->CreateLayer("curves", &srs, wkbUnknown, nullptr);
  ASSERT_NE(layer, nullptr);
  for(int i = 0; i < 3; ++i)
  {
    OGRCircularString curve;
    curve.addPoint(-70.80 + 0.01 * i, 43.00);
    curve.addPoint(-70.79 + 0.01 * i, 43.01);
    curve.addPoint(-70.78 + 0.01 * i, 43.00);
    OGRFeature* f = OGRFeature::CreateFeature(layer->GetLayerDefn());
    f->SetGeometry(&curve);
    ASSERT_EQ(layer->CreateFeature(f), OGRERR_NONE);
    OGRFeature::DestroyFeature(f);
  }

  // Layer 2: two compound curves — a DIFFERENT unhandled type, so the name in
  // this layer's line cannot be inherited from layer 1 without the test noticing.
  OGRLayer* second = ds->CreateLayer("compound", &srs, wkbUnknown, nullptr);
  ASSERT_NE(second, nullptr);
  for(int i = 0; i < 2; ++i)
  {
    OGRCircularString arc;
    arc.addPoint(-70.60 + 0.01 * i, 43.00);
    arc.addPoint(-70.59 + 0.01 * i, 43.01);
    arc.addPoint(-70.58 + 0.01 * i, 43.00);
    OGRCompoundCurve curve;
    ASSERT_EQ(curve.addCurve(&arc), OGRERR_NONE);
    OGRFeature* f = OGRFeature::CreateFeature(second->GetLayerDefn());
    f->SetGeometry(&curve);
    ASSERT_EQ(second->CreateFeature(f), OGRERR_NONE);
    OGRFeature::DestroyFeature(f);
  }

  // The type names as OGR spells them — asserted against the library rather than
  // hard-coded, so a GDAL wording change is not a test failure.
  const QString circular_name = QString::fromUtf8(OGRGeometryTypeToName(wkbCircularString));
  const QString compound_name = QString::fromUtf8(OGRGeometryTypeToName(wkbCompoundCurve));
  ASSERT_FALSE(circular_name.isEmpty());
  ASSERT_FALSE(compound_name.isEmpty());
  ASSERT_NE(circular_name, compound_name) << "the two layers must differ in type name";

  // Collect the warnings the parse emits about them.
  static QStringList unhandled_warnings;
  unhandled_warnings.clear();
  QtMessageHandler previous = qInstallMessageHandler(
      [](QtMsgType, const QMessageLogContext&, const QString& message)
      {
        if(message.contains("unhandled type"))
          unhandled_warnings.append(message);
      });

  camp::vector::ParseDiagnostics diag;
  const std::vector<ParsedLayer> layers =
    camp::vector::parseVectorLayers(ds.get(), camp::vector::ParseOptions(), &diag);
  qInstallMessageHandler(previous);

  ASSERT_EQ(layers.size(), 2u);
  EXPECT_TRUE(layers.front().geometries.empty()) << "a curve type is not drawn";
  EXPECT_TRUE(layers.back().geometries.empty());
  EXPECT_EQ(diag.geometries_unhandled, 5) << "every skipped geometry is still COUNTED";
  EXPECT_EQ(diag.first_unhandled_geometry_type, circular_name)
      << "the field is PARSE-wide: the first type the whole parse saw";

  ASSERT_EQ(unhandled_warnings.size(), 2)
      << "one summary line per layer, not one per geometry: "
      << unhandled_warnings.join(" | ").toStdString();

  const QString& first_line = unhandled_warnings.at(0);
  const QString& second_line = unhandled_warnings.at(1);
  EXPECT_TRUE(first_line.contains("curves")) << first_line.toStdString();
  EXPECT_TRUE(first_line.contains("3")) << "layer 1 skipped three: " << first_line.toStdString();
  EXPECT_TRUE(first_line.contains(circular_name)) << first_line.toStdString();

  EXPECT_TRUE(second_line.contains("compound")) << second_line.toStdString();
  EXPECT_TRUE(second_line.contains("2")) << "layer 2 skipped two: " << second_line.toStdString();
  EXPECT_TRUE(second_line.contains(compound_name))
      << "layer 2 must name ITS OWN first type, not layer 1's: " << second_line.toStdString();
  EXPECT_FALSE(second_line.contains(circular_name))
      << "layer 1's type leaked into layer 2's line: " << second_line.toStdString();
}

// Projected metres in, latitude/longitude out — the UTM survey file this layer
// exists to read.
TEST(VectorParseAttributes, ProjectedCoordinatesAreReprojectedToLatLon)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString path = writeUtmGeoPackage(dir);
  ASSERT_FALSE(path.isEmpty()) << "could not build the EPSG:32619 fixture";

  DatasetPtr dataset = openDataset(path);
  ASSERT_TRUE(dataset);
  camp::vector::ParseDiagnostics diag;
  const std::vector<ParsedLayer> layers =
    camp::vector::parseVectorLayers(dataset.get(), camp::vector::ParseOptions(), &diag);
  ASSERT_EQ(layers.size(), 1u);
  EXPECT_EQ(diag.layers_failed, 0) << "a transformation to WGS84 must be available for UTM 19N";
  EXPECT_EQ(diag.points_dropped, 0);

  const ParsedLayer& layer = layers.front();
  ASSERT_EQ(layer.geometries.size(), 2u);

  // Tight tolerances: this is the assertion that would fail if the eastings were
  // passed through as degrees, or the axis order of the target were wrong.
  const ParsedGeometry& point = layer.geometries.front();
  ASSERT_EQ(point.type, ParsedGeometry::Point);
  ASSERT_EQ(point.exterior.size(), 1u);
  EXPECT_NEAR(point.exterior.front().latitude(), 43.05888, 0.0005);
  EXPECT_NEAR(point.exterior.front().longitude(), -70.84204, 0.0005);

  const ParsedGeometry& line = layer.geometries.back();
  ASSERT_EQ(line.type, ParsedGeometry::LineString);
  ASSERT_EQ(line.exterior.size(), 2u);
  EXPECT_NEAR(line.exterior.front().latitude(), 43.05888, 0.0005);
  EXPECT_NEAR(line.exterior.front().longitude(), -70.84204, 0.0005);
  // 1 km east is about 0.0123 degrees of longitude at this latitude, and the
  // latitude barely moves — a vertex read in the wrong axis order or left in
  // metres cannot satisfy both.
  EXPECT_NEAR(line.exterior.back().latitude(), 43.05908, 0.0005);
  EXPECT_NEAR(line.exterior.back().longitude(), -70.82977, 0.0005);
  EXPECT_GT(line.exterior.back().longitude(), line.exterior.front().longitude())
      << "the second vertex is EAST of the first";
}

// [camp#22 round-3 should-fix] `geometry_cap_reached` means "the rest of the file
// was NOT READ" — the sentence VectorLayer puts in the Layers tab.
//
// It used to be set the moment the running total reached max_geometries, without
// establishing that anything remained, so a file holding EXACTLY max_geometries
// geometries reported a partial read of a file it had read in full — a false
// claim in the one status line this design leans on.
TEST(VectorParseAttributes, CapAtExactlyTheFileSizeIsNotAPartialRead)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString path = writeGeoPackage(dir);
  ASSERT_FALSE(path.isEmpty());

  // How many geometries the fixture holds.
  DatasetPtr baseline_ds = openDataset(path);
  ASSERT_TRUE(baseline_ds);
  const std::vector<ParsedLayer> full =
    camp::vector::parseVectorLayers(baseline_ds.get(), camp::vector::ParseOptions(), nullptr);
  ASSERT_EQ(full.size(), 1u);
  const size_t full_count = full.front().geometries.size();
  ASSERT_GT(full_count, 1u);

  // Cap set to EXACTLY that: every geometry comes back and nothing was left.
  DatasetPtr exact_ds = openDataset(path);
  ASSERT_TRUE(exact_ds);
  camp::vector::ParseOptions exact;
  exact.max_geometries = static_cast<int>(full_count);
  camp::vector::ParseDiagnostics exact_diag;
  const std::vector<ParsedLayer> all =
    camp::vector::parseVectorLayers(exact_ds.get(), exact, &exact_diag);
  ASSERT_EQ(all.size(), 1u);
  EXPECT_EQ(all.front().geometries.size(), full_count);
  EXPECT_FALSE(exact_diag.geometry_cap_reached)
      << "the file was read in full; claiming an unread remainder is a false partial read";

  // One below: the same cap branch, but a geometry really is left unread.
  DatasetPtr short_ds = openDataset(path);
  ASSERT_TRUE(short_ds);
  camp::vector::ParseOptions one_short;
  one_short.max_geometries = static_cast<int>(full_count) - 1;
  camp::vector::ParseDiagnostics short_diag;
  const std::vector<ParsedLayer> most =
    camp::vector::parseVectorLayers(short_ds.get(), one_short, &short_diag);
  ASSERT_EQ(most.size(), 1u);
  EXPECT_EQ(most.front().geometries.size(), full_count - 1);
  EXPECT_TRUE(short_diag.geometry_cap_reached) << "input remains, so the cap IS a partial read";
}

// The same rule INSIDE one multi-part feature: a cap that falls on the feature's
// last part left nothing unread, while one that falls mid-feature did.
TEST(VectorParseAttributes, CapInsideAFeatureReportsOnlyAnActualRemainder)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString path = writeGeometryCollectionPackage(dir);   // ONE feature, four parts
  ASSERT_FALSE(path.isEmpty());

  DatasetPtr mid_ds = openDataset(path);
  ASSERT_TRUE(mid_ds);
  camp::vector::ParseOptions mid;
  mid.max_geometries = 2;
  camp::vector::ParseDiagnostics mid_diag;
  camp::vector::parseVectorLayers(mid_ds.get(), mid, &mid_diag);
  EXPECT_TRUE(mid_diag.geometry_cap_reached) << "two parts of the feature went unread";

  DatasetPtr exact_ds = openDataset(path);
  ASSERT_TRUE(exact_ds);
  camp::vector::ParseOptions exact;
  exact.max_geometries = 4;
  camp::vector::ParseDiagnostics exact_diag;
  camp::vector::parseVectorLayers(exact_ds.get(), exact, &exact_diag);
  EXPECT_FALSE(exact_diag.geometry_cap_reached)
      << "the cap fell on the feature's last part; nothing was left unread";
}

// [camp#22 round-7 suggestion] The lookahead's MULTI-LAYER branch: the cap falls
// on the last feature of layer 0, so whether anything was left unread is decided
// entirely by what the remaining layers hold. Every other cap test uses a
// single-layer fixture, which never reaches this loop.
TEST(VectorParseAttributes, CapOnALayerBoundaryLooksAtTheRemainingLayers)
{
  GDALAllRegister();
  GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("Memory");
  ASSERT_NE(driver, nullptr);

  OGRSpatialReference srs;
  srs.SetWellKnownGeogCS("WGS84");
  srs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

  // Two layers: `first` always holds two points; `second` holds `tail_features`.
  auto build = [&driver, &srs](const char* name, int tail_features)
  {
    DatasetPtr ds(driver->Create(name, 0, 0, 0, GDT_Unknown, nullptr), gdal_closer);
    if(!ds)
      return ds;
    auto addPoints = [](OGRLayer* layer, int count, double base)
    {
      for(int i = 0; i < count; ++i)
      {
        OGRPoint pt(base + 0.01 * i, 43.00);
        OGRFeature* f = OGRFeature::CreateFeature(layer->GetLayerDefn());
        f->SetGeometry(&pt);
        layer->CreateFeature(f);
        OGRFeature::DestroyFeature(f);
      }
    };
    addPoints(ds->CreateLayer("first", &srs, wkbUnknown, nullptr), 2, -70.80);
    // The second layer EXISTS in both cases — an empty layer is not the same
    // thing as no layer, and it is the empty one the flag used to lie about.
    addPoints(ds->CreateLayer("second", &srs, wkbUnknown, nullptr), tail_features, -70.60);
    return ds;
  };

  camp::vector::ParseOptions capped;
  capped.max_geometries = 2;   // exactly what layer `first` holds

  // Trailing layer EMPTY: the cap fell on the last geometry of the file, so
  // nothing was left unread.
  DatasetPtr empty_tail = build("empty_tail", 0);
  ASSERT_TRUE(empty_tail);
  camp::vector::ParseDiagnostics empty_diag;
  const std::vector<ParsedLayer> stopped_at_end =
    camp::vector::parseVectorLayers(empty_tail.get(), capped, &empty_diag);
  ASSERT_EQ(stopped_at_end.size(), 1u) << "the parse returns at the cap, before layer 2";
  EXPECT_EQ(stopped_at_end.front().geometries.size(), 2u);
  EXPECT_FALSE(empty_diag.geometry_cap_reached)
      << "the remaining layer is empty; claiming an unread remainder is a false partial read";

  // Trailing layer holding ONE feature: that feature is the unread remainder,
  // and only the cross-layer lookahead can see it.
  DatasetPtr one_tail = build("one_tail", 1);
  ASSERT_TRUE(one_tail);
  camp::vector::ParseDiagnostics one_diag;
  const std::vector<ParsedLayer> stopped_short =
    camp::vector::parseVectorLayers(one_tail.get(), capped, &one_diag);
  ASSERT_EQ(stopped_short.size(), 1u);
  EXPECT_EQ(stopped_short.front().geometries.size(), 2u);
  EXPECT_TRUE(one_diag.geometry_cap_reached)
      << "a feature in a LATER layer went unread, so the cap IS a partial read";
}

// [camp#22 round-3 should-fix] A supplied ParseDiagnostics describes THIS parse.
//
// The header documents the parameter as "filled in with what was skipped and
// why", but every field is accumulated into, so reusing one object across two
// parses used to carry `geometry_cap_reached`, `aborted`, `layers_total` and
// every counter forward — an uncapped parse reported as capped. The parser now
// resets a supplied object before binding to it.
TEST(VectorParseAttributes, SuppliedDiagnosticsAreResetPerParse)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString path = writeGeoPackage(dir);
  ASSERT_FALSE(path.isEmpty());

  camp::vector::ParseDiagnostics diag;

  // First parse: capped, so the flag and the counters are set.
  DatasetPtr capped_ds = openDataset(path);
  ASSERT_TRUE(capped_ds);
  camp::vector::ParseOptions capped;
  capped.max_geometries = 2;
  camp::vector::parseVectorLayers(capped_ds.get(), capped, &diag);
  ASSERT_TRUE(diag.geometry_cap_reached);
  ASSERT_EQ(diag.layers_total, 1);

  // Second parse of the same file with NO cap, through the SAME object.
  DatasetPtr full_ds = openDataset(path);
  ASSERT_TRUE(full_ds);
  camp::vector::parseVectorLayers(full_ds.get(), camp::vector::ParseOptions(), &diag);
  EXPECT_FALSE(diag.geometry_cap_reached) << "an uncapped parse must not report the previous cap";
  EXPECT_FALSE(diag.aborted);
  EXPECT_EQ(diag.layers_total, 1) << "layers_total must count this parse, not both";
}

// [camp#22 suggestion] The cap and the abort predicate reach INSIDE a multi-part
// feature, not only between features.
//
// appendGeometry() recurses through every part of a Multi* / GeometryCollection
// (and every nested collection) before returning to the per-feature poll, so a
// single huge MultiPolygon used to run to its end whatever the cap or the abort
// flag said. Memory overshoot was bounded by one feature and trimmed afterwards;
// ABORT LATENCY was unbounded — and bounding abort latency is the whole point of
// the predicate, since VectorLayer's destructor joins this worker on the GUI
// thread. The budget is now threaded through the recursion.
//
// The fixture is one feature holding four parts (point, line, polygon, nested
// collection), so every assertion below is about stopping mid-feature: between
// features there is nowhere to stop.
TEST(VectorParseAttributes, CapAndAbortStopInsideAMultiPartFeature)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString path = writeGeometryCollectionPackage(dir);
  ASSERT_FALSE(path.isEmpty());

  DatasetPtr full_ds = openDataset(path);
  ASSERT_TRUE(full_ds);
  const std::vector<ParsedLayer> full =
    camp::vector::parseVectorLayers(full_ds.get(), camp::vector::ParseOptions(), nullptr);
  ASSERT_EQ(full.size(), 1u);
  ASSERT_EQ(full.front().geometries.size(), 4u)
      << "fixture must be ONE feature of four parts for this test to mean anything";

  // The cap falls in the middle of the feature's parts.
  DatasetPtr capped_ds = openDataset(path);
  ASSERT_TRUE(capped_ds);
  camp::vector::ParseOptions capped;
  capped.max_geometries = 2;
  camp::vector::ParseDiagnostics capped_diag;
  const std::vector<ParsedLayer> layers =
    camp::vector::parseVectorLayers(capped_ds.get(), capped, &capped_diag);
  ASSERT_EQ(layers.size(), 1u);
  EXPECT_EQ(layers.front().geometries.size(), 2u);
  EXPECT_TRUE(capped_diag.geometry_cap_reached);

  // Abort raised before the parse reaches the feature's second part: the
  // remaining parts are not built at all. Counting the polls is what distinguishes
  // "stopped inside the feature" from "ran the feature and stopped after it" —
  // without the budget the predicate is asked once per feature, so a four-part
  // file would report a single poll and four geometries.
  DatasetPtr aborting_ds = openDataset(path);
  ASSERT_TRUE(aborting_ds);
  int polls = 0;
  camp::vector::ParseOptions aborting;
  aborting.aborted = [&polls]() { return ++polls > 2; };
  camp::vector::ParseDiagnostics abort_diag;
  const std::vector<ParsedLayer> partial =
    camp::vector::parseVectorLayers(aborting_ds.get(), aborting, &abort_diag);
  EXPECT_TRUE(abort_diag.aborted);
  size_t emitted = 0;
  for(const ParsedLayer& layer : partial)
    emitted += layer.geometries.size();
  EXPECT_LT(emitted, 4u) << "the abort must stop the feature part way, not after it";
  EXPECT_GT(polls, 2) << "the predicate must be polled inside the feature's parts";
}

// [camp#22 round-4 must-fix] A parse that is BOTH aborted mid-ring and at its
// geometry cap must report `aborted`.
//
// The vertex-level abort poll inside readRing() TRUNCATES the ring it is
// building. The only thing that keeps a truncated ring off the screen is
// ParseDiagnostics::aborted, which VectorLayer::loadFinished() discards the whole
// result on. The per-feature cap check used to run FIRST and return from its own
// branch, so a feature that crossed `max_geometries` and carried a ring the abort
// had just cut short came back with `geometry_cap_reached` set and `aborted`
// UNSET — a partial shape presented to the caller as a complete one, outside the
// discard net.
//
// The fixture is one feature of one LineString with 4000 vertices, so the ONLY
// place the abort can be honoured is the in-loop vertex poll, and emitting that
// one geometry is exactly what reaches a cap of one: the two returns are forced
// to disagree. Asserting the ring came back truncated (neither empty nor whole)
// is what proves the abort fired mid-ring rather than at one of the coarser poll
// sites before it.
TEST(VectorParseAttributes, AbortMidRingAtTheCapIsStillReportedAsAborted)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString path = writeTruncatableRingPackage(dir);
  ASSERT_FALSE(path.isEmpty());

  // Baseline: the whole ring, uncapped and unaborted.
  DatasetPtr full_ds = openDataset(path);
  ASSERT_TRUE(full_ds);
  const std::vector<ParsedLayer> full =
    camp::vector::parseVectorLayers(full_ds.get(), camp::vector::ParseOptions(), nullptr);
  ASSERT_EQ(full.size(), 1u);
  ASSERT_EQ(full.front().geometries.size(), 1u);
  ASSERT_EQ(full.front().geometries.front().exterior.size(),
            static_cast<size_t>(kTruncatableRingVertices));

  // Poll sites reached before the first in-loop vertex poll, in order: the
  // per-layer check, appendGeometry()'s budget check, readRing()'s per-ring
  // check. Firing on the fourth call therefore fires INSIDE the vertex loop.
  DatasetPtr ds = openDataset(path);
  ASSERT_TRUE(ds);
  int polls = 0;
  camp::vector::ParseOptions options;
  options.max_geometries = 1;
  options.aborted = [&polls]() { return ++polls > 3; };
  camp::vector::ParseDiagnostics diag;
  const std::vector<ParsedLayer> layers =
    camp::vector::parseVectorLayers(ds.get(), options, &diag);

  ASSERT_EQ(layers.size(), 1u);
  ASSERT_EQ(layers.front().geometries.size(), 1u);
  const size_t vertices = layers.front().geometries.front().exterior.size();
  EXPECT_GT(vertices, 0u) << "the abort fired before the vertex loop, not inside it";
  EXPECT_LT(vertices, static_cast<size_t>(kTruncatableRingVertices))
      << "the ring was not truncated, so this fixture does not exercise the interaction";

  // The assertion this test exists for: the truncated result is reported as
  // aborted, so loadFinished() discards it. Reaching the cap in the same feature
  // must not mask that.
  EXPECT_TRUE(diag.aborted)
      << "a parse truncated mid-ring must report aborted even when it also hit the cap";
}

// [camp#22] A polygon with no exterior ring has no outline to draw, so it is
// dropped — and COUNTED. Every ParseDiagnostics field exists so the caller can
// report what was deliberately left out; this one used to vanish silently.
TEST(VectorParseAttributes, PolygonWithoutExteriorRingIsCounted)
{
  GDALAllRegister();
  GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("Memory");
  ASSERT_NE(driver, nullptr) << "GDAL Memory driver is required for this test";
  DatasetPtr dataset(driver->Create("ringless", 0, 0, 0, GDT_Unknown, nullptr), gdal_closer);
  ASSERT_TRUE(dataset);

  OGRSpatialReference wgs84;
  wgs84.SetWellKnownGeogCS("WGS84");
  OGRLayer* layer = dataset->CreateLayer("shapes", &wgs84, wkbUnknown, nullptr);
  ASSERT_NE(layer, nullptr);

  // One empty polygon (no exterior ring — what a malformed or truncated source
  // produces) and one good one, so the count is provably the empty one alone.
  {
    OGRFeature feature(layer->GetLayerDefn());
    OGRPolygon empty;
    ASSERT_TRUE(empty.IsEmpty());
    feature.SetGeometry(&empty);
    ASSERT_EQ(layer->CreateFeature(&feature), OGRERR_NONE);
  }
  {
    OGRFeature feature(layer->GetLayerDefn());
    OGRLinearRing ring;
    ring.addPoint(-70.8, 43.0);
    ring.addPoint(-70.6, 43.0);
    ring.addPoint(-70.6, 43.2);
    ring.addPoint(-70.8, 43.0);
    OGRPolygon polygon;
    polygon.addRing(&ring);
    feature.SetGeometry(&polygon);
    ASSERT_EQ(layer->CreateFeature(&feature), OGRERR_NONE);
  }

  camp::vector::ParseDiagnostics diagnostics;
  const std::vector<ParsedLayer> layers =
    camp::vector::parseVectorLayers(dataset.get(), camp::vector::ParseOptions(), &diagnostics);
  ASSERT_EQ(layers.size(), 1u);
  EXPECT_EQ(layers.front().geometries.size(), 1u) << "only the ringed polygon is drawable";
  EXPECT_EQ(diagnostics.polygons_without_exterior_ring, 1);
  EXPECT_EQ(diagnostics.geometries_unhandled, 0) << "an empty polygon is dropped, not unhandled";
}

// [camp#22 round-5 should-fix] A LINE OR POLYGON WHOSE EXTERIOR LOSES EVERY
// VERTEX IS NOT EMITTED AND DOES NOT SPEND A CAP SLOT.
//
// readRing() drops each vertex whose transform fails; when it drops them ALL
// there is nothing left to draw, and the line/polygon branches used to push the
// empty geometry and spend() anyway. VectorLayer rejects the empty item
// (hasPlaceableCoordinate), so nothing wrong was ever DRAWN — the cost is that a
// run of out-of-domain features burns cap slots later valid features needed, and
// a mixed file can report the cap reached having produced fewer items than the
// cap. The wkbPoint branch already counted-and-dropped without spending.
//
// The fixture makes transforms FAIL for real rather than mocking them: an
// orthographic projection has no inverse outside the visible hemisphere, so a
// coordinate a thousand Earth radii out cannot come back as a latitude/longitude.
TEST(VectorParseAttributes, AllDroppedExteriorIsNeitherEmittedNorCharged)
{
  GDALAllRegister();
  GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("Memory");
  ASSERT_NE(driver, nullptr) << "GDAL Memory driver is required for this test";
  DatasetPtr dataset(driver->Create("outofdomain", 0, 0, 0, GDT_Unknown, nullptr), gdal_closer);
  ASSERT_TRUE(dataset);

  OGRSpatialReference ortho;
  ASSERT_EQ(ortho.SetFromUserInput(
              "+proj=ortho +lat_0=0 +lon_0=0 +datum=WGS84 +units=m +no_defs"),
            OGRERR_NONE);
  OGRLayer* layer = dataset->CreateLayer("shapes", &ortho, wkbUnknown, nullptr);
  ASSERT_NE(layer, nullptr);

  // Far outside the hemisphere the projection can represent: every vertex fails.
  constexpr double kOutOfDomain = 1.0e10;

  {  // a line whose three vertices all fail
    OGRFeature feature(layer->GetLayerDefn());
    OGRLineString line;
    line.addPoint(kOutOfDomain, kOutOfDomain);
    line.addPoint(kOutOfDomain + 10.0, kOutOfDomain);
    line.addPoint(kOutOfDomain + 20.0, kOutOfDomain + 10.0);
    feature.SetGeometry(&line);
    ASSERT_EQ(layer->CreateFeature(&feature), OGRERR_NONE);
  }
  {  // a polygon whose EXTERIOR all fails, carrying a hole that is ALSO out of
     // domain. [camp#22 round-8 suggestion] The hole used to be in domain, so its
     // vertices added nothing to points_dropped whether the parser read them or
     // not, and the property the early return exists for — the holes of a polygon
     // that cannot be drawn are never read — was untested. Out of domain, reading
     // them would show up as 4 more dropped points: 11 instead of 7.
    OGRFeature feature(layer->GetLayerDefn());
    OGRLinearRing exterior;
    exterior.addPoint(kOutOfDomain, kOutOfDomain);
    exterior.addPoint(kOutOfDomain + 100.0, kOutOfDomain);
    exterior.addPoint(kOutOfDomain + 100.0, kOutOfDomain + 100.0);
    exterior.addPoint(kOutOfDomain, kOutOfDomain);
    OGRLinearRing hole;
    hole.addPoint(kOutOfDomain + 20.0, kOutOfDomain + 20.0);
    hole.addPoint(kOutOfDomain + 40.0, kOutOfDomain + 20.0);
    hole.addPoint(kOutOfDomain + 40.0, kOutOfDomain + 40.0);
    hole.addPoint(kOutOfDomain + 20.0, kOutOfDomain + 20.0);
    OGRPolygon polygon;
    polygon.addRing(&exterior);
    polygon.addRing(&hole);
    feature.SetGeometry(&polygon);
    ASSERT_EQ(layer->CreateFeature(&feature), OGRERR_NONE);
  }
  for(int i = 0; i < 3; ++i)
  {  // three points well inside the domain
    OGRFeature feature(layer->GetLayerDefn());
    OGRPoint point(1000.0 * (i + 1), 2000.0);
    feature.SetGeometry(&point);
    ASSERT_EQ(layer->CreateFeature(&feature), OGRERR_NONE);
  }

  // Uncapped: only the three points come back, and the dropped vertices are
  // counted. The hole's vertices are NOT among them — the polygon branch returns
  // before reading the interior rings of a polygon it cannot draw.
  camp::vector::ParseDiagnostics diagnostics;
  const std::vector<ParsedLayer> layers =
    camp::vector::parseVectorLayers(dataset.get(), camp::vector::ParseOptions(), &diagnostics);
  ASSERT_EQ(layers.size(), 1u);
  ASSERT_EQ(layers.front().geometries.size(), 3u)
      << "an exterior with no surviving vertex must not be emitted";
  for(const ParsedGeometry& geometry : layers.front().geometries)
    EXPECT_EQ(geometry.type, ParsedGeometry::Point);
  EXPECT_EQ(diagnostics.points_dropped, 7)
      << "3 line vertices + 4 exterior-ring vertices. The hole's 4 vertices are out of "
         "domain too, so reading them would make this 11 — 7 is what proves the polygon "
         "branch returns before the interior rings of a polygon it cannot draw";
  EXPECT_EQ(diagnostics.polygons_without_exterior_ring, 0)
      << "the polygon HAS an exterior ring; its vertices are what could not be transformed";
  // [camp#22 round-8 must-fix] The skip has its OWN counter, because nothing else
  // counts it: without this the caller sees an empty result and reports an empty
  // FILE. Both the line and the polygon are counted here.
  EXPECT_EQ(diagnostics.geometries_with_empty_exterior, 2)
      << "the line and the polygon were each dropped for having no usable exterior";

  // Capped at two: the cap buys two DRAWABLE geometries. Before this fix the
  // empty line and the empty polygon spent both slots and the parse returned two
  // geometries that draw nothing at all.
  DatasetPtr capped_ds(driver->Create("outofdomain_capped", 0, 0, 0, GDT_Unknown, nullptr),
                       gdal_closer);
  ASSERT_TRUE(capped_ds);
  ASSERT_EQ(capped_ds->CopyLayer(layer, "shapes", nullptr) != nullptr, true);
  camp::vector::ParseOptions capped;
  capped.max_geometries = 2;
  camp::vector::ParseDiagnostics capped_diag;
  const std::vector<ParsedLayer> capped_layers =
    camp::vector::parseVectorLayers(capped_ds.get(), capped, &capped_diag);
  ASSERT_EQ(capped_layers.size(), 1u);
  ASSERT_EQ(capped_layers.front().geometries.size(), 2u);
  for(const ParsedGeometry& geometry : capped_layers.front().geometries)
  {
    EXPECT_EQ(geometry.type, ParsedGeometry::Point);
    EXPECT_FALSE(geometry.exterior.empty()) << "a cap slot must buy a drawable geometry";
  }
  EXPECT_TRUE(capped_diag.geometry_cap_reached)
      << "the third point was never read, so the file really was left partly unread";
}

// [camp#22 round-5 should-fix] THE CAP PATH STILL REPORTS WHAT THE LAYER LEFT OUT.
//
// The geometry-cap return sat ABOVE the three per-layer summary blocks, so a
// layer that hit the cap reported none of them. VectorLayer re-reports
// polygons_without_exterior_ring and layers_failed itself, but nothing anywhere
// reports points_dropped or geometries_unhandled — so on the one path where the
// operator is already being told the read was partial, two of the four "what was
// left out" diagnostics vanished entirely.
TEST(VectorParseAttributes, CapPathStillReportsTheLayerSummaries)
{
  GDALAllRegister();
  GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("Memory");
  ASSERT_NE(driver, nullptr);
  DatasetPtr ds(driver->Create("capped_summaries", 0, 0, 0, GDT_Unknown, nullptr), gdal_closer);
  ASSERT_TRUE(ds);

  // Orthographic again: a coordinate outside the visible hemisphere has no
  // inverse, which is how a point is made to FAIL its transform for real.
  OGRSpatialReference ortho;
  ASSERT_EQ(ortho.SetFromUserInput(
              "+proj=ortho +lat_0=0 +lon_0=0 +datum=WGS84 +units=m +no_defs"),
            OGRERR_NONE);
  OGRLayer* layer = ds->CreateLayer("mixed", &ortho, wkbUnknown, nullptr);
  ASSERT_NE(layer, nullptr);

  {  // one point that cannot be transformed
    OGRFeature feature(layer->GetLayerDefn());
    OGRPoint point(1.0e10, 1.0e10);
    feature.SetGeometry(&point);
    ASSERT_EQ(layer->CreateFeature(&feature), OGRERR_NONE);
  }
  {  // one geometry of a type this parser does not handle
    OGRFeature feature(layer->GetLayerDefn());
    OGRCircularString curve;
    curve.addPoint(1000.0, 1000.0);
    curve.addPoint(1100.0, 1100.0);
    curve.addPoint(1200.0, 1000.0);
    feature.SetGeometry(&curve);
    ASSERT_EQ(layer->CreateFeature(&feature), OGRERR_NONE);
  }
  for(int i = 0; i < 3; ++i)
  {  // and enough drawable points to reach a cap of two with a remainder
    OGRFeature feature(layer->GetLayerDefn());
    OGRPoint point(1000.0 * (i + 1), 2000.0);
    feature.SetGeometry(&point);
    ASSERT_EQ(layer->CreateFeature(&feature), OGRERR_NONE);
  }

  static QStringList warnings;
  warnings.clear();
  QtMessageHandler previous = qInstallMessageHandler(
      [](QtMsgType, const QMessageLogContext&, const QString& message)
      {
        warnings.append(message);
      });

  camp::vector::ParseOptions capped;
  capped.max_geometries = 2;
  camp::vector::ParseDiagnostics diag;
  const std::vector<ParsedLayer> layers =
    camp::vector::parseVectorLayers(ds.get(), capped, &diag);
  qInstallMessageHandler(previous);

  ASSERT_EQ(layers.size(), 1u);
  EXPECT_EQ(layers.front().geometries.size(), 2u);
  ASSERT_TRUE(diag.geometry_cap_reached) << "the fixture must actually reach the cap";
  EXPECT_EQ(diag.points_dropped, 1);
  EXPECT_EQ(diag.geometries_unhandled, 1);

  const QString all = warnings.join(" | ");
  EXPECT_TRUE(all.contains("coordinate transformation failed"))
      << "a capped layer must still report its dropped points: " << all.toStdString();
  EXPECT_TRUE(all.contains("unhandled type"))
      << "a capped layer must still report its unhandled geometries: " << all.toStdString();
}

// [camp#22 round-5 suggestion] AN ABORT THAT RISES AT ANY POINT OF THE PARSE IS
// REPORTED AS AN ABORT — INCLUDING INSIDE THE CAP LOOKAHEAD.
//
// moreInputRemains() polls the abort predicate on entry and between layers, but
// the flag can rise during the GetNextFeature() it runs: this iteration's
// per-feature aborted() check has already passed, so the cap branch used to
// return with ParseDiagnostics::aborted UNSET. The header promises the opposite —
// "the parse stops where it is and returns what it has, with
// ParseDiagnostics::aborted set". The window cuts both ways, which is why it is
// worth closing: once an abort is requested the lookahead answers "nothing
// remains", so a genuinely capped parse could come back with
// geometry_cap_reached = false — a "read in full" claim in the one status line
// this design leans on — and nothing at all saying the parse was cut short.
//
// Tested over EVERY poll position rather than a guessed one: the fixture is
// parsed once with a predicate that only counts its polls, then re-parsed once
// per position with the flag rising exactly there. There is no position at which
// a parse can end claiming a clean read, so the lookahead window cannot hide in
// the gaps of a hand-picked index.
TEST(VectorParseAttributes, AnAbortAtAnyPollIsReportedAsAborted)
{
  GDALAllRegister();
  GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("Memory");
  ASSERT_NE(driver, nullptr);
  DatasetPtr ds(driver->Create("lookahead_abort", 0, 0, 0, GDT_Unknown, nullptr), gdal_closer);
  ASSERT_TRUE(ds);

  OGRSpatialReference wgs84;
  wgs84.SetWellKnownGeogCS("WGS84");
  wgs84.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
  OGRLayer* layer = ds->CreateLayer("points", &wgs84, wkbUnknown, nullptr);
  ASSERT_NE(layer, nullptr);
  for(int i = 0; i < 4; ++i)
  {
    OGRFeature feature(layer->GetLayerDefn());
    OGRPoint point(-70.80 + 0.01 * i, 43.00);
    feature.SetGeometry(&point);
    ASSERT_EQ(layer->CreateFeature(&feature), OGRERR_NONE);
  }

  // The reference run: never aborted, and it reaches the cap with a remainder.
  int polls = 0;
  camp::vector::ParseOptions counting;
  counting.max_geometries = 2;
  counting.aborted = [&polls]() { ++polls; return false; };
  camp::vector::ParseDiagnostics counting_diag;
  const std::vector<ParsedLayer> counted =
    camp::vector::parseVectorLayers(ds.get(), counting, &counting_diag);
  ASSERT_EQ(counted.size(), 1u);
  ASSERT_EQ(counted.front().geometries.size(), 2u);
  ASSERT_TRUE(counting_diag.geometry_cap_reached) << "the fixture must reach the cap";
  ASSERT_FALSE(counting_diag.aborted);
  ASSERT_GT(polls, 2) << "the parse must poll the predicate per layer and per feature";

  for(int rise_on = 1; rise_on <= polls; ++rise_on)
  {
    int call = 0;
    camp::vector::ParseOptions aborting;
    aborting.max_geometries = 2;
    aborting.aborted = [&call, rise_on]() { return ++call >= rise_on; };
    camp::vector::ParseDiagnostics aborting_diag;
    const std::vector<ParsedLayer> aborted_layers =
      camp::vector::parseVectorLayers(ds.get(), aborting, &aborting_diag);
    (void)aborted_layers;
    EXPECT_TRUE(aborting_diag.aborted)
        << "the predicate rose at poll " << rise_on << " of " << polls
        << " and the parse did not report the abort; a result whose aborted flag is clear "
           "is one the caller is entitled to draw";
    // [camp#22 round-8 suggestion] The two flags are mutually exclusive: an
    // aborted result is thrown away, so "the rest of the file was not read" — a
    // statement about a result the caller KEEPS — must never ride along with it.
    // The abort re-check inside the cap return is where they could both be set.
    EXPECT_FALSE(aborting_diag.geometry_cap_reached)
        << "the predicate rose at poll " << rise_on << " of " << polls
        << " and the parse reported BOTH aborted and the cap; ParseDiagnostics says a "
           "parse that hits both reports aborted";
  }
}

// [camp#22 round-11 must-fix] THE MISSION-ITEM CONSUMER FILTERS EVERY VERTEX.
//
// The parser admits a geometry on ANY ONE placeable vertex (hasPlaceableCoordinate)
// and says so in terms, because the DISPLAY consumer filters again per vertex
// before it projects anything into the scene. VectorDataset — File > Open Geometry,
// which builds EDITABLE mission items that are dragged, saved into the mission
// file and are candidates for transmission to the robot — has no second filter, so
// the vertices the weak bound admits used to become waypoints at coordinates that
// are nowhere on earth.
//
// The motivating input is exactly this fixture: a layer with NO spatial reference,
// whose projected eastings/northings are read as degrees. No transform runs, no
// vertex "fails", and points_dropped stays zero — nothing in the parse diagnostics
// reports the loss, which is why the guard has to be applied by the consumer.
//
// placeableGeometry() is that guard, tested here rather than through
// VectorDataset::buildItems(): buildItems is a private member of a MissionItem
// whose construction needs an AutonomousVehicleProject, which is not constructible
// in a test harness (the same reason mission_insertion.cpp exists as its own seam).
// What buildItems adds over this function is the item construction it already did.
TEST(VectorParseAttributes, UnplaceableVerticesAreFilteredForTheMissionItemConsumer)
{
  GDALAllRegister();
  GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("Memory");
  ASSERT_NE(driver, nullptr) << "GDAL Memory driver is required for this test";
  DatasetPtr dataset(driver->Create("nosrs", 0, 0, 0, GDT_Unknown, nullptr), gdal_closer);
  ASSERT_TRUE(dataset);

  // NO spatial reference: the .prj-less shapefile case. x/y are read as
  // longitude/latitude verbatim.
  OGRLayer* layer = dataset->CreateLayer("shapes", nullptr, wkbUnknown, nullptr);
  ASSERT_NE(layer, nullptr);

  // A UTM northing read as a latitude. Finite, perfectly well formed, and 4.8
  // million degrees north.
  constexpr double kNorthingAsLatitude = 4800000.0;

  {  // a line: one real position, three that are not
    OGRFeature feature(layer->GetLayerDefn());
    OGRLineString line;
    line.addPoint(-70.71, 43.07);
    line.addPoint(340000.0, kNorthingAsLatitude);
    line.addPoint(340100.0, kNorthingAsLatitude + 100.0);
    line.addPoint(340200.0, kNorthingAsLatitude + 200.0);
    feature.SetGeometry(&line);
    ASSERT_EQ(layer->CreateFeature(&feature), OGRERR_NONE);
  }
  {  // a polygon whose exterior is mixed and whose hole is entirely unplaceable
    OGRFeature feature(layer->GetLayerDefn());
    OGRLinearRing exterior;
    exterior.addPoint(-70.80, 43.00);
    exterior.addPoint(-70.60, 43.00);
    exterior.addPoint(340000.0, kNorthingAsLatitude);
    exterior.addPoint(-70.80, 43.00);
    OGRLinearRing hole;
    hole.addPoint(340010.0, kNorthingAsLatitude + 10.0);
    hole.addPoint(340020.0, kNorthingAsLatitude + 10.0);
    hole.addPoint(340020.0, kNorthingAsLatitude + 20.0);
    hole.addPoint(340010.0, kNorthingAsLatitude + 10.0);
    OGRPolygon polygon;
    polygon.addRing(&exterior);
    polygon.addRing(&hole);
    feature.SetGeometry(&polygon);
    ASSERT_EQ(layer->CreateFeature(&feature), OGRERR_NONE);
  }

  camp::vector::ParseDiagnostics diagnostics;
  const std::vector<ParsedLayer> layers =
    camp::vector::parseVectorLayers(dataset.get(), camp::vector::ParseOptions(), &diagnostics);
  ASSERT_EQ(layers.size(), 1u);
  ASSERT_EQ(layers.front().geometries.size(), 2u)
      << "both geometries have a placeable vertex, so the parser admits them WHOLE - "
         "this is the bound the consumer has to narrow";
  EXPECT_EQ(diagnostics.points_dropped, 0)
      << "no transform ran, so no vertex failed: the parse diagnostics report nothing "
         "at all about this file, which is why the consumer cannot rely on them";
  EXPECT_EQ(diagnostics.geometries_without_placeable_vertex, 0);

  const ParsedGeometry& parsed_line = layers.front().geometries.at(0);
  ASSERT_EQ(parsed_line.type, ParsedGeometry::LineString);
  ASSERT_EQ(parsed_line.exterior.size(), 4u);
  const camp::vector::PlaceableGeometry line = camp::vector::placeableGeometry(parsed_line);
  EXPECT_TRUE(line.placeable);
  ASSERT_EQ(line.exterior.size(), 1u)
      << "only the one real position may become a mission waypoint";
  EXPECT_DOUBLE_EQ(line.exterior.front().latitude(), 43.07);
  EXPECT_DOUBLE_EQ(line.exterior.front().longitude(), -70.71);
  EXPECT_EQ(line.vertices_dropped, 3);
  EXPECT_EQ(line.rings_dropped, 0);

  const ParsedGeometry& parsed_polygon = layers.front().geometries.at(1);
  ASSERT_EQ(parsed_polygon.type, ParsedGeometry::Polygon);
  ASSERT_EQ(parsed_polygon.interiorRings.size(), 1u);
  const camp::vector::PlaceableGeometry polygon = camp::vector::placeableGeometry(parsed_polygon);
  EXPECT_TRUE(polygon.placeable);
  EXPECT_EQ(polygon.exterior.size(), 3u) << "the ring's three real vertices survive";
  EXPECT_TRUE(polygon.interiorRings.empty())
      << "a hole with no placeable vertex is omitted, not carried as an empty ring";
  EXPECT_EQ(polygon.rings_dropped, 1);
  EXPECT_EQ(polygon.vertices_dropped, 5)
      << "1 exterior vertex + all 4 vertices of the hole (a ring is read as the file "
         "states it, closing vertex included)";
}

// [camp#22 round-11 must-fix] An exterior with NO placeable vertex takes the whole
// geometry, holes included — the rule firstPlaceableCoordinate() states: a polygon
// rebuilt from its hole alone is painted as SOLID by Qt::OddEvenFill, turning a
// hole into a feature in a file CAMP has just said it cannot place.
TEST(VectorParseAttributes, AnUnplaceableExteriorTakesTheHolesWithIt)
{
  ParsedGeometry geometry;
  geometry.type = ParsedGeometry::Polygon;
  geometry.exterior = {QGeoCoordinate(4800000.0, 340000.0),
                       QGeoCoordinate(4800100.0, 340000.0),
                       QGeoCoordinate(4800100.0, 340100.0)};
  geometry.interiorRings = {{QGeoCoordinate(43.05, -70.70), QGeoCoordinate(43.06, -70.70),
                             QGeoCoordinate(43.06, -70.69)}};

  const camp::vector::PlaceableGeometry placeable = camp::vector::placeableGeometry(geometry);
  EXPECT_FALSE(placeable.placeable);
  EXPECT_TRUE(placeable.exterior.empty());
  EXPECT_TRUE(placeable.interiorRings.empty())
      << "the perfectly good hole must NOT become the item's outline";
  EXPECT_EQ(placeable.vertices_dropped, 3)
      << "the exterior's three vertices; the geometry is dropped WHOLE, so its holes are "
         "not counted a second time as a per-vertex loss";
  EXPECT_EQ(placeable.rings_dropped, 0);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
