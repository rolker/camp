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

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
