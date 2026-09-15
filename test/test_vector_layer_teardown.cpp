// [camp#22] Thread-safe teardown + GDAL hygiene for the read-only vector layer,
// and the load path that feeds them.
//
// VectorLayer parses its file on a QtConcurrent pool thread that reads the
// layer's own members. If the destructor did not abort and JOIN that worker
// before tearing down (the camp#213 pattern, as RasterLayer and the ROS geometry
// items do it), closing a layer while its file was still loading would be a
// use-after-free — the crash class that is hardest to reproduce and worst to meet
// on the water. The abort-during-load case below is deliberately destroyed
// WITHOUT waiting for the load, which is the racing shape.
//
// The dataset-count baseline additionally proves the worker's RAII closes the
// GDAL handle on every path, including the aborted one.
//
// [camp#22] The file also carries the interaction case that needs a real, loaded
// layer rather than a hand-built item: a click delivered through a real
// QGraphicsView onto a real feature (`VectorLayerInteraction`), from the operator
// GUI test of 2026-09-15.

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>

#include <gdal_priv.h>

#include <QApplication>
#include <QElapsedTimer>
#include <QFile>
#include <QGraphicsView>
#include <QMouseEvent>
#include <QTemporaryDir>
#include <QToolTip>

#include "map/layer_list.h"
#include "map/map.h"
#include "vector/vector_feature_item.h"
#include "vector/vector_layer.h"
#include "vector/vector_parse.h"

namespace
{

int openDatasetCount()
{
  int count = 0;
  GDALDataset::GetOpenDatasets(&count);
  return count;
}

// A GeoJSON with points, a line and a polygon-with-a-hole, plus attributes — one
// of everything the layer builds items for.
QString writeGeoJson(const QTemporaryDir& dir)
{
  const QString path = dir.filePath("features.geojson");
  QFile file(path);
  if(!file.open(QIODevice::WriteOnly | QIODevice::Text))
    return QString();
  file.write(R"({
    "type": "FeatureCollection",
    "features": [
      {"type": "Feature",
       "geometry": {"type": "Point", "coordinates": [-70.71, 43.07]},
       "properties": {"signal": 58.0, "assessment": "candidate C"}},
      {"type": "Feature",
       "geometry": {"type": "Point", "coordinates": [-70.70, 43.06]},
       "properties": {"signal": 10.0, "assessment": "background"}},
      {"type": "Feature",
       "geometry": {"type": "Point", "coordinates": [-70.69, 43.05]},
       "properties": {"assessment": "no signal field"}},
      {"type": "Feature",
       "geometry": {"type": "LineString",
                    "coordinates": [[-70.68, 43.04], [-70.67, 43.03], [-70.66, 43.05]]},
       "properties": {"signal": 30.0}},
      {"type": "Feature",
       "geometry": {"type": "Polygon",
                    "coordinates": [[[-70.80, 43.00], [-70.60, 43.00], [-70.60, 43.20],
                                     [-70.80, 43.20], [-70.80, 43.00]],
                                    [[-70.74, 43.06], [-70.66, 43.06], [-70.66, 43.14],
                                     [-70.74, 43.14], [-70.74, 43.06]]]},
       "properties": {"signal": 20.0}}
    ]
  })");
  file.close();
  return path;
}

// [camp#22] Two point features far enough apart that neither marker is anywhere
// near the other, and NO polygon or line over them: what a click lands on is then
// decided by the point's own shape() and not by which item happens to be on top.
QString writeTwoPoints(const QTemporaryDir& dir)
{
  const QString path = dir.filePath("two_points.geojson");
  QFile file(path);
  if(!file.open(QIODevice::WriteOnly | QIODevice::Text))
    return QString();
  file.write(R"({
    "type": "FeatureCollection",
    "features": [
      {"type": "Feature",
       "geometry": {"type": "Point", "coordinates": [-70.71, 43.07]},
       "properties": {"tfa_nT": 41.5, "assessment": "candidate C"}},
      {"type": "Feature",
       "geometry": {"type": "Point", "coordinates": [-70.60, 43.00]},
       "properties": {"tfa_nT": 3.0, "assessment": "background"}}
    ]
  })");
  file.close();
  return path;
}

// [camp#22] A GeoJSON with `count` point features — big enough that parsing it is
// measurably slower than an aborted load, which is what the bounded-join test
// below needs.
QString writeManyPoints(const QTemporaryDir& dir, int count)
{
  const QString path = dir.filePath("many.geojson");
  QFile file(path);
  if(!file.open(QIODevice::WriteOnly | QIODevice::Text))
    return QString();
  QByteArray json = R"({"type": "FeatureCollection", "features": [)";
  for(int i = 0; i < count; ++i)
  {
    if(i)
      json += ',';
    json += QStringLiteral("{\"type\": \"Feature\", \"geometry\": {\"type\": \"Point\", "
                           "\"coordinates\": [%1, %2]}, \"properties\": {\"n\": %3}}")
              .arg(-70.8 + (i % 1000) * 0.0001, 0, 'f', 6)
              .arg(43.0 + (i / 1000) * 0.0001, 0, 'f', 6)
              .arg(i)
              .toUtf8();
  }
  json += "]}";
  file.write(json);
  file.close();
  return path;
}

// [camp#22] A GeoJSON holding ONE feature: a single LineString of `count`
// vertices. The many-points fixture above exercises only PART boundaries — every
// poll site it reaches is a feature or a collection part — so it cannot tell a
// parser that polls the abort flag per part from one that also polls it per
// vertex. This fixture has exactly one part, so the only place an abort can be
// honoured is inside the vertex loop.
QString writeOneHugeLineString(const QTemporaryDir& dir, int count)
{
  const QString path = dir.filePath("huge_ring.geojson");
  QFile file(path);
  if(!file.open(QIODevice::WriteOnly | QIODevice::Text))
    return QString();
  QByteArray json = R"({"type": "FeatureCollection", "features": [{"type": "Feature", )"
                    R"("properties": {"n": 1}, "geometry": {"type": "LineString", )"
                    R"("coordinates": [)";
  for(int i = 0; i < count; ++i)
  {
    if(i)
      json += ',';
    json += QStringLiteral("[%1,%2]")
              .arg(-70.8 + (i % 1000) * 0.0001, 0, 'f', 6)
              .arg(43.0 + (i / 1000) * 0.0001, 0, 'f', 6)
              .toUtf8();
  }
  json += "]}}]}";
  file.write(json);
  file.close();
  return path;
}

// The parse is async; the status leaves "(loading...)" when it completes.
bool waitForLoad(const camp::vector::VectorLayer* layer, int timeout_ms = 5000)
{
  QElapsedTimer timer;
  timer.start();
  while(layer->status() == QStringLiteral("(loading...)") && timer.elapsed() < timeout_ms)
    QCoreApplication::processEvents(QEventLoop::AllEvents, 50);
  return layer->loaded();
}

}  // namespace

// The load builds one item per feature, exposes the attribute fields, and leaves
// no GDAL handle behind; destruction joins the (already finished) worker.
TEST(VectorLayerTeardown, LoadsFeaturesAndLeaksNoDataset)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString path = writeGeoJson(dir);
  ASSERT_FALSE(path.isEmpty());

  camp::map::Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);

  const int baseline = openDatasetCount();

  auto* layer = new camp::vector::VectorLayer(layers, path);
  ASSERT_TRUE(waitForLoad(layer)) << "load did not complete: " << layer->status().toStdString();
  EXPECT_EQ(layer->featureCount(), 5);
  EXPECT_TRUE(layer->fields().contains("signal"));
  EXPECT_TRUE(layer->fields().contains("assessment"));
  EXPECT_FALSE(layer->boundingRect().isEmpty());

  // Styling a loaded layer is a pass over existing items — no re-parse, no
  // re-open, so the dataset count cannot move.
  layer->setColorField("signal");
  layer->setSizeField("signal");
  EXPECT_EQ(openDatasetCount(), baseline);

  // Direct delete (not removeFromMap, which defers to deleteLater) so the dtor's
  // join is synchronous and the assertion below is firm rather than flaky.
  delete layer;
  EXPECT_EQ(openDatasetCount(), baseline);
}

// The racing shape: destroy the layer WITHOUT waiting for the load. The dtor must
// abort and join the worker — a worker still running here would be reading a
// destroyed object.
TEST(VectorLayerTeardown, DestroyDuringLoadJoinsTheWorker)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString path = writeGeoJson(dir);
  ASSERT_FALSE(path.isEmpty());

  camp::map::Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);

  const int baseline = openDatasetCount();
  for(int i = 0; i < 5; ++i)
  {
    auto* layer = new camp::vector::VectorLayer(layers, path);
    delete layer;   // no processEvents: the load may well still be in flight
  }
  // Every load — completed or aborted — closed its dataset.
  EXPECT_EQ(openDatasetCount(), baseline);
}

// A file GDAL cannot open fails loudly in the Layers tab rather than presenting
// as an empty-but-fine layer.
TEST(VectorLayerTeardown, UnopenableFileReportsLoadFailed)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString path = dir.filePath("not-a-vector-file.geojson");
  QFile file(path);
  ASSERT_TRUE(file.open(QIODevice::WriteOnly));
  file.write("this is not vector data");
  file.close();

  camp::map::Map map;
  auto* layer = new camp::vector::VectorLayer(map.topLevelLayers(), path);
  QElapsedTimer timer;
  timer.start();
  while(layer->status() == QStringLiteral("(loading...)") && timer.elapsed() < 5000)
    QCoreApplication::processEvents(QEventLoop::AllEvents, 50);
  EXPECT_FALSE(layer->loaded());
  EXPECT_EQ(layer->status(), QStringLiteral("(load failed)"));
  EXPECT_EQ(layer->featureCount(), 0);
  delete layer;
}

// [camp#22] A GDAL virtual-file-system path is REFUSED without being opened.
//
// The driver allowlist cannot stop a remote read: GDAL resolves /vsicurl/,
// /vsizip/ and /vsis3/ ahead of driver selection, so the fetch happens and the
// allowed GeoJSON driver reads the result. The path check is the gate, and it has
// to hold on the RESTORE path too, which reopens every persisted entry at startup
// with no operator present. Opening no dataset is the assertion that matters here:
// a /vsicurl/ URL that reached GDAL would go to the network from a unit test.
TEST(VectorLayerTeardown, VirtualFileSystemPathsAreRefusedWithoutOpening)
{
  camp::map::Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);

  const int baseline = openDatasetCount();

  for(const QString& path : {QStringLiteral("/vsicurl/https://example.invalid/x.geojson"),
                             QStringLiteral("/vsizip//data/archive.zip/x.shp"),
                             QStringLiteral("/vsis3/bucket/x.gpkg"),
                             QStringLiteral("  /vsicurl/https://example.invalid/x.geojson")})
  {
    auto* layer = new camp::vector::VectorLayer(layers, path);
    // Refused in the constructor: no worker was started, so this is final without
    // waiting for anything.
    EXPECT_FALSE(layer->loaded()) << path.toStdString();
    EXPECT_EQ(layer->status(), QStringLiteral("(refused: not a local file)"))
        << path.toStdString();
    EXPECT_EQ(layer->featureCount(), 0) << path.toStdString();
    EXPECT_EQ(openDatasetCount(), baseline)
        << "a /vsi path must never reach GDALOpenEx: " << path.toStdString();
    delete layer;
  }

  // The predicate itself, since both the project's open path and its restore path
  // call it directly to keep a refused path out of `vectorLayers/files`.
  EXPECT_TRUE(camp::vector::isVirtualFileSystemPath("/vsicurl/https://h/x.geojson"));
  EXPECT_TRUE(camp::vector::isVirtualFileSystemPath("/vsizip//data/a.zip/x.shp"));
  EXPECT_TRUE(camp::vector::isVirtualFileSystemPath("/VSIS3/bucket/x.gpkg"));
  EXPECT_FALSE(camp::vector::isVirtualFileSystemPath("/data/survey/x.geojson"));
  EXPECT_FALSE(camp::vector::isVirtualFileSystemPath("/data/vsicurl/x.geojson"));
  EXPECT_FALSE(camp::vector::isVirtualFileSystemPath(QString()));

  // [camp#22 suggestion] An ordinary local DIRECTORY whose name merely begins
  // with the same four characters is not a virtual file system, and refusing it
  // made real files unopenable. The check asks GDAL for its registered handler
  // prefixes (VSIGetFileSystemsPrefixes) instead of matching "/vsi" raw; a
  // prefix-boundary check would not have helped, since "/vsidata/" is still
  // "/vsi<word>/".
  EXPECT_FALSE(camp::vector::isVirtualFileSystemPath("/vsidata/survey.geojson"));
  EXPECT_FALSE(camp::vector::isVirtualFileSystemPath("/vsi_survey/2026/x.shp"));
  EXPECT_FALSE(camp::vector::isVirtualFileSystemPath("/vsi/x.geojson"));
  // And the real ones are still refused, including the archive handlers and the
  // in-memory one, which are registered prefixes like any other.
  EXPECT_TRUE(camp::vector::isVirtualFileSystemPath("/vsimem/x.geojson"));
  EXPECT_TRUE(camp::vector::isVirtualFileSystemPath("/vsitar//data/a.tar/x.shp"));
  EXPECT_TRUE(camp::vector::isVirtualFileSystemPath("/vsigzip//data/x.geojson.gz"));
}

// [camp#22 must-fix 7] Per-feature item construction happens on the GUI thread and
// the Open Vector Layer dialog does not bound what an operator can pick. A
// coastline shapefile is millions of features; without a cap CAMP freezes with no
// message and no way out. The cap draws the first N and REPORTS the shortfall.
TEST(VectorLayerTeardown, FeatureCapBoundsGuiThreadWorkAndIsReported)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString path = writeGeoJson(dir);   // five features
  ASSERT_FALSE(path.isEmpty());

  camp::map::Map map;
  auto* layer = new camp::vector::VectorLayer(map.topLevelLayers(), path, 2);
  ASSERT_TRUE(waitForLoad(layer)) << "load did not complete: " << layer->status().toStdString();

  EXPECT_EQ(layer->featureCap(), 2);
  EXPECT_EQ(layer->featureCount(), 2) << "the cap must bound the items actually built";
  // Silence is the failure mode that matters: a layer showing 2 of 5 features and
  // saying "(2 features)" is indistinguishable from a file that holds 2.
  EXPECT_TRUE(layer->status().contains("cap"))
      << "status must report that the cap was hit: " << layer->status().toStdString();
  // ...and it must say that the rest of the file went UNREAD, which is the whole
  // point of capping the parse rather than its result. How many more features the
  // file holds is not reported: reading that far is the cost the cap avoids.
  EXPECT_TRUE(layer->status().contains("not read"))
      << "status must say the rest of the file was not read: "
      << layer->status().toStdString();
  delete layer;

  // The shipped cap is the default and is not applied to ordinary files.
  auto* uncapped = new camp::vector::VectorLayer(map.topLevelLayers(), path);
  ASSERT_TRUE(waitForLoad(uncapped));
  EXPECT_EQ(uncapped->featureCap(), camp::vector::VectorLayer::kMaxFeatureItems);
  EXPECT_EQ(uncapped->featureCount(), 5);
  EXPECT_EQ(uncapped->status(), QStringLiteral("(5 features)"));
  delete uncapped;
}

// [camp#22 must-fix 6] The destructor's join must be bounded by the ABORT, not by
// the size of the file. The abort flag used to be read once, before GDALOpenEx, so
// destroying a layer part way through a large parse blocked the GUI thread until
// the whole file had been read. The parser now polls the flag per feature.
TEST(VectorLayerTeardown, DestroyDuringLoadDoesNotWaitOutTheWholeParse)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString path = writeManyPoints(dir, 200000);
  ASSERT_FALSE(path.isEmpty());

  camp::map::Map map;
  camp::map::LayerList* layers = map.topLevelLayers();

  // How long the full parse takes on this machine, measured rather than assumed.
  // Measured through parseVectorLayers directly, with NO cap: a capped layer stops
  // early by design now (the cap is carried into the parse), so it can no longer
  // stand in for a full read, and going through VectorLayer would fold 200 000
  // GUI-thread item constructions into the number. What is being timed is the
  // PARSE, which is exactly what the abort has to cut short.
  const auto gdal_closer = [](GDALDataset* d){ if(d) GDALClose(d); };
  QElapsedTimer full_timer;
  full_timer.start();
  {
    GDALAllRegister();
    std::unique_ptr<GDALDataset, decltype(gdal_closer)> dataset(
      static_cast<GDALDataset*>(
        GDALOpenEx(path.toUtf8().constData(), GDAL_OF_READONLY | GDAL_OF_VECTOR,
                   nullptr, nullptr, nullptr)),
      gdal_closer);
    ASSERT_TRUE(dataset);
    const auto parsed = camp::vector::parseVectorLayers(dataset.get());
    ASSERT_EQ(parsed.size(), 1u);
    ASSERT_EQ(parsed.front().geometries.size(), 200000u);
  }
  const qint64 full_ms = full_timer.elapsed();

  // [camp#22 suggestion] The OBSERVABLE condition, asserted unconditionally: an
  // abort raised part way through leaves a PARTIAL parse and says so. This is what
  // the test is really about, and unlike a wall-clock comparison it means the same
  // thing on every machine. (The flag is polled per feature and, since the budget
  // was threaded into the geometry recursion, inside a multi-part feature too.)
  {
    GDALAllRegister();
    std::unique_ptr<GDALDataset, decltype(gdal_closer)> dataset(
      static_cast<GDALDataset*>(
        GDALOpenEx(path.toUtf8().constData(), GDAL_OF_READONLY | GDAL_OF_VECTOR,
                   nullptr, nullptr, nullptr)),
      gdal_closer);
    ASSERT_TRUE(dataset);
    int polls = 0;
    camp::vector::ParseOptions options;
    options.aborted = [&polls]() { return ++polls > 10; };
    camp::vector::ParseDiagnostics diagnostics;
    const auto parsed = camp::vector::parseVectorLayers(dataset.get(), options, &diagnostics);
    EXPECT_TRUE(diagnostics.aborted) << "an abort part way through must be reported";
    size_t emitted = 0;
    for(const auto& layer : parsed)
      emitted += layer.geometries.size();
    EXPECT_LT(emitted, 200000u)
        << "the parse ran to the end of the file despite the abort flag";
  }

  // The wall-clock comparison below is a SECOND, weaker signal: it catches an
  // abort that is honoured but arrives late. It is only meaningful when the full
  // parse is slow enough for "half of it" to be outside the timer's noise, so on a
  // machine fast enough to read the fixture in under 200 ms it is reported and
  // skipped rather than failed — a correct implementation must not fail for being
  // run on better hardware. The assertion above is the one that always holds.

  // Now destroy immediately. The dtor sets the abort flag and joins; with the flag
  // polled inside the parse loop this returns long before the file is read. The
  // cap is set ABOVE the fixture's feature count, so the only thing that can end
  // the parse early is the abort.
  QElapsedTimer abort_timer;
  abort_timer.start();
  {
    auto* layer = new camp::vector::VectorLayer(layers, path, 200001);
    delete layer;
  }
  const qint64 abort_ms = abort_timer.elapsed();
  // Only meaningful when the full parse is slow enough for "half of it" to sit
  // outside the timer's noise. On a machine that reads the fixture in under 200 ms
  // the comparison is reported and skipped rather than failed: a correct
  // implementation must not fail for being run on faster hardware. The
  // partial-parse assertion above is the one that always holds.
  if(full_ms > 200)
    EXPECT_LT(abort_ms, full_ms / 2)
        << "aborted teardown took " << abort_ms << " ms against a full parse of " << full_ms
        << " ms — the abort flag is not being polled inside the parse";
  else
    GTEST_LOG_(INFO) << "full parse of the 200 000-point fixture took only " << full_ms
                     << " ms; the wall-clock ratio check is unmeasurable here and was"
                     << " skipped (aborted teardown: " << abort_ms << " ms).";
}

// [camp#22 should-fix, round 2] One ring must not be an unbounded stretch inside
// the worker the destructor joins. readRing used to consume EVERY vertex before
// returning, so the abort flag — polled per feature and per collection part —
// bounded nothing for a single LineString or ring of millions of points. The poll
// now also runs inside the vertex loop (every kVertexPollInterval vertices; the
// predicate takes a mutex, so per-vertex polling is not free).
//
// The fixture is deliberately ONE feature with ONE part: every poll site the
// 200 000-point fixture above exercises is a part boundary, so it passes whether
// or not the vertex loop polls anything.
TEST(VectorLayerTeardown, AbortCutsShortASingleHugeRing)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  constexpr int kVertices = 400000;
  const QString path = writeOneHugeLineString(dir, kVertices);
  ASSERT_FALSE(path.isEmpty());

  const auto gdal_closer = [](GDALDataset* d){ if(d) GDALClose(d); };
  GDALAllRegister();

  // The fixture really is one feature of kVertices vertices — otherwise the
  // truncation asserted below would prove nothing.
  {
    std::unique_ptr<GDALDataset, decltype(gdal_closer)> dataset(
      static_cast<GDALDataset*>(
        GDALOpenEx(path.toUtf8().constData(), GDAL_OF_READONLY | GDAL_OF_VECTOR,
                   nullptr, nullptr, nullptr)),
      gdal_closer);
    ASSERT_TRUE(dataset);
    const auto parsed = camp::vector::parseVectorLayers(dataset.get());
    ASSERT_EQ(parsed.size(), 1u);
    ASSERT_EQ(parsed.front().geometries.size(), 1u);
    ASSERT_EQ(parsed.front().geometries.front().exterior.size(),
              static_cast<size_t>(kVertices));
  }

  // Now abort a few polls in. With the flag polled only at part boundaries this
  // ring is read to its last vertex; with the in-loop poll it stops within one
  // poll interval of where the flag was raised.
  {
    std::unique_ptr<GDALDataset, decltype(gdal_closer)> dataset(
      static_cast<GDALDataset*>(
        GDALOpenEx(path.toUtf8().constData(), GDAL_OF_READONLY | GDAL_OF_VECTOR,
                   nullptr, nullptr, nullptr)),
      gdal_closer);
    ASSERT_TRUE(dataset);
    int polls = 0;
    camp::vector::ParseOptions options;
    options.aborted = [&polls]() { return ++polls > 4; };
    camp::vector::ParseDiagnostics diagnostics;
    const auto parsed = camp::vector::parseVectorLayers(dataset.get(), options, &diagnostics);
    EXPECT_TRUE(diagnostics.aborted) << "an abort raised inside the ring must be reported";
    size_t vertices = 0;
    for(const auto& layer : parsed)
      for(const auto& geometry : layer.geometries)
        vertices += geometry.exterior.size();
    // Generous against the poll interval (1024) and the handful of polls the
    // layer/feature/geometry checks spend before the ring is reached, but two
    // orders of magnitude below a ring read to its end — which is the failure.
    EXPECT_LT(vertices, 20000u)
        << "the ring was read to " << vertices
        << " vertices despite the abort flag — the vertex loop is not polling it";
  }
}

// [camp#22] The no-data SECOND CHANNEL is wired to the layer, not just to the
// free function. `isNoData()` is unit-tested in test_vector_layer_styling, but
// nothing asserted that applyStyle() actually sets it on the items — a layer that
// computed the flag and never delivered it would paint a feature with no value at
// the bottom of the ramp, the exact misreading this feature exists to prevent.
// The reverse direction matters just as much: clearing the colour field means
// "no field, so nothing can be missing", and a stale dashed outline would claim
// the opposite.
TEST(VectorLayerTeardown, ApplyStyleFlagsFeaturesWithNoValue)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString path = writeGeoJson(dir);
  ASSERT_FALSE(path.isEmpty());

  camp::map::Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);

  auto* layer = new camp::vector::VectorLayer(layers, path);
  ASSERT_TRUE(waitForLoad(layer)) << "load did not complete: " << layer->status().toStdString();

  // The fixture holds one point with no "signal" property; every other feature
  // carries a numeric one.
  layer->setColorField("signal");
  int with_value = 0;
  int flagged = 0;
  for(QGraphicsItem* child : layer->childItems())
  {
    auto* feature = dynamic_cast<camp::vector::VectorFeatureItem*>(child);
    if(!feature)
      continue;
    if(feature->attributes().contains("signal"))
    {
      ++with_value;
      EXPECT_FALSE(feature->isNoData()) << "a feature WITH a value was flagged no-data";
    }
    else
    {
      ++flagged;
      EXPECT_TRUE(feature->isNoData()) << "a feature with no value was not flagged";
    }
  }
  EXPECT_EQ(with_value, 4);
  EXPECT_EQ(flagged, 1);

  // No colour field at all: nothing is missing, so the flag must clear everywhere.
  layer->setColorField(QString());
  for(QGraphicsItem* child : layer->childItems())
  {
    auto* feature = dynamic_cast<camp::vector::VectorFeatureItem*>(child);
    if(feature)
      EXPECT_FALSE(feature->isNoData()) << "no-data survived clearing the colour field";
  }

  delete layer;
}

// [camp#22] A click delivered THROUGH A REAL VIEW onto a real loaded feature
// shows that feature's attributes.
//
// Every other click test in this repo sends a QGraphicsSceneMouseEvent straight
// to the item, which skips everything between the operator's mouse and it: the
// viewport widget, the view's y-flip (CAMP's map view is scaled (1, -1) so north
// is up), ScrollHandDrag's own press handling, and the scene's hit test against
// shape(). The operator's GUI test of 2026-09-15 reported never seeing a tooltip
// with all of that in the path, so the regression test has to have it in the path
// too: a QGraphicsView over the Map's scene, a synthesized QMouseEvent on its
// VIEWPORT, and the item found by mapping ITS scene position back to the
// viewport.
//
// The off-centre click is the point of the test: 7 px is outside the 5 px drawn
// marker and inside the 9 px click target, which is the slack added because the
// pan cursor is an open hand whose hotspot the operator cannot see.
TEST(VectorLayerInteraction, ClickThroughARealViewShowsTheAttributes)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString path = writeTwoPoints(dir);
  ASSERT_FALSE(path.isEmpty());

  camp::map::Map map;
  auto* layer = new camp::vector::VectorLayer(map.topLevelLayers(), path);
  ASSERT_TRUE(waitForLoad(layer)) << "load did not complete: " << layer->status().toStdString();
  ASSERT_EQ(layer->featureCount(), 2);

  QGraphicsView view(map.scene());
  view.setDragMode(QGraphicsView::ScrollHandDrag);   // pan mode: the popup's gate
  view.scale(1.0, -1.0);                             // CAMP's map view: north up
  view.resize(800, 600);
  view.show();
  QCoreApplication::processEvents();

  camp::vector::VectorFeatureItem* target = nullptr;
  for(QGraphicsItem* child : layer->childItems())
  {
    auto* feature = dynamic_cast<camp::vector::VectorFeatureItem*>(child);
    if(feature && feature->isPoint())
    {
      target = feature;
      break;
    }
  }
  ASSERT_NE(target, nullptr);
  ASSERT_FALSE(target->attributeText().isEmpty());

  view.centerOn(target->scenePos());
  QCoreApplication::processEvents();
  const QPoint centre = view.mapFromScene(target->scenePos());

  // Press and release at the same viewport point — a click, not a pan.
  auto clickAt = [&view](const QPoint& viewport_pos)
  {
    const QPointF global = view.viewport()->mapToGlobal(viewport_pos);
    QMouseEvent press(QEvent::MouseButtonPress, QPointF(viewport_pos), global,
                      Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(view.viewport(), &press);
    QMouseEvent release(QEvent::MouseButtonRelease, QPointF(viewport_pos), global,
                        Qt::LeftButton, Qt::NoButton, Qt::NoModifier);
    QApplication::sendEvent(view.viewport(), &release);
    QCoreApplication::processEvents();
  };

  // Dead centre.
  QToolTip::showText(QPoint(0, 0), QStringLiteral("sentinel: no popup was shown"));
  clickAt(centre);
  EXPECT_EQ(QToolTip::text(), target->attributeText())
      << "a click on the marker, through a real view, showed no attributes";

  // 3 px off-centre: well inside the target, and the kind of aim the operator
  // actually has under a hand cursor.
  QToolTip::showText(QPoint(0, 0), QStringLiteral("sentinel: no popup was shown"));
  clickAt(centre + QPoint(3, 2));
  EXPECT_EQ(QToolTip::text(), target->attributeText())
      << "a click 3 px off centre missed the feature";

  // 7 px off-centre: OUTSIDE the drawn 5 px marker, inside the click slack. This
  // is what the slack is for, and what fails without it.
  QToolTip::showText(QPoint(0, 0), QStringLiteral("sentinel: no popup was shown"));
  clickAt(centre + QPoint(7, 0));
  EXPECT_EQ(QToolTip::text(), target->attributeText())
      << "a click just outside the marker missed the feature: the click slack is gone";

  // The slack is BOUNDED: a click well away from any feature does not answer with
  // THIS feature's attributes. Asserted as "not the feature's text" rather than
  // against a planted sentinel, because a press on empty map is dispatched to the
  // view, which hides any tooltip standing — so both "the sentinel survived" and
  // "the tooltip was cleared" are correct outcomes here and only one of them is a
  // sentinel comparison.
  QToolTip::showText(QPoint(0, 0), QStringLiteral("sentinel: no popup was shown"));
  clickAt(centre + QPoint(60, 40));
  EXPECT_NE(QToolTip::text(), target->attributeText())
      << "a click nowhere near a feature answered with that feature's attributes";

  delete layer;
}

int main(int argc, char** argv)
{
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(argc, argv);
  // Map's ctor writes a tile-layer seed into QSettings; a test org/app name keeps
  // that out of the developer's real camp settings (camp#117 precedent).
  QCoreApplication::setOrganizationName("camp_test");
  QCoreApplication::setApplicationName("test_vector_layer_teardown");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
