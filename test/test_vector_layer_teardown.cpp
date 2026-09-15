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
// [camp#22] The file also carries the layer-level styling and interaction cases
// that need a real, loaded layer rather than a hand-built item: which fields the
// styling menus may offer (`VectorLayerStyleFields`), and a click delivered
// through a real QGraphicsView onto a real feature (`VectorLayerInteraction`):
// the hover label, and a press falling through to the view's pan gesture.
// Both come from the operator GUI test of 2026-09-15.

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>

#include <gdal_priv.h>

#include <QApplication>
#include <QElapsedTimer>
#include <QFile>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsSimpleTextItem>
#include <QAction>
#include <QMenu>
#include <QMouseEvent>
#include <QTemporaryDir>

#include "map/layer_list.h"
#include "map/map.h"
#include "vector/vector_feature_item.h"
#include "vector/vector_layer.h"
#include "vector/vector_parse.h"
#include "vector/vector_style.h"

namespace
{

// [camp#22] VectorLayer::contextMenu() is protected (map::Layer declares it so),
// and the menus it builds are the only way the operator reaches the styling
// fields — so the probe exposes it rather than the test asserting on something
// adjacent.
struct MenuProbe: public camp::vector::VectorLayer
{
  using VectorLayer::VectorLayer;
  using VectorLayer::contextMenu;
};

// The entries of the submenu titled `title`, or an empty list if there is none.
QStringList submenuEntries(const QMenu& menu, const QString& title)
{
  QStringList entries;
  for(QAction* action : menu.actions())
    if(action->menu() && action->text() == title)
      for(QAction* entry : action->menu()->actions())
        entries << entry->text();
  return entries;
}

QAction* submenuAction(const QMenu& menu, const QString& title, const QString& entry_text)
{
  for(QAction* action : menu.actions())
    if(action->menu() && action->text() == title)
      for(QAction* entry : action->menu()->actions())
        if(entry->text() == entry_text)
          return entry;
  return nullptr;
}

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
       "properties": {"signal": 58.0, "assessment": "candidate C", "depth_m": 12.5}},
      {"type": "Feature",
       "geometry": {"type": "Point", "coordinates": [-70.70, 43.06]},
       "properties": {"signal": 10.0, "assessment": "background", "depth_m": "n/a"}},
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

// [camp#22] Every property is free-text: numericFields() must come back EMPTY,
// so a stale style field set over this file exercises the contextMenu() guard
// with field_names.isEmpty() true.
QString writeStringOnlyGeoJson(const QTemporaryDir& dir)
{
  const QString path = dir.filePath("string_only.geojson");
  QFile file(path);
  if(!file.open(QIODevice::WriteOnly | QIODevice::Text))
    return QString();
  file.write(R"({
    "type": "FeatureCollection",
    "features": [
      {"type": "Feature",
       "geometry": {"type": "Point", "coordinates": [-70.71, 43.07]},
       "properties": {"assessment": "candidate C"}},
      {"type": "Feature",
       "geometry": {"type": "Point", "coordinates": [-70.70, 43.06]},
       "properties": {"assessment": "background"}}
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

// [camp#22] The styling menus may offer only fields a RAMP CAN READ.
//
// The operator's GUI test of 2026-09-15 coloured by `assessment`, a free-text
// field: no feature has a numeric value for it, so the range came back invalid,
// every feature was marked no-data, and the layer went hollow grey — which read
// as the features disappearing. `numericFields()` is what the menus offer now.
// `fields()` is deliberately unchanged: the attribute popup and a future
// label-by-field want every field.
TEST(VectorLayerStyleFields, NumericFieldsExcludeAStringOnlyField)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString path = writeGeoJson(dir);
  ASSERT_FALSE(path.isEmpty());

  camp::map::Map map;
  auto* layer = new camp::vector::VectorLayer(map.topLevelLayers(), path);
  ASSERT_TRUE(waitForLoad(layer)) << "load did not complete: " << layer->status().toStdString();

  const QStringList all = layer->fields();
  const QStringList numeric = layer->numericFields();

  // Every field is still reported by fields() — that accessor did not narrow.
  EXPECT_TRUE(all.contains("signal"));
  EXPECT_TRUE(all.contains("assessment"));
  EXPECT_TRUE(all.contains("depth_m"));

  // A field every feature holds a number for: offerable.
  EXPECT_TRUE(numeric.contains("signal"));
  // A MIXED field — one feature has 12.5, another the string "n/a". A ramp reads
  // it fine (the string feature is simply no-data against the others), so it is
  // offerable; excluding it would hide a real measurement because of one bad row.
  EXPECT_TRUE(numeric.contains("depth_m"));
  // Free text on every feature: nothing for a ramp to read, so not offered.
  EXPECT_FALSE(numeric.contains("assessment"))
      << "a string-only field was offered as a colour/size ramp";

  EXPECT_TRUE(std::is_sorted(numeric.begin(), numeric.end()));

  delete layer;
}

// [camp#22] A colour field NO feature has a number for falls back to UNSTYLED,
// not to "everything is no-data".
//
// The menus cannot produce this any more, but a persisted style can: the style
// group is keyed on the file path and restored whenever that path is reopened,
// and the file on disk may have changed. Marking the whole layer no-data is the
// wrong answer — no-data means "this feature lacks a value its neighbours have",
// and here there is no ramp for anything to be missing from.
TEST(VectorLayerStyleFields, AColorFieldWithNoNumbersFallsBackToUnstyled)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString path = writeGeoJson(dir);
  ASSERT_FALSE(path.isEmpty());

  camp::map::Map map;
  auto* layer = new camp::vector::VectorLayer(map.topLevelLayers(), path);
  ASSERT_TRUE(waitForLoad(layer)) << "load did not complete: " << layer->status().toStdString();

  // Style by a real field first, so the fallback has something to undo: the
  // feature with no "signal" IS no-data here, which is the correct answer there.
  layer->setColorField("signal");
  bool saw_no_data = false;
  for(QGraphicsItem* child : layer->childItems())
    if(auto* feature = dynamic_cast<camp::vector::VectorFeatureItem*>(child))
      saw_no_data = saw_no_data || feature->isNoData();
  ASSERT_TRUE(saw_no_data) << "harness: the numeric field should flag one feature";

  // Now the string-only field. Every feature must come back unstyled.
  layer->setColorField("assessment");
  int checked = 0;
  for(QGraphicsItem* child : layer->childItems())
  {
    auto* feature = dynamic_cast<camp::vector::VectorFeatureItem*>(child);
    if(!feature)
      continue;
    ++checked;
    EXPECT_FALSE(feature->isNoData())
        << "a field no feature has a number for marked the whole layer no-data";
    EXPECT_EQ(feature->color(), QColor(Qt::darkCyan))
        << "the fallback must be the layer's default colour, not the no-data grey";
    EXPECT_NE(feature->color(), camp::vector::noDataColor());
  }
  EXPECT_EQ(checked, 5);

  // The setting itself is kept — the operator chose it, and it is what would be
  // persisted; only its EFFECT on this data is nothing.
  EXPECT_EQ(layer->colorField(), QStringLiteral("assessment"));

  delete layer;
}

// [camp#22] A persisted field the file no longer offers is SHOWN in the menu,
// and can be cleared there.
//
// The style group is keyed on the file path, so the same path reopened over a
// changed file can restore a colour or size field that `numericFields()` no
// longer lists. `applyStyle()` already paints that correctly (unstyled, above) —
// but the submenus were built ONLY when `numericFields()` was non-empty and only
// FROM that list, so a stale field appeared nowhere: no "(none)" to clear it
// with, no sign of what was set, and `writeSettings()` kept re-persisting it. The
// setting was stuck. It is shown rather than cleared on load because the operator
// chose it and the file may read as numeric again next time.
TEST(VectorLayerStyleFields, AStaleStyleFieldIsShownInTheMenuAndCanBeCleared)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString path = writeGeoJson(dir);
  ASSERT_FALSE(path.isEmpty());

  camp::map::Map map;
  auto* layer = new MenuProbe(map.topLevelLayers(), path);
  ASSERT_TRUE(waitForLoad(layer)) << "load did not complete: " << layer->status().toStdString();
  ASSERT_FALSE(layer->numericFields().contains("assessment"))
      << "harness: 'assessment' must be a field no ramp can read";

  // What a restored setting over a changed file looks like.
  layer->setColorField("assessment");
  layer->setSizeField("assessment");

  QMenu menu;
  layer->contextMenu(&menu);

  const QStringList color_entries = submenuEntries(menu, QStringLiteral("Color by"));
  ASSERT_FALSE(color_entries.isEmpty()) << "no Color by submenu: the setting is unreachable";
  EXPECT_TRUE(color_entries.contains(QStringLiteral("(none)")))
      << "nothing in the menu clears the stale field";
  EXPECT_TRUE(color_entries.contains(QStringLiteral("assessment (no numbers)")))
      << "the stale field is not named, so the operator cannot see what is set";

  const QStringList size_entries = submenuEntries(menu, QStringLiteral("Size by"));
  EXPECT_TRUE(size_entries.contains(QStringLiteral("(none)")));
  EXPECT_TRUE(size_entries.contains(QStringLiteral("assessment (no numbers)")));

  // The stale entry reads as the setting in force, and "(none)" as not-chosen.
  QAction* stale = submenuAction(menu, QStringLiteral("Color by"),
                                 QStringLiteral("assessment (no numbers)"));
  ASSERT_NE(stale, nullptr);
  EXPECT_TRUE(stale->isChecked());
  QAction* none = submenuAction(menu, QStringLiteral("Color by"), QStringLiteral("(none)"));
  ASSERT_NE(none, nullptr);
  EXPECT_FALSE(none->isChecked());

  // And "(none)" actually clears it — the whole point of showing the submenu.
  none->trigger();
  EXPECT_TRUE(layer->colorField().isEmpty()) << "the stale colour field could not be cleared";
  QAction* none_size = submenuAction(menu, QStringLiteral("Size by"), QStringLiteral("(none)"));
  ASSERT_NE(none_size, nullptr);
  none_size->trigger();
  EXPECT_TRUE(layer->sizeField().isEmpty()) << "the stale size field could not be cleared";

  // Once cleared, the menu is the ordinary one again: no stale entry left behind.
  QMenu after;
  layer->contextMenu(&after);
  const QStringList cleared = submenuEntries(after, QStringLiteral("Color by"));
  EXPECT_FALSE(cleared.contains(QStringLiteral("assessment (no numbers)")));
  EXPECT_TRUE(cleared.contains(QStringLiteral("signal")));

  delete layer;
}

// [camp#22] The combination the two-part guard in contextMenu() is most likely
// to regress on silently: numericFields() EMPTY (no field a ramp can read at
// all) AND a stale style field set. `field_names.isEmpty() && !stale_color &&
// !stale_size` is the only early return — if a future edit dropped either
// `stale_color`/`stale_size` term, or reordered the guard, this is the file
// that would go back to offering no submenu at all with the setting stuck
// (ADR-0016 D14's original defect), exactly when there is also nothing else in
// the menu to notice its absence next to.
TEST(VectorLayerStyleFields, AStaleFieldIsShownEvenWhenNumericFieldsIsEmpty)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString path = writeStringOnlyGeoJson(dir);
  ASSERT_FALSE(path.isEmpty());

  camp::map::Map map;
  auto* layer = new MenuProbe(map.topLevelLayers(), path);
  ASSERT_TRUE(waitForLoad(layer)) << "load did not complete: " << layer->status().toStdString();
  ASSERT_TRUE(layer->numericFields().isEmpty())
      << "harness: this file must offer no field a ramp can read";

  // What a restored setting over a now-all-text file looks like.
  layer->setColorField("assessment");

  QMenu menu;
  layer->contextMenu(&menu);

  const QStringList color_entries = submenuEntries(menu, QStringLiteral("Color by"));
  ASSERT_FALSE(color_entries.isEmpty())
      << "numericFields() being empty must not suppress the submenu: the stale "
         "setting would be unreachable and stuck";
  EXPECT_TRUE(color_entries.contains(QStringLiteral("(none)")));
  EXPECT_TRUE(color_entries.contains(QStringLiteral("assessment (no numbers)")));

  // No colormap menu: a palette with no field a ramp can sample changes nothing.
  bool saw_colormap = false;
  for(QAction* action : menu.actions())
    if(action->menu() && action->text() == QStringLiteral("Colormap"))
      saw_colormap = true;
  EXPECT_FALSE(saw_colormap);

  QAction* none = submenuAction(menu, QStringLiteral("Color by"), QStringLiteral("(none)"));
  ASSERT_NE(none, nullptr);
  none->trigger();
  EXPECT_TRUE(layer->colorField().isEmpty()) << "the stale colour field could not be cleared";

  delete layer;
}

// [camp#22 / ADR-0016 D5] A HOVER delivered THROUGH A REAL VIEW onto a real
// loaded feature shows that feature's attributes as an IN-SCENE LABEL — and a
// PRESS over the same feature is not taken by any item, so the view's pan
// gesture survives.
//
// Every other interaction test in this repo talks to the item directly, which
// skips everything between the operator's mouse and it: the viewport widget, the
// view's y-flip (CAMP's map view is scaled (1, -1) so north is up), and the
// scene's hit test against shape(). The operator's GUI test of 2026-09-15
// reported never seeing a popup with all of that in the path, so the regression
// test has to have it in the path too.
//
// What is synthesized here is an ordinary mouse MOVE with no buttons held, which
// is what the operator's hand produces: QGraphicsView turns it into a scene mouse
// move, and QGraphicsScene dispatches hover enter/leave from that to the item
// under the cursor (there is no mouse grabber, so nothing intercepts it).
// QGraphicsView enables mouse tracking on its viewport itself, so a move arrives
// with no button pressed. This is deliberately NOT a QHelpEvent any more: the
// popup is no longer a tooltip, and the operator's complaint about the tooltip
// version was precisely that it waited.
//
// The off-centre case is the point of the test: 3 px of aim is what an operator
// actually has, and the cursor must not have to be dead centre on a 5 px marker.
TEST(VectorLayerInteraction, HoverThroughARealViewShowsTheAttributes)
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
  view.setDragMode(QGraphicsView::ScrollHandDrag);   // pan mode, as CAMP idles in
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

  // The label is an ordinary child text item, created on the first hover.
  auto labelText = [target]() -> QString
  {
    for(QGraphicsItem* child : target->childItems())
      if(auto* text = dynamic_cast<QGraphicsSimpleTextItem*>(child))
        return text->text();
    return QString();
  };

  auto moveTo = [&view](const QPoint& viewport_pos)
  {
    const QPointF global = view.viewport()->mapToGlobal(viewport_pos);
    QMouseEvent move(QEvent::MouseMove, QPointF(viewport_pos), global,
                     Qt::NoButton, Qt::NoButton, Qt::NoModifier);
    QApplication::sendEvent(view.viewport(), &move);
    QCoreApplication::processEvents();
  };

  // Dead centre.
  moveTo(centre);
  EXPECT_EQ(labelText(), target->attributeText())
      << "hovering the marker, through a real view, showed no attributes";

  // 3 px off-centre: well inside the target, and the kind of aim the operator
  // actually has.
  moveTo(centre + QPoint(3, 2));
  EXPECT_EQ(labelText(), target->attributeText())
      << "hovering 3 px off centre missed the feature";

  // Moving clear of every feature clears the label — this is what keeps exactly
  // one label on screen, and what a missing hoverLeaveEvent() would break by
  // leaving a trail of attribute text behind the cursor.
  moveTo(centre + QPoint(60, 40));
  EXPECT_TRUE(labelText().isEmpty())
      << "the label survived the cursor leaving the feature";

  delete layer;
}

// [camp#22 / camp#225] A left PRESS over a feature is not taken by any item, so
// the view's ScrollHandDrag gets it and a pan that starts on a feature pans.
//
// This is the whole of camp#225's fix: the item accepts no mouse button
// (VectorFeatureItem.AcceptsNoMouseButtonSoThePressReachesTheView), so there is
// nothing to gate and nothing to get wrong. Asserted through the real view,
// because what is being claimed is about the scene's dispatch, not about a flag:
// after the press the scene has NO mouse grabber, which is exactly the state in
// which QGraphicsView keeps the gesture for itself.
TEST(VectorLayerInteraction, APressOverAFeatureFallsThroughToTheView)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString path = writeTwoPoints(dir);
  ASSERT_FALSE(path.isEmpty());

  camp::map::Map map;
  auto* layer = new camp::vector::VectorLayer(map.topLevelLayers(), path);
  ASSERT_TRUE(waitForLoad(layer)) << "load did not complete: " << layer->status().toStdString();

  QGraphicsView view(map.scene());
  view.setDragMode(QGraphicsView::ScrollHandDrag);
  view.scale(1.0, -1.0);
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

  view.centerOn(target->scenePos());
  QCoreApplication::processEvents();
  const QPoint centre = view.mapFromScene(target->scenePos());

  // Sanity: the press really is over the feature — the same hit test the hover
  // dispatch uses picks it. Without this the assertion below would pass on a miss.
  // (`items()`, not `itemAt()`: the topmost item at that point may be the layer
  // itself or a neighbouring feature; what matters is that this feature is under
  // the cursor and still does not take the press.)
  ASSERT_TRUE(view.items(centre).contains(static_cast<QGraphicsItem*>(target)))
      << "harness: the press point is not over the feature";

  const QPointF global = view.viewport()->mapToGlobal(centre);
  QMouseEvent press(QEvent::MouseButtonPress, QPointF(centre), global,
                    Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
  QApplication::sendEvent(view.viewport(), &press);
  QCoreApplication::processEvents();

  EXPECT_EQ(map.scene()->mouseGrabberItem(), nullptr)
      << "a feature grabbed the press: the view's pan gesture never starts (camp#225)";

  QMouseEvent release(QEvent::MouseButtonRelease, QPointF(centre), global,
                      Qt::LeftButton, Qt::NoButton, Qt::NoModifier);
  QApplication::sendEvent(view.viewport(), &release);
  QCoreApplication::processEvents();

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
