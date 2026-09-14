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

#include <gtest/gtest.h>

#include <memory>

#include <gdal_priv.h>

#include <QApplication>
#include <QElapsedTimer>
#include <QFile>
#include <QTemporaryDir>

#include "map/layer_list.h"
#include "map/map.h"
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
  ASSERT_GT(full_ms, 200) << "fixture is too small to distinguish an aborted parse";

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
  EXPECT_LT(abort_ms, full_ms / 2)
      << "aborted teardown took " << abort_ms << " ms against a full parse of " << full_ms
      << " ms — the abort flag is not being polled inside the parse";
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
