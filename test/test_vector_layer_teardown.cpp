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

#include <gdal_priv.h>

#include <QApplication>
#include <QElapsedTimer>
#include <QFile>
#include <QTemporaryDir>

#include "map/layer_list.h"
#include "map/map.h"
#include "vector/vector_layer.h"

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
