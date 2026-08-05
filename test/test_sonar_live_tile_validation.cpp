// [camp#170] Headless regression test: incoming coverage-tile dimensions are
// validated BEFORE allocation. During the 2026-07-23 deployment camp crashed
// at the operator station under sustained tile load; the most direct path is
// that applyPatch allocates width*height floats per band straight from the
// message fields — a single oversized or corrupt message is an unbounded
// GUI-thread allocation on receipt (the ADR-0010 eviction budget only counts
// tiles after residency). handleTile must reject absurd width/height/band
// counts (and a combined byte ceiling) with a warning, and still accept
// legitimate tiles. handleTile is a private slot: invoked by name via
// QMetaObject (DirectConnection), mirroring the ROS callback's GUI-thread
// marshal. Offscreen QApplication + Map harness; GL-free, ROS-free.

#include <gtest/gtest.h>

#include <cstdint>

#include <QApplication>
#include <QDir>
#include <QMetaObject>
#include <QSettings>
#include <QTemporaryDir>

#include "marine_autonomy/gggs.h"
#include "marine_interfaces/msg/sonar_visualization_tile.hpp"
#include "marine_interfaces/msg/visualization_band.hpp"

#include "map/map.h"
#include "map/layer_list.h"
#include "ros/live_coverage/sonar_live_cache_layer.h"
#include "ros/live_coverage/sonar_live_tile.h"

namespace mi = marine_interfaces::msg;
using camp::map::Map;
using camp::ros::live_coverage::SonarLiveCacheLayer;
using camp::ros::live_coverage::tileIndexFromGridIndex;

namespace
{

constexpr int kLevel = 10;
constexpr int kEdge = 8;

// A minimal full-tile INT16 depth patch for @p grid with explicit dimensions.
// Band data is sized for kEdge*kEdge regardless of the claimed width/height —
// for rejection cases the message must be cheap to build (the whole point is
// that the layer must reject BEFORE trusting the claimed dimensions).
mi::SonarVisualizationTile makeTile(const gggs::GridIndex& grid,
                                    std::uint16_t width, std::uint16_t height,
                                    std::size_t band_count = 1)
{
  mi::SonarVisualizationTile msg;
  msg.header.stamp.sec = 100;
  msg.header.frame_id = "gggs";
  msg.index = tileIndexFromGridIndex(grid);
  msg.width = width;
  msg.height = height;
  msg.window_col = 0;
  msg.window_row = 0;
  msg.window_width = width;
  msg.window_height = height;

  for(std::size_t b = 0; b < band_count; ++b)
  {
    mi::VisualizationBand band;
    band.name = b == 0 ? "depth" : ("band" + std::to_string(b));
    band.dtype = mi::VisualizationBand::INT16;
    band.scale = 0.01;
    band.offset = 0.0;
    band.nodata = -32768.0;
    for(int i = 0; i < kEdge * kEdge; ++i)
    {
      band.data.push_back(static_cast<std::uint8_t>(0x2c));
      band.data.push_back(static_cast<std::uint8_t>(0x01));
    }
    msg.bands.push_back(band);
  }
  return msg;
}

bool deliverTile(SonarLiveCacheLayer* layer, const mi::SonarVisualizationTile& msg)
{
  return QMetaObject::invokeMethod(layer, "handleTile", Qt::DirectConnection,
                                   Q_ARG(marine_interfaces::msg::SonarVisualizationTile, msg));
}

class SonarLiveTileValidation: public ::testing::Test
{
protected:
  void SetUp() override
  {
    QSettings().clear();
    ASSERT_TRUE(cache_.isValid());
    QSettings().setValue("LiveTileCache/cache_dir", cache_.path());
    map_ = std::make_unique<Map>();
    auto layers = map_->topLevelLayers();
    ASSERT_NE(layers, nullptr);
    layer_ = new SonarLiveCacheLayer(layers, nullptr, "/tile_validation_test");
    layer_->enableLiveCoverage();
    grid_ = gggs::Level(kLevel).gridIndex(43.0, -70.5);
    ASSERT_TRUE(grid_.valid());
  }

  QTemporaryDir cache_;
  std::unique_ptr<Map> map_;
  SonarLiveCacheLayer* layer_ = nullptr;
  gggs::GridIndex grid_;
};

}  // namespace

TEST_F(SonarLiveTileValidation, RejectsOversizedEdge)
{
  // 8192 > kMaxImageEdge (4096): rejected before any allocation.
  ASSERT_TRUE(deliverTile(layer_, makeTile(grid_, 8192, kEdge)));
  EXPECT_EQ(layer_->residentTileCount(), std::size_t(0));
  ASSERT_TRUE(deliverTile(layer_, makeTile(grid_, kEdge, 8192)));
  EXPECT_EQ(layer_->residentTileCount(), std::size_t(0));
}

TEST_F(SonarLiveTileValidation, RejectsZeroDimensionsAndAbsurdBandCount)
{
  ASSERT_TRUE(deliverTile(layer_, makeTile(grid_, 0, kEdge)));
  EXPECT_EQ(layer_->residentTileCount(), std::size_t(0));
  ASSERT_TRUE(deliverTile(layer_, makeTile(grid_, kEdge, 0)));
  EXPECT_EQ(layer_->residentTileCount(), std::size_t(0));
  // 65 bands > kMaxBandCount (64).
  ASSERT_TRUE(deliverTile(layer_, makeTile(grid_, kEdge, kEdge, 65)));
  EXPECT_EQ(layer_->residentTileCount(), std::size_t(0));
}

TEST_F(SonarLiveTileValidation, RejectsCombinedByteCeiling)
{
  // Each edge within the 4096 cap and bands within the 64 cap, but combined
  // 4096*4096*16*4B = 1 GiB > the 256 MiB ceiling.
  ASSERT_TRUE(deliverTile(layer_, makeTile(grid_, 4096, 4096, 16)));
  EXPECT_EQ(layer_->residentTileCount(), std::size_t(0));
}

TEST_F(SonarLiveTileValidation, AcceptsLegitimateTile)
{
  ASSERT_TRUE(deliverTile(layer_, makeTile(grid_, kEdge, kEdge)));
  EXPECT_EQ(layer_->residentTileCount(), std::size_t(1));
}

int main(int argc, char** argv)
{
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(argc, argv);
  QCoreApplication::setOrganizationName("camp_test");
  QCoreApplication::setApplicationName("test_sonar_live_tile_validation");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
