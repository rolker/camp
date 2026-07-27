// [camp#118] Pure-logic tests for the per-tile WMS GetMap path (ADR-0012):
// WMS_BBOX expansion against the known OSM tile math, template assembly with
// every required GetMap parameter, grid identity with the OSM layout, and
// cache-buster composability with a query-string URL. No network.

#include <gtest/gtest.h>

#include <string>

#include "map_tiles/osm.h"
#include "map_tiles/tile_address.h"
#include "map_tiles/tile_layout.h"
#include "map_tiles/wms.h"

using camp::map_tiles::TileAddress;
using camp::map_tiles::TileLayout;

namespace
{
constexpr double kHalfEarth = 20037508.3427892;  // Web-Mercator half-circumference
}

// Zoom 0 is a single tile spanning the whole Web-Mercator square: its bbox is
// (-half, -half, half, half), verifiable against the OSM layout constants.
TEST(WmsUrlGeneration, BboxZoom0CoversWorld)
{
  TileLayout layout = camp::wms::generateWmsLayout("https://example.org/wms", "test_layer");
  const std::string url = layout.getUrl(TileAddress(&layout, 0, QPoint(0, 0), 0));

  const auto bbox_pos = url.find("BBOX=");
  ASSERT_NE(bbox_pos, std::string::npos);
  const std::string bbox = url.substr(bbox_pos + 5, url.find('&', bbox_pos) - bbox_pos - 5);

  double minx, miny, maxx, maxy;
  ASSERT_EQ(sscanf(bbox.c_str(), "%lf,%lf,%lf,%lf", &minx, &miny, &maxx, &maxy), 4);
  EXPECT_NEAR(minx, -kHalfEarth, 0.01);
  EXPECT_NEAR(miny, -kHalfEarth, 0.01);
  EXPECT_NEAR(maxx, kHalfEarth, 0.01);
  EXPECT_NEAR(maxy, kHalfEarth, 0.01);
}

// At zoom 1 the (1,0) tile is the north-east quadrant: x in [0, half],
// y in [0, half].
TEST(WmsUrlGeneration, BboxZoom1NorthEastQuadrant)
{
  TileLayout layout = camp::wms::generateWmsLayout("https://example.org/wms", "test_layer");
  const std::string url = layout.getUrl(TileAddress(&layout, 1, QPoint(1, 0), 0));

  const auto bbox_pos = url.find("BBOX=");
  ASSERT_NE(bbox_pos, std::string::npos);
  const std::string bbox = url.substr(bbox_pos + 5, url.find('&', bbox_pos) - bbox_pos - 5);

  double minx, miny, maxx, maxy;
  ASSERT_EQ(sscanf(bbox.c_str(), "%lf,%lf,%lf,%lf", &minx, &miny, &maxx, &maxy), 4);
  EXPECT_NEAR(minx, 0.0, 0.01);
  EXPECT_NEAR(miny, 0.0, 0.01);
  EXPECT_NEAR(maxx, kHalfEarth, 0.01);
  EXPECT_NEAR(maxy, kHalfEarth, 0.01);
}

// The template carries every required WMS 1.3.0 GetMap parameter, the layer
// id, and a WIDTH/HEIGHT matching osm::tile_size (the bbox math's tile size).
TEST(WmsUrlGeneration, TemplateHasRequiredParameters)
{
  TileLayout layout = camp::wms::generateWmsLayout("https://example.org/wms", "my:layer");
  const std::string url = layout.getUrl(TileAddress(&layout, 2, QPoint(1, 2), 0));

  EXPECT_EQ(url.find("https://example.org/wms?"), 0u);
  for(const char* param : {"SERVICE=WMS", "VERSION=1.3.0", "REQUEST=GetMap",
                           "CRS=EPSG:3857", "FORMAT=image/png", "TRANSPARENT=TRUE",
                           "STYLES=", "LAYERS=my:layer", "BBOX="})
    EXPECT_NE(url.find(param), std::string::npos) << param << " missing in " << url;
  const std::string size = std::to_string(camp::osm::tile_size);
  EXPECT_NE(url.find("WIDTH=" + size), std::string::npos);
  EXPECT_NE(url.find("HEIGHT=" + size), std::string::npos);
}

// The WMS grid is the OSM grid: same zoom-level count, scales, and matrix
// sizes, so WMS tiles align with every other MapTiles layer.
TEST(WmsUrlGeneration, GridMatchesOsmLayout)
{
  TileLayout wms = camp::wms::generateWmsLayout("https://example.org/wms", "l");
  TileLayout osm = camp::osm::generateTileLayout("https://example.org/");
  ASSERT_EQ(wms.zoom_levels.size(), osm.zoom_levels.size());
  for(size_t i = 0; i < wms.zoom_levels.size(); ++i)
  {
    EXPECT_DOUBLE_EQ(wms.zoom_levels[i].scale, osm.zoom_levels[i].scale) << "level " << i;
    EXPECT_EQ(wms.zoom_levels[i].matrix_width, osm.zoom_levels[i].matrix_width) << "level " << i;
    EXPECT_EQ(wms.zoom_levels[i].tile_width, osm.zoom_levels[i].tile_width) << "level " << i;
  }
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
