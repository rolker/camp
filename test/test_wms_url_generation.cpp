// [camp#118] Pure-logic tests for the per-tile WMS GetMap path (ADR-0012):
// WMS_BBOX expansion against the known OSM tile math, template assembly with
// every required GetMap parameter, grid identity with the OSM layout, and
// cache-buster composability with a query-string URL. No network.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <string>

#include "map_tiles/osm.h"
#include "map_tiles/tile_address.h"
#include "map_tiles/tile_layout.h"
#include "map_tiles/wms.h"
#include "map_view/web_mercator.h"

using camp::map_tiles::TileAddress;
using camp::map_tiles::TileLayout;

namespace
{
// Web-Mercator half-circumference, derived from the same constant
// osm::generateTileLayout() uses rather than a repeated magic literal.
constexpr double kHalfEarth = web_mercator::earth_radius_at_equator * M_PI;
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

// [camp#178] The {TileMatrix} substitution must emit the zoom level's declared
// identifier, not the bare loop index. GeoServer GWC (BlueTopo, nowCOAST) names
// its tile matrices <gridset>:<z> and rejects a bare numeric with HTTP 400. The
// OSM layout is a faithful stand-in: it carries the same {TileMatrix} template
// and, like every bare-numeric server, sets each id to the numeric string.

// Bare-numeric servers (OSM/XYZ) declare id = "<z>", so the URL keeps the plain
// zoom index and stays byte-for-byte compatible after the fix.
TEST(WmsUrlGeneration, TileMatrixUsesBareNumericIdForOsm)
{
  TileLayout layout = camp::osm::generateTileLayout("https://example.org/");
  const std::string url = layout.getUrl(TileAddress(&layout, 3, QPoint(2, 5), 0));
  EXPECT_EQ(url, "https://example.org/3/2/5.png");
}

// GeoServer GWC declares id = "EPSG:3857:<z>"; the URL must carry that exact
// gridset-prefixed identifier so the tile matrix is recognized (the bug: the
// bare "3" produced "Unknown TILEMATRIX" / HTTP 400).
TEST(WmsUrlGeneration, TileMatrixUsesGridsetPrefixedId)
{
  TileLayout layout = camp::osm::generateTileLayout("https://example.org/");
  for(std::size_t z = 0; z < layout.zoom_levels.size(); ++z)
    layout.zoom_levels[z].id = "EPSG:3857:" + std::to_string(z);
  const std::string url = layout.getUrl(TileAddress(&layout, 3, QPoint(2, 5), 0));
  EXPECT_EQ(url, "https://example.org/EPSG:3857:3/2/5.png");
}

// An empty id (no <ows:Identifier> parsed) falls back to the bare numeric index
// so a capabilities document without explicit ids still yields usable URLs.
TEST(WmsUrlGeneration, TileMatrixFallsBackToNumericWhenIdEmpty)
{
  TileLayout layout = camp::osm::generateTileLayout("https://example.org/");
  for(auto& level : layout.zoom_levels)
    level.id.clear();
  const std::string url = layout.getUrl(TileAddress(&layout, 3, QPoint(2, 5), 0));
  EXPECT_EQ(url, "https://example.org/3/2/5.png");
}

// [camp#178 review R1] The OSM stand-in above sets id == numeric index, so it
// cannot distinguish "emit the declared id" from "emit the loop index". Give one
// level an id whose numeric tail deliberately differs from its position and
// assert the URL carries the id, not the index — proving id-over-index directly.
TEST(WmsUrlGeneration, TileMatrixPrefersIdOverIndexWhenMismatched)
{
  TileLayout layout = camp::osm::generateTileLayout("https://example.org/");
  layout.zoom_levels[3].id = "EPSG:3857:99";  // numeric tail 99 != index 3
  const std::string url = layout.getUrl(TileAddress(&layout, 3, QPoint(2, 5), 0));
  EXPECT_EQ(url, "https://example.org/EPSG:3857:99/2/5.png");
  EXPECT_EQ(url.find("/3/"), std::string::npos) << "must not fall back to the bare index";
}

// [camp#178 review R1] A server-controlled id is percent-encoded before it is
// spliced into the path, so a hostile <Identifier> cannot inject path or query
// delimiters. ':' is preserved (legal path-segment char, load-bearing for the
// EPSG:3857:<z> ids); '/' is encoded to %2F, neutralizing path traversal.
TEST(WmsUrlGeneration, TileMatrixIdIsPercentEncoded)
{
  TileLayout layout = camp::osm::generateTileLayout("https://example.org/");
  layout.zoom_levels[3].id = "EPSG:3857:3/x";
  const std::string url = layout.getUrl(TileAddress(&layout, 3, QPoint(2, 5), 0));
  EXPECT_EQ(url, "https://example.org/EPSG:3857:3%2Fx/2/5.png");
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
