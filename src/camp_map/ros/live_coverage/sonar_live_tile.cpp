#include "sonar_live_tile.h"

#include <gdal_priv.h>
#include <ogr_spatialref.h>
#include <cpl_string.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <limits>

namespace camp
{
namespace ros
{
namespace live_coverage
{

namespace
{

// Byte width of a VisualizationBand dtype (mirrors the constants in the .msg).
int dtypeBytes(std::uint8_t dtype)
{
  using VB = marine_interfaces::msg::VisualizationBand;
  switch(dtype)
  {
    case VB::UINT8:  return 1;
    case VB::INT16:  return 2;
    case VB::UINT16: return 2;
    default:         return 0;   // unknown -> caller skips the band
  }
}

// Read one little-endian raw cell at element index @p i from a packed band buffer
// as a double (its RAW, pre-dequantization value), per the band's dtype.
double readRaw(const std::vector<std::uint8_t>& data, std::size_t i, std::uint8_t dtype)
{
  using VB = marine_interfaces::msg::VisualizationBand;
  switch(dtype)
  {
    case VB::UINT8:
      return static_cast<double>(data[i]);
    case VB::INT16:
    {
      const std::uint16_t lo = data[2 * i];
      const std::uint16_t hi = data[2 * i + 1];
      return static_cast<double>(static_cast<std::int16_t>(lo | (hi << 8)));
    }
    case VB::UINT16:
    {
      const std::uint16_t lo = data[2 * i];
      const std::uint16_t hi = data[2 * i + 1];
      return static_cast<double>(static_cast<std::uint16_t>(lo | (hi << 8)));
    }
    default:
      return 0.0;
  }
}

}  // namespace

SonarLiveTile::SonarLiveTile(const gggs::GridIndex& index, int width, int height):
  index_(index), width_(width), height_(height)
{
}

void SonarLiveTile::applyPatch(const marine_interfaces::msg::SonarVisualizationTile& msg)
{
  // [camp#121] Generic dequantize: value = raw * scale + offset, no per-band
  // hardcoded knowledge (uma ADR-0008 D1). The dirty sub-window is patched into
  // the band at the GGGS cell offset; GGGS cell order has row 0 = south, while we
  // hold north-up (row 0 = north) to match the GggsTile-derived render path, so
  // the destination row is flipped.
  const int wc = msg.window_col;
  const int wr = msg.window_row;
  const int ww = msg.window_width;
  const int wh = msg.window_height;

  // Reject a window that doesn't fit the tile rather than writing out of bounds.
  if(wc < 0 || wr < 0 || ww < 0 || wh < 0 ||
     wc + ww > width_ || wr + wh > height_)
    return;

  for(const auto& vb : msg.bands)
  {
    const int width_bytes = dtypeBytes(vb.dtype);
    if(width_bytes == 0)
      continue;   // unknown dtype
    const std::size_t cell_count = static_cast<std::size_t>(ww) * wh;
    if(vb.data.size() != cell_count * width_bytes)
      continue;   // declared window doesn't match the payload length

    // [camp#208] NoData is normalised to NaN, NOT kept as the dequantized
    // producer sentinel. Bands each carry their own sentinel on the wire
    // (quantization needs an in-range integer), but GeoTIFF's TIFFTAG_GDAL_NODATA
    // stores ONE value per dataset — so writing three different per-band
    // sentinels silently kept only the last and applied it to every band. On
    // reload the other bands then had no recognised NoData, their empty cells
    // read back as real data, and a mostly-empty tile painted solid over the
    // chart (an 8x8 degree apex tile covering New England, in the case that
    // found this).
    //
    // NaN sidesteps the one-value limit entirely: every band's sentinel is the
    // same value, so the tag cannot be lossy, and NaN needs no tag at all to be
    // recognised. This is also what the world store already does
    // (marine_bathymetry_store s102/convert.cpp writes SetNoDataValue(nan)), so
    // this brings the live cache into line with that convention rather than
    // inventing a second one.
    //
    // Consumers are unaffected: every reader here guards with
    // `!std::isfinite(v) || (has_nodata && v == nodata)`, and the !isfinite
    // clause catches NaN — an `== nodata` test alone never would, since NaN
    // compares unequal to itself.
    const float nodata_value = std::numeric_limits<float>::quiet_NaN();

    SonarLiveBand& band = bands_[vb.name];
    if(band.data.empty())
    {
      band.name = vb.name;
      band.data.assign(static_cast<std::size_t>(width_) * height_, nodata_value);
    }
    band.has_nodata = true;
    band.nodata = nodata_value;

    for(int r = 0; r < wh; ++r)
    {
      const int north_row = (height_ - 1) - (wr + r);   // GGGS south-up -> north-up
      for(int c = 0; c < ww; ++c)
      {
        const std::size_t src = static_cast<std::size_t>(r) * ww + c;
        const double raw = readRaw(vb.data, src, vb.dtype);
        const std::size_t dst =
          static_cast<std::size_t>(north_row) * width_ + (wc + c);
        band.data[dst] = (raw == vb.nodata)
                           ? nodata_value
                           : static_cast<float>(raw * vb.scale + vb.offset);
      }
    }
    refoldRange(band);
  }

  version_ = std::max(version_, toNanoseconds(msg.header));
}

void SonarLiveTile::foldChild(const SonarLiveTile& child)
{
  // [camp#160] Fold a finer child into this coarser parent's matching sub-window for
  // the overview pyramid. Both tiles are the same width x height (a parent cell
  // therefore spans ~2x2 child cells). We area-map each child cell to the parent cell
  // containing its geographic centre and average the finite, non-NoData samples landing
  // in each parent cell — north-up, so row 0 is the northernmost row. Only cells the
  // child covers are written, so folding each of a parent's children in turn accumulates
  // the full parent.
  // [camp#171] This is the same geographic-centre + MEAN cell fold as the uma shared
  // fold engine (marine_tiled_raster_store overview_builder.hpp buildParentTile<float>,
  // imagery = mean policy) at the fixed uniform TiledRasterTile::edge — the live overview
  // pyramid CONVERGES with the merged store's pyramid (ADR-0010 D3). The same-size
  // parent/child is required here (each child fills a 1/4 sub-window), which is why the
  // caller must build the parent at the child's width/height, not a decimated size.
  if(width_ <= 0 || height_ <= 0 || child.width_ <= 0 || child.height_ <= 0)
    return;
  const double parent_lon0 = minLon();
  const double parent_lat1 = maxLat();
  const double parent_lon_span = maxLon() - parent_lon0;
  const double parent_lat_span = parent_lat1 - minLat();
  if(parent_lon_span <= 0.0 || parent_lat_span <= 0.0)
    return;
  const double child_lon0 = child.minLon();
  const double child_lat1 = child.maxLat();
  const double child_lon_span = child.maxLon() - child_lon0;
  const double child_lat_span = child_lat1 - child.minLat();

  const std::size_t cells = static_cast<std::size_t>(width_) * height_;
  for(const auto& [name, cband] : child.bands_)
  {
    SonarLiveBand& pband = bands_[name];
    if(pband.data.empty())
    {
      // Uncovered parent cells stay transparent: the child's NoData sentinel if
      // it has one, else NaN (both are discarded by the renderer, camp#134).
      const float fill = cband.has_nodata ? cband.nodata
                                          : std::numeric_limits<float>::quiet_NaN();
      pband.name = name;
      pband.data.assign(cells, fill);
      pband.has_nodata = cband.has_nodata;
      pband.nodata = cband.nodata;
    }

    std::vector<double> sum(cells, 0.0);
    std::vector<std::uint32_t> count(cells, 0);
    for(int cr = 0; cr < child.height_; ++cr)
    {
      const double lat = child_lat1 - ((cr + 0.5) / child.height_) * child_lat_span;
      const int pr = static_cast<int>((parent_lat1 - lat) / parent_lat_span * height_);
      if(pr < 0 || pr >= height_)
        continue;
      for(int cc = 0; cc < child.width_; ++cc)
      {
        const float v = cband.data[static_cast<std::size_t>(cr) * child.width_ + cc];
        if(!std::isfinite(v) || (cband.has_nodata && v == cband.nodata))
          continue;
        const double lon = child_lon0 + ((cc + 0.5) / child.width_) * child_lon_span;
        const int pc = static_cast<int>((lon - parent_lon0) / parent_lon_span * width_);
        if(pc < 0 || pc >= width_)
          continue;
        const std::size_t idx = static_cast<std::size_t>(pr) * width_ + pc;
        sum[idx] += v;
        count[idx] += 1;
      }
    }
    for(std::size_t i = 0; i < cells; ++i)
      if(count[i] > 0)
        pband.data[i] = static_cast<float>(sum[i] / count[i]);
    refoldRange(pband);
  }
}

void SonarLiveTile::refoldRange(SonarLiveBand& band)
{
  // A patch can overwrite a former extreme cell, so re-fold the whole band rather
  // than only widening — the tile is small (one GGGS grid) and patches arrive at
  // survey rates, so the full scan is cheap and always correct.
  float lo = std::numeric_limits<float>::max();
  float hi = std::numeric_limits<float>::lowest();
  bool any = false;
  for(float v : band.data)
  {
    if(!std::isfinite(v) || (band.has_nodata && v == band.nodata))
      continue;
    lo = std::min(lo, v);
    hi = std::max(hi, v);
    any = true;
  }
  if(any)
  {
    band.data_min = lo;
    band.data_max = hi;
  }
  else
  {
    band.data_min = 1.0f;   // crossed: no valid samples
    band.data_max = 0.0f;
  }
}

const SonarLiveBand* SonarLiveTile::band(const std::string& name) const
{
  auto it = bands_.find(name);
  return it == bands_.end() ? nullptr : &it->second;
}

std::vector<std::string> SonarLiveTile::bandNames() const
{
  std::vector<std::string> names;
  names.reserve(bands_.size());
  for(const auto& entry : bands_)
    names.push_back(entry.first);
  return names;
}

std::optional<SonarLiveTile> SonarLiveTile::loadFromGeoTiff(const std::string& path,
                                                            const gggs::Level& level)
{
  if(GDALGetDriverCount() == 0)
    GDALAllRegister();

  auto* dataset = GDALDataset::FromHandle(GDALOpen(path.c_str(), GA_ReadOnly));
  if(!dataset)
    return std::nullopt;

  const int width = dataset->GetRasterXSize();
  const int height = dataset->GetRasterYSize();
  const int band_count = dataset->GetRasterCount();
  double geo[6];
  if(width <= 0 || height <= 0 || band_count < 1 ||
     dataset->GetGeoTransform(geo) != CE_None)
  {
    GDALClose(dataset);
    return std::nullopt;
  }

  // Require a geographic WGS84 raster: the geotransform is read as degrees to
  // recover the GridIndex (same guard as marine_tiled_raster_store::loadTile).
  const OGRSpatialReference* srs = dataset->GetSpatialRef();
  OGRErr axis_error = OGRERR_NONE;
  if(srs == nullptr || !srs->IsGeographic() ||
     std::abs(srs->GetSemiMajor(&axis_error) - 6378137.0) > 1.0 ||
     axis_error != OGRERR_NONE)
  {
    GDALClose(dataset);
    return std::nullopt;
  }

  const double west = geo[0];
  const double pixel_x = geo[1];
  const double north = geo[3];
  const double pixel_y = geo[5];
  const double east = west + pixel_x * width;
  const double south = north + pixel_y * height;

  // Recover the GridIndex: the cell center maps back through the level.
  gggs::GridIndex index;
  try
  {
    index = level.gridIndex(0.5 * (north + south), 0.5 * (west + east));
  }
  catch(const std::exception&)
  {
    GDALClose(dataset);
    return std::nullopt;
  }
  const double tol_lon = 0.5 * std::abs(pixel_x);
  const double tol_lat = 0.5 * std::abs(pixel_y);
  if(!index.valid() ||
     std::abs(index.westLongitude() - west) > tol_lon ||
     std::abs(index.northLatitude() - north) > tol_lat)
  {
    GDALClose(dataset);
    return std::nullopt;
  }

  SonarLiveTile tile(index, width, height);
  const std::size_t cells = static_cast<std::size_t>(width) * height;
  for(int b = 1; b <= band_count; ++b)
  {
    GDALRasterBand* raster = dataset->GetRasterBand(b);
    SonarLiveBand band;
    const char* desc = raster->GetDescription();
    band.name = (desc && desc[0] != '\0') ? desc : ("band" + std::to_string(b));
    int has_nodata = 0;
    const double nodata = raster->GetNoDataValue(&has_nodata);
    band.has_nodata = has_nodata != 0;
    band.nodata = static_cast<float>(nodata);
    band.data.resize(cells);
    if(raster->RasterIO(GF_Read, 0, 0, width, height, band.data.data(),
                        width, height, GDT_Float32, 0, 0) != CE_None)
    {
      GDALClose(dataset);
      return std::nullopt;
    }
    refoldRange(band);
    tile.bands_[band.name] = std::move(band);
  }
  GDALClose(dataset);
  return tile;
}

std::vector<SonarLiveTile> SonarLiveTile::loadCacheDir(const std::string& dir,
                                                       const gggs::Level& level)
{
  std::vector<SonarLiveTile> tiles;
  std::error_code ec;
  if(!std::filesystem::is_directory(dir, ec))
    return tiles;
  for(const auto& entry : std::filesystem::directory_iterator(dir, ec))
  {
    if(ec)
      break;
    if(!entry.is_regular_file())
      continue;
    if(entry.path().extension() != ".tif")
      continue;   // skip `.tif.tmp` crash-partials and non-tile files
    if(auto tile = loadFromGeoTiff(entry.path().string(), level))
      tiles.push_back(std::move(*tile));
  }
  return tiles;
}

bool SonarLiveTile::writeToGeoTiff(const std::string& path) const
{
  if(bands_.empty() || width_ <= 0 || height_ <= 0)
    return false;
  if(GDALGetDriverCount() == 0)
    GDALAllRegister();

  GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("GTiff");
  if(!driver)
    return false;

  char** options = nullptr;
  options = CSLSetNameValue(options, "COMPRESS", "LZW");
  GDALDataset* out = driver->Create(path.c_str(), width_, height_,
                                    static_cast<int>(bands_.size()), GDT_Float32,
                                    options);
  CSLDestroy(options);
  if(!out)
    return false;

  // North-up GGGS geotransform for this tile's grid (the GggsTile / saveTile
  // convention). Memory is already north-up (row 0 = north), so bands write
  // straight through with no flip.
  const double west = index_.westLongitude();
  const double north = index_.northLatitude();
  const double pixel_x = index_.longitudinalSpan() / width_;
  const double pixel_y = -index_.latitudinalSpan() / height_;
  double geo[6] = {west, pixel_x, 0.0, north, 0.0, pixel_y};
  bool ok = out->SetGeoTransform(geo) == CE_None;

  OGRSpatialReference wgs84;
  wgs84.SetWellKnownGeogCS("WGS84");
  char* wkt = nullptr;
  if(ok && wgs84.exportToWkt(&wkt) == OGRERR_NONE && wkt)
    ok = out->SetProjection(wkt) == CE_None;
  else
    ok = false;
  CPLFree(wkt);

  int band_index = 1;
  for(auto it = bands_.begin(); ok && it != bands_.end(); ++it, ++band_index)
  {
    const SonarLiveBand& band = it->second;
    GDALRasterBand* raster = out->GetRasterBand(band_index);
    raster->SetDescription(band.name.c_str());
    // [camp#208] Always NaN, and the same for every band — see the normalisation
    // comment in applyPatch(). Writing per-band sentinels here is what lost the
    // transparency: GeoTIFF keeps only one.
    if(band.has_nodata)
      raster->SetNoDataValue(std::numeric_limits<double>::quiet_NaN());
    // Const buffer: GDAL's RasterIO takes a non-const void*, but GF_Write only
    // reads from it.
    std::vector<float> scratch(band.data);
    if(raster->RasterIO(GF_Write, 0, 0, width_, height_, scratch.data(),
                        width_, height_, GDT_Float32, 0, 0) != CE_None)
      ok = false;
  }

  if(GDALClose(out) != CE_None)
    ok = false;
  return ok;
}

// -------------------------- node-boundary conversions --------------------------

marine_tiled_raster_store::TileVersion toNanoseconds(
  const builtin_interfaces::msg::Time& time)
{
  return static_cast<marine_tiled_raster_store::TileVersion>(time.sec) * 1000000000LL +
         static_cast<marine_tiled_raster_store::TileVersion>(time.nanosec);
}

marine_tiled_raster_store::TileVersion toNanoseconds(const std_msgs::msg::Header& header)
{
  return toNanoseconds(header.stamp);
}

gggs::GridIndex gridIndexFromTileIndex(const marine_interfaces::msg::TileIndex& index)
{
  if(index.level >= gggs::levels.size())
    return gggs::GridIndex();   // invalid sentinel

  const gggs::LevelSpecs& spec = gggs::levels[index.level];
  // Cell-center lat/lon for (row, col), then round-trip through Level::gridIndex
  // (the GridIndex ctor is private; this is the only public construction path).
  const double grid_span = spec.grid_angular_span;
  const double south = -96.0 + static_cast<double>(index.row) * grid_span;
  const double center_lat = south + 0.5 * grid_span;
  const double lon_span = spec.gridLongitudinalSpan(index.row);
  const double west = -180.0 + static_cast<double>(index.col) * lon_span;
  const double center_lon = west + 0.5 * lon_span;

  try
  {
    const gggs::Level level(index.level);
    const gggs::GridIndex recovered = level.gridIndex(center_lat, center_lon);
    // Reject anything that doesn't round-trip exactly (e.g. an out-of-range row/
    // col that the truncating lookup would snap to a different real grid).
    if(recovered.valid() && recovered.level() == index.level &&
       recovered.row() == index.row && recovered.column() == index.col)
      return recovered;
  }
  catch(const std::exception&)
  {
  }
  return gggs::GridIndex();
}

marine_interfaces::msg::TileIndex tileIndexFromGridIndex(const gggs::GridIndex& index)
{
  marine_interfaces::msg::TileIndex out;
  out.level = index.level();
  out.row = index.row();
  out.col = index.column();
  return out;
}

marine_tiled_raster_store::TileCatalog toReconcilerCatalog(
  const marine_interfaces::msg::TileCatalog& catalog)
{
  marine_tiled_raster_store::TileCatalog out;
  out.generation_time = toNanoseconds(catalog.header);
  out.entries.reserve(catalog.entries.size());
  for(const auto& entry : catalog.entries)
  {
    const gggs::GridIndex index = gridIndexFromTileIndex(entry.index);
    if(!index.valid())
      continue;   // invalid indices are ignored at ingestion (reconciler contract)
    marine_tiled_raster_store::TileCatalogEntry out_entry;
    out_entry.index = index;
    out_entry.version = toNanoseconds(entry.version);
    out.entries.push_back(out_entry);
  }
  return out;
}

}  // namespace live_coverage
}  // namespace ros
}  // namespace camp
