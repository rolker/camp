#include "raster_layer.h"
#include <gdal_priv.h>
#include <gdalwarper.h>
#include "../map_view/web_mercator.h"
#include "colormap_range_dialog.h"
#include <marine_colormap/palette.hpp>
#include <QPainter>
#include <QOpenGLTexture>
#include <QTransform>
#include <QtConcurrent>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <QMenu>
#include <QAction>
#include <QSettings>
#include <QFileInfo>

#include <QDebug>

namespace camp
{

namespace raster
{

RasterLayer::RasterLayer(map::MapItem* parentItem, const QString& filename):
  map::Layer(parentItem, QFileInfo(filename).fileName()),
  filename_(filename)
{
  if(GDALGetDriverCount() == 0)
    GDALAllRegister();
  // [camp#63] Default scalar ramp is viridis (the renderer defaults to grayscale).
  renderer_.setColormap("viridis");
  connect(&future_watcher_, &QFutureWatcher<LoadResult>::finished, this, &RasterLayer::imageReady);
  // [#59 ADR-0003] Establish the scene extent + world transform synchronously,
  // before kicking off the async pixel load, so the layer knows where it is
  // (valid boundingRect/scenePos) immediately — fit-to-extent / zoom-on-open
  // works at load time instead of only after the warp completes.
  initExtent(filename);
  loadFile(filename);
}

RasterLayer::~RasterLayer()
{
  abort_flag_mutex_.lock();
  abort_flag_ = true;
  abort_flag_mutex_.unlock();
  future_watcher_.waitForFinished();
  // [camp#134] Release the chart texture under the renderer's context (the texture
  // is owned here, not by the renderer). The renderer frees its own
  // program/LUT/FBO/context in its destructor right after this.
  if(renderer_.makeCurrent())
  {
    texture_.reset();
    renderer_.doneCurrent();
  }
}

QRectF RasterLayer::boundingRect() const
{
  // [camp#134] Local space spans (0,0)..(width_m,height_m) in Web-Mercator metres;
  // the item is setPos()'d at the NW corner with a fromScale(1,-1) transform (the
  // camp_map raster convention shared with GggsTileLayer).
  if(!scene_bounds_.isNull())
    return QRectF(QPointF(0.0, 0.0), scene_bounds_.size());
  return QRectF();
}


void RasterLayer::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
  if(scene_bounds_.isNull())
    return;
  if(is_scalar_ && data_min_ > data_max_)   // scalar: nothing valid to colour yet
    return;

  // Target the offscreen render at the extent's on-screen size, so the image is
  // crisp at the current zoom. Re-render only when that size changes (zoom); pan
  // reuses the cached image (drawImage repositions it via the world transform).
  const QRectF dev = painter->worldTransform().mapRect(boundingRect());
  const int w = std::min(kMaxImageEdge,
                         std::max(1, int(std::ceil(std::abs(dev.width())))));
  const int h = std::min(kMaxImageEdge,
                         std::max(1, int(std::ceil(std::abs(dev.height())))));
  const QSize size(w, h);

  if(cached_image_.isNull() || cached_size_ != size)
  {
    cached_image_ = renderImage(size);
    cached_size_ = size;
  }
  if(cached_image_.isNull())
    return;

  painter->save();
  painter->setRenderHint(QPainter::SmoothPixmapTransform);
  painter->drawImage(boundingRect(), cached_image_);
  painter->restore();
}


void RasterLayer::initExtent(const QString& filename)
{
  // [#59 ADR-0003] Read only the reprojected geotransform + dimensions (no
  // pixels) and apply the Web-Mercator scene placement synchronously, so the layer
  // has a valid extent and scene position before the async pixel load.
  // GDALAutoCreateWarpedVRT is metadata-only and effectively instant.
  auto dataset = GDALDataset::FromHandle(GDALOpen(filename.toLatin1(), GA_ReadOnly));
  if(!dataset)
    return;
  auto reprojected = GDALDataset::FromHandle(
    GDALAutoCreateWarpedVRT(dataset, nullptr, web_mercator::wkt, GRA_Bilinear, 0.0, nullptr));
  if(reprojected)
  {
    double geo_transform[6] = {0.0};
    reprojected->GetGeoTransform(geo_transform);
    reprojected_width_ = reprojected->GetRasterXSize();
    reprojected_height_ = reprojected->GetRasterYSize();
    // [camp#134] Web-Mercator extent from the reprojected geotransform (geo[2] ==
    // geo[4] == 0, geo[1] > 0, geo[5] < 0 — north-up): the NW corner is (geo[0],
    // geo[3]); the SE corner subtracts width/height. north = max y, south = min y.
    const double x0 = geo_transform[0];
    const double y0 = geo_transform[3];
    const double x1 = x0 + reprojected_width_ * geo_transform[1];
    const double y1 = y0 + reprojected_height_ * geo_transform[5];
    merc_x_min_ = std::min(x0, x1);
    merc_x_max_ = std::max(x0, x1);
    merc_y_min_ = std::min(y0, y1);
    merc_y_max_ = std::max(y0, y1);
    prepareGeometryChange();
    scene_bounds_ = QRectF(QPointF(merc_x_min_, merc_y_min_),
                           QPointF(merc_x_max_, merc_y_max_)).normalized();
    // North-up image anchored at the NW corner with a negative-Y transform (matches
    // GggsTileLayer); the MapView's own scale(s,-s) composes to a net upright draw.
    setTransform(QTransform::fromScale(1.0, -1.0));
    setPos(QPointF(scene_bounds_.left(), scene_bounds_.bottom()));   // NW corner
    GDALClose(reprojected);
  }
  GDALClose(dataset);
}

void RasterLayer::loadFile(const QString& filename)
{
  // Abort and join any in-flight load before starting a new one (e.g. a rapid
  // colormap change). setFuture() only tracks the latest future, so a replaced
  // job would otherwise become untracked: it would keep running, race the new
  // job on shared state, and — if the layer is destroyed first — outlive `this`
  // (use-after-free), since the dtor only waits on the watched future.
  if(future_watcher_.isRunning())
  {
    abort_flag_mutex_.lock();
    abort_flag_ = true;
    abort_flag_mutex_.unlock();
    future_watcher_.waitForFinished();
  }
  abort_flag_mutex_.lock();
  abort_flag_ = false;   // re-arm for the new job
  abort_flag_mutex_.unlock();

  setStatus("(loading...)");
  future_watcher_.setFuture(QtConcurrent::run(this, &RasterLayer::loadAndReprojectFile, filename));
}

RasterLayer::LoadResult RasterLayer::loadAndReprojectFile(const QString& filename)
{
  LoadResult result;

  // [#96] RAII-close both GDAL handles so every return path — normal, the
  // !reprojected_dataset early return, and the abort-flag `return {}` paths in
  // the scan loops below — frees them. Declaration order matters: local
  // unique_ptrs destruct in reverse declaration order, so `dataset` MUST stay
  // declared before `reprojected_dataset` — the warped VRT references the source
  // dataset, so it has to close first, then the source. Do not reorder.
  const auto gdal_closer = [](GDALDataset* d){ if(d) GDALClose(d); };
  std::unique_ptr<GDALDataset, decltype(gdal_closer)> dataset(
    GDALDataset::FromHandle(GDALOpen(filename.toLatin1(), GA_ReadOnly)), gdal_closer);

  if(!dataset)
  {
    qDebug("RasterLayer::loadFile null dataset");
    return result;
  }

  std::unique_ptr<GDALDataset, decltype(gdal_closer)> reprojected_dataset(
    GDALDataset::FromHandle(GDALAutoCreateWarpedVRT(dataset.get(), nullptr, web_mercator::wkt, GRA_Bilinear, 0.0, nullptr)),
    gdal_closer);

  if(!reprojected_dataset)
  {
    qDebug("RasterLayer::loadFile error creating reprojected dataset");
    return result;
  }

  double reprojected_geo_transform[6] = {0.0};
  reprojected_dataset->GetGeoTransform(reprojected_geo_transform);

  const int width = reprojected_dataset->GetRasterXSize();
  const int height = reprojected_dataset->GetRasterYSize();
  result.width = width;
  result.height = height;
  result.reprojected_width = width;
  result.reprojected_height = height;

  // [camp#134] Web-Mercator extent (same derivation as initExtent), so imageReady()
  // can re-apply the placement defensively if initExtent() did not run.
  const double x0 = reprojected_geo_transform[0];
  const double y0 = reprojected_geo_transform[3];
  const double x1 = x0 + width * reprojected_geo_transform[1];
  const double y1 = y0 + height * reprojected_geo_transform[5];
  result.merc_x_min = std::min(x0, x1);
  result.merc_x_max = std::max(x0, x1);
  result.merc_y_min = std::min(y0, y1);
  result.merc_y_max = std::max(y0, y1);

  auto first_band = reprojected_dataset->GetRasterBand(1);
  if(reprojected_dataset->GetRasterCount() == 1 &&
     first_band->GetColorTable() == nullptr)
  {
    // [camp#63/#59 PR3c / camp#90 / camp#134] Single-band scalar field (depth
    // raster, GGGS backscatter/bathy tiles, etc.): read as Float32 and upload as an
    // R32F value texture coloured by the shared shader's LUT (NoData / NaN ->
    // transparent on the GPU). Read as Float32 so any numeric dtype works. Paletted
    // single-band rasters still use the colour-table path below.
    result.is_scalar = true;
    int has_nodata = 0;
    const double nodata = first_band->GetNoDataValue(&has_nodata);
    result.has_nodata = has_nodata != 0;
    result.nodata = float(nodata);
    std::vector<float> values(static_cast<size_t>(width) * height);
    if(first_band->RasterIO(GF_Read, 0, 0, width, height, values.data(), width, height, GDT_Float32, 0, 0) == CE_None)
    {
      double min_value = std::numeric_limits<double>::max();
      double max_value = std::numeric_limits<double>::lowest();
      for(float v : values)
      {
        // Exclude NaN + the finite NoData sentinel from the auto-range, so the CPU
        // range and the shader's discard (`v != v` and v == u_nodata) agree.
        if(std::isnan(v) || (result.has_nodata && v == result.nodata))
          continue;
        min_value = std::min(min_value, double(v));
        max_value = std::max(max_value, double(v));
      }
      // A constant-valued raster (all valid samples equal) would otherwise hit the
      // degenerate max<=min guard and render transparent. Widen the range so the
      // single value maps to the top of the ramp. If there were no valid samples at
      // all, min/max stay crossed and the layer correctly shows nothing.
      if(min_value == max_value)
        min_value -= 1.0;
      result.data_min = float(min_value);
      result.data_max = float(max_value);
      // Periodic abort check (cheap, whole-buffer granularity is fine post-read).
      {
        QMutexLocker lock(&abort_flag_mutex_);
        if(abort_flag_)
          return {};
      }
      result.values = std::move(values);
    }
    else
    {
      // [camp#134] A failed scalar read must NOT fall through to result.ok = true:
      // that reported success while uploading nothing (an empty values buffer ->
      // no texture -> a blank layer). Leave ok = false so imageReady() surfaces the
      // failure ("(load failed)"), matching the RGBA path's load-failure handling.
      qDebug("RasterLayer::loadFile scalar band RasterIO read failed");
      return result;   // result.ok stays false (default)
    }
  }
  else
  {
    // [camp#134] Palette / RGB chart: composite to an RGBA QImage on the CPU (as
    // before) and upload it as an RGBA8 texture the shader samples directly (the
    // LUT is bypassed). Per-band channel select for colour files is a future issue.
    QImage image(width, height, QImage::Format_ARGB32);
    image.fill(Qt::black);
    for(auto&& band: reprojected_dataset->GetBands())
    {
      auto color_table = band->GetColorTable();
      std::vector<uint32_t> buffer(width);
      for(int j = 0; j<height; ++j)
      {
        if(band->RasterIO(GF_Read, 0, j, width, 1, &buffer.front(), width, 1, GDT_UInt32, 0, 0)== CE_None)
        {
          uchar *scanline = image.scanLine(j);
          for(int i = 0; i < width; ++i)
          {
            if(color_table)
            {
              GDALColorEntry const *ce = color_table->GetColorEntry(buffer[i]);
              if(!ce)
                continue;   // index outside the palette -> leave the fill (skip)
              scanline[i*4] = ce->c3;
              scanline[i*4+1] = ce->c2;
              scanline[i*4+2] = ce->c1;
              scanline[i*4+3] = ce->c4;
            }
            else
            {
              if(band->GetColorInterpretation() == GCI_GrayIndex)
              {
                  scanline[i*4+0] = buffer[i];
                  scanline[i*4+1] = buffer[i];
                  scanline[i*4+2] = buffer[i];
              }
              if(band->GetColorInterpretation() == GCI_RedBand)
                  scanline[i*4+2] = buffer[i];
              if(band->GetColorInterpretation() == GCI_GreenBand)
                  scanline[i*4+1] = buffer[i];
              if(band->GetColorInterpretation() == GCI_BlueBand)
                  scanline[i*4+0] = buffer[i];
              if(band->GetColorInterpretation() == GCI_AlphaBand)
                  scanline[i*4+3] = buffer[i];
            }
          }
        }
        QMutexLocker lock(&abort_flag_mutex_);
        if(abort_flag_)
          return {};
      }
    }
    result.rgba = std::move(image);
  }

  result.ok = true;
  return result;
}

void RasterLayer::imageReady()
{
  auto result = future_watcher_.result();
  if(!result.ok)              // failed/aborted load (null/unreprojectable dataset)
  {
    setStatus("(load failed)");
    return;
  }
  is_scalar_ = result.is_scalar;
  format_ = is_scalar_ ? RasterFieldItem::Format::Scalar
                       : RasterFieldItem::Format::Rgba;
  data_min_ = result.data_min;
  data_max_ = result.data_max;
  has_nodata_ = result.has_nodata;
  nodata_ = result.nodata;
  // [camp#142] Keep the Auto resolved range current with the loaded scalar extents
  // (a no-op while the operator holds a Manual override). Only scalar charts shade
  // through the range; RGB charts bypass the LUT, so their crossed default extents
  // are left untracked.
  if(is_scalar_ && data_min_ <= data_max_)
    range_model_.update_auto(data_min_, data_max_);

  // [#59 ADR-0003] The placement was already applied synchronously in initExtent()
  // (identical reprojected geotransform). Re-apply defensively only if initExtent()
  // did not establish the extent (e.g. a transient open failure) but the async load
  // nonetheless succeeded.
  if(scene_bounds_.isNull())
  {
    prepareGeometryChange();
    reprojected_width_ = result.reprojected_width;
    reprojected_height_ = result.reprojected_height;
    merc_x_min_ = result.merc_x_min;
    merc_x_max_ = result.merc_x_max;
    merc_y_min_ = result.merc_y_min;
    merc_y_max_ = result.merc_y_max;
    scene_bounds_ = QRectF(QPointF(merc_x_min_, merc_y_min_),
                           QPointF(merc_x_max_, merc_y_max_)).normalized();
    setTransform(QTransform::fromScale(1.0, -1.0));
    setPos(QPointF(scene_bounds_.left(), scene_bounds_.bottom()));
  }

  // [camp#134] Upload the reprojected pixels to a GL texture under the renderer's
  // context (the texture is owned here). Scalar -> R32F (Nearest-filtered by the
  // renderer); colour -> RGBA8 with mipmaps (so a chart zoomed far out keeps the
  // LOD the old QPainter mipmap pyramid provided).
  if(renderer_.makeCurrent())
  {
    texture_.reset();
    if(is_scalar_ && !result.values.empty())
    {
      texture_ = std::make_unique<QOpenGLTexture>(QOpenGLTexture::Target2D);
      texture_->setFormat(QOpenGLTexture::R32F);
      texture_->setSize(result.width, result.height);
      texture_->setMipLevels(1);
      texture_->allocateStorage(QOpenGLTexture::Red, QOpenGLTexture::Float32);
      texture_->setData(QOpenGLTexture::Red, QOpenGLTexture::Float32,
                        result.values.data());
      texture_->setWrapMode(QOpenGLTexture::ClampToEdge);
    }
    else if(!is_scalar_ && !result.rgba.isNull())
    {
      // [camp#134] Upload PREMULTIPLIED RGBA so mipmap generation (and linear
      // minification) box-filter premultiplied texels — straight-alpha averaging
      // bleeds the transparent cells' black RGB into the edges as dark fringes when
      // the chart is zoomed out. The shared shader's Rgba branch samples this texel
      // as already-premultiplied (no second c.rgb*c.a), matching the premultiplied
      // GL_ONE / GL_ONE_MINUS_SRC_ALPHA blend.
      const QImage rgba8 = result.rgba.convertToFormat(QImage::Format_RGBA8888_Premultiplied);
      texture_ = std::make_unique<QOpenGLTexture>(QOpenGLTexture::Target2D);
      texture_->setFormat(QOpenGLTexture::RGBA8_UNorm);
      texture_->setSize(rgba8.width(), rgba8.height());
      texture_->setMipLevels(texture_->maximumMipLevels());
      texture_->allocateStorage(QOpenGLTexture::RGBA, QOpenGLTexture::UInt8);
      texture_->setData(QOpenGLTexture::RGBA, QOpenGLTexture::UInt8, rgba8.constBits());
      texture_->generateMipMaps();
      texture_->setWrapMode(QOpenGLTexture::ClampToEdge);
    }
    renderer_.doneCurrent();
  }

  cached_image_ = QImage();
  update(boundingRect());
  setStatus("");
}

QImage RasterLayer::renderImage(const QSize& size)
{
  if(scene_bounds_.isNull() || size.isEmpty())
    return QImage();
  if(is_scalar_ && data_min_ > data_max_)
    return QImage();
  if(!renderer_.makeCurrent())
    return QImage();
  const QList<RasterFieldItem> draw = items();
  // [camp#142] Feed the resolved range (Auto tracks data_min_/data_max_; Manual is
  // the operator override) into the shader's u_min/u_max. Scalar charts shade
  // through it; RGB charts bypass the LUT, so the range is a don't-care for them.
  const QImage image = renderer_.renderToImage(draw, scene_bounds_, range_model_.lo(),
                                               range_model_.hi(), size);
  renderer_.doneCurrent();
  return image;
}

QStringList RasterLayer::bands() const
{
  // A reprojected chart is a single composited field; one nominal band.
  return QStringList() << "1";
}

RasterBandMeta RasterLayer::metadata(const QString&) const
{
  RasterBandMeta meta;
  meta.has_nodata = has_nodata_;
  meta.nodata = nodata_;
  return meta;
}

QPair<float, float> RasterLayer::dataRange() const
{
  return {data_min_, data_max_};
}

QList<RasterFieldItem> RasterLayer::items()
{
  // [camp#134] One non-geographic item: GDAL already reprojected the chart to
  // Web-Mercator, so the renderer draws it as a single linear quad over the
  // mercator extent (no per-vertex geo warp).
  QList<RasterFieldItem> result;
  if(!texture_)
    return result;
  RasterFieldItem item;
  item.texture = texture_.get();
  item.format = format_;
  item.geographic = false;
  item.west = merc_x_min_;
  item.east = merc_x_max_;
  item.south = merc_y_min_;
  item.north = merc_y_max_;
  item.has_nodata = has_nodata_;
  item.nodata = nodata_;
  result.push_back(item);
  return result;
}

void RasterLayer::setColormap(const std::string& name)
{
  if(name == renderer_.colormap())
    return;
  // [camp#134] A colormap change is now just an LUT re-bake — no re-warp. The
  // scalar value texture already holds the raw data, so the shader recolours it on
  // the next render.
  renderer_.setColormap(name);
  writeSettings();
  cached_image_ = QImage();
  update(boundingRect());
}

void RasterLayer::setRangeOverride(float lo, float hi)
{
  // [camp#142] Pin the resolved range to the operator's [lo, hi] (set_manual swaps
  // an inverted pair so lo() <= hi()). Re-render with the new range + persist.
  range_model_.set_manual(lo, hi);
  cached_image_ = QImage();
  writeSettings();
  update(boundingRect());
}

void RasterLayer::resetRangeToAuto()
{
  // [camp#142] Return to data-driven Auto, then re-track the current scalar extents
  // so lo()/hi() reflect the data without waiting for a reload.
  range_model_.reset();
  if(is_scalar_ && data_min_ <= data_max_)
    range_model_.update_auto(data_min_, data_max_);
  cached_image_ = QImage();
  writeSettings();
  update(boundingRect());
}

void RasterLayer::contextMenu(QMenu* menu)
{
  map::Layer::contextMenu(menu);
  if(!is_scalar_)              // colormap only applies to scalar (depth) rasters
    return;
  // [camp#141] Expose the FULL marine_colormap registry, not just the legacy ramps.
  QMenu* colormap_menu = menu->addMenu("Colormap");
  for(const std::string& name : marine_colormap::palette_names())
  {
    QAction* action = colormap_menu->addAction(QString::fromStdString(name));
    action->setCheckable(true);
    action->setChecked(name == renderer_.colormap());
    connect(action, &QAction::triggered, this, [this, name]() { setColormap(name); });
  }

  // [camp#142 PR2] Colormap range override (scalar only — gated by the is_scalar_
  // early return above, the same condition as the Colormap submenu). Opens the
  // interactive colorbar — drag the handles or edit the bounds to pin a Manual
  // override, reset to track the data extents — the successor to PR1's sequential
  // numeric prompts.
  QAction* range_action = menu->addAction("Colormap range…");
  connect(range_action, &QAction::triggered, this, [this]()
  {
    ColormapRangeState state;
    const auto idx = marine_colormap::palette_index(renderer_.colormap());
    state.palette_index = idx ? static_cast<int>(*idx) : 0;
    state.data_min = static_cast<float>(data_min_);
    state.data_max = static_cast<float>(data_max_);
    state.mode = range_model_.mode();
    state.lo = range_model_.lo();
    state.hi = range_model_.hi();
    showColormapRangeDialog(
      nullptr, "Colormap range", state,
      [this](float lo, float hi) { setRangeOverride(lo, hi); },
      [this]() { resetRangeToAuto(); });
  });
}

void RasterLayer::readSettings()
{
  map::Layer::readSettings();
  QSettings settings;
  settings.beginGroup("MapItem");
  settings.beginGroup(itemID());
  // [camp#141] Persisted palette name; case-insensitive read + registry-validated
  // (unknown -> grayscale).
  std::string colormap = settings.value(
    "colormap", QString::fromStdString(renderer_.colormap())).toString().toLower().toStdString();
  if(!marine_colormap::palette_index(colormap))
    colormap = "grayscale";
  // [camp#142] Persisted colormap range (grouped under itemID(), the key this
  // layer's read/writeSettings already use — NOT settingsKey()). "manual" restores
  // the operator override; anything else (default "auto") leaves Auto.
  // Only honor a Manual override when the mode says so AND both extents are
  // present — a partial/corrupt entry falls back to Auto rather than snapping to
  // the [0,1] read-defaults.
  const QString range_mode = settings.value("range_mode", "auto").toString();
  const bool has_manual_range = range_mode == "manual" &&
    settings.contains("range_min") && settings.contains("range_max");
  const float range_min = settings.value("range_min", 0.0).toFloat();
  const float range_max = settings.value("range_max", 1.0).toFloat();
  settings.endGroup();
  settings.endGroup();
  // Apply the persisted ramp (re-bake + re-render if it differs); don't re-persist.
  if(colormap != renderer_.colormap())
  {
    renderer_.setColormap(colormap);
    cached_image_ = QImage();
    update(boundingRect());
  }
  // [camp#142] Apply the persisted range. A Manual override is independent of the
  // data extents and is restored as-is; "auto" leaves the model tracking the data.
  if(has_manual_range)
    range_model_.set_manual(range_min, range_max);
  else
    range_model_.reset();
}

void RasterLayer::onRemovedFromMap()
{
  // [camp#90] Drop this file from the BackgroundManager restore list so a
  // user-removed raster stays gone next session.
  QSettings settings;
  QStringList files = settings.value("GggsRasters/files").toStringList();
  if(files.removeAll(filename_) > 0)
    settings.setValue("GggsRasters/files", files);
}

void RasterLayer::writeSettings()
{
  map::Layer::writeSettings();
  QSettings settings;
  settings.beginGroup("MapItem");
  settings.beginGroup(itemID());
  settings.setValue("colormap", QString::fromStdString(renderer_.colormap()));
  // [camp#142] Persist the colormap range mode + bounds so a Manual override (and
  // its [lo, hi]) survives a restart; Auto persists as "auto".
  settings.setValue("range_mode",
                    range_model_.mode() == marine_colormap::RangeMode::Manual ? "manual"
                                                                              : "auto");
  settings.setValue("range_min", range_model_.lo());
  settings.setValue("range_max", range_model_.hi());
  settings.endGroup();
  settings.endGroup();
}

} // namespace raster

} // namespace camp
