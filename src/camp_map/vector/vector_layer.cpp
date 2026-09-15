#include "vector_layer.h"

#include <algorithm>
#include <memory>
#include <set>

#include <QDebug>
#include <QFileInfo>
#include <QMutexLocker>
#include <QMenu>
#include <QSettings>
#include <QUrl>
#include <QtConcurrent/QtConcurrent>

#include <gdal_priv.h>

#include <marine_colormap/palette.hpp>

#include <crash_handler.h>

#include "vector_feature_item.h"

namespace camp::vector
{

namespace
{

// The palette a colour-by-field pass samples. Unknown name -> grayscale, the
// same fallback the raster layers and the grid renderer use (ADR-0008).
const marine_colormap::Palette* resolvePalette(const std::string& name)
{
  const marine_colormap::Palette* palette = marine_colormap::find_palette(name);
  if(!palette)
    palette = marine_colormap::find_palette("grayscale");
  return palette;
}

// [camp#22] The drivers this layer will open. GDALOpenEx is given an operator-
// supplied STRING, which OGR treats as a connection string rather than a path, so
// an unrestricted driver set would let a typed (or persisted) "PG:host=...",
// "MySQL:", "WFS:" or "OAPIF:" open a DATABASE OR SERVICE CONNECTION from the
// load worker. CAMP's Open Vector Layer means "read this file", so the set is
// pinned to file-based vector drivers. Adding a format here is a deliberate act;
// the list is the contract.
//
// What this list does NOT do is stop a remote fetch: GDAL resolves a /vsicurl/,
// /vsizip/ or /vsis3/ prefix in its VIRTUAL FILE SYSTEM layer BEFORE a driver is
// chosen, so "/vsicurl/https://host/x.geojson" would still be fetched — through
// the allowed GeoJSON driver. `isVirtualFileSystemPath()` is what blocks that,
// and it is checked before this call (see the constructor).
//
// Remaining limitation, documented in ADR-0016 D12: KML/LIBKML are file drivers
// that can carry a NetworkLink, and nothing here stops the driver following one.
// The formats are on the list because operators are handed KML routinely; the
// exposure is a fetch initiated by file CONTENT, not by the path CAMP was given.
const char* const kAllowedDrivers[] = {
  "GeoJSON", "GeoJSONSeq", "TopoJSON", "ESRI Shapefile", "GPKG", "SQLite",
  "KML", "LIBKML", "GML", "GMT", "CSV", "DXF", "FlatGeobuf", "OpenFileGDB",
  "MapInfo File", "GPX", "S57",
  nullptr
};

}  // namespace

bool isVirtualFileSystemPath(const QString& path)
{
  // GDAL's virtual file systems are selected by a leading "/vsi..." token in the
  // filename, resolved BEFORE any driver is chosen, so the driver allowlist does
  // not see them. Nested forms ("/vsizip//vsicurl/https://...") start with the
  // same token. Whitespace is trimmed first: GDAL does not, but a path pasted
  // into the file dialog or carried in a settings file can arrive padded, and a
  // check that a space defeats is not a check.
  return path.trimmed().startsWith(QStringLiteral("/vsi"), Qt::CaseInsensitive);
}

VectorLayer::VectorLayer(map::MapItem* parentItem, const QString& filename, int feature_cap):
  map::Layer(parentItem, QFileInfo(filename).fileName()),
  filename_(filename),
  feature_cap_(feature_cap > 0 ? feature_cap : kMaxFeatureItems)
{
  if(GDALGetDriverCount() == 0)
    GDALAllRegister();
  // [camp#22] A /vsi path is refused HERE, before anything is opened: the worker
  // would otherwise hand it to GDALOpenEx, which resolves the virtual file system
  // before driver selection and fetches. This runs on the restore path too —
  // restorePersistedVectorLayers() reopens every persisted entry at startup with
  // no operator present to confirm anything — which is why the refusal lives in
  // the constructor rather than at the menu action.
  if(isVirtualFileSystemPath(filename))
  {
    qWarning() << "camp::vector::VectorLayer:" << filename
               << "- refused: a /vsi path is a GDAL virtual file system, which is"
               << "resolved before driver selection and can fetch over the network."
               << "Open Vector Layer reads local files only.";
    setStatus("(refused: not a local file)");
    return;
  }
  connect(&future_watcher_, &QFutureWatcher<LoadResult>::finished, this, &VectorLayer::loadFinished);
  setStatus("(loading...)");
  future_watcher_.setFuture(QtConcurrent::run(this, &VectorLayer::loadVectorFile, filename));
}

VectorLayer::~VectorLayer()
{
  // [#213] Tell the worker to stop, then JOIN it before this object — whose
  // members the worker reads — starts tearing down.
  abort_flag_mutex_.lock();
  abort_flag_ = true;
  abort_flag_mutex_.unlock();
  future_watcher_.waitForFinished();
}

VectorLayer::LoadResult VectorLayer::loadVectorFile(const QString& filename)
{
  // [#217] First statement of a QtConcurrent worker: give this pool thread an
  // alternate signal stack, without which a stack-overflow SIGSEGV here cannot be
  // reported at all. Idempotent; guarded by the check_worker_alt_stacks test.
  camp_crash::install_thread_alt_stack();

  LoadResult result;
  if(isAborted())
    return result;

  // [#152] RAII-close the dataset on every path; parseVectorLayers opens none of
  // its own and frees the transforms/iterators it does create.
  const auto gdal_closer = [](GDALDataset* d){ if(d) GDALClose(d); };
  std::unique_ptr<GDALDataset, decltype(gdal_closer)> dataset(
    static_cast<GDALDataset*>(
      GDALOpenEx(filename.toUtf8().constData(), GDAL_OF_READONLY | GDAL_OF_VECTOR,
                 const_cast<char**>(kAllowedDrivers), nullptr, nullptr)),
    gdal_closer);
  if(!dataset)
    return result;

  result.opened = true;
  // [camp#22 / #213] Hand the parser the abort flag so it can be polled PER
  // FEATURE. Checking it once up here bounds nothing: the destructor's join then
  // waits out the whole parse on the GUI thread — minutes, for a coastline
  // shapefile. RasterLayer re-checks inside its work loops for the same reason.
  ParseOptions options;
  options.aborted = [this]() { return isAborted(); };
  // [camp#22] The cap is carried INTO the parse. Applying it afterwards — which
  // is what this did first — bounds only the items built on the GUI thread: the
  // worker has by then materialised every geometry and attribute map of the whole
  // file, which for the coastline shapefile the cap exists for is the OOM the
  // header names. Stopping the parse is what bounds the memory.
  options.max_geometries = feature_cap_;
  result.layers = parseVectorLayers(dataset.get(), options, &result.diagnostics);
  return result;
}

bool VectorLayer::isAborted()
{
  QMutexLocker lock(&abort_flag_mutex_);
  return abort_flag_;
}

void VectorLayer::loadFinished()
{
  const LoadResult result = future_watcher_.result();
  if(result.diagnostics.aborted)
    return;   // the parse was cancelled; this layer is on its way out
  if(!result.opened)
  {
    setStatus("(load failed)");
    return;
  }

  // [camp#22] The parse already stopped at the cap (ParseOptions::max_geometries),
  // so this flag — not a count — is what the layer has to report: how many more
  // features the file holds was deliberately never read.
  const bool capped = result.diagnostics.geometry_cap_reached;

  prepareGeometryChange();
  int skipped = 0;
  for(const ParsedLayer& layer : result.layers)
  {
    for(const ParsedGeometry& geometry : layer.geometries)
    {
      // [camp#22] Every item below is built on the GUI THREAD, and the file
      // dialog does not bound what an operator can open: a national coastline
      // shapefile is millions of features, and building an item for each one
      // freezes CAMP with no way out. The parser is capped at the same number, so
      // this bound is normally never the one that bites; it stays as the backstop
      // for a caller that hands this class an over-long parse result.
      if(static_cast<int>(features_.size()) >= feature_cap_)
        break;
      // [camp#22] A feature with no placeable coordinate — a .prj-less shapefile
      // read as degrees, a NaN from a failed transform — is skipped rather than
      // placed 1e17 metres away, where it would poison childrenBoundingRect()
      // (fit-to-extent) and the scene index for every other feature.
      if(!hasPlaceableCoordinate(geometry))
      {
        ++skipped;
        continue;
      }
      features_.push_back(new VectorFeatureItem(this, geometry));
    }
    if(static_cast<int>(features_.size()) >= feature_cap_)
      break;
  }

  if(skipped > 0)
    qWarning() << "camp::vector::VectorLayer:" << filename_ << "- skipped" << skipped
               << "feature(s) whose coordinates are not a valid latitude/longitude"
               << "(a shapefile missing its .prj sidecar is the usual cause)";
  if(capped)
    qWarning() << "camp::vector::VectorLayer:" << filename_ << "- stopped at the"
               << feature_cap_ << "feature cap; the REST OF THE FILE WAS NOT READ, so"
               << "what is shown is the first" << feature_cap_ << "features and no more."
               << "The cap bounds both the items built on the GUI thread and the memory"
               << "the parse itself takes.";
  if(result.diagnostics.polygons_without_exterior_ring > 0)
    qWarning() << "camp::vector::VectorLayer:" << filename_ << "- dropped"
               << result.diagnostics.polygons_without_exterior_ring
               << "polygon(s) with no exterior ring";
  if(result.diagnostics.layers_failed > 0)
    qWarning() << "camp::vector::VectorLayer:" << filename_ << "-"
               << result.diagnostics.layers_failed << "of" << result.diagnostics.layers_total
               << "layer(s) were skipped: no coordinate transformation to WGS84 could be"
               << "built from their spatial reference";

  loaded_ = !features_.empty();
  if(!loaded_)
  {
    // The driver opened the file but this layer shows nothing. Say WHY — an empty
    // layer that claims to have loaded is indistinguishable from one drawn
    // off-screen, and each of these has a different remedy.
    if(result.diagnostics.layers_failed > 0)
      setStatus("(load failed: no usable coordinate system)");
    else if(skipped > 0)
      setStatus(QString("(no placeable features; %1 skipped)").arg(skipped));
    else
      setStatus("(no features)");
    return;
  }

  QStringList notes;
  notes << QString("%1 features").arg(features_.size());
  if(capped)
    notes << QString("stopped at the %1-feature cap; rest of file not read").arg(feature_cap_);
  if(skipped > 0)
    notes << QString("%1 unplaceable").arg(skipped);
  if(result.diagnostics.layers_failed > 0)
    notes << QString("%1 layer(s) failed").arg(result.diagnostics.layers_failed);
  setStatus("(" + notes.join(", ") + ")");

  // readSettings() may have restored a style before the features existed.
  applyStyle();
}

QRectF VectorLayer::boundingRect() const
{
  // MapItem::boundingRect() is empty; the layer's extent is its features'.
  return childrenBoundingRect();
}

QString VectorLayer::settingsKey() const
{
  // [camp#126 precedent] Percent-encode the path into ONE flat key rather than a
  // nested group tree, and namespace it so it reads as what it is.
  return "file:" + QString::fromLatin1(QUrl::toPercentEncoding(filename_));
}

QStringList VectorLayer::fields() const
{
  std::set<QString> names;
  for(const VectorFeatureItem* feature : features_)
    for(auto it = feature->attributes().begin(); it != feature->attributes().end(); ++it)
      names.insert(it.key());
  QStringList list;
  for(const QString& name : names)
    list << name;
  return list;
}

void VectorLayer::setColorField(const QString& field)
{
  if(field == color_field_)
    return;
  color_field_ = field;
  applyStyle();
  writeSettings();
}

void VectorLayer::setSizeField(const QString& field)
{
  if(field == size_field_)
    return;
  size_field_ = field;
  applyStyle();
  writeSettings();
}

void VectorLayer::setColormap(const std::string& name)
{
  if(name == colormap_)
    return;
  colormap_ = name;
  applyStyle();
  writeSettings();
}

void VectorLayer::applyStyle()
{
  if(features_.empty())
    return;

  // Ranges are computed over the FEATURES, once per style change, through the
  // same accumulateValue() fold fieldRange() uses — a feature with no value for
  // the field neither widens the extent nor pulls it toward zero.
  FieldRange color_range;
  FieldRange size_range;
  for(const VectorFeatureItem* feature : features_)
  {
    accumulateValue(color_range, numericAttribute(feature->attributes(), color_field_));
    // [camp#22] Size-by-field applies to POINTS only, so its range is folded over
    // points only. Including lines and polygons would let geometry that is never
    // sized set the extent — on a mixed file whose polygons carry the largest
    // values, every marker would be squeezed into the bottom of the radius range.
    if(feature->isPoint())
      accumulateValue(size_range, numericAttribute(feature->attributes(), size_field_));
  }

  const marine_colormap::Palette* palette = resolvePalette(colormap_);
  const QColor default_color(Qt::darkCyan);

  for(VectorFeatureItem* feature : features_)
  {
    if(color_field_.isEmpty())
      feature->setColor(default_color);
    else
      feature->setColor(colorForValue(palette,
                                      numericAttribute(feature->attributes(), color_field_),
                                      color_range, default_color));
    if(!feature->isPoint())
      continue;   // size-by-field is meaningless for lines and polygons
    if(size_field_.isEmpty())
      feature->setRadius(kDefaultPointRadius);
    else
      feature->setRadius(radiusForValue(numericAttribute(feature->attributes(), size_field_),
                                        size_range, kMinPointRadius, kMaxPointRadius,
                                        kDefaultPointRadius));
  }
}

void VectorLayer::onRemovedFromMap()
{
  // [camp#22 / camp#90] Layer::removeFromMap() calls this synchronously, BEFORE
  // it detaches the item through the Map model — and a drag-reorder, which goes
  // through the same model detach, never reaches here. Owners sync (and
  // re-persist) from this signal rather than from rowsAboutToBeRemoved.
  emit removedFromMap();
}

void VectorLayer::contextMenu(QMenu* menu)
{
  map::Layer::contextMenu(menu);

  const QStringList field_names = fields();
  if(field_names.isEmpty())
    return;   // nothing loaded, or a file with no attributes: nothing to style by

  QMenu* color_menu = menu->addMenu("Color by");
  QAction* no_color = color_menu->addAction("(none)");
  no_color->setCheckable(true);
  no_color->setChecked(color_field_.isEmpty());
  connect(no_color, &QAction::triggered, this, [this]() { setColorField(QString()); });
  for(const QString& field : field_names)
  {
    QAction* action = color_menu->addAction(field);
    action->setCheckable(true);
    action->setChecked(field == color_field_);
    connect(action, &QAction::triggered, this, [this, field]() { setColorField(field); });
  }

  QMenu* size_menu = menu->addMenu("Size by");
  QAction* no_size = size_menu->addAction("(none)");
  no_size->setCheckable(true);
  no_size->setChecked(size_field_.isEmpty());
  connect(no_size, &QAction::triggered, this, [this]() { setSizeField(QString()); });
  for(const QString& field : field_names)
  {
    QAction* action = size_menu->addAction(field);
    action->setCheckable(true);
    action->setChecked(field == size_field_);
    connect(action, &QAction::triggered, this, [this, field]() { setSizeField(field); });
  }

  // [ADR-0008] The full marine_colormap registry, as the raster layers expose it.
  QMenu* colormap_menu = menu->addMenu("Colormap");
  for(const std::string& name : marine_colormap::palette_names())
  {
    QAction* action = colormap_menu->addAction(QString::fromStdString(name));
    action->setCheckable(true);
    action->setChecked(name == colormap_);
    connect(action, &QAction::triggered, this, [this, name]() { setColormap(name); });
  }
}

void VectorLayer::readSettings()
{
  map::Layer::readSettings();
  QSettings settings;
  settings.beginGroup("MapItem");
  settings.beginGroup(settingsKey());
  color_field_ = settings.value("color_field").toString();
  size_field_ = settings.value("size_field").toString();
  colormap_ = settings.value("colormap", QString::fromStdString(colormap_)).toString().toStdString();
  settings.endGroup();
  settings.endGroup();
  // Restoring can run before the async load finishes (MapItem::itemConstructed
  // fires on the next event-loop turn); applyStyle() is a no-op until then and
  // loadFinished() calls it again once the features exist.
  applyStyle();
}

void VectorLayer::writeSettings()
{
  map::Layer::writeSettings();
  QSettings settings;
  settings.beginGroup("MapItem");
  settings.beginGroup(settingsKey());
  settings.setValue("color_field", color_field_);
  settings.setValue("size_field", size_field_);
  settings.setValue("colormap", QString::fromStdString(colormap_));
  settings.endGroup();
  settings.endGroup();
}

QString vectorLayerFilesKey()
{
  return QStringLiteral("vectorLayers/files");
}

QStringList persistedVectorLayerFiles()
{
  QSettings settings;
  return settings.value(vectorLayerFilesKey()).toStringList();
}

void writePersistedVectorLayerFiles(const QStringList& files)
{
  QSettings settings;
  settings.setValue(vectorLayerFilesKey(), files);
}

QStringList withVectorLayerFile(const QStringList& files, const QString& filename)
{
  QStringList result;
  for(const QString& file : files)
    if(!result.contains(file))
      result << file;
  if(!filename.isEmpty() && !result.contains(filename))
    result << filename;
  return result;
}

QStringList rebuildPersistedVectorLayerFiles(const QStringList& restoredOrder,
                                             const QStringList& unavailable,
                                             const QStringList& loadedFiles)
{
  QStringList files;
  for(const QString& restored : restoredOrder)
  {
    if(unavailable.contains(restored))
    {
      files = withVectorLayerFile(files, restored);
      continue;
    }
    if(loadedFiles.contains(restored))
      files = withVectorLayerFile(files, restored);
  }
  // withVectorLayerFile is append-if-absent, so a layer already placed above
  // keeps its slot.
  for(const QString& loaded : loadedFiles)
    files = withVectorLayerFile(files, loaded);
  return files;
}

}  // namespace camp::vector
