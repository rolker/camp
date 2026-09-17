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

#include <cpl_string.h>
#include <cpl_vsi.h>
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
// KML/LIBKML are on the list because operators are handed KML routinely. A KML
// file can carry a NetworkLink pointing anywhere, and this list would not stop a
// driver following one — but MEASURED against GDAL 3.8.4, the version CAMP builds
// on, neither driver does: a NetworkLink with a relative local href, and one with
// an http:// href, both come back as the local placemark alone with no fetch
// attempted. This comment used to assert the fetch as a live limitation, which
// was derived from the format rather than read off the software. See ADR-0016 D12
// — and re-measure there when the GDAL version moves, rather than re-deriving.
const char* const kAllowedDrivers[] = {
  "GeoJSON", "GeoJSONSeq", "TopoJSON", "ESRI Shapefile", "GPKG", "SQLite",
  "KML", "LIBKML", "GML", "GMT", "CSV", "DXF", "FlatGeobuf", "OpenFileGDB",
  "MapInfo File", "GPX", "S57",
  nullptr
};

}  // namespace

bool isVirtualFileSystemPath(const QString& path)
{
  // GDAL's virtual file systems are selected by a leading prefix in the filename,
  // resolved BEFORE any driver is chosen, so the driver allowlist does not see
  // them. Nested forms ("/vsizip//vsicurl/https://...") start with the same
  // prefix. Whitespace is trimmed first: GDAL does not, but a path pasted into
  // the file dialog or carried in a settings file can arrive padded, and a check
  // that a space defeats is not a check.
  //
  // [camp#22] The prefixes are asked of GDAL (VSIGetFileSystemsPrefixes), not
  // matched as the four raw characters "/vsi". That spelling refused any path
  // BEGINNING with them, so an ordinary local directory named /vsidata — or
  // /vsi_survey, or a mount point someone called /vsi — could not be opened or
  // persisted at all, with a loud and wrong explanation. A prefix-BOUNDARY check
  // does not fix it either ("/vsidata/" is still "/vsi<word>/"); only the actual
  // registered handler list distinguishes a virtual file system from a directory
  // whose name starts the same way. The list is what GDAL itself dispatches on,
  // so this check now tracks GDAL's real behaviour rather than approximating it:
  // a prefix GDAL does not have a handler for is a prefix GDAL will not resolve.
  const QString candidate = path.trimmed();
  if(candidate.isEmpty())
    return false;
  char** prefixes = VSIGetFileSystemsPrefixes();
  if(!prefixes)
  {
    // Nothing registered to ask — refuse the whole "/vsi" family rather than
    // letting one through on the strength of an empty list.
    return candidate.startsWith(QStringLiteral("/vsi"), Qt::CaseInsensitive);
  }
  bool virtual_path = false;
  for(int i = 0; prefixes[i] && !virtual_path; ++i)
    virtual_path = candidate.startsWith(QString::fromUtf8(prefixes[i]), Qt::CaseInsensitive);
  CSLDestroy(prefixes);
  return virtual_path;
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
               << "item(s) whose coordinates are not a valid latitude/longitude"
               << "(a shapefile missing its .prj sidecar is the usual cause)";
  if(capped)
    qWarning() << "camp::vector::VectorLayer:" << filename_ << "- stopped at the"
               << feature_cap_ << "item cap; the REST OF THE FILE WAS NOT READ, so"
               << "what is shown is the first" << feature_cap_ << "drawn items and no more."
               << "The cap bounds both the items built on the GUI thread and the memory"
               << "the parse itself takes.";
  if(result.diagnostics.polygons_without_exterior_ring > 0)
    qWarning() << "camp::vector::VectorLayer:" << filename_ << "- dropped"
               << result.diagnostics.polygons_without_exterior_ring
               << "polygon(s) with no exterior ring";
  if(result.diagnostics.geometries_with_empty_exterior > 0)
    qWarning() << "camp::vector::VectorLayer:" << filename_ << "- dropped"
               << result.diagnostics.geometries_with_empty_exterior
               << "geometry(ies) whose exterior holds no usable vertex (every vertex outside"
               << "the file's projection is the usual cause)";
  // [camp#22 round-8 must-fix] The geometries the PARSE dropped as undrawable are
  // skipped items too, and the loop above cannot count them: they never reach it.
  // Folding them in is what keeps the status honest — without it a file whose
  // features all fall outside their projection's inverse domain, or whose polygons
  // carry no exterior ring, reported the bare "(no items)", which is the
  // verdict an EMPTY FILE gets and has a completely different remedy. The
  // per-reason detail is in the log lines above; the status carries the count.
  skipped += result.diagnostics.geometries_with_empty_exterior +
             result.diagnostics.polygons_without_exterior_ring;
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
    //
    // [camp#22 round-4 should-fix] The CAP is said on this path too. "The cap was
    // hit and every capped geometry was unplaceable" is exactly what a .prj-less
    // national shapefile does, and reporting it as "(no placeable items; 50000
    // skipped)" alone reads as a verdict on the whole file when only its first
    // 50 000 items were ever read. The log line said so; the Layers tab — the
    // only status the operator actually looks at — did not.
    const QString capped_note =
        capped ? QString("; stopped at the %1-item cap, rest of file not read")
                     .arg(feature_cap_)
               : QString();
    if(result.diagnostics.layers_failed > 0)
      setStatus("(load failed: no usable coordinate system" + capped_note + ")");
    else if(skipped > 0)
      setStatus(QString("(no placeable items; %1 skipped%2)").arg(skipped).arg(capped_note));
    else
      setStatus("(no items" + capped_note + ")");
    return;
  }

  QStringList notes;
  // [camp#22 round-5 nit] "items", not "features": the cap is spent per emitted
  // GEOMETRY PART and one item is built per part, so a multi-part feature (a KML
  // placemark, a multipolygon coastline) yields several. Reporting these as
  // features told the operator a count their file does not have. The constant's
  // own name, kMaxFeatureItems, was the accurate one all along; the header, this
  // status line and the log above now all say the same thing.
  notes << QString("%1 items").arg(features_.size());
  if(capped)
    notes << QString("stopped at the %1-item cap; rest of file not read").arg(feature_cap_);
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

QStringList VectorLayer::numericFields() const
{
  // [camp#22] "Numeric" is asked of numericAttribute() — the same function the
  // ramp itself reads values through — rather than of the QVariant's declared
  // type, so the menu and the ramp cannot disagree about what a field is. A
  // GeoJSON writer that quotes its numbers produces a string QVariant that
  // numericAttribute() reads as a measurement, and that field must be offerable.
  std::set<QString> names;
  for(const VectorFeatureItem* feature : features_)
    for(auto it = feature->attributes().begin(); it != feature->attributes().end(); ++it)
      if(numericAttribute(feature->attributes(), it.key()))
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

  // [camp#22] A colour field NO feature has a numeric value for is treated as no
  // colour field at all for this pass — not as "every feature is missing its
  // value". No-data means "this feature has no value where its neighbours do",
  // and with an invalid range there is no ramp for anything to be missing from;
  // marking the whole layer no-data painted every feature a hollow grey ring,
  // which the operator's GUI test of 2026-09-15 read as the features
  // disappearing. The styling menu offers numericFields() only, so the reachable
  // way in is a PERSISTED color_field_: the style group is keyed on the file path
  // and restored whenever that path is reopened, and the file on disk may have
  // changed since. (Size-by-field already falls back this way —
  // radiusForValue() returns the default radius on an invalid range.)
  const bool color_by_field = !color_field_.isEmpty() && color_range.valid;

  for(VectorFeatureItem* feature : features_)
  {
    if(!color_by_field)
    {
      // Not styled by a field at all: the layer's default colour, and nothing is
      // "missing" — there is no field to be missing from.
      feature->setColor(default_color);
      feature->setNoData(false);
    }
    else
    {
      const auto value = numericAttribute(feature->attributes(), color_field_);
      feature->setColor(colorForValue(palette, value, color_range, default_color));
      // [camp#22] The second channel: a dashed outline and a hatched fill, which
      // no palette can imitate. noDataColor()'s grey is the same grey as the
      // middle of grayscale — a palette the operator can pick and the fallback
      // for an unknown name — so colour alone cannot say "this has no value".
      feature->setNoData(isNoData(value, color_range));
    }
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

  // [camp#22] NUMERIC fields only — a ramp cannot read a free-text one, and
  // offering it produced a layer of hollow grey rings the operator read as the
  // features disappearing (ADR-0016 D14). The Colormap menu is gated on the same
  // list because a palette with no colour field to sample changes nothing.
  const QStringList field_names = numericFields();

  // [camp#22] A PERSISTED field that numericFields() no longer offers. The style
  // group is keyed on the file path and restored whenever that path is reopened,
  // so the file on disk may have changed under it (a column re-typed, or a column
  // dropped). applyStyle() already paints that correctly — as UNSTYLED, see
  // AColorFieldWithNoNumbersFallsBackToUnstyled — but the menus used to be built
  // only from field_names, so the stale setting appeared nowhere and there was no
  // "(none)" to clear it with either: the layer was stuck, silently re-persisting
  // a field it was not using. It is SHOWN instead of being cleared on load,
  // because the operator chose it and the same path may be reopened over a file
  // where it reads as a number again; clearing it here would discard that choice
  // without anyone seeing it happen.
  const bool stale_color = !color_field_.isEmpty() && !field_names.contains(color_field_);
  const bool stale_size = !size_field_.isEmpty() && !field_names.contains(size_field_);
  if(field_names.isEmpty() && !stale_color && !stale_size)
    return;   // nothing loaded, no field a ramp can read, and nothing set: nothing to offer

  // The two field submenus are identical but for their title and their setter.
  auto addFieldMenu = [this, menu, &field_names](const QString& title, const QString& current,
                                                 void (VectorLayer::*setter)(const QString&))
  {
    QMenu* submenu = menu->addMenu(title);
    QAction* none = submenu->addAction("(none)");
    none->setCheckable(true);
    none->setChecked(current.isEmpty());
    connect(none, &QAction::triggered, this, [this, setter]() { (this->*setter)(QString()); });
    for(const QString& field : field_names)
    {
      QAction* action = submenu->addAction(field);
      action->setCheckable(true);
      action->setChecked(field == current);
      connect(action, &QAction::triggered, this,
              [this, setter, field]() { (this->*setter)(field); });
    }
    if(!current.isEmpty() && !field_names.contains(current))
    {
      // The stale persisted field, named and marked so the operator can see WHAT
      // is set and WHY it is doing nothing. Checked — it is the current setting —
      // and left selectable rather than greyed out, because a disabled entry is
      // read as "not available" when the point is that it IS what is in force.
      // "(none)" above is what clears it.
      QAction* action = submenu->addAction(current + " (no numbers)");
      action->setCheckable(true);
      action->setChecked(true);
      connect(action, &QAction::triggered, this,
              [this, setter, current]() { (this->*setter)(current); });
    }
  };

  addFieldMenu(QStringLiteral("Color by"), color_field_, &VectorLayer::setColorField);
  addFieldMenu(QStringLiteral("Size by"), size_field_, &VectorLayer::setSizeField);

  if(field_names.isEmpty())
    return;   // a palette with no field a ramp can sample changes nothing

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

QString canonicalVectorLayerPath(const QString& fname)
{
  // Empty for a path that does not resolve (a dangling symlink, an unmounted
  // share) — see the header: the given spelling is then the best identity there
  // is, and withoutVectorLayerFile() is what copes with the two spellings one
  // file can end up stored under.
  const QString canonical = QFileInfo(fname).canonicalFilePath();
  return canonical.isEmpty() ? fname : canonical;
}

QStringList withoutVectorLayerFile(const QStringList& files, const QString& canonicalFile)
{
  QStringList result;
  for(const QString& file : files)
  {
    if(file == canonicalFile)
      continue;
    // The entry may have been written under a raw spelling that only NOW
    // resolves — the dangling-symlink-then-target-appears case the header
    // describes. Resolving each entry as it is tested is what makes the two
    // spellings one identity.
    if(canonicalVectorLayerPath(file) == canonicalFile)
      continue;
    result << file;
  }
  return result;
}

QStringList withVectorLayerFilePromoted(const QStringList& files, const QString& canonicalFile)
{
  QStringList result;
  for(const QString& file : files)
  {
    // Same identity test withoutVectorLayerFile() uses — an exact match, or an
    // entry whose raw spelling only NOW resolves to this file — but the entry is
    // rewritten in its slot instead of being dropped.
    const bool same_file =
      file == canonicalFile || canonicalVectorLayerPath(file) == canonicalFile;
    const QString entry = same_file ? canonicalFile : file;
    if(!result.contains(entry))
      result << entry;
  }
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
