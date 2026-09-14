#include "vector_layer.h"

#include <algorithm>
#include <memory>
#include <set>

#include <QFileInfo>
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

}  // namespace

VectorLayer::VectorLayer(map::MapItem* parentItem, const QString& filename):
  map::Layer(parentItem, QFileInfo(filename).fileName()),
  filename_(filename)
{
  if(GDALGetDriverCount() == 0)
    GDALAllRegister();
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
  {
    abort_flag_mutex_.lock();
    const bool aborted = abort_flag_;
    abort_flag_mutex_.unlock();
    if(aborted)
      return result;
  }

  // [#152] RAII-close the dataset on every path; parseVectorLayers opens none of
  // its own and frees the transforms/iterators it does create.
  const auto gdal_closer = [](GDALDataset* d){ if(d) GDALClose(d); };
  std::unique_ptr<GDALDataset, decltype(gdal_closer)> dataset(
    static_cast<GDALDataset*>(
      GDALOpenEx(filename.toUtf8().constData(), GDAL_OF_READONLY | GDAL_OF_VECTOR,
                 nullptr, nullptr, nullptr)),
    gdal_closer);
  if(!dataset)
    return result;

  result.opened = true;
  result.layers = parseVectorLayers(dataset.get());
  return result;
}

void VectorLayer::loadFinished()
{
  const LoadResult result = future_watcher_.result();
  if(!result.opened)
  {
    setStatus("(load failed)");
    return;
  }

  prepareGeometryChange();
  for(const ParsedLayer& layer : result.layers)
    for(const ParsedGeometry& geometry : layer.geometries)
      features_.push_back(new VectorFeatureItem(this, geometry));

  loaded_ = !features_.empty();
  if(!loaded_)
  {
    // The driver opened the file but this parser found nothing it renders. Say
    // so — an empty layer that claims to have loaded is indistinguishable from
    // one drawn off-screen.
    setStatus("(no features)");
    return;
  }
  setStatus(QString("(%1 features)").arg(features_.size()));

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

QStringList withoutVectorLayerFile(const QStringList& files, const QString& filename)
{
  QStringList result;
  for(const QString& file : files)
    if(file != filename && !result.contains(file))
      result << file;
  return result;
}

}  // namespace camp::vector
