#ifndef CAMP_VECTOR_LAYER_H
#define CAMP_VECTOR_LAYER_H

#include <string>
#include <vector>

#include <QFutureWatcher>
#include <QMutex>
#include <QString>
#include <QStringList>

#include "../map/layer.h"
#include "vector_parse.h"
#include "vector_style.h"

namespace camp::vector
{

class VectorFeatureItem;

/// [camp#22 / ADR-0003] Read-only display layer for an OGR-readable vector file
/// (GeoJSON, shapefile, GeoPackage, KML — whatever the driver opens).
///
/// **VectorLayer vs. VectorDataset.** CAMP has two vector-file entry points and
/// they are not interchangeable:
///   - `VectorDataset` (src/camp/vector/, File > Open Geometry) imports a file as
///     EDITABLE mission-tree nodes — Group/Point/LineString/Polygon MissionItems
///     the operator can drag, rename and send to the robot. It persists in the
///     mission project file.
///   - `VectorLayer` (this class, File > Open Vector Layer) DISPLAYS a file as an
///     ordinary Layers-tab layer with attribute-driven styling and click-to-
///     inspect: nothing about it is editable, nothing reaches the robot, and it
///     persists as app state in QSettings alongside the chart list (ADR-0003 §4),
///     independent of any mission file.
/// Both read the file through `camp::vector::parseVectorLayers`.
///
/// Shape follows `raster::RasterLayer`: the file is parsed off the GUI thread
/// (QFutureWatcher + an abort flag joined in the destructor — the #213 pattern),
/// and the result is turned into one `VectorFeatureItem` child per feature, whose
/// coordinates are transformed to the Web-Mercator scene once (ADR-0002). Per-
/// feature child items — rather than RasterLayer's single painted surface — are
/// what make click-to-inspect Qt's problem instead of ours.
class VectorLayer: public map::Layer
{
  Q_OBJECT
  Q_INTERFACES(QGraphicsItem)
public:
  /// [camp#22] Upper bound on the number of feature ITEMS this layer builds.
  ///
  /// Item construction happens on the GUI thread (a QGraphicsItem cannot be built
  /// off it), and the Open Vector Layer dialog does not bound what an operator can
  /// pick: a national coastline shapefile or an OSM extract is millions of
  /// features, which would freeze CAMP with no way out and no message. Beyond this
  /// many the layer draws the first `kMaxFeatureItems` and REPORTS the shortfall in
  /// its Layers-tab status and the log — a visibly partial layer rather than a hung
  /// application. The number is a GUI-responsiveness budget, not a data limit:
  /// 50 000 items build in well under a second and the scene index handles them.
  static constexpr int kMaxFeatureItems = 50000;

  /// @param feature_cap  test seam; see kMaxFeatureItems, which is the value the
  ///                     application uses. Values <= 0 are treated as the default.
  VectorLayer(map::MapItem* parentItem, const QString& filename,
              int feature_cap = kMaxFeatureItems);
  ~VectorLayer();

  enum { Type = map::VectorLayerType };
  int type() const override
  {
    return Type;
  }

  /// The union of the feature items' extents. MapItem::boundingRect() is empty by
  /// default, so this override is what gives the layer a meaningful extent.
  QRectF boundingRect() const override;

  /// Source file this layer renders — its identity for persistence and removal.
  const QString& filename() const { return filename_; }

  /// [camp#126 precedent] Keyed on the full path, NOT the itemID() default: that
  /// derives from objectName() = the file's BASENAME, so two vector files named
  /// candidates.geojson in different directories would share one settings group
  /// and overwrite each other's style.
  QString settingsKey() const override;

  /// Attribute field names present on any loaded feature, sorted. Empty until the
  /// load completes.
  QStringList fields() const;

  /// Colour each feature by its value for @p field, sampled from the active
  /// palette across the field's extent over the features that HAVE a value.
  /// Empty field -> the layer's default colour. A feature whose value is missing
  /// or non-numeric is painted in `noDataColor()`, never at the bottom of the ramp.
  void setColorField(const QString& field);
  const QString& colorField() const { return color_field_; }

  /// Scale each POINT feature's marker radius by its value for @p field, between
  /// kMinPointRadius and kMaxPointRadius. Lines and polygons ignore this. A
  /// missing or non-numeric value gets kDefaultPointRadius, not the smallest radius.
  void setSizeField(const QString& field);
  const QString& sizeField() const { return size_field_; }

  /// marine_colormap palette for colour-by-field (ADR-0008). Unknown name ->
  /// grayscale, matching the raster layers.
  void setColormap(const std::string& name);
  const std::string& colormap() const { return colormap_; }

  /// The cap actually in force for this layer (kMaxFeatureItems unless overridden).
  int featureCap() const { return feature_cap_; }

  /// Number of feature items built from the file. Zero until the load completes.
  int featureCount() const { return static_cast<int>(features_.size()); }

  /// True once the load finished and produced at least one feature. A file that
  /// failed to open, or held nothing this parser handles, is false and shows
  /// "(load failed)" / "(no features)" in the Layers tab.
  bool loaded() const { return loaded_; }

signals:
  /// [camp#22] This layer was REMOVED from the map — emitted from
  /// onRemovedFromMap(), so it fires only on `Layer::removeFromMap()` (the
  /// Layers-tab Remove action or a programmatic detach) and NOT on the
  /// remove-then-insert that `Map::setMapItemParent()` performs when a layer is
  /// dragged to a new position in the list.
  ///
  /// That distinction is the whole reason this hook exists: the owner used to
  /// watch the model's `rowsAboutToBeRemoved`, which cannot tell a reorder from a
  /// removal, so dragging a vector layer up or down the Layers tab silently
  /// dropped its file from `vectorLayers/files` and the layer did not come back
  /// on the next launch. RasterLayer and GggsTileLayer de-persist through
  /// `onRemovedFromMap()` for exactly this reason (camp#90 / camp#104).
  ///
  /// The signal carries no payload: the owner (AutonomousVehicleProject) matches
  /// on `sender()` and remains the single writer of `vectorLayers/files`.
  void removedFromMap();

protected:
  /// [camp#22 / camp#90] Reorder-safe removal notification — see removedFromMap().
  /// Deliberately does NOT write `vectorLayers/files` itself: that key has one
  /// writer, `AutonomousVehicleProject::persistVectorLayers()`, which rebuilds it
  /// from the layers it tracks. (RasterLayer writes its key here because it owns
  /// a key of its own; this layer does not.)
  void onRemovedFromMap() override;

  void contextMenu(QMenu* menu) override;
  void readSettings() override;
  void writeSettings() override;

private:
  struct LoadResult
  {
    bool opened = false;
    std::vector<ParsedLayer> layers;
    ParseDiagnostics diagnostics;
  };

  /// Worker body (QtConcurrent pool thread): open the file with GDAL and parse it.
  LoadResult loadVectorFile(const QString& filename);

  /// Thread-safe read of the abort flag. Called on the worker thread, including
  /// from inside the parser's per-feature poll (ParseOptions::aborted).
  bool isAborted();

  QFutureWatcher<LoadResult> future_watcher_;
  // Set under the mutex to tell an in-flight load to stop; the destructor sets it
  // and then joins, so the worker never outlives `this` (#213).
  bool abort_flag_ = false;
  QMutex abort_flag_mutex_;

  QString filename_;
  bool loaded_ = false;
  int feature_cap_ = kMaxFeatureItems;

  std::vector<VectorFeatureItem*> features_;   // children; owned by the scene tree
  QString color_field_;
  QString size_field_;
  std::string colormap_ = "viridis";

  /// Recompute every feature's colour and radius from the current style. One pass
  /// over the already-loaded features — no re-parse, no re-load.
  void applyStyle();

private slots:
  void loadFinished();
};

/// [camp#22 / ADR-0003 §4] The persisted vector-layer file list — app state, the
/// same shape as `backgrounds/files` for charts, NOT the mission project file: a
/// Layers-tab layer is not mission data, and the operator expects it back when
/// CAMP reopens whether or not a mission is loaded.
///
/// These are the mechanism only. `AutonomousVehicleProject::persistVectorLayers()`
/// is the single writer of the key — it rebuilds the whole list from its tracked
/// layers on both the add and the remove path — so the two halves can never
/// disagree about what is persisted. They live here (rather than in the project,
/// which no test can construct) so the add/dedup/remove rules are testable.
QString vectorLayerFilesKey();
QStringList persistedVectorLayerFiles();
void writePersistedVectorLayerFiles(const QStringList& files);
/// @p files with @p filename appended if it is not already present (de-dup by
/// exact path), preserving order — the chart-list convention.
QStringList withVectorLayerFile(const QStringList& files, const QString& filename);
/// @p files with EVERY entry equal to @p filename removed.
QStringList withoutVectorLayerFile(const QStringList& files, const QString& filename);

}  // namespace camp::vector

#endif  // CAMP_VECTOR_LAYER_H
