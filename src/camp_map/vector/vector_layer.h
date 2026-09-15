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
///     ordinary Layers-tab layer with attribute-driven styling and hover-to-
///     inspect: nothing about it is editable, nothing reaches the robot, and it
///     persists as app state in QSettings alongside the chart list (ADR-0003 §4),
///     independent of any mission file.
/// Both read the file through `camp::vector::parseVectorLayers`.
///
/// [ADR-0016 D5] Hover-to-inspect shows an IN-SCENE LABEL the instant the cursor
/// reaches a feature — CAMP's house convention for "tell me what this is"
/// (Platform, AISContact and the mission items all answer to hover), by the same
/// mechanism GeoGraphicsItem gives them, and not a persistent panel. A Qt tooltip
/// was tried first and rejected by the operator in the 2026-09-15 GUI test: it
/// waits out Qt's hover delay, and nothing else in CAMP does. A feature accepts NO mouse button, so every press over one
/// falls through to the view: a pan gesture that starts on a feature pans, and a
/// left-press in one of ProjectView's add-* modes places its mission item with
/// nothing in the way. See VectorFeatureItem.
///
/// [ADR-0016] The design decisions and the persisted schema (`vectorLayers/files`
/// plus the per-layer style group) are recorded in
/// `docs/decisions/0016-read-only-vector-file-layer.md`. Known limitation: a line
/// or polygon whose vertices straddle the antimeridian is drawn the long way
/// round the world and stretches this layer's extent with it; nothing splits
/// geometry at the seam.
///
/// Shape follows `raster::RasterLayer`: the file is parsed off the GUI thread
/// (QFutureWatcher + an abort flag joined in the destructor — the #213 pattern),
/// and the result is turned into one `VectorFeatureItem` child per feature, whose
/// coordinates are transformed to the Web-Mercator scene once (ADR-0002). Per-
/// feature child items — rather than RasterLayer's single painted surface — are
/// what make hover-to-inspect Qt's problem instead of ours.
class VectorLayer: public map::Layer
{
  Q_OBJECT
  Q_INTERFACES(QGraphicsItem)
public:
  /// [camp#22] Upper bound on how much of a vector file this layer reads and draws.
  ///
  /// It bounds TWO things, and it has to be both: item construction happens on the
  /// GUI thread (a QGraphicsItem cannot be built off it), and the parse that feeds
  /// it materialises every geometry and attribute map in the worker. The Open
  /// Vector Layer dialog does not bound what an operator can pick — a national
  /// coastline shapefile or an OSM extract is millions of features — so a cap on
  /// the items alone would leave CAMP responsive and out of memory. The cap is
  /// therefore carried into the parse (`ParseOptions::max_geometries`), which
  /// STOPS at it: the rest of the file is never read.
  ///
  /// What is shown is the first `kMaxFeatureItems` features, and the Layers-tab
  /// status and the log say the cap was hit — a visibly partial layer rather than a
  /// hung or dead application. How many features were left unread is deliberately
  /// not reported: finding out means reading the file the cap exists to stop
  /// reading. The number is a responsiveness-and-memory budget, not a data limit:
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
  /// load completes. This is EVERY field — attribute inspection (the hover label,
  /// ADR-0016 D5) and a future label-by-field want them all; the styling menus
  /// want `numericFields()`.
  QStringList fields() const;

  /// [camp#22] The subset of `fields()` a colour or size ramp can read: a field
  /// is included when at least one loaded feature holds a finite numeric value
  /// for it (`numericAttribute()`). Sorted; empty until the load completes.
  ///
  /// The styling menus offer THESE. Offering every field let the operator colour
  /// by a free-text field, which no ramp can read: the range came back invalid,
  /// every feature was marked no-data, and the whole layer went hollow grey — in
  /// the GUI test of 2026-09-15 that read as the features DISAPPEARING. A ramp
  /// over categories is a different mapping (a distinct colour per class, a
  /// legend), not a degenerate case of this one; it is a follow-on, and until it
  /// exists a field no ramp can read is not offered as one (ADR-0016 D14).
  QStringList numericFields() const;

  /// Colour each feature by its value for @p field, sampled from the active
  /// palette across the field's extent over the features that HAVE a value.
  /// Empty field -> the layer's default colour. A feature whose value is missing
  /// or non-numeric is painted in `noDataColor()`, never at the bottom of the ramp.
  ///
  /// A field NO feature has a numeric value for (a persisted style whose file has
  /// changed, say) is treated as an empty field — default colour, nothing marked
  /// no-data — rather than marking the entire layer no-data. See applyStyle().
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
/// [camp#22] True if @p path would be resolved through GDAL's VIRTUAL FILE SYSTEM
/// — a leading `/vsicurl/`, `/vsizip/`, `/vsis3/`, `/vsigs/` … token.
///
/// This is the check that keeps "open this file" from becoming a network fetch.
/// The driver allowlist in vector_layer.cpp cannot do it: GDAL resolves the /vsi
/// prefix BEFORE it selects a driver, so `/vsicurl/https://host/x.geojson` is
/// downloaded and then handed to the perfectly-allowed GeoJSON driver. A /vsi
/// path is refused by `VectorLayer`'s constructor (before anything is opened) and
/// by `AutonomousVehicleProject` on both the open and the restore path — the
/// restore path matters most, since it reopens every persisted entry at startup
/// with nobody there to confirm it.
bool isVirtualFileSystemPath(const QString& path);

QString vectorLayerFilesKey();
QStringList persistedVectorLayerFiles();
void writePersistedVectorLayerFiles(const QStringList& files);
/// @p files with @p filename appended if it is not already present (de-dup by
/// exact path), preserving order — the chart-list convention.
///
/// Removal from the persisted key itself is still "do not include it in the
/// rebuild", not an edit — see `rebuildPersistedVectorLayerFiles()`.
/// `withoutVectorLayerFile()` below is for the in-memory UNAVAILABLE list, which
/// is an input to that rebuild.
QStringList withVectorLayerFile(const QStringList& files, const QString& filename);

/// The identity of a vector-layer file: its canonical path, or the given
/// spelling when the file does not resolve.
///
/// [camp#22] "./survey.geojson", an absolute path and a symlink all name one
/// file, and de-dup, removal and persistence all key on the file itself rather
/// than on the spelling that reached us. `QFileInfo::canonicalFilePath()`
/// resolves symlinks and "." / ".." and returns EMPTY for a path that does not
/// resolve — a dangling symlink, an unmounted share — in which case the given
/// path is the best identity available. That fallback is exactly why removal
/// needs `withoutVectorLayerFile()` below and not a plain string compare.
QString canonicalVectorLayerPath(const QString& fname);

/// @p files with every entry NAMING THE SAME FILE as @p canonicalFile dropped:
/// an exact string match, plus any entry whose `canonicalVectorLayerPath()`
/// resolves to @p canonicalFile. Order of the survivors is preserved.
///
/// [camp#22 / camp#90] Why identity and not `removeAll()`: an entry is only
/// canonical when its file resolved AT THE TIME IT WAS WRITTEN. A dangling
/// symlink is persisted under its RAW spelling; when the target later appears
/// and the operator opens that same symlink, the path in hand is now the
/// resolved target, and an exact-match removal misses the raw entry. The rebuild
/// then keeps finding it on the unavailable branch and writes it back, so
/// removing the reopened layer through the Layers tab does not stick and the
/// layer returns on the next launch — the camp#90/#117 class again. (This is the
/// counterpart the header once said it deliberately lacked; it now has the
/// production caller — `AutonomousVehicleProject::openVectorLayer()` — that its
/// absence was justified by.)
///
/// [camp#22 round-7 suggestion] COST, and why it is accepted. Resolving each
/// surviving entry means one `canonicalFilePath()` — a stat/readlink — per entry,
/// on the GUI thread, and the motivating case in the paragraph above is an
/// UNMOUNTED SHARE, the kind of path that can block for a mount's timeout rather
/// than returning promptly. Accepted because:
///  * it is bounded by the length of the in-memory unavailable list, which is
///    operator-sized (the files they opened and could not load), not file-sized;
///  * the call site already stats the same class of path immediately before, via
///    `QFileInfo::exists(fname)`, so this amplifies an existing exposure by the
///    list length rather than introducing a new one; and
///  * the alternative is making the purge asynchronous, which would race
///    `persistVectorLayers()` — the purge has to have happened before the next
///    persist, or the removed entry is written back and the camp#90/#117 bug
///    returns. A latency risk on a list of a few entries is the smaller cost.
/// If the unavailable list ever grows unbounded (it is not persisted per-session
/// today), this is the site to revisit.
QStringList withoutVectorLayerFile(const QStringList& files, const QString& canonicalFile);

/// The persisted vector-layer list rebuilt from scratch — the whole rule behind
/// `AutonomousVehicleProject::persistVectorLayers()`, in one pure function so it
/// can be exercised (the project itself is not constructible in a test harness).
///
/// @p restoredOrder is the order of record read at startup, @p unavailable the
/// subset of it whose files could not be opened THEN and are carried forward
/// rather than forgotten, and @p loadedFiles the filenames of the layers tracked
/// right now, in load order.
///
/// Order comes from @p restoredOrder first, so one launch with a share unmounted
/// cannot reshuffle the operator's layers; an entry in it that is neither loaded
/// nor unavailable was REMOVED through the Layers tab and is dropped. Anything
/// opened since the restore follows, in load order.
///
/// [camp#22 / camp#90] The caller owes this function an @p unavailable list that
/// has been kept CURRENT: an entry must be dropped from it as soon as its file is
/// successfully opened, or removing that layer afterwards will not stick — the
/// rebuild would keep finding it on the unavailable branch and write it back on
/// every launch, which is the camp#90/#117 bug this whole mechanism exists to
/// avoid. `AutonomousVehicleProject::openVectorLayer()` is where that happens.
QStringList rebuildPersistedVectorLayerFiles(const QStringList& restoredOrder,
                                             const QStringList& unavailable,
                                             const QStringList& loadedFiles);

}  // namespace camp::vector

#endif  // CAMP_VECTOR_LAYER_H
