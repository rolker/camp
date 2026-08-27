#ifndef CAMP_ROS_LIVE_COVERAGE_SONAR_LIVE_CACHE_LAYER_H
#define CAMP_ROS_LIVE_COVERAGE_SONAR_LIVE_CACHE_LAYER_H

#include "../layer.h"
#include "../../raster/raster_field_source.h"
#include "../../raster/raster_gl_renderer.h"
#include "sonar_live_tile.h"

#include <marine_colormap/transfer.hpp>

#include "marine_tiled_raster_store/tile_catalog.hpp"

#include "marine_interfaces/msg/sonar_visualization_tile.hpp"
#include "marine_interfaces/msg/tile_catalog.hpp"
#include "marine_interfaces/msg/tile_request.hpp"

#include <rclcpp/qos.hpp>

#include <QFutureWatcher>
#include <QImage>
#include <QPointF>
#include <QRectF>
#include <QSize>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

class QOpenGLTexture;

namespace camp
{
namespace ros
{
namespace live_coverage
{

/// QoS used for the `<base>/coverage_catalog` subscription: RELIABLE, depth 1,
/// and — [field 2026-08-27] — **VOLATILE**.
///
/// Exposed (rather than inlined at the subscribe site) so the durability can be
/// pinned by a regression test: the operator-side subscriber receives the catalog
/// republished by `udp_bridge`, which is volatile, and a transient-local
/// subscriber never matches it. Nothing pinned that, which is exactly why the
/// mismatch went unnoticed. See the definition for the full rationale.
rclcpp::QoS catalogSubscriptionQos();

/// [camp#121] Live coverage cache layer for ONE boat-side source namespace.
///
/// Renders dequantized in-memory `SonarLiveTile`s through a GL pipeline
/// duplicated from `GggsTileLayer` (no GDAL on the hot path) and drives
/// anti-entropy convergence with a `TileCatalogReconciler`. See ADR-0006.
///
/// **Activation model (ADR-0006 D5).** The layer is spawned by discovery in a
/// *discovered-but-inactive* state: it may subscribe to the cheap, low-rate
/// `coverage_catalog` to advertise availability, but it does NOT subscribe to the
/// best-effort `coverage_tiles` stream and does NOT publish `TileRequest` until the
/// operator enables it (context-menu "Enable live coverage"). Enabling warm-loads
/// the disk cache, subscribes to the tile stream, and starts request/prune. The
/// enabled flag persists per source (QSettings), so an enabled source re-subscribes
/// on warm restart while a never-enabled one stays passive. This serves #71 (no
/// surprise bandwidth on a slow link).
///
/// **Threading invariant (ADR-0001 / ADR-0006 D4).** `TileCatalogReconciler` and
/// the tile map are NOT thread-safe. They are touched ONLY on the GUI thread. The
/// ROS subscription callbacks do nothing but copy the message and marshal it to the
/// GUI thread (QMetaObject::invokeMethod, queued); the GUI-thread handlers do all
/// reconcile / markHave / drop / applyPatch / write-through scheduling. Warm-load
/// runs on the GUI thread before the subscriptions are created, so it cannot race a
/// callback.
class SonarLiveCacheLayer: public Layer, public raster::RasterFieldSource
{
  Q_OBJECT
  Q_INTERFACES(QGraphicsItem)
public:
  /// @param base_namespace the remapped source namespace (e.g. "/cube_bathymetry").
  ///   The topics are `<base>/coverage_tiles`, `<base>/coverage_catalog`,
  ///   `<base>/coverage_requests`.
  SonarLiveCacheLayer(MapItem* parent, Node* node, const QString& base_namespace);
  ~SonarLiveCacheLayer() override;

  enum { Type = map::SonarLiveCacheLayerType };
  int type() const override { return Type; }

  QRectF boundingRect() const override;
  void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;

  /// Identity-stable QSettings key (the source namespace, not the display label).
  QString settingsKey() const override;

  /// Whether the tile stream is currently subscribed (operator-enabled).
  bool isEnabled() const { return enabled_; }

  /// [camp#160] Resident fine-tile count and coarse overview (pyramid) tile count.
  /// Introspection for tests and the future status indicator (#158).
  std::size_t residentTileCount() const { return tiles_.size(); }
  std::size_t overviewTileCount() const { return overview_tiles_.size(); }
  /// [camp#171] Per-pool resident footprint (bytes). The eviction-headroom test needs
  /// the overview-pool total to assert the pyramid stays bounded well under budget
  /// (accountedBytes() is fine + overview combined). CPU band data + any GL texture.
  std::size_t fineResidentBytes() const;
  std::size_t overviewResidentBytes() const;
  /// [camp#160] Total resident footprint (fine + overview), the eviction budget metric.
  std::size_t accountedBytes() const;
  /// [camp#172] Count of fine indices evicted but still recoverable from disk (the
  /// on-demand reload set). Introspection for the reload regression test.
  std::size_t evictedFineCount() const { return evicted_fine_indices_.size(); }
  /// [camp#160] Number of indices the reconciler holds — used by tests to assert
  /// overview tiles never enter the anti-entropy set (ADR-0010 D4).
  std::size_t reconcilerHeldCount() const { return reconciler_.size(); }

  /// [camp#121] Render the in-memory tiles into an offscreen image of @p size
  /// spanning the layer extent. Null image if there is no data / GL is
  /// unavailable. Exposed for a headless render check (skips with no GL).
  QImage renderImage(const QSize& size);

  /// [camp#103 / ADR-0011] Clip-aware overload: render only the tiles whose
  /// scene extent intersects @p clip_bounds (a sub-rect of sceneBounds(),
  /// Web-Mercator metres) into an image of @p size spanning exactly
  /// @p clip_bounds. Filters BOTH pools while preserving the overviews-first
  /// draw order (the ADR-0010 LOD fallback). paint() uses it with the
  /// viewport-derived clip; public (mirroring renderImage(size)) for tests.
  QImage renderImage(const QSize& size, const QRectF& clip_bounds);

  /// The layer's Web-Mercator extent (union of tile extents). Exposed for tests.
  QRectF sceneBounds() const { return scene_bounds_; }

  /// [camp#172] Test/headless seam mirroring `GggsTileLayer::waitForLoad()`: kick the
  /// on-demand reload for @p viewport_scene (if idle, the viewport moved since the last
  /// kick, and it exposes an evicted visible tile), then join the worker and run the
  /// ready-slot so the reloaded tiles are resident on return. paint() drives the same
  /// path live; this lets a test drive it deterministically without a scene/view.
  void waitForReload(const QRectF& viewport_scene);

  /// [camp#172] Test seam: override the resident-footprint eviction budget after
  /// construction (the ctor reads it once from QSettings). Lets a reload test create
  /// budget headroom — or remove it — mid-test to exercise the D6 hysteresis gate.
  void setResidentBudgetForTest(std::size_t bytes) { vram_budget_bytes_ = bytes; }

  /// [camp#142] Per-layer colormap range override. Auto tracks the data extents
  /// (data_min_/data_max_, folded in foldAutoRange()); Manual pins an operator
  /// [lo, hi] so an outlier band can't collapse the useful colour range. Applied at
  /// render time (shader u_min/u_max via renderToImage), transparent to the fold.
  /// [camp#132] Toggle the QPainter blit smoothing (default OFF = Nearest).
  /// Blit hint only — the GL scalar filter stays Nearest (camp#122 NoData-halo
  /// guard). Persists (settingsKey group) and repaints.
  void setSmoothInterpolation(bool smooth);
  bool smoothInterpolation() const { return smooth_interpolation_; }

  void setRangeOverride(float lo, float hi);   ///< -> Manual [lo, hi]
  void resetRangeToAuto();                      ///< -> Auto (tracks the data extents)
  marine_colormap::RangeMode rangeMode() const { return range_model_.mode(); }
  float rangeLo() const { return range_model_.lo(); }
  float rangeHi() const { return range_model_.hi(); }

  // [camp#134] RasterFieldSource: feed the shared RasterGlRenderer. bands() are the
  // named live bands; items() returns the held tiles as Scalar items (textures
  // uploaded lazily under the renderer's current context).
  QStringList bands() const override;
  raster::RasterBandMeta metadata(const QString& band) const override;
  QList<raster::RasterFieldItem> items() override;
  QPair<float, float> dataRange() const override;

public slots:
  /// [camp#121] Subscribe to the tile stream, warm-load the disk cache, start
  /// request/prune, and persist enabled=true. No-op if already enabled.
  void enableLiveCoverage();
  /// [camp#121] Unsubscribe from the tile stream and persist enabled=false. The
  /// in-memory + on-disk cache and the (cheap) catalog subscription are kept.
  void disableLiveCoverage();

protected:
  void contextMenu(QMenu* menu) override;
  void readSettings() override;
  void writeSettings() override;

private slots:
  /// GUI thread: a tile patch arrived (newest-wins, applyPatch, markHave,
  /// schedule write-through, repaint).
  void handleTile(const marine_interfaces::msg::SonarVisualizationTile& msg);
  /// GUI thread: a catalog arrived (reconcile -> request + prune).
  void handleCatalog(const marine_interfaces::msg::TileCatalog& msg);

private:
  // [camp#121] One held tile plus its lazily-(re)uploaded GL texture for the
  // currently selected band. texture_dirty marks the band data changed since the
  // last upload so the render re-uploads it.
  struct Entry
  {
    SonarLiveTile tile;
    std::unique_ptr<QOpenGLTexture> texture;
    bool texture_dirty = true;
    // [camp#160] Monotonic access stamp for the LRU eviction fallback (bumped on
    // patch + on render). Unused for overview entries (they are never evicted).
    std::uint64_t last_access_seq = 0;
  };

  void subscribeCatalog();
  void subscribeTiles();
  void unsubscribeTiles();
  void publishRequest(const std::vector<gggs::GridIndex>& tiles);
  void warmLoad();
  // [camp#121] Coalesced, joinable per-tile write-through. scheduleWriteThrough()
  // snapshots the latest tile state and either launches a worker (if the tile is
  // idle) or marks it dirty (if a worker is already in flight for that tile);
  // startWriteThrough() moves the snapshot into a self-contained worker;
  // onWriteThroughFinished() (GUI thread) re-launches once more if a newer patch
  // arrived during the write — so writes for a tile are serialized and never race
  // on the shared <stem>.tif.tmp path.
  // [camp#160] @p subdir is a cache sub-directory ("" = fine tiles at cache_dir_
  // root, "overviews" = pyramid parents) so fine and overview writes never share
  // a `<stem>.tif.tmp` path. Fine and overview indices are at different GGGS
  // levels, so the per-index write-state map keys never collide.
  void scheduleWriteThrough(const SonarLiveTile& tile, const std::string& subdir = "");
  void startWriteThrough(const gggs::GridIndex& index);
  void onWriteThroughFinished(const gggs::GridIndex& index,
                              QFutureWatcher<void>* watcher);

  // [camp#160] Bounded eviction + overview pyramid (view-based LOD).
  // accountedBytes(): resident fine-tile footprint (CPU band data + any uploaded
  // GL texture). evictIfOverBudget(): while over vram_budget_bytes_, fold the
  // fine tile farthest from the viewport centre into its coarse parent, persist
  // it, free its texture, and drop it (LRU by last_access_seq when no view is
  // attached). foldIntoParent(): 2x2-decimate a fine tile up the full overview
  // chain to level 0. currentViewCentre(): viewport centre in Web-Mercator scene
  // coords, or nullopt when headless.
  // [camp#171] Resident footprint of one entry (CPU band data + any GL texture). The
  // per-pool byte seams and accountedBytes() all sum this.
  static std::size_t entryBytes(const Entry& entry);
  void evictIfOverBudget();
  void foldIntoParent(const SonarLiveTile& fine);
  std::optional<QPointF> currentViewCentre() const;

  // [camp#172] On-demand reload of evicted fine tiles (the ADR-0013 §"camp#172 hook").
  // hasUnloadedVisibleTiles(): does the viewport expose an evicted fine index whose
  // disk copy is not yet resident? (the ADR-0013-named reload predicate; tests the
  // GGGS extent of the index, no live tile needed). kickReload(): snapshot the visible
  // evicted indices + cache_dir_ + level into a self-contained QtConcurrent worker that
  // loads each cached fine GeoTIFF; connect its watcher to onReloadFinished().
  // onReloadFinished() (GUI thread): insert the loaded tiles back into tiles_, clear
  // EVERY attempted index from evicted_fine_indices_ (loaded or not, so a permanently
  // unloadable index can't re-kick forever), and run evictIfOverBudget() (D6 hysteresis
  // keeps it from re-evicting the inserts). See ADR-0010 D2/D6.
  bool hasUnloadedVisibleTiles(const QRectF& viewport_scene) const;
  void kickReload(const QRectF& viewport_scene);
  void onReloadFinished();

  void recomputeBounds();
  void resetAutoRange();
  void foldAutoRange();
  void updateDisplay();   // refresh display name + status from enabled_/tile count

  // [camp#121] Band selection (by NAME, since live bands are named, unlike the
  // GggsTile 1-indexed bands). Default picks "depth" if present else the first.
  std::string defaultBand() const;
  void setColormap(const std::string& name);   // [camp#141] marine_colormap palette
  void setBandName(const std::string& name);

  // [camp#134] (Re)upload the selected band of @p entry as an R32F value texture
  // (owned by the Entry). The shader + LUT + tessellation now live in renderer_.
  QOpenGLTexture* textureFor(Entry& entry);

  /// [camp#103] items() body with an optional scene-space clip: a non-null
  /// @p clip_scene keeps only tiles whose Web-Mercator extent intersects it,
  /// tested BEFORE the lazy texture upload. Filters both pools, preserving the
  /// overviews-first draw order (ADR-0010 LOD fallback). items() (the
  /// RasterFieldSource interface) delegates with a null rect.
  QList<raster::RasterFieldItem> itemsIntersecting(const QRectF& clip_scene);

  static constexpr int kMaxImageEdge = 4096;

  // [camp#160] Overview tiles at level <= this coarse "apex" are never evicted, so a
  // whole-survey zoomed-out view always has coverage. The apex is inherently a small,
  // bounded handful for any realistic survey extent (level-6 tiles span ~0.125 deg),
  // while the numerous near-fine overview levels ARE evicted by view like fine tiles —
  // so total resident memory stays bounded (ADR-0010 D1/D3).
  static constexpr std::uint8_t kApexProtectLevel = 6;

  // [camp#172 / ADR-0010 D6] Reload only when resident footprint is below this fraction
  // of the budget, so a reload insert can't immediately re-trigger eviction (no
  // reload<->evict ping-pong). 0.75 leaves a quarter-budget headroom for the inserts.
  static constexpr double kReloadHysteresisFactor = 0.75;

  std::string base_namespace_;   // e.g. "/cube_bathymetry"
  std::string cache_dir_;        // <base cache dir>/<sanitized source ns>
  bool enabled_ = false;         // tile stream subscribed (persisted)

  // GGGS level of this source's tiles, learned from the first message/cached tile.
  // Needed to recover a GridIndex from a cached GeoTIFF on warm-load.
  std::optional<std::uint8_t> level_;

  // [camp#169] Last catalog seen, buffered even while disabled. The catalog is
  // published only on change and the boat's catalog is stable, so a sample
  // arriving while enabled_ is false would otherwise be discarded and never
  // re-delivered — requests would not resume until a restart (the 2026-07-23
  // field incident's timing race). enableLiveCoverage() replays this buffer
  // through handleCatalog() so reconcile/request fire on every enable.
  // [field 2026-08-27] With the subscription now volatile (see catalogSubscriptionQos())
  // there is no latched sample either, which makes this buffer the only bridge
  // from a catalog seen while disabled to a reconcile on enable.
  std::optional<marine_interfaces::msg::TileCatalog> last_catalog_;

  // [camp#134] The shared GL raster renderer (its own offscreen context + the
  // unified shader + colormap LUT). Replaces the shader/program/LUT/FBO this layer
  // used to duplicate from GggsTileLayer. Default ramp Grayscale (renderer default).
  // Declared BEFORE the texture-holding tiles_ so reverse-declaration destruction
  // tears the tiles (and their Entry GL textures) down before the renderer's
  // context — the invariant the dtor body already enforces explicitly, now also
  // structural.
  raster::RasterGlRenderer renderer_;

  marine_tiled_raster_store::TileCatalogReconciler reconciler_;
  std::map<gggs::GridIndex, Entry> tiles_;

  // [camp#160] Coarse overview (pyramid) tiles keyed by their own GGGS index at a
  // level below tiles_'s. Built by folding evicted fine tiles into their parents,
  // kept resident (never evicted — a handful of tiles), and rendered *under* the
  // fine tiles so an evicted area degrades to a coarser resolution instead of
  // going blank. These are a LOCAL derived product: they are NEVER entered into
  // `reconciler_`, so anti-entropy prune-on-absence can't delete them.
  std::map<gggs::GridIndex, Entry> overview_tiles_;

  // [camp#172] Fine indices that were evicted (folded to overview + dropped from
  // tiles_) but whose disk copy is still present — the on-demand reload candidate set
  // (ADR-0010 D2). Populated in evictIfOverBudget() phase 1; an index leaves the set
  // when it is reloaded (onReloadFinished, whether the load succeeded or not),
  // re-received live (handleTile), or pruned by the catalog (handleCatalog).
  std::set<gggs::GridIndex> evicted_fine_indices_;

  // [camp#172] Async reload worker (loads evicted fine GeoTIFFs off the GUI thread) and
  // the moved-since-last-kick guard, mirroring GggsTileLayer's demand-driven loader.
  // reload_attempted_ is the snapshot of indices the in-flight worker is loading, so
  // onReloadFinished() can clear them from evicted_fine_indices_ (GUI-thread-only, the
  // worker never touches it). It is non-empty for exactly the span between a kickReload()
  // and its matching onReloadFinished(), so the kick gates also require it empty — that
  // closes the queued-`finished` race where isRunning() has flipped false but the result
  // is not yet consumed (see kickReload). last_reload_viewport_ gates paint()'s re-kick to
  // actual viewport moves (a permanently-unloadable index re-kicks at most once per
  // viewport).
  QFutureWatcher<std::vector<SonarLiveTile>> reload_watcher_;
  std::vector<gggs::GridIndex> reload_attempted_;
  QRectF last_reload_viewport_;

  // [camp#172] Footprint of the last fine tile evicted (set in evictIfOverBudget phase 1,
  // so it is always non-zero once there is anything in evicted_fine_indices_ to reload).
  // kickReload uses it to estimate how many fine tiles a reload would bring resident and
  // cap the per-kick volume, so a wide zoom-out can't reload the whole survey in one shot
  // and spike over budget before onReloadFinished()'s evict runs (ADR-0010 D6).
  std::size_t last_evicted_fine_bytes_ = 0;

  // [camp#160] Monotonic access counter feeding Entry::last_access_seq (LRU
  // eviction fallback), and the resident-footprint budget for eviction
  // (QSettings "LiveTileCache/max_vram_bytes", default 512 MiB — ADR-0006 D2 /
  // #117 no-hardcoded-defaults).
  std::uint64_t access_seq_ = 0;
  std::size_t vram_budget_bytes_ = 0;
  bool eviction_warned_ = false;   // log the shed-load path once, not per evict

  QRectF scene_bounds_;
  double data_min_ = 1.0;        // auto-range over the selected band (crossed => none)
  double data_max_ = 0.0;

  // [camp#142] Resolved colormap range (Auto tracks data_min_/data_max_ via
  // update_auto() in foldAutoRange(); Manual pins an operator override). Fed to the
  // renderer's u_min/u_max at render time, replacing the raw data_min_/data_max_.
  marine_colormap::RangeModel range_model_;

  std::string band_name_;        // selected band (persisted)

  rclcpp::Subscription<marine_interfaces::msg::SonarVisualizationTile>::SharedPtr tile_sub_;
  rclcpp::Subscription<marine_interfaces::msg::TileCatalog>::SharedPtr catalog_sub_;
  rclcpp::Publisher<marine_interfaces::msg::TileRequest>::SharedPtr request_pub_;

  // [camp#121] Per-tile write-through state (GUI thread only). `in_flight` marks a
  // worker is serializing this tile; `dirty` marks a newer patch arrived during
  // that write (coalesced into one follow-up); `pending` holds the latest snapshot
  // to write (moved into the worker on launch).
  struct WriteState
  {
    bool in_flight = false;
    bool dirty = false;
    std::optional<SonarLiveTile> pending;   // SonarLiveTile has no default ctor
    std::string subdir;                     // [camp#160] "" (root) or "overviews"
  };
  std::map<gggs::GridIndex, WriteState> write_states_;

  // Every in-flight write worker is tracked here so the dtor joins ALL of them (not
  // just the latest). Workers are self-contained (file serialization + rename only);
  // they never touch this object. `shutting_down_` suppresses coalesced relaunches
  // once teardown starts.
  std::vector<QFutureWatcher<void>*> write_watchers_;
  bool shutting_down_ = false;

  // [camp#103] Last render, keyed by FBO size AND viewport clip: zoom changes
  // the size, pan changes the clip, so both re-render (the FBO is viewport-sized,
  // so the per-frame pan re-render is cheap).
  QImage cached_image_;
  QSize cached_size_;
  QRectF cached_clip_;
  bool smooth_interpolation_ = false;   // [camp#132] blit hint only (persisted)
};

}  // namespace live_coverage
}  // namespace ros
}  // namespace camp

#endif
