#ifndef CAMP_ROS_LIVE_COVERAGE_SONAR_LIVE_CACHE_LAYER_H
#define CAMP_ROS_LIVE_COVERAGE_SONAR_LIVE_CACHE_LAYER_H

#include "../layer.h"
#include "../../map/color_map.h"
#include "sonar_live_tile.h"

#include "marine_tiled_raster_store/tile_catalog.hpp"

#include "marine_interfaces/msg/sonar_visualization_tile.hpp"
#include "marine_interfaces/msg/tile_catalog.hpp"
#include "marine_interfaces/msg/tile_request.hpp"

#include <QFutureWatcher>
#include <QImage>
#include <QSize>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

class QOpenGLContext;
class QOffscreenSurface;
class QOpenGLFramebufferObject;
class QOpenGLShaderProgram;
class QOpenGLTexture;

namespace camp
{
namespace ros
{
namespace live_coverage
{

/// [camp#121] Live coverage cache layer for ONE boat-side source namespace.
///
/// Renders dequantized in-memory `SonarLiveTile`s through a GL pipeline
/// duplicated from `GggsTileLayer` (no GDAL on the hot path) and drives
/// anti-entropy convergence with a `TileCatalogReconciler`. See ADR-0006.
///
/// **Activation model (ADR-0006 D5).** The layer is spawned by discovery in a
/// *discovered-but-inactive* state: it may subscribe to the cheap, transient-local
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
class SonarLiveCacheLayer: public Layer
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

  /// [camp#121] Render the in-memory tiles into an offscreen image of @p size
  /// spanning the layer extent. Null image if there is no data / GL is
  /// unavailable. Exposed for a headless render check (skips with no GL).
  QImage renderImage(const QSize& size);

  /// The layer's Web-Mercator extent (union of tile extents). Exposed for tests.
  QRectF sceneBounds() const { return scene_bounds_; }

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
  void scheduleWriteThrough(const SonarLiveTile& tile);
  void startWriteThrough(const gggs::GridIndex& index);
  void onWriteThroughFinished(const gggs::GridIndex& index,
                              QFutureWatcher<void>* watcher);

  void recomputeBounds();
  void resetAutoRange();
  void foldAutoRange();
  void updateDisplay();   // refresh display name + status from enabled_/tile count

  // [camp#121] Band selection (by NAME, since live bands are named, unlike the
  // GggsTile 1-indexed bands). Default picks "depth" if present else the first.
  std::string defaultBand() const;
  void setColormap(map::ColorMap::Type type);
  void setBandName(const std::string& name);

  // GL helpers (duplicated from GggsTileLayer; see camp#134).
  bool ensureGL();
  bool ensureProgram();
  QOpenGLTexture* ensureLut();
  QOpenGLTexture* textureFor(Entry& entry);
  void releaseGL();

  static constexpr int kLatSubdivisions = 16;
  static constexpr int kMaxImageEdge = 4096;

  std::string base_namespace_;   // e.g. "/cube_bathymetry"
  std::string cache_dir_;        // <base cache dir>/<sanitized source ns>
  bool enabled_ = false;         // tile stream subscribed (persisted)

  // GGGS level of this source's tiles, learned from the first message/cached tile.
  // Needed to recover a GridIndex from a cached GeoTIFF on warm-load.
  std::optional<std::uint8_t> level_;

  marine_tiled_raster_store::TileCatalogReconciler reconciler_;
  std::map<gggs::GridIndex, Entry> tiles_;

  QRectF scene_bounds_;
  double data_min_ = 1.0;        // auto-range over the selected band (crossed => none)
  double data_max_ = 0.0;

  std::string band_name_;        // selected band (persisted)
  map::ColorMap colormap_{map::ColorMap::Grayscale};
  bool lut_dirty_ = true;

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
  };
  std::map<gggs::GridIndex, WriteState> write_states_;

  // Every in-flight write worker is tracked here so the dtor joins ALL of them (not
  // just the latest). Workers are self-contained (file serialization + rename only);
  // they never touch this object. `shutting_down_` suppresses coalesced relaunches
  // once teardown starts.
  std::vector<QFutureWatcher<void>*> write_watchers_;
  bool shutting_down_ = false;

  // Offscreen GL (the layer owns it; never touches the GUI context). Duplicated
  // from GggsTileLayer — unify via RasterFieldSource camp#134.
  QOpenGLContext* gl_context_ = nullptr;
  QOffscreenSurface* gl_surface_ = nullptr;
  std::unique_ptr<QOpenGLFramebufferObject> fbo_;
  std::unique_ptr<QOpenGLShaderProgram> program_;
  std::unique_ptr<QOpenGLTexture> lut_texture_;
  bool gl_failed_ = false;

  QImage cached_image_;
  QSize cached_size_;
};

}  // namespace live_coverage
}  // namespace ros
}  // namespace camp

#endif
