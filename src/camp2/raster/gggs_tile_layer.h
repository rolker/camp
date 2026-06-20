#ifndef RASTER_GGGS_TILE_LAYER_H
#define RASTER_GGGS_TILE_LAYER_H

#include "../map/layer.h"

#include <QOpenGLFunctions>
#include <QPointer>
#include <memory>
#include <vector>

class QOpenGLShaderProgram;
class QOpenGLWidget;

namespace camp
{
namespace raster
{

class GggsTile;

/// [camp#90 / I4] Map layer that renders a directory of GGGS raster tiles
/// (native-geographic WGS84 GeoTIFFs from marine_sidescan_mosaic #173 / the
/// bathymetry store) by warping each tile into the Web-Mercator scene on the
/// GPU at display time. Slice 1: single-band grayscale (auto-ranged), fixed
/// tessellation; band-select + colormap are Slice 3 (camp#63 GPU facility).
///
/// The layer lives at the scene origin with an identity item transform, so its
/// local coordinates ARE Web-Mercator scene coordinates; each tile vertex is
/// supplied in lon/lat and the vertex shader applies web_mercator::geoToMap
/// (the only nonlinearity is the 1-D latitude warp), then the MVP built from the
/// live QPainter transform maps scene → NDC so tiles register with vector
/// overlays and the existing CPU raster/tile layers.
class GggsTileLayer: public map::Layer, protected QOpenGLFunctions
{
  Q_OBJECT
  Q_INTERFACES(QGraphicsItem)
public:
  GggsTileLayer(map::MapItem* parentItem, const QString& directory);
  ~GggsTileLayer();

  enum { Type = map::GggsTileLayerType };
  int type() const override { return Type; }

  QRectF boundingRect() const override;
  void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;

  /// Directory of `<level>_<row>_<col>.tif` tiles this layer renders.
  const QString& directory() const { return directory_; }

  /// True once at least one valid tile loaded.
  bool valid() const { return !tiles_.empty(); }

private:
  void loadDirectory(const QString& directory);
  bool ensureProgram();
  void releaseGL();

  // Latitude tessellation per tile. The geo→Web-Mercator warp is separable:
  // longitude is linear (no subdivision needed), latitude is the lone
  // nonlinearity. 16 strips is sub-pixel over a tile at these zooms (Slice 2
  // tunes/justifies this against a < 0.5 px error budget).
  static constexpr int kLatSubdivisions = 16;

  QString directory_;
  std::vector<std::unique_ptr<GggsTile>> tiles_;
  QRectF scene_bounds_;        // union of tile extents in Web-Mercator scene units
  double data_min_ = 1.0;      // auto-range over all tiles (crossed => no data)
  double data_max_ = 0.0;

  std::unique_ptr<QOpenGLShaderProgram> program_;
  QPointer<QOpenGLWidget> gl_widget_;   // for making the context current on teardown
  bool gl_ready_ = false;
};

}  // namespace raster
}  // namespace camp

#endif
