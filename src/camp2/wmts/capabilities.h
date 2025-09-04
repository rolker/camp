#ifndef CAMP_WMTS_CAPABILITIES_H
#define CAMP_WMTS_CAPABILITIES_H

#include <QObject>
#include "layer.h"
#include "tile_matrix_set.h"
#include "map_tiles/tile_layout.h"

namespace camp
{

class CachedFileClient;

namespace wmts
{

class Capabilities: public QObject
{
  Q_OBJECT
public:
  Capabilities(QString label, QObject* parent=nullptr);
  
  void setUrl(QString url);
  map_tiles::TileLayout getLayout(QString layer_id = {}, QString tile_matrix_set = {}) const;
signals:
  void ready();

private slots:
  void dataLoaded(QByteArray &data, CachedFileClient* client);

private:
  QString label_;

  std::vector<Layer> layers_;
  std::vector<TileMatrixSet> tile_matrix_sets_;
};

} // namespace wmts

} // namespace camp

#endif
