#ifndef MAP_MAP_ITEM_MIME_DATA_H
#define MAP_MAP_ITEM_MIME_DATA_H

#include <QMimeData>

namespace map
{

class MapItem;

// QMimeData subclass used to handle pointers to MapItems.
class MapItemMimeData: public QMimeData
{
  Q_OBJECT
public:
  bool hasFormat(const QString &mimeType) const override;
  QStringList formats() const override;
  QVariant retrieveData(const QString& mimeType, QVariant::Type type) const override;

  MapItem* mapItem() const;
  void setMapItem(MapItem* map_item);
  bool hasMapItem() const;

private:
  MapItem* map_item_ = nullptr;
};

} // namespace map

#endif
