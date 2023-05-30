#include "map_item_mime_data.h"
#include "map_item.h"

namespace map
{

bool MapItemMimeData::hasFormat(const QString &mimeType) const
{
  if(mimeType == MapItem::MimeType && map_item_ != nullptr)
    return true;
  return QMimeData::hasFormat(mimeType);
}

QStringList MapItemMimeData::formats() const
{
  auto ret = QMimeData::formats();
  if(map_item_ != nullptr)
    ret.insert(0, MapItem::MimeType);
  return ret;
}

QVariant MapItemMimeData::retrieveData(const QString& mimeType, QVariant::Type type) const
{
  if(mimeType == MapItem::MimeType)
  {
    QVariant ret;
    ret.setValue(map_item_);
    return ret;
  }
  return QMimeData::retrieveData(mimeType, type);
}

MapItem* MapItemMimeData::mapItem() const
{
  return map_item_;
}

void MapItemMimeData::setMapItem(MapItem* map_item)
{
  map_item_ = map_item;
  if(map_item != nullptr)
    setText(map_item->objectName());
}

bool MapItemMimeData::hasMapItem() const
{
  return map_item_ != nullptr;
}

} // namespace map
