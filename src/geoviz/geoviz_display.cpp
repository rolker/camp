#include "geoviz_display.h"
#include <QPainter>

GeovizDisplay::GeovizDisplay(QWidget* parent, QGraphicsItem *parentItem):QWidget(parent), GeoGraphicsItem(parentItem)
{

}


void GeovizDisplay::updateRobotNamespace(QString robotNamespace)
{
  ros::NodeHandle nh;
  m_display_subscriber = nh.subscribe("/"+robotNamespace.toStdString()+"/project11/display", 10, &GeovizDisplay::geoVizDisplayCallback, this);
}

QRectF GeovizDisplay::boundingRect() const
{
  return shape().boundingRect();
}

void GeovizDisplay::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
  painter->save();

  QPen p;
  p.setCosmetic(true);

  for(auto display_item: m_display_items)
  {
      for (auto point_group: display_item.second->point_groups)
      {
          p.setColor(point_group.color);
          p.setWidth(point_group.size);
          painter->setPen(p);
          for(auto point: point_group.points)
              painter->drawPoint(point.pos);
      }
      
      for(auto line: display_item.second->lines)
      {
          p.setColor(line.color);
          p.setWidth(line.size);
          painter->setPen(p);
          painter->drawPath(shape(line.points));
      }
      
      for(auto polygon: display_item.second->polygons)
      {
          p.setColor(polygon.edge_color);
          p.setWidth(polygon.edge_size);
          painter->setPen(p);
          painter->setBrush(QBrush(polygon.fill_color));
          painter->drawPath(polygon.path);
      }
      painter->setBrush(Qt::NoBrush);
  }

  painter->restore();
}

QPainterPath GeovizDisplay::shape() const
{
  QPainterPath ret;
    
  for(auto display_item: m_display_items)
  {
      for(auto plist: display_item.second->point_groups)
      {
          for(auto p: plist.points)
              ret.addEllipse(p.pos,plist.size,plist.size);
      }
      for(auto line: display_item.second->lines)
      {
          ret.addPath(shape(line.points));
      }
      for(auto polygon: display_item.second->polygons)
      {
          ret.addPath(polygon.path);
      }
  }

  return ret;
}

QPainterPath GeovizDisplay::shape(std::vector<LocationPosition> const &lps) const
{
  QPainterPath ret;
  if(lps.size() > 1)
  {
    auto p = lps.begin();
    ret.moveTo(p->pos);
    p++;
    while(p != lps.end())
    {
      ret.lineTo(p->pos);
      p++;
    }
  }
  return ret;
}

void GeovizDisplay::geoVizDisplayCallback(const geographic_visualization_msgs::GeoVizItem::ConstPtr& message)
{
  BackgroundRaster* bgr = findParentBackgroundRaster();
  geoviz::Item *item = new geoviz::Item();
  item->id = message->id;
  item->label = message->label;
  item->label_position.location.setLatitude(message->label_position.latitude);
  item->label_position.location.setLongitude(message->label_position.longitude);
  item->label_position.pos = geoToPixel(item->label_position.location, bgr);
  for(auto pg: message->point_groups)
  {
    geoviz::PointList pl;
    pl.color.setRedF(pg.color.r);
    pl.color.setGreenF(pg.color.g);
    pl.color.setBlueF(pg.color.b);
    pl.color.setAlphaF(pg.color.a);
    pl.size = pg.size;
    
    for(auto p: pg.points)
    {
      LocationPosition lp;
      lp.location.setLatitude(p.latitude);
      lp.location.setLongitude(p.longitude);
      lp.pos = geoToPixel(lp.location,bgr);
      pl.points.push_back(lp);
    }
    item->point_groups.push_back(pl);
  }
  for(auto l: message->lines)
  {
    geoviz::PointList pl;
    pl.color.setRedF(l.color.r);
    pl.color.setGreenF(l.color.g);
    pl.color.setBlueF(l.color.b);
    pl.color.setAlphaF(l.color.a);
    pl.size = l.size;
    
    for(auto p: l.points)
    {
        LocationPosition lp;
        lp.location.setLatitude(p.latitude);
        lp.location.setLongitude(p.longitude);
        lp.pos = geoToPixel(lp.location, bgr);
        pl.points.push_back(lp);
    }
    item->lines.push_back(pl);
  }
  for(auto p: message->polygons)
  {
    geoviz::Polygon polygon;
    QPolygonF qPoly;
    for(auto op: p.outer.points) // outer points
    {
      LocationPosition lp;
      lp.location.setLatitude(op.latitude);
      lp.location.setLongitude(op.longitude);
      lp.pos = geoToPixel(lp.location, bgr);
      qPoly << lp.pos;
      polygon.outer.push_back(lp);
    }
    polygon.path.addPolygon(qPoly);
    QPainterPath innerPath;
    for(auto ir: p.inner) //inner rings
    {
      polygon.inner.push_back(std::vector<LocationPosition>());
      QPolygonF innerPoly;
      for(auto ip: ir.points) // inner ring points
      {
        LocationPosition lp;
        lp.location.setLatitude(ip.latitude);
        lp.location.setLongitude(ip.longitude);
        lp.pos = geoToPixel(lp.location, bgr);
        innerPoly << lp.pos;
        polygon.inner.back().push_back(lp);
      }
      innerPath.addPolygon(innerPoly);
    }
    polygon.path = polygon.path.subtracted(innerPath);
    polygon.fill_color.setRedF(p.fill_color.r);
    polygon.fill_color.setGreenF(p.fill_color.g);
    polygon.fill_color.setBlueF(p.fill_color.b);
    polygon.fill_color.setAlphaF(p.fill_color.a);
    polygon.edge_color.setRedF(p.edge_color.r);
    polygon.edge_color.setGreenF(p.edge_color.g);
    polygon.edge_color.setBlueF(p.edge_color.b);
    polygon.edge_color.setAlphaF(p.edge_color.a);
    polygon.edge_size = p.edge_size;
    item->polygons.push_back(polygon);
  }
    
  QMetaObject::invokeMethod(this,"updateDisplayItem", Qt::QueuedConnection, Q_ARG(geoviz::Item*, item));
}

void GeovizDisplay::updateDisplayItem(geoviz::Item *item)
{
  prepareGeometryChange();
  m_display_items[item->id] = std::shared_ptr<geoviz::Item>(item);
}

void GeovizDisplay::updateProjectedPoints()
{
  prepareGeometryChange();
  BackgroundRaster* bgr = findParentBackgroundRaster();

  for(std::pair<std::string, std::shared_ptr<geoviz::Item> > display_item: m_display_items)
  {
    display_item.second->label_position.pos = geoToPixel(display_item.second->label_position.location,bgr);
    for(auto pl: display_item.second->point_groups)
    {
      for(auto& p: pl.points)
        p.pos = geoToPixel(p.location, bgr);
    }
    for(auto& l: display_item.second->lines)
    {
      for(auto& p: l.points)
      {
        p.pos = geoToPixel(p.location, bgr);
      }
    }
    for(auto& poly: display_item.second->polygons)
    {
      for(auto& op: poly.outer)
        op.pos = geoToPixel(op.location, bgr);
      for(auto& ir: poly.inner)
        for(auto& ip: ir)
          ip.pos = geoToPixel(ip.location, bgr);
    }
  } 
}