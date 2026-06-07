#include "geographicsitem.h"
#include "autonomousvehicleproject.h"
#include "missionitem.h"
#include "map_view/web_mercator.h"
#include <QGraphicsSimpleTextItem>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QFont>
#include <QBrush>
#include <QPen>
#include <QDebug>
#include <cmath>

GeoGraphicsItem::GeoGraphicsItem(QGraphicsItem *parentItem): QGraphicsItem(parentItem), m_showLabelFlag(false)
{
    m_label = new QGraphicsSimpleTextItem(this);
    m_label->setFlag(GraphicsItemFlag::ItemIgnoresTransformations);
    auto font = m_label->font();
    font.setPointSize(20);
    font.setBold(true);
    m_label->setFont(font);
    m_label->setBrush(QBrush(QColor("black")));
    QPen p(QColor("white"));
    p.setWidth(0);
    m_label->setPen(p);
    //m_label->setFlag(QGraphicsItem::ItemIsMovable); this caused other elements to move while trying to move the label!
}

QPointF GeoGraphicsItem::geoToPixel(const QGeoCoordinate &point) const
{
    // [#59 PR3a] The scene is Web Mercator (ADR-0002). Position comes from
    // web_mercator::geoToMap, independent of any background raster. The
    // parent-offset subtraction is coordinate-agnostic, so nested items still
    // resolve to parent-local coords.
    QPointF ret = web_mercator::geoToMap(point);
    QGraphicsItem *pi = parentItem();
    if(pi)
        return ret - pi->scenePos();
    return ret;
}

qreal GeoGraphicsItem::metresPerPixel(const QGeoCoordinate &at) const
{
    // Display-pixels per scene-unit from the active view's transform (m11 is the
    // positive X scale; a Y-flip leaves it positive). Scene units are
    // Web-Mercator metres-at-equator, so metersPerUnit() applies the cos(latitude)
    // correction to recover real metres. Result = real metres per display pixel.
    qreal pixels_per_unit = 1.0;
    if(scene() && !scene()->views().isEmpty())
        pixels_per_unit = std::abs(scene()->views().first()->transform().m11());
    if(pixels_per_unit <= 0.0)
        pixels_per_unit = 1.0;
    const double metres_per_unit = web_mercator::metersPerUnit(web_mercator::geoToMap(at));
    return metres_per_unit / pixels_per_unit;
}

void GeoGraphicsItem::prepareGeometryChange()
{
    QGraphicsItem::prepareGeometryChange();
}

void GeoGraphicsItem::setLabel(const QString &label)
{
    m_labelText = label;
    if(m_showLabelFlag)
        m_label->setText(m_labelText);
}

void GeoGraphicsItem::setLabelPosition(QPointF pos)
{
    m_label->setPos(pos);
}

bool GeoGraphicsItem::showLabelFlag() const
{
    return m_showLabelFlag;
}

void GeoGraphicsItem::setShowLabelFlag(bool show)
{
    m_showLabelFlag = show;
    if(show)
        m_label->setText(m_labelText);
    else
        m_label->setText("");
}

