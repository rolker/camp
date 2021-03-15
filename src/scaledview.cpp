#include "scaledview.h"

ScaledView::ScaledView():m_scale(1.0),m_zoomFactor(0.8)
{
}

void ScaledView::setScale(double scale)
{
    m_scale = scale;
}

double ScaledView::scale() const
{
    return m_scale;
}

void ScaledView::setOffset(const QPointF &offset)
{
    m_offset = offset;
}

QPointF const &ScaledView::offset() const
{
    return m_offset;
}

void ScaledView::setSizeInPixels(const QSize &psize)
{
    m_sizeInPixels = psize;
}

QSize const &ScaledView::sizeInPixels() const
{
    return m_sizeInPixels;
}

void ScaledView::zoomIn(const QPoint &mousePosition)
{
    zoom(mousePosition,1.0/m_zoomFactor);
}

void ScaledView::zoomOut(const QPoint &mousePosition)
{
    zoom(mousePosition,m_zoomFactor);
}

void ScaledView::zoom(const QPoint &mousePosition, double factor)
{
    QPointF offCenter = mousePosition-QPointF(m_sizeInPixels.width(),m_sizeInPixels.height())/2.0;
    m_offset += offCenter/m_scale;
    m_scale *= factor;
    m_offset -= offCenter/m_scale;
}

void ScaledView::pan(const QPointF &deltaMouse)
{
    m_offset -= deltaMouse/m_scale;
}

QPointF ScaledView::scaledOffset() const
{
    return (QPointF(m_sizeInPixels.width(),m_sizeInPixels.height())/2.0) - m_offset*m_scale;
}

QPointF ScaledView::transform(const QPointF &screenPoint) const
{
    return (screenPoint-scaledOffset())/m_scale;
}

