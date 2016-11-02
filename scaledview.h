#ifndef SCALEDVIEW_H
#define SCALEDVIEW_H

#include <QSize>
#include <QPointF>

class ScaledView
{
public:
    ScaledView();

    void setScale(double scale);
    double scale() const;
    void setOffset(QPointF const &offset);
    QPointF const &offset() const;
    void setSizeInPixels(QSize const &psize);
    QSize const & sizeInPixels() const;

    /// Origin of transformed coordinate system in screen space
    QPointF scaledOffset() const;
    QPointF transform(QPointF const &screenPoint) const;

    void zoomIn(QPoint const &mousePosition);
    void zoomOut(QPoint const &mousePosition);
    void zoom(QPoint const &mousePosition, double zoomFactor);

    void pan(QPointF const &deltaMouse);
private:
    double m_scale;
    QPointF m_offset;
    QSize m_sizeInPixels;
    double m_zoomFactor;
};

#endif // SCALEDVIEW_H
