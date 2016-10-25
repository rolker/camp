#include "missioncanvas.h"
#include <QPainter>
#include <QWheelEvent>

MissionCanvas::MissionCanvas(QWidget *parent) : QWidget(parent), scale(1.0), isPanning(false)
{

}

void MissionCanvas::paintEvent(QPaintEvent*)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::SmoothPixmapTransform);

    if(!backgroundImages.empty())
    {
        QPixmap selectedBackground;
        int selectedScale;
        auto l = backgroundImages.lower_bound(1/scale);
        if (l == backgroundImages.end())
        {
            auto last = backgroundImages.rbegin();
            selectedBackground = last->second;
            selectedScale = last->first;
        }
        else
        {
            selectedBackground = l->second;
            selectedScale = l->first;
        }
        double effectiveScale = scale*selectedScale;
        painter.scale(effectiveScale,effectiveScale);

        QSize wsize = contentsRect().size();
        QPointF offset = QPointF(wsize.width(),wsize.height())/2.0;
        offset -= backgroundDisplayCenter*scale;
        offset /= effectiveScale;

        painter.drawPixmap(offset,selectedBackground);
    }
}

void MissionCanvas::wheelEvent(QWheelEvent *event)
{
    QPointF offCenter = event->posF()-QPointF(contentsRect().size().width(),contentsRect().size().height())/2.0;
    backgroundDisplayCenter += offCenter/scale;
    if(event->angleDelta().y()<0)
        scale *= 0.8;
    if(event->angleDelta().y()>0)
    {
        scale /= 0.8;
    }
    backgroundDisplayCenter -= offCenter/scale;
    update();
}

void MissionCanvas::mousePressEvent(QMouseEvent *event)
{
    if(event->button() == Qt::LeftButton)
    {
        isPanning = true;
        backgroundDisplayCenterStart = backgroundDisplayCenter;
        mouseStart = event->screenPos();
    }
}

void MissionCanvas::mouseMoveEvent(QMouseEvent *event)
{
    if(isPanning)
    {
        QPointF dMouse = event->screenPos()-mouseStart;
        backgroundDisplayCenter = backgroundDisplayCenterStart - (dMouse/scale);
        update();
    }
}

void MissionCanvas::mouseReleaseEvent(QMouseEvent *event)
{
    if(event->button() == Qt::LeftButton)
    {
        isPanning = false;
    }
}


void MissionCanvas::setBackgroundImage(QImage &bgimage)
{
    backgroundDisplayCenter.setX(bgimage.width()/2.0);
    backgroundDisplayCenter.setY(bgimage.height()/2.0);
    backgroundImages.clear();
    backgroundImages[1] = QPixmap::fromImage(bgimage);
    for(int i = 2; i < 128; i*=2)
    {
        backgroundImages[i] = QPixmap::fromImage(bgimage.scaledToWidth(bgimage.width()/float(i),Qt::SmoothTransformation));
    }
}
