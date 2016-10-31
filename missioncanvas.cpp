#include "missioncanvas.h"
#include "backgroundraster.h"
#include <QPainter>
#include <QWheelEvent>
#include <QAbstractItemModel>

MissionCanvas::MissionCanvas(QWidget *parent) : QWidget(parent), m_model(0), scale(1.0), isPanning(false)
{

}

void MissionCanvas::setModel(QAbstractItemModel *model)
{
    m_model = model;
}

void MissionCanvas::paintEvent(QPaintEvent*)
{
    QPainter painter(this);
    if(m_model)
    {
        QModelIndex bgindex = m_model->index(0,0,m_model->index(0,0));
        BackgroundRaster *ngr = m_model->data(bgindex,Qt::UserRole+1).value<BackgroundRaster*>();
        if(ngr)
            ngr->paint(&painter,scale,displayCenter,contentsRect());
    }


}

void MissionCanvas::wheelEvent(QWheelEvent *event)
{
    QPointF offCenter = event->posF()-QPointF(contentsRect().size().width(),contentsRect().size().height())/2.0;
    displayCenter += offCenter/scale;
    if(event->angleDelta().y()<0)
        scale *= 0.8;
    if(event->angleDelta().y()>0)
    {
        scale /= 0.8;
    }
    displayCenter -= offCenter/scale;
    update();
}

void MissionCanvas::mousePressEvent(QMouseEvent *event)
{
    if(event->button() == Qt::LeftButton)
    {
        isPanning = true;
        displayCenterStart = displayCenter;
        mouseStart = event->screenPos();
    }
}

void MissionCanvas::mouseMoveEvent(QMouseEvent *event)
{
    if(isPanning)
    {
        QPointF dMouse = event->screenPos()-mouseStart;
        displayCenter = displayCenterStart - (dMouse/scale);
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


//void MissionCanvas::setBackgroundImage(QImage &bgimage)
//{
//    backgroundDisplayCenter.setX(bgimage.width()/2.0);
//    backgroundDisplayCenter.setY(bgimage.height()/2.0);
//}
