#include "missioncanvas.h"
#include "backgroundraster.h"
#include <QPainter>
#include <QWheelEvent>
#include <QAbstractItemModel>
#include <QLabel>
#include <QStatusBar>

MissionCanvas::MissionCanvas(QWidget *parent) : QWidget(parent), m_model(0), statusBar(0), positionLabel(new QLabel()), modeLabel(new QLabel()), mouseMode(MouseMode::pan), isPanning(false)
{
    positionLabel->setText("(,)");
    modeLabel->setText("Mode: pan");
}

void MissionCanvas::setModel(QAbstractItemModel *model)
{
    m_model = model;
}

void MissionCanvas::setStatusBar(QStatusBar *bar)
{
    statusBar = bar;
    statusBar->addWidget(positionLabel);
    statusBar->addPermanentWidget(modeLabel);
}

void MissionCanvas::resizeEvent(QResizeEvent *event)
{
    view.setSizeInPixels(event->size());
    QWidget::resizeEvent(event);
}

void MissionCanvas::paintEvent(QPaintEvent*)
{
    QPainter painter(this);
    BackgroundRaster *ngr = getBackgroundRaster();
    if(ngr)
        ngr->paint(&painter,view);
}

BackgroundRaster *MissionCanvas::getBackgroundRaster() const
{
    if(m_model)
    {
        QModelIndex bgindex = m_model->index(0,0,m_model->index(0,0));
        return m_model->data(bgindex,Qt::UserRole+1).value<BackgroundRaster*>();
    }
    return 0;
}

void MissionCanvas::wheelEvent(QWheelEvent *event)
{
    if(event->angleDelta().y()<0)
        view.zoomOut(event->pos());
    if(event->angleDelta().y()>0)
        view.zoomIn(event->pos());
    update();
}

void MissionCanvas::mousePressEvent(QMouseEvent *event)
{
    switch(event->button())
    {
    case Qt::LeftButton:
        switch(mouseMode)
        {
        case MouseMode::pan:
            isPanning = true;
            lastMousePosition = event->screenPos();
            break;
        case MouseMode::addWaypoint:
            mouseMode = MouseMode::pan;
            modeLabel->setText("Mode: pan");
            unsetCursor();
            break;
        case MouseMode::addTrackline:
            break;
        }
        break;
    case Qt::RightButton:
        if(mouseMode == MouseMode::addTrackline || mouseMode == MouseMode::addWaypoint)
            mouseMode = MouseMode::pan;
        break;
    default:
        break;
    }
}

void MissionCanvas::mouseMoveEvent(QMouseEvent *event)
{
    QString posText = QString::number(event->pos().x())+","+QString::number(event->pos().y());
    QPointF transformedMouse = view.transform(event->pos());
    posText += " Transformed mouse: "+QString::number(transformedMouse.x())+","+QString::number(transformedMouse.y());
    positionLabel->setText(posText);
    if(isPanning)
    {
        view.pan(event->screenPos()-lastMousePosition);
        lastMousePosition = event->screenPos();
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

void MissionCanvas::addWaypoint()
{
    mouseMode = MouseMode::addWaypoint;
    modeLabel->setText("Mode: add waypoint");
    setCursor(Qt::CrossCursor);
}
