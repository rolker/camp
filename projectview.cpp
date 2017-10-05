#include "projectview.h"

#include <QWheelEvent>
#include <QLabel>
#include <QStatusBar>
#include <QStandardItemModel>
#include "autonomousvehicleproject.h"
#include "backgroundraster.h"
#include "waypoint.h"
#include "trackline.h"
#include "surveypattern.h"

ProjectView::ProjectView(QWidget *parent) : QGraphicsView(parent),
    statusBar(0), positionLabel(new QLabel()), modeLabel(new QLabel()), mouseMode(MouseMode::pan), currentTrackLine(nullptr), pendingSurveyPattern(nullptr), pendingTrackLineWaypoint(nullptr)
{

    positionLabel->setText("(,)");
    modeLabel->setText("Mode: pan");

}

void ProjectView::wheelEvent(QWheelEvent *event)
{
    if(event->angleDelta().y()<0)
        scale(.8,.8);
    if(event->angleDelta().y()>0)
        scale(1.25,1.25);
    event->accept();
}

void ProjectView::mousePressEvent(QMouseEvent *event)
{
    BackgroundRaster *bg =  m_project->getBackgroundRaster();
    switch(event->button())
    {
    case Qt::LeftButton:
        switch(mouseMode)
        {
        case MouseMode::pan:
            break;
        case MouseMode::addWaypoint:
            if(bg)
            {
                m_project->addWaypoint(bg->pixelToGeo(mapToScene(event->pos())),bg);
            }
            setPanMode();
            break;
        case MouseMode::addTrackline:
            if(!currentTrackLine)
            {
                if(bg)
                {
                    currentTrackLine = m_project->addTrackLine(bg->pixelToGeo(mapToScene(event->pos())),bg);
                    pendingTrackLineWaypoint = currentTrackLine->addWaypoint(bg->pixelToGeo(mapToScene(event->pos())));
                }
            }
            else
            {
                pendingTrackLineWaypoint = currentTrackLine->addWaypoint(bg->pixelToGeo(mapToScene(event->pos())));

            }
            break;
        case MouseMode::addSurveyPattern:
            if(!pendingSurveyPattern)
            {
                if(bg)
                {
                    pendingSurveyPattern = m_project->addSurveyPattern(bg->pixelToGeo(mapToScene(event->pos())),bg);
                    QModelIndex i = m_project->model()->indexFromItem(pendingSurveyPattern->item());
                    emit  currentChanged(i);
                }
            }
            else
            {
                if(pendingSurveyPattern->hasSpacingLocation())
                {
                    setPanMode();
                }
                else
                {
                    pendingSurveyPattern->setSpacingLocation(bg->pixelToGeo(mapToScene(event->pos())));
                }
            }
            break;
        }
        break;
    case Qt::RightButton:
        if(mouseMode == MouseMode::addTrackline || mouseMode == MouseMode::addWaypoint || mouseMode == MouseMode::addSurveyPattern)
        {
            if(mouseMode == MouseMode::addTrackline && currentTrackLine)
            {
                m_project->deleteItem(pendingTrackLineWaypoint->item());
                currentTrackLine->removeWaypoint(pendingTrackLineWaypoint);
                m_project->scene()->removeItem(pendingTrackLineWaypoint);
                m_project->scene()->update();
                delete pendingTrackLineWaypoint;
                pendingTrackLineWaypoint = nullptr;
                update();
            }
            setPanMode();
        }
        break;
    default:
        break;
    }
    QGraphicsView::mousePressEvent(event);
}

void ProjectView::mouseMoveEvent(QMouseEvent *event)
{
    QString posText = QString::number(event->pos().x())+","+QString::number(event->pos().y());

    QPointF transformedMouse = mapToScene(event->pos());
    BackgroundRaster *bg =  m_project->getBackgroundRaster();
    if(bg)
    {
        QPointF projectedMouse = bg->pixelToProjectedPoint(transformedMouse);
        posText += " Projected mouse: "+QString::number(projectedMouse.x(),'f')+","+QString::number(projectedMouse.y(),'f');
        QGeoCoordinate llMouse = bg->unproject(projectedMouse);
        posText += " WGS84: " + llMouse.toString();
        if(pendingSurveyPattern)
        {
            if(pendingSurveyPattern->hasSpacingLocation())
                pendingSurveyPattern->setSpacingLocation(bg->pixelToGeo(mapToScene(event->pos())));
            else
                pendingSurveyPattern->setEndLocation(bg->pixelToGeo(mapToScene(event->pos())));
        }
        if(pendingTrackLineWaypoint)
        {
            pendingTrackLineWaypoint->setLocation(bg->pixelToGeo(mapToScene(event->pos())));
        }
    }
    positionLabel->setText(posText);
    QGraphicsView::mouseMoveEvent(event);
}

void ProjectView::mouseReleaseEvent(QMouseEvent *event)
{
    if(event->button() == Qt::LeftButton)
    {
    }
    QGraphicsView::mouseReleaseEvent(event);
}

void ProjectView::setAddWaypointMode()
{
    setDragMode(NoDrag);
    mouseMode = MouseMode::addWaypoint;
    modeLabel->setText("Mode: add waypoint");
    setCursor(Qt::CrossCursor);
}

void ProjectView::setAddTracklineMode()
{
    setDragMode(NoDrag);
    mouseMode = MouseMode::addTrackline;
    modeLabel->setText("Mode: add trackline");
    setCursor(Qt::CrossCursor);
}

void ProjectView::setAddSurveyPatternMode()
{
    setDragMode(NoDrag);
    mouseMode = MouseMode::addSurveyPattern;
    modeLabel->setText("Mode: add survey pattern");
    setCursor(Qt::CrossCursor);
}

void ProjectView::setStatusBar(QStatusBar *bar)
{
    statusBar = bar;
    statusBar->addWidget(positionLabel);
    statusBar->addPermanentWidget(modeLabel);
}



void ProjectView::setProject(AutonomousVehicleProject *project)
{
    m_project = project;
    setScene(project->scene());
}

void ProjectView::setPanMode()
{
    setDragMode(ScrollHandDrag);
    mouseMode = MouseMode::pan;
    modeLabel->setText("Mode: pan");
    unsetCursor();
    pendingSurveyPattern = nullptr;
    currentTrackLine = nullptr;
}
