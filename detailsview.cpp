#include "detailsview.h"
#include <QStandardItemModel>
#include "autonomousvehicleproject.h"
#include "backgroundraster.h"
#include "backgrounddetails.h"
#include "waypoint.h"
#include "waypointdetails.h"
#include "trackline.h"
#include "tracklinedetails.h"
#include "surveypattern.h"
#include "surveypatterndetails.h"
#include "platform.h"
#include "platformdetails.h"
#include <QDebug>

DetailsView::DetailsView(QWidget *parent) : QWidget(parent), m_project(nullptr),currentWidget(nullptr)
{
    backgroundDetails = new BackgroundDetails(this);
    backgroundDetails->hide();
    waypointDetails = new WaypointDetails(this);
    waypointDetails->hide();
    trackLineDetails = new TrackLineDetails(this);
    trackLineDetails->hide();
    surveyPatternDetails = new SurveyPatternDetails(this);
    surveyPatternDetails->hide();
    platformDetails = new PlatformDetails(this);
    platformDetails->hide();
}

QSize DetailsView::sizeHint() const
{
    if(currentWidget)
        return currentWidget->sizeHint();
    return QWidget::sizeHint();
}

void DetailsView::setProject(AutonomousVehicleProject *project)
{
    m_project = project;
}

void DetailsView::setCurrentWidget(QWidget *widget)
{
    if(currentWidget != widget)
    {
        if(currentWidget)
            currentWidget->hide();
        currentWidget = widget;
        if(currentWidget)
        {
            currentWidget->show();
            updateGeometry();
        }
    }
}

void DetailsView::onCurrentItemChanged(const QModelIndex &current, const QModelIndex &previous)
{
    QVariant item = m_project->model()->data(current,Qt::UserRole+1);
    MissionItem* mi = reinterpret_cast<MissionItem*>(item.value<quintptr>());
    qDebug() << "metaobject class name: " << mi->metaObject()->className();
    QString itemType = mi->metaObject()->className();

    if (itemType == "BackgroundRaster")
    {
        BackgroundRaster *bg = qobject_cast<BackgroundRaster*>(mi);
        setCurrentWidget(backgroundDetails);
        backgroundDetails->setBackgroundRaster(bg);
    }
    else if (itemType == "Waypoint")
    {
        Waypoint *wp = qobject_cast<Waypoint*>(mi);
        setCurrentWidget(waypointDetails);
        waypointDetails->setWaypoint(wp);
    }
    else if (itemType == "TrackLine")
    {
        TrackLine *tl = qobject_cast<TrackLine*>(mi);
        setCurrentWidget(trackLineDetails);
        trackLineDetails->setTrackLine(tl);
    }
    else if (itemType == "SurveyPattern")
    {
        SurveyPattern *sp = qobject_cast<SurveyPattern*>(mi);
        setCurrentWidget(surveyPatternDetails);
        surveyPatternDetails->setSurveyPattern(sp);
    }
    else if (itemType == "Platform")
    {
        Platform *p = qobject_cast<Platform*>(mi);
        setCurrentWidget(platformDetails);
        platformDetails->setPlatform(p);
    }
    else
        setCurrentWidget(nullptr);
    m_project->setCurrent(current);
}
