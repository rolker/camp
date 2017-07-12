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
    QString itemType = item.typeName();
    qDebug() << "QVariant: " << itemType;

    if (itemType == "BackgroundRaster*")
    {
        BackgroundRaster *bg = item.value<BackgroundRaster*>();
        setCurrentWidget(backgroundDetails);
        backgroundDetails->setBackgroundRaster(bg);
    }
    else if (itemType == "Waypoint*")
    {
        Waypoint *wp = item.value<Waypoint*>();
        setCurrentWidget(waypointDetails);
        waypointDetails->setWaypoint(wp);
    }
    else if (itemType == "TrackLine*")
    {
        TrackLine *tl = item.value<TrackLine *>();
        setCurrentWidget(trackLineDetails);
        trackLineDetails->setTrackLine(tl);
    }
    else if (itemType == "SurveyPattern*")
    {
        SurveyPattern *sp = item.value<SurveyPattern *>();
        setCurrentWidget(surveyPatternDetails);
        surveyPatternDetails->setSurveyPattern(sp);
    }
    else if (itemType == "Platform*")
    {
        Platform *p = m_project->model()->data(current,Qt::UserRole+1).value<Platform *>();
        setCurrentWidget(platformDetails);
        platformDetails->setPlatform(p);
    }
    else
        setCurrentWidget(nullptr);
    m_project->setCurrent(current);
}
