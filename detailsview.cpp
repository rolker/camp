#include "detailsview.h"
#include <QStandardItemModel>
#include "autonomousvehicleproject.h"
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
            currentWidget->show();
    }
}

void DetailsView::onCurrentItemChanged(const QModelIndex &current, const QModelIndex &previous)
{
    Waypoint *wp = m_project->model()->data(current,Qt::UserRole+1).value<Waypoint *>();
    if(wp)
    {
        qDebug() << "Waypoint!";
        setCurrentWidget(waypointDetails);
        waypointDetails->setWaypoint(wp);
    }
    else
    {
        TrackLine *tl = m_project->model()->data(current,Qt::UserRole+1).value<TrackLine *>();
        if(tl)
        {
            setCurrentWidget(trackLineDetails);
            trackLineDetails->setTrackLine(tl);
        }
        else
        {
            SurveyPattern *sp = m_project->model()->data(current,Qt::UserRole+1).value<SurveyPattern *>();
            if(sp)
            {
                setCurrentWidget(surveyPatternDetails);
                surveyPatternDetails->setSurveyPattern(sp);
            }
            else
            {
                Platform *p = m_project->model()->data(current,Qt::UserRole+1).value<Platform *>();
                if(p)
                {
                    setCurrentWidget(platformDetails);
                    platformDetails->setPlatform(p);
                }
                else
                {
                    setCurrentWidget(nullptr);
                }
            }
        }
    }
    m_project->setCurrent(current);
}
