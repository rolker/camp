#include "waypointdetails.h"
#include "ui_waypointdetails.h"
#include <QDebug>
#include "waypoint.h"

WaypointDetails::WaypointDetails(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::WaypointDetails),m_waypoint(nullptr)
{
    ui->setupUi(this);
}

WaypointDetails::~WaypointDetails()
{
    delete ui;
}

void WaypointDetails::setWaypoint(Waypoint *waypoint)
{
    m_waypoint = waypoint;
    qDebug() << "setting waypoint " << static_cast<const void *>(m_waypoint) << m_waypoint->location();
    disconnect(moveConnection);
    moveConnection = connect(waypoint,&Waypoint::waypointMoved,this,&WaypointDetails::onLocationChanged);
    onLocationChanged();
}

void WaypointDetails::onLocationChanged()
{
    //qDebug() << m_waypoint->location();
    ui->latitudeLineEdit->setText(QString::number(m_waypoint->location().latitude(),'f',8));
    ui->longitudeLineEdit->setText(QString::number(m_waypoint->location().longitude(),'f',8));
}

void WaypointDetails::updateWaypoint()
{
    if(m_waypoint)
    {
        QGeoCoordinate location;
        location.setLatitude(ui->latitudeLineEdit->text().toDouble());
        location.setLongitude(ui->longitudeLineEdit->text().toDouble());
        m_waypoint->setLocation(location);
    }
}

void WaypointDetails::on_latitudeLineEdit_editingFinished()
{
    updateWaypoint();
}

void WaypointDetails::on_longitudeLineEdit_editingFinished()
{
    updateWaypoint();
}
