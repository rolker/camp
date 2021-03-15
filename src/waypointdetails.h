#ifndef WAYPOINTDETAILS_H
#define WAYPOINTDETAILS_H

#include <QWidget>

namespace Ui {
class WaypointDetails;
}

class Waypoint;

class WaypointDetails : public QWidget
{
    Q_OBJECT

public:
    explicit WaypointDetails(QWidget *parent = 0);
    ~WaypointDetails();

    void setWaypoint(Waypoint *waypoint);

public slots:
    void onLocationChanged();

private slots:
    void on_latitudeLineEdit_editingFinished();

    void on_longitudeLineEdit_editingFinished();

private:
    Ui::WaypointDetails *ui;
    Waypoint * m_waypoint;

    QMetaObject::Connection moveConnection;

    void updateWaypoint();
};

#endif // WAYPOINTDETAILS_H
