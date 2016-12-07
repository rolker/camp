#ifndef AUTONOMOUSVEHICLEPROJECT_H
#define AUTONOMOUSVEHICLEPROJECT_H

#include <QObject>
#include <QGeoCoordinate>

class QStandardItemModel;
class QGraphicsScene;
class QGraphicsItem;
class QStandardItem;
class QLabel;
class QStatusBar;
class BackgroundRaster;
class Waypoint;

class AutonomousVehicleProject : public QObject
{
    Q_OBJECT
public:
    explicit AutonomousVehicleProject(QObject *parent = 0);
    ~AutonomousVehicleProject();

    QStandardItemModel *model() const;
    QGraphicsScene *scene() const;
    void openBackground(QString const &fname);
    BackgroundRaster * getBackgroundRaster() const;
    void addWaypoint(QGeoCoordinate position, BackgroundRaster *parentItem =0);

signals:

public slots:


private:
    QStandardItemModel* m_model;
    QGraphicsScene* m_scene;
    std::map<QString,QStandardItem*> topLevelItems;
};

#endif // AUTONOMOUSVEHICLEPROJECT_H
