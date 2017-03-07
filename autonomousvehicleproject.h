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
class TrackLine;
class SurveyPattern;

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
    SurveyPattern * addSurveyPattern(QGeoCoordinate position, BackgroundRaster *parentItem =0);
    TrackLine * addTrackLine(QGeoCoordinate position, BackgroundRaster *parentItem =0);
    QString const &filename() const;
    void save(QString const &fname = QString());
    void open(QString const &fname);
    void loadTrackLine(const QJsonObject &json);

signals:

public slots:

    void exportHypack(QModelIndex const &index);


private:
    QStandardItemModel* m_model;
    QGraphicsScene* m_scene;
    std::map<QString,QStandardItem*> topLevelItems;
    QString m_filename;
};

#endif // AUTONOMOUSVEHICLEPROJECT_H
