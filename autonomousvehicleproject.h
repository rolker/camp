#ifndef AUTONOMOUSVEHICLEPROJECT_H
#define AUTONOMOUSVEHICLEPROJECT_H

#include <QObject>
#include <QGeoCoordinate>
#include <QModelIndex>

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

    Waypoint * createWaypoint(BackgroundRaster *parentItem=0);
    void addWaypoint(QGeoCoordinate position, BackgroundRaster *parentItem =0);

    SurveyPattern * createSurveyPattern(BackgroundRaster *parentItem=0);
    SurveyPattern * addSurveyPattern(QGeoCoordinate position, BackgroundRaster *parentItem =0);

    TrackLine * createTrackLine(BackgroundRaster *parentItem =0);
    TrackLine * addTrackLine(QGeoCoordinate position, BackgroundRaster *parentItem =0);

    QString const &filename() const;
    void save(QString const &fname = QString());
    void open(QString const &fname);

    void setCurrent(const QModelIndex &index);

signals:

public slots:

    void exportHypack(QModelIndex const &index);
    void deleteItems(QModelIndexList const &indices);


private:
    QStandardItemModel* m_model;
    QGraphicsScene* m_scene;
    QString m_filename;
    BackgroundRaster* m_currentBackground;

    void setCurrentBackground(BackgroundRaster *bgr);
};

#endif // AUTONOMOUSVEHICLEPROJECT_H
