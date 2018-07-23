#ifndef PROJECTVIEW_H
#define PROJECTVIEW_H

#include<QGraphicsView>
#include <QGeoCoordinate>

class QStatusBar;
class QLabel;
class AutonomousVehicleProject;
class TrackLine;
class SurveyPattern;
class Waypoint;
class BackgroundRaster;

class ProjectView : public QGraphicsView
{
    Q_OBJECT
public:
    explicit ProjectView(QWidget *parent = 0);
    void setStatusBar(QStatusBar *bar);
    void setAddWaypointMode();
    void setAddTracklineMode();
    void setAddSurveyPatternMode();
    void setPanMode();
    void setProject(AutonomousVehicleProject *project);
signals:
    void currentChanged(QModelIndex &index);

public slots:
    void sendLoiterAt();
    void sendGotoAt();
    void updateBackground(BackgroundRaster * bg);


protected:
    void wheelEvent(QWheelEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void contextMenuEvent(QContextMenuEvent *event) override;

private:
    enum class MouseMode {pan, addWaypoint, addTrackline, addSurveyPattern};
    QStatusBar * statusBar;
    QLabel * positionLabel;
    QLabel * modeLabel;
    MouseMode mouseMode;
    AutonomousVehicleProject *m_project;
    TrackLine * currentTrackLine;
    Waypoint * pendingTrackLineWaypoint;
    SurveyPattern * pendingSurveyPattern;
    QGeoCoordinate m_contextMenuLocation;

};

#endif // PROJECTVIEW_H
