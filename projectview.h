#ifndef PROJECTVIEW_H
#define PROJECTVIEW_H

#include<QGraphicsView>
#include <QGeoCoordinate>

class QStatusBar;
class QLabel;
class AutonomousVehicleProject;
class TrackLine;
class SurveyPattern;
class SurveyArea;
class Waypoint;
class BackgroundRaster;
class MeasuringTool;

class ProjectView : public QGraphicsView
{
    Q_OBJECT
public:
    explicit ProjectView(QWidget *parent = 0);
    void setStatusBar(QStatusBar *bar);
    void setAddWaypointMode();
    void setAddTracklineMode();
    void setAddSurveyPatternMode();
    void setAddSurveyAreaMode();
    void setPanMode();
    void setProject(AutonomousVehicleProject *project);
signals:
    void currentChanged(QModelIndex &index);
    void scaleChanged(qreal scale);

public slots:
#ifdef AMP_ROS
    void sendHover();
    void sendGoto();
    void sendLookAt();
    void sendLookAtASV();
#endif
    void updateBackground(BackgroundRaster * bg);


protected:
    void wheelEvent(QWheelEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void contextMenuEvent(QContextMenuEvent *event) override;

private:
    enum class MouseMode {pan, addWaypoint, addTrackline, addSurveyPattern, addSurveyArea};
    QStatusBar * statusBar;
    QLabel * positionLabel;
    QLabel * modeLabel;
    MouseMode mouseMode;
    AutonomousVehicleProject *m_project;
    TrackLine * currentTrackLine;
    Waypoint * pendingTrackLineWaypoint;
    SurveyPattern * pendingSurveyPattern;
    SurveyArea * pendingSurveyArea;
    Waypoint * pendingSurveyAreaWaypoint;
    MeasuringTool * measuringTool;

    QGeoCoordinate m_contextMenuLocation;

};

#endif // PROJECTVIEW_H
