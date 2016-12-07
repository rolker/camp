#ifndef PROJECTVIEW_H
#define PROJECTVIEW_H

#include<QGraphicsView>

class QStatusBar;
class QLabel;
class AutonomousVehicleProject;

class ProjectView : public QGraphicsView
{
    Q_OBJECT
public:
    explicit ProjectView(QWidget *parent = 0);
    void setStatusBar(QStatusBar *bar);
    void setAddWaypointMode();
    void setProject(AutonomousVehicleProject *project);
signals:

public slots:

protected:
    void wheelEvent(QWheelEvent *event) Q_DECL_OVERRIDE;
    void mousePressEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseMoveEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseReleaseEvent(QMouseEvent *event) Q_DECL_OVERRIDE;

private:
    enum class MouseMode {pan, addWaypoint, addTrackline};
    QStatusBar * statusBar;
    QLabel * positionLabel;
    QLabel * modeLabel;
    MouseMode mouseMode;
    AutonomousVehicleProject *m_project;

};

#endif // PROJECTVIEW_H
