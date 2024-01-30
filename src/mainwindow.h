#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QModelIndex>
#include <QGeoCoordinate>

namespace Ui {
class MainWindow;
}

class AISManager;
class RadarManager;
class SoundPlay;
class SpeechAlerts;
class GridManager;
class MarkersManager;

class AutonomousVehicleProject;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void closeEvent(QCloseEvent *event) override;

signals:
    void closing();
    void speedUpdated(double speed);

public slots:
    void open(QString const &fname);
    void openBackground(QString const &fname);
    void setWorkspace(QString const &dir);

    void setCurrent(const QModelIndex &index, const QModelIndex &previous);
    void onROSConnected(bool connected);
    void activePlatformPosition(QGeoCoordinate position);

private slots:
    void on_actionOpen_triggered();
    void on_actionSave_triggered();
    void on_actionSaveAs_triggered();
    void on_actionImport_triggered();

    void on_actionOpenBackground_triggered();

    void on_actionWaypoint_triggered();
    void on_actionWaypointFromContext_triggered();
    void on_actionTrackline_triggered();
    void on_actionTracklineFromContext_triggered();
    void on_actionSurveyPattern_triggered();
    void on_actionSurveyPatternFromContext_triggered();
    void on_actionSurveyArea_triggered();
    void on_actionSurveyAreaFromContext_triggered();
    void on_actionSearchPattern_triggered();
    void on_actionSearchPatternFromContext_triggered();
    void on_actionGroup_triggered();
    void on_actionGroupFromContext_triggered();
    void on_actionBehavior_triggered();
    void on_actionBehaviorFromContext_triggered();
    void on_actionOpenGeometry_triggered();
    void on_actionOrbit_triggered();
    void on_actionOrbitFromContext_triggered();

    void on_treeView_customContextMenuRequested(const QPoint &pos);

    void on_actionRadar_triggered();
    void on_actionRadarColor_triggered();
    void on_actionShowTail_triggered();
    void on_actionAISManager_triggered();
    void on_actionGridManager_triggered();
    void on_actionMarkersManager_triggered();
    void on_actionRadarManager_triggered();
    void on_actionSay_something_triggered();
    void on_actionFollow_triggered();

    void on_speedLineEdit_editingFinished();
    void on_priorityLineEdit_editingFinished();
    void on_taskDataLineEdit_editingFinished();

private:
    Ui::MainWindow *m_ui;
    AutonomousVehicleProject *project;
    QString m_workspace_path;
    AISManager* m_ais_manager;
    RadarManager* m_radar_manager = nullptr;
    GridManager* m_grid_manager = nullptr;
    MarkersManager* m_markers_manager = nullptr;
    SoundPlay* m_sound_play;
    SpeechAlerts* m_speech_alerts;

    void exportHypack() const;
    void exportMissionPlan() const;
    void sendToROS() const;
    void appendMission() const;
    void prependMission() const;
    void updateMission() const;
};

#endif // MAINWINDOW_H
