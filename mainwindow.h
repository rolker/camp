#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QModelIndex>

namespace Ui {
class MainWindow;
}

class AutonomousVehicleProject;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

public slots:

    void setCurrent(QModelIndex &index);

private slots:
    void on_actionOpen_triggered();
    void on_actionWaypoint_triggered();
    void on_actionTrackline_triggered();
    void on_treeView_customContextMenuRequested(const QPoint &pos);
    void on_actionSave_triggered();
    void on_actionSaveAs_triggered();
    void on_actionOpenBackground_triggered();
    void on_actionSurveyPattern_triggered();
    void on_actionPlatform_triggered();
    void on_actionOpenGeometry_triggered();
    void on_actionROS_Node_triggered();
    void on_actionGroup_triggered();


private:
    Ui::MainWindow *ui;
    AutonomousVehicleProject *project;

    void exportHypack() const;
    void sendToROS() const;
};

#endif // MAINWINDOW_H
