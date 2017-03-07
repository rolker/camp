#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>


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

private slots:
    void on_action_Open_triggered();

    void on_action_Waypoint_triggered();

    void on_action_Trackline_triggered();

    void on_treeView_customContextMenuRequested(const QPoint &pos);

    void on_actionSave_triggered();

    void on_actionSave_As_triggered();

    void on_actionOpen_Background_triggered();

    void on_actionSurvey_Pattern_triggered();

private:
    Ui::MainWindow *ui;
    AutonomousVehicleProject *project;

    void exportHypack() const;
};

#endif // MAINWINDOW_H
