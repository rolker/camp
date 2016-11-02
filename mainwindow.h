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

private:
    Ui::MainWindow *ui;
    AutonomousVehicleProject *project;
};

#endif // MAINWINDOW_H
