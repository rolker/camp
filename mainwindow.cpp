#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <QStandardItemModel>
#include <gdal_priv.h>
#include <cstdint>

#include "autonomousvehicleproject.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    GDALAllRegister();
    project = new AutonomousVehicleProject(this);

    ui->treeView->setModel(project->model());
    ui->projectView->setStatusBar(statusBar());
    ui->projectView->setProject(project);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_action_Open_triggered()
{
    QString fname = QFileDialog::getOpenFileName(this,tr("Open"),"/home/roland/data/BSB_ROOT/13283");

    project->openBackground(fname);

}

void MainWindow::on_action_Waypoint_triggered()
{
    ui->projectView->setAddWaypointMode();
}
