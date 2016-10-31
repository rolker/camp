#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <gdal_priv.h>
#include <cstdint>

#include "autonomousvehicleproject.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    project(new AutonomousVehicleProject(this))
{
    ui->setupUi(this);
    GDALAllRegister();

    ui->treeView->setModel(project->getModel());
    ui->missionCanvas->setModel(project->getModel());
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_action_Open_triggered()
{
    QString fname = QFileDialog::getOpenFileName(this,tr("Open"),"/home/roland/data/BSB_ROOT/");

    project->openBackground(fname);

}
