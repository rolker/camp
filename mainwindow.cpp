#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <QStandardItemModel>
#include <gdal_priv.h>
#include <cstdint>

#include "autonomousvehicleproject.h"
#include "waypoint.h"

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

    ui->detailsView->setProject(project);
    connect(ui->treeView->selectionModel(),&QItemSelectionModel::currentChanged,ui->detailsView,&DetailsView::onCurrentItemChanged);


    connect(ui->projectView,&ProjectView::currentChanged,this,&MainWindow::setCurrent);

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setCurrent(QModelIndex &index)
{
    ui->treeView->setCurrentIndex(index);
    project->setCurrent(index);
}

void MainWindow::on_action_Open_triggered()
{
    QString fname = QFileDialog::getOpenFileName(this,tr("Open"));

    project->open(fname);
}

void MainWindow::on_action_Waypoint_triggered()
{
    ui->projectView->setAddWaypointMode();
}

void MainWindow::on_action_Trackline_triggered()
{
    ui->projectView->setAddTracklineMode();
}

void MainWindow::on_treeView_customContextMenuRequested(const QPoint &pos)
{
    QModelIndex index = ui->treeView->indexAt(pos);

    QMenu menu(this);

    QAction *exportAction = menu.addAction("Export");
    connect(exportAction, &QAction::triggered, this, &MainWindow::exportHypack);

    QAction *openBackgroundAction = menu.addAction("Open Background");
    connect(openBackgroundAction, &QAction::triggered, this, &MainWindow::on_actionOpen_Background_triggered);

    QAction *addWaypointAction = menu.addAction("Add Waypoint");
    connect(addWaypointAction, &QAction::triggered, this, &MainWindow::on_action_Waypoint_triggered);

    QAction *addTrackLineAction = menu.addAction("Add Track Line");
    connect(addTrackLineAction, &QAction::triggered, this, &MainWindow::on_action_Trackline_triggered);

    QAction *addSurveyPatternAction = menu.addAction("Add Survey Pattern");
    connect(addSurveyPatternAction, &QAction::triggered, this, &MainWindow::on_actionSurvey_Pattern_triggered);

    if(index.isValid())
    {
        QAction *deleteItemAction = menu.addAction("Delete");
        connect(deleteItemAction, &QAction::triggered, [=](){this->project->deleteItems(ui->treeView->selectionModel()->selectedRows());});
    }

    menu.exec(ui->treeView->mapToGlobal(pos));
}

void MainWindow::exportHypack() const
{
    project->exportHypack(ui->treeView->selectionModel()->currentIndex());
}

void MainWindow::on_actionSave_triggered()
{
    on_actionSave_As_triggered();
}

void MainWindow::on_actionSave_As_triggered()
{
    QString fname = QFileDialog::getSaveFileName(this);
    project->save(fname);
}

void MainWindow::on_actionOpen_Background_triggered()
{
    QString fname = QFileDialog::getOpenFileName(this,tr("Open"),"/home/roland/data/BSB_ROOT/13283");

    project->openBackground(fname);

}

void MainWindow::on_actionSurvey_Pattern_triggered()
{
    ui->projectView->setAddSurveyPatternMode();
}
