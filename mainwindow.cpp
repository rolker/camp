#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <QStandardItemModel>
#include <gdal_priv.h>
#include <cstdint>

#include "autonomousvehicleproject.h"
#include "waypoint.h"
#include "roslink.h"
#include <modeltest.h>
#include "backgroundraster.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    GDALAllRegister();
    project = new AutonomousVehicleProject(this);
    
    new ModelTest(project,this);

    ui->treeView->setModel(project);
    ui->projectView->setStatusBar(statusBar());
    ui->projectView->setProject(project);

    ui->detailsView->setProject(project);
    connect(ui->treeView->selectionModel(),&QItemSelectionModel::currentChanged,ui->detailsView,&DetailsView::onCurrentItemChanged);

    connect(project, &AutonomousVehicleProject::backgroundUpdated, ui->projectView, &ProjectView::updateBackground);

    connect(ui->projectView,&ProjectView::currentChanged,this,&MainWindow::setCurrent);

    ui->rosDetails->setEnabled(false);
    connect(project->rosLink(), &ROSLink::rosConnected,this,&MainWindow::onROSConnected);
    ui->rosDetails->setROSLink(project->rosLink());

    connect(ui->projectView,&ProjectView::scaleChanged,project->rosLink(),&ROSLink::updateMapScale);

    project->rosLink()->connectROS();
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

void MainWindow::on_actionOpen_triggered()
{
    QString fname = QFileDialog::getOpenFileName(this,tr("Open"));
    setCursor(Qt::WaitCursor);
    project->open(fname);
    unsetCursor();
}

void MainWindow::on_actionImport_triggered()
{
    QString fname = QFileDialog::getOpenFileName(this,tr("Import"));

    project->import(fname);
}


void MainWindow::on_actionWaypoint_triggered()
{
    ui->projectView->setAddWaypointMode();
}

void MainWindow::on_actionTrackline_triggered()
{
    ui->projectView->setAddTracklineMode();
}

void MainWindow::on_treeView_customContextMenuRequested(const QPoint &pos)
{
    QModelIndex index = ui->treeView->indexAt(pos);

    QMenu menu(this);

    QAction *exportAction = menu.addAction("Export");
    connect(exportAction, &QAction::triggered, this, &MainWindow::exportHypack);
    
#ifdef AMP_ROS
    QAction *sendToROSAction = menu.addAction("Send to ROS");
    connect(sendToROSAction, &QAction::triggered, this, &MainWindow::sendToROS);
#endif

    QAction *openBackgroundAction = menu.addAction("Open Background");
    connect(openBackgroundAction, &QAction::triggered, this, &MainWindow::on_actionOpenBackground_triggered);

    QAction *addWaypointAction = menu.addAction("Add Waypoint");
    connect(addWaypointAction, &QAction::triggered, this, &MainWindow::on_actionWaypoint_triggered);

    QAction *addTrackLineAction = menu.addAction("Add Track Line");
    connect(addTrackLineAction, &QAction::triggered, this, &MainWindow::on_actionTrackline_triggered);

    QAction *addSurveyPatternAction = menu.addAction("Add Survey Pattern");
    connect(addSurveyPatternAction, &QAction::triggered, this, &MainWindow::on_actionSurveyPattern_triggered);

    QAction *addGroupAction = menu.addAction("Add Group");
    connect(addGroupAction, &QAction::triggered, this, &MainWindow::on_actionGroup_triggered);
    
    QAction *addPlatformAction = menu.addAction("Add Platform");
    connect(addPlatformAction, &QAction::triggered, this, &MainWindow::on_actionPlatform_triggered);

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

void MainWindow::sendToROS() const
{
    project->sendToROS(ui->treeView->selectionModel()->currentIndex());
}


void MainWindow::on_actionSave_triggered()
{
    on_actionSaveAs_triggered();
}

void MainWindow::on_actionSaveAs_triggered()
{
    QString fname = QFileDialog::getSaveFileName(this);
    project->save(fname);
}

void MainWindow::on_actionOpenBackground_triggered()
{
    QString fname = QFileDialog::getOpenFileName(this,tr("Open"));//,"/home/roland/data/BSB_ROOT/13283");

    if(!fname.isEmpty())
    {
        setCursor(Qt::WaitCursor);
        project->openBackground(fname);
        unsetCursor();
    }

}

void MainWindow::on_actionSurveyPattern_triggered()
{
    ui->projectView->setAddSurveyPatternMode();
}

void MainWindow::on_actionSurveyArea_triggered()
{
    ui->projectView->setAddSurveyAreaMode();
}

void MainWindow::on_actionPlatform_triggered()
{
    project->createPlatform();
}

void MainWindow::on_actionOpenGeometry_triggered()
{
    QString fname = QFileDialog::getOpenFileName(this,tr("Open"));

    if(!fname.isEmpty())
        project->openGeometry(fname);
}

void MainWindow::on_actionGroup_triggered()
{
    project->addGroup();
}

void MainWindow::onROSConnected(bool connected)
{
    ui->rosDetails->setEnabled(connected);
}


