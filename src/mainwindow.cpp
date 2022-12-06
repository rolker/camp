#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <QStandardItemModel>
#include <gdal_priv.h>
#include <cstdint>
#include <QOpenGLWidget>

#include "autonomousvehicleproject.h"
#include "waypoint.h"

#include "roslink.h"

#include <modeltest.h>
#include "backgroundraster.h"
#include "trackline.h"
#include "surveypattern.h"
#include "surveyarea.h"
#include "searchpattern.h"

#include "ais/ais_manager.h"
#include "radar/radar_manager.h"
#include "sound_play/sound_play_widget.h"
#include "sound_play/speech_alerts.h"
#include "platform_manager/platform.h"

#include <QDebug>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    m_ui(new Ui::MainWindow)
{
    m_ui->setupUi(this);
    GDALAllRegister();
    project = new AutonomousVehicleProject(this);
    
    new ModelTest(project,this);

    m_ui->treeView->setModel(project);
    m_ui->projectView->setStatusBar(statusBar());
    m_ui->projectView->setProject(project);

    m_ui->detailsView->setProject(project);
    connect(m_ui->treeView->selectionModel(),&QItemSelectionModel::currentChanged,m_ui->detailsView,&DetailsView::onCurrentItemChanged);

    connect(project, &AutonomousVehicleProject::backgroundUpdated, m_ui->projectView, &ProjectView::updateBackground);
    connect(project, &AutonomousVehicleProject::aboutToUpdateBackground, m_ui->projectView, &ProjectView::beforeUpdateBackground);

    connect(m_ui->projectView,&ProjectView::currentChanged,this,&MainWindow::setCurrent);

    connect(project, &AutonomousVehicleProject::backgroundUpdated, m_ui->platformManager, &PlatformManager::updateBackground);

    connect(m_ui->platformManager, &PlatformManager::currentPlatform, project, &AutonomousVehicleProject::updateActivePlatform);
    connect(m_ui->platformManager, &PlatformManager::currentPlatformPosition, this, &MainWindow::activePlatfromPosition);

    //m_ui->rosDetails->setEnabled(false);
    //connect(project->rosLink(), &ROSLink::robotNamespaceUpdated, m_ui->helmManager, &HelmManager::updateRobotNamespace);
    //connect(project->rosLink(), &ROSLink::rosConnected,this,&MainWindow::onROSConnected);
    //m_ui->rosDetails->setROSLink(project->rosLink());

    //project->rosLink()->connectROS();
    m_ui->rosLink->connectROS();

    //connect(project->rosLink(), &ROSLink::centerMap, m_ui->projectView, &ProjectView::centerMap);

    //connect(m_ui->detailsView, &DetailsView::clearTasks, project->rosLink(), &ROSLink::clearTasks);
    
    connect(m_ui->projectView,&ProjectView::scaleChanged,project,&AutonomousVehicleProject::updateMapScale);

    m_ais_manager = new AISManager();
    connect(project, &AutonomousVehicleProject::backgroundUpdated, m_ais_manager, &AISManager::updateBackground);
    connect(m_ui->projectView, &ProjectView::viewportChanged, m_ais_manager, &AISManager::updateViewport);

    //m_radar_manager = new RadarManager();
    //m_radar_manager->setTFBuffer(m_ui->rosLink->tfBuffer());
    //connect(project, &AutonomousVehicleProject::backgroundUpdated, m_radar_manager, &RadarManager::updateBackground);

    m_sound_play = new SoundPlay();

    m_speech_alerts = new SpeechAlerts(this);
    connect(m_speech_alerts, &SpeechAlerts::tell, m_sound_play, &SoundPlay::say);
    //connect(m_ui->helmManager, &HelmManager::pilotingModeUpdated, m_speech_alerts, &SpeechAlerts::updatePilotingMode);

    m_ui->platformManager->loadFromParameters();
}

MainWindow::~MainWindow()
{
    delete m_ui;
    delete m_ais_manager;
    delete m_radar_manager;
}

void MainWindow::setWorkspace(const QString& path)
{
    m_workspace_path = path;
}

void MainWindow::open(const QString& fname)
{
    //setCursor(Qt::WaitCursor);
    project->open(fname);
    //unsetCursor();
}

void MainWindow::openBackground(const QString& fname)
{
    //setCursor(Qt::WaitCursor);
    project->openBackground(fname);
    //unsetCursor();
}

void MainWindow::setCurrent(QModelIndex &index)
{
    m_ui->treeView->setCurrentIndex(index);
    project->setCurrent(index);
    MissionItem* i = project->itemFromIndex(index);
    m_ui->speedLineEdit->setText(QString::number(i->speed()));
}

void MainWindow::on_speedLineEdit_editingFinished()
{
    auto item = project->currentSelected();
    if(item) 
        item->setSpeed(m_ui->speedLineEdit->text().toDouble());
}

void MainWindow::on_actionOpen_triggered()
{
    QString fname = QFileDialog::getOpenFileName(this,tr("Open"),m_workspace_path);
    open(fname);
}

void MainWindow::on_actionImport_triggered()
{
    QString fname = QFileDialog::getOpenFileName(this,tr("Import"),m_workspace_path);

    project->import(fname);
}

void MainWindow::on_actionWaypoint_triggered()
{
    project->setContextMode(false);
    m_ui->projectView->setAddWaypointMode();
}

void MainWindow::on_actionWaypointFromContext_triggered()
{
    project->setContextMode(true);
    m_ui->projectView->setAddWaypointMode();
}

void MainWindow::on_actionTrackline_triggered()
{
    project->setContextMode(false);
    m_ui->projectView->setAddTracklineMode();
}

void MainWindow::on_actionTracklineFromContext_triggered()
{
    project->setContextMode(true);
    m_ui->projectView->setAddTracklineMode();
}


void MainWindow::on_treeView_customContextMenuRequested(const QPoint &pos)
{
    QModelIndex index = m_ui->treeView->indexAt(pos);
    MissionItem  *mi = nullptr;
    if(index.isValid())
        mi = project->itemFromIndex(index);

    QMenu menu(this);

    if(mi && mi->canBeSentToRobot())
    {
        QAction *sendToROSAction = menu.addAction("Send to ROS (Use Execute button)");
        sendToROSAction->setEnabled(false);
        //connect(sendToROSAction, &QAction::triggered, this, &MainWindow::sendToROS);
        
        QMenu *missionMenu = menu.addMenu("Mission");
        
        QAction *appendMissionAction = missionMenu->addAction("append");
        connect(appendMissionAction, &QAction::triggered, this, &MainWindow::appendMission);

        QAction *prependMissionAction = missionMenu->addAction("prepend");
        connect(prependMissionAction, &QAction::triggered, this, &MainWindow::prependMission);

        QAction *updateMissionAction = missionMenu->addAction("update");
        connect(updateMissionAction, &QAction::triggered, this, &MainWindow::updateMission);
        
        QMenu *exportMenu = menu.addMenu("Export");

        QAction *exportHypackAction = exportMenu->addAction("Export Hypack");
        connect(exportHypackAction, &QAction::triggered, this, &MainWindow::exportHypack);

        QAction *exportMPAction = exportMenu->addAction("Export Mission Plan");
        connect(exportMPAction, &QAction::triggered, this, &MainWindow::exportMissionPlan);
    }

    
    QAction *openBackgroundAction = menu.addAction("Open Background");
    connect(openBackgroundAction, &QAction::triggered, this, &MainWindow::on_actionOpenBackground_triggered);
    
    QMenu *addMenu = menu.addMenu("Add");

    if(!index.isValid())
    {
        QAction *addWaypointAction = addMenu->addAction("Add Waypoint");
        connect(addWaypointAction, &QAction::triggered, this, &MainWindow::on_actionWaypoint_triggered);

        QAction *addTrackLineAction = addMenu->addAction("Add Track Line");
        connect(addTrackLineAction, &QAction::triggered, this, &MainWindow::on_actionTrackline_triggered);

        QAction *addSurveyPatternAction = addMenu->addAction("Add Survey Pattern");
        connect(addSurveyPatternAction, &QAction::triggered, this, &MainWindow::on_actionSurveyPattern_triggered);

        QAction *addSearchPatternAction = addMenu->addAction("Add Search Pattern");
        connect(addSearchPatternAction, &QAction::triggered, this, &MainWindow::on_actionSearchPattern_triggered);

        QAction *addGroupAction = addMenu->addAction("Add Group");
        connect(addGroupAction, &QAction::triggered, this, &MainWindow::on_actionOrbit_triggered);

        QAction *addOrbitAction = addMenu->addAction("Add Orbit");
        connect(addOrbitAction, &QAction::triggered, this, &MainWindow::on_actionOrbit_triggered);
    }
    else
    {
        if(mi && mi->canAcceptChildType("Waypoint"))
        {
            QAction *addWaypointAction = addMenu->addAction("Add Waypoint");
            connect(addWaypointAction, &QAction::triggered, this, &MainWindow::on_actionWaypointFromContext_triggered);
        }

        if(mi && mi->canAcceptChildType("TrackLine"))
        {
            QAction *addTrackLineAction = addMenu->addAction("Add Track Line");
            connect(addTrackLineAction, &QAction::triggered, this, &MainWindow::on_actionTracklineFromContext_triggered);
        }

        if(mi && mi->canAcceptChildType("SurveyPattern"))
        {
            QAction *addSurveyPatternAction = addMenu->addAction("Add Survey Pattern");
            connect(addSurveyPatternAction, &QAction::triggered, this, &MainWindow::on_actionSurveyPatternFromContext_triggered);
        }

        if(mi && mi->canAcceptChildType("SurveyArea"))
        {
            QAction *addSurveyAreaAction = addMenu->addAction("Add Survey Area");
            connect(addSurveyAreaAction, &QAction::triggered, this, &MainWindow::on_actionSurveyAreaFromContext_triggered);
        }

        if(mi && mi->canAcceptChildType("SearchPattern"))
        {
            QAction *addSearchPatternAction = addMenu->addAction("Add Search Pattern");
            connect(addSearchPatternAction, &QAction::triggered, this, &MainWindow::on_actionSearchPatternFromContext_triggered);
        }

        if(mi && mi->canAcceptChildType("Group"))
        {
            QAction *addGroupAction = addMenu->addAction("Add Group");
            connect(addGroupAction, &QAction::triggered, this, &MainWindow::on_actionGroupFromContext_triggered);
        }

        if(mi && mi->canAcceptChildType("Orbit"))
        {
            QAction *addOrbitAction = addMenu->addAction("Add Orbit");
            connect(addOrbitAction, &QAction::triggered, this, &MainWindow::on_actionOrbitFromContext_triggered);
        }
        
        QAction *deleteItemAction = menu.addAction("Delete");
        connect(deleteItemAction, &QAction::triggered, [=](){this->project->deleteItems(m_ui->treeView->selectionModel()->selectedRows());});
        
        
        TrackLine *tl = qobject_cast<TrackLine*>(mi);
        if(tl)
        {
            QAction *reverseDirectionAction = menu.addAction("Reverse Direction");
            connect(reverseDirectionAction, &QAction::triggered, tl, &TrackLine::reverseDirection);
            if(project->getBackgroundRaster() && project->getDepthRaster())
            {
                QAction *planPathAction = menu.addAction("Plan path");
                connect(planPathAction, &QAction::triggered, tl, &TrackLine::planPath);
            }

        }

        SurveyPattern *sp = qobject_cast<SurveyPattern*>(mi);
        if(sp)
        {
            QAction *reverseDirectionAction = menu.addAction("Reverse Direction");
            connect(reverseDirectionAction, &QAction::triggered, sp, &SurveyPattern::reverseDirection);
        }

        SearchPattern *spat = qobject_cast<SearchPattern*>(mi);
        if(spat)
        {
            QAction *switchDirectionAction = menu.addAction("Switch Direction");
            connect(switchDirectionAction, &QAction::triggered, spat, &SearchPattern::switchDirection);
        }

        GeoGraphicsMissionItem *gmi = qobject_cast<GeoGraphicsMissionItem*>(mi);
        if(gmi)
        {
            if(gmi->locked())
            {
                QAction *unlockItemAction = menu.addAction("Unlock");
                connect(unlockItemAction, &QAction::triggered, gmi, &GeoGraphicsMissionItem::unlock);
            }
            else
            {
                QAction *lockItemAction = menu.addAction("Lock");
                connect(lockItemAction, &QAction::triggered, gmi, &GeoGraphicsMissionItem::lock);
            }
        }
        
        SurveyArea *sa = qobject_cast<SurveyArea*>(mi);
        if(sa)
        {
            if(project->getBackgroundRaster() && project->getDepthRaster())
            {
                QAction *generateAdaptiveTrackLinesAction = menu.addAction("Generate Adaptive Track Lines");
                connect(generateAdaptiveTrackLinesAction, &QAction::triggered, sa, &SurveyArea::generateAdaptiveTrackLines);
            }
        }
    }

    menu.exec(m_ui->treeView->mapToGlobal(pos));
}

void MainWindow::exportHypack() const
{
    project->exportHypack(m_ui->treeView->selectionModel()->currentIndex());
}

void MainWindow::exportMissionPlan() const
{
    project->exportMissionPlan(m_ui->treeView->selectionModel()->currentIndex());
}

void MainWindow::sendToROS() const
{
    project->sendToROS(m_ui->treeView->selectionModel()->currentIndex());
}

void MainWindow::appendMission() const
{
    project->appendMission(m_ui->treeView->selectionModel()->currentIndex());
}

void MainWindow::prependMission() const
{
    project->prependMission(m_ui->treeView->selectionModel()->currentIndex());
}

void MainWindow::updateMission() const
{
    project->updateMission(m_ui->treeView->selectionModel()->currentIndex());
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
    QString fname = QFileDialog::getOpenFileName(this,tr("Open"),m_workspace_path);

    if(!fname.isEmpty())
    {
        setCursor(Qt::WaitCursor);
        project->openBackground(fname);
        unsetCursor();
    }

}

void MainWindow::on_actionSurveyPattern_triggered()
{
    project->setContextMode(false);
    m_ui->projectView->setAddSurveyPatternMode();
}

void MainWindow::on_actionSurveyPatternFromContext_triggered()
{
    project->setContextMode(true);
    m_ui->projectView->setAddSurveyPatternMode();
}

void MainWindow::on_actionSurveyArea_triggered()
{
    project->setContextMode(false);
    m_ui->projectView->setAddSurveyAreaMode();
}

void MainWindow::on_actionSurveyAreaFromContext_triggered()
{
    project->setContextMode(true);
    m_ui->projectView->setAddSurveyAreaMode();
}

void MainWindow::on_actionSearchPattern_triggered()
{
    project->setContextMode(false);
    m_ui->projectView->setAddSearchPatternMode();
}

void MainWindow::on_actionSearchPatternFromContext_triggered()
{
    project->setContextMode(true);
    m_ui->projectView->setAddSearchPatternMode();
}


void MainWindow::on_actionBehavior_triggered()
{
    project->setContextMode(false);
    project->createBehavior();
}

void MainWindow::on_actionOpenGeometry_triggered()
{
    project->setContextMode(false);
    QString fname = QFileDialog::getOpenFileName(this,tr("Open"),m_workspace_path);

    if(!fname.isEmpty())
        project->openGeometry(fname);
}

void MainWindow::on_actionGroup_triggered()
{
    project->setContextMode(false);
    project->addGroup();
}

void MainWindow::on_actionGroupFromContext_triggered()
{
    project->setContextMode(true);
    project->addGroup();
}

void MainWindow::on_actionOrbit_triggered()
{
    project->setContextMode(false);
    project->addOrbit();
}

void MainWindow::on_actionOrbitFromContext_triggered()
{
    project->setContextMode(true);
    project->addOrbit();
}

void MainWindow::on_actionRadar_triggered()
{
    qDebug() << "radar: " << m_ui->actionRadar->isChecked();
    emit project->showRadar(m_ui->actionRadar->isChecked());
}

void MainWindow::on_actionFollow_triggered()
{
    //emit project->followRobot(m_ui->actionFollow->isChecked());
}

void MainWindow::activePlatfromPosition(QGeoCoordinate position)
{
    if(m_ui->actionFollow->isChecked())
      m_ui->projectView->centerMap(position);
}

void MainWindow::on_actionRadarColor_triggered()
{
    emit project->selectRadarColor();
}

void MainWindow::on_actionShowTail_triggered()
{
    emit project->showTail(m_ui->actionShowTail->isChecked());
}

void MainWindow::onROSConnected(bool connected)
{
    //m_ui->rosDetails->setEnabled(connected);
}

void MainWindow::on_actionAISManager_triggered()
{
    m_ais_manager->show();
}

void MainWindow::on_actionRadarManager_triggered()
{
    //m_radar_manager->show();
}

void MainWindow::on_actionSay_something_triggered()
{
    m_sound_play->show();
}
