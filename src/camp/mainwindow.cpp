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

#include "trackline.h"
#include "surveypattern.h"
#include "surveyarea.h"
#include "searchpattern.h"

#include "ais/ais_manager.h"
#include "platform_manager/platform.h"
#include "collision_monitor/collision_monitor_manager.h"
#include "footprint/footprint_manager.h"

#include "map/map.h"
#include "map/layer.h"
#include "map/layer_list.h"
#include "ros/node.h"          // camp2's camp::ros::Node (src/camp2/ros/node.h)
#include "map_tree_view/map_tree_view.h"
#include <QTabWidget>

#include <QCloseEvent>
#include <QSettings>
#include <QTimer>

#include <QDebug>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    m_ui(new Ui::MainWindow)
{
    m_ui->setupUi(this);
    GDALAllRegister();
    project = new AutonomousVehicleProject(this);

    m_ui->treeView->setModel(project);
    m_ui->projectView->setStatusBar(statusBar());
    m_ui->projectView->setProject(project);

    m_ui->detailsView->setProject(project);
    connect(m_ui->treeView->selectionModel(),&QItemSelectionModel::currentChanged,m_ui->detailsView,&DetailsView::onCurrentItemChanged);

    connect(m_ui->treeView->selectionModel(),&QItemSelectionModel::currentChanged,this,&MainWindow::setCurrent);

    // [#59 PR3b] Two-model split (ADR-0002): tab the existing mission tree
    // alongside a Web-Mercator layer tree (camp::map::Map) in the left panel.
    // The mission tree edits the plan; the layer tree manages backgrounds,
    // OSM/WMTS tiles, and chart raster layers (inline visibility checkboxes +
    // opacity delegate). detailsView follows the active tab (mission selection
    // on the Mission tab; cleared on the Layers tab — layer detail widgets are
    // future work). Built in code to avoid reworking the .ui splitter layout.
    auto treeTabs = new QTabWidget(this);
    const int treeSlot = m_ui->missionElementsSplitter->indexOf(m_ui->treeView);
    auto mapTreeView = new camp::map_tree_view::MapTreeView(treeTabs);
    mapTreeView->setMap(project->map());
    treeTabs->addTab(m_ui->treeView, "Mission");   // reparents treeView out of the splitter
    treeTabs->addTab(mapTreeView, "Layers");
    m_ui->missionElementsSplitter->insertWidget(treeSlot, treeTabs);
    connect(treeTabs, &QTabWidget::currentChanged, this, [this, treeTabs](int)
    {
        if(treeTabs->currentWidget() == m_ui->treeView)
            m_ui->detailsView->onCurrentItemChanged(m_ui->treeView->currentIndex(), QModelIndex());
        else
            m_ui->detailsView->onCurrentItemChanged(QModelIndex(), QModelIndex());  // clear for layer tab
    });

    connect(project, &AutonomousVehicleProject::backgroundUpdated, m_ui->projectView, &ProjectView::updateBackground);
    connect(project, &AutonomousVehicleProject::aboutToUpdateBackground, m_ui->projectView, &ProjectView::beforeUpdateBackground);

    //connect(m_ui->projectView,&ProjectView::currentChanged,this,&MainWindow::setCurrent);

    // [#59 PR6] Anchor platform overlays to the map's persistent scene root so
    // they render with or without a chart loaded (OSM/WMTS-only).
    m_ui->platformManager->setAnchor(project->originAnchor());
    connect(project, &AutonomousVehicleProject::backgroundUpdated, m_ui->platformManager, &PlatformManager::updateBackground);

    connect(m_ui->platformManager, &PlatformManager::currentPlatform, project, &AutonomousVehicleProject::updateActivePlatform);
    connect(m_ui->platformManager, &PlatformManager::currentPlatformPosition, this, &MainWindow::activePlatformPosition);
    connect(m_ui->rosLink, &ROSLink::rosConnected, m_ui->platformManager, &PlatformManager::nodeStarted);

    connect(m_ui->projectView,&ProjectView::scaleChanged,project,&AutonomousVehicleProject::updateMapScale);

    m_ais_manager = new AISManager(this);
    // [#59 PR5] AIS contacts live under a non-removable "AIS" layer in the Layers
    // tab; its checkbox toggles all contacts. No separate window. The layer sits
    // at the scene origin (like the root anchor) so geoToPixel resolves contacts
    // correctly, and renders with or without a chart (OSM/WMTS-only).
    auto* ais_layer = new camp::map::Layer(project->map()->topLevelLayers(), "AIS");
    ais_layer->setRemovable(false);
    m_ais_manager->setAnchor(ais_layer);
    connect(project, &AutonomousVehicleProject::backgroundUpdated, m_ais_manager, &AISManager::updateBackground);
    connect(m_ui->projectView, &ProjectView::viewportChanged, m_ais_manager, &AISManager::updateViewport);
    connect(m_ui->rosLink, &ROSLink::rosConnected, m_ais_manager, &AISManager::nodeStarted);

    // [#59 PR5] Grids and markers are now provided by camp2's scene-correct
    // ros overlays, hosted on camp's existing ROS node (no second node). When
    // ROSLink connects, attach a camp::ros::Node to the Map's ToolsManager; it
    // auto-discovers grid/marker/geometry topics and creates layers in the
    // Layers tab. Replaces camp's GridManager/MarkersManager (retired).
    connect(m_ui->rosLink, &ROSLink::rosConnected, this,
        [this](rclcpp::Node::SharedPtr node, tf2_ros::Buffer::SharedPtr buffer)
        {
            if(m_map_ros_started)
                return;
            m_map_ros_started = true;
            new camp::ros::Node(project->map()->toolsManager(), node, buffer);
        });

    m_collision_monitor_manager = new CollisionMonitorManager(this);
    // [#59 PR5] Collision zones live under a non-removable "Collision Monitor"
    // layer in the Layers tab; its checkbox toggles all zones. No separate window.
    auto* collision_layer = new camp::map::Layer(project->map()->topLevelLayers(), "Collision Monitor");
    collision_layer->setRemovable(false);
    m_collision_monitor_manager->setAnchor(collision_layer);
    connect(m_ui->rosLink, &ROSLink::rosConnected, m_collision_monitor_manager, &CollisionMonitorManager::nodeStarted);
    connect(project, &AutonomousVehicleProject::backgroundUpdated, m_collision_monitor_manager, &CollisionMonitorManager::updateBackground);

    // [#64] Boat footprint lives under a non-removable "Boat Footprint" layer in
    // the Layers tab (checkbox toggles it). Discovers nav2 published_footprint
    // PolygonStamped topics and renders the outline; same pattern as collision.
    m_footprint_manager = new FootprintManager(this);
    auto* footprint_layer = new camp::map::Layer(project->map()->topLevelLayers(), "Boat Footprint");
    footprint_layer->setRemovable(false);
    m_footprint_manager->setAnchor(footprint_layer);
    connect(m_ui->rosLink, &ROSLink::rosConnected, m_footprint_manager, &FootprintManager::nodeStarted);
    connect(project, &AutonomousVehicleProject::backgroundUpdated, m_footprint_manager, &FootprintManager::updateBackground);

    m_ui->rosLink->connectROS();

    // [#59 ADR-0003] Recreate the persisted chart layers now that the background
    // signals are wired, so fit-to-extent and the overlay managers refresh for
    // the restored charts. Charts are app state, independent of any mission file.
    project->restorePersistedBackgrounds();

    // [camp#90] Restore window geometry/state and the map view position+zoom from
    // the previous session (saved in closeEvent). Geometry applies now; the map
    // view scale/center is deferred to the next event-loop turn so it isn't
    // overridden by initial layout or the restored charts' fit-to-extent.
    QSettings settings;
    restoreGeometry(settings.value("MainWindow/geometry").toByteArray());
    restoreState(settings.value("MainWindow/state").toByteArray());
    QTimer::singleShot(0, this, [this]()
    {
        QSettings s;
        const double scale = s.value("MainWindow/mapScale", 0.0).toDouble();
        if(scale > 0.0 && m_ui->projectView && m_ui->projectView->transform().m11() > 0.0)
        {
            const double factor = scale / m_ui->projectView->transform().m11();
            m_ui->projectView->scale(factor, factor);
            if(s.contains("MainWindow/mapCenter"))
                m_ui->projectView->centerOn(s.value("MainWindow/mapCenter").toPointF());
        }
    });
}

MainWindow::~MainWindow()
{
    delete m_ui;
    delete m_ais_manager;
    delete m_collision_monitor_manager;
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    // [camp#90] Persist window geometry/state + map view position/zoom so they
    // restore next session (restored in the constructor).
    QSettings settings;
    settings.setValue("MainWindow/geometry", saveGeometry());
    settings.setValue("MainWindow/state", saveState());
    if(m_ui->projectView)
    {
        settings.setValue("MainWindow/mapScale", m_ui->projectView->transform().m11());
        settings.setValue("MainWindow/mapCenter",
            m_ui->projectView->mapToScene(m_ui->projectView->viewport()->rect().center()));
    }
    emit closing();
    QMainWindow::closeEvent(event);
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

void MainWindow::setCurrent(const QModelIndex &index, const QModelIndex &previous)
{
    //m_ui->treeView->setCurrentIndex(index);
    //project->setCurrent(index);
    MissionItem* i = project->itemFromIndex(index);
    // [#86] currentChanged fires with an invalid index when the selection is
    // cleared — which happens when the selected mission item is deleted
    // (endRemoveRows leaves no current row, e.g. last item or a whole-mission
    // select-all delete). itemFromIndex then returns nullptr; dereferencing it
    // here was a synchronous crash on the delete path. Clear the fields instead.
    if(!i)
    {
        m_ui->speedLineEdit->clear();
        m_ui->throttleLineEdit->clear();
        m_ui->priorityLineEdit->clear();
        m_ui->taskDataLineEdit->clear();
        return;
    }
    m_ui->speedLineEdit->setText(QString::number(i->speed()));
    emit speedUpdated(i->speed());
    m_ui->throttleLineEdit->setText(QString::number(i->throttle()*100.0));
    m_ui->priorityLineEdit->setText(QString::number(i->priority()));
    m_ui->taskDataLineEdit->setText(QString(i->taskData().c_str()));
 }

void MainWindow::on_speedLineEdit_editingFinished()
{
    auto item = project->currentSelected();
    if(item) 
        item->setSpeed(m_ui->speedLineEdit->text().toDouble());
    bool ok;
    auto speed = m_ui->speedLineEdit->text().toDouble(&ok);
    if(ok)
    {
        emit speedUpdated(speed);
        project->setSpeed(speed);
    }
}

void MainWindow::on_throttleLineEdit_editingFinished()
{
    auto item = project->currentSelected();
    if(item) 
      item->setThrottle(m_ui->throttleLineEdit->text().toDouble()/100.0);
    bool ok;
    auto throttle = m_ui->throttleLineEdit->text().toDouble(&ok);
    if(ok)
    {
        throttle /= 100.0;
        emit throttleUpdated(throttle);
        project->setThrottle(throttle);
    }
}

void MainWindow::on_priorityLineEdit_editingFinished()
{
    bool ok;
    auto priority = m_ui->priorityLineEdit->text().toInt(&ok);
    if(ok)
    {
        auto item = project->currentSelected();
        if(item) 
            item->setPriority(priority);
    }
}

void MainWindow::on_taskDataLineEdit_editingFinished()
{
    auto item = project->currentSelected();
    if(item) 
        item->setTaskData(m_ui->taskDataLineEdit->text().toStdString());
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

        QAction *exportGeoJsonAction = exportMenu->addAction("Export GeoJSON");
        connect(exportGeoJsonAction, &QAction::triggered, [=](){this->project->exportGeoJson(index);});
        
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

        QAction *addBehaviorAction = addMenu->addAction("Add Behavior");
        connect(addBehaviorAction, &QAction::triggered, this, &MainWindow::on_actionBehavior_triggered);
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

        if(mi && mi->canAcceptChildType("Behavior"))
        {
            QAction *addBehaviorAction = addMenu->addAction("Add Behavior");
            connect(addBehaviorAction, &QAction::triggered, this, &MainWindow::on_actionBehaviorFromContext_triggered);
        }

        QAction *deleteItemAction = menu.addAction("Delete");
        connect(deleteItemAction, &QAction::triggered, [=](){this->project->deleteItems(m_ui->treeView->selectionModel()->selectedRows());});
        
        
        TrackLine *tl = qobject_cast<TrackLine*>(mi);
        if(tl)
        {
            QAction *reverseDirectionAction = menu.addAction("Reverse Direction");
            connect(reverseDirectionAction, &QAction::triggered, tl, &TrackLine::reverseDirection);
            if(project->hasDepth())
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
            if(project->hasDepth())
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

void MainWindow::on_actionBehaviorFromContext_triggered()
{
    project->setContextMode(true);
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

void MainWindow::on_actionAvoid_triggered()
{
    project->setContextMode(false);
    m_ui->projectView->setAddAvoidAreaMode();
}

void MainWindow::on_actionAvoidFromContext_triggered()
{
    project->setContextMode(true);
    m_ui->projectView->setAddAvoidAreaMode();
}


void MainWindow::on_actionFollow_triggered()
{
    //emit project->followRobot(m_ui->actionFollow->isChecked());
}

void MainWindow::activePlatformPosition(QGeoCoordinate position)
{
    if(m_ui->actionFollow->isChecked())
      m_ui->projectView->centerMap(position);
}

void MainWindow::on_actionShowTail_triggered()
{
    emit project->showTail(m_ui->actionShowTail->isChecked());
}

void MainWindow::onROSConnected(bool connected)
{
    //m_ui->rosDetails->setEnabled(connected);
}

