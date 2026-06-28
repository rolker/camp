#include "autonomousvehicleproject.h"

#include <QStandardItemModel>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QGraphicsItem>
#include <QFileDialog>
#include <QTextStream>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QSvgRenderer>
#include <QMimeData>
#include <QDebug>

#include "geographicsitem.h"
#include "depth_raster.h"
#include "waypoint.h"
#include "trackline.h"
#include "surveypattern.h"
#include "surveyarea.h"
#include "group.h"
#include "searchpattern.h"
#include <gdal_priv.h>
#include "vector/vectordataset.h"
#include "vector/point.h"
#include "vector/polygon.h"
#include "vector/linestring.h"
#include "behavior.h"
#include "orbit.h"
#include "avoid_area.h"

#include "platform_manager/platform.h"
#include "mission_manager/mission_manager.h"

#include "map/map.h"
#include "map/map_item.h"
#include "map/layer_list.h"
#include "raster/raster_layer.h"
#include <QSettings>
#include <algorithm>

#include <iostream>
#include <sstream>
#include <cmath>

AutonomousVehicleProject::AutonomousVehicleProject(QObject *parent) : QAbstractItemModel(parent), m_currentGroup(nullptr), m_currentSelected(nullptr), m_symbols(new QSvgRenderer(QString(":/symbols.svg"),this)), m_map_scale(1.0), unique_label_counter(0)
{
    GDALAllRegister();

    // [#59 ADR-0002/0003] The Web-Mercator scene is owned by camp::map::Map.
    // Overlays position via web_mercator::geoToMap through the geoToPixel shim
    // and parent to the Map's persistent scene-origin anchor (originAnchor), so
    // they resolve independently of whether any chart layer is loaded.
    m_map = new camp::map::Map(this);
    m_scene = m_map->scene();
    // [#59 ADR-0003] Keep chart/depth bookkeeping in sync when a chart layer is
    // removed via the Layers-tab Remove action (camp_map Layer detaches through the
    // Map model; we react here so camp_map stays unaware of the project).
    connect(m_map, &QAbstractItemModel::rowsAboutToBeRemoved, this, &AutonomousVehicleProject::onChartLayerRemoved);

    m_root = new Group();
    m_root->setParent(this);
    m_root->setObjectName("root");
    m_currentGroup = m_root;
    setObjectName("projectModel");
    
    //m_ROSLink =  new ROSLink(this);
    //connect(this,&AutonomousVehicleProject::showTail,m_ROSLink, &ROSLink::showTail);
    //connect(this,&AutonomousVehicleProject::followRobot,m_ROSLink, &ROSLink::followRobot);
}

AutonomousVehicleProject::~AutonomousVehicleProject()
{
    for(auto* depth : m_depthRasters)
        delete depth;
}

QGraphicsScene *AutonomousVehicleProject::scene() const
{
    return m_scene;
}

camp::map::Map *AutonomousVehicleProject::map() const
{
    return m_map;
}

QString const &AutonomousVehicleProject::filename() const
{
    return m_filename;
}

QSvgRenderer * AutonomousVehicleProject::symbols() const
{
    return m_symbols;
}


void AutonomousVehicleProject::save(const QString &fname)
{
    QString saveName = fname;
    if(saveName.isEmpty())
        saveName = m_filename;
    if(!saveName.isEmpty())
    {
        QJsonObject projectObject;
        m_root->write(projectObject);

        projectObject["name"] = "project";


        QFile saveFile(saveName);
        if(saveFile.open(QFile::WriteOnly))
        {
            QJsonDocument saveDoc(projectObject);
            saveFile.write(saveDoc.toJson());
            m_filename = saveName;
        }
    }

}

void AutonomousVehicleProject::open(const QString &fname)
{
    qDebug() << "open:" << fname;
    QFile loadFile(fname);
    if(loadFile.open(QFile::ReadOnly))
    {
        QByteArray loadData = loadFile.readAll();
        QJsonDocument loadDoc(QJsonDocument::fromJson(loadData));
        m_root->read(loadDoc.object());
        emit layoutChanged();
    }
}


void AutonomousVehicleProject::openBackground(const QString &fname, QString label)
{
    // [#59 ADR-0003] Load + persist. The chart is now app state (a Map layer),
    // not a mission-tree node, so it no longer touches the mission model.
    addBackgroundLayer(fname, label);
    persistBackgrounds();
}

void AutonomousVehicleProject::addBackgroundLayer(const QString &fname, const QString &label)
{
    // [#59 ADR-0003] De-dup by filename: a chart already loaded must not stack a
    // second copy. Without this, the command-line chart arg (main.cpp always
    // openBackground()s it) plus restorePersistedBackgrounds re-loading the same
    // file would double-load it, and persist would then accumulate a duplicate
    // on every launch.
    for(auto* existing : m_chartLayers)
        if(existing->filename() == fname)
            return;

    // [#59 ADR-0003] Display the chart as a stacked, reprojecting RasterLayer
    // (exact GDAL warp to EPSG:3857), Map-owned (parented to topLevelLayers).
    // Charts STACK — we do not replace the previous one. The layer establishes
    // its scene extent synchronously in its constructor (stage 1), so valid()
    // and fit-to-extent are meaningful immediately, before pixels load.
    auto layers = m_map->topLevelLayers();
    if(!layers)
        return;
    auto* layer = new camp::raster::RasterLayer(layers, fname);
    if(!layer->valid())
    {
        // Not a usable raster. Detach through the Map model (keeps the model's
        // row count and any attached tree view in sync — a bare delete would
        // leave a dangling index) and discard. No depth provider, no signals.
        m_map->setMapItemParent(layer, nullptr);
        delete layer;
        return;
    }
    if(!label.isEmpty())
        layer->setObjectName(label);

    // [#59 ADR-0003] Notify before recording the layer so ProjectView can save
    // the current view center (to recenter rather than jump when stacking onto
    // an existing chart); hasBackground() still reflects the pre-add state here.
    emit aboutToUpdateBackground();
    m_chartLayers.push_back(layer);

    // [#59 ADR-0003] Append this chart's depth band to the provider list (one
    // entry per loaded chart; charts with no depth band contribute nothing).
    auto* depth = new DepthRaster(fname);
    if(depth->depthValid())
        m_depthRasters.push_back(depth);
    else
        delete depth;

    // [#59 ADR-0003] Emit after the layer exists and is recorded: ProjectView
    // fits to the new chart (currentBackgroundExtent = the last layer's scene
    // extent) and the overlay managers refresh their projected positions.
    emit updatingBackground();
    emit backgroundUpdated();
}

void AutonomousVehicleProject::openGeometry(const QString& fname, QString label)
{
    VectorDataset * vd;
    {
        RowInserter ri(*this,m_currentGroup);
        vd = new VectorDataset(m_currentGroup);
        if(label.isEmpty())
            vd->setObjectName(QFileInfo(fname).fileName());
        else
            vd->setObjectName(label);
        vd->open(fname);
    }
    connect(this,&AutonomousVehicleProject::updatingBackground,vd,&VectorDataset::updateProjectedPoints);
    emit layoutChanged();
}

bool AutonomousVehicleProject::importGeoJson(const QString& fname)
{
    QFile infile(fname);
    if(infile.open(QIODevice::ReadOnly|QIODevice::Text))
    {
        QByteArray loadData = infile.readAll();
        QJsonDocument loadDoc(QJsonDocument::fromJson(loadData));
        bool ret = m_root->readGeoJson(loadDoc.object());
        emit layoutChanged();
        qDebug() << "importGeoJson:" << ret;
        return ret;
    }
    return false;
}
void AutonomousVehicleProject::import(const QString& fname)
{
    if(importGeoJson(fname))
        return;

    // try Hypack L84 file
    QFile infile(fname);
    if(infile.open(QIODevice::ReadOnly|QIODevice::Text))
    {
        RowInserter ri(*this,m_currentGroup);
        Group * hypackGroup = new Group(m_currentGroup);
        QFileInfo info(fname);
        hypackGroup->setObjectName(info.fileName());
        TrackLine * currentLine = nullptr;
        QTextStream instream(&infile);
        while(!instream.atEnd())
        {
            QString line = instream.readLine();
            QStringList parts = line.split(" ");
            if(!parts.empty())
            {
                // hypack files seem to have lines that all start with a 3 character identifier
                if(parts[0].length() == 3)
                {
                    qDebug() << parts;
                    if(parts[0] == "LIN")
                    {
                        currentLine = new TrackLine(hypackGroup);
                        currentLine->setObjectName("trackline");
                    }
                    if(parts[0] == "LNN" && currentLine)
                    {
                        parts.removeAt(0);
                        currentLine->setObjectName(parts.join(" "));
                    }
                    if(parts[0] == "PTS" && currentLine)
                    {
                        for(int i = 1; i < parts.size()-1; i += 2)
                        {
                            bool ok = false;
                            double lat = parts[i].toDouble(&ok);
                            if(ok)
                            {
                                double lon = parts[i+1].toDouble(&ok);
                                if(ok)
                                    currentLine->addWaypoint(QGeoCoordinate(lat,lon))->setObjectName("waypoint");
                            }
                        }
                    }
                }
            }
        }
    }
}


bool AutonomousVehicleProject::hasBackground() const
{
    return !m_chartLayers.empty();
}

void AutonomousVehicleProject::persistBackgrounds() const
{
    // [#59 ADR-0003] Persist the ordered chart filename list as app state.
    // Per-layer settings (visible/opacity/colormap) already persist via camp_map's
    // QSettings-by-itemID mechanism; this records which charts to recreate, in
    // order, so itemIDs (and thus those per-layer settings) line up on restore.
    QStringList files;
    for(auto* layer : m_chartLayers)
        files.push_back(layer->filename());
    QSettings settings;
    settings.setValue("backgrounds/files", files);
}

void AutonomousVehicleProject::restorePersistedBackgrounds()
{
    // [#59 ADR-0003] Recreate the persisted chart layers (app state). Read the
    // list first (addBackgroundLayer does not persist, so the stored list is
    // stable across the loop) and recreate in order; per-layer settings restore
    // by itemID as each layer is rebuilt. Skipped silently for files that no
    // longer open (addBackgroundLayer rejects an invalid raster).
    QSettings settings;
    const QStringList files = settings.value("backgrounds/files").toStringList();
    for(const auto& fname : files)
        addBackgroundLayer(fname, QString());
    // Re-persist once: addBackgroundLayer de-dups and drops files that no longer
    // open, so this self-heals a stored list that had accumulated duplicates or
    // stale entries down to what actually loaded.
    if(files.size() != static_cast<int>(m_chartLayers.size()))
        persistBackgrounds();
}

void AutonomousVehicleProject::onChartLayerRemoved(const QModelIndex& parent, int first, int last)
{
    // [#59 ADR-0003] A layer is being detached from the Map model (Layers-tab
    // Remove). The item still exists during rowsAboutToBeRemoved, so we can read
    // its filename. For each removed row that is one of our tracked chart layers,
    // drop the matching depth provider + bookkeeping entry, then re-persist.
    bool changed = false;
    for(int row = first; row <= last; ++row)
    {
        auto idx = m_map->index(row, 0, parent);
        auto* item = reinterpret_cast<camp::map::MapItem*>(idx.internalPointer());
        auto* layer = qobject_cast<camp::raster::RasterLayer*>(item);
        if(!layer)
            continue;
        auto it = std::find(m_chartLayers.begin(), m_chartLayers.end(), layer);
        if(it == m_chartLayers.end())
            continue;  // a non-chart layer (e.g. an OSM/WMTS base layer)
        const QString fname = layer->filename();
        for(auto dit = m_depthRasters.begin(); dit != m_depthRasters.end(); ++dit)
            if((*dit)->filename() == fname)
            {
                delete *dit;
                m_depthRasters.erase(dit);
                break;
            }
        m_chartLayers.erase(it);
        changed = true;
    }
    if(changed)
    {
        persistBackgrounds();
        // Refresh overlays (depth-dependent planning, fit-to-extent presence).
        emit backgroundUpdated();
    }
}

QGraphicsItem *AutonomousVehicleProject::originAnchor() const
{
    // [#59 PR6] The map's persistent scene-root item: always in the scene, at
    // the origin, regardless of whether a chart is loaded. Top-level mission
    // items parent to it so they render (and don't crash) over OSM/WMTS-only.
    return m_map ? m_map->rootItem() : nullptr;
}

QRectF AutonomousVehicleProject::currentBackgroundExtent() const
{
    // [#59 ADR-0003] Fit-to-extent targets the most-recently-added chart layer.
    // The RasterLayer establishes its scene transform/position synchronously in
    // its constructor (stage 1), so this is valid the moment a chart loads — no
    // BackgroundRaster georeference round-trip needed.
    if(!m_chartLayers.empty())
        return m_chartLayers.back()->sceneBoundingRect();
    return QRectF();
}

float AutonomousVehicleProject::getDepth(QGeoCoordinate const &location) const
{
    // [#59 ADR-0003] Walk the depth-provider list in load order; the first
    // provider with a valid (non-NaN) sounding at this location wins (order
    // resolves overlap between charts). NaN if no provider covers the point.
    for(auto* depth : m_depthRasters)
    {
        if(!depth->depthValid())
            continue;
        const float d = depth->getDepth(location);
        if(!std::isnan(d))
            return d;
    }
    return std::nanf("");
}

bool AutonomousVehicleProject::hasDepth() const
{
    for(auto* depth : m_depthRasters)
        if(depth->depthValid())
            return true;
    return false;
}

Behavior * AutonomousVehicleProject::createBehavior()
{
    Behavior *b = potentialParentItemFor("Behavior")->createMissionItem<Behavior>(generateUniqueLabel("behavior"));
    emit layoutChanged();
    return b;
}

Group * AutonomousVehicleProject::createGroup(MissionItem* parent, int row, QString label)
{
    Group *g;
    if(label.isEmpty())
        label = generateUniqueLabel("group");
    if(!parent)
        g = potentialParentItemFor("Group")->createMissionItem<Group>(label, row);
    else
        g = parent->createMissionItem<Group>(label, row);
    emit layoutChanged();
    return g;
}

Group * AutonomousVehicleProject::addGroup()
{
    Group *g = potentialParentItemFor("Group")->createMissionItem<Group>(generateUniqueLabel("group"));
    emit layoutChanged();
    return g;
}

Orbit * AutonomousVehicleProject::createOrbit(MissionItem* parent, int row, QString label)
{
    Orbit *o;
    if(label.isEmpty())
        label = generateUniqueLabel("orbit");
    if(!parent)
        o = potentialParentItemFor("Orbit")->createMissionItem<Orbit>(label, row);
    else
        o = parent->createMissionItem<Orbit>(label, row);
    emit layoutChanged();
    return o;
}

Orbit * AutonomousVehicleProject::addOrbit()
{
    Orbit *o = potentialParentItemFor("Orbit")->createMissionItem<Orbit>(generateUniqueLabel("orbit"));
    emit layoutChanged();
    return o;
}

void AutonomousVehicleProject::setContextMode(bool mode)
{
    m_contextMode = mode;
}

MissionItem *AutonomousVehicleProject::potentialParentItemFor(std::string const &childType)
{
    if(!m_contextMode)
        return m_root;
    MissionItem * parentItem = m_currentSelected;
    if(!parentItem)
        parentItem = m_root;
    while(parentItem && !parentItem->canAcceptChildType(childType))
        parentItem = qobject_cast<MissionItem*>(parentItem->parent());
    return parentItem;
}

Waypoint *AutonomousVehicleProject::addWaypoint(QGeoCoordinate position)
{
    
    Waypoint *wp = potentialParentItemFor("Waypoint")->createMissionItem<Waypoint>(generateUniqueLabel("waypoint"));
    wp->setLocation(position);
    connect(this,&AutonomousVehicleProject::updatingBackground,wp,&Waypoint::updateBackground);
    emit layoutChanged();
    return wp;
}


SurveyPattern * AutonomousVehicleProject::createSurveyPattern(MissionItem* parent, int row, QString label)
{
    SurveyPattern *sp;
    if(label.isEmpty())
        label = generateUniqueLabel("pattern");
    if(!parent) 
        sp = potentialParentItemFor("SurveyPattern")->createMissionItem<SurveyPattern>(label, row);
    else
        sp = parent->createMissionItem<SurveyPattern>(label, row);
    connect(this,&AutonomousVehicleProject::updatingBackground,sp,&SurveyPattern::updateBackground);
    emit layoutChanged();
    return sp;

}

SurveyPattern *AutonomousVehicleProject::addSurveyPattern(QGeoCoordinate position)
{
    SurveyPattern *sp = createSurveyPattern();
    sp->setStartLocation(position);
//    connect(this,&AutonomousVehicleProject::updatingBackground,sp,&SurveyPattern::updateBackground);
    return sp;
}

SurveyArea * AutonomousVehicleProject::createSurveyArea(MissionItem* parent, int row, QString label)
{
    SurveyArea *sa;
    if(label.isEmpty())
        label = generateUniqueLabel("area");
    if(!parent)
        sa = potentialParentItemFor("SurveyArea")->createMissionItem<SurveyArea>(label, row);
    else
        sa = parent->createMissionItem<SurveyArea>(label, row);
    return sa;
}

SurveyArea * AutonomousVehicleProject::addSurveyArea(QGeoCoordinate position)
{
    SurveyArea *sa = createSurveyArea();
    sa->setPos(sa->geoToPixel(position));
    sa->addWaypoint(position);
    connect(this,&AutonomousVehicleProject::updatingBackground,sa,&SurveyArea::updateBackground);
    return sa;
}

AvoidArea * AutonomousVehicleProject::createAvoidArea(MissionItem* parent, int row, QString label)
{
    AvoidArea *aa;
    if(label.isEmpty())
        label = generateUniqueLabel("avoid");
    if(!parent)
        aa = potentialParentItemFor("AvoidArea")->createMissionItem<AvoidArea>(label, row);
    else
        aa = parent->createMissionItem<AvoidArea>(label, row);
    connect(aa, &AvoidArea::avoidAreaChanged, this, &AutonomousVehicleProject::updateAvoidanceAreas);
    return aa;
}

AvoidArea * AutonomousVehicleProject::addAvoidArea(QGeoCoordinate position)
{
    AvoidArea *aa = createAvoidArea();
    aa->setPos(aa->geoToPixel(position));
    aa->addPoint(position);
    connect(this,&AutonomousVehicleProject::updatingBackground,aa,&AvoidArea::updateBackground);
    return aa;
}

SearchPattern * AutonomousVehicleProject::createSearchPattern(MissionItem* parent, int row, QString label)
{
  SearchPattern *sp;
  if(label.isEmpty())
    label = generateUniqueLabel("pattern");
  if(!parent) 
    sp = potentialParentItemFor("SearchPattern")->createMissionItem<SearchPattern>(label, row);
  else
    sp = parent->createMissionItem<SearchPattern>(label, row);
  connect(this,&AutonomousVehicleProject::updatingBackground,sp,&SearchPattern::updateBackground);
  emit layoutChanged();
  return sp;
}

SearchPattern *AutonomousVehicleProject::addSearchPattern(QGeoCoordinate position)
{
  SearchPattern *sp = createSearchPattern();
  sp->setStartLocation(position);
  return sp;
}


TrackLine * AutonomousVehicleProject::createTrackLine(MissionItem* parent, int row, QString label)
{
    TrackLine *tl;
    if(label.isEmpty())
        label = generateUniqueLabel("trackline");
    if(!parent)
        tl = potentialParentItemFor("TrackLine")->createMissionItem<TrackLine>(label, row);
    else
        tl = parent->createMissionItem<TrackLine>(label, row);
    return tl;
}


TrackLine * AutonomousVehicleProject::addTrackLine(QGeoCoordinate position)
{
    TrackLine *tl = createTrackLine();
    tl->setPos(tl->geoToPixel(position));
    tl->addWaypoint(position);
    connect(this,&AutonomousVehicleProject::updatingBackground,tl,&TrackLine::updateBackground);
    return tl;
}

void AutonomousVehicleProject::exportHypack(const QModelIndex &index)
{
    MissionItem *item = itemFromIndex(index);
    TrackLine *tl = qobject_cast<TrackLine*>(item);
    if(tl)
    {
        QString fname = QFileDialog::getSaveFileName(qobject_cast<QWidget*>(QObject::parent()));
        if(fname.length() > 0)
        {
            QFile outfile(fname);
            if(outfile.open(QFile::WriteOnly))
            {
                QTextStream outstream(&outfile);
                outstream << "LNS 1\n";
                auto waypoints = tl->childMissionItems();
                outstream << "LIN " << waypoints.size() << "\n";
                for(auto i: waypoints)
                {
                    const Waypoint *wp = qobject_cast<Waypoint const*>(i);
                    if(wp)
                    {
                        auto ll = wp->location();
                        outstream << "PTS " << ll.latitude() << " " << ll.longitude() << "\n";
                    }
                }
                outstream << "LNN 1\n";
                outstream << "EOL\n";
            }
        }
    }
    SurveyPattern *sp = qobject_cast<SurveyPattern*>(item);
    if(sp)
    {
        QString fname = QFileDialog::getSaveFileName(qobject_cast<QWidget*>(QObject::parent()));
        if(fname.length() > 0)
        {
            QFile outfile(fname);
            if(outfile.open(QFile::WriteOnly))
            {
                auto lines = sp->getLines();
                QTextStream outstream(&outfile);
                outstream.setRealNumberPrecision(8);
                outstream << "LNS " << lines.length() << "\n";
                int lineNum = 1;
                for (auto l: lines)
                {
                    outstream << "LIN " << l.length() << "\n";
                    for (auto p:l)
                        outstream << "PTS " << p.latitude() << " " << p.longitude() << "\n";
                    outstream << "LNN " << lineNum << "\n";
                    lineNum++;
                    outstream << "EOL\n";
                }
            }
        }
    }
}

QJsonDocument AutonomousVehicleProject::generateMissionPlan(const QModelIndex& index)
{
    MissionItem *item = itemFromIndex(index);
    QJsonDocument plan;
    QJsonObject topLevel;
    QJsonObject defaultParameters;
    topLevel["DEFAULT_PARAMETERS"] = defaultParameters;
    QJsonArray navArray;
    item->writeToMissionPlan(navArray);
    topLevel["NAVIGATION"] = navArray;
    plan.setObject(topLevel);
    return plan;
}

QJsonDocument AutonomousVehicleProject::generateGeoJson(const QModelIndex& index)
{
  MissionItem *item = itemFromIndex(index);
  QJsonDocument doc;
  QJsonObject topLevel;
  topLevel["type"] = "FeatureCollection";
  topLevel["name"] = item->objectName();
  QJsonArray features;
  item->writeToGeoJson(features);
  topLevel["features"] = features;
  doc.setObject(topLevel);
  return doc;
}

void AutonomousVehicleProject::exportMissionPlan(const QModelIndex& index)
{
    QString fname = QFileDialog::getSaveFileName(qobject_cast<QWidget*>(QObject::parent()));
    if(fname.length() > 0)
    {
        QJsonDocument plan = generateMissionPlan(index);
        QFile saveFile(fname);
        if(saveFile.open(QFile::WriteOnly))
        {
            saveFile.write(plan.toJson());
        }
    }
}

void AutonomousVehicleProject::exportGeoJson(const QModelIndex& index)
{
  QString fname = QFileDialog::getSaveFileName(qobject_cast<QWidget*>(QObject::parent()));
  if(fname.length() > 0)
  {
    QJsonDocument plan = generateGeoJson(index);
    QFile saveFile(fname);
    if(saveFile.open(QFile::WriteOnly))
    {
        saveFile.write(plan.toJson());
    }
  }
}

QJsonDocument AutonomousVehicleProject::generateMissionTask(const QModelIndex& index)
{
    MissionItem *mi = itemFromIndex(index);
    QJsonDocument plan;// = generateMissionPlan(index);
    
    QJsonArray topArray;
    QJsonObject miObject;
    mi->write(miObject);
    topArray.append(miObject);
    plan.setArray(topArray);
    
    return plan;
}

void AutonomousVehicleProject::sendToROS(const QModelIndex& index)
{
    MissionItem *mi = itemFromIndex(index);
    QJsonDocument plan = generateMissionTask(index);
    
    if(m_activePlatform)
    {
        m_activePlatform->missionManager()->sendMissionPlan(plan.toJson());
    }

    GeoGraphicsMissionItem * gmi = qobject_cast<GeoGraphicsMissionItem*>(mi);
    if(gmi)
        gmi->lock();
}

void AutonomousVehicleProject::updateAvoidanceAreas()
{
    marine_interfaces::msg::GeoOccupancyVectorMap avoidance_map;
    avoidance_map.header.frame_id = "wgs84";
    avoidance_map.bounds.min_pt.altitude = std::nan("");
    bool first_waypoint = true;

    for(auto &mission_item: m_root->childMissionItems())
    {
        auto avoid_area = qobject_cast<AvoidArea*>(mission_item);
        if(avoid_area)
        {
            auto waypoints = avoid_area->childMissionItems();
            marine_interfaces::msg::GeoOccupancyPolygon polygon;
            polygon.occupancy_probability = 100;
            for(auto item: waypoints)
            {
                auto wp_item = qobject_cast<Waypoint*>(item);
                if(wp_item)
                {
                    geographic_msgs::msg::GeoPoint gp;
                    gp.latitude = wp_item->location().latitude();
                    gp.longitude = wp_item->location().longitude();
                    // following is probably not dateline proof
                    if(first_waypoint)
                    {
                        avoidance_map.bounds.max_pt.latitude = gp.latitude;
                        avoidance_map.bounds.max_pt.longitude = gp.longitude;
                        avoidance_map.bounds.min_pt.latitude = gp.latitude;
                        avoidance_map.bounds.min_pt.longitude = gp.longitude;
                        first_waypoint = false;
                    }
                    else
                    {
                        avoidance_map.bounds.max_pt.latitude = std::max(gp.latitude, avoidance_map.bounds.max_pt.latitude);
                        avoidance_map.bounds.max_pt.longitude = std::max(gp.longitude, avoidance_map.bounds.max_pt.longitude);
                        avoidance_map.bounds.min_pt.latitude = std::min(gp.latitude, avoidance_map.bounds.min_pt.latitude);
                        avoidance_map.bounds.min_pt.longitude = std::min(gp.longitude, avoidance_map.bounds.min_pt.longitude);
                    }
                    polygon.polygon.points.push_back(gp);
                }
            }
            if(polygon.polygon.points.size() > 1)
                avoidance_map.polygons.push_back(polygon);

        }
    }

    if(m_activePlatform)
    {
        m_activePlatform->missionManager()->sendAvoidanceAreas(avoidance_map);
    }
}

void AutonomousVehicleProject::appendMission(const QModelIndex& index)
{
    QJsonDocument plan = generateMissionTask(index);
    if(m_activePlatform)
    {
        m_activePlatform->missionManager()->appendMission(plan.toJson());
    }
}

void AutonomousVehicleProject::prependMission(const QModelIndex& index)
{
    QJsonDocument plan = generateMissionTask(index);
    if(m_activePlatform)
    {
        m_activePlatform->missionManager()->prependMission(plan.toJson());
    }
}

void AutonomousVehicleProject::updateMission(const QModelIndex& index)
{
    QJsonDocument plan = generateMissionTask(index);
    if(m_activePlatform)
    {
        m_activePlatform->missionManager()->updateMission(plan.toJson());
    }
}


void AutonomousVehicleProject::deleteItems(const QModelIndexList &indices)
{
    // [#65] Resolve to items first, then delete only the TOPMOST selected ones.
    // If a parent (e.g. a trackline, group, or survey area) and one of its
    // descendants (e.g. a waypoint) are both selected, deleting the parent frees
    // the descendant — deleting it afterwards would removeItem()/qobject_cast a
    // freed pointer (the trackline-removal crash, #65). Deleting a parent already
    // takes its children with it.
    std::vector<MissionItem*> items;
    for(auto index: indices)
        if(auto* item = itemFromIndex(index))
            items.push_back(item);

    for(auto* item: items)
    {
        bool has_selected_ancestor = false;
        for(QObject* a = item->parent(); a && !has_selected_ancestor; a = a->parent())
            has_selected_ancestor = std::find(items.begin(), items.end(), a) != items.end();
        if(!has_selected_ancestor)
            deleteItem(item);
    }
}

void AutonomousVehicleProject::deleteItem(const QModelIndex &index)
{
    MissionItem *item = itemFromIndex(index);
    // [#65] Defensive: a stale index (e.g. an item already freed as a child of a
    // previously-deleted parent) resolves to no/garbage item — never touch it.
    if(!item)
        return;
    // [#65] Clear selection/group bookkeeping if it points at the item being
    // deleted (or a descendant of it — deleting a parent frees its children).
    // Otherwise endRemoveRows below fires the tree's currentChanged cascade,
    // which dereferences the now-freed m_currentSelected → use-after-free.
    for(MissionItem* s = m_currentSelected; s; s = qobject_cast<MissionItem*>(s->parent()))
        if(s == item) { m_currentSelected = nullptr; break; }
    for(MissionItem* g = m_currentGroup; g; g = qobject_cast<MissionItem*>(g->parent()))
        if(g == item) { m_currentGroup = m_root; break; }

    GeoGraphicsMissionItem *ggi = qobject_cast<GeoGraphicsMissionItem*>(item);
    if(ggi)
    {
        GeoGraphicsItem *pggi = qgraphicsitem_cast<GeoGraphicsItem*>(ggi->parentItem());
        if(pggi)
            pggi->prepareGeometryChange();
        m_scene->removeItem(ggi);
    }
    // [#59 ADR-0003] Charts are no longer mission-tree items, so deleteItem never
    // sees a chart here — chart layers (and their depth providers) are removed
    // via the Layers-tab Remove action (see onChartLayerRemoved).
    QModelIndex p = parent(index);
    MissionItem * pi = itemFromIndex(p);
    // [#86] Guard the parent/row lookup. pi is null if parent(index) is invalid
    // (an orphaned or root-edge item); rownum is -1 if the item is no longer in
    // its parent's child list (a re-entrant or double delete reaching a still-
    // alive, already-detached item — a window the deleteLater() below widens).
    // Either way there is nothing to remove from the model: just free the item.
    // beginRemoveRows(p,-1,-1) would otherwise be an invalid range and assert.
    int rownum = pi ? pi->childMissionItems().indexOf(item) : -1;
    if(rownum < 0)
    {
        item->deleteLater();
        return;
    }
    // [#86] Complete the model removal with the item still alive, then defer the
    // delete. The old synchronous `delete item` ran mid-cascade (before
    // endRemoveRows), so any slot reached by endRemoveRows' currentChanged could
    // touch a freed object. deleteLater() — the same idiom the camp_map layer-delete
    // path uses (camp::map::Layer::removeFromMap) — lets connected slots unwind
    // first; QPointer observers in the detail panels null out when it finally dies.
    beginRemoveRows(p,rownum,rownum);
    pi->removeChildMissionItem(item);
    endRemoveRows();
    item->deleteLater();
}

void AutonomousVehicleProject::deleteItem(MissionItem *item)
{
    deleteItem(indexFromItem(item));
}

void AutonomousVehicleProject::setCurrent(const QModelIndex &index)
{
    auto last_selected = m_currentSelected;
    
    m_currentSelected = itemFromIndex(index);
    if(m_currentSelected)
    {
        QString itemType = m_currentSelected->metaObject()->className();

        Group *g = qobject_cast<Group*>(m_currentSelected);
        if(g)
            m_currentGroup = g;
        else
            m_currentGroup = m_root;
        GeoGraphicsMissionItem * ggmi = qobject_cast<GeoGraphicsMissionItem*>(m_currentSelected);
        if(ggmi)
            ggmi->update();
    }
    GeoGraphicsMissionItem * ggmi = qobject_cast<GeoGraphicsMissionItem*>(last_selected);
    if(ggmi)
        ggmi->update();
}

MissionItem * AutonomousVehicleProject::currentSelected() const
{
    return m_currentSelected;
}

void AutonomousVehicleProject::renameItem(MissionItem * item, const QString & label)
{
    // [#85] Apply the new label and tell the view. data() returns objectName for
    // DisplayRole, so a dataChanged on the item's index (all roles by default) is
    // what makes the tree refresh the displayed name after a rename.
    if(!item)
        return;
    item->setObjectName(label);
    QModelIndex idx = indexFromItem(item);
    if(idx.isValid())
        emit dataChanged(idx, idx);
}

QModelIndex AutonomousVehicleProject::index(int row, int column, const QModelIndex& parent) const
{
    if(column != 0 || row < 0)
        return QModelIndex();
    MissionItem * parentItem = m_root;
    if(parent.isValid())
        parentItem = itemFromIndex(parent);
    if(parentItem)
    {
        auto subitems = parentItem->childMissionItems();
        if(row < subitems.size())
            return createIndex(row,0,subitems[row]);
    }
    return QModelIndex();
}

QModelIndex AutonomousVehicleProject::indexFromItem(MissionItem* item) const
{
    if(item) 
    {
        if(item == m_root)
            return createIndex(0,0,item);
        MissionItem * parentItem = qobject_cast<MissionItem*>(item->parent());
        if(parentItem)
            return createIndex(parentItem->childMissionItems().indexOf(item),0,item);
    }
    return QModelIndex();
}

MissionItem * AutonomousVehicleProject::itemFromIndex(const QModelIndex& index) const
{
    if(index.isValid())
        return reinterpret_cast<MissionItem*>(index.internalPointer());
    //return m_root;
    return nullptr;
}

int AutonomousVehicleProject::rowCount(const QModelIndex& parent) const
{
    //qDebug() << "rowCount valid parent: " << parent.isValid();
    MissionItem * item = m_root;
    if(parent.isValid())
        item = itemFromIndex(parent);
    //qDebug() << " item: " << bool(item);
    //if(item)
    //    qDebug() << "   " << item->objectName() << " rows: " << item->childMissionItems().size();
    if(item)
        return item->childMissionItems().size();
    return 0;
}

int AutonomousVehicleProject::columnCount(const QModelIndex& parent) const
{
    return 1;
}

QModelIndex AutonomousVehicleProject::parent(const QModelIndex& child) const
{
    if(child.isValid() && itemFromIndex(child))
    {        
        MissionItem* item = qobject_cast<MissionItem*>(itemFromIndex(child)->parent());
        if(item)
            return createIndex(item->row(),0,item);
    }
    return QModelIndex();
}

QVariant AutonomousVehicleProject::data(const QModelIndex& index, int role) const
{
    MissionItem * item = itemFromIndex(index);
    if(item)
    {
        if (role == Qt::DisplayRole)
            return item->objectName();
    }
    return QVariant();
}

Qt::ItemFlags AutonomousVehicleProject::flags(const QModelIndex& index) const
{
    MissionItem * item = itemFromIndex(index);
    if(item)
    {
        if(qobject_cast<Waypoint*>(item))
            if(qobject_cast<SurveyPattern*>(item->parent()))
                return QAbstractItemModel::flags(index);
            else
              if(qobject_cast<SearchPattern*>(item->parent()))
                return QAbstractItemModel::flags(index);
              else
                return QAbstractItemModel::flags(index)|Qt::ItemIsDragEnabled;

        if(qobject_cast<SurveyPattern*>(item))
            return QAbstractItemModel::flags(index)|Qt::ItemIsDragEnabled;

        if(qobject_cast<SearchPattern*>(item))
            return QAbstractItemModel::flags(index)|Qt::ItemIsDragEnabled;

        if(qobject_cast<VectorDataset*>(item))
            return QAbstractItemModel::flags(index);

        if(qobject_cast<Point*>(item))
            return QAbstractItemModel::flags(index)|Qt::ItemIsDragEnabled;
        
        if(qobject_cast<Polygon*>(item))
            return QAbstractItemModel::flags(index)|Qt::ItemIsDragEnabled;

        if(qobject_cast<LineString*>(item))
            return QAbstractItemModel::flags(index)|Qt::ItemIsDragEnabled;

        return QAbstractItemModel::flags(index)|Qt::ItemIsDragEnabled|Qt::ItemIsDropEnabled;
    }
    
    return Qt::ItemIsDropEnabled;
}

QVariant AutonomousVehicleProject::headerData(int section, Qt::Orientation orientation, int role) const
{
    return QVariant();
}

Qt::DropActions AutonomousVehicleProject::supportedDropActions() const
{
    return Qt::MoveAction;
}


bool AutonomousVehicleProject::removeRows(int row, int count, const QModelIndex& parent)
{
    MissionItem * parentItem = m_root;
    if(parent.isValid())
        parentItem = itemFromIndex(parent);
    qDebug() << "removeRows from " << parentItem->objectName() << " row " << row << " count " << count;
    for(auto c: parentItem->childMissionItems())
        qDebug() << "      " << c->objectName();
    if(parentItem)
    {
        if(qobject_cast<SurveyPattern*>(parentItem))
            return false;
        if(qobject_cast<SearchPattern*>(parentItem))
            return false;
        for (int i = 0; i < count; i++)
            if(parentItem->childMissionItems().size() > row)
                deleteItem(parentItem->childMissionItems()[row]);
            else
                return false;
        return true;
    }
    return false;
}

QStringList AutonomousVehicleProject::mimeTypes() const
{
    QStringList ret;
    ret.append("application/json");
    ret.append("text/plain");
    return ret;
}

QMimeData * AutonomousVehicleProject::mimeData(const QModelIndexList& indexes) const
{
    QList<MissionItem*> itemList;
    for(QModelIndex itemIndex: indexes)
    {
        MissionItem * item = itemFromIndex(itemIndex);
        if(item)
            itemList.append(item);
    }
    
    if(itemList.empty())
        return nullptr;

    QMimeData *mimeData = new QMimeData();
    
    QJsonArray mimeArray;
    
    for(MissionItem *item: itemList)
    {
        QJsonObject itemObject;
        item->write(itemObject);
        mimeArray.append(itemObject);
    }
    
    mimeData->setData("application/json", QJsonDocument(mimeArray).toJson());
    mimeData->setData("text/plain", QJsonDocument(mimeArray).toJson());
        
    return mimeData;
}

bool AutonomousVehicleProject::canDropMimeData(const QMimeData* data, Qt::DropAction action, int row, int column, const QModelIndex& parent) const
{
    // qDebug() << "can drop?";
    // qDebug() << "  parent valid:" << parent.isValid();
    // qDebug() << "  dropMimeData: " << row << ", " << column;
    // qDebug() << "  mime encoded: " << data->data("application/json");
    
    QJsonDocument doc(QJsonDocument::fromJson(data->data("application/json")));
    
    MissionItem * parentItem = itemFromIndex(parent);
    if(!parentItem)
    {
        parentItem = m_root;
        row = -1;
    }
    
    // qDebug() << "  parent: " << parentItem->objectName();

    if(doc.array().empty())
        return false;

    bool ret = true;
    for(auto child: doc.array())
    {
        QJsonObject object = child.toObject();
        ret = ret && parentItem->canAcceptChildType(object["type"].toString().toStdString());
    }
    return ret;
}

bool AutonomousVehicleProject::dropMimeData(const QMimeData* data, Qt::DropAction action, int row, int column, const QModelIndex& parent)
{
    // qDebug() << "parent valid:" << parent.isValid();
    qDebug() << "dropMimeData: " << row << ", " << column;
    qDebug() << "mime encoded: " << data->data("application/json");
    
    QJsonDocument doc(QJsonDocument::fromJson(data->data("application/json")));
    
    MissionItem * parentItem = itemFromIndex(parent);
    if(!parentItem)
    {
        parentItem = m_root;
        row = -1;
    }
    
    // qDebug() << "parent: " << parentItem->objectName();

    parentItem->readChildren(doc.array(), row);

    return true;
        
}

void AutonomousVehicleProject::updateMapScale(qreal scale)
{
    // [#59 ADR-0003] Project-level map scale (driven by ProjectView::scaleChanged).
    // Glyph readers (Waypoint::shape, drawArrow, updateETE) read it via mapScale();
    // no per-chart copy to update now that BackgroundRaster is retired.
    //
    // The scale change resizes constant-pixel glyphs (Waypoint markers/arrows,
    // task-overlay markers, etc.), so their item-coordinate boundingRect changes.
    // Without notifying the scene, it keeps the stale cached rect and clips the
    // glyph when zooming out. Invalidate every scale-dependent scene item's
    // geometry here (before the scale changes) so the scene re-indexes and
    // repaints them at the new size. This covers all GeoGraphicsItems in the
    // scene, including overlays (e.g. running-task items, camp#129) that live
    // outside the mission-item tree.
    if (m_scene)
        for (QGraphicsItem* item : m_scene->items())
            if (auto* ggi = dynamic_cast<GeoGraphicsItem*>(item))
                ggi->prepareGeometryChange();
    m_map_scale = scale;
}

qreal AutonomousVehicleProject::mapScale() const
{
    return m_map_scale;
}

QString AutonomousVehicleProject::generateUniqueLabel(std::string const &prefix)
{
    std::stringstream ret;
    ret << prefix;

    std::stringstream number;
    number << unique_label_counter;
    unique_label_counter++;
    
    int padding = 4-number.str().length();
    for(int i = 0; i < padding; i++)
        ret << '0';
    
    ret << number.str();
    
    return QString(ret.str().c_str());
}

void AutonomousVehicleProject::updateActivePlatform(Platform* platform)
{
    m_activePlatform = platform;
}

Platform* AutonomousVehicleProject::activePlatform() const
{
    return m_activePlatform;
}

void AutonomousVehicleProject::setSpeed(double speed)
{
    m_speed = speed;
}

double AutonomousVehicleProject::speed() const
{
    return m_speed;
}

void AutonomousVehicleProject::setThrottle(double throttle)
{
    throttle_ = throttle;
}

double AutonomousVehicleProject::throttle() const
{
    return throttle_;
}


AutonomousVehicleProject::RowInserter::RowInserter(AutonomousVehicleProject& project, MissionItem* parent, int row):m_project(project)
{
    //qDebug() << "RowInserter: row " << row << " parent " << parent->objectName();
    if(row < 0) // append
        project.beginInsertRows(project.indexFromItem(parent),parent->childMissionItems().size(),parent->childMissionItems().size());
    else
        project.beginInsertRows(project.indexFromItem(parent), row, row);
}

AutonomousVehicleProject::RowInserter::~RowInserter()
{
    m_project.endInsertRows();
}

