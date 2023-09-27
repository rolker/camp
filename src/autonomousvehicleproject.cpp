#include "autonomousvehicleproject.h"

#include <QStandardItemModel>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QFileDialog>
#include <QTextStream>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QSvgRenderer>
#include <QMimeData>
#include <QDebug>

#include "backgroundraster.h"
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

#include "platform_manager/platform.h"
#include "mission_manager/mission_manager.h"

#include <iostream>
#include <sstream>

AutonomousVehicleProject::AutonomousVehicleProject(QObject *parent) : QAbstractItemModel(parent), m_currentBackground(nullptr), m_currentDepthRaster(nullptr), m_currentGroup(nullptr), m_currentSelected(nullptr), m_symbols(new QSvgRenderer(QString(":/symbols.svg"),this)), m_map_scale(1.0), unique_label_counter(0)
{
    GDALAllRegister();

    m_scene = new QGraphicsScene(this);
    m_root = new Group();
    m_root->setParent(this);
    m_root->setObjectName("root");
    m_currentGroup = m_root;
    setObjectName("projectModel");
    
    //m_ROSLink =  new ROSLink(this);
    //connect(this,&AutonomousVehicleProject::showRadar,m_ROSLink, &ROSLink::showRadar);
    //connect(this,&AutonomousVehicleProject::selectRadarColor,m_ROSLink, &ROSLink::selectRadarColor);
    //connect(this,&AutonomousVehicleProject::showTail,m_ROSLink, &ROSLink::showTail);
    //connect(this,&AutonomousVehicleProject::followRobot,m_ROSLink, &ROSLink::followRobot);
}

AutonomousVehicleProject::~AutonomousVehicleProject()
{
}

QGraphicsScene *AutonomousVehicleProject::scene() const
{
    return m_scene;
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


BackgroundRaster* AutonomousVehicleProject::openBackground(const QString &fname, QString label)
{
    beginInsertRows(indexFromItem(m_root),m_root->childMissionItems().size(),m_root->childMissionItems().size());
    BackgroundRaster *bgr = new BackgroundRaster(fname, m_root);
    if(bgr->valid())
    {   
        if(label.isEmpty())
            bgr->setObjectName(QFileInfo(fname).fileName());
        else
            bgr->setObjectName(label);
        setCurrentBackground(bgr);
        endInsertRows();
        emit layoutChanged();
        return bgr;
    }
    else
    {
        endInsertRows();
        deleteItem(bgr);
    }
    emit layoutChanged();
    return nullptr;
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

void AutonomousVehicleProject::import(const QString& fname)
{
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


BackgroundRaster *AutonomousVehicleProject::getBackgroundRaster() const
{
    return m_currentBackground;
}

BackgroundRaster *AutonomousVehicleProject::getDepthRaster() const
{
    return m_currentDepthRaster;
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
    sa->setPos(sa->geoToPixel(position,this));
    sa->addWaypoint(position);
    connect(this,&AutonomousVehicleProject::updatingBackground,sa,&SurveyArea::updateBackground);
    return sa;
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
    tl->setPos(tl->geoToPixel(position,this));
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
    for(auto index: indices)
        deleteItem(index);
}

void AutonomousVehicleProject::deleteItem(const QModelIndex &index)
{
    MissionItem *item = itemFromIndex(index);
    GeoGraphicsMissionItem *ggi = qobject_cast<GeoGraphicsMissionItem*>(item);
    if(ggi)
    {
        GeoGraphicsItem *pggi = qgraphicsitem_cast<GeoGraphicsItem*>(ggi->parentItem());
        if(pggi)
            pggi->prepareGeometryChange();
        m_scene->removeItem(ggi);
    }
    BackgroundRaster *bgr = qobject_cast<BackgroundRaster*>(item);
    if(bgr)
    {
        m_scene->removeItem(bgr);
        if(m_currentBackground == bgr)
            setCurrentBackground(nullptr);
            //m_currentBackground = nullptr;
        if(m_currentDepthRaster == bgr)
            m_currentDepthRaster = nullptr;
    }
    QModelIndex p = parent(index);
    MissionItem * pi = itemFromIndex(p);
    int rownum = pi->childMissionItems().indexOf(item);
    beginRemoveRows(p,rownum,rownum);
    pi->removeChildMissionItem(item);
    delete item;
    endRemoveRows();
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

        BackgroundRaster *bgr = qobject_cast<BackgroundRaster*>(m_currentSelected);
        if(bgr)
            setCurrentBackground(bgr);
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

void AutonomousVehicleProject::setCurrentBackground(BackgroundRaster *bgr)
{
    emit aboutToUpdateBackground();
    if(m_currentBackground)
        m_scene->removeItem(m_currentBackground);
    m_currentBackground = bgr;
    if(bgr)
    {
        bgr->updateMapScale(m_map_scale);
        m_scene->addItem(bgr);
        if(bgr->depthValid())
            m_currentDepthRaster = bgr;
    }
    emit updatingBackground(bgr);
    emit backgroundUpdated(bgr);
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
        if(qobject_cast<BackgroundRaster*>(item))
            return QAbstractItemModel::flags(index);

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
    if(m_currentBackground)
        m_currentBackground->updateMapScale(scale);
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

