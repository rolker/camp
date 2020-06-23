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
#include "platform.h"
#include "group.h"
#include <gdal_priv.h>
#include "vector/vectordataset.h"
#include "behavior.h"

#ifdef AMP_ROS
#include "roslink.h"
#endif

#include <iostream>
#include <sstream>

AutonomousVehicleProject::AutonomousVehicleProject(QObject *parent) : QAbstractItemModel(parent), m_currentBackground(nullptr), m_currentDepthRaster(nullptr), m_currentPlatform(nullptr), m_currentGroup(nullptr), m_currentSelected(nullptr), m_symbols(new QSvgRenderer(QString(":/symbols.svg"),this)), m_map_scale(1.0), unique_label_counter(0)
{
    GDALAllRegister();

    m_scene = new QGraphicsScene(this);
    m_root = new Group();
    m_root->setParent(this);
    m_currentGroup = m_root;
    setObjectName("projectModel");
    
#ifdef AMP_ROS
    m_ROSLink =  new ROSLink(this);
    connect(this,&AutonomousVehicleProject::backgroundUpdated,m_ROSLink ,&ROSLink::updateBackground);
    connect(this,&AutonomousVehicleProject::showRadar,m_ROSLink, &ROSLink::showRadar);
#endif
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
    QFile loadFile(fname);
    if(loadFile.open(QFile::ReadOnly))
    {
        QByteArray loadData = loadFile.readAll();
        QJsonDocument loadDoc(QJsonDocument::fromJson(loadData));
        m_root->read(loadDoc.object());
    }
}


void AutonomousVehicleProject::openBackground(const QString &fname)
{
    beginInsertRows(indexFromItem(m_currentGroup),m_currentGroup->childMissionItems().size(),m_currentGroup->childMissionItems().size());
    BackgroundRaster *bgr = new BackgroundRaster(fname, m_currentGroup);
    if(bgr->valid())
    {        
        bgr->setObjectName(fname);
        setCurrentBackground(bgr);
        endInsertRows();
    }
    else
    {
        endInsertRows();
        deleteItem(bgr);
    }
}

void AutonomousVehicleProject::openGeometry(const QString& fname)
{
    RowInserter ri(*this,m_currentGroup);
    VectorDataset * vd = new VectorDataset(m_currentGroup);
    vd->setObjectName(fname);
    vd->open(fname);
    connect(this,&AutonomousVehicleProject::backgroundUpdated,vd,&VectorDataset::updateProjectedPoints);
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


Platform * AutonomousVehicleProject::createPlatform()
{
    RowInserter ri(*this,m_currentGroup);
    Platform *p = new Platform(m_currentGroup);
    p->setObjectName("platform");
    m_currentPlatform = p;
    return p;
}

Behavior * AutonomousVehicleProject::createBehavior()
{
    Behavior *b = potentialParentItemFor("Behavior")->createMissionItem<Behavior>(generateUniqueLabel("behavior"));
    return b;
}


Group * AutonomousVehicleProject::addGroup()
{
    RowInserter ri(*this,m_currentGroup);
    Group *g = new Group(m_currentGroup);
    g->setObjectName("group");
    return g;
}

MissionItem *AutonomousVehicleProject::potentialParentItemFor(std::string const &childType)
{
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
    connect(this,&AutonomousVehicleProject::backgroundUpdated,wp,&Waypoint::updateBackground);
    return wp;
}


SurveyPattern * AutonomousVehicleProject::createSurveyPattern()
{
    SurveyPattern *sp = potentialParentItemFor("SurveyPattern")->createMissionItem<SurveyPattern>(generateUniqueLabel("pattern"));
    connect(this,&AutonomousVehicleProject::currentPlaformUpdated,sp,&GeoGraphicsMissionItem::onCurrentPlatformUpdated);
    connect(this,&AutonomousVehicleProject::backgroundUpdated,sp,&SurveyPattern::updateBackground);
  
    return sp;

}

SurveyPattern *AutonomousVehicleProject::addSurveyPattern(QGeoCoordinate position)
{
    SurveyPattern *sp = createSurveyPattern();
    sp->setStartLocation(position);
//    connect(this,&AutonomousVehicleProject::backgroundUpdated,sp,&SurveyPattern::updateBackground);
    return sp;
}

SurveyArea * AutonomousVehicleProject::createSurveyArea()
{
    SurveyArea *sa = potentialParentItemFor("SurveyArea")->createMissionItem<SurveyArea>(generateUniqueLabel("area"));
    connect(this,&AutonomousVehicleProject::currentPlaformUpdated,sa,&GeoGraphicsMissionItem::onCurrentPlatformUpdated);
    return sa;
}

SurveyArea * AutonomousVehicleProject::addSurveyArea(QGeoCoordinate position)
{
    SurveyArea *sa = createSurveyArea();
    sa->setPos(sa->geoToPixel(position,this));
    sa->addWaypoint(position);
    connect(this,&AutonomousVehicleProject::backgroundUpdated,sa,&SurveyArea::updateBackground);
    return sa;
}


TrackLine * AutonomousVehicleProject::createTrackLine()
{
    TrackLine *tl = potentialParentItemFor("TrackLine")->createMissionItem<TrackLine>(generateUniqueLabel("trackline"));
    return tl;
}


TrackLine * AutonomousVehicleProject::addTrackLine(QGeoCoordinate position)
{
    TrackLine *tl = createTrackLine();
    tl->setPos(tl->geoToPixel(position,this));
    tl->addWaypoint(position);
    connect(this,&AutonomousVehicleProject::backgroundUpdated,tl,&TrackLine::updateBackground);
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
                auto waypoints = tl->childItems();
                outstream << "LIN " << waypoints.size() << "\n";
                for(auto i: waypoints)
                {
                    const Waypoint *wp = qgraphicsitem_cast<Waypoint const*>(i);
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
    Platform *platform = currentPlatform();
    if(platform)
    {
        defaultParameters["defaultspeed_ms"] = platform->speed()*0.514444; // knots to m/s
    }
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
    if(m_currentPlatform)
    {
        QJsonObject platformObject;
        m_currentPlatform->write(platformObject);
        topArray.append(platformObject);
    }
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
    
#ifdef AMP_ROS
    if(m_ROSLink)
    {
        m_ROSLink->sendMissionPlan(plan.toJson());
    }
#endif

    GeoGraphicsMissionItem * gmi = qobject_cast<GeoGraphicsMissionItem*>(mi);
    if(gmi)
        gmi->lock();
}

void AutonomousVehicleProject::appendMission(const QModelIndex& index)
{
    QJsonDocument plan = generateMissionTask(index);
#ifdef AMP_ROS
    if(m_ROSLink)
    {
        m_ROSLink->appendMission(plan.toJson());
    }
#endif
}

void AutonomousVehicleProject::prependMission(const QModelIndex& index)
{
    QJsonDocument plan = generateMissionTask(index);
#ifdef AMP_ROS
    if(m_ROSLink)
    {
        m_ROSLink->prependMission(plan.toJson());
    }
#endif
}

void AutonomousVehicleProject::updateMission(const QModelIndex& index)
{
    QJsonDocument plan = generateMissionTask(index);
#ifdef AMP_ROS
    if(m_ROSLink)
    {
        m_ROSLink->updateMission(plan.toJson());
    }
#endif
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
            m_currentBackground = nullptr;
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
        Platform *p = qobject_cast<Platform*>(m_currentSelected);
        if(p && p != m_currentPlatform)
        {
            m_currentPlatform = p;
            connect(m_currentPlatform,&Platform::speedChanged,[=](){emit currentPlaformUpdated();});
            emit currentPlaformUpdated();
        }
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
    if(m_currentBackground)
        m_scene->removeItem(m_currentBackground);
    m_currentBackground = bgr;
    if(bgr)
    {
        bgr->updateMapScale(m_map_scale);
        m_scene->addItem(bgr);
        emit backgroundUpdated(bgr);
        if(bgr->depthValid())
            m_currentDepthRaster = bgr;
    }
}

Platform * AutonomousVehicleProject::currentPlatform() const
{
    return m_currentPlatform;
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
    if(item && item != m_root)
    {
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
    return nullptr;
}

int AutonomousVehicleProject::rowCount(const QModelIndex& parent) const
{
    MissionItem * item = m_root;
    if(parent.isValid())
        item = itemFromIndex(parent);
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
    if(child.isValid())
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
    if (index.isValid())
        return QAbstractItemModel::flags(index)|Qt::ItemIsDragEnabled|Qt::ItemIsDropEnabled;
    
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
    if(parentItem)
    {
        if(parentItem->childMissionItems().size() <= row+count)
        {
            beginRemoveRows(parent,row,row+count);
            for (int i = row+count-1; i >= row; --i)
                delete parentItem->childMissionItems()[i];
            endRemoveRows();
        }
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

bool AutonomousVehicleProject::dropMimeData(const QMimeData* data, Qt::DropAction action, int row, int column, const QModelIndex& parent)
{
    qDebug() << "dropMimeData: " << row << ", " << column;
    qDebug() << "mime encoded: " << data->data("application/json");
    
    QJsonDocument doc(QJsonDocument::fromJson(data->data("application/json")));
    
    MissionItem * parentItem = itemFromIndex(parent);
    if(!parentItem)
        parentItem = m_root;
    
    parentItem->readChildren(doc.array());
        
}

#ifdef AMP_ROS
ROSLink * AutonomousVehicleProject::rosLink() const
{
    return m_ROSLink;
}
#endif

void AutonomousVehicleProject::updateMapScale(qreal scale)
{
    if(m_currentBackground)
        m_currentBackground->updateMapScale(scale);
    m_map_scale = scale;
    
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

AutonomousVehicleProject::RowInserter::RowInserter(AutonomousVehicleProject& project, MissionItem* parent):m_project(project)
{
    project.beginInsertRows(project.indexFromItem(parent),parent->childMissionItems().size(),parent->childMissionItems().size());
}

AutonomousVehicleProject::RowInserter::~RowInserter()
{
    m_project.endInsertRows();
}

