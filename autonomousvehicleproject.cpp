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
#include "platform.h"
#include "group.h"
#include <gdal_priv.h>
#include "vectordataset.h"

#ifdef AMP_ROS
#include "rosnode.h"
#endif

#include <iostream>

AutonomousVehicleProject::AutonomousVehicleProject(QObject *parent) : QAbstractItemModel(parent), m_currentBackground(nullptr), m_currentPlatform(nullptr), m_currentGroup(nullptr), m_currentSelected(nullptr), m_symbols(new QSvgRenderer(QString(":/symbols.svg"),this))
{
    GDALAllRegister();

    m_scene = new QGraphicsScene(this);
    m_root = new Group();
    m_root->setParent(this);
    m_currentGroup = m_root;
    setObjectName("projectModel");
    
#ifdef AMP_ROS
    m_currentROSNode = nullptr;
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
    bgr->setObjectName(fname);
    setCurrentBackground(bgr);
    endInsertRows();
}

void AutonomousVehicleProject::openGeometry(const QString& fname)
{
    RowInserter ri(*this,m_currentGroup);
    VectorDataset * vd = new VectorDataset(m_currentGroup);
    vd->setObjectName(fname);
    vd->open(fname);
    connect(this,&AutonomousVehicleProject::backgroundUpdated,vd,&VectorDataset::updateProjectedPoints);
}


BackgroundRaster *AutonomousVehicleProject::getBackgroundRaster() const
{
    return m_currentBackground;
}

Platform * AutonomousVehicleProject::createPlatform()
{
    RowInserter ri(*this,m_currentGroup);
    Platform *p = new Platform(m_currentGroup);
    p->setObjectName("platform");
    return p;
}

Group * AutonomousVehicleProject::addGroup()
{
    RowInserter ri(*this,m_currentGroup);
    Group *g = new Group(m_currentGroup);
    g->setObjectName("group");
    return g;
}


#ifdef AMP_ROS
ROSNode * AutonomousVehicleProject::createROSNode()
{
    ROSNode *rn = m_root->createMissionItem<ROSNode>("ROS");
    m_currentROSNode = rn;
    connect(this,&AutonomousVehicleProject::backgroundUpdated,rn,&ROSNode::updateBackground);
    return rn;
}
#else
void AutonomousVehicleProject::createROSNode()
{
}
#endif

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
    
    Waypoint *wp = potentialParentItemFor("Waypoint")->createMissionItem<Waypoint>("waypoint");
    wp->setLocation(position);
    connect(this,&AutonomousVehicleProject::backgroundUpdated,wp,&Waypoint::updateBackground);
    return wp;
}


SurveyPattern * AutonomousVehicleProject::createSurveyPattern()
{
    SurveyPattern *sp = potentialParentItemFor("SurveyPattern")->createMissionItem<SurveyPattern>("pattern");
    connect(this,&AutonomousVehicleProject::currentPlaformUpdated,sp,&SurveyPattern::onCurrentPlatformUpdated);

    return sp;

}

SurveyPattern *AutonomousVehicleProject::addSurveyPattern(QGeoCoordinate position)
{
    SurveyPattern *sp = createSurveyPattern();
    sp->setStartLocation(position);
    connect(this,&AutonomousVehicleProject::backgroundUpdated,sp,&SurveyPattern::updateBackground);
    return sp;
}

TrackLine * AutonomousVehicleProject::createTrackLine()
{
    TrackLine *tl = potentialParentItemFor("TrackLine")->createMissionItem<TrackLine>("trackline");
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
                int arcCount = sp->arcCount();
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

void AutonomousVehicleProject::sendToROS(const QModelIndex& index)
{
#ifdef AMP_ROS
    if(m_currentROSNode)
    {
        MissionItem *mi = itemFromIndex(index);
        //QVariant item = m_model->data(index,Qt::UserRole+1);
        //MissionItem* mi = reinterpret_cast<MissionItem*>(item.value<quintptr>());
        QString itemType = mi->metaObject()->className();
        if (itemType == "TrackLine")
        {
            TrackLine *tl = qobject_cast<TrackLine*>(mi);
            QList<QGeoCoordinate> wps;
            auto waypoints = tl->waypoints();
            for(auto i: waypoints)
            {
                const Waypoint *wp = qgraphicsitem_cast<Waypoint const*>(i);
                if(wp)
                {
                    auto ll = wp->location();
                    wps.append(ll);
                }
            }
            m_currentROSNode->sendWaypoints(wps);
        }
        if (itemType == "SurveyPattern")
        {
            SurveyPattern *sp = qobject_cast<SurveyPattern*>(mi);
            QList<QGeoCoordinate> wps;
            auto lines = sp->getLines();
            for (auto l: lines)
                for (auto p: l)
                    wps.append(p);
            m_currentROSNode->sendWaypoints(wps);
        }
        if (itemType == "Waypoint")
        {
            Waypoint *wp = qobject_cast<Waypoint*>(mi);
            QList<QGeoCoordinate> wps;
            wps.append(wp->location());
            m_currentROSNode->sendWaypoints(wps);
        }
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
#ifdef AMP_ROS
        ROSNode *rn = qobject_cast<ROSNode*>(m_currentSelected);
        if(rn)
            m_currentROSNode = rn;
#endif
        Group *g = qobject_cast<Group*>(m_currentSelected);
        if(g)
            m_currentGroup = g;
        else
            m_currentGroup = m_root;
    }
}

void AutonomousVehicleProject::setCurrentBackground(BackgroundRaster *bgr)
{
    if(m_currentBackground)
        m_scene->removeItem(m_currentBackground);
    m_currentBackground = bgr;
    if(bgr)
    {
        m_scene->addItem(bgr);
        emit backgroundUpdated(bgr);
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


AutonomousVehicleProject::RowInserter::RowInserter(AutonomousVehicleProject& project, MissionItem* parent):m_project(project)
{
    project.beginInsertRows(project.indexFromItem(parent),parent->childMissionItems().size(),parent->childMissionItems().size());
}

AutonomousVehicleProject::RowInserter::~RowInserter()
{
    m_project.endInsertRows();
}
