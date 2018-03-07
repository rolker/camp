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

AutonomousVehicleProject::AutonomousVehicleProject(QObject *parent) : QAbstractItemModel(parent), m_currentBackground(nullptr), m_currentPlatform(nullptr), m_currentGroup(nullptr), m_symbols(new QSvgRenderer(QString(":/symbols.svg"),this))
{
    GDALAllRegister();

    m_scene = new QGraphicsScene(this);
    m_root = new Group(this);
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

        QJsonArray objArray;
        int row = 0;
        for(auto child: children())
        {
            MissionItem * childItem = qobject_cast<MissionItem*>(child);
            if(childItem)
            {
                QJsonObject miObject;
                childItem->write(miObject);
                objArray.append(miObject);
            }
        }
        
        projectObject["type"] = "Group";
        projectObject["name"] = "project";
        projectObject["children"] = objArray;


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
        QJsonArray childrenArray = loadDoc.object()["children"].toArray();
        for (int childIndex = 0; childIndex < childrenArray.size(); ++childIndex)
        {
            QJsonObject object = childrenArray[childIndex].toObject();
            if(object["type"] == "BackgroundRaster")
                openBackground(object["filename"].toString());
            MissionItem *item = nullptr;
            if(object["type"] == "Waypoint")
                item = createWaypoint();
            if(object["type"] == "TrackLine")
                item = createTrackLine();
            if(object["type"] == "SurveyPattern")
                item = createSurveyPattern();
            if(object["type"] == "Platform")
                item = createPlatform();
            if(item)
                item->read(object);
        }
    }
}

void AutonomousVehicleProject::openBackground(const QString &fname)
{
    BackgroundRaster *bgr = new BackgroundRaster(fname, this);
    bgr->setObjectName(fname);
    if(m_currentGroup)
        bgr->setParent(m_currentGroup);
    setCurrentBackground(bgr);
}

void AutonomousVehicleProject::openGeometry(const QString& fname)
{
    VectorDataset * vd = new VectorDataset(this);
    vd->setObjectName(fname);
    if(m_currentGroup)
        vd->setParent(m_currentGroup);
    vd->open(fname);
    connect(this,&AutonomousVehicleProject::backgroundUpdated,vd,&VectorDataset::updateProjectedPoints);
}


BackgroundRaster *AutonomousVehicleProject::getBackgroundRaster() const
{
    return m_currentBackground;
}

Platform * AutonomousVehicleProject::createPlatform()
{
    Platform *p = new Platform(this);
    p->setObjectName("platform");
    if(m_currentGroup)
        p->setParent(m_currentGroup);
    return p;
}

Group * AutonomousVehicleProject::addGroup()
{
    Group *g = new Group(this);
    g->setObjectName("group");
    if(m_currentGroup)
        g->setParent(m_currentGroup);
    return g;
}


#ifdef AMP_ROS
ROSNode * AutonomousVehicleProject::createROSNode()
{
    ROSNode *rn = new ROSNode(this,getBackgroundRaster());
    m_model->appendRow(rn->createItem("ROS"));
    connect(this,&AutonomousVehicleProject::backgroundUpdated,rn,&ROSNode::updateBackground);
    return rn;
}
#else
void AutonomousVehicleProject::createROSNode()
{
}
#endif

Waypoint * AutonomousVehicleProject::createWaypoint(BackgroundRaster *parentItem)
{
    if(!parentItem)
        parentItem = getBackgroundRaster();
    Waypoint *wp = new Waypoint(this,parentItem);
    wp->setObjectName("waypoint");
    if(m_currentGroup)
        wp->setParent(m_currentGroup);
    wp->setFlag(QGraphicsItem::ItemIsMovable);
    wp->setFlag(QGraphicsItem::ItemIsSelectable);
    wp->setFlag(QGraphicsItem::ItemSendsGeometryChanges);

    return wp;
}

void AutonomousVehicleProject::addWaypoint(QGeoCoordinate position, BackgroundRaster *parentItem)
{
    Waypoint *wp = createWaypoint(parentItem);
    wp->setLocation(position);
    wp->setPos(parentItem->geoToPixel(position));
    connect(this,&AutonomousVehicleProject::backgroundUpdated,wp,&Waypoint::updateBackground);
}

SurveyPattern * AutonomousVehicleProject::createSurveyPattern(BackgroundRaster *parentItem)
{
    if(!parentItem)
        parentItem = getBackgroundRaster();
    SurveyPattern *sp = new SurveyPattern(this,parentItem);
    sp->setObjectName("pattern");
    if(m_currentGroup)
        sp->setParent(m_currentGroup);
    sp->setFlag(QGraphicsItem::ItemIsMovable);
    sp->setFlag(QGraphicsItem::ItemIsSelectable);
    sp->setFlag(QGraphicsItem::ItemSendsGeometryChanges);
    connect(this,&AutonomousVehicleProject::currentPlaformUpdated,sp,&SurveyPattern::onCurrentPlatformUpdated);

    return sp;

}

SurveyPattern *AutonomousVehicleProject::addSurveyPattern(QGeoCoordinate position, BackgroundRaster *parentItem)
{
    SurveyPattern *sp = createSurveyPattern();
    sp->setPos(parentItem->geoToPixel(position));
    sp->setStartLocation(position);
    connect(this,&AutonomousVehicleProject::backgroundUpdated,sp,&SurveyPattern::updateBackground);
    return sp;
}

TrackLine * AutonomousVehicleProject::createTrackLine(BackgroundRaster *parentItem)
{
    if(!parentItem)
        parentItem = getBackgroundRaster();
    TrackLine *tl = new TrackLine(this,parentItem);
    tl->setObjectName("trackline");
    if(m_currentGroup)
        tl->setParent(m_currentGroup);
    tl->setFlag(QGraphicsItem::ItemIsMovable);
    tl->setFlag(QGraphicsItem::ItemIsSelectable);
    tl->setFlag(QGraphicsItem::ItemSendsGeometryChanges);
    return tl;
}


TrackLine * AutonomousVehicleProject::addTrackLine(QGeoCoordinate position, BackgroundRaster *parentItem)
{
    TrackLine *tl = createTrackLine(parentItem);
    tl->setPos(parentItem->geoToPixel(position));
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
        QVariant item = m_model->data(index,Qt::UserRole+1);
        MissionItem* mi = reinterpret_cast<MissionItem*>(item.value<quintptr>());
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
    removeRow(index.row(),index.parent());
}

void AutonomousVehicleProject::deleteItem(MissionItem *item)
{

}

void AutonomousVehicleProject::setCurrent(const QModelIndex &index)
{
    QVariant item = data(index,Qt::UserRole+1);
    MissionItem* mi = reinterpret_cast<MissionItem*>(item.value<quintptr>());
    QString itemType = mi->metaObject()->className();

    BackgroundRaster *bgr = qobject_cast<BackgroundRaster*>(mi);
    if(bgr)
        setCurrentBackground(bgr);
    Platform *p = qobject_cast<Platform*>(mi);
    if(p && p != m_currentPlatform)
    {
        m_currentPlatform = p;
        connect(m_currentPlatform,&Platform::speedChanged,[=](){emit currentPlaformUpdated();});
        emit currentPlaformUpdated();
    }
#ifdef AMP_ROS
    ROSNode *rn = qobject_cast<ROSNode*>(mi);
    if(rn)
        m_currentROSNode = rn;
#endif
    Group *g = qobject_cast<Group*>(mi);
    if(g)
        m_currentGroup = g;
    else
        m_currentGroup = nullptr;
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
    if(column>0)
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
    return QVariant();
}

Qt::ItemFlags AutonomousVehicleProject::flags(const QModelIndex& index) const
{
    if (!index.isValid())
        return 0;

    return QAbstractItemModel::flags(index);
}

QVariant AutonomousVehicleProject::headerData(int section, Qt::Orientation orientation, int role) const
{
    return QVariant();
}
