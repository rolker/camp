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
#include <gdal_priv.h>
#include "vectordataset.h"

#ifdef AMP_ROS
#include "rosnode.h"
#endif

#include <iostream>

AutonomousVehicleProject::AutonomousVehicleProject(QObject *parent) : QObject(parent), m_currentBackground(nullptr), m_currentPlatform(nullptr),m_symbols(new QSvgRenderer(QString(":/symbols.svg"),this))
{
    GDALAllRegister();

    m_model = new QStandardItemModel(this);
    m_scene = new QGraphicsScene(this);
    m_model->setObjectName("projectModel");
}

AutonomousVehicleProject::~AutonomousVehicleProject()
{
}

QStandardItemModel *AutonomousVehicleProject::model() const
{
    return m_model;
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
        QStandardItem *child = m_model->item(row);
        while(child)
        {
            MissionItem* mi = reinterpret_cast<MissionItem*>(child->data().value<quintptr>());
            if(mi)
            {
                QJsonObject miObject;
                mi->write(miObject);
                objArray.append(miObject);
            }
            row++;
            child = m_model->item(row);
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
    m_model->appendRow(bgr->createItem(fname));
    setCurrentBackground(bgr);
}

void AutonomousVehicleProject::openGeometry(const QString& fname)
{
    VectorDataset * vd = new VectorDataset(this);
    m_model->appendRow(vd->createItem(fname));
    vd->open(fname);
    connect(this,&AutonomousVehicleProject::backgroundUpdated,vd,&VectorDataset::updateProjectedPoints);
}


BackgroundRaster *AutonomousVehicleProject::getBackgroundRaster() const
{
    return m_currentBackground;
    if(m_model)
    {
        int row = 0;
        QStandardItem *item = m_model->item(row);
        while(item)
        {
            BackgroundRaster *bgr = item->data().value<BackgroundRaster*>();
            if(bgr)
                return bgr;
            row += 1;
            item = m_model->item(row);
        }

    }
    return 0;
}

Platform * AutonomousVehicleProject::createPlatform()
{
    Platform *p = new Platform(this);
    m_model->appendRow(p->createItem("platform"));
    return p;
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
    m_model->appendRow(wp->createItem("waypoint"));
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
    m_model->appendRow(sp->createItem("pattern"));
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
    m_model->appendRow(tl->createItem("trackline"));
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
    QStandardItem *item = m_model->itemFromIndex(index);
    TrackLine *tl = item->data().value<TrackLine*>();
    if(tl)
    {
        QString fname = QFileDialog::getSaveFileName(qobject_cast<QWidget*>(parent()));
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
    SurveyPattern *sp = item->data().value<SurveyPattern*>();
    if(sp)
    {
        QString fname = QFileDialog::getSaveFileName(qobject_cast<QWidget*>(parent()));
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

void AutonomousVehicleProject::deleteItems(const QModelIndexList &indices)
{
    for(auto index: indices)
        deleteItem(index);
}

void AutonomousVehicleProject::deleteItem(const QModelIndex &index)
{
    QStandardItem *item = m_model->itemFromIndex(index);
    GeoGraphicsItem *ggi = item->data().value<GeoGraphicsItem*>();
    if(ggi)
    {
        GeoGraphicsItem *pggi = qgraphicsitem_cast<GeoGraphicsItem*>(ggi->parentItem());
        if(pggi)
            pggi->prepareGeometryChange();
        m_scene->removeItem(ggi);
    }
    BackgroundRaster *bgr = item->data().value<BackgroundRaster*>();
    if(bgr)
    {
        m_scene->removeItem(bgr);
        if(m_currentBackground == bgr)
            m_currentBackground = nullptr;
    }
    m_model->removeRow(index.row(),index.parent());
}

void AutonomousVehicleProject::deleteItem(QStandardItem *item)
{
    deleteItem(m_model->indexFromItem(item));
}

void AutonomousVehicleProject::setCurrent(const QModelIndex &index)
{
    QVariant item = m_model->data(index,Qt::UserRole+1);
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
