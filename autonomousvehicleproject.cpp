#include "autonomousvehicleproject.h"

#include <QStandardItemModel>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QFileDialog>
#include <QTextStream>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>

#include "backgroundraster.h"
#include "waypoint.h"
#include "trackline.h"
#include "surveypattern.h"
#include "platform.h"
#include <gdal_priv.h>

#include <iostream>

AutonomousVehicleProject::AutonomousVehicleProject(QObject *parent) : QObject(parent), m_currentBackground(nullptr), m_currentPlatform(nullptr)
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
            MissionItem *mi = child->data().value<MissionItem*>();
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
    QStandardItem *item = new QStandardItem(fname);
    item->setData(QVariant::fromValue<BackgroundRaster*>(bgr));
    m_model->appendRow(item);
    setCurrentBackground(bgr);
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
    QStandardItem *item = new QStandardItem("platform");
    item->setData(QVariant::fromValue<Platform*>(p));
    m_model->appendRow(item);
    p->setItem(item);
    return p;
}

Waypoint * AutonomousVehicleProject::createWaypoint(BackgroundRaster *parentItem)
{
    if(!parentItem)
        parentItem = getBackgroundRaster();
    Waypoint *wp = new Waypoint(this,parentItem);

    QStandardItem *item = new QStandardItem("wayoint");
    item->setData(QVariant::fromValue<Waypoint*>(wp));
    m_model->appendRow(item);
    wp->setItem(item);
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
}

SurveyPattern * AutonomousVehicleProject::createSurveyPattern(BackgroundRaster *parentItem)
{
    if(!parentItem)
        parentItem = getBackgroundRaster();
    SurveyPattern *sp = new SurveyPattern(this,parentItem);
    QStandardItem *item = new QStandardItem("pattern");
    item->setData(QVariant::fromValue<SurveyPattern*>(sp));
    sp->setItem(item);
    m_model->appendRow(item);
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
    return sp;
}

TrackLine * AutonomousVehicleProject::createTrackLine(BackgroundRaster *parentItem)
{
    if(!parentItem)
        parentItem = getBackgroundRaster();
    TrackLine *tl = new TrackLine(this,parentItem);
    QStandardItem *item = new QStandardItem("trackline");
    item->setData(QVariant::fromValue<TrackLine*>(tl));
    m_model->appendRow(item);
    tl->setItem(item);
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
                QList<QGeoCoordinate> wpList = sp->getPath();
                int lineCount = wpList.size()/2;
                if(arcCount > 2)
                {
                    lineCount = 1 + wpList.size()/(arcCount+1);
                }

                QTextStream outstream(&outfile);
                outstream.setRealNumberPrecision(8);
                if(arcCount > 2)
                    outstream << "LNS " << lineCount*2-1 << "\n";
                else
                    outstream << "LNS " << lineCount << "\n";
                for(int i = 0; i < lineCount; i++)
                {
                    if(arcCount > 2)
                    {
                        outstream << "LIN 2\n";
                        outstream << "PTS " << wpList[i*(arcCount+1)].latitude() << " " << wpList[i*(arcCount+1)].longitude() << "\n";
                        outstream << "PTS " << wpList[i*(arcCount+1)+1].latitude() << " " << wpList[i*(arcCount+1)+1].longitude() << "\n";
                        outstream << "LNN " << i << "\n";
                        outstream << "EOL\n";
                        if (i < lineCount-1)
                        {
                            outstream << "LIN " << arcCount-1 << "\n";
                            for(int j = 0; j < arcCount-1; j++)
                            {
                                outstream << "PTS " << wpList[i*(arcCount+1)+j+2].latitude() << " " << wpList[i*(arcCount+1)+j+2].longitude() << "\n";
                            }
                            outstream << "LNN ARC" << i << "\n";
                            outstream << "EOL\n";
                        }
                    }
                    else
                    {
                        outstream << "LIN 2\n";
                        outstream << "PTS " << wpList[i*2].latitude() << " " << wpList[i*2].longitude() << "\n";
                        outstream << "PTS " << wpList[i*2+1].latitude() << " " << wpList[i*2+1].longitude() << "\n";
                        outstream << "LNN " << i << "\n";
                        outstream << "EOL\n";
                    }
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
    QStandardItem *item = m_model->itemFromIndex(index);
    BackgroundRaster *bgr = item->data().value<BackgroundRaster*>();
    if(bgr)
        setCurrentBackground(bgr);
    Platform *p = item->data().value<Platform*>();
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
        for(int i = 0; i < m_model->rowCount(); ++i)
        {
            QStandardItem *item = m_model->item(i);
            GeoGraphicsItem *ggi = item->data().value<GeoGraphicsItem*>();
            if(ggi)
            {
                ggi->setParentItem(bgr);
                ggi->updateProjectedPoints();
            }
        }
    }
}

Platform * AutonomousVehicleProject::currentPlatform() const
{
    return m_currentPlatform;
}
