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
#include <gdal_priv.h>

#include <iostream>

AutonomousVehicleProject::AutonomousVehicleProject(QObject *parent) : QObject(parent)
{
    GDALAllRegister();

    m_model = new QStandardItemModel(this);
    m_scene = new QGraphicsScene(this);
    m_model->setObjectName("projectModel");
    QStandardItem *item = new QStandardItem("Background");
    m_model->setItem(0,item);
    topLevelItems["Background"] = item;
    item = new QStandardItem("Platform");
    m_model->setItem(1,item);
    topLevelItems["Platform"] = item;
    item = new QStandardItem("Mission");
    m_model->setItem(2,item);
    topLevelItems["Mission"] = item;
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

        QJsonArray bgArray;
        QStandardItem *bgitems = topLevelItems["Background"];
        int row = 0;
        QStandardItem *child = bgitems->child(row);
        while(child)
        {
            BackgroundRaster *bg = child->data().value<BackgroundRaster*>();
            QJsonObject bgObject;
            bg->write(bgObject);
            bgArray.append(bgObject);
            row++;
            child = bgitems->child(row);
        }
        projectObject["background"] = bgArray;

        QJsonArray missionArray;

        QStandardItem *missionitems = topLevelItems["Mission"];
        row = 0;
        child = missionitems->child(row);
        while(child)
        {
            TrackLine *tl = child->data().value<TrackLine*>();
            if(tl)
            {
                QJsonObject tlObject;
                tl->write(tlObject);
                missionArray.append(tlObject);
            }
            row++;
            child = missionitems->child(row);
        }
        projectObject["mission"] = missionArray;

        QFile saveFile(saveName);
        if(saveFile.open(QFile::WriteOnly))
        {
            QJsonDocument saveDoc(projectObject);
            saveFile.write(saveDoc.toJson());
            m_filename = saveName;
        }
    }

}

void AutonomousVehicleProject::openBackground(const QString &fname)
{
    BackgroundRaster *bgr = new BackgroundRaster(fname, this);
    QStandardItem *item = new QStandardItem(fname);
    item->setData(QVariant::fromValue<BackgroundRaster*>(bgr));
    topLevelItems["Background"]->appendRow(item);
    m_scene->addItem(bgr);
}

BackgroundRaster *AutonomousVehicleProject::getBackgroundRaster() const
{
    if(m_model)
    {
        QModelIndex bgindex = m_model->index(0,0,m_model->index(0,0));
        return m_model->data(bgindex,Qt::UserRole+1).value<BackgroundRaster*>();
    }
    return 0;
}

void AutonomousVehicleProject::addWaypoint(QGeoCoordinate position, BackgroundRaster *parentItem)
{
    Waypoint *wp = new Waypoint(this,parentItem);
    wp->setLocation(position);
    wp->setPos(parentItem->geoToPixel(position));
    QStandardItem *item = new QStandardItem("wayoint");
    item->setData(QVariant::fromValue<Waypoint*>(wp));
    topLevelItems["Mission"]->appendRow(item);
    wp->setFlag(QGraphicsItem::ItemIsMovable);
    wp->setFlag(QGraphicsItem::ItemIsSelectable);
    wp->setFlag(QGraphicsItem::ItemSendsGeometryChanges);
}

TrackLine * AutonomousVehicleProject::addTrackLine(QGeoCoordinate position, BackgroundRaster *parentItem)
{
    TrackLine *tl = new TrackLine(this,parentItem);
    tl->setPos(parentItem->geoToPixel(position));
    QStandardItem *item = new QStandardItem("trackline");
    item->setData(QVariant::fromValue<TrackLine*>(tl));
    topLevelItems["Mission"]->appendRow(item);
    tl->setFlag(QGraphicsItem::ItemIsMovable);
    tl->setFlag(QGraphicsItem::ItemIsSelectable);
    tl->setFlag(QGraphicsItem::ItemSendsGeometryChanges);

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

}
