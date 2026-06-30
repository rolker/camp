#include "vectordataset.h"
#include <memory>
#include <gdal_priv.h>
#include "group.h"
#include "point.h"
#include "linestring.h"
#include "polygon.h"
#include "vector_parse.h"
#include "autonomousvehicleproject.h"
#include <ogrsf_frmts.h>
#include <QDebug>
#include <QStandardItem>
#include <QJsonObject>

VectorDataset::VectorDataset(MissionItem* parent):Group(parent)
{
}

void VectorDataset::open(const QString& fname)
{
    if(!fname.isEmpty())
        m_filename = fname;

    // [#152] RAII-close the dataset on every return path. Previously the handle
    // from GDALOpenEx was never GDALClose'd, leaking it on each chart load.
    const auto gdal_closer = [](GDALDataset* d){ if(d) GDALClose(d); };
    std::unique_ptr<GDALDataset, decltype(gdal_closer)> dataset(
        reinterpret_cast<GDALDataset*>(GDALOpenEx(m_filename.toStdString().c_str(), GDAL_OF_READONLY, nullptr, nullptr, nullptr)),
        gdal_closer);
    if(!dataset)
        return;

    extractGeoreference(dataset.get());
    buildItems(camp::vector::parseVectorLayers(dataset.get()));
}

void VectorDataset::buildItems(const std::vector<camp::vector::ParsedLayer>& layers)
{
    for(const auto& parsedLayer : layers)
    {
        Group *group = new Group(this);
        group->setObjectName(parsedLayer.name);
        for(const auto& geometry : parsedLayer.geometries)
        {
            switch(geometry.type)
            {
            case camp::vector::ParsedGeometry::Point:
            {
                Point *p = new Point(group);
                if(!geometry.exterior.empty())
                    p->setLocation(geometry.exterior.front());
                p->setObjectName("point");
                p->lock();
                connect(autonomousVehicleProject(),&AutonomousVehicleProject::updatingBackground,p,&Point::updateBackground);
                break;
            }
            case camp::vector::ParsedGeometry::LineString:
            {
                LineString *ls = new LineString(group);
                ls->setObjectName("lineString");
                for(const auto& location : geometry.exterior)
                    ls->addPoint(location);
                ls->lock();
                connect(autonomousVehicleProject(),&AutonomousVehicleProject::updatingBackground, ls, &LineString::updateBackground);
                break;
            }
            case camp::vector::ParsedGeometry::Polygon:
            {
                Polygon *p = new Polygon(group);
                p->setObjectName("polygon");
                for(const auto& location : geometry.exterior)
                    p->addExteriorPoint(location);
                for(const auto& ring : geometry.interiorRings)
                {
                    p->addInteriorRing();
                    for(const auto& location : ring)
                        p->addInteriorPoint(location);
                }
                p->updateBBox();
                p->lock();
                connect(autonomousVehicleProject(),&AutonomousVehicleProject::updatingBackground,p,&Polygon::updateBackground);
                break;
            }
            }
        }
    }
}

void VectorDataset::write(QJsonObject& json) const
{
    json["type"] = "VectorDataset";
    json["filename"] = m_filename;
}

void VectorDataset::read(const QJsonObject& json)
{

}

void VectorDataset::updateProjectedPoints()
{
    for(auto child: children())
    {
        MissionItem * childItem = qobject_cast<MissionItem*>(child);
        if(childItem)
            childItem->updateProjectedPoints();
    }
}

bool VectorDataset::canBeSentToRobot() const
{
    return false;
}
