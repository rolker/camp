#include "vectordataset.h"
#include <memory>
#include <gdal_priv.h>
#include "group.h"
#include "point.h"
#include "linestring.h"
#include "polygon.h"
#include "vector/vector_parse.h"
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
    // [camp#22 round-11 must-fix] EVERY VERTEX IS TESTED FOR PLACEABILITY BEFORE
    // IT BECOMES A MISSION ITEM.
    //
    // The parser is faithful (ADR-0016 D4) and admits a geometry on any ONE
    // placeable vertex, because the display consumer (VectorFeatureItem) filters
    // again per vertex. This consumer is the one that does not: the items built
    // below are EDITABLE — draggable in the mission tree, written into the mission
    // file and candidates for transmission to the robot — so a vertex that is not
    // a place on the earth must never reach one. The motivating input is a
    // shapefile shipped without its .prj sidecar, whose UTM northings are read as
    // degrees: one vertex happens to fall inside +/-90 and the geometry is
    // admitted, and the rest used to import as waypoints at latitude 4 800 000.
    //
    // Dropping vertices reshapes a line or a ring, which is why the PARSER does
    // not do it; here the alternative is a waypoint no vessel can be sent to. The
    // counts below are what keeps that honest: the operator is told what was left
    // out rather than shown a silently different shape. See
    // camp::vector::placeableGeometry(), which holds the rule so the display and
    // mission paths cannot drift apart.
    //
    // [round-12] That rule is camp::vector::isProjectable(), which also excludes
    // the POLES. A latitude of exactly +/-90 is a valid WGS84 coordinate, so
    // isPlaceable() admits it and the display path merely clamps it to the edge of
    // the Mercator world — but the items built below project through
    // GeoGraphicsItem::geoToPixel(), i.e. raw web_mercator::geoToMap(), which puts
    // a polar vertex ~2.4e8 m out: twelve world half-extents of empty scene, and an
    // editable waypoint no vessel can be sent to.
    int vertices_dropped = 0;
    int rings_dropped = 0;
    int geometries_dropped = 0;
    for(const auto& parsedLayer : layers)
    {
        Group *group = new Group(this);
        group->setObjectName(parsedLayer.name);
        for(const auto& parsedGeometry : parsedLayer.geometries)
        {
            const camp::vector::PlaceableGeometry geometry =
                camp::vector::placeableGeometry(parsedGeometry);
            vertices_dropped += geometry.vertices_dropped;
            rings_dropped += geometry.rings_dropped;
            if(!geometry.placeable)
            {
                // No exterior vertex survived: there is nothing to build, and an
                // item built from what is left would be positioned nowhere.
                ++geometries_dropped;
                continue;
            }
            switch(parsedGeometry.type)
            {
            case camp::vector::ParsedGeometry::Point:
            {
                Point *p = new Point(group);
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
    // One line per import, not one per vertex: a mis-projected file has these by
    // the million. This is the only channel VectorDataset has — it carries no
    // status of its own, unlike the read-only VectorLayer's Layers-tab line.
    if(vertices_dropped > 0 || rings_dropped > 0 || geometries_dropped > 0)
        qWarning() << "VectorDataset:" << m_filename << "- left out" << vertices_dropped
                   << "vertex(es)," << rings_dropped << "interior ring(s) and"
                   << geometries_dropped << "geometry(ies) whose coordinates are not a"
                   << "valid latitude/longitude. A file with no spatial reference (a"
                   << "shapefile missing its .prj sidecar is the usual case) reads its"
                   << "projected metres as degrees; the imported items hold only the"
                   << "vertices that are real positions.";
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
