#include "vectordataset.h"
#include <gdal_priv.h>
#include "group.h"
#include "point.h"
#include "linestring.h"
#include "backgroundraster.h"
#include "polygon.h"
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

    GDALDataset * dataset = reinterpret_cast<GDALDataset*>(GDALOpenEx(m_filename.toStdString().c_str(),GDAL_OF_READONLY,nullptr,nullptr,nullptr));
    if (dataset)
    {
        extractGeoreference(dataset);
        for(int i = 0; i < dataset->GetLayerCount(); ++i)
        {
            OGRLayer *layer = dataset->GetLayer(i);
            OGRCoordinateTransformation *unprojectTransformation = nullptr;
            OGRSpatialReference *projected = layer->GetSpatialRef();
            if(projected)
            {
                OGRSpatialReference wgs84;
                wgs84.SetWellKnownGeogCS("WGS84");
                unprojectTransformation = OGRCreateCoordinateTransformation(projected,&wgs84);
            }

            Group *group = new Group(this);
            group->setObjectName(layer->GetName());
            layer->ResetReading();
            OGRFeature * feature = layer->GetNextFeature();
            while(feature)
            {
                OGRGeometry * geometry = feature->GetGeometryRef();
                if(geometry)
                {
                    OGRwkbGeometryType gtype = geometry->getGeometryType();
                    if(gtype == wkbPoint)
                    {
                        OGRPoint *op = dynamic_cast<OGRPoint*>(geometry);
                        Point *p = new Point(group);
                        QGeoCoordinate location(op->getY(),op->getX());
                        if(unprojectTransformation)
                        {
                            double x = op->getX();
                            double y = op->getY();
                            unprojectTransformation->Transform(1,&x,&y);
                            location.setLatitude(y);
                            location.setLongitude(x);
                        }
                        p->setLocation(location);
                        p->setObjectName("point");
                    }
                    else if(gtype == wkbLineString)
                    {
                        OGRLineString *ols = dynamic_cast<OGRLineString*>(geometry);
                        LineString *ls = new LineString(group);
                        ls->setObjectName("lineString");
                        OGRPointIterator *pi = ols->getPointIterator();
                        OGRPoint p;
                        while(pi->getNextPoint(&p))
                        {
                            if(unprojectTransformation)
                            {
                                double x = p.getX();
                                double y = p.getY();
                                unprojectTransformation->Transform(1,&x,&y);
                                p.setX(x);
                                p.setY(y);
                            }
                            QGeoCoordinate location(p.getY(),p.getX());
                            ls->addPoint(location);
                        }                        
                    }
                    else if(gtype == wkbPolygon)
                    {
                        OGRPolygon *op = dynamic_cast<OGRPolygon*>(geometry);
                        OGRLinearRing *lr = op->getExteriorRing();
                        if(lr)
                        {
                            Polygon *p = new Polygon(group);
                            p->setObjectName("polygon");
                            qDebug() << "polygon exterior ring point count " << lr->getNumPoints();
                            OGRPointIterator *pi = lr->getPointIterator();
                            OGRPoint pt;
                            while(pi->getNextPoint(&pt))
                            {
                                if(unprojectTransformation)
                                {
                                    double x = pt.getX();
                                    double y = pt.getY();
                                    unprojectTransformation->Transform(1,&x,&y);
                                    pt.setX(x);
                                    pt.setY(y);
                                }
                                QGeoCoordinate location(pt.getY(),pt.getX());
                                p->addExteriorPoint(location);
                            }                        
                            for(int ringNum = 0; ringNum < op->getNumInteriorRings(); ringNum++)
                            {
                                p->addInteriorRing();
                                lr = op->getInteriorRing(ringNum);
                                pi = lr->getPointIterator();
                                while(pi->getNextPoint(&pt))
                                {
                                    if(unprojectTransformation)
                                    {
                                        double x = pt.getX();
                                        double y = pt.getY();
                                        unprojectTransformation->Transform(1,&x,&y);
                                        pt.setX(x);
                                        pt.setY(y);
                                    }
                                    QGeoCoordinate location(pt.getY(),pt.getX());
                                    p->addInteriorPoint(location);
                                }                        
                            }
                            p->updateBBox();
                            connect(autonomousVehicleProject(),&AutonomousVehicleProject::backgroundUpdated,p,&Polygon::updateBackground);
                        }
                    }
                    else
                        qDebug() << "type: " << gtype;
                }
                OGRFeature::DestroyFeature(feature);
                feature = layer->GetNextFeature();
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
