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

VectorDataset::VectorDataset(QObject* parent):MissionItem(parent)
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
        BackgroundRaster *bg = autonomousVehicleProject()->getBackgroundRaster();
        for(int i = 0; i < dataset->GetLayerCount(); ++i)
        {
            OGRLayer *layer = dataset->GetLayer(i);
            Group *group = new Group(this);
            group->setObjectName(layer->GetName());
            if(item())
                item()->appendRow(group->createItem(layer->GetName()));
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
                        Point *p = new Point(parent(),bg);
                        QGeoCoordinate location(op->getY(),op->getX());
                        p->setLocation(location);
                        group->item()->appendRow(p->createItem("point"));
                    }
                    else if(gtype == wkbLineString)
                    {
                        OGRLineString *ols = dynamic_cast<OGRLineString*>(geometry);
                        LineString *ls = new LineString(parent(),bg);
                        group->item()->appendRow(ls->createItem("lineString"));
                        OGRPointIterator *pi = ols->getPointIterator();
                        OGRPoint *p;
                        while(pi->getNextPoint(p))
                        {
                            QGeoCoordinate location(p->getY(),p->getX());
                            ls->addPoint(location);
                        }                        
                    }
                    else if(gtype == wkbPolygon)
                    {
                        OGRPolygon *op = dynamic_cast<OGRPolygon*>(geometry);
                        Polygon *p = new Polygon(parent(),bg);
                        group->item()->appendRow(p->createItem("polygon"));
                        OGRLinearRing *lr = op->getExteriorRing();
                        OGRPointIterator *pi = lr->getPointIterator();
                        OGRPoint *pt;
                        while(pi->getNextPoint(pt))
                        {
                            QGeoCoordinate location(pt->getY(),pt->getX());
                            p->addExteriorPoint(location);
                        }                        
                        for(int ringNum = 0; ringNum < op->getNumInteriorRings(); ringNum++)
                        {
                            p->addInteriorRing();
                            lr = op->getInteriorRing(ringNum);
                            pi = lr->getPointIterator();
                            while(pi->getNextPoint(pt))
                            {
                                QGeoCoordinate location(pt->getY(),pt->getX());
                                p->addInteriorPoint(location);
                            }                        
                        }
                        p->updateBBox();
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
