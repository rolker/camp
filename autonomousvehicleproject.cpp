#include "autonomousvehicleproject.h"
#include "backgroundraster.h"
#include <QStandardItemModel>
#include <gdal_priv.h>


AutonomousVehicleProject::AutonomousVehicleProject(QObject *parent) : QObject(parent)
{
    GDALAllRegister();

    model = new QStandardItemModel(this);
    model->setObjectName("projectModel");
    QStandardItem *item = new QStandardItem("Background");
    model->setItem(0,item);
    topLevelItems["Background"] = item;
    item = new QStandardItem("Platform");
    model->setItem(1,item);
    topLevelItems["Platform"] = item;
    item = new QStandardItem("Mission");
    model->setItem(2,item);
    topLevelItems["Mission"] = item;
}

AutonomousVehicleProject::~AutonomousVehicleProject()
{
}

QStandardItemModel *AutonomousVehicleProject::getModel() const
{
    return findChild<QStandardItemModel*>("projectModel");
}

void AutonomousVehicleProject::openBackground(const QString &fname)
{
    BackgroundRaster *bgr = new BackgroundRaster(this,fname);
    QStandardItem *item = new QStandardItem(fname);
    item->setData(QVariant::fromValue<BackgroundRaster*>(bgr));
    topLevelItems["Background"]->appendRow(item);
}
