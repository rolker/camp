#include "georeferenced.h"

#include <gdal_priv.h>

Georeferenced::Georeferenced(): geoTransform{0.0,1.0,0.0,0.0,0.0,1.0}
{

}

void Georeferenced::extractGeoreference(GDALDataset *dataset)
{
    dataset->GetGeoTransform(geoTransform);
}
