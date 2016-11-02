#ifndef GEOREFERENCED_H
#define GEOREFERENCED_H

class GDALDataset;

class Georeferenced
{
public:
    Georeferenced();
protected:
    void extractGeoreference(GDALDataset *dataset);
private:
    double geoTransform[6];
};

#endif // GEOREFERENCED_H
