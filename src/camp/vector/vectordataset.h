#ifndef VECTORDATASET_H
#define VECTORDATASET_H

#include <vector>

#include "../georeferenced.h"
#include "../group.h"

namespace camp::vector { struct ParsedLayer; }

class VectorDataset :public Group, public Georeferenced
{
    Q_OBJECT
    
public:
    VectorDataset(MissionItem *parent = 0);
    
    void write(QJsonObject &json) const;
    void read(const QJsonObject &json);
    
    void open(const QString &fname);
    bool canBeSentToRobot() const override;
    
public slots:
    void updateProjectedPoints() override;
    
private:
    // Build the project item graph (Group/Point/LineString/Polygon + the
    // updatingBackground connections) from already-parsed WGS84 geometry. The
    // GDAL/OGR resource lifecycle lives in camp::vector::parseVectorLayers so it
    // can be unit-tested without this MissionItem coupling (issue #152).
    void buildItems(const std::vector<camp::vector::ParsedLayer> &layers);

    QString m_filename;
};

#endif // VECTORDATASET_H
