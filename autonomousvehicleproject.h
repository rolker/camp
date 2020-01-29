#ifndef AUTONOMOUSVEHICLEPROJECT_H
#define AUTONOMOUSVEHICLEPROJECT_H

#include <QAbstractItemModel>
#include <QGeoCoordinate>
#include <QModelIndex>


class QGraphicsScene;
class QGraphicsItem;
class QStandardItem;
class QLabel;
class QStatusBar;
class MissionItem;
class BackgroundRaster;
class Waypoint;
class TrackLine;
class SurveyPattern;
class SurveyArea;
class Platform;
class Group;
class QSvgRenderer;
#ifdef AMP_ROS
class ROSLink;
#endif
class Behavior;

class AutonomousVehicleProject : public QAbstractItemModel
{
    Q_OBJECT
public:
    explicit AutonomousVehicleProject(QObject *parent = 0);
    ~AutonomousVehicleProject();

    QGraphicsScene *scene() const;
    void openBackground(QString const &fname);
    BackgroundRaster * getBackgroundRaster() const;
    BackgroundRaster * getDepthRaster() const;
    MissionItem *potentialParentItemFor(std::string const &childType);

    Waypoint *addWaypoint(QGeoCoordinate position);

    SurveyPattern * createSurveyPattern();
    SurveyPattern * addSurveyPattern(QGeoCoordinate position);
    
    SurveyArea * createSurveyArea();
    SurveyArea * addSurveyArea(QGeoCoordinate position);

    TrackLine * createTrackLine();
    TrackLine * addTrackLine(QGeoCoordinate position);

    Platform * createPlatform();
    Platform * currentPlatform() const;
    
    Behavior * createBehavior();
    
    Group * addGroup();
    
    MissionItem *itemFromIndex(QModelIndex const &index) const;

    Qt::ItemFlags flags(const QModelIndex & index) const override;
    QVariant data(const QModelIndex & index, int role) const override;
    QVariant headerData(int section, Qt::Orientation orientation, int role) const override;
    int rowCount(const QModelIndex & parent) const override;
    int columnCount(const QModelIndex & parent) const override;
    
    QModelIndex index(int row, int column, const QModelIndex & parent) const override;
    QModelIndex parent(const QModelIndex & child) const override;
    QModelIndex indexFromItem(MissionItem * item) const;
    
    Qt::DropActions supportedDropActions() const override;
    
    bool removeRows(int row, int count, const QModelIndex & parent) override;
    
    QStringList mimeTypes() const override;
    QMimeData * mimeData(const QModelIndexList & indexes) const override;
    bool dropMimeData(const QMimeData * data, Qt::DropAction action, int row, int column, const QModelIndex & parent) override;

    QString const &filename() const;
    void save(QString const &fname = QString());
    void open(QString const &fname);
    
    void openGeometry(QString const &fname);
    
    void import(QString const &fname);

    void setCurrent(const QModelIndex &index);
    MissionItem *currentSelected() const;
    
    QSvgRenderer * symbols() const;
    
    
#ifdef AMP_ROS
    ROSLink * rosLink() const;
#endif

    QJsonDocument generateMissionPlan(QModelIndex const &index);
    QJsonDocument generateMissionTask(QModelIndex const &index);
    
signals:
    void currentPlaformUpdated();
    void backgroundUpdated(BackgroundRaster *bg);
    void showRadar(bool show);

public slots:

    void exportHypack(QModelIndex const &index);
    void exportMissionPlan(QModelIndex const &index);

    void sendToROS(QModelIndex const &index);
    void appendMission(QModelIndex const &index);
    void prependMission(QModelIndex const &index);
    void updateMission(QModelIndex const &index);
    
    void deleteItems(QModelIndexList const &indices);
    void deleteItem(QModelIndex const &index);
    void deleteItem(MissionItem *item);
    void updateMapScale(qreal scale);


private:
    QGraphicsScene* m_scene;
    QString m_filename;
    BackgroundRaster* m_currentBackground;
    BackgroundRaster* m_currentDepthRaster;
    Platform* m_currentPlatform;
    Group* m_currentGroup;
    Group* m_root;
    MissionItem * m_currentSelected;
#ifdef AMP_ROS
    ROSLink* m_ROSLink;
#endif
    
    QSvgRenderer* m_symbols;
    

    void setCurrentBackground(BackgroundRaster *bgr);
    QString generateUniqueLabel(std::string const &prefix);

    
public:
    
    class RowInserter
    {
    public:
        RowInserter(AutonomousVehicleProject &project, MissionItem *parent);
        
        ~RowInserter();
    private:
        AutonomousVehicleProject &m_project;
    };

private:    
    friend class RowInserter;
    
    qreal m_map_scale;
    
    // Counter to generate unique labels. Should probably be static, but if only once instance of AutonomousVehicleProject, then doesn't matter.
    int unique_label_counter; 
};



#endif // AUTONOMOUSVEHICLEPROJECT_H
