#ifndef AUTONOMOUSVEHICLEPROJECT_H
#define AUTONOMOUSVEHICLEPROJECT_H

#include <QAbstractItemModel>
#include <QGeoCoordinate>
#include <QModelIndex>
#include <QRectF>

#include <vector>

class QGraphicsScene;
class QGraphicsItem;
class QStandardItem;
class QLabel;
class QStatusBar;
class MissionItem;
class DepthRaster;
class Waypoint;
class TrackLine;
class SurveyPattern;
class SurveyArea;
class SearchPattern;
class Group;
class Orbit;
class QSvgRenderer;
//class ROSLink;
class Behavior;
class Platform;
class AvoidArea;

namespace camp { namespace map { class Map; } }
namespace camp { namespace raster { class RasterLayer; } }

class AutonomousVehicleProject : public QAbstractItemModel
{
    Q_OBJECT
public:
    explicit AutonomousVehicleProject(QObject *parent = 0);
    ~AutonomousVehicleProject();

    QGraphicsScene *scene() const;

    // [#59 PR3a] The Web-Mercator scene is owned by camp::map::Map (ADR-0002);
    // map() exposes the layer model so chart rasters can be added as layers.
    camp::map::Map *map() const;

    // [#59 ADR-0003] Load a chart as a stacked, Map-owned RasterLayer (display)
    // plus a depth provider — NOT a mission-tree BackgroundRaster node. Multiple
    // charts stack. Persists the loaded-chart list as app state (see
    // restorePersistedBackgrounds), so charts survive across sessions
    // independently of any mission file.
    void openBackground(QString const &fname, QString label = "");

    // [#59 ADR-0003] Re-create the persisted chart layers (app state). Call once
    // after the MainWindow has wired up the background signals, so fit-to-extent
    // and overlay refresh fire for the restored charts.
    void restorePersistedBackgrounds();

    // [#59 ADR-0003] True while at least one chart layer is loaded.
    bool hasBackground() const;

    // [#59 PR6] Persistent scene-origin anchor (camp::map::Map's root item),
    // always present in the scene regardless of whether a chart is loaded.
    // Top-level mission items / overlays parent to this instead of the
    // (possibly-null) BackgroundRaster, so they survive OSM/WMTS-only operation.
    QGraphicsItem * originAnchor() const;

    // [#59 ADR-0003] The displayed chart's extent in Web-Mercator scene space,
    // read from the RasterLayer (which knows its extent synchronously — ADR-0003
    // stage 1). Empty when no chart is displayed. Used by ProjectView to fit the
    // view to a newly-loaded chart, replacing the BackgroundRaster georeference.
    QRectF currentBackgroundExtent() const;

    // [#59 PR3c] Depth query over the depth-provider list (first valid wins),
    // independent of the scene projection. Returns NaN where no provider has
    // data. hasDepth() gates depth-aware planning. See ADR-0002.
    float getDepth(QGeoCoordinate const &location) const;
    bool hasDepth() const;
    MissionItem *potentialParentItemFor(std::string const &childType);

    Waypoint *addWaypoint(QGeoCoordinate position);

    SurveyPattern * createSurveyPattern(MissionItem* parent=nullptr, int row=-1, QString label = "");
    SurveyPattern * addSurveyPattern(QGeoCoordinate position);
    
    SurveyArea * createSurveyArea(MissionItem* parent=nullptr, int row=-1, QString label = "");
    SurveyArea * addSurveyArea(QGeoCoordinate position);

    AvoidArea * createAvoidArea(MissionItem* parent=nullptr, int row=-1, QString label = "");
    AvoidArea * addAvoidArea(QGeoCoordinate position);

    SearchPattern * createSearchPattern(MissionItem* parent=nullptr, int row=-1, QString label = "");
    SearchPattern * addSearchPattern(QGeoCoordinate position);

    TrackLine * createTrackLine(MissionItem* parent=nullptr, int row=-1, QString label = "");
    TrackLine * addTrackLine(QGeoCoordinate position);

    Behavior * createBehavior();
    
    Group * createGroup(MissionItem* parent=nullptr, int row=-1, QString label = "");
    Group * addGroup();

    Orbit * createOrbit(MissionItem* parent=nullptr, int row=-1, QString label = "");
    Orbit * addOrbit();
    
    MissionItem *itemFromIndex(QModelIndex const &index) const;

    Qt::ItemFlags flags(const QModelIndex & index) const override;
    QVariant data(const QModelIndex & index, int role) const override;
    QVariant headerData(int section, Qt::Orientation orientation, int role) const override;
    int rowCount(const QModelIndex & parent) const override;
    int columnCount(const QModelIndex & parent) const override;
    
    QModelIndex index(int row, int column, const QModelIndex & parent) const override;
    QModelIndex parent(const QModelIndex & child) const override;
    QModelIndex indexFromItem(MissionItem * item) const;

    // [#85] Rename a mission item and notify the view. setObjectName alone
    // changes the label the tree displays (data() returns objectName for
    // DisplayRole) but emits no model signal, so the view never refreshes —
    // the rename silently has no visible effect. Route renames through here.
    void renameItem(MissionItem * item, const QString & label);
    
    Qt::DropActions supportedDropActions() const override;
    
    bool removeRows(int row, int count, const QModelIndex & parent) override;
    
    QStringList mimeTypes() const override;
    QMimeData * mimeData(const QModelIndexList & indexes) const override;
    bool canDropMimeData(const QMimeData * data, Qt::DropAction action, int row, int column, const QModelIndex & parent) const override;
    bool dropMimeData(const QMimeData * data, Qt::DropAction action, int row, int column, const QModelIndex & parent) override;

    QString const &filename() const;
    void save(QString const &fname = QString());
    void open(QString const &fname);
    
    void openGeometry(QString const &fname, QString label = "");
    
    void import(QString const &fname);

    bool importGeoJson(QString const &fname);

    void setCurrent(const QModelIndex &index);
    MissionItem *currentSelected() const;
    
    QSvgRenderer * symbols() const;
    
    qreal mapScale() const;
    
    Platform* activePlatform() const;

    QJsonDocument generateMissionPlan(QModelIndex const &index);
    QJsonDocument generateMissionTask(QModelIndex const &index);
    QJsonDocument generateGeoJson(QModelIndex const &index);

    double speed() const;
    double throttle() const;

signals:
    // [#59 ADR-0003] Parameterless since BackgroundRaster retired — consumers
    // refresh positions / fit-to-extent from the chart layers, not a bg pointer.
    void backgroundUpdated();
    void aboutToUpdateBackground();
    void updatingBackground();
    void showTail(bool show);

public slots:

    void exportHypack(QModelIndex const &index);
    void exportMissionPlan(QModelIndex const &index);
    void exportGeoJson(QModelIndex const &index);

    void sendToROS(QModelIndex const &index);
    void appendMission(QModelIndex const &index);
    void prependMission(QModelIndex const &index);
    void updateMission(QModelIndex const &index);
    
    void deleteItems(QModelIndexList const &indices);
    void deleteItem(QModelIndex const &index);
    void deleteItem(MissionItem *item);
    void updateMapScale(qreal scale);
    void setContextMode(bool);

    void updateActivePlatform(Platform *platform);

    void setSpeed(double speed);
    void setThrottle(double throttle);

    void updateAvoidanceAreas();


private:
    camp::map::Map* m_map;
    QGraphicsScene* m_scene;        // owned by m_map; cached for internal use
    QString m_filename;
    // [#59 ADR-0003] Stacked chart display layers, Map-owned (parented to
    // m_map->topLevelLayers()); we keep raw pointers in load order for
    // fit-to-extent (last wins), persistence, and removal. Lifetime belongs to
    // the Map model — drop a pointer here only after detaching+deleting via it.
    std::vector<camp::raster::RasterLayer*> m_chartLayers;
    // [#59 ADR-0003] Depth provider list — one entry per loaded chart that
    // carries a depth band. getDepth(geo) walks the list in load order (first
    // valid wins); an entry is removed when its chart is removed (matched by
    // filename). This is the multi-background depth model (stage 4 will front it
    // with a tree).
    std::vector<DepthRaster*> m_depthRasters;
    Group* m_currentGroup;
    Group* m_root;
    MissionItem * m_currentSelected;

    Platform* m_activePlatform = nullptr;
    
    QSvgRenderer* m_symbols;

    bool m_contextMode = false;

    double m_speed = 0.0;
    double throttle_ = 0.4;

    // [#59 ADR-0003] Core chart loader shared by openBackground and
    // restorePersistedBackgrounds: append a stacked RasterLayer + DepthRaster,
    // emit the background signals. Does NOT persist (the callers decide).
    void addBackgroundLayer(QString const &fname, QString const &label);
    // [#59 ADR-0003] Persist the loaded-chart filename list (ordered) as app
    // state. Per-layer settings (visible/opacity/colormap) already persist via
    // camp_map's QSettings-by-itemID mechanism; this persists the *list*.
    void persistBackgrounds() const;
    // [#59 ADR-0003] React to a chart layer being removed via the Layers-tab
    // Remove action: drop its depth provider + bookkeeping entry and re-persist.
    // Connected to m_map's rowsAboutToBeRemoved so the Map model (camp_map) stays
    // unaware of the project's chart/depth bookkeeping.
    void onChartLayerRemoved(const QModelIndex& parent, int first, int last);
    QString generateUniqueLabel(std::string const &prefix);

    
public:
    
    class RowInserter
    {
    public:
        RowInserter(AutonomousVehicleProject &project, MissionItem *parent, int row=-1);
        
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
