#include "projectview.h"

#include <QWheelEvent>
#include <QLabel>
#include <QStatusBar>
#include <QStandardItemModel>
#include "autonomousvehicleproject.h"
#include "waypoint.h"
#include "trackline.h"
#include "surveypattern.h"
#include "surveyarea.h"
#include "searchpattern.h"
#include "avoid_area.h"
#include <QDebug>
#include <QMenu>
#include "measuringtool.h"
#include "map_view/web_mercator.h"
#include <QPolygonF>
#include <QAbstractSlider>
#include <cmath>
#include <QScrollBar>
#include "roslink.h"
#include "platform_manager/platform.h"
#include "mission_manager/mission_manager.h"
#include "helm_manager/helm_manager.h"


ProjectView::ProjectView(QWidget *parent) : QGraphicsView(parent),
    statusBar(0), positionLabel(new QLabel()), modeLabel(new QLabel()), mouseMode(MouseMode::pan), currentTrackLine(nullptr), pendingTrackLineWaypoint(nullptr), pendingSurveyPattern(nullptr), pendingSurveyArea(nullptr),pendingSurveyAreaWaypoint(nullptr),measuringTool(nullptr)
{

    positionLabel->setText("(,)");
    modeLabel->setText("Mode: pan");

    connect(horizontalScrollBar(), &QAbstractSlider::valueChanged, this, &ProjectView::sendViewport);
    connect(verticalScrollBar(), &QAbstractSlider::valueChanged, this , &ProjectView::sendViewport);

    // [#59 PR3a] Web Mercator uses Y-increasing-north, but a QGraphicsView is
    // Y-increasing-down. Flip the Y axis so north renders up (matches the sign
    // convention in camp::MapView). Subsequent uniform zooms preserve the sign,
    // and fitInView() multiplies the current transform so the flip survives.
    // See ADR-0002.
    scale(1.0, -1.0);
}

void ProjectView::wheelEvent(QWheelEvent *event)
{
    if(event->angleDelta().y()<0)
        scale(.8,.8);
    if(event->angleDelta().y()>0)
        scale(1.25,1.25);
    emit scaleChanged(matrix().m11());
    sendViewport();
    event->accept();
}

void ProjectView::mousePressEvent(QMouseEvent *event)
{
    // [#59] Mission-item placement uses web_mercator::mapToGeo (Web-Mercator
    // scene → WGS84), independent of any loaded chart, so items can be created
    // over OSM/WMTS-only backgrounds. (Was gated on a BackgroundRaster being
    // loaded back when the raster defined the coordinate system.)
    switch(event->button())
    {
    case Qt::LeftButton:
        switch(mouseMode)
        {
        case MouseMode::pan:
            break;
        case MouseMode::addWaypoint:
            m_project->addWaypoint(web_mercator::mapToGeo(mapToScene(event->pos())));
            setPanMode();
            break;
        case MouseMode::addTrackline:
            if(!currentTrackLine)
            {
                currentTrackLine = m_project->addTrackLine(web_mercator::mapToGeo(mapToScene(event->pos())));
                pendingTrackLineWaypoint = currentTrackLine->addWaypoint(web_mercator::mapToGeo(mapToScene(event->pos())));
            }
            else
            {
                pendingTrackLineWaypoint = currentTrackLine->addWaypoint(web_mercator::mapToGeo(mapToScene(event->pos())));

            }
            break;
        case MouseMode::addSurveyPattern:
            if(!pendingSurveyPattern)
            {
                pendingSurveyPattern = m_project->addSurveyPattern(web_mercator::mapToGeo(mapToScene(event->pos())));
                //QModelIndex i = m_project-> indexFromItem(pendingSurveyPattern);
                //emit  currentChanged(i);
            }
            else
            {
                if(pendingSurveyPattern->hasSpacingLocation())
                {
                    setPanMode();
                }
                else
                {
                    pendingSurveyPattern->setSpacingLocation(web_mercator::mapToGeo(mapToScene(event->pos())));
                }
            }
            break;
        case MouseMode::addSurveyArea:
            if(!pendingSurveyArea)
            {
                pendingSurveyArea = m_project->addSurveyArea(web_mercator::mapToGeo(mapToScene(event->pos())));
                pendingSurveyAreaWaypoint = pendingSurveyArea->addWaypoint(web_mercator::mapToGeo(mapToScene(event->pos())));
            }
            else
            {
                pendingSurveyAreaWaypoint = pendingSurveyArea->addWaypoint(web_mercator::mapToGeo(mapToScene(event->pos())));
            }
            break;
        case MouseMode::addAvoidArea:
            if(!pendingAvoidArea)
            {
                pendingAvoidArea = m_project->addAvoidArea(web_mercator::mapToGeo(mapToScene(event->pos())));
                pendingAvoidAreaWaypoint = pendingAvoidArea->addPoint(web_mercator::mapToGeo(mapToScene(event->pos())));
            }
            else
            {
                pendingAvoidAreaWaypoint = pendingAvoidArea->addPoint(web_mercator::mapToGeo(mapToScene(event->pos())));
            }
            break;
        case MouseMode::addSearchPattern:
            if(!pendingSearchPattern)
            {
                pendingSearchPattern = m_project->addSearchPattern(web_mercator::mapToGeo(mapToScene(event->pos())));
                //QModelIndex i = m_project-> indexFromItem(pendingSurveyPattern);
                //emit  currentChanged(i);
            }
            else
            {
                if(pendingSearchPattern->hasSpacingLocation())
                {
                    setPanMode();
                }
                else
                {
                    pendingSearchPattern->setSpacingLocation(web_mercator::mapToGeo(mapToScene(event->pos())));
                }
            }
            break;
        }
        break;

    case Qt::RightButton:
        if(mouseMode == MouseMode::addTrackline || mouseMode == MouseMode::addWaypoint || mouseMode == MouseMode::addSurveyPattern || mouseMode == MouseMode::addSurveyArea || mouseMode == MouseMode::addSearchPattern || mouseMode == MouseMode::addAvoidArea)
        {
            if(mouseMode == MouseMode::addTrackline && currentTrackLine)
            {
                m_project->scene()->removeItem(pendingTrackLineWaypoint);
                m_project->deleteItem(pendingTrackLineWaypoint);
                m_project->scene()->update();
                pendingTrackLineWaypoint = nullptr;
                update();
            }
            if(mouseMode == MouseMode::addSurveyArea && pendingSurveyArea)
            {
                m_project->scene()->removeItem(pendingSurveyAreaWaypoint);
                m_project->deleteItem(pendingSurveyAreaWaypoint);
                m_project->scene()->update();
                pendingSurveyAreaWaypoint = nullptr;
                pendingSurveyArea = nullptr;
                update();
            }
            if(mouseMode == MouseMode::addAvoidArea && pendingAvoidArea)
            {
                m_project->scene()->removeItem(pendingAvoidAreaWaypoint);
                m_project->deleteItem(pendingAvoidAreaWaypoint);
                m_project->scene()->update();
                pendingAvoidAreaWaypoint = nullptr;
                pendingAvoidArea = nullptr;
                m_project->updateAvoidanceAreas();
                update();
            }
            setPanMode();
            event->accept();
        }
        break;
    case Qt::MiddleButton:
        if(!measuringTool)
        {
            // [#59 ADR-0003] Parent the tool to the Map scene-origin anchor (always
            // present), not a chart, so measuring works over OSM/WMTS-only too.
            measuringTool = new MeasuringTool(m_project->originAnchor(), m_project);
            measuringTool->setStart(web_mercator::mapToGeo(mapToScene(event->pos())));
            measuringTool->setFinish(web_mercator::mapToGeo(mapToScene(event->pos())));
        }
        break;
    default:
        break;
    }
    QGraphicsView::mousePressEvent(event);
}

void ProjectView::mouseMoveEvent(QMouseEvent *event)
{
    QString posText = QString::number(event->pos().x())+","+QString::number(event->pos().y());

    QPointF transformedMouse = mapToScene(event->pos());
    // [#59] Cursor readout and pending-item updates use web_mercator::mapToGeo
    // (Web-Mercator scene → WGS84), independent of any loaded chart. Depth is
    // queried by geo and simply omitted when no source covers the cursor, so
    // the readout works over OSM/WMTS-only backgrounds too.
    posText += " WebMercator: "+QString::number(transformedMouse.x(),'f')+","+QString::number(transformedMouse.y(),'f');
    QGeoCoordinate llMouse = web_mercator::mapToGeo(transformedMouse);
    posText += " WGS84: " + llMouse.toString(QGeoCoordinate::Degrees) + " (" + llMouse.toString(QGeoCoordinate::DegreesMinutesWithHemisphere) + ")";

    // [camp#180] GGGS store layers report ellipsoidal up-positive elevation, not
    // chart-datum depth. Query stores first (they take precedence) and label the
    // value distinctly so the operator can tell the two datums apart until the
    // datum service (#288). Both labels can appear when a store and a depth chart
    // overlap the cursor.
    const float elevation = m_project->getStoreElevation(llMouse);
    if(!std::isnan(elevation))
        posText += " Elev: " + QString::number(elevation) + " (ellipsoid)";

    const float depth = m_project->getDepth(llMouse);
    if(!std::isnan(depth))
        posText += " Depth: " +QString::number(depth);

    if(pendingSurveyPattern)
    {
        if(pendingSurveyPattern->hasSpacingLocation())
            pendingSurveyPattern->setSpacingLocation(web_mercator::mapToGeo(mapToScene(event->pos())));
        else
            pendingSurveyPattern->setEndLocation(web_mercator::mapToGeo(mapToScene(event->pos())));
    }
    if(pendingTrackLineWaypoint)
    {
        pendingTrackLineWaypoint->setLocation(web_mercator::mapToGeo(mapToScene(event->pos())));
    }
    if(pendingSurveyAreaWaypoint)
    {
        pendingSurveyAreaWaypoint->setLocation(web_mercator::mapToGeo(mapToScene(event->pos())));
    }
    if(pendingAvoidAreaWaypoint)
    {
        pendingAvoidAreaWaypoint->setLocation(web_mercator::mapToGeo(mapToScene(event->pos())));
    }
    if(pendingSearchPattern)
    {
        if(pendingSearchPattern->hasSpacingLocation())
            pendingSearchPattern->setSpacingLocation(web_mercator::mapToGeo(mapToScene(event->pos())));
        else
            pendingSearchPattern->setEndLocation(web_mercator::mapToGeo(mapToScene(event->pos())));
    }
    if(measuringTool)
        measuringTool->setFinish(llMouse);

    positionLabel->setText(posText);
    QGraphicsView::mouseMoveEvent(event);
}

void ProjectView::mouseReleaseEvent(QMouseEvent *event)
{
    if(event->button() == Qt::MiddleButton)
    {
        if(measuringTool)
            delete measuringTool;
        measuringTool = nullptr;
    }
    QGraphicsView::mouseReleaseEvent(event);
}

void ProjectView::setAddWaypointMode()
{
    setDragMode(NoDrag);
    mouseMode = MouseMode::addWaypoint;
    modeLabel->setText("Mode: add waypoint");
    setCursor(Qt::CrossCursor);
}

void ProjectView::setAddTracklineMode()
{
    setDragMode(NoDrag);
    mouseMode = MouseMode::addTrackline;
    modeLabel->setText("Mode: add trackline");
    setCursor(Qt::CrossCursor);
}

void ProjectView::setAddSurveyPatternMode()
{
    setDragMode(NoDrag);
    mouseMode = MouseMode::addSurveyPattern;
    modeLabel->setText("Mode: add survey pattern");
    setCursor(Qt::CrossCursor);
}

void ProjectView::setAddSurveyAreaMode()
{
    setDragMode(NoDrag);
    mouseMode = MouseMode::addSurveyArea;
    modeLabel->setText("Mode: add survey area");
    setCursor(Qt::CrossCursor);
}

void ProjectView::setAddAvoidAreaMode()
{
    setDragMode(NoDrag);
    mouseMode = MouseMode::addAvoidArea;
    modeLabel->setText("Mode: add area to avoid");
    setCursor(Qt::CrossCursor);
}


void ProjectView::setAddSearchPatternMode()
{
    setDragMode(NoDrag);
    mouseMode = MouseMode::addSearchPattern;
    modeLabel->setText("Mode: add search pattern");
    setCursor(Qt::CrossCursor);
}

void ProjectView::setStatusBar(QStatusBar *bar)
{
    statusBar = bar;
    statusBar->addWidget(positionLabel);
    statusBar->addPermanentWidget(modeLabel);
}

void ProjectView::setProject(AutonomousVehicleProject *project)
{
    m_project = project;
    setScene(project->scene());
}

void ProjectView::setPanMode()
{
    setDragMode(ScrollHandDrag);
    mouseMode = MouseMode::pan;
    modeLabel->setText("Mode: pan");
    unsetCursor();
    pendingSurveyPattern = nullptr;
    currentTrackLine = nullptr;
    pendingSearchPattern = nullptr;
    pendingAvoidArea = nullptr;
}

void ProjectView::contextMenuEvent(QContextMenuEvent* event)
{
    // [#59] Boat commands (Hover/Goto/Idle/Look Here) place by web_mercator geo,
    // independent of any loaded chart, so the menu works over OSM/WMTS-only
    // backgrounds. Braced to scope the local QMenu.
    {
        m_contextMenuLocation = web_mercator::mapToGeo(mapToScene(event->pos()));
        qDebug() << m_contextMenuLocation;
        QMenu menu(this);

        menu.addSeparator();
        menu.addAction("(Blank to avoid accidental hover)");
        menu.addSeparator();

        QAction *hoverAction = menu.addAction("Hover Here");
        connect(hoverAction, &QAction::triggered, this, &ProjectView::sendHover);

        QAction *gotoAction = menu.addAction("Goto Here");
        connect(gotoAction, &QAction::triggered, this, &ProjectView::sendGoto);

        QAction *idleAction = menu.addAction("Idle in place");
        connect(idleAction, &QAction::triggered, this, &ProjectView::sendIdle);

        menu.addSeparator();
        menu.addAction("(Above moves or idles boat)");
        menu.addSeparator();
        menu.addAction("(Below moves camera)");
        menu.addSeparator();


        QAction *lookAtAction = menu.addAction("Look Here");
        connect(lookAtAction, &QAction::triggered, this, &ProjectView::sendLookAt);

        QAction *lookAtASVAction = menu.addAction("Look at ASV");
        connect(lookAtASVAction, &QAction::triggered, this, &ProjectView::sendLookAtASV);

        menu.exec(event->globalPos());
    }
    event->accept();

}

void ProjectView::sendHover()
{
  auto p = m_project->activePlatform();
  if(p)
  {
    p->missionManager()->sendHover(m_contextMenuLocation);
    p->helmManager()->sendPilotingModeRequest("autonomous");
  }
}

void ProjectView::sendGoto()
{
  auto p = m_project->activePlatform();
  if(p)
  {
    p->missionManager()->sendGoto(m_contextMenuLocation);
    p->helmManager()->sendPilotingModeRequest("autonomous");
  }
}

void ProjectView::sendIdle()
{
  auto p = m_project->activePlatform();
  if(p)
  {
    p->missionManager()->sendIdle();
  }
}


void ProjectView::sendLookAt()
{
    //m_project->rosLink()->sendLookAt(m_contextMenuLocation);
}

void ProjectView::sendLookAtASV()
{
    //m_project->rosLink()->sendLookAtMode("follow_vehicle");
}

void ProjectView::beforeUpdateBackground()
{
    // [#59 ADR-0003] If a chart is already loaded, save the current view center so
    // the next chart stacks without the view jumping; otherwise (first chart)
    // leave it invalid so updateBackground fits to the new chart.
    if(m_project->hasBackground())
    {
        QRect view = frameRect();
        QPointF center = mapToScene(view.center());
        m_savedCenter = web_mercator::mapToGeo(center);
    }
    else
    {
        // clear saved position
        m_savedCenter = QGeoCoordinate();
    }
    qDebug() << "saved center: " << m_savedCenter;

}

void ProjectView::updateBackground()
{
    // [#59 PR3a] The chart is displayed by the reprojected RasterLayer; fit the
    // view to the chart's extent in Web-Mercator scene space. fitInView multiplies
    // the current transform, so the constructor's Y-flip (north up) is preserved.
    // When stacking onto an existing chart, recenter on the saved geo position
    // instead of jumping. See ADR-0002/0003.
    if(!m_project->hasBackground())
        return;
    // [#59 ADR-0003] The chart extent comes from the RasterLayer's
    // sceneBoundingRect (already Web-Mercator scene space, valid synchronously
    // per stage 1) — no BackgroundRaster pixel→geo→map round-trip. The warp
    // produced an axis-aligned EPSG:3857 raster, so its scene bounding box is the
    // chart extent directly.
    QRectF chartBox = m_project->currentBackgroundExtent();
    if(m_savedCenter.isValid())
        centerOn(web_mercator::geoToMap(m_savedCenter));
    else if(!chartBox.isEmpty())
        fitInView(chartBox, Qt::KeepAspectRatio);
}

void ProjectView::centerMap(QGeoCoordinate location)
{
    centerOn(web_mercator::geoToMap(location));
}

void ProjectView::sendViewport()
{
    QPointF ll = mapToScene(frameRect().bottomLeft());
    QPointF ur = mapToScene(frameRect().topRight());
    emit viewportChanged(ll,ur);
}
