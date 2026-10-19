// [camp#242] The map centre must survive a view resize.
//
// CAMP restores its persisted centre at the window's pre-maximize size; the window
// manager's maximize then resizes the view. With QGraphicsView's default NoAnchor
// the top-left scene point stays put and the centre moves by half the size change
// (observed live: ~1.7 km on a maximize). camp::keepCenterOnResize() (called from
// ProjectView's constructor) sets AnchorViewCenter.
//
// ProjectView itself cannot be built in a gtest (roslink + managers); the view here
// is a plain QGraphicsView set up like ProjectView: Y-flipped, a scene far larger
// than the viewport, scaled and centred on a persisted point.

#include <gtest/gtest.h>

#include <QApplication>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QPointF>

#include <cmath>

#include "view_resize_anchor.h"

namespace
{
QPointF viewCentre(const QGraphicsView &v)
{
    return v.mapToScene(v.viewport()->rect().center());
}

// Scene coordinates are Web-Mercator metres (millions); mirror that magnitude.
const QPointF kSavedCentre(-7840000.0, 5290000.0);

void restoreView(QGraphicsView &v, QGraphicsScene &scene)
{
    scene.setSceneRect(-20037508.0, -20037508.0, 40075016.0, 40075016.0);
    v.setScene(&scene);
    v.scale(1.0, -1.0);
    v.resize(1033, 807);
    v.show();
    QApplication::processEvents();
    v.scale(0.05, 0.05);
    v.centerOn(kSavedCentre);
}
}  // namespace

TEST(ViewResizeAnchor, CentreSurvivesResize)
{
    QGraphicsScene scene;
    QGraphicsView view;
    camp::keepCenterOnResize(&view);
    restoreView(view, scene);

    const QPointF before = viewCentre(view);
    const double scaleBefore = view.transform().m11();
    // Pixel size in scene units: tolerance of one pixel.
    const double pixel = 1.0 / std::abs(scaleBefore);
    EXPECT_NEAR(before.x(), kSavedCentre.x(), pixel);
    EXPECT_NEAR(before.y(), kSavedCentre.y(), pixel);

    view.resize(3774, 2091);  // the maximize observed live
    QApplication::processEvents();

    const QPointF after = viewCentre(view);
    EXPECT_NEAR(after.x(), before.x(), pixel);
    EXPECT_NEAR(after.y(), before.y(), pixel);
    EXPECT_DOUBLE_EQ(view.transform().m11(), scaleBefore);

    view.resize(800, 600);  // and shrinking again
    QApplication::processEvents();
    const QPointF shrunk = viewCentre(view);
    EXPECT_NEAR(shrunk.x(), before.x(), pixel);
    EXPECT_NEAR(shrunk.y(), before.y(), pixel);
}

// Control: documents the bug. With the Qt default (NoAnchor) the same resize moves
// the centre by far more than a pixel, so the test above would fail without the fix.
TEST(ViewResizeAnchor, DefaultNoAnchorShiftsCentre)
{
    QGraphicsScene scene;
    QGraphicsView view;
    ASSERT_EQ(view.resizeAnchor(), QGraphicsView::NoAnchor);
    restoreView(view, scene);

    const QPointF before = viewCentre(view);
    const double pixel = 1.0 / std::abs(view.transform().m11());

    view.resize(3774, 2091);
    QApplication::processEvents();

    const QPointF after = viewCentre(view);
    const double shift = std::hypot(after.x() - before.x(), after.y() - before.y());
    EXPECT_GT(shift, 100.0 * pixel);
}

TEST(ViewResizeAnchor, SetsOnlyTheResizeAnchor)
{
    QGraphicsView view;
    const auto transformation = view.transformationAnchor();
    camp::keepCenterOnResize(&view);
    EXPECT_EQ(view.resizeAnchor(), QGraphicsView::AnchorViewCenter);
    EXPECT_EQ(view.transformationAnchor(), transformation);
}

int main(int argc, char **argv)
{
    qputenv("QT_QPA_PLATFORM", "offscreen");
    QApplication app(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
