#ifndef VIEW_RESIZE_ANCHOR_H
#define VIEW_RESIZE_ANCHOR_H

#include <QGraphicsView>

namespace camp
{

// [camp#242] Keep the map centre fixed when the view is resized.
//
// QGraphicsView's default resize anchor is NoAnchor, which keeps the scene point
// at the TOP-LEFT corner fixed, so any resize moves the centre by half the size
// change. CAMP restores its persisted centre on the first event-loop turn, at the
// window's pre-maximize size; the window manager's maximize then arrives as a
// resize and shifted the restored view by kilometres. AnchorViewCenter makes every
// resize (maximize, dragging the window edge, a dock panel appearing) keep the
// centre. This is separate from transformationAnchor (used by scale()), which is
// left alone.
//
// Lives in a header of its own so ProjectView (which cannot be constructed in a
// gtest: it pulls in roslink and the manager classes) and the test share one call.
inline void keepCenterOnResize(QGraphicsView *view)
{
    view->setResizeAnchor(QGraphicsView::AnchorViewCenter);
}

} // namespace camp

#endif // VIEW_RESIZE_ANCHOR_H
