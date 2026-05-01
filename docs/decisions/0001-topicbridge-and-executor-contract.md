# ADR-0001: `TopicBridge` and the executor contract

## Status

Accepted

## Context

`camp` predates most of the Project11 framework. It has accumulated structural
drift in how it crosses the ROS-callback / Qt-main-thread boundary and how it
consumes TF. The drift produces concrete user-visible problems:

- TF lookups happen synchronously inside ROS subscriber callbacks. The shared
  helper `ROSWidget::getGeoCoordinate()` (`src/camp/ros/ros_widget.cpp:14`) uses
  a 1.5 s timeout. `Grid` reimplements it with a 0.5 s timeout and a different
  exception policy (`src/camp/grids/grid.cpp:152`). When upstream TF stops, each
  callback can block for the full timeout per call.
- All camp subscriptions register on the node's default callback group
  (`MutuallyExclusive`). One stalled callback queues every other camp
  subscriber behind it, including heartbeat and nav. The `helm_manager`
  watchdog then turns red because *its* callback can't fire — not because the
  publisher is gone.
- Several callbacks touch Qt scene APIs from the executor thread.
  `Platform::pathCallback` (`src/camp/platform_manager/platform.cpp:241`) calls
  `findParentBackgroundRaster()` and emits scene-related signals from the
  executor; this has been latent-buggy for a long time.
- Subscriber implementations duplicate the same skeleton (subscription
  lifecycle, optional `MessageFilter`, optional try/catch, optional
  cross-thread emit) with subtle drift each time. `Markers` does it correctly
  via `tf2_ros::MessageFilter`; `Platform`'s path subscription does it raw,
  with no try/catch, looping `getGeoCoordinate` per pose.
- Class hierarchies mix ROS and Qt scene concerns by inheritance
  (`ROSWidget` + `GeoGraphicsItem`). Adding a new TF-consuming widget means
  understanding both halves and the seam between them.

The pattern is strong enough that fixing it once class-by-class will keep
regenerating warts. The codebase needs an explicit contract that names the
right abstractions and makes the wrong patterns uncompilable.

## Decision

Adopt four interlocking conventions for any ROS-driven Qt component in `camp`.

### 1. `TopicBridge<MsgT, PayloadT>` is the only sanctioned subscription path.

`TopicBridge` is a header-only template that owns the `rclcpp::Subscription`,
optionally wraps a `tf2_ros::MessageFilter`, runs a converter to produce a
value-typed `PayloadT`, and dispatches the payload to a `QObject*` receiver via
`QMetaObject::invokeMethod(..., Qt::QueuedConnection)`. It is **not** a
`QObject` itself — using a non-`QObject` template avoids the moc-on-templates
problem cleanly and lets a single template instance work for any payload type.

For sub-stream cases (e.g. `MarkerArray` containing many stamped `Marker`s
with different frame_ids), a sibling primitive `TfDispatcher<MsgT, PayloadT>`
exposes the same TF-gated dispatch pipeline without owning a subscription —
callers feed it messages by hand.

Bridges are **owned by the consuming Qt class as a member**, not inherited.
The class hierarchy is being migrated away from mixing ROS concerns and Qt
scene concerns through multiple inheritance; each port moves the consumer
toward composition until the old `ROSWidget` / `GeoGraphicsItem` pattern
disappears entirely. (The `Markers` exemplar in this PR still inherits both
bases — that wider hierarchy cleanup is part of the migration plan below,
not this PR.)

### 2. TF policy is expressed through `MessageFilter`, not through timeouts.

If a message has a header with a `frame_id`, its bridge wraps a `MessageFilter`
keyed on the target frame. By the time the converter runs, the transform is
already in the buffer; lookups inside the converter pass the message stamp
(or `tf2::TimePointZero` when the latest transform is genuinely intended) and
use **no timeout**. No callback ever blocks waiting on TF.

`ROSWidget::getGeoCoordinate()` and its `Grid::getGeoCoordinate()` clone are
deleted **once all consumers are ported**. While ports are in flight (this PR
ports `Markers` only) the helper remains for unported consumers and is removed
in the final port PR.

Direct calls to `transform_buffer_->transform(...)` outside a bridge converter
are a review-blocking smell from this point forward.

### 3. The threading contract: no Qt scene access from the executor thread.

Bridge converters and any code reachable from a ROS callback may only:

- Read message data and TF buffer state (with no timeout).
- Construct value-typed payloads.
- Write to thread-safe internal buffers (mutex-protected) on the consumer.
- Trigger the bridge's queued dispatch.

They may **not** call `findParentBackgroundRaster`, `geoToPixel`,
`prepareGeometryChange`, `QGraphicsItem::update`, or read/mutate any
QGraphicsScene state. Those happen in the receiver slot, on the Qt main
thread, after the queued dispatch.

This is enforced structurally: the bridge holds a `QObject*` receiver only and
does not hand the converter access to the consuming class's Qt parent.
A converter that wants to call `geoToPixel` cannot — the data needed for
`geoToPixel` (the BackgroundRaster) is reachable only from the receiver slot.

### 4. Callback groups split by latency budget.

The camp node exposes two `MutuallyExclusive` callback groups:

- **realtime** — heartbeat, nav, helm, mission status, AIS heartbeat
  (anything driving a watchdog or status indicator).
- **scene** — markers, grid, path, AIS contacts (bulk display data, may have
  expensive payload conversion).

The existing `MultiThreadedExecutor` (`src/camp/ros/node_thread.cpp:16`)
already supports two-group concurrency. With this split, a slow scene-callback
cannot starve the watchdog inputs. New bridges select their group at
construction; the default for TF-gated bridges is **scene**.

### Supporting change: `RosContext`

Replaces the manual `nodeStarted(node, buffer)` propagation through Qt parent
chains. `RosContext` is a small process-singleton whose lifetime is owned by
the ROS node thread: it is set during `camp_ros::NodeThread::start()`
(immediately before the executor begins spinning) and cleared when the
executor returns at shutdown. `ROSLink` only spawns the `NodeThread`; it
does not own the singleton. `RosContext` exposes the node, the TF buffer,
and the named callback groups. Bridges discover their dependencies on
construction; the `ROSClient::onNodeUpdated()` plumbing can be retired once
all consumers have moved off it.

## Consequences

### Positive

- New TF-consuming subscribers collapse to ~10 lines per consumer: declare a
  bridge, declare a receiver slot, connect.
- The "topic stops → camp stops updating everything" cascade is structurally
  prevented (point 4) and the per-callback stall is eliminated (point 2).
- Qt-from-executor-thread footguns become uncompilable rather than reviewable.
- TF policy lives in one file. Tuning, instrumentation, and caching all have
  an obvious home.
- The ROS-side and Qt-side concerns of each consumer are physically
  separated; the call boundary is value-typed and explicit.

### Negative

- Migration touches every TF-consuming widget: markers, grid, platform path,
  nav source, AIS, and a handful of smaller consumers. Estimate 4-6 PRs for
  full coverage.
- `MessageFilter` drops messages whose TF doesn't resolve within
  `buffer_timeout`. That is correct behavior, but it is a visible change for
  any consumer that previously left stale data on screen by accident.
- Templates in `TopicBridge` will increase build time modestly and produce
  uglier compile errors. Mitigatable with explicit instantiation in a `.cpp`
  if it bites in practice.
- `RosContext` is a singleton, which is a small concession to ergonomics over
  purity. Alternative (a `QObject` carried as an application-root property) is
  available if the singleton form proves problematic.

### Migration plan

1. **This PR:** Land `TopicBridge`, `TfDispatcher`, `RosContext`, the ADR,
   the `Markers` port, and test infrastructure.
2. **Next PR:** Port `Platform::pathCallback`. This is the bug-fix port:
   introduces `MessageFilter` for path, eliminates per-pose blocking, removes
   Qt scene access from the executor thread, and adds the missing try/catch
   for free.
3. **Subsequent PRs:** Port `Grid` (delete its local `getGeoCoordinate`
   override), then `NavSource`, `AisManager`, and remaining smaller consumers.
4. **Cleanup PR:** Delete `ROSWidget::getGeoCoordinate()`, audit for any
   remaining direct `transform_buffer_->transform(...)` calls, and retire the
   `ROSClient::onNodeUpdated` plumbing.

Each step is independently mergeable; reviewers can verify the per-class
change in isolation.

## Out of scope

- The heavy per-pixel `QImage::setPixelColor` work in
  `Grid::occupancyGridCallback` — a CPU concern, not a network/TF concern.
  Worth a follow-up issue but not part of this restructure.
- The `radar/` subsystem — confirmed not currently used.
- `camp2`. This decision applies to the legacy `camp/` subsystem only;
  `camp2` already starts from a cleaner separation.
- The topic-discovery pattern in `*Manager::scanForSources()` (#44).
  `TopicBridge` is the natural building block for that work, but the discovery
  layer itself is independent and is not addressed here.

## Related

- camp #48 — this work
- camp #44 — topic discovery factor-out (complementary, not subsumed)
- camp #31 — ROS reconnect robustness (made easier by this contract, not
  fixed here)

---
**Authored-By**: `Claude Code Agent`
**Model**: `Claude Opus 4.7 (1M context)`
