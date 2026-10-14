// [camp#142] Per-layer colormap range override: persist/restore round-trip across
// ALL THREE raster layers (GggsTileLayer, RasterLayer, SonarLiveCacheLayer).
//
// The override lives in a marine_colormap::RangeModel each layer owns. Auto mode
// tracks the data extents; Manual mode pins an operator [lo, hi] that is fed to the
// shader's u_min/u_max at render time (replacing the raw data_min_/data_max_). These
// tests pin the public API + the QSettings round-trip:
//   - default is Auto;
//   - setRangeOverride(lo, hi) -> Manual with lo()/hi() == the override;
//   - resetRangeToAuto() -> Auto;
//   - a Manual override (mode + [lo, hi]) survives writeSettings()->readSettings();
//   - an Auto layer persists + restores as Auto.
//
// GL-FREE by construction: the RangeModel is pure data and renderImage() is never
// called, so these RUN (not SKIP) in-container — exactly like the band round-trip in
// test_gggs_persistence.cpp. Each layer is exercised through a tiny Testable subclass
// that exposes the protected read/writeSettings hooks (the same pattern that test
// uses), so the round-trip is asserted synchronously without the deferred
// itemConstructed() timer.
//
// Per-layer settings identity (two instances must share a key to round-trip):
//   - GggsTileLayer    -> settingsKey() = "dir:" + the tile-set directory;
//   - SonarLiveCacheLayer -> settingsKey() = "live:" + the source namespace;
//   - RasterLayer      -> itemID() (parent path + the file's basename) — NOT
//     settingsKey(); RasterLayer's read/writeSettings group under itemID().
// An empty tile-set dir / a never-opened file / a null ROS node keep all three
// GL/GDAL/ROS-free.

#include <gtest/gtest.h>

#include <map>
#include <optional>
#include <vector>

#include <QApplication>
#include <QSettings>
#include <QTemporaryDir>

#include <marine_colormap/transfer.hpp>

#include "map/map.h"
#include "map/layer_list.h"
#include "raster/gggs_tile_layer.h"
#include "raster/shoreline_anchor.h"
#include "raster/raster_layer.h"
#include "ros/live_coverage/sonar_live_cache_layer.h"

using camp::map::Map;
using camp::raster::GggsTileLayer;
using camp::raster::RasterLayer;
using camp::ros::live_coverage::SonarLiveCacheLayer;
using marine_colormap::RangeMode;
using Source = camp::raster::ShorelineAnchor::Source;

namespace
{

// Expose the protected settings hooks so the round-trip is testable synchronously
// (mirrors test_gggs_persistence.cpp's TestableGggsTileLayer).
class TestableGggsTileLayer: public GggsTileLayer
{
public:
  using GggsTileLayer::GggsTileLayer;
  using GggsTileLayer::readSettings;
  using GggsTileLayer::writeSettings;
};

class TestableRasterLayer: public RasterLayer
{
public:
  using RasterLayer::RasterLayer;
  using RasterLayer::readSettings;
  using RasterLayer::writeSettings;
};

class TestableSonarLiveCacheLayer: public SonarLiveCacheLayer
{
public:
  using SonarLiveCacheLayer::SonarLiveCacheLayer;
  using SonarLiveCacheLayer::readSettings;
  using SonarLiveCacheLayer::writeSettings;
};

}  // namespace

// ----------------------------- GggsTileLayer ---------------------------------

TEST(RangePersist, GggsDefaultOverrideReset)
{
  QSettings().clear();
  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);
  QTemporaryDir tileset;
  ASSERT_TRUE(tileset.isValid());

  auto* layer = new TestableGggsTileLayer(layers, tileset.path());
  EXPECT_EQ(layer->rangeMode(), RangeMode::Auto);   // default before any override

  layer->setRangeOverride(1.0f, 5.0f);
  EXPECT_EQ(layer->rangeMode(), RangeMode::Manual);
  EXPECT_FLOAT_EQ(layer->rangeLo(), 1.0f);
  EXPECT_FLOAT_EQ(layer->rangeHi(), 5.0f);

  layer->resetRangeToAuto();
  EXPECT_EQ(layer->rangeMode(), RangeMode::Auto);
}

TEST(RangePersist, GggsManualRoundTrips)
{
  QSettings().clear();
  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);
  QTemporaryDir tileset;
  ASSERT_TRUE(tileset.isValid());

  {
    auto* layer = new TestableGggsTileLayer(layers, tileset.path());
    layer->setRangeOverride(1.0f, 5.0f);
    layer->writeSettings();
  }
  {
    auto* layer = new TestableGggsTileLayer(layers, tileset.path());
    layer->readSettings();
    EXPECT_EQ(layer->rangeMode(), RangeMode::Manual)
        << "restored range mode must be the persisted Manual, not Auto";
    EXPECT_FLOAT_EQ(layer->rangeLo(), 1.0f);
    EXPECT_FLOAT_EQ(layer->rangeHi(), 5.0f);
  }
}

// [camp#132] The per-layer blit-smoothing opt-in round-trips: default OFF,
// enabled + persisted by one layer instance, restored by the next.
TEST(RangePersist, GggsSmoothInterpolationRoundTrips)
{
  QSettings().clear();
  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);
  QTemporaryDir tileset;
  ASSERT_TRUE(tileset.isValid());

  {
    auto* layer = new TestableGggsTileLayer(layers, tileset.path());
    EXPECT_FALSE(layer->smoothInterpolation());   // faithful-QA default
    layer->setSmoothInterpolation(true);          // persists via writeSettings
  }
  {
    auto* layer = new TestableGggsTileLayer(layers, tileset.path());
    layer->readSettings();
    EXPECT_TRUE(layer->smoothInterpolation())
        << "persisted smooth-interpolation opt-in must survive a restart";
  }
}

TEST(RangePersist, GggsAutoRoundTrips)
{
  QSettings().clear();
  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);
  QTemporaryDir tileset;
  ASSERT_TRUE(tileset.isValid());

  {
    auto* layer = new TestableGggsTileLayer(layers, tileset.path());
    layer->writeSettings();   // never overridden -> persists Auto
  }
  {
    auto* layer = new TestableGggsTileLayer(layers, tileset.path());
    layer->readSettings();
    EXPECT_EQ(layer->rangeMode(), RangeMode::Auto);
    // An Auto round-trip must leave the model tracking the data, not pin a manual
    // override; with no tiles loaded it stays at the RangeModel default extents.
    EXPECT_FLOAT_EQ(layer->rangeLo(), 0.0f);
    EXPECT_FLOAT_EQ(layer->rangeHi(), 1.0f);
  }
}

// [camp#181 / ADR-0015] The manual shoreline anchor round-trips through QSettings,
// and — the part that matters — its ABSENCE restores as unanchored None, never as
// a 0.0 anchor. D6 forbids substituting 0.0 because anchor values are ellipsoidal
// heights: 0.0 would put the land/sea break ~28 m into deep water at the Shoals.
// GL-free: the anchor holder is pure data and renderImage() is never called.
TEST(RangePersist, GggsShorelineAnchorRoundTrips)
{
  QSettings().clear();
  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);
  QTemporaryDir tileset;
  ASSERT_TRUE(tileset.isValid());

  {
    auto* layer = new TestableGggsTileLayer(layers, tileset.path());
    EXPECT_EQ(layer->shorelineAnchorMode(), Source::None);      // unanchored default
    EXPECT_FALSE(layer->shorelineManualAnchor().has_value());
    layer->applyShorelineAnchor(Source::Manual, -28.038);       // persists
  }
  {
    auto* layer = new TestableGggsTileLayer(layers, tileset.path());
    layer->readSettings();
    EXPECT_EQ(layer->shorelineAnchorMode(), Source::Manual)
        << "a persisted manual anchor must restore its MODE, not just its value";
    ASSERT_TRUE(layer->shorelineManualAnchor().has_value());
    EXPECT_DOUBLE_EQ(*layer->shorelineManualAnchor(), -28.038);
    ASSERT_TRUE(layer->resolvedShorelineAnchor().has_value());
    EXPECT_DOUBLE_EQ(*layer->resolvedShorelineAnchor(), -28.038);
  }
}

TEST(RangePersist, GggsAbsentShorelineAnchorRestoresUnanchoredNotZero)
{
  QSettings().clear();
  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);
  QTemporaryDir tileset;
  ASSERT_TRUE(tileset.isValid());

  {
    auto* layer = new TestableGggsTileLayer(layers, tileset.path());
    layer->writeSettings();   // never anchored -> the key is removed, not zeroed
  }
  {
    auto* layer = new TestableGggsTileLayer(layers, tileset.path());
    layer->readSettings();
    EXPECT_EQ(layer->shorelineAnchorMode(), Source::None);
    EXPECT_FALSE(layer->resolvedShorelineAnchor().has_value())
        << "ADR-0015 D6: an absent anchor is unanchored, never 0.0";
  }
}

// [camp#181 / ADR-0015 D6] A corrupt/unparsable persisted key must NOT read back as
// 0.0 — QSettings::toDouble()'s no-ok overload returns exactly that.
TEST(RangePersist, GggsCorruptShorelineAnchorRestoresUnanchored)
{
  QSettings().clear();
  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);
  QTemporaryDir tileset;
  ASSERT_TRUE(tileset.isValid());

  auto* layer = new TestableGggsTileLayer(layers, tileset.path());
  layer->writeSettings();   // establish the group so settingsKey() is known-good
  {
    QSettings settings;
    settings.beginGroup("MapItem");
    settings.beginGroup(layer->settingsKey());
    settings.setValue("shoreline_anchor", "not-a-number");
    settings.endGroup();
    settings.endGroup();
  }
  layer->readSettings();
  EXPECT_EQ(layer->shorelineAnchorMode(), Source::None);
  EXPECT_FALSE(layer->resolvedShorelineAnchor().has_value())
      << "an unparsable anchor must fall back to unanchored, never to 0.0";
}

// [camp#181 / ADR-0015 D6] The anchor MODE is persisted, not inferred from the
// value's presence. The range dialog deliberately KEEPS a typed manual value when
// the operator selects None (so switching back restores it), so the sequence
// "Manual -28.038 -> None" leaves a stored value with no anchor selected. Inferring
// Manual from that value would restore an anchor nobody chose — D6's failure mode
// reached through mode inference instead of through 0.0.
TEST(RangePersist, GggsNoneWithARetainedManualValueRestoresUnanchored)
{
  QSettings().clear();
  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);
  QTemporaryDir tileset;
  ASSERT_TRUE(tileset.isValid());

  {
    auto* layer = new TestableGggsTileLayer(layers, tileset.path());
    layer->applyShorelineAnchor(Source::Manual, -28.038);
    layer->applyShorelineAnchor(Source::None, -28.038);   // value kept, anchor off
    ASSERT_TRUE(layer->shorelineManualAnchor().has_value());
  }
  {
    auto* layer = new TestableGggsTileLayer(layers, tileset.path());
    layer->readSettings();
    EXPECT_EQ(layer->shorelineAnchorMode(), Source::None)
        << "None must restore as None even with a manual value still stored";
    EXPECT_FALSE(layer->resolvedShorelineAnchor().has_value())
        << "the retained value must not resolve: the operator chose unanchored";
    ASSERT_TRUE(layer->shorelineManualAnchor().has_value())
        << "the typed value is still kept, so switching back to Manual restores it";
    EXPECT_DOUBLE_EQ(*layer->shorelineManualAnchor(), -28.038);
  }
}

// A settings file written before the mode key existed (or hand-edited to an
// unrecognized token) restores unanchored rather than guessing Manual.
TEST(RangePersist, GggsUnrecognizedAnchorModeRestoresUnanchored)
{
  QSettings().clear();
  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);
  QTemporaryDir tileset;
  ASSERT_TRUE(tileset.isValid());

  auto* layer = new TestableGggsTileLayer(layers, tileset.path());
  layer->applyShorelineAnchor(Source::Manual, -28.038);
  {
    QSettings settings;
    settings.beginGroup("MapItem");
    settings.beginGroup(layer->settingsKey());
    settings.setValue("shoreline_anchor_mode", "tide_gauge_7");   // not a Source
    settings.endGroup();
    settings.endGroup();
  }
  layer->readSettings();
  EXPECT_EQ(layer->shorelineAnchorMode(), Source::None);
  EXPECT_FALSE(layer->resolvedShorelineAnchor().has_value())
      << "an unrecognized mode token must restore unanchored, never infer Manual";
}

// The upper D3 sources have no provider until PR2/PR3, but their SELECTION must
// still survive a restart — persisting a mode is what makes that possible at all.
TEST(RangePersist, GggsChartDatumModeRoundTripsWithoutAValue)
{
  QSettings().clear();
  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);
  QTemporaryDir tileset;
  ASSERT_TRUE(tileset.isValid());

  {
    auto* layer = new TestableGggsTileLayer(layers, tileset.path());
    layer->applyShorelineAnchor(Source::ChartDatum, std::nullopt);
  }
  {
    auto* layer = new TestableGggsTileLayer(layers, tileset.path());
    layer->readSettings();
    EXPECT_EQ(layer->shorelineAnchorMode(), Source::ChartDatum);
    EXPECT_FALSE(layer->resolvedShorelineAnchor().has_value())
        << "no source resolves chart datum in PR1 — it must stay unanchored";
  }
}

// ------------------------------ RasterLayer ----------------------------------


TEST(RangePersist, RasterDefaultOverrideReset)
{
  QSettings().clear();
  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString file = dir.filePath("chart.tif");   // need not exist for the range API

  auto* layer = new TestableRasterLayer(layers, file);
  EXPECT_EQ(layer->rangeMode(), RangeMode::Auto);

  layer->setRangeOverride(2.0f, 8.0f);
  EXPECT_EQ(layer->rangeMode(), RangeMode::Manual);
  EXPECT_FLOAT_EQ(layer->rangeLo(), 2.0f);
  EXPECT_FLOAT_EQ(layer->rangeHi(), 8.0f);

  layer->resetRangeToAuto();
  EXPECT_EQ(layer->rangeMode(), RangeMode::Auto);
}

TEST(RangePersist, RasterManualRoundTrips)
{
  QSettings().clear();
  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString file = dir.filePath("chart.tif");

  // RasterLayer groups under itemID() (parent path + the file's basename), so two
  // layers over the SAME filename + parent share a settings group.
  {
    auto* layer = new TestableRasterLayer(layers, file);
    layer->setRangeOverride(2.0f, 8.0f);
    layer->writeSettings();
  }
  {
    auto* layer = new TestableRasterLayer(layers, file);
    layer->readSettings();
    EXPECT_EQ(layer->rangeMode(), RangeMode::Manual);
    EXPECT_FLOAT_EQ(layer->rangeLo(), 2.0f);
    EXPECT_FLOAT_EQ(layer->rangeHi(), 8.0f);
  }
}

// [camp#132] RasterLayer's smooth opt-in persists under the itemID() group (NOT
// settingsKey()) — this round-trip catches a future group/key mismatch.
TEST(RangePersist, RasterSmoothInterpolationRoundTrips)
{
  QSettings().clear();
  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString file = dir.filePath("chart.tif");   // need not exist for this API

  {
    auto* layer = new TestableRasterLayer(layers, file);
    EXPECT_FALSE(layer->smoothInterpolation());   // faithful-QA default
    layer->setSmoothInterpolation(true);          // persists via writeSettings
  }
  {
    auto* layer = new TestableRasterLayer(layers, file);
    layer->readSettings();
    EXPECT_TRUE(layer->smoothInterpolation())
        << "persisted smooth opt-in must restore from the itemID() group";
  }
}

TEST(RangePersist, RasterAutoRoundTrips)
{
  QSettings().clear();
  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString file = dir.filePath("chart.tif");

  {
    auto* layer = new TestableRasterLayer(layers, file);
    layer->writeSettings();
  }
  {
    auto* layer = new TestableRasterLayer(layers, file);
    layer->readSettings();
    EXPECT_EQ(layer->rangeMode(), RangeMode::Auto);
    // An Auto round-trip must leave the model tracking the data, not pin a manual
    // override; with no raster loaded it stays at the RangeModel default extents.
    EXPECT_FLOAT_EQ(layer->rangeLo(), 0.0f);
    EXPECT_FLOAT_EQ(layer->rangeHi(), 1.0f);
  }
}

// --------------------------- SonarLiveCacheLayer -----------------------------

TEST(RangePersist, SonarDefaultOverrideReset)
{
  QSettings().clear();
  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);

  // A null ROS node keeps this GL/ROS-free: subscribeCatalog() no-ops with no node.
  auto* layer = new TestableSonarLiveCacheLayer(layers, nullptr, "/test_source");
  EXPECT_EQ(layer->rangeMode(), RangeMode::Auto);

  layer->setRangeOverride(3.0f, 9.0f);
  EXPECT_EQ(layer->rangeMode(), RangeMode::Manual);
  EXPECT_FLOAT_EQ(layer->rangeLo(), 3.0f);
  EXPECT_FLOAT_EQ(layer->rangeHi(), 9.0f);

  layer->resetRangeToAuto();
  EXPECT_EQ(layer->rangeMode(), RangeMode::Auto);
}

TEST(RangePersist, SonarManualRoundTrips)
{
  QSettings().clear();
  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);

  // SonarLiveCacheLayer groups under settingsKey() = "live:" + the source namespace,
  // so two layers over the same namespace share a settings group.
  {
    auto* layer = new TestableSonarLiveCacheLayer(layers, nullptr, "/test_source");
    layer->setRangeOverride(3.0f, 9.0f);
    layer->writeSettings();
  }
  {
    auto* layer = new TestableSonarLiveCacheLayer(layers, nullptr, "/test_source");
    layer->readSettings();
    EXPECT_EQ(layer->rangeMode(), RangeMode::Manual);
    EXPECT_FLOAT_EQ(layer->rangeLo(), 3.0f);
    EXPECT_FLOAT_EQ(layer->rangeHi(), 9.0f);
  }
}

TEST(RangePersist, SonarAutoRoundTrips)
{
  QSettings().clear();
  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);

  {
    auto* layer = new TestableSonarLiveCacheLayer(layers, nullptr, "/test_source");
    layer->writeSettings();
  }
  {
    auto* layer = new TestableSonarLiveCacheLayer(layers, nullptr, "/test_source");
    layer->readSettings();
    EXPECT_EQ(layer->rangeMode(), RangeMode::Auto);
    // An Auto round-trip must leave the model tracking the data, not pin a manual
    // override; with no source loaded it stays at the RangeModel default extents.
    EXPECT_FLOAT_EQ(layer->rangeLo(), 0.0f);
    EXPECT_FLOAT_EQ(layer->rangeHi(), 1.0f);
  }
}

// [camp#132] SonarLiveCacheLayer's smooth opt-in persists under its settingsKey()
// group ("live:" + namespace) — round-trip catches a future group/key mismatch.
TEST(RangePersist, LiveSmoothInterpolationRoundTrips)
{
  QSettings().clear();
  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);

  {
    auto* layer = new TestableSonarLiveCacheLayer(layers, nullptr, "/test_source");
    EXPECT_FALSE(layer->smoothInterpolation());   // faithful-QA default
    layer->setSmoothInterpolation(true);          // persists via writeSettings
  }
  {
    auto* layer = new TestableSonarLiveCacheLayer(layers, nullptr, "/test_source");
    layer->readSettings();
    EXPECT_TRUE(layer->smoothInterpolation())
        << "persisted smooth opt-in must restore from the settingsKey() group";
  }
}

// [camp#239 / ADR-0017] Uncertainty-driven opacity on the live layer: opt-in (default
// OFF with the shipped IHO S-44 Order 1a standard), persisted under settingsKey().
TEST(RangePersist, LiveOpacityDefaultsOffAndRoundTrips)
{
  QSettings().clear();
  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);

  {
    auto* layer = new TestableSonarLiveCacheLayer(layers, nullptr, "/test_source");
    EXPECT_FALSE(layer->opacityMode()) << "opt-in: default behaviour must be unchanged";
    EXPECT_EQ(layer->opacityBandName(), "uncertainty");
    const auto& r = layer->opacityRamp();
    EXPECT_EQ(r.kind, camp::raster::OpacityRamp::Kind::Standard);
    EXPECT_FLOAT_EQ(r.a, 0.5f);
    EXPECT_FLOAT_EQ(r.b, 0.013f);
    EXPECT_FLOAT_EQ(r.multiple, 3.0f);

    EXPECT_TRUE(layer->setOpacityMode(true));
    camp::raster::OpacityRamp manual;
    manual.kind = camp::raster::OpacityRamp::Kind::Manual;
    manual.lo = 0.8f;
    manual.hi = 2.4f;
    layer->setOpacityRamp(manual);
  }
  {
    auto* layer = new TestableSonarLiveCacheLayer(layers, nullptr, "/test_source");
    layer->readSettings();
    EXPECT_TRUE(layer->opacityMode());
    EXPECT_EQ(layer->opacityRamp().kind, camp::raster::OpacityRamp::Kind::Manual);
    EXPECT_FLOAT_EQ(layer->opacityRamp().lo, 0.8f);
    EXPECT_FLOAT_EQ(layer->opacityRamp().hi, 2.4f);
  }
}

TEST(RangePersist, LiveOpacityStandardParametersRoundTrip)
{
  QSettings().clear();
  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);
  {
    auto* layer = new TestableSonarLiveCacheLayer(layers, nullptr, "/test_source");
    camp::raster::OpacityRamp order1b;   // Standard kind, other parameters
    order1b.a = 0.5f;
    order1b.b = 0.013f;
    order1b.multiple = 2.0f;
    layer->setOpacityRamp(order1b);
  }
  {
    auto* layer = new TestableSonarLiveCacheLayer(layers, nullptr, "/test_source");
    layer->readSettings();
    EXPECT_EQ(layer->opacityRamp().kind, camp::raster::OpacityRamp::Kind::Standard);
    EXPECT_FLOAT_EQ(layer->opacityRamp().multiple, 2.0f);
  }
}

// A corrupt/nonsensical persisted ramp must not render garbage: it falls back to the
// shipped standard. An absent key set restores today's behaviour (mode off).
TEST(RangePersist, LiveOpacityCorruptRampFallsBackToTheShippedStandard)
{
  QSettings().clear();
  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);
  auto* layer = new TestableSonarLiveCacheLayer(layers, nullptr, "/test_source");
  {
    QSettings settings;
    settings.beginGroup("MapItem");
    settings.beginGroup(layer->settingsKey());
    settings.setValue("opacity_mode", true);
    settings.setValue("opacity_ramp_kind", "manual");
    settings.setValue("opacity_lo", "nan");
    settings.setValue("opacity_hi", -4.0);
    settings.setValue("opacity_multiple", 0.0);
  }
  layer->readSettings();
  EXPECT_EQ(layer->opacityRamp().kind, camp::raster::OpacityRamp::Kind::Standard);
  EXPECT_FLOAT_EQ(layer->opacityRamp().a, 0.5f);
  EXPECT_FLOAT_EQ(layer->opacityRamp().multiple, 3.0f);
}

// A garbled number, or a Manual ramp missing a bound, must not restore as [0, 0] (which
// would hide every positive-uncertainty cell with no explanation): the whole ramp falls
// back to the shipped standard.
TEST(RangePersist, LiveOpacityGarbledOrIncompleteRampFallsBack)
{
  struct Case
  {
    const char * name;
    std::map<QString, QVariant> keys;
  };
  const std::vector<Case> cases = {
    {"garbled a", {{"opacity_ramp_kind", "standard"}, {"opacity_a", "banana"}}},
    {"manual without bounds", {{"opacity_ramp_kind", "manual"}}},
    {"manual one bound", {{"opacity_ramp_kind", "manual"}, {"opacity_lo", 1.0}}},
    {"manual garbled hi", {{"opacity_ramp_kind", "manual"}, {"opacity_lo", 1.0},
                           {"opacity_hi", "x"}}},
    {"manual hi <= lo", {{"opacity_ramp_kind", "manual"}, {"opacity_lo", 2.0},
                         {"opacity_hi", 2.0}}},
  };
  for(const Case& c : cases)
  {
    QSettings().clear();
    Map map;
    camp::map::LayerList* layers = map.topLevelLayers();
    ASSERT_NE(layers, nullptr);
    auto* layer = new TestableSonarLiveCacheLayer(layers, nullptr, "/test_source");
    {
      QSettings settings;
      settings.beginGroup("MapItem");
      settings.beginGroup(layer->settingsKey());
      for(const auto& kv : c.keys)
        settings.setValue(kv.first, kv.second);
    }
    layer->readSettings();
    EXPECT_EQ(layer->opacityRamp().kind, camp::raster::OpacityRamp::Kind::Standard) << c.name;
    EXPECT_FLOAT_EQ(layer->opacityRamp().a, 0.5f) << c.name;
  }
}

// The Standard ramp reads the COLOUR band's value as the cell's depth, so it is only
// applied with the depth band as colour band; a Manual ramp works with any colour band.
TEST(RangePersist, LiveOpacityStandardRampNeedsTheDepthColourBand)
{
  QSettings().clear();
  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);
  auto* layer = new TestableSonarLiveCacheLayer(layers, nullptr, "/test_source");
  auto write_band = [&](const char * band) {
    QSettings settings;
    settings.beginGroup("MapItem");
    settings.beginGroup(layer->settingsKey());
    settings.setValue("band", band);
    settings.setValue("opacity_mode", true);
  };
  write_band("backscatter");
  layer->readSettings();
  EXPECT_TRUE(layer->opacityMode());
  EXPECT_FALSE(layer->opacityActive()) << "Standard ramp + non-depth colour band";
  camp::raster::OpacityRamp manual;
  manual.kind = camp::raster::OpacityRamp::Kind::Manual;
  manual.lo = 1.0f;
  manual.hi = 2.0f;
  layer->setOpacityRamp(manual);
  EXPECT_TRUE(layer->opacityActive()) << "Manual ramp is depth-independent";
  layer->setOpacityRamp(camp::raster::OpacityRamp());   // Standard (persists)
  write_band("depth");                                  // then overwrite the stored band
  layer->readSettings();
  EXPECT_TRUE(layer->opacityActive());
}

// Colour band == opacity band is meaningless: it restores with the mode OFF.
TEST(RangePersist, LiveOpacityWithSameColourAndOpacityBandRestoresOff)
{
  QSettings().clear();
  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);
  auto* layer = new TestableSonarLiveCacheLayer(layers, nullptr, "/test_source");
  {
    QSettings settings;
    settings.beginGroup("MapItem");
    settings.beginGroup(layer->settingsKey());
    settings.setValue("band", "uncertainty");
    settings.setValue("opacity_mode", true);
  }
  layer->readSettings();
  EXPECT_FALSE(layer->opacityMode());
  EXPECT_FALSE(layer->opacityActive());
}

// [camp#181 / ADR-0015] Same anchor round-trip on RasterLayer, whose settings group
// is itemID() rather than settingsKey().
TEST(RangePersist, RasterShorelineAnchorRoundTrips)
{
  QSettings().clear();
  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString file = dir.filePath("chart.tif");

  {
    auto* layer = new TestableRasterLayer(layers, file);
    EXPECT_EQ(layer->shorelineAnchorMode(), Source::None);
    layer->applyShorelineAnchor(Source::Manual, -28.038);
  }
  {
    auto* layer = new TestableRasterLayer(layers, file);
    layer->readSettings();
    EXPECT_EQ(layer->shorelineAnchorMode(), Source::Manual);
    ASSERT_TRUE(layer->shorelineManualAnchor().has_value());
    EXPECT_DOUBLE_EQ(*layer->shorelineManualAnchor(), -28.038);
  }
}

TEST(RangePersist, RasterAbsentShorelineAnchorRestoresUnanchoredNotZero)
{
  QSettings().clear();
  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString file = dir.filePath("chart.tif");

  {
    auto* layer = new TestableRasterLayer(layers, file);
    layer->writeSettings();
  }
  {
    auto* layer = new TestableRasterLayer(layers, file);
    layer->readSettings();
    EXPECT_EQ(layer->shorelineAnchorMode(), Source::None);
    EXPECT_FALSE(layer->resolvedShorelineAnchor().has_value())
        << "ADR-0015 D6: an absent anchor is unanchored, never 0.0";
  }
}

// [camp#181 / ADR-0015 D6] The mode-not-inferred contract on RasterLayer too — its
// settings group is itemID() rather than settingsKey(), so the two paths are
// separate code and each needs its own guard.
TEST(RangePersist, RasterNoneWithARetainedManualValueRestoresUnanchored)
{
  QSettings().clear();
  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString file = dir.filePath("chart.tif");

  {
    auto* layer = new TestableRasterLayer(layers, file);
    layer->applyShorelineAnchor(Source::Manual, -28.038);
    layer->applyShorelineAnchor(Source::None, -28.038);   // value kept, anchor off
  }
  {
    auto* layer = new TestableRasterLayer(layers, file);
    layer->readSettings();
    EXPECT_EQ(layer->shorelineAnchorMode(), Source::None)
        << "None must restore as None even with a manual value still stored";
    EXPECT_FALSE(layer->resolvedShorelineAnchor().has_value())
        << "the retained value must not resolve: the operator chose unanchored";
    ASSERT_TRUE(layer->shorelineManualAnchor().has_value());
    EXPECT_DOUBLE_EQ(*layer->shorelineManualAnchor(), -28.038);
  }
}

int main(int argc, char** argv)
{
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(argc, argv);
  QCoreApplication::setOrganizationName("camp_test");
  QCoreApplication::setApplicationName("test_range_persist");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
