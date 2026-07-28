#ifndef BACKGROUND_TILE_LAYER_PRESETS_H
#define BACKGROUND_TILE_LAYER_PRESETS_H

#include <QString>
#include <QUrl>
#include <QVector>

namespace camp
{
namespace background
{

// [camp#117] A creatable tile-layer definition: the built-in preset table below
// and the "Custom" entry of AddTileLayerDialog both produce one of these. It
// carries every attribute needed to (re)construct the layer plus the preset's
// default presentation state, so e.g. the NEXRAD preset reproduces its #99
// behavior (default-off, 0.65 opacity, 5-minute refresh) when added.
struct TileLayerPreset
{
  QString name;                  // layer objectName + display name
  QString type;                  // "xyz" | "wmts" | "wms" (per-tile GetMap, #118)
  QString url;
  QString layer_id;              // WMTS: advertised layer id ({} = first); WMS: LAYERS value
  QString tile_matrix_set;       // WMTS: matrix set id ({} = auto)
  qreal opacity = 1.0;
  bool visible = true;
  int refresh_ms = 0;            // <= 0: never refresh
  bool enabled = true;           // false: shown greyed-out in the dialog
  QString note;                  // tooltip, e.g. why a preset is disabled
};

// [camp#117] QSettings schema (documented in ADR-0003 addendum). Construction
// parameters live under BackgroundTileLayers; presentation state (opacity,
// visibility) is owned by the existing MapItem/<settingsKey()> mechanism.
//   BackgroundTileLayers/seeded          — first-run sentinel; seed fires only
//                                          when absent, so an operator who
//                                          removes every layer is not re-seeded
//   BackgroundTileLayers/ids             — QStringList of layer names, in
//                                          creation order
//   BackgroundTileLayers/<enc(name)>/... — per-layer group: type, url,
//                                          refresh_ms, layer_id,
//                                          tile_matrix_set
inline QString tileLayerSeededKey() { return QStringLiteral("BackgroundTileLayers/seeded"); }
inline QString tileLayerIdsKey() { return QStringLiteral("BackgroundTileLayers/ids"); }
inline QString tileLayerRootGroup() { return QStringLiteral("BackgroundTileLayers"); }

// Percent-encode the layer name so it is a single flat child key of
// BackgroundTileLayers rather than a nested group tree (a name may contain
// '/'), mirroring GggsTileLayer::settingsKey().
inline QString tileLayerGroupKey(const QString& name)
{
  return QString::fromLatin1(QUrl::toPercentEncoding(name));
}

// The built-in preset list: the four layers that were hard-coded in
// BackgroundManager::createDefaultLayers() until #117, plus verified
// bathymetry sources. Names of the former hard-coded four are kept verbatim so
// an upgrading operator's persisted per-layer state (MapItem/<settingsKey>)
// still applies when the preset is re-added.
inline QVector<TileLayerPreset> builtinPresets()
{
  QVector<TileLayerPreset> presets;

  TileLayerPreset osm;
  osm.name = "openstreetmap";
  osm.type = "xyz";
  osm.url = "https://tile.openstreetmap.org/";
  presets.append(osm);

  TileLayerPreset openseamap;
  openseamap.name = "openseamap";
  openseamap.type = "xyz";
  openseamap.url = "https://tiles.openseamap.org/seamark/";
  presets.append(openseamap);

  TileLayerPreset noaa_charts;
  noaa_charts.name = "NOAA_charts";
  noaa_charts.type = "wmts";
  noaa_charts.url = "https://gis.charttools.noaa.gov/arcgis/rest/services/MarineChart_Services/NOAACharts/MapServer/WMTS";
  presets.append(noaa_charts);

  // [#99/#118] Weather radar as a stacked, auto-refreshing overlay — now
  // authoritative NOAA nowCOAST (`base_reflectivity_mosaic`, MRMS-based,
  // time-enabled, WMS-only) via the per-tile GetMap path (ADR-0012), replacing
  // the IEM XYZ redistribution stopgap that #118 retired. Latest-frame only:
  // TIME is omitted so the server returns the newest frame; the 5-minute
  // refresh + #111 cache-buster keep it fresh. Default OFF, ~0.65 opacity so
  // it reads as a transparent overlay. On a fetch failure the tile stays blank
  // (graceful degradation in CachedFileLoader).
  // The preset name stays "nexrad_radar" for persisted-state continuity (ids
  // list + MapItem/<settingsKey>) even though the MRMS mosaic is broader than
  // NEXRAD proper.
  TileLayerPreset radar;
  radar.name = "nexrad_radar";
  radar.type = "wms";
  radar.url = "https://nowcoast.noaa.gov/geoserver/wms";
  // Workspace-qualified: GeoServer rejects the bare mosaic name with
  // LayerNotDefined (smoke-checked live 2026-07-24).
  radar.layer_id = "weather_radar:base_reflectivity_mosaic";
  radar.opacity = 0.65;
  radar.visible = false;
  radar.refresh_ms = 5 * 60 * 1000;
  presets.append(radar);

  // NOAA BlueTopo bathymetry — authoritative NOAA compiled bathymetry, served
  // as cached EPSG:3857 tiles by nowCOAST's GeoWebCache WMTS. Endpoint and
  // layer ids verified live 2026-07-24 (camp#117 addendum).
  TileLayerPreset bluetopo;
  bluetopo.name = "NOAA_bluetopo";
  bluetopo.type = "wmts";
  bluetopo.url = "https://nowcoast.noaa.gov/geoserver/gwc/service/wmts?REQUEST=GetCapabilities";
  bluetopo.layer_id = "bluetopo:bathymetry";
  bluetopo.tile_matrix_set = "EPSG:3857";
  presets.append(bluetopo);

  TileLayerPreset bluetopo_hillshade;
  bluetopo_hillshade.name = "NOAA_bluetopo_hillshade";
  bluetopo_hillshade.type = "wmts";
  bluetopo_hillshade.url = "https://nowcoast.noaa.gov/geoserver/gwc/service/wmts?REQUEST=GetCapabilities";
  bluetopo_hillshade.layer_id = "bluetopo:hillshade";
  bluetopo_hillshade.tile_matrix_set = "EPSG:3857";
  presets.append(bluetopo_hillshade);

  // GEBCO global bathymetry — GEBCO publishes WMS only (no official WMTS);
  // served via the per-tile GetMap path (#118, ADR-0012). Global ~450 m grid:
  // coarse next to BlueTopo nearshore, but worldwide coverage.
  TileLayerPreset gebco;
  gebco.name = "GEBCO_bathymetry";
  gebco.type = "wms";
  gebco.url = "https://wms.gebco.net/mapserv";
  gebco.layer_id = "GEBCO_LATEST";
  presets.append(gebco);

  return presets;
}

} // namespace background
} // namespace camp

#endif
