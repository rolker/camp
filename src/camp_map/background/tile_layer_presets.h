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
  QString type;                  // "xyz" | "wmts" | "wms" (wms inert until #118)
  QString url;
  QString layer_id;              // WMTS: advertised layer id ({} = first)
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

  // [#99] Weather radar (NEXRAD base reflectivity) as a stacked, auto-refreshing
  // overlay. Source: NOAA NEXRAD base reflectivity, redistributed as XYZ
  // Web-Mercator (EPSG:3857) tiles by Iowa State University's Environmental
  // Mesonet (IEM). nowCOAST itself serves this product only via WMS (dynamic
  // GetMap), not tiled WMTS, so it does not fit the MapTiles z/x/y path; IEM's
  // tile cache does (same NEXRAD origin). The "n0q" product alias always serves
  // the LATEST frame, so the 5-minute refresh genuinely fetches fresh imagery.
  // Default OFF, ~0.65 opacity so it reads as a transparent overlay, refreshed
  // every 5 minutes since radar imagery is time-varying. On a fetch failure the
  // tile stays blank (graceful degradation in CachedFileLoader). #118 tracks
  // switching this to authoritative NOAA nowCOAST once WMS support lands.
  TileLayerPreset radar;
  radar.name = "nexrad_radar";
  radar.type = "xyz";
  radar.url = "https://mesonet.agron.iastate.edu/cache/tile.py/1.0.0/nexrad-n0q-900913/";
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

  // GEBCO global bathymetry — GEBCO publishes WMS only (no official WMTS), so
  // this preset is inert (greyed out) until #118 lands the WMS layer type
  // (operator decision 2026-07-24: keep the preset table complete in one place).
  TileLayerPreset gebco;
  gebco.name = "GEBCO_bathymetry";
  gebco.type = "wms";
  gebco.url = "https://wms.gebco.net/mapserv";
  gebco.layer_id = "GEBCO_LATEST";
  gebco.enabled = false;
  gebco.note = "GEBCO serves WMS only — requires WMS layer support (#118)";
  presets.append(gebco);

  return presets;
}

} // namespace background
} // namespace camp

#endif
