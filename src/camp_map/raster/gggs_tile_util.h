#ifndef RASTER_GGGS_TILE_UTIL_H
#define RASTER_GGGS_TILE_UTIL_H

#include <QRegularExpression>
#include <QString>

namespace camp
{
namespace raster
{

/// [camp#112] True iff @p filename is a base value tile `<level>_<row>_<col>.tif`
/// (or `.tiff`). The positive, anchored full-match pattern is the canonical
/// value-tile name shared with the stores' own tile-naming convention: exactly
/// three underscore-separated digit groups then `.tif`/`.tiff`. Companion tiles
/// (`<level>_<row>_<col>_time.tif` Int64, `<level>_<row>_<col>_source.tif`
/// uint16) carry a 4th `_time`/`_source` component and so fail the match — and
/// any future companion suffix is excluded for free by not matching, without a
/// denylist to maintain. Match @p filename only (what QDir::entryList returns),
/// never a full path.
///
/// [camp#112] Case-insensitive: the scan-site globs (`*.tif`/`*.tiff`) can admit
/// an upper/mixed-case extension (`.TIF`/`.TIFF`), so the pattern matches case-
/// insensitively too — otherwise such a value tile would pass the glob but be
/// silently dropped here. (Producers emit lowercase today; this keeps the filter
/// from diverging from the globs if that ever changes.)
inline bool isValueTile(const QString& filename)
{
  static const QRegularExpression re(
    QRegularExpression::anchoredPattern(QStringLiteral("\\d+_\\d+_\\d+\\.tiff?")),
    QRegularExpression::CaseInsensitiveOption);
  return re.match(filename).hasMatch();
}

/// [camp#180] Parse the LEVEL (the first digit group) from a value-tile basename
/// `<level>_<row>_<col>.tif`. Returns -1 if @p filename is not a value tile.
/// GggsTileLayer::getElevation() uses it to query the finest (highest-level)
/// covering tile first, independent of the rendered LOD. Match the basename only
/// (as QDir::entryList / QFileInfo::fileName return), never a full path — a path
/// separator makes isValueTile() (and so this) return no match.
inline int tileLevel(const QString& filename)
{
  if(!isValueTile(filename))
    return -1;
  // isValueTile() guarantees the leading group is all digits, so section() +
  // toInt() is unambiguous (the tail is stripped at the first '_').
  return filename.section('_', 0, 0).toInt();
}

}  // namespace raster
}  // namespace camp

#endif
