# Plan: WMS ServiceExceptionReport cache poisoning — decode gate

## Issue

https://github.com/rolker/camp/issues/177

## Context

`CachedFileLoader::downloadFinished` writes any body to disk when
`reply->error() == QNetworkReply::NoError`, with no content validation.
WMS servers return errors as HTTP 200 + XML `ServiceExceptionReport`, so a
transient GEBCO blip writes an XML file to `<cache>/<z>/<x>/<y>.png`.
`CachedTileLoader::dataLoaded` tries `QPixmap::loadFromData(data, "png")`,
which silently returns a null pixmap — the tile shows blank. Because GEBCO sets
no `refresh_ms`, `invalidateCache()` is never called and the poisoned entry
persists until LRU eviction (#98).

**Operator decisions (2026-07-28):**
- BOTH a Content-Type header check (fast reject) and a QImage decode gate
  (the authority) are required.
- The header check is an early-out only; it must not by itself reject bodies
  with odd-but-valid Content-Type (e.g. `application/octet-stream`). Decode
  decides for anything that passes the header check.
- `&EXCEPTIONS=BLANK` is out of scope: a blank error image would decode
  successfully and be cached as a persistent "valid" blank tile.

## Approach

1. **Extract `CachedFileLoader::isAcceptableImageBody` (static, testable)** —
   add a `static bool isAcceptableImageBody(const QByteArray& data,
   const QString& content_type)` method that encodes both checks:
   - Fast reject: if `content_type` is non-empty and does not start with
     `"image/"`, return false immediately (non-image MIME types such as
     `text/xml`, `application/xml`, `application/vnd.ogc.se_xml` are skipped
     without decoding).
   - Decode gate (authority): `QImage check; return check.loadFromData(data);`
     Any body that passes the header check (or has no/odd Content-Type) must
     decode as an image to be cached.

2. **Gate the cache write and `dataLoaded` emit in `downloadFinished`** —
   after reading `data` (line 85), call `isAcceptableImageBody` before the
   disk-write block (lines 87–113) and before `emit client->dataLoaded`.
   If it returns false, skip both — no cache file, no pixmap emit, no tile
   displayed. Log a `qDebug` line for observability.

3. **Add `#include <QImage>` to `cached_file_loader.cpp`.**

4. **Add test `test/test_cached_file_loader_validation.cpp`** covering:
   - XML body + `text/xml` Content-Type → false (fast reject)
   - XML body + `application/xml` Content-Type → false (fast reject)
   - Valid PNG bytes + `image/png` → true (fast path + decode pass)
   - Valid PNG bytes + `application/octet-stream` → true (odd type, decode decides)
   - XML bytes + `image/png` Content-Type → false (header passes, decode gates)
   Use a minimal valid PNG constructed in-memory via `QImage`/`QBuffer`/`QImageWriter`.

5. **Register the test in `CMakeLists.txt`** with `ament_add_gtest`, linking
   `camp_map Qt5::Core Qt5::Gui`.

## Files to Change

| File | Change |
|------|--------|
| `src/camp_map/util/cached_file_loader.h` | Declare `static bool isAcceptableImageBody(const QByteArray&, const QString&)` |
| `src/camp_map/util/cached_file_loader.cpp` | Add `#include <QImage>`; implement `isAcceptableImageBody`; call it in `downloadFinished` before cache write + emit |
| `test/test_cached_file_loader_validation.cpp` | New test: 5 cases for `isAcceptableImageBody` (fast reject, decode authority, odd-type pass-through) |
| `CMakeLists.txt` | Register `test_cached_file_loader_validation` with `ament_add_gtest` |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| A change includes its consequences | Test coverage added; all existing tile sources (OSM, WMTS, radar) must continue passing |
| Only what's needed | `&EXCEPTIONS=BLANK` excluded per operator; no new infrastructure |
| Test what breaks | Unit test exercises the exact failure mode (XML body passed off as image), not just happy-path coverage |
| Enforce over document | Gate is in production code path, not a comment |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0001 (adopt ADRs) | No | Rationale is fully documented in the issue body + operator decisions; no new cross-cutting policy warranting a separate ADR |
| ADR-0018 (local-first CI) | Yes | Build and run tests locally before push; no branch protection bypass |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `CachedFileLoader::downloadFinished` behavior | All callers that rely on `dataLoaded` being emitted for non-image 200 responses | Yes — no callers expect XML bodies to succeed; the only callers are tile loaders and WMTS capabilities (which use separate loaders) |
| `isAcceptableImageBody` static function | Expose in header for test access | Yes |

## Open Questions

- [ ] No open questions — plan is review-plan-ready.

## Estimated Scope

Single PR.
