#ifndef RASTER_TILE_RESIDENCY_H
#define RASTER_TILE_RESIDENCY_H

#include <cstddef>
#include <cstdint>
#include <list>
#include <vector>

namespace camp
{
namespace raster
{

/// [camp#195 / uma-ADR-0013 D4] Current-frame protection for a tile-indexed
/// residency budget, as a **structural** property rather than a checked
/// condition.
///
/// D4 requires that "tiles selected in the current frame are protected
/// regardless of recency" be a property of the data structure — the sentinel-node
/// list — so that an eviction pass literally cannot reach them, instead of an
/// `if` a future edit can drop. This type is that structure, reduced to what a
/// `std::vector`-indexed tile store needs: two `std::list<std::size_t>`
/// partitions (protected / evictable) plus an index of iterators, so every
/// operation is an O(1) splice and no iterator is ever invalidated.
///
/// Usage per frame:
/// @code
///   residency.sync(tiles.size());        // admit tiles appended since last frame
///   residency.beginFrame();              // everything becomes evictable, O(1)
///   for(i : tiles the frame selected)
///     residency.protect(i);              // O(1) each, idempotent within a frame
///   ...
///   for(i : residency.candidates())      // CANNOT contain a protected index
///     consider evicting tiles[i];
/// @endcode
///
/// The eviction pass is handed `candidates()` — a const view of the evictable
/// partition alone. There is no accessor that exposes the protected partition
/// as eviction candidates, which is the guarantee: "cannot evict what this frame
/// selected" holds by construction.
///
/// Indices are positions in the owner's tile vector and must be stable;
/// `GggsTileLayer::tiles_` only ever grows (loadDirectory/rescan push_back), so
/// `sync()` is append-only by design and asserts nothing about shrinking beyond
/// ignoring it.
///
/// Deliberately Qt-free and GL-free so it can be unit-tested standalone
/// (`test/test_tile_residency.cpp`).
class TileResidency
{
public:
  /// Admit indices `[size(), count)` as evictable. Append-only: a @p count below
  /// the current size is ignored (the owner's tile vector never shrinks).
  void sync(std::size_t count)
  {
    while(where_.size() < count)
    {
      const std::size_t index = where_.size();
      where_.push_back(evictable_.insert(evictable_.end(), index));
      protected_frame_.push_back(0);   // 0 != frame_ (which starts at 1)
    }
  }

  /// Start a new frame: every protected index becomes a candidate again, in O(1)
  /// (one whole-list splice), and the frame epoch advances so `protect()` treats
  /// this frame's protections as fresh without touching the per-index vector.
  void beginFrame()
  {
    evictable_.splice(evictable_.end(), protected_);
    ++frame_;
  }

  /// Protect @p index for the current frame. O(1), idempotent within the frame.
  /// Out-of-range indices are ignored (the caller syncs first).
  void protect(std::size_t index)
  {
    if(index >= where_.size() || protected_frame_[index] == frame_)
      return;
    protected_.splice(protected_.end(), evictable_, where_[index]);
    protected_frame_[index] = frame_;
  }

  /// True if @p index is protected in the current frame.
  bool isProtected(std::size_t index) const
  {
    return index < where_.size() && protected_frame_[index] == frame_;
  }

  /// The evictable partition — the ONLY thing an eviction pass may consider.
  /// Never contains an index protected in the current frame.
  const std::list<std::size_t>& candidates() const { return evictable_; }

  /// Number of indices protected in the current frame — the working set the
  /// residency cap is floored at (uma-ADR-0013 D4: "the budget must exceed
  /// [the current-frame set] or the system thrashes by construction").
  std::size_t protectedCount() const { return protected_.size(); }

  /// Number of indices admitted so far.
  std::size_t size() const { return where_.size(); }

private:
  // The two partitions. An index lives in exactly one of them at all times; the
  // iterator recorded in where_ stays valid across every splice (std::list
  // splice preserves iterators), which is what makes protect() O(1).
  std::list<std::size_t> protected_;
  std::list<std::size_t> evictable_;
  std::vector<std::list<std::size_t>::iterator> where_;

  // Per-index frame epoch, so beginFrame() need not clear a flag vector: an
  // index is protected iff its recorded epoch equals the current one. Starts at
  // 1 so the 0 seeded by sync() means "never protected".
  std::vector<std::uint64_t> protected_frame_;
  std::uint64_t frame_ = 1;
};

}  // namespace raster
}  // namespace camp

#endif
