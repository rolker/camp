#include "catalog_browser.h"

#include "catalog_item.h"
#include "catalog_model.h"

#include <QAction>
#include <QDir>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QMenu>
#include <QPushButton>
#include <QSettings>
#include <QTreeView>
#include <QVBoxLayout>

namespace camp
{
namespace catalog
{

namespace
{
// Generic persisted-seeds key (not GGGS-specific — the browser is source-
// agnostic). Each entry is "sourceId\troot"; a tab delimiter is safe because
// store paths may contain commas but not tabs.
const char* const kSeedsKey = "CatalogBrowser/seeds";
const QChar kSeedDelim = QLatin1Char('\t');
}  // namespace

CatalogBrowser::CatalogBrowser(QWidget* parent):
  QWidget(parent),
  model_(new CatalogModel(this)),
  view_(new QTreeView(this)),
  open_button_(new QPushButton(tr("Open store…"), this)),
  add_button_(new QPushButton(tr("Add to map"), this))
{
  view_->setModel(model_);
  view_->setHeaderHidden(true);
  view_->setSelectionMode(QAbstractItemView::SingleSelection);
  view_->setContextMenuPolicy(Qt::CustomContextMenu);   // "Remove from browser"
  add_button_->setEnabled(false);   // enabled only when a leaf is selected

  auto* buttons = new QHBoxLayout;
  buttons->addWidget(open_button_);
  buttons->addStretch(1);
  buttons->addWidget(add_button_);

  auto* layout = new QVBoxLayout(this);
  layout->addWidget(view_);
  layout->addLayout(buttons);

  connect(open_button_, &QPushButton::clicked, this, &CatalogBrowser::openStore);
  connect(add_button_, &QPushButton::clicked, this, &CatalogBrowser::addSelected);
  connect(view_->selectionModel(), &QItemSelectionModel::selectionChanged,
          this, &CatalogBrowser::onSelectionChanged);
  connect(view_, &QTreeView::doubleClicked, this, &CatalogBrowser::addSelected);
  connect(view_, &QTreeView::customContextMenuRequested,
          this, &CatalogBrowser::onContextMenu);
}

CatalogBrowser::~CatalogBrowser() = default;

void CatalogBrowser::addSource(std::unique_ptr<CatalogSource> source)
{
  if(source)
    sources_.push_back(std::move(source));
}

CatalogSource* CatalogBrowser::sourceForId(const QString& id) const
{
  for(const auto& source : sources_)
    if(source->sourceId() == id)
      return source.get();
  return nullptr;
}

void CatalogBrowser::openStore()
{
  if(sources_.empty())
    return;
  // Seed-then-browse (3a decision): pick a store root, then browse that root's
  // catalog. 3a ships a single directory-based source, so the first source seeds.
  CatalogSource* source = sources_.front().get();
  const QString seed = QFileDialog::getExistingDirectory(this, source->seedLabel());
  if(seed.isEmpty())
    return;
  seedRoot(source, seed);   // discover + add + persist (or re-select if present)
}

bool CatalogBrowser::seedRoot(CatalogSource* source, const QString& root)
{
  if(!source || root.isEmpty())
    return false;

  // Dedup-on-select: an already-browsed (source, root) is re-selected, not added
  // a second time (mirrors GggsStoreSource::instantiate's flat-layer dedup).
  for(size_t i = 0; i < seeds_.size(); ++i)
    if(seeds_[i].sourceId == source->sourceId() && seeds_[i].root == root)
    {
      selectTopLevel(static_cast<int>(i));
      return true;
    }

  auto node = source->discover(root);
  if(!node)
    return false;

  model_->addTopLevel(std::move(node));   // appends exactly one top-level row...
  seeds_.push_back(Seed{source->sourceId(), root});   // ...kept parallel here
  persistSeeds();
  selectTopLevel(static_cast<int>(seeds_.size()) - 1);
  return true;
}

void CatalogBrowser::restoreSeeds()
{
  const QStringList stored = QSettings().value(kSeedsKey).toStringList();
  if(stored.isEmpty())
    return;

  bool pruned = false;   // dropped a stale/duplicate/malformed entry -> rewrite
  for(const QString& entry : stored)
  {
    const int tab = entry.indexOf(kSeedDelim);
    if(tab < 0)   // malformed (pre-delimiter or hand-edited) -> drop
    {
      pruned = true;
      continue;
    }
    const QString source_id = entry.left(tab);
    const QString root = entry.mid(tab + 1);

    CatalogSource* source = sourceForId(source_id);
    // Skip-missing (like the flat-layer restore): unknown source or vanished root.
    if(!source || root.isEmpty() || !QDir(root).exists())
    {
      pruned = true;
      continue;
    }

    // Dedup against already-present seeds (e.g. a duplicate persisted entry, or a
    // root already seeded earlier this session).
    bool present = false;
    for(const Seed& s : seeds_)
      if(s.sourceId == source_id && s.root == root)
      {
        present = true;
        break;
      }
    if(present)
    {
      pruned = true;   // collapse the duplicate out of the persisted list
      continue;
    }

    auto node = source->discover(root);
    if(!node)   // existed last session, browsable-empty now -> drop
    {
      pruned = true;
      continue;
    }
    model_->addTopLevel(std::move(node));
    seeds_.push_back(Seed{source_id, root});
  }
  view_->expandToDepth(0);

  // Rewrite once if anything was dropped so vanished/duplicate roots don't
  // linger; a clean restore leaves the persisted key byte-for-byte untouched
  // (restoring isn't an operator change).
  if(pruned)
    persistSeeds();
}

void CatalogBrowser::removeSeed(int row)
{
  if(row < 0 || row >= static_cast<int>(seeds_.size()))
    return;
  model_->removeTopLevel(row);                 // drop the parallel model row...
  seeds_.erase(seeds_.begin() + row);          // ...and its seed in lockstep
  persistSeeds();
}

int CatalogBrowser::topLevelCount() const
{
  return model_->rowCount(QModelIndex());
}

void CatalogBrowser::selectTopLevel(int seed_index)
{
  const QModelIndex index = model_->index(seed_index, 0, QModelIndex());
  if(!index.isValid())
    return;
  view_->setCurrentIndex(index);
  view_->expand(index);
  view_->scrollTo(index);
}

void CatalogBrowser::persistSeeds() const
{
  QStringList list;
  list.reserve(static_cast<int>(seeds_.size()));
  for(const Seed& s : seeds_)
    list.append(s.sourceId + kSeedDelim + s.root);

  QSettings settings;
  if(list.isEmpty())
    settings.remove(kSeedsKey);   // avoid an @Invalid() empty-list round-trip
  else
    settings.setValue(kSeedsKey, list);
}

void CatalogBrowser::onContextMenu(const QPoint& pos)
{
  const QModelIndex index = view_->indexAt(pos);
  // Only a top-level node (a browsed store root) is removable from the browser;
  // leaves/groups inside a root are not seeds.
  if(!index.isValid() || index.parent().isValid())
    return;
  const int row = index.row();

  QMenu menu;
  QAction* remove = menu.addAction(tr("Remove from browser"));
  if(menu.exec(view_->viewport()->mapToGlobal(pos)) == remove)
    removeSeed(row);
}

void CatalogBrowser::onSelectionChanged(const QItemSelection&, const QItemSelection&)
{
  const QModelIndexList selected = view_->selectionModel()->selectedIndexes();
  const CatalogItem* item = selected.isEmpty()
    ? nullptr : model_->itemForIndex(selected.first());
  add_button_->setEnabled(item && item->isLeaf());
}

void CatalogBrowser::addSelected()
{
  if(!target_)
    return;
  const QModelIndexList selected = view_->selectionModel()->selectedIndexes();
  if(selected.isEmpty())
    return;
  const CatalogItem* item = model_->itemForIndex(selected.first());
  if(!item || !item->isLeaf())
    return;
  CatalogSource* source = sourceForId(item->sourceId());
  if(!source)
    return;
  if(map::Layer* layer = source->instantiate(target_, item->key()))
    emit layerAdded(layer);
}

}  // namespace catalog
}  // namespace camp
