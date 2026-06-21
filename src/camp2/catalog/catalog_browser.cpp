#include "catalog_browser.h"

#include "catalog_item.h"
#include "catalog_model.h"

#include <QFileDialog>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QPushButton>
#include <QTreeView>
#include <QVBoxLayout>

namespace camp
{
namespace catalog
{

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
  auto node = source->discover(seed);
  if(node)
  {
    model_->addTopLevel(std::move(node));
    view_->expandToDepth(0);
  }
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
