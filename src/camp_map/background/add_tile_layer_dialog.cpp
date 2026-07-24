#include "add_tile_layer_dialog.h"

#include <QComboBox>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QVBoxLayout>

namespace camp
{
namespace background
{

AddTileLayerDialog::AddTileLayerDialog(QWidget* parent):
  QDialog(parent), presets_(builtinPresets())
{
  setWindowTitle(tr("Add tile layer"));

  auto layout = new QVBoxLayout(this);

  preset_list_ = new QListWidget(this);
  for(const auto& preset : presets_)
  {
    auto item = new QListWidgetItem(preset.name, preset_list_);
    if(!preset.enabled)
    {
      // Inert preset (e.g. WMS until #118): visible so the operator sees the
      // full table, but not selectable.
      item->setFlags(item->flags() & ~(Qt::ItemIsEnabled | Qt::ItemIsSelectable));
      item->setToolTip(preset.note);
    }
  }
  // "Custom" reveals the editable fields below.
  preset_list_->addItem(tr("Custom..."));
  layout->addWidget(preset_list_);

  auto form = new QFormLayout;
  name_edit_ = new QLineEdit(this);
  form->addRow(tr("Name"), name_edit_);
  type_combo_ = new QComboBox(this);
  type_combo_->addItem("xyz");
  type_combo_->addItem("wmts");
  form->addRow(tr("Type"), type_combo_);
  url_edit_ = new QLineEdit(this);
  form->addRow(tr("URL"), url_edit_);
  layout->addLayout(form);

  buttons_ = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
  connect(buttons_, &QDialogButtonBox::accepted, this, &QDialog::accept);
  connect(buttons_, &QDialogButtonBox::rejected, this, &QDialog::reject);
  layout->addWidget(buttons_);

  connect(preset_list_, &QListWidget::currentRowChanged, this, &AddTileLayerDialog::updateFields);
  connect(name_edit_, &QLineEdit::textChanged, this, &AddTileLayerDialog::updateFields);
  connect(url_edit_, &QLineEdit::textChanged, this, &AddTileLayerDialog::updateFields);

  preset_list_->setCurrentRow(0);
  updateFields();
}

bool AddTileLayerDialog::customSelected() const
{
  return preset_list_->currentRow() == presets_.size();
}

void AddTileLayerDialog::updateFields()
{
  const int row = preset_list_->currentRow();
  const bool custom = customSelected();
  if(!custom && row >= 0 && row < presets_.size())
  {
    // Prefill from the preset; fields stay read-only so the preset's identity
    // (and its persisted reconstruction) can't silently drift from its name.
    name_edit_->setText(presets_[row].name);
    type_combo_->setCurrentText(presets_[row].type);
    url_edit_->setText(presets_[row].url);
  }
  name_edit_->setReadOnly(!custom);
  url_edit_->setReadOnly(!custom);
  type_combo_->setEnabled(custom);

  const bool acceptable = row >= 0 &&
    !name_edit_->text().trimmed().isEmpty() && !url_edit_->text().trimmed().isEmpty();
  buttons_->button(QDialogButtonBox::Ok)->setEnabled(acceptable);
}

TileLayerPreset AddTileLayerDialog::selection() const
{
  const int row = preset_list_->currentRow();
  if(!customSelected() && row >= 0 && row < presets_.size())
    return presets_[row];
  TileLayerPreset custom;
  custom.name = name_edit_->text().trimmed();
  custom.type = type_combo_->currentText();
  custom.url = url_edit_->text().trimmed();
  return custom;
}

} // namespace background
} // namespace camp
