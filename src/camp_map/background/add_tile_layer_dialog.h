#ifndef BACKGROUND_ADD_TILE_LAYER_DIALOG_H
#define BACKGROUND_ADD_TILE_LAYER_DIALOG_H

#include <QDialog>

#include "tile_layer_presets.h"

class QComboBox;
class QDialogButtonBox;
class QLineEdit;
class QListWidget;

namespace camp
{
namespace background
{

// [camp#117] "Add tile layer" dialog: the built-in preset list plus a "Custom"
// entry that reveals editable name/type/URL fields. Presets with
// enabled == false (WMS sources, until #118) are shown greyed-out with their
// note as the tooltip and cannot be accepted. selection() returns the chosen
// preset (or the custom fields as one) after the dialog is accepted.
class AddTileLayerDialog: public QDialog
{
  Q_OBJECT
public:
  AddTileLayerDialog(QWidget* parent = nullptr);

  TileLayerPreset selection() const;

private slots:
  void updateFields();

private:
  bool customSelected() const;

  QVector<TileLayerPreset> presets_;
  QListWidget* preset_list_;
  QLineEdit* name_edit_;
  QComboBox* type_combo_;
  QLineEdit* url_edit_;
  QDialogButtonBox* buttons_;
};

} // namespace background
} // namespace camp

#endif
