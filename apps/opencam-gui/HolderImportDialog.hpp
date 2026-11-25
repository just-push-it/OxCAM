// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Turns a holder's solid model (a vendor STEP file) into a holder outline: the axis, the end the
// tool comes out of and the gauge line, with a preview.

#pragma once

#include <ocam/cad/Revolve.hpp>
#include <ocam/tooling/Tool.hpp>

#include <QDialog>

class QComboBox;
class QLabel;

namespace ocam::gui {

class LengthSpinBox;
class ToolProfileView;

class HolderImportDialog : public QDialog {
    Q_OBJECT

public:
    HolderImportDialog(cad::Model model, const QString& name, QWidget* parent = nullptr);

    /// The holder as shown (a new id, named after the file).
    tooling::Holder holder() const;

private:
    /// Recomputes the outline; `keepLength` keeps the entered gauge length, else the suggested
    /// one is taken.
    void recompute(bool keepLength);

    cad::Model model_;
    QString name_;
    cad::HolderOutline outline_;
    bool loading_ = false;
    QComboBox* axis_ = nullptr;
    QComboBox* toolEnd_ = nullptr;
    LengthSpinBox* length_ = nullptr;
    QLabel* info_ = nullptr;
    ToolProfileView* preview_ = nullptr;
};

} // namespace ocam::gui
