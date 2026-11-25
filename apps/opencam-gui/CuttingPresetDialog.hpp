// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Editor for one tool's cutting presets: the list of presets beside a form with the cutting
// data in the tool's unit, and what they give for this tool (spindle speed, feed rate, removal
// rate). The spindle speed and the feed rate can be typed too; the cutting speed and the feed
// per tooth follow.

#pragma once

#include <ocam/tooling/Tool.hpp>

#include <QDialog>

class QComboBox;
class QDoubleSpinBox;
class QFormLayout;
class QLabel;
class QLineEdit;
class QListWidget;
class QPushButton;

namespace ocam::gui {

class CuttingPresetDialog : public QDialog {
    Q_OBJECT

public:
    /// Edits a copy of `tool`'s presets, showing preset `index` first (the first when out of
    /// range).
    CuttingPresetDialog(const tooling::Tool& tool, int index, QWidget* parent = nullptr);

    const std::vector<tooling::CuttingPreset>& presets() const noexcept { return tool_.presets; }

    /// Adds a preset (the tool type's typical values) and shows it.
    void addPreset();

private:
    void rebuildList();
    void select(int row);
    void load();
    /// Shows the derived values; `all` also the field that has the focus.
    void refreshResults(bool all);
    void refreshItem();
    tooling::CuttingPreset* current();
    double lengthFactor() const;
    bool partsOff() const; ///< The tool takes a part-off, so its presets have an end feed.
    bool leadsPerRev() const; ///< The tool turns with lead moves, set per revolution.

    tooling::Tool tool_;
    int current_ = -1;
    bool loading_ = false;
    bool turning_ = false;

    QListWidget* list_ = nullptr;
    QPushButton* duplicate_ = nullptr;
    QPushButton* remove_ = nullptr;
    QPushButton* up_ = nullptr;
    QPushButton* down_ = nullptr;
    QWidget* form_ = nullptr;
    QFormLayout* speedForm_ = nullptr;
    QFormLayout* depthForm_ = nullptr;
    QLineEdit* name_ = nullptr;
    QComboBox* material_ = nullptr;
    QComboBox* coolant_ = nullptr;
    QDoubleSpinBox* surfaceSpeed_ = nullptr;
    QDoubleSpinBox* spindle_ = nullptr;
    QDoubleSpinBox* feedPerTooth_ = nullptr;
    QDoubleSpinBox* feedRate_ = nullptr;
    QDoubleSpinBox* plungeFeed_ = nullptr;
    QDoubleSpinBox* rampFeed_ = nullptr;
    QDoubleSpinBox* leadFeed_ = nullptr;
    QDoubleSpinBox* pullFeed_ = nullptr; ///< Bar pullers (DL-0091).
    QDoubleSpinBox* leadFeedPerRev_ = nullptr;
    QLabel* leadPerRevShare_ = nullptr;
    QDoubleSpinBox* partOffEndFeed_ = nullptr;
    QLabel* partOffShare_ = nullptr;
    QDoubleSpinBox* rampAngle_ = nullptr;
    QDoubleSpinBox* axialDepth_ = nullptr;
    QDoubleSpinBox* radialDepth_ = nullptr;
    QLabel* axialRatio_ = nullptr;
    QLabel* radialRatio_ = nullptr;
    QLabel* plungeShare_ = nullptr;
    QLabel* rampShare_ = nullptr;
    QLabel* leadShare_ = nullptr;
    QLabel* removalRate_ = nullptr;
    QLabel* problems_ = nullptr;
};

} // namespace ocam::gui
