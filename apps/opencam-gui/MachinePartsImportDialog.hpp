// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Importing a machine's CAD as model parts (DL-0037): the files are read in the background
// (progress, cancel), assemblies split into their parts; the dialog lists them and merges what
// moves together (by subassembly, or by hand); STEP parts keep their exact geometry, meshed at the
// machine's tolerance.

#pragma once

#include <ocam/document/MachineImport.hpp>
#include <ocam/document/Workholding.hpp>

#include <QDialog>
#include <QStringList>
#include <functional>
#include <vector>

class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QTableWidget;

namespace ocam::gui {

class MachinePartsImportDialog : public QDialog {
    Q_OBJECT

public:
    /// `solids`: for workholding parts, kept as solids (no tessellation, no STL unit).
    MachinePartsImportDialog(std::vector<document::ImportedPart> parts, QWidget* parent = nullptr,
                             bool solids = false);

    /// Reads `files`, lets the user group the parts and returns the machine parts, STEP parts
    /// meshed at `tolerance` (the machine's), empty when cancelled or nothing could be read
    /// (problems are reported in message boxes).
    static std::vector<machine::ModelPart> run(const QStringList& files, double tolerance,
                                               QWidget* parent);
    /// The same for workholding parts (DL-0035): STEP files, every body a part, merged as the
    /// user groups them, kept as solids.
    static std::vector<document::SolidPart> runSolids(const QStringList& files, QWidget* parent);

    /// The group of each part (empty: left out).
    std::vector<std::string> groups() const;
    /// Millimetres per STL unit.
    double stlScale() const;
    /// Test hook: merges at an assembly level (0 = each part on its own).
    void setLevel(int level);
    const std::vector<document::ImportedPart>& parts() const noexcept { return parts_; }

private:
    void applyLevel();
    void mergeSelected();
    void updateSummary();

    std::vector<document::ImportedPart> parts_;
    QComboBox* level_ = nullptr;
    QTableWidget* table_ = nullptr;
    QLabel* summary_ = nullptr;
    QComboBox* stlUnit_ = nullptr;
};

/// Runs `work` on a worker thread behind a modal progress dialog with a Cancel button. Returns
/// false when cancelled; other failures are reported in a message box titled `title` (false).
bool runWithProgress(QWidget* parent, const QString& title,
                     const std::function<void(CancelToken&, ProgressSink&)>& work);

} // namespace ocam::gui
