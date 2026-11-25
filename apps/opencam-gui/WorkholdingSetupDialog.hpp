// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sets up a setup's workholding (DL-0035, DL-0069): what holds the stock (a vise, a chuck, a
// fixture, from the project or the library) and, on a chuck with a jaw mount, which jaws; what the
// project has copied in, and whether the library's copy differs; how it holds the stock (the jaw
// position, the grip, the step, the jaw face offset, the soft jaws' bore); with a view of the stock
// held. It edits a snapshot of the document: OK hands it back (Document::takeWorkholding), Cancel
// leaves the project as it was.

#pragma once

#include "WorkholdingLibrary.hpp"

#include <ocam/document/Document.hpp>
#include <ocam/foundation/Units.hpp>

#include <QDialog>
#include <QImage>
#include <QStringList>
#include <functional>
#include <string>
#include <vector>

class QGroupBox;
class QLabel;
class QPushButton;
class QTreeWidget;
class QVBoxLayout;

namespace ocam::gui {

class OcctView;

class WorkholdingSetupDialog : public QDialog {
    Q_OBJECT

public:
    WorkholdingSetupDialog(const document::Document& doc, const Uuid& setupId, LengthUnit unit,
                           QWidget* parent = nullptr);

    /// The document as set up here (a snapshot of the one it was opened on).
    const document::Document& edited() const noexcept { return doc_; }
    const Uuid& setupId() const noexcept { return setupId_; }

    /// Test hooks: the rows of each list (name, then what it says of it, " | " between), choosing
    /// one by its name (false when there is none), the copies' buttons, and what is said under the
    /// view.
    QStringList holdingRows() const;
    QStringList jawRows() const;
    QStringList copyRows() const;
    bool chooseHolding(const QString& name);
    bool chooseJaws(const QString& name);
    bool updateCopy(const QString& name);
    QString problemText() const;
    /// The dialog with its 3D view (grab() leaves the view out).
    QImage snapshot();

private:
    void reloadLibrary();
    void rebuild();
    void fillHoldings();
    void fillJaws();
    void fillCopies();
    /// How the workholding holds the stock: the fields of the setup's placement.
    void fillPlacement();
    /// Rebuilds once the current signal has returned (its widget is replaced).
    void rebuildLater();
    /// Changes the setup's workholding in the snapshot, then rebuilds.
    void place(const std::string& label,
               const std::function<void(document::SetupWorkholding&)>& change);
    void updatePreview();
    void holdingChosen();
    void jawsChosen();
    void copySelected();
    /// A chuck that takes jaw sets and has none on: puts the first that fits on it, from `earlier`
    /// (the project's templates before the chuck changed), else from the library.
    void pickJaws(const std::vector<document::WorkholdingTemplate>& earlier);
    /// The library's entry of a template id, or null.
    const document::WorkholdingPackage* inLibrary(const Uuid& id) const;
    /// Puts a library template into the snapshot and on the setup (a jaw set: on its chuck).
    bool useFromLibrary(const document::WorkholdingPackage& package);
    const document::Setup& setup() const;

    document::Document doc_;
    Uuid setupId_;
    LengthUnit unit_;
    std::vector<WorkholdingLibrary::Entry> library_;
    bool filling_ = false;
    bool rebuildPending_ = false;
    bool fitted_ = false;

    QTreeWidget* holdings_ = nullptr;
    QGroupBox* jawsBox_ = nullptr;
    QTreeWidget* jaws_ = nullptr;
    QLabel* otherJaws_ = nullptr;
    QGroupBox* placementBox_ = nullptr;
    QVBoxLayout* placementLayout_ = nullptr;
    QWidget* placement_ = nullptr;
    QTreeWidget* copies_ = nullptr;
    QPushButton* update_ = nullptr;
    QPushButton* save_ = nullptr;
    OcctView* preview_ = nullptr;
    QLabel* summary_ = nullptr;
    QLabel* problem_ = nullptr;
};

} // namespace ocam::gui
