// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Tool library manager. Edits a copy of a ToolLibrary: either a library file (*.ocamtools), with
// open/save and "copy to project", or the tool snapshot of the open project. Tools and holders
// have a tab each; holders are made from dimensions, edited point by point or imported from a
// STEP model. The tool list filters by type or family, diameter and text (the type and diameter
// filters are kept per user); a tool's holder is picked among holders of its kind, and its
// stickout and the assembly's gauge length are edited together. A tool's cutting presets are edited
// in their own window (CuttingPresetDialog). A library file's name is edited at the top; closing
// with unsaved changes asks first.

#pragma once

#include <ocam/tooling/ToolLibrary.hpp>

#include <QDialog>
#include <QMap>
#include <filesystem>
#include <functional>
#include <optional>

class QComboBox;
class QDoubleSpinBox;
class QFormLayout;
class QGroupBox;
class QKeyEvent;
class QLabel;
class QLineEdit;
class QListWidget;
class QPushButton;
class QSpinBox;
class QStackedWidget;
class QTabWidget;
class QTableWidget;

namespace ocam::gui {

class LengthSpinBox;
class ToolProfileView;

class ToolLibraryDialog : public QDialog {
    Q_OBJECT

public:
    enum class Mode { LibraryFile, ProjectTools };

    ToolLibraryDialog(tooling::ToolLibrary library, Mode mode, QWidget* parent = nullptr);

    const tooling::ToolLibrary& library() const noexcept { return lib_; }

    /// LibraryFile mode: the file being edited (if any).
    void setFile(const std::optional<std::filesystem::path>& file);
    const std::optional<std::filesystem::path>& file() const noexcept { return file_; }

    /// LibraryFile mode: enables "Copy to project"; the handler receives the selected tools and
    /// the holders they reference.
    using AddToProjectHandler =
        std::function<void(const std::vector<tooling::Tool>&, const std::vector<tooling::Holder>&)>;
    void setAddToProjectHandler(AddToProjectHandler handler);

    /// Adds a tool of `type` sized for the preferred unit (10 mm or 1/2 in) and selects it.
    void addTool(tooling::ToolType type);
    void addSolidBoringBar();
    /// Adds a holder (a generic stepped one) and shows it in the holders tab.
    void addHolder();
    /// Adds an ISO turning holder, a square shank or a boring bar, and shows it in the holders tab.
    void addTurningHolder(bool boringBar);
    void addTurningStick(bool bar);
    /// Adds a holder made from a STEP model, after the import dialog; false when cancelled or
    /// the file cannot be read.
    bool importHolder(const QString& path);
    /// Shows the holders tab.
    void showHolderTab();
    /// Selects the tool numbered `number` (when the filters show it); false when there is none.
    bool selectToolNumber(int number);
    /// Sets the tool list's filters.
    void setToolFilter(const tooling::ToolFilter& filter);
    /// The filters the tool list shows.
    tooling::ToolFilter toolFilter() const;
    /// Opens the cutting preset editor for the current tool at preset `index`; true when its
    /// changes were kept.
    bool editPresets(int index);

    /// LibraryFile mode: true when the library differs from the file.
    bool isModified() const;

public slots:
    /// LibraryFile mode: asks to save unsaved changes first (Save, Discard or Cancel).
    void reject() override;

protected:
    /// Enter only commits the field it is pressed in: it presses no button (Qt would press the
    /// first one, Duplicate, or OK).
    void keyPressEvent(QKeyEvent* event) override;

private:
    void buildUi();
    void rebuildTable();
    void selectRow(int row);
    void loadEditor();
    void commitEditor();
    void loadPresets();
    void updateTitle();
    /// LibraryFile mode: with unsaved changes, asks to save them. False when cancelled.
    bool maybeSave();
    void updateFieldVisibility();
    void applyUnitSuffixes(LengthUnit unit);
    void refreshCurrentRow();
    void updateIssues();
    void duplicateTool();
    void deleteTools();
    /// Hides the tools the filters leave out. `keepCurrent` keeps the current tool shown;
    /// otherwise a hidden current tool gives way to the first one shown.
    void applyFilter(bool keepCurrent);
    /// Selected rows the filters show.
    std::vector<int> selectedToolRows() const;
    /// Shows the assembly's gauge length (rotating tools only).
    void updateGauge();
    /// The gauge length was edited: the stickout follows.
    void commitGauge();
    void openFile();
    bool saveFile(bool askForPath);
    tooling::Tool* currentTool();

    // Holders.
    QWidget* buildHolderList();
    QWidget* buildHolderEditor();
    void showHolders(bool on);
    QWidget* buildIsoHolderBox();
    QWidget* buildStickHolderBox();
    void commitStickHolder();
    void loadIsoHolder(const tooling::TurningHolderCode& code);
    void commitIsoHolder();
    void commitIsoDesignation();
    /// Sets the current holder's code, keeps the tools in it fitted, and shows the result.
    void applyIsoHolder(const tooling::TurningHolderCode& code);
    void rebuildHolderTable();
    void refreshHolderRow();
    void refreshHolderCombo();
    /// Why `holder` does not take `tool`, in a sentence.
    QString holderMismatchText(const tooling::Tool& tool, const tooling::Holder& holder) const;
    void selectHolder(int row);
    void loadHolderEditor();
    void commitHolderEditor();
    void commitHolderPoints();
    void duplicateHolder();
    void deleteHolders();
    void holderFromDimensions();
    tooling::Holder* currentHolder();

    tooling::ToolLibrary lib_;
    Mode mode_;
    int current_ = -1;
    bool loading_ = false;
    std::optional<std::filesystem::path> file_;
    tooling::ToolLibrary saved_; ///< LibraryFile mode: the library as the file holds it.
    AddToProjectHandler addToProject_;

    QTableWidget* table_ = nullptr;
    QFormLayout* form_ = nullptr;
    QSpinBox* number_ = nullptr;
    QLineEdit* name_ = nullptr;
    QComboBox* type_ = nullptr;
    QLineEdit* material_ = nullptr;
    QLineEdit* coating_ = nullptr;
    QLineEdit* vendor_ = nullptr;
    QLineEdit* partNumber_ = nullptr;
    QLineEdit* comment_ = nullptr;
    QComboBox* holder_ = nullptr;
    QLabel* holderNote_ = nullptr;
    QComboBox* locking_ = nullptr;
    QDoubleSpinBox* stickout_ = nullptr;
    QDoubleSpinBox* gauge_ = nullptr;
    QLineEdit* filterText_ = nullptr;
    QComboBox* filterType_ = nullptr;
    LengthSpinBox* filterMin_ = nullptr;
    LengthSpinBox* filterMax_ = nullptr;
    QLabel* filterCount_ = nullptr;
    QSpinBox* flutes_ = nullptr;
    QComboBox* threadHand_ = nullptr;
    QSpinBox* orientation_ = nullptr;
    QComboBox* hand_ = nullptr;
    QComboBox* unit_ = nullptr;
    QComboBox* insertShape_ = nullptr;
    QMap<QString, QDoubleSpinBox*> lengths_; ///< keyed by field id
    QLineEdit* libraryName_ = nullptr;
    QListWidget* presets_ = nullptr;
    QPushButton* editPresets_ = nullptr;
    ToolProfileView* preview_ = nullptr;
    QLabel* issues_ = nullptr;
    QPushButton* copyToProject_ = nullptr;
    QGroupBox* presetsBox_ = nullptr;
    QTabWidget* lists_ = nullptr;
    QStackedWidget* editors_ = nullptr;

    int currentHolder_ = -1;
    bool loadingHolder_ = false;
    QTableWidget* holderTable_ = nullptr;
    QWidget* holderEditor_ = nullptr;
    QLineEdit* holderName_ = nullptr;
    QComboBox* holderKind_ = nullptr;
    QLineEdit* holderVendor_ = nullptr;
    QLineEdit* holderPartNumber_ = nullptr;
    QLineEdit* holderComment_ = nullptr;
    QTableWidget* holderPoints_ = nullptr;
    QLabel* holderInfo_ = nullptr;
    QFormLayout* holderForm_ = nullptr;
    QGroupBox* holderOutline_ = nullptr;
    // ISO turning holders: the designation, letter by letter.
    QGroupBox* isoBox_ = nullptr;
    QGroupBox* stickBox_ = nullptr;
    QComboBox* stickType_ = nullptr;
    QDoubleSpinBox* stickWidth_ = nullptr;
    QDoubleSpinBox* stickF_ = nullptr;
    QDoubleSpinBox* stickLength_ = nullptr;
    QFormLayout* isoForm_ = nullptr;
    QLineEdit* isoDesignation_ = nullptr;
    QComboBox* isoType_ = nullptr;
    QComboBox* isoBarType_ = nullptr;
    QComboBox* isoClamping_ = nullptr;
    QComboBox* isoShape_ = nullptr;
    QComboBox* isoStyle_ = nullptr;
    QDoubleSpinBox* isoStyleAngle_ = nullptr;
    QComboBox* isoClearance_ = nullptr;
    QDoubleSpinBox* isoClearanceAngle_ = nullptr;
    QDoubleSpinBox* isoLengthValue_ = nullptr;
    QComboBox* isoHand_ = nullptr;
    QDoubleSpinBox* isoHeight_ = nullptr;
    QDoubleSpinBox* isoWidth_ = nullptr;
    QComboBox* isoLength_ = nullptr;
    QSpinBox* isoSize_ = nullptr;
    QDoubleSpinBox* isoF_ = nullptr;
};

} // namespace ocam::gui
