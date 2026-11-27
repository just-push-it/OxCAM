// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Workholding editor (DL-0035), along the lines of the machine editor: the user's templates
// (WorkholdingLibrary) on the left; for the selected one, a 3D preview of its STEP parts and the
// parts table (rename, merge, hide, isolate, carried by the base or a member), the moving
// members (linear, with limits, optionally linked), the datums (picked on faces, edges and
// vertices in the preview, typed, offset or midway), and the stock holdings (jaws, dovetail,
// seat), with the mount datum the workholding sits on the machine by.

#pragma once

#include <ocam/cad/Model.hpp>
#include <ocam/document/Workholding.hpp>

#include <QDialog>
#include <QImage>
#include <filesystem>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <vector>

class QCheckBox;
class QComboBox;
class QToolButton;
class QDoubleSpinBox;
class QFormLayout;
class QLabel;
class QLineEdit;
class QListWidget;
class QPushButton;
class QSlider;
class QTableWidget;
class QSpinBox;
class QVBoxLayout;
class QWidget;

namespace ocam::gui {

class OcctView;
class PartTable;

class WorkholdingLibraryDialog : public QDialog {
    Q_OBJECT

public:
    explicit WorkholdingLibraryDialog(QWidget* parent = nullptr);

    /// Shows a "Use for <setup>" button that passes the selected template to `handler`.
    void setUseHandler(const QString& setupName,
                       std::function<void(const document::WorkholdingPackage&)> handler);

    /// Selects a template by id.
    void selectTemplate(const Uuid& id);

    /// The template as edited (with its part files).
    document::WorkholdingPackage editedPackage() const;
    /// The dialog as shown, with the 3D preview drawn in (for screenshots).
    QImage snapshot();
    /// Test hooks: a new template from solid parts (as the import would give them); picks a
    /// datum on a part's face, edge or vertex as a click in the preview would, returning its
    /// name; moves a member in the preview; the text of the issues shown.
    void newFromParts(const std::string& name, std::vector<document::SolidPart> parts);
    /// Test hook: shows or hides the Advanced section (parts, members, datums, holdings).
    void showAdvanced(bool on);
    /// Test hook: closes the jaws on a trial stock in the preview (nullopt: the slider again).
    void setTrialStock(const std::optional<Vec3>& size);
    std::string pickDatum(const std::string& partName, cad::SubShapeType type, int index);
    void jog(const std::string& member, double travel);
    QString issuesText() const;
    /// Test hooks for up: picks it on a part's face or edge as a click with Pick... would (a
    /// direction off the model's axes is snapped to the nearest, as when the user says No),
    /// returning the note shown; the warning shown when it disagrees with the jaw tops (empty:
    /// none); the "Use the jaw tops' direction" button.
    QString pickUp(const std::string& partName, cad::SubShapeType type, int index);
    QString upWarningText() const;
    bool useSeatUp();

protected:
    void reject() override;
    void done(int result) override;

private:
    struct Item {
        document::WorkholdingPackage package;
        std::filesystem::path file; ///< Empty until saved.
    };
    enum class Pick { None, Datum, PointOnFace, Up };

    void buildUi();
    void reloadList(const std::optional<Uuid>& selectId = std::nullopt);
    void onCurrentRowChanged(int row);
    void loadEditor(const Item& item);
    void markDirty();
    void updateIssues();
    bool saveCurrent();
    bool confirmDiscard();
    void newTemplate();
    void duplicateCurrent();
    void deleteCurrent();
    void importTemplate();
    void exportCurrent();
    Item* currentItem();
    const Item* currentItem() const;

    // Parts.
    void importParts();
    void addParts(std::vector<document::SolidPart> parts);
    void addPartRow(const document::WorkholdingPart& part);
    void mergeSelectedParts();
    void removeSelectedParts();
    std::vector<document::WorkholdingPart> editedParts() const;
    /// The part's model, read from its file (its sub-shape indices are the ones stored).
    const cad::Model* modelOf(const std::string& sha256);

    // Members.
    void addMemberRow(const document::WorkholdingMember& member);
    std::vector<document::WorkholdingMember> editedMembers() const;
    /// Member names changed in the table: parts, datums and links follow; carriers refilled.
    void membersEdited();

    // Datums.
    void addDatum(document::WorkholdingDatum datum);
    std::string uniqueDatumName(const std::string& base) const;
    void refreshDatumTable();
    void datumSelected();
    void datumFormEdited();
    void renameDatum(const std::string& from, const std::string& to);
    /// Refills the combos that choose datums (holdings, mount, the datum form).
    void refreshDatumChoices();

    // Holdings.
    void addHoldingRow(const document::StockHolding& holding);
    std::vector<document::StockHolding> editedHoldings() const;

    // The simple part: the wizard, the opening range, the slider and the trial stock.
    /// The template's parts with their files and models, for the wizard.
    std::pair<document::WorkholdingPackage, std::map<std::string, cad::Model>>
    packageFromParts(const std::string& name, std::vector<document::SolidPart> parts) const;
    /// Runs the setup wizard on `start`; true when it set something up (in `result`).
    bool runWizard(const document::WorkholdingPackage& start,
                   const std::map<std::string, cad::Model>& models,
                   document::WorkholdingPackage& result);
    void setUpWithWizard();
    void newFromPackage(document::WorkholdingPackage package);
    /// The summary, opening range, grip depth and slider from the edited template.
    void refreshSimple();
    void openingRangeEdited();

    // Up: which axis of the model points out of the fixture.
    /// Shows `direction` in the combo (an axis, or the exact direction as its own entry).
    void showUp(const Vec3& direction);
    /// Sets up as the user did (marks the template changed, redraws).
    void setUp(const Vec3& direction);
    Vec3 shownUp() const;
    /// Up from a face or edge picked on a part; `ask` whether to keep a direction off the axes.
    QString applyUpPick(const document::WorkholdingPart& part, const cad::Model& model,
                        cad::SubShapeType type, int index, bool ask);
    void refreshUpWarning(const document::WorkholdingTemplate& w);

    // Preview.
    void startPick(Pick mode);
    void previewPicked();
    void updatePreview();
    void rebuildJog();

    std::vector<Item> items_;
    int currentRow_ = -1;
    bool dirty_ = false;
    bool loading_ = false;
    std::map<std::string, std::string> files_; ///< sha256 -> STEP bytes of the edited template
    std::map<std::string, cad::Model> models_; ///< sha256 -> its model
    std::vector<document::WorkholdingPart> partData_; ///< Per parts table row (id, file).
    std::vector<std::string> memberNames_;            ///< Per members table row, as last seen.
    std::vector<document::WorkholdingDatum> datums_;
    std::map<std::string, double> jogTravel_;
    std::vector<int> viewShown_; ///< Per part: shown (1), hidden (0) or as set (-1). View only.
    Pick pick_ = Pick::None;
    bool previewFitted_ = false;

    QListWidget* list_ = nullptr;
    QWidget* editor_ = nullptr;
    QLabel* summary_ = nullptr;
    QWidget* rangeRow_ = nullptr;
    QDoubleSpinBox* minOpening_ = nullptr;
    QDoubleSpinBox* maxOpening_ = nullptr;
    QDoubleSpinBox* gripDepth_ = nullptr;
    QToolButton* advancedToggle_ = nullptr;
    QWidget* advanced_ = nullptr;
    QWidget* openingBar_ = nullptr;
    QSlider* opening_ = nullptr;
    QLabel* openingValue_ = nullptr;
    QCheckBox* trial_ = nullptr;
    QDoubleSpinBox* trialSize_[3] = {nullptr, nullptr, nullptr};
    QLineEdit* name_ = nullptr;
    QLineEdit* description_ = nullptr;
    QComboBox* up_ = nullptr;
    QPushButton* pickUp_ = nullptr;
    QPushButton* flipUp_ = nullptr;
    QWidget* upWarningRow_ = nullptr;
    QLabel* upWarning_ = nullptr;
    QPushButton* useSeatUp_ = nullptr;
    std::optional<Vec3> seatUp_;  ///< The jaw tops' direction while it disagrees with up.
    std::optional<Vec3> exactUp_; ///< An up off the model's axes: the combo's 7th entry.
    QComboBox* mount_ = nullptr;
    PartTable* parts_ = nullptr;
    QTableWidget* members_ = nullptr;
    QTableWidget* datumTable_ = nullptr;
    QWidget* datumForm_ = nullptr;
    QFormLayout* datumLayout_ = nullptr;
    QLineEdit* datumName_ = nullptr;
    QComboBox* datumMember_ = nullptr;
    QDoubleSpinBox* datumPoint_[3] = {nullptr, nullptr, nullptr};
    QDoubleSpinBox* datumDirection_[3] = {nullptr, nullptr, nullptr};
    QComboBox* datumFrom_ = nullptr;
    QComboBox* datumTo_ = nullptr;
    QComboBox* datumAlong_ = nullptr;
    QDoubleSpinBox* datumDistance_ = nullptr;
    QTableWidget* holdings_ = nullptr;
    // Jaws (DL-0069): a chuck that takes jaw sets, or a jaw set.
    QComboBox* jawRole_ = nullptr; ///< 0 none, 1 a chuck taking jaw sets, 2 a jaw set.
    QLineEdit* jawInterface_ = nullptr;
    QWidget* chuckJawRows_ = nullptr;
    QComboBox* jawSeat_ = nullptr; ///< A chuck's seat on master jaw 1, or an option's seat.
    QListWidget* topJaws_ = nullptr;
    QWidget* jawSetRows_ = nullptr;
    QComboBox* jawKind_ = nullptr;
    QDoubleSpinBox* jawInward_[3] = {nullptr, nullptr, nullptr};
    QComboBox* jawFront_ = nullptr;
    /// A jaw set's options (the ways it goes on, as a vise's holdings): the list, the one shown
    /// (its seat, inward and steps in the fields below), and the others as last edited.
    QComboBox* jawOption_ = nullptr;
    QLineEdit* jawOptionName_ = nullptr;
    std::vector<document::JawOption> jawOptionData_;
    int jawOptionShown_ = -1;
    QTableWidget* jawSteps_ = nullptr;
    /// The option the fields show, as edited.
    document::JawOption shownJawOption() const;
    /// Keeps the fields in jawOptionData_, then shows option `index`.
    void showJawOption(int index);
    void buildJawsUi(QVBoxLayout* into);
    void loadJaws(const document::WorkholdingTemplate& w);
    void editedJaws(document::WorkholdingTemplate& w) const;
    void refreshTopJaws();
    void addJawStepRow(const document::JawStep& step);
    void updateJawRows();
    OcctView* preview_ = nullptr;
    QFormLayout* jogForm_ = nullptr;
    std::map<std::string, QSlider*> jog_;
    QLabel* pickNote_ = nullptr;
    QPushButton* pickDatum_ = nullptr;
    QPushButton* pickPoint_ = nullptr;
    QLabel* issues_ = nullptr;
    QPushButton* save_ = nullptr;
    QPushButton* delete_ = nullptr;
    QPushButton* use_ = nullptr;
    std::function<void(const document::WorkholdingPackage&)> useHandler_;
};

} // namespace ocam::gui
