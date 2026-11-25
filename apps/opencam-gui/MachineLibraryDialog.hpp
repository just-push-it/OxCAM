// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Machine library manager: lists the built-in machines (read-only) and the user's machines
// (MachineLibrary), and edits a machine's kinematics, axes and limits, spindle and feed rates,
// rotary pivot, collision components and machine model (DL-0037: mesh parts imported from STEP
// or STL, each carried by the base or an axis of the kinematic tree, shown in a preview that
// jogs the axes). Built-ins are customized by duplicating them.

#pragma once

#include <ocam/cad/SubShape.hpp>
#include <ocam/geom3d/AxisFit.hpp>
#include <ocam/machine/Machine.hpp>

#include <QDialog>
#include <QImage>
#include <array>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <vector>

class QComboBox;
class QDoubleSpinBox;
class QFormLayout;
class QKeyEvent;
class QCheckBox;
class QGroupBox;
class QScrollArea;
class QSlider;
class QTimer;
class QToolButton;
class QVBoxLayout;
class QLabel;
class QLineEdit;
class QListWidget;
class QPushButton;
class QSpinBox;
class QTableWidget;
class QWidget;

namespace ocam::gui {

class OcctView;

class MachineLibraryDialog : public QDialog {
    Q_OBJECT

public:
    explicit MachineLibraryDialog(QWidget* parent = nullptr);

    /// Shows a "Use for <setup>" button that passes the selected machine to `handler`.
    void setUseHandler(const QString& setupName,
                       std::function<void(const machine::Machine&)> handler);

    /// Selects a machine by id (built-in or library).
    void selectMachine(const std::string& id);

    /// The dialog as shown, with the 3D preview drawn in (for screenshots).
    QImage snapshot();
    /// Test hook: puts a machine axis of the preview at `value` (mm or degrees).
    void jogPreview(const std::string& axis, double value);
    /// Test hook: scrolls the editor to the machine model.
    void showModelSection();
    /// Test hooks: selects a part or component row by name; the selected part's name; shows only
    /// the selected parts in the preview; clicks the preview and returns the part selected.
    void selectPartByName(const std::string& name);
    std::string selectedPartName() const;
    void isolateSelected();
    std::string pickPreview(const QPoint& position);
    /// Test hook: picks rotary axis `axis` on a round face at `point` (model frame at machine
    /// zero) by clicking where it shows in the preview; returns the result line shown.
    std::string pickAxisAt(const std::string& axis, const Vec3& point);
    /// Test hook: picks the spindle gauge point on a round face of the spindle at `round` and
    /// the seat face at `seat` (seen from below), both in the model frame at machine zero.
    std::string pickGaugeAt(const Vec3& round, const Vec3& seat);
    /// The preview's size in its own pixels (for pickPreview).
    QSize previewSize() const;
    /// Test hooks: chooses how the machine stands as a user would (the Up and Front lists' texts,
    /// such as "+Y" and "+Z"), and says how the preview stands: which way is up in it and the
    /// machine axes the view cube's triad names along the world's right, back and up.
    void chooseStanding(const QString& up, const QString& front);
    std::string standingReport();

protected:
    void done(int result) override;
    void keyPressEvent(QKeyEvent* event) override;

protected:
    void reject() override;

private:
    struct Item {
        machine::Machine machine;
        bool builtin = false;
    };

    void buildUi();
    void reloadList(const std::string& selectId = {});
    /// Hides the built-in machines unless Show built-ins is ticked; a hidden current one gives
    /// way to the first one shown.
    void applyShowBuiltins();
    void onCurrentRowChanged(int row);
    void loadEditor(const Item& item);
    machine::Machine editedMachine() const;
    void markDirty();
    void updateIssues();
    bool saveCurrent();
    bool confirmDiscard();
    void addFromTemplate();
    void duplicateCurrent();
    void deleteCurrent();
    void importMachine();
    void exportCurrent();
    void ensureRotaryAxes();
    /// Fills the default post choices for the kinematics shown, selecting `postId`.
    void fillPosts(const std::string& postId);
    /// Sets the machine's values of its default post's program options (PostOptionsForm).
    void configurePostOptions();
    std::string uniqueId(const std::string& base) const;
    const Item* currentItem() const;

    // Machine model (DL-0037).
    void buildModelUi(QVBoxLayout* editorLayout);
    void loadModel(const machine::Machine& m);
    machine::MachineModel editedModel() const;
    void addModelAxisRow(const machine::ModelAxis& axis);
    void addPartRow(const machine::ModelPart& part);
    /// The axis names of the model's axes table, for the "carried by" choices.
    std::vector<std::string> modelAxisNames() const;
    /// Refills every "carried by" choice from the axes table, keeping what each one chose.
    void refreshCarriers();
    void fillDefaultAxes();
    void importParts();
    void mergeSelectedParts();
    /// Preview scene indices of the selected rows: parts, then components after the parts.
    std::vector<int> selectedSceneParts() const;
    void setViewShown(int index, bool shown);
    /// Selects the row of a preview scene part (a click in the preview).
    void selectScenePart(int index);

    // A rotary axis's centre line from the geometry: a round face, or three points on a rim.
    enum class AxisPick { None, Face, Rim, GaugeAxis, GaugeSeat };
    void startAxisPick(AxisPick mode);
    /// Sets the spindle gauge point picked on `part` and ends the pick.
    void finishGauge(const machine::ModelPart& part, const Vec3& gauge);
    /// The spindle gauge point: the spindle's centre line from a round face, then its height
    /// from the face the holder seats against.
    void startGaugePick();
    void stopAxisPick(const QString& note = {});
    void previewClicked(int index, double x, double y, double z);
    void applyAxisLine(int row, const geom3d::AxisLine& line, const QString& from);
    std::vector<int> selectedPartRows() const;
    void schedulePreview();
    void updatePreview();
    void rebuildJog();

    std::vector<Item> items_;
    int currentRow_ = -1;
    bool dirty_ = false;
    bool loading_ = false;
    std::string loadedId_; ///< Id of the user machine in the editor when it was loaded.

    QListWidget* list_ = nullptr;
    QCheckBox* showBuiltins_ = nullptr;
    QCheckBox* enclosureBox_ = nullptr; ///< The preview shows the machine's enclosure.
    QWidget* editor_ = nullptr;
    QLabel* readOnlyNote_ = nullptr;
    QLineEdit* id_ = nullptr;
    QLineEdit* name_ = nullptr;
    QLineEdit* vendor_ = nullptr;
    QComboBox* kinematics_ = nullptr;
    /// How it stands (Machine::orientation): the axis up (the first item: as its kinematics) and
    /// the one towards the operator.
    QComboBox* standUp_ = nullptr;
    QComboBox* standFront_ = nullptr;
    /// Names the kinematics' standing in the first item, and puts its front in while it is chosen.
    void updateStanding();
    QComboBox* post_ = nullptr; ///< Machine::post
    QPushButton* postOptionsButton_ = nullptr;
    Json postOptions_ = Json::object(); ///< Machine::postOptions of the machine edited
    QTableWidget* axes_ = nullptr;
    QDoubleSpinBox* minRpm_ = nullptr;
    QDoubleSpinBox* maxRpm_ = nullptr;
    QDoubleSpinBox* maxFeed_ = nullptr;
    QDoubleSpinBox* rapid_ = nullptr;
    QSpinBox* toolCapacity_ = nullptr;
    QDoubleSpinBox* toolChangeTime_ = nullptr;
    QSpinBox* workOffsets_ = nullptr;
    QDoubleSpinBox* pivot_ = nullptr;
    // Lathes: what the turret can do.
    QGroupBox* latheBox_ = nullptr;
    QCheckBox* liveTooling_ = nullptr;
    QDoubleSpinBox* liveMaxRpm_ = nullptr;
    QCheckBox* yAxis_ = nullptr;
    void updateLatheBox();
    void setYAxis(bool on);
    // Lathes: its layout for the machine simulation (DL-0067).
    QCheckBox* layoutOn_ = nullptr;
    QWidget* layoutFields_ = nullptr;
    QCheckBox* indexAtHome_ = nullptr;
    QSpinBox* stations_ = nullptr; ///< The tool changer's capacity, under the turret's name.
    QCheckBox* tailstockOn_ = nullptr;
    QLineEdit* blockMounts_ = nullptr;      ///< The kinds of block it takes, by name, with commas.
    QLineEdit* quillInterface_ = nullptr;   ///< What its quill takes.
    QTableWidget* stationMounts_ = nullptr; ///< Stations that take other blocks: stations, kinds.
    QCheckBox* subOn_ = nullptr;
    QDoubleSpinBox* subMinRpm_ = nullptr; ///< The sub-spindle's speed range (ADR-0007).
    QDoubleSpinBox* subMaxRpm_ = nullptr;
    /// The layout's numbers, each with its edit box.
    std::vector<std::pair<std::function<double&(machine::LatheLayout&)>, QDoubleSpinBox*>>
        layoutNumbers_;
    void buildLatheLayoutUi(QFormLayout* form);
    void loadLatheLayout(const machine::Machine& m);
    std::optional<machine::LatheLayout> editedLatheLayout() const;
    QTableWidget* components_ = nullptr;
    QTableWidget* modelAxes_ = nullptr;
    QComboBox* spindleCarrier_ = nullptr;
    QDoubleSpinBox* meshTolerance_ = nullptr; ///< MachineModel::meshTolerance
    std::array<QDoubleSpinBox*, 3> spindlePoint_{};
    QTableWidget* parts_ = nullptr;
    QToolButton* simpleToggle_ = nullptr;
    QGroupBox* simpleShapes_ = nullptr;
    /// Per row of parts_: the part as imported or loaded (its mesh).
    std::vector<machine::ModelPart> partData_;
    OcctView* preview_ = nullptr;
    QFormLayout* jogForm_ = nullptr;
    std::map<std::string, QSlider*> jog_;
    std::vector<std::string> jogAxes_; ///< The axes jog_ was built for.
    QTimer* previewTimer_ = nullptr;
    QLabel* previewNote_ = nullptr;
    QScrollArea* scroll_ = nullptr;
    QGroupBox* modelBox_ = nullptr;
    bool previewShown_ = false; ///< The preview was fitted to the current machine.
    /// Per preview scene part, shown in the preview regardless of its default (1), hidden (0) or
    /// as the machine has it (-1). View only, never saved.
    std::vector<int> viewShown_;
    AxisPick axisPick_ = AxisPick::None;
    int axisPickRow_ = -1;
    std::vector<Vec3> rimPoints_;
    QPushButton* pickFace_ = nullptr;
    QPushButton* pickRim_ = nullptr;
    QPushButton* pickGauge_ = nullptr;
    std::optional<geom3d::AxisLine> gaugeAxis_;
    /// The face, edge or vertex under the last click on a part drawn as B-rep (its STEP file):
    /// the pickers use its exact geometry; mesh parts fall back to fitting the mesh.
    struct ExactPick {
        int index = -1;
        cad::SubShapeType type = cad::SubShapeType::Face;
        int subIndex = 0;
    };
    std::optional<ExactPick> exactPick_;
    /// Preview scene parts that can be picked (the machine's; the stand-in tool comes after).
    std::size_t pickableParts_ = 0;
    QLabel* pickNote_ = nullptr;
    /// The preview's parts and where each one's carrier has moved it (model frame at machine
    /// zero -> preview), to take picked points back to machine zero.
    std::shared_ptr<const std::vector<machine::ModelPart>> previewParts_;
    std::vector<Transform3> previewCarried_;
    double axisLineLength_ = 400.0;

    QLabel* issues_ = nullptr;
    QPushButton* save_ = nullptr;
    QPushButton* delete_ = nullptr;
    QPushButton* use_ = nullptr;
    std::function<void(const machine::Machine&)> useHandler_;
};

} // namespace ocam::gui
