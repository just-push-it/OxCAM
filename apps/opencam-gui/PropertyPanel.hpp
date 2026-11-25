// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Editor for the selected project item. Every edit is applied through Document::modify, so it
// is undoable; the panel is rebuilt from the project after each change.

#pragma once

#include "ProjectTree.hpp"

#include <ocam/document/Document.hpp>
#include <ocam/document/Workholding.hpp>
#include <ocam/strategies/Common.hpp>

#include <QScrollArea>
#include <functional>
#include <map>
#include <set>

class QFormLayout;
class QVBoxLayout;
class QPushButton;
class QLabel;

namespace ocam::gui {

class PropertyPanel : public QScrollArea {
    Q_OBJECT

public:
    explicit PropertyPanel(QWidget* parent = nullptr);

    void setDocument(document::Document* doc) { doc_ = doc; }

    /// Supplies the faces and edges currently selected in the 3D view, as references.
    using PickedShapesFn = std::function<std::vector<document::ShapeRef>()>;
    void setPickedShapesProvider(PickedShapesFn fn) { pickedShapes_ = std::move(fn); }

    /// Supplies everything selected in the 3D view, workholding parts included (by part id).
    struct ViewPick {
        Uuid model;
        cad::SubShapeType type = cad::SubShapeType::Face;
        int index = 0;
    };
    using ViewPicksFn = std::function<std::vector<ViewPick>()>;
    void setViewPicksProvider(ViewPicksFn fn) { viewPicks_ = std::move(fn); }

    /// Supplies a status text for an operation's toolpath (generating, error, statistics).
    using ToolpathStatusFn = std::function<QString(const Uuid&)>;
    void setToolpathStatusProvider(ToolpathStatusFn fn) { toolpathStatus_ = std::move(fn); }

    /// Stops picking an operation's geometry.
    void stopGeometryPick();
    /// True while picking a 2D contour's geometry (edges come as whole outlines).
    bool pickingOutlines() const;
    /// The edge indices of the closed outline edge `edgeIndex` belongs to, across the tool axis
    /// of the operation being edited ({edgeIndex} when there is none).
    std::vector<int> outlineOf(const Uuid& model, int edgeIndex) const;
    /// The 3D view's selection changed: finishes an action waiting for a pick.
    /// `individual`: Shift was held (pick one edge, not its whole outline).
    void viewSelectionChanged(bool individual = false);
    /// Stops waiting for a pick.
    void cancelPick();
    bool waitingForPick() const noexcept { return pending_.has_value() || wcsPick_.has_value(); }

    /// Updates the toolpath status line of the operation page, if shown.
    void refreshToolpathStatus();

    /// Rebuilds the editor for `selection` from the current project.
    void showSelection(const Selection& selection);

signals:
    /// A prompt while an action waits for a pick in the view; empty when it no longer waits.
    void pickPrompt(const QString& prompt);
    /// Show these faces and edges selected in the view.
    void highlightShapes(const std::vector<document::ShapeRef>& shapes);
    /// Clicks in the view toggle faces and edges (while picking an operation's geometry).
    void toggleSelectionRequested(bool on);
    /// Vertices can be picked in the view (while picking a start point).
    void vertexPickingRequested(bool on);
    /// The setup's workholding parts can be picked in the view (while picking a WCS axis or
    /// origin, which may be on them).
    void workholdingPickingRequested(bool on);
    void editProjectToolsRequested();
    /// Open the machine library. Emitted after the setup page's Machine list has returned from
    /// its signal: the library's Use for the setup rebuilds the page, the list with it.
    void manageMachinesRequested();
    /// Open the workholding library (to choose a vise for this setup).
    void manageWorkholdingRequested();
    /// Choose this setup's workholding and jaws, and see the project's copies
    /// (WorkholdingSetupDialog).
    void workholdingSetupRequested(const ocam::Uuid& setupId);
    /// Open the turret blocks and centres library; after the list it is chosen from has returned
    /// from its signal, as for manageMachinesRequested.
    void manageFittingsRequested();
    /// Place this setup's work on its machine (SetupPlacementDialog).
    void placeOnMachineRequested(const ocam::Uuid& setupId);
    /// Choose this setup's post and set its options (PostProcessDialog, without posting).
    void postSettingsRequested(const ocam::Uuid& setupId);
    /// Set up this setup's turret in its own window (TurretWindow): its stations' blocks and
    /// tools beside the turret preview, `station` chosen.
    void turretWindowRequested(const ocam::Uuid& setupId, int station);
    /// A row of the setup's turret table was chosen: its station number, or a list on its row.
    void turretStationSelected(int station);

public:
    /// The station chosen in the setup's turret table (its row marked), as the turret preview
    /// has it.
    void setTurretStation(int station);
    int turretStation() const noexcept { return turretStation_; }
    /// The turret window's panel: a setup shows only its turret's table (blocks and tools on the
    /// stations); the setup page shows a summary and a button that opens that window instead.
    void setTurretOnly(bool on) noexcept { turretOnly_ = on; }

private:
    /// Chooses a row of the turret table: marks it and says so (turretStationSelected).
    void selectTurretStation(int station);
    int turretStation_ = 1;
    bool turretOnly_ = false;
    /// The setup page's turret section: what is on the stations, and the button that opens the
    /// turret window.
    void buildTurretSummary(QFormLayout* form, const document::Setup& setup,
                            const machine::Machine& machine);
    QFormLayout* newForm(const QString& title);
    /// A section of the page that folds: its title on a button over rows of its own, below the
    /// page's rows and the sections added before it. It stays open or closed as it was last left
    /// (by `key`, while the program runs), else as `open` says.
    QFormLayout* addSection(const QString& key, const QString& title, bool open);
    /// A jaw set as the setup lists it: its name, and the way it goes on when it has more than one
    /// ("HB10A1 (reversed)").
    QString jawEntryName(const document::WorkholdingTemplate& set, const std::string& option) const;
    QVBoxLayout* pageLayout_ = nullptr; ///< The page newForm made, for its sections.
    std::map<QString, bool> sectionOpen_;
    void buildProject();
    void buildModel(const document::ModelEntry& model);
    void buildTool(const tooling::Tool& tool);
    void buildSetup(const document::Setup& setup);
    void buildOperation(const document::Operation& op, const document::Setup& setup);
    void commit(const std::string& label, const std::function<void(document::Project&)>& change);

    document::Document* doc_ = nullptr;
    PickedShapesFn pickedShapes_;
    ViewPicksFn viewPicks_;
    /// A face, edge or vertex being picked for a setup's WCS (on the part, a fixture model or the
    /// workholding): an axis of its part orientation or its origin.
    enum class WcsTarget { FirstAxis, ZAxis, Origin };
    struct WcsPick {
        Uuid setup;
        WcsTarget target = WcsTarget::Origin;
    };
    std::optional<WcsPick> wcsPick_;
    void startWcsPick(const Uuid& setupId, WcsTarget target);
    void stopWcsPick();
    /// Applies a pick in the view to the WCS pick waiting for it; false when it is not on the
    /// part, a fixture or the workholding.
    bool applyWcsPick(const ViewPick& pick);
    /// Speeds and feeds of the operation: the preset's values, changed for this operation.
    /// A transfer to the sub-spindle: where Z2 takes the part (ADR-0007).
    void buildTransferPickOff(QFormLayout* form, const document::Operation& op,
                              const document::Setup& setup);
    void buildPresetOverrides(QFormLayout* form, const document::Operation& op,
                              const tooling::CuttingPreset& preset, const tooling::Tool& tool);
    /// The operation's heights (clearance, retract, feed, top, bottom): reference and offset.
    void buildHeights(QFormLayout* form, const document::Operation& op,
                      const document::Setup& setup);
    /// Work coordinate system rows of the setup page.
    void buildWcs(QFormLayout* form, const document::Setup& setup);
    /// The setup's post processor (DL-0039): the machine's, a built-in, one the project carries,
    /// one from the user's post library or a post file; and its program options.
    void buildPost(QFormLayout* form, const document::Setup& setup,
                   const machine::Machine& machine);
    /// The per-operation options of the setup's post on the operation page.
    void buildPostOptions(QFormLayout* form, const document::Operation& op,
                          const document::Setup& setup);
    /// The setup's stock now as a box by size (along the WCS axes) and shift (from centred on
    /// the part, along the axes); nullopt when it cannot be resolved.
    std::optional<std::pair<Vec3, Vec3>> sizedBox(const Uuid& setupId) const;
    /// The setup's stock now as fixed stock of `kind` where it is (DL-0038): a box by size, a
    /// cylinder or tube by diameter and length along the nearest WCS axis (a box's corners
    /// inside it), with the shift from centred on the part; nullopt when it cannot be resolved.
    std::optional<document::Stock> fixedStock(const Uuid& setupId, document::StockKind kind) const;
    /// The stock rows of the setup page (DL-0038): shape, size and the fields they use.
    void buildStock(QFormLayout* form, const document::Setup& setup);
    /// A lathe with a layout: what is in the turret and where the work sits (DL-0067).
    void buildTurret(QFormLayout* form, const document::Setup& setup,
                     const machine::Machine& machine);
    /// Workholding rows of the setup page: the vise, its jaws and where the stock sits in it.
    void buildLatheSupports(QFormLayout* form, const document::Setup& setup);
    void buildWorkholding(QFormLayout* form, const document::Setup& setup);
    /// A row naming what holds the stock (or its jaws), with Set up... (WorkholdingSetupDialog).
    QWidget* setUpRow(const QString& name, const QString& objectName, const Uuid& setupId);
    /// An action that needs a picked face or edge (a "Z ⟂ face" button): run on the current
    /// selection, or on the next pick when nothing suitable is selected.
    struct PendingPick {
        QString id;                            ///< The button that started it.
        std::optional<cad::SubShapeType> type; ///< nullopt: a face or an edge.
        std::function<void(const document::ShapeRef&)> apply;
    };
    std::optional<PendingPick> pending_;
    /// The operation whose geometry follows the view's selection while picking.
    std::optional<Uuid> geometryPick_;
    /// The geometry list of the operation page (rows select the faces in the view).
    void buildGeometry(QFormLayout* form, const document::Operation& op,
                       strategies::GeometryUse use, const document::Setup& setup);
    /// Whether the operation can use this face or edge: faces for most, faces or edges for
    /// contours, edges across the tool axis for chamfers (no vertical edges).
    bool accepts(const document::Operation& op, const document::Setup& setup,
                 const document::ShapeRef& ref) const;
    /// The contour's start point: automatic, or a picked vertex or vertical edge.
    void buildStartPoint(QFormLayout* form, const document::Operation& op);
    /// The operation's tool orientation (3+2): its tool axis, origin and work offset, and the
    /// rotary positions the setup's machine indexes to.
    void buildOrientation(QFormLayout* form, const document::Operation& op,
                          const document::Setup& setup);
    void withPick(const QString& id, std::optional<cad::SubShapeType> type, const QString& prompt,
                  std::function<void(const document::ShapeRef&)> apply);
    void refreshLater();
    /// A checkable button shown pressed while its pick is pending.
    QPushButton* pickButton(const QString& id, const QString& text);
    /// The picked shapes of one kind.
    std::vector<document::ShapeRef> picked(cad::SubShapeType type) const;
    /// Lets the current page shrink to the dock's width (no horizontal scrolling).
    void compactPage();
    ToolpathStatusFn toolpathStatus_;
    QLabel* statusLabel_ = nullptr;
    Uuid statusOperation_;
    Selection selection_;
    LengthUnit unit_ = LengthUnit::Millimeter; ///< The project's display unit.
};

} // namespace ocam::gui
