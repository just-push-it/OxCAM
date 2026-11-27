// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "Commands.hpp"
#include "ProjectTree.hpp"

#include <ocam/document/Document.hpp>
#include <ocam/toolpath/Toolpath.hpp>

#include <QMainWindow>
#include <QPointer>
#include <deque>
#include <filesystem>
#include <map>
#include <memory>
#include <optional>

class QAction;
class QDockWidget;
class QHBoxLayout;
class QMenu;
class QLabel;
class QTimer;

namespace ocam::gui {

class OcctView;
class PropertyPanel;
class SetupPlacementDialog;
class WorkholdingSetupDialog;
class SimulationPanel;
class TurretWindow;
class ToolpathDataDialog;

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

    /// Opens a project (.ocam) or imports a STEP model into the current project. Long work runs
    /// on a worker thread; loadFinished() reports the outcome.
    void openFile(const QString& path);

    OcctView* view() const noexcept { return view_; }
    /// Test hook: the open document.
    const document::Document* document() const noexcept { return doc_.get(); }

    /// Posts the current setup and shows the program (Setup > Post Process).
    void postProcessSetup();

    /// The setup's post and its option values (the setup page's Configure button); true when
    /// the user saved them.
    bool configurePost(const Uuid& setupId, bool forPosting = false);

    /// True while toolpaths are being generated or queued.
    bool toolpathsBusy() const noexcept { return jobRunning_ || !jobs_.empty(); }

    /// Simulates the current setup and shows the result (Setup > Simulate).
    void simulateCurrentSetup();
    bool simulationBusy() const noexcept;
    SimulationPanel* simulationPanel() const noexcept { return simPanel_; }
    /// True in simulation mode: the program locked while the simulation dock shows (test hook).
    bool simulationMode() const noexcept { return simulationMode_; }
    /// Selects the first setup in the project tree (test hook).
    void selectFirstSetup();
    void selectLastSetup();
    /// Selects an operation of the first setup (test hook; 0-based, clamped).
    void selectOperation(int index);

    /// The start hub (new, open, recent projects, tips); runs the chosen action.
    void showStartHub();
    /// The machine library; with a current setup, offers to use a machine for it.
    void showMachineLibrary();
    /// The workholding library; with a current setup, offers to use a vise for it.
    void showWorkholdingLibrary();
    void showFittingsLibrary();
    /// Places a setup's work on its machine (the current setup when unset): modal, the change
    /// committed on OK. With `modal` false (test hook) the dialog is shown and returned; OK still
    /// commits.
    SetupPlacementDialog* placeOnMachine(std::optional<Uuid> setupId = std::nullopt,
                                         bool modal = true);
    /// Sets up a setup's workholding (the current setup when unset): what holds the stock, its
    /// jaws, the project's copies; modal, taken on OK. With `modal` false (test hook) the dialog is
    /// shown and returned; OK still takes it.
    WorkholdingSetupDialog* setUpWorkholding(std::optional<Uuid> setupId = std::nullopt,
                                             bool modal = true);
    /// Edit > Preferences, on one of its pages.
    void showPreferences(int page = 0);

    /// The commands users bind keys to and pin to the toolbar.
    const CommandRegistry& commands() const noexcept { return commands_; }

signals:
    void loadFinished(bool success);

protected:
    void closeEvent(QCloseEvent* event) override;

private:
    void createActions();
    void setDocument(std::unique_ptr<document::Document> doc);
    void scheduleRefresh();
    void refresh();
    void refreshView();
    void updateTitle();

    void newProject();
    void openProjectDialog();
    void importStepDialog();
    /// Imports a STEP model; with `fixtureFor`, as a fixture of that setup.
    void importStep(const QString& path, std::optional<Uuid> fixtureFor = std::nullopt);
    /// Asks for a STEP file and imports it as a fixture of `setupId` (default: the current
    /// setup).
    void importFixtureDialog(std::optional<Uuid> setupId = std::nullopt);
    void openProject(const QString& path);
    bool save();
    bool saveAs();
    bool maybeSave();

    void addSetup();
    void addOperation(const QString& type, const QString& name);
    /// The add-operation submenus; `toolTips` shows what each operation is for (not in
    /// right-click menus).
    void populateOperationMenu(QMenu* menu, bool toolTips = false);
    /// The "add operation" commands, by group (2D, holes, 3D, multi-axis, turning).
    void createOperationActions();
    std::vector<std::vector<QAction*>> operationActions_;
    /// The operations of one group as menu actions.
    void addOperationActions(QMenu* menu, std::size_t group);
    void createToolBar();
    /// The toolbar's pinned commands, from the settings (hidden when there are none).
    void updatePinnedSection();
    QHBoxLayout* pinnedRow_ = nullptr;
    std::vector<QAction*> pinnedSection_;
    CommandRegistry commands_;
    QAction* simulateAction_ = nullptr;
    QAction* postAction_ = nullptr;
    QAction* deleteAction_ = nullptr;
    QAction* renameAction_ = nullptr;
    /// Simulation mode (the simulation dock is open): editing actions and the property panel
    /// are locked so the program cannot change under the simulation.
    void setSimulationMode(bool on);
    bool simulationMode_ = false;
    bool workholdingPickable_ = false; ///< The workholding's parts take picks (a WCS origin).
    std::vector<QAction*> editActions_;
    std::vector<QWidget*> editWidgets_;
    QLabel* simulationBanner_ = nullptr;
    void deleteSelected();
    void showContextMenu(const Selection& selection, const QString& group, const QPoint& pos);
    void renameSelected();
    void duplicateSetup(const Uuid& setupId);
    void duplicateOperation(const Uuid& operationId);
    void regenerateToolpaths(const Uuid& setupId);
    void updateRecentMenu();
    /// The view's colours from the preferences.
    void applyColors();
    /// A new empty document in the preferred unit.
    std::unique_ptr<document::Document> newDocument() const;
    void editToolLibrary(const std::filesystem::path& file);
    void showToolLibraries();
    /// Shows the in-process stock after the selected operation (View > In-Process Stock).
    void updateStockPreview();
    void editProjectTools();

    // Toolpaths: generated in the background, cached by input key.
    void updateToolpaths();
    void startNextToolpathJob();
    void refreshToolpathDisplay();
    QString toolpathStatus(const Uuid& operationId) const;
    ProjectTree::OperationState toolpathState(const Uuid& operationId) const;
    /// An operation's moves in a window of their own (its right-click menu).
    void showToolpathData(const Uuid& operationId);
    /// Brings the open toolpath data windows up to date with the operations' toolpaths.
    void refreshToolpathData();

    /// Setup that the current selection belongs to (or the first setup).
    std::optional<Uuid> currentSetupId() const;

    struct ToolpathEntry {
        std::string key;
        std::shared_ptr<const toolpath::Toolpath> path;
        QString error;
        bool pending = false;
    };
    struct ToolpathJob {
        Uuid operationId;
        Uuid setupId;
        std::string key;
    };
    std::map<Uuid, ToolpathEntry> toolpaths_;
    std::map<Uuid, QPointer<ToolpathDataDialog>> toolpathData_;
    std::deque<ToolpathJob> jobs_;
    bool jobRunning_ = false;
    ToolpathJob runningJob_;
    std::shared_ptr<CancelToken> jobCancel_;
    /// A project was opened: once its toolpaths are generated, its setups' in-process stock is
    /// worked out in the background (SimulationPanel::prepareSetups).
    bool prepareOnOpen_ = false;

    std::unique_ptr<document::Document> doc_;
    int listener_ = 0;
    bool refreshPending_ = false;
    std::shared_ptr<CancelToken> loadCancel_;

    OcctView* view_ = nullptr;
    ProjectTree* tree_ = nullptr;
    PropertyPanel* panel_ = nullptr;
    SimulationPanel* simPanel_ = nullptr;
    QDockWidget* simDock_ = nullptr;
    /// The current setup's machine and turret, beside the setup page (its turret section's
    /// Turret preview shows it).
    TurretWindow* turretWindow_ = nullptr;
    QAction* undo_ = nullptr;
    QAction* redo_ = nullptr;
    QMenu* recentMenu_ = nullptr;
    /// Toolpaths shown in the view: 0 = every operation of the setup, 1 = the selected
    /// operation only, 2 = none.
    int toolpathMode_ = 0;
    QAction* previewAction_ = nullptr;
    QAction* transparentStockAction_ = nullptr;
    QTimer* previewTimer_ = nullptr;
};

} // namespace ocam::gui
