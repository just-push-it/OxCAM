// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "MainWindow.hpp"

#include "LatheFittingsDialog.hpp"
#include "MachineLibrary.hpp"
#include "MachineLibraryDialog.hpp"
#include "OcctView.hpp"
#include "OperationDefaults.hpp"
#include "PostDialog.hpp"
#include "PostProcessDialog.hpp"
#include "PreferencesDialog.hpp"
#include "PropertyPanel.hpp"
#include "SetupPlacementDialog.hpp"
#include "SimulationPanel.hpp"
#include "StartHub.hpp"
#include "ToolLibraries.hpp"
#include "ToolLibrariesDialog.hpp"
#include "ToolLibraryDialog.hpp"
#include "ToolTips.hpp"
#include "ToolpathDataDialog.hpp"
#include "Translations.hpp"
#include "TurretPreview.hpp"
#include "TurretWindow.hpp"
#include "Units.hpp"
#include "ViewColors.hpp"
#include "WorkholdingLibraryDialog.hpp"
#include "WorkholdingSetupDialog.hpp"

#include <ocam/cad/SubShape.hpp>
#include <ocam/document/Generate.hpp>
#include <ocam/document/PostSetup.hpp>
#include <ocam/document/Workholding.hpp>
#include <ocam/foundation/Error.hpp>
#include <ocam/foundation/Version.hpp>
#include <ocam/strategies/Common.hpp>

#include <QActionGroup>
#include <QApplication>
#include <QCloseEvent>
#include <QDockWidget>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QLabel>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QProgressDialog>
#include <QSettings>
#include <QStatusBar>
#include <QThread>
#include <QTimer>
#include <QToolBar>
#include <QToolButton>
#include <QVBoxLayout>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <set>
#include <tuple>

namespace ocam::gui {
namespace {

/// Transparency of the simulated stock with View > Transparent Stock: enough to see the part
/// inside, solid enough to see where stock is left.
constexpr double kStockTransparency = 0.55;

QString version() {
    return QString::fromUtf8(kVersionString.data(), static_cast<int>(kVersionString.size()));
}

std::filesystem::path toPath(const QString& s) {
    return std::filesystem::path(s.toStdU16String());
}

QString fromPath(const std::filesystem::path& p) {
    return QString::fromStdU16String(p.u16string());
}

// Progress of background work, read by the GUI thread.
class AtomicProgress final : public ProgressSink {
public:
    explicit AtomicProgress(std::shared_ptr<std::atomic<double>> value)
        : value_(std::move(value)) {}
    void report(double fraction, std::string_view /*stage*/) override { value_->store(fraction); }

private:
    std::shared_ptr<std::atomic<double>> value_;
};

// Runs `work` on a worker thread and `done` on the GUI thread afterwards.
template <typename Work, typename Done>
void runInBackground(QObject* context, Work work, Done done) {
    QThread* worker = QThread::create(std::move(work));
    QObject::connect(worker, &QThread::finished, context, [worker, done = std::move(done)] {
        worker->deleteLater();
        done();
    });
    worker->start();
}

} // namespace

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), commands_(this) {
    resize(1400, 860);

    view_ = new OcctView(this);
    setCentralWidget(view_);

    tree_ = new ProjectTree(this);
    auto* treeDock = new QDockWidget(tr("Project"), this);
    treeDock->setObjectName(QStringLiteral("projectDock"));
    treeDock->setWidget(tree_);
    addDockWidget(Qt::LeftDockWidgetArea, treeDock);

    panel_ = new PropertyPanel(this);
    auto* panelDock = new QDockWidget(tr("Properties"), this);
    panelDock->setObjectName(QStringLiteral("propertiesDock"));
    panelDock->setWidget(panel_);
    addDockWidget(Qt::RightDockWidgetArea, panelDock);

    simPanel_ = new SimulationPanel(
        view_,
        [this]() -> std::shared_ptr<const document::Document> {
            return doc_ ? std::make_shared<const document::Document>(doc_->snapshot()) : nullptr;
        },
        [this](const document::Document& doc, const Uuid& setupId) {
            // Toolpaths shown in the view whose inputs are unchanged.
            std::map<Uuid, std::shared_ptr<const toolpath::Toolpath>> paths;
            if (const document::Setup* setup = doc.project().findSetup(setupId)) {
                for (const auto& op : setup->operations) {
                    const auto it = toolpaths_.find(op.id);
                    if (it != toolpaths_.end() && it->second.path &&
                        it->second.key == document::toolpathInputKey(doc, *setup, op)) {
                        paths.emplace(op.id, it->second.path);
                    }
                }
            }
            return paths;
        },
        this);
    simDock_ = new QDockWidget(tr("Simulation"), this);
    simDock_->setObjectName(QStringLiteral("simulationDock"));
    simDock_->setWidget(simPanel_);
    addDockWidget(Qt::RightDockWidgetArea, simDock_);
    simDock_->hide();
    // Hiding the simulation dock brings the model and toolpaths back; showing it restores the
    // simulated stock.
    connect(simDock_, &QDockWidget::visibilityChanged, this, [this](bool visible) {
        simPanel_->setDockShown(visible);
        setSimulationMode(visible);
    });
    // The simulation's Close leaves simulation mode.
    connect(simPanel_, &SimulationPanel::closed, simDock_, &QDockWidget::hide);

    // A lathe setup's turret in a window of its own, opened from the setup page's turret
    // section: its stations' table beside the turret preview. It follows the project.
    turretWindow_ = new TurretWindow(this);
    turretWindow_->hide();
    connect(panel_, &PropertyPanel::turretWindowRequested, this,
            [this](const Uuid& setupId, int station) {
                turretWindow_->setDocument(doc_.get());
                turretWindow_->showSetup(setupId, station);
            });
    connect(turretWindow_->panel(), &PropertyPanel::manageFittingsRequested, this,
            &MainWindow::showFittingsLibrary);
    connect(turretWindow_->panel(), &PropertyPanel::editProjectToolsRequested, this,
            &MainWindow::editProjectTools);

    connect(tree_, &ProjectTree::currentSelectionChanged, this, [this](const Selection& s) {
        panel_->showSelection(s);
        refreshView();
        updateStockPreview();
    });
    simulationBanner_ = new QLabel(tr("  Simulation: the program is locked; close the simulation "
                                      "to edit it  "));
    simulationBanner_->setStyleSheet(
        QStringLiteral("background: palette(highlight); color: palette(highlighted-text);"));
    simulationBanner_->hide();
    statusBar()->addPermanentWidget(simulationBanner_);
    // Picking geometry hides the in-process stock, so the part can be picked.
    connect(panel_, &PropertyPanel::pickPrompt, this,
            [this](const QString& prompt) { simPanel_->setSuppressed(!prompt.isEmpty()); });
    // After edits the preview follows once things settle.
    previewTimer_ = new QTimer(this);
    previewTimer_->setSingleShot(true);
    previewTimer_->setInterval(400);
    connect(previewTimer_, &QTimer::timeout, this, &MainWindow::updateStockPreview);
    // The view says when the in-process stock it shows is still being worked out.
    connect(simPanel_, &SimulationPanel::previewBusyChanged, view_, &OcctView::setBusyText);
    connect(simPanel_, &SimulationPanel::simulationFinished, this, [this](bool) {
        if (simPanel_->previewing()) {
            statusBar()->clearMessage();
        }
    });
    connect(tree_, &ProjectTree::contextMenuRequested, this, &MainWindow::showContextMenu);
    connect(tree_, &ProjectTree::operationMoveRequested, this,
            [this](const Uuid& op, const Uuid& setup, int index) {
                if (simulationMode_) {
                    return;
                }
                const document::Setup* from = nullptr;
                const document::Operation* o = doc_->project().findOperation(op, &from);
                if (!o || !doc_->project().findSetup(setup)) {
                    return;
                }
                if (from->id == setup) {
                    const auto at = static_cast<int>(o - from->operations.data());
                    if (index == at || index == at + 1) {
                        return; // dropped where it already was
                    }
                }
                doc_->modify("Move operation", [&](document::Project& pr) {
                    pr.moveOperation(op, setup, static_cast<std::size_t>(std::max(0, index)));
                });
            });
    tree_->setOperationState([this](const Uuid& id) { return toolpathState(id); });
    connect(panel_, &PropertyPanel::editProjectToolsRequested, this, &MainWindow::editProjectTools);
    connect(panel_, &PropertyPanel::manageMachinesRequested, this, &MainWindow::showMachineLibrary);
    connect(panel_, &PropertyPanel::manageWorkholdingRequested, this,
            &MainWindow::showWorkholdingLibrary);
    connect(panel_, &PropertyPanel::manageFittingsRequested, this,
            &MainWindow::showFittingsLibrary);
    connect(panel_, &PropertyPanel::placeOnMachineRequested, this,
            [this](const Uuid& setupId) { placeOnMachine(setupId); });
    connect(panel_, &PropertyPanel::workholdingSetupRequested, this,
            [this](const Uuid& setupId) { setUpWorkholding(setupId); });
    connect(panel_, &PropertyPanel::postSettingsRequested, this,
            [this](const Uuid& setupId) { configurePost(setupId); });
    connect(view_, &OcctView::selectionChanged, this, [this](const QString& text) {
        statusBar()->showMessage(text);
        // May finish a "pick then apply" action; Shift picks single edges.
        panel_->viewSelectionChanged(view_->lastClickWithShift());
    });
    connect(panel_, &PropertyPanel::highlightShapes, this,
            [this](const std::vector<document::ShapeRef>& shapes) {
                std::vector<OcctView::PickedShape> picked;
                for (const auto& s : shapes) {
                    picked.push_back({s.modelId, s.type, s.index});
                }
                view_->selectShapes(picked);
            });
    connect(panel_, &PropertyPanel::toggleSelectionRequested, this, [this](bool on) {
        view_->setToggleSelection(on);
        // Contours: hovering an edge shows its whole outline.
        view_->setOutlineHover(on && panel_->pickingOutlines()
                                   ? OcctView::OutlineFn([this](const Uuid& model, int edge) {
                                         return panel_->outlineOf(model, edge);
                                     })
                                   : OcctView::OutlineFn());
    });
    connect(panel_, &PropertyPanel::vertexPickingRequested, view_, &OcctView::setVertexPicking);
    connect(panel_, &PropertyPanel::workholdingPickingRequested, this, [this](bool on) {
        workholdingPickable_ = on;
        refreshView();
    });
    panel_->setViewPicksProvider([this] {
        std::vector<PropertyPanel::ViewPick> picks;
        for (const auto& picked : view_->selectedShapes()) {
            picks.push_back({picked.model, picked.type, picked.index});
        }
        return picks;
    });
    connect(panel_, &PropertyPanel::pickPrompt, this, [this](const QString& prompt) {
        if (prompt.isEmpty()) {
            statusBar()->clearMessage();
        } else {
            statusBar()->showMessage(prompt);
        }
    });
    auto* cancelPick = new QAction(this);
    cancelPick->setShortcut(QKeySequence(Qt::Key_Escape));
    connect(cancelPick, &QAction::triggered, panel_, &PropertyPanel::cancelPick);
    addAction(cancelPick);
    panel_->setToolpathStatusProvider([this](const Uuid& id) { return toolpathStatus(id); });
    panel_->setPickedShapesProvider([this] {
        std::vector<document::ShapeRef> refs;
        for (const auto& picked : view_->selectedShapes()) {
            if (const cad::Model* m = doc_->model(picked.model)) {
                refs.push_back({picked.model, picked.type, picked.index,
                                cad::signatureOf(*m, picked.type, picked.index)});
            }
        }
        return refs;
    });

    const QSettings settings;

    toolpathMode_ = settings.value(QStringLiteral("toolpathDisplay"), 0).toInt();
    applyColors();
    createActions();
    setDocument(newDocument());
    statusBar()->showMessage(tr("Import a STEP model (File > Import STEP) or open a project."));
}

MainWindow::~MainWindow() {
    // QWidget's destructor may still close the window, hiding the simulation dock after this
    // class's members are gone: its handler must not run then.
    disconnect(simDock_, nullptr, this, nullptr);
    if (loadCancel_) {
        loadCancel_->requestCancel();
    }
    if (jobCancel_) {
        jobCancel_->requestCancel();
    }
    if (doc_) {
        doc_->removeListener(listener_);
    }
}

void MainWindow::createActions() {
    // Every command has a stable id: the user binds keys to it and pins it to the toolbar
    // (Edit > Preferences).
    auto command = [this](const char* id, const QString& category, const QString& label,
                          QAction* action) {
        return commands_.add(QString::fromLatin1(id), category, label, action);
    };
    const QString fileCategory = tr("File");
    QMenu* file = menuBar()->addMenu(tr("&File"));
    command("file.new", fileCategory, tr("New Project"),
            file->addAction(tr("&New Project"), QKeySequence::New, this, &MainWindow::newProject));
    command("file.open", fileCategory, tr("Open Project"),
            file->addAction(tr("&Open Project"), QKeySequence::Open, this,
                            &MainWindow::openProjectDialog));
    recentMenu_ = file->addMenu(tr("Open &Recent"));
    connect(recentMenu_, &QMenu::aboutToShow, this, &MainWindow::updateRecentMenu);
    command("file.save", fileCategory, tr("Save"),
            file->addAction(tr("&Save"), QKeySequence::Save, this, &MainWindow::save));
    command("file.saveAs", fileCategory, tr("Save As"),
            file->addAction(tr("Save &As"), QKeySequence::SaveAs, this, &MainWindow::saveAs));
    file->addSeparator();
    editActions_.push_back(
        command("file.importStep", fileCategory, tr("Import STEP"),
                file->addAction(tr("&Import STEP"), QKeySequence(QStringLiteral("Ctrl+I")), this,
                                &MainWindow::importStepDialog)));
    editActions_.push_back(
        command("file.importFixture", fileCategory, tr("Import Fixture"),
                file->addAction(tr("Import &Fixture"), this, [this] { importFixtureDialog(); })));
    file->addSeparator();
    QAction* startHub =
        command("file.startHub", fileCategory, tr("Start Hub"),
                file->addAction(tr("Start &Hub"), QKeySequence(QStringLiteral("Ctrl+Shift+H")),
                                this, &MainWindow::showStartHub));
    file->addSeparator();
    command("file.exit", fileCategory, tr("Exit"),
            file->addAction(tr("E&xit"), QKeySequence::Quit, this, &QWidget::close));

    const QString editCategory = tr("Edit");
    QMenu* edit = menuBar()->addMenu(tr("&Edit"));
    undo_ =
        command("edit.undo", editCategory, tr("Undo"),
                edit->addAction(tr("&Undo"), QKeySequence::Undo, this, [this] { doc_->undo(); }));
    redo_ =
        command("edit.redo", editCategory, tr("Redo"),
                edit->addAction(tr("&Redo"), QKeySequence::Redo, this, [this] { doc_->redo(); }));
    edit->addSeparator();
    deleteAction_ = command(
        "edit.delete", editCategory, tr("Delete"),
        edit->addAction(tr("&Delete"), QKeySequence::Delete, this, &MainWindow::deleteSelected));
    editActions_.push_back(deleteAction_);
    renameAction_ = command("edit.rename", editCategory, tr("Rename"),
                            edit->addAction(tr("Re&name"), QKeySequence(QStringLiteral("F2")), this,
                                            &MainWindow::renameSelected));
    editActions_.push_back(renameAction_);
    edit->addSeparator();
    command("edit.preferences", editCategory, tr("Preferences"),
            edit->addAction(tr("&Preferences"), QKeySequence::Preferences, this,
                            [this] { showPreferences(); }));

    const QString setupCategory = tr("Setup");
    QMenu* setup = menuBar()->addMenu(tr("&Setup"));
    editActions_.push_back(
        command("setup.add", setupCategory, tr("Add Setup"),
                setup->addAction(tr("Add &Setup"), this, &MainWindow::addSetup)));
    simulateAction_ = command("setup.simulate", setupCategory, tr("Simulate"),
                              setup->addAction(tr("Si&mulate"), QKeySequence(QStringLiteral("F5")),
                                               this, &MainWindow::simulateCurrentSetup));
    postAction_ =
        command("setup.post", setupCategory, tr("Post Process"),
                setup->addAction(tr("&Post Process"), QKeySequence(QStringLiteral("Ctrl+P")), this,
                                 &MainWindow::postProcessSetup));
    editActions_.push_back(command("setup.regenerate", setupCategory, tr("Regenerate Toolpaths"),
                                   setup->addAction(tr("&Regenerate Toolpaths"), this, [this] {
                                       if (const auto id = currentSetupId()) {
                                           regenerateToolpaths(*id);
                                       }
                                   })));
    setup->addSeparator();
    createOperationActions();
    QMenu* addOperationMenu = setup->addMenu(tr("Add &Operation"));
    populateOperationMenu(addOperationMenu, true);
    editActions_.push_back(addOperationMenu->menuAction());

    const QString toolsCategory = tr("Tools");
    QMenu* tools = menuBar()->addMenu(tr("&Tools"));
    command("tools.toolLibraries", toolsCategory, tr("Tool Libraries"),
            tools->addAction(tr("Tool Li&braries"), this, &MainWindow::showToolLibraries));
    editActions_.push_back(
        command("tools.projectTools", toolsCategory, tr("Project Tools"),
                tools->addAction(tr("&Project Tools"), this, &MainWindow::editProjectTools)));
    tools->addSeparator();
    command("tools.machineLibrary", toolsCategory, tr("Machine Library"),
            tools->addAction(tr("&Machine Library"), this, &MainWindow::showMachineLibrary));
    command(
        "tools.workholdingLibrary", toolsCategory, tr("Workholding Library"),
        tools->addAction(tr("&Workholding Library"), this, &MainWindow::showWorkholdingLibrary));
    command("tools.fittingsLibrary", toolsCategory, tr("Turret Blocks and Tailstock Centres"),
            tools->addAction(tr("Turret &Blocks and Tailstock Centres"), this,
                             &MainWindow::showFittingsLibrary));

    const QString viewCategory = tr("View");
    QMenu* viewMenu = menuBar()->addMenu(tr("&View"));
    viewMenu->addAction(command("view.simulation", viewCategory, tr("Simulation Panel"),
                                simDock_->toggleViewAction()));
    viewMenu->addSeparator();
    QAction* fit = viewMenu->addAction(tr("&Fit All"), this, [this] { view_->fitAll(); });
    // F, or Alt + 0 (main row or keypad) like the standard views below.
    fit->setShortcuts(
        {QKeySequence(QStringLiteral("F")),
         QKeySequence(QKeyCombination(Qt::AltModifier, Qt::Key_0)),
         QKeySequence(QKeyCombination(Qt::AltModifier | Qt::KeypadModifier, Qt::Key_0))});
    command("view.fitAll", viewCategory, tr("Fit All"), fit);
    // Standard views in the current setup's frame: the view cube, or Alt + a digit (main row or
    // keypad).
    QMenu* standardViews = viewMenu->addMenu(tr("Standard &Views"));
    const struct {
        const char* id;
        Qt::Key key;
        V3d_TypeOfOrientation orientation;
        const char* name;
    } views[] = {
        {"view.top", Qt::Key_1, V3d_Zpos, QT_TR_NOOP("Top")},
        {"view.front", Qt::Key_2, V3d_Yneg, QT_TR_NOOP("Front")},
        {"view.right", Qt::Key_3, V3d_Xpos, QT_TR_NOOP("Right")},
        {"view.bottom", Qt::Key_4, V3d_Zneg, QT_TR_NOOP("Bottom")},
        {"view.back", Qt::Key_5, V3d_Ypos, QT_TR_NOOP("Back")},
        {"view.left", Qt::Key_6, V3d_Xneg, QT_TR_NOOP("Left")},
        {"view.isometric", Qt::Key_7, V3d_XposYnegZpos, QT_TR_NOOP("Isometric")},
        {"view.rearIsometric", Qt::Key_8, V3d_XnegYposZpos, QT_TR_NOOP("Rear isometric")},
    };
    for (const auto& v : views) {
        const V3d_TypeOfOrientation o = v.orientation;
        const QString name = tr(v.name);
        QAction* a = standardViews->addAction(name, this, [this, o, name] {
            view_->snapView(o);
            statusBar()->showMessage(tr("%1 view").arg(name), 2000);
        });
        a->setShortcuts(
            {QKeySequence(QKeyCombination(Qt::AltModifier, v.key)),
             QKeySequence(QKeyCombination(Qt::AltModifier | Qt::KeypadModifier, v.key))});
        // One key can step through the views (Edit > Preferences > Keyboard Shortcuts).
        commands_.add(QString::fromLatin1(v.id), viewCategory, tr("%1 View").arg(name), a,
                      QStringLiteral("views"));
    }
    viewMenu->addSeparator();
    previewAction_ =
        viewMenu->addAction(tr("In-Process &Stock"), this, [this] { updateStockPreview(); });
    previewAction_->setCheckable(true);
    previewAction_->setShortcut(QKeySequence(QStringLiteral("Ctrl+Shift+P")));
    previewAction_->setToolTip(
        formatToolTip(tr("Show the stock after the selected operation and the ones before it "
                         "(simulated as needed)"),
                      tr("In-Process Stock")));
    previewAction_->setChecked(QSettings().value(QStringLiteral("stockPreview"), false).toBool());
    connect(previewAction_, &QAction::toggled, this,
            [](bool on) { QSettings().setValue(QStringLiteral("stockPreview"), on); });
    command("view.inProcessStock", viewCategory, tr("In-Process Stock"), previewAction_);
    transparentStockAction_ = viewMenu->addAction(tr("&Transparent Stock"));
    transparentStockAction_->setCheckable(true);
    transparentStockAction_->setToolTip(
        formatToolTip(tr("See through the simulated stock"), tr("Transparent Stock")));
    transparentStockAction_->setChecked(
        QSettings().value(QStringLiteral("transparentStock"), false).toBool());
    view_->setStockTransparency(transparentStockAction_->isChecked() ? kStockTransparency : 0.0);
    connect(transparentStockAction_, &QAction::toggled, this, [this](bool on) {
        QSettings().setValue(QStringLiteral("transparentStock"), on);
        view_->setStockTransparency(on ? kStockTransparency : 0.0);
    });
    command("view.transparentStock", viewCategory, tr("Transparent Stock"),
            transparentStockAction_);
    QMenu* pathsMenu = viewMenu->addMenu(tr("&Toolpaths"));
    auto* pathsGroup = new QActionGroup(this);
    const struct {
        const char* id;
        const char* text;
    } pathModes[] = {{"view.toolpaths.all", QT_TR_NOOP("All Operations of the Setup")},
                     {"view.toolpaths.selected", QT_TR_NOOP("Selected Operation Only")},
                     {"view.toolpaths.hidden", QT_TR_NOOP("Hidden")}};
    for (int mode = 0; mode < 3; ++mode) {
        QAction* a = pathsMenu->addAction(tr(pathModes[mode].text), this, [this, mode] {
            toolpathMode_ = mode;
            QSettings().setValue(QStringLiteral("toolpathDisplay"), mode);
            refreshToolpathDisplay();
        });
        a->setCheckable(true);
        a->setChecked(mode == toolpathMode_);
        pathsGroup->addAction(a);
        // One key can toggle the toolpaths, or step through the modes.
        commands_.add(QString::fromLatin1(pathModes[mode].id), viewCategory,
                      tr("Toolpaths: %1").arg(tr(pathModes[mode].text)), a,
                      QStringLiteral("toolpaths"));
    }

    QMenu* help = menuBar()->addMenu(tr("&Help"));
    help->addAction(startHub);
    command("help.about", tr("Help"), tr("About OxCAM"),
            help->addAction(tr("&About OxCAM"), this, [this] {
                QMessageBox::about(this, tr("About OxCAM"),
                                   tr("<b>OxCAM</b> %1<br>Clean-room open-source CAM.<br>"
                                      "Licensed under GPL-3.0-or-later.")
                                       .arg(version()));
            }));

    // What the commands do: the tooltips of their toolbar buttons (Simulate, pinned commands).
    const std::pair<const char*, const char*> tips[] = {
        {"file.new", QT_TR_NOOP("Start a new, empty project")},
        {"file.open", QT_TR_NOOP("Open a saved project (.ocam)")},
        {"file.save", QT_TR_NOOP("Save the project")},
        {"file.saveAs", QT_TR_NOOP("Save the project under a new name")},
        {"file.importStep", QT_TR_NOOP("Import a STEP model into the project")},
        {"file.startHub", QT_TR_NOOP("New and recent projects, STEP import, tips and what is new")},
        {"edit.undo", QT_TR_NOOP("Undo the last change")},
        {"edit.redo", QT_TR_NOOP("Redo the change undone last")},
        {"edit.delete", QT_TR_NOOP("Delete the item selected in the project tree")},
        {"edit.rename", QT_TR_NOOP("Rename the item selected in the project tree")},
        {"edit.preferences", QT_TR_NOOP("Units, language, keyboard shortcuts, toolbar buttons and "
                                        "the starting values of new operations")},
        {"setup.add", QT_TR_NOOP("Add a setup: a work coordinate system, stock and machine for a "
                                 "set of operations")},
        {"setup.simulate", QT_TR_NOOP("Simulate the current setup on its stock: material "
                                      "removal, gouges and machine collisions")},
        {"setup.post", QT_TR_NOOP("Choose the post and its options, then post process the "
                                  "current setup into an NC program for its machine")},
        {"setup.regenerate", QT_TR_NOOP("Generate the toolpaths of the current setup again")},
        {"tools.toolLibraries", QT_TR_NOOP("Manage your tool libraries and choose the default")},
        {"tools.projectTools", QT_TR_NOOP("Edit the tools this project uses")},
        {"tools.machineLibrary", QT_TR_NOOP("Your machine definitions: kinematics, travel, "
                                            "spindle, feeds and collision parts")},
        {"tools.workholdingLibrary", QT_TR_NOOP("Your vises: bodies from STEP files, the moving "
                                                "jaw and its range, for the setups to hold the "
                                                "stock in")},
        {"view.simulation", QT_TR_NOOP("Show or hide the simulation panel")},
        {"view.fitAll", QT_TR_NOOP("Fit the whole model in the view")},
    };
    for (const auto& [id, tip] : tips) {
        if (const Command* c = commands_.find(QString::fromLatin1(id))) {
            c->action->setToolTip(formatToolTip(tr(tip), c->label));
        }
    }
    commands_.applyShortcuts(); // keys commands share step through them
    createToolBar();
}

namespace {
struct OperationEntry {
    const char* type;
    const char* label; ///< In menus.
    const char* name;  ///< Of the new operation.
};
struct OperationGroup {
    const char* title;
    const char* tip; ///< What the group's operations are for.
    std::vector<OperationEntry> entries;
};
const std::vector<OperationGroup>& operationGroups() {
    static const std::vector<OperationGroup> groups = {
        {QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "2D"),
         QT_TRANSLATE_NOOP(
             "ocam::gui::MainWindow",
             "Machining from above: facing, pockets, contours, chamfers, traces along edges and "
             "plunge roughing"),
         {{"facing", QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Facing"),
           QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Facing")},
          {"pocket2d", QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "2D pocket"),
           QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Pocket")},
          {"contour2d", QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "2D contour"),
           QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Contour")},
          {"chamfer2d", QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Chamfer"),
           QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Chamfer")},
          {"trace2d", QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "2D trace"),
           QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Trace")},
          {"plunge_rough", QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Plunge roughing"),
           QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Plunge rough")}}},
        {QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Holes"),
         QT_TRANSLATE_NOOP(
             "ocam::gui::MainWindow",
             "Hole making: drilling cycles, tapping, thread milling, boring, and back boring and "
             "chamfering"),
         {{"drill", QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Drilling"),
           QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Drill")},
          {"tap", QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Tapping"),
           QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Tap")},
          {"threadmill", QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Thread milling"),
           QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Thread mill")},
          {"bore", QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Boring"),
           QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Bore")},
          {"backbore", QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Back boring"),
           QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Back bore")}}},
        {QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "3D"),
         QT_TRANSLATE_NOOP("ocam::gui::MainWindow",
                           "Machining 3D surfaces: roughing, then parallel, contour, waterline, "
                           "flat area, steep and shallow, and pencil finishing"),
         {{"zlevel_rough", QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "3D roughing"),
           QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "3D rough")},
          {"parallel3d", QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "3D parallel finishing"),
           QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Parallel finish")},
          {"contour3d", QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "3D contour finishing"),
           QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Contour finish")},
          {"waterline3d", QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "3D waterline finishing"),
           QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Waterline")},
          {"flat3d", QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "3D flat area finishing"),
           QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Flat areas")},
          {"steep_shallow3d",
           QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "3D steep and shallow finishing"),
           QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Steep and shallow")},
          {"pencil3d", QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "3D pencil finishing"),
           QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Pencil")},
          {"edgebreak3d", QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "3D edge break"),
           QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Edge break")}}},
        {QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Multi-axis"),
         QT_TRANSLATE_NOOP("ocam::gui::MainWindow",
                           "Wrapped pockets and contours on the 4th axis or a lathe's C axis, and "
                           "simultaneous 5-axis finishing and swarf"),
         {{"rotary_pocket", QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Wrapped pocket"),
           QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Wrapped pocket")},
          {"rotary_contour", QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Wrapped contour"),
           QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Wrapped contour")},
          {"multiaxis_finish", QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "5-axis finishing"),
           QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "5-axis finish")},
          {"swarf5x", QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "5-axis swarf"),
           QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Swarf")}}},
        {QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Turning"),
         QT_TRANSLATE_NOOP(
             "ocam::gui::MainWindow",
             "Lathe operations: facing, roughing, finishing, grooving, threading and parting off, "
             "pulling the bar, and drilling, tapping and reaming on the spindle axis"),
         {{"turn_face", QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Face"),
           QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Face")},
          {"turn_rough", QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Rough"),
           QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Rough turn")},
          {"turn_finish", QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Finish"),
           QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Finish turn")},
          {"turn_groove", QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Groove"),
           QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Groove")},
          {"turn_thread", QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Thread"),
           QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Thread")},
          {"turn_partoff", QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Part off"),
           QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Part off")},
          {"bar_pull", QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Bar pull"),
           QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Bar pull")},
          {"spindle_transfer", QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Sub-spindle transfer"),
           QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Transfer")},
          {"turn_soft_jaws", QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Bore soft jaws"),
           QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Bore soft jaws")}}},
        // Probing with a spindle probe (DL-0090).
        {QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Probing"),
         QT_TRANSLATE_NOOP("ocam::gui::MainWindow",
                           "A spindle probe: set a work offset from a surface, web, pocket, bore, "
                           "boss or corner, or check a feature against the model"),
         {{"probe_surface", QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Surface"),
           QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Probe surface")},
          {"probe_width", QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Web or pocket width"),
           QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Probe width")},
          {"probe_circle", QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Bore or boss centre"),
           QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Probe bore")},
          {"probe_corner", QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Outside or inside corner"),
           QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Probe corner")},
          {"probe_check", QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Feature check"),
           QT_TRANSLATE_NOOP("ocam::gui::MainWindow", "Feature check")}}},
    };
    return groups;
}
} // namespace

void MainWindow::createOperationActions() {
    // One action per operation type, shared by the menus, the toolbar and the context menus, so
    // a key binding or a pin applies everywhere.
    for (std::size_t g = 0; g < operationGroups().size(); ++g) {
        const OperationGroup& group = operationGroups()[g];
        std::vector<QAction*>& actions = operationActions_.emplace_back();
        for (const OperationEntry& e : group.entries) {
            const QString type = QString::fromLatin1(e.type);
            const QString name = tr(e.name);
            auto* a = new QAction(tr(e.label), this);
            a->setIconText(name); // on a pinned toolbar button
            // What the operation is for: in the menus, on a pinned button.
            const strategies::StrategyInfo* info = strategies::findStrategy(e.type);
            a->setToolTip(
                formatToolTip(info ? trStrategy(info->description) : QString(), tr(e.label)));
            connect(a, &QAction::triggered, this, [this, type, name] { addOperation(type, name); });
            commands_.add(QStringLiteral("operation.") + type,
                          tr("Add Operation: %1").arg(tr(group.title)), tr(e.label), a);
            actions.push_back(a);
            editActions_.push_back(a); // locked while simulating
        }
    }
}

void MainWindow::addOperationActions(QMenu* menu, std::size_t group) {
    for (QAction* a : operationActions_[group]) {
        menu->addAction(a);
    }
    // A lathe drills on its spindle axis with the same operations as a mill (DL-0043).
    if (std::string_view(operationGroups()[group].entries.front().type) != "turn_face") {
        return;
    }
    menu->addSeparator();
    for (std::size_t g = 0; g < operationGroups().size(); ++g) {
        const auto& entries = operationGroups()[g].entries;
        for (std::size_t k = 0; k < entries.size(); ++k) {
            const std::string_view type = entries[k].type;
            if (type == "drill" || type == "tap" || type == "bore") {
                menu->addAction(operationActions_[g][k]);
            }
        }
    }
}

void MainWindow::populateOperationMenu(QMenu* addOp, bool toolTips) {
    for (std::size_t g = 0; g < operationGroups().size(); ++g) {
        QMenu* menu = addOp->addMenu(tr(operationGroups()[g].title));
        menu->setToolTipsVisible(toolTips);
        addOperationActions(menu, g);
    }
}

void MainWindow::createToolBar() {
    // Sections with a caption under their buttons: toolpaths, simulation, stock display, and
    // the commands the user pinned.
    QToolBar* bar = addToolBar(tr("Main"));
    bar->setObjectName(QStringLiteral("mainToolBar"));
    bar->setMovable(false);
    // Checked toggles show a solid highlight, not the style's dotted pattern.
    bar->setStyleSheet(QStringLiteral(
        "QToolButton { padding: 4px 9px; border-radius: 3px; }"
        "QToolButton:hover { background: palette(midlight); }"
        "QToolButton:checked { background: palette(highlight); color: palette(highlighted-text); }"
        "QToolButton::menu-indicator { image: none; }"));
    // Returns the row the buttons are in and the section's toolbar entries (box, separator).
    auto section = [&](const QString& title, const std::vector<QWidget*>& buttons) {
        auto* box = new QWidget(bar);
        auto* column = new QVBoxLayout(box);
        column->setContentsMargins(6, 3, 6, 1);
        column->setSpacing(1);
        auto* row = new QHBoxLayout;
        row->setSpacing(2);
        for (QWidget* w : buttons) {
            w->setMinimumHeight(30);
            row->addWidget(w);
        }
        column->addLayout(row);
        auto* caption = new QLabel(title, box);
        caption->setAlignment(Qt::AlignCenter);
        caption->setStyleSheet(QStringLiteral("color: gray; font-size: 8pt;"));
        column->addWidget(caption);
        QAction* entry = bar->addWidget(box);
        QAction* separator = bar->addSeparator();
        return std::tuple{row, entry, separator};
    };
    auto actionButton = [&](QAction* action) {
        auto* button = new QToolButton(bar);
        button->setDefaultAction(action); // follows the action, checked state included
        return button;
    };
    std::vector<QWidget*> toolpathButtons;
    for (std::size_t g = 0; g < operationGroups().size(); ++g) {
        auto* button = new QToolButton(bar);
        button->setText(tr(operationGroups()[g].title) + QStringLiteral(" ▾"));
        button->setToolTip(
            formatToolTip(tr(operationGroups()[g].tip), tr(operationGroups()[g].title)));
        button->setPopupMode(QToolButton::InstantPopup);
        auto* menu = new QMenu(button);
        menu->setToolTipsVisible(true); // what each operation is for
        addOperationActions(menu, g);
        button->setMenu(menu);
        toolpathButtons.push_back(button);
        editWidgets_.push_back(button); // locked while simulating
    }
    section(tr("Toolpaths"), toolpathButtons);
    section(tr("Simulation"), {actionButton(simulateAction_)});
    previewAction_->setIconText(tr("In-Process"));
    transparentStockAction_->setIconText(tr("Transparent"));
    section(tr("Stock"), {actionButton(previewAction_), actionButton(transparentStockAction_)});
    const auto [row, entry, separator] = section(tr("Pinned"), {});
    pinnedRow_ = row;
    pinnedSection_ = {entry, separator};
    updatePinnedSection();
}

void MainWindow::updatePinnedSection() {
    while (QLayoutItem* item = pinnedRow_->takeAt(0)) {
        delete item->widget();
        delete item;
    }
    for (const QString& id : pinnedCommands()) {
        if (const Command* c = commands_.find(id)) {
            auto* button = new QToolButton;
            button->setDefaultAction(c->action);
            button->setMinimumHeight(30);
            pinnedRow_->addWidget(button);
        }
    }
    const bool any = pinnedRow_->count() > 0;
    for (QAction* a : pinnedSection_) {
        a->setVisible(any);
    }
}

void MainWindow::setDocument(std::unique_ptr<document::Document> doc) {
    if (doc_) {
        doc_->removeListener(listener_);
    }
    if (jobCancel_) {
        jobCancel_->requestCancel();
    }
    jobs_.clear();
    toolpaths_.clear();
    simPanel_->closeSimulation();
    // Another document leaves simulation mode: the simulation shown was of the one before.
    simDock_->hide();
    setSimulationMode(false);
    doc_ = std::move(doc);
    listener_ = doc_->addListener([this] {
        simPanel_->markStale();
        scheduleRefresh();
        if (simPanel_->previewing() && previewTimer_) {
            previewTimer_->start(); // follow the edit with the preview
        }
    });
    panel_->setDocument(doc_.get());
    turretWindow_->setDocument(doc_.get());
    prepareOnOpen_ = !doc_->project().setups.empty();
    tree_->clear(); // a new document starts with a fully expanded tree
    view_->setModels({});
    refresh();
}

void MainWindow::scheduleRefresh() {
    // Changes often come from widgets inside the panel that a refresh rebuilds, so refresh on
    // the next event loop iteration rather than inside their signal handlers.
    if (!refreshPending_) {
        refreshPending_ = true;
        QTimer::singleShot(0, this, [this] {
            refreshPending_ = false;
            refresh();
        });
    }
}

void MainWindow::refresh() {
    updateToolpaths();
    tree_->rebuild(doc_->project());
    panel_->showSelection(tree_->selection());
    turretWindow_->refresh();
    refreshView();
    undo_->setEnabled(doc_->canUndo() && !simulationMode_);
    undo_->setText(doc_->canUndo() ? tr("&Undo %1").arg(QString::fromStdString(doc_->undoLabel()))
                                   : tr("&Undo"));
    redo_->setEnabled(doc_->canRedo() && !simulationMode_);
    redo_->setText(doc_->canRedo() ? tr("&Redo %1").arg(QString::fromStdString(doc_->redoLabel()))
                                   : tr("&Redo"));
    updateTitle();
}

void MainWindow::refreshView() {
    const auto setupId = currentSetupId();
    const document::Setup* setup = setupId ? doc_->project().findSetup(*setupId) : nullptr;
    // Fixtures show with the current setup only (each setup holds the part its own way).
    std::set<Uuid> fixtures;
    for (const auto& s : doc_->project().setups) {
        fixtures.insert(s.fixtures.begin(), s.fixtures.end());
    }
    std::vector<OcctView::ViewModel> models;
    for (const auto& entry : doc_->project().models) {
        const bool fixture = fixtures.contains(entry.id);
        if (fixture && !(setup && std::find(setup->fixtures.begin(), setup->fixtures.end(),
                                            entry.id) != setup->fixtures.end())) {
            continue;
        }
        if (const cad::Model* m = doc_->model(entry.id)) {
            models.push_back({entry.id, *m, entry.placement.toParent(), fixture});
        }
    }
    // The current setup's vise, placed around its stock (pickable only for a WCS origin on it:
    // its parts are no project models).
    if (setup) {
        try {
            // Stock beyond its reach: the jaws as far as they go, in red (the setup page says why).
            if (const auto pose = document::shownWorkholdingPose(*doc_, *setup)) {
                const Transform3 wcsToProject = setup->wcs.toParent();
                for (const document::PosedPart& body : pose->parts) {
                    if (!body.shown) {
                        continue;
                    }
                    if (const cad::Model* m = document::posedPartModel(*doc_, body)) {
                        models.push_back({body.id, *m, wcsToProject * body.toWcs, true,
                                          workholdingPickable_,
                                          body.outOfReach ? std::optional<std::uint32_t>(0xD03A2Fu)
                                                          : std::nullopt});
                    }
                }
            }
        } catch (const Error&) {
            // the setup page says why the vise cannot be placed
        }
        try {
            for (const document::LatheSupport& support : document::latheSupports(*doc_, *setup)) {
                models.push_back({support.id, support.model, setup->wcs.toParent(), true, false});
            }
        } catch (const Error&) {
            // the simulation and the toolpaths say why
        }
    }
    view_->setModels(models);
    simPanel_->setCurrentSetup(setup ? std::optional(setup->id) : std::nullopt);
    view_->setStock(setup ? doc_->stockShape(setup->id) : std::nullopt);
    // The triad shows the tool axis: the selected operation's own orientation, else the WCS.
    std::optional<document::Frame> triad;
    if (setup) {
        triad = setup->wcs;
        const Selection selected = tree_->selection();
        const document::Operation* op = selected.kind == Selection::Kind::Operation
                                            ? doc_->project().findOperation(selected.id)
                                            : nullptr;
        if (op && op->orientation) {
            try {
                triad = document::operationSetup(*setup, *op).wcs;
            } catch (const Error&) {
                // an invalid orientation shows the setup's WCS
            }
        }
    }
    view_->setWcs(triad);
    // Standard views and the view cube follow the current setup's WCS, and how its machine stands
    // (DL-0077): a lathe's spindle across the view, a mill's or a vertical lathe's Z up. Another
    // machine's standing turns the view to it.
    Mat3 frame = Mat3::Identity();
    Mat3 standing = Mat3::Identity();
    if (setup) {
        try {
            frame = setup->wcs.normalized().toParent().linear();
        } catch (const Error&) {
            // an invalid WCS keeps the project axes
        }
        if (const machine::Machine* m = document::findMachineFor(doc_->project(), *setup)) {
            standing = m->standing().worldAxes();
        }
    }
    if (view_->setReferenceFrame(frame, standing)) {
        view_->snapView(V3d_XposYnegZpos);
    }
    view_->setDisplayUnit(doc_->project().displayUnit);
    simPanel_->setDisplayUnit(doc_->project().displayUnit);
    refreshToolpathDisplay();
}

void MainWindow::updateToolpaths() {
    const document::Project& project = doc_->project();
    std::set<Uuid> live;
    for (const auto& setup : project.setups) {
        for (const auto& op : setup.operations) {
            if (!op.enabled) {
                continue;
            }
            live.insert(op.id);
            const std::string key = document::toolpathInputKey(*doc_, setup, op);
            ToolpathEntry& entry = toolpaths_[op.id];
            if (entry.key == key) {
                continue;
            }
            entry = ToolpathEntry{key, nullptr, QString(), true};
            std::erase_if(jobs_, [&](const ToolpathJob& j) { return j.operationId == op.id; });
            jobs_.push_back({op.id, setup.id, key});
        }
    }
    std::erase_if(toolpaths_, [&](const auto& item) { return !live.contains(item.first); });
    if (jobRunning_ && jobCancel_) {
        auto it = toolpaths_.find(runningJob_.operationId);
        if (it == toolpaths_.end() || it->second.key != runningJob_.key) {
            jobCancel_->requestCancel(); // its inputs changed: the result would be stale
        }
    }
    startNextToolpathJob();
    refreshToolpathData();
}

void MainWindow::startNextToolpathJob() {
    if (!jobRunning_ && jobs_.empty() && prepareOnOpen_ && doc_) {
        // A project just opened has its toolpaths: work out every setup's in-process stock in
        // the background (from the local simulation cache when it has them).
        prepareOnOpen_ = false;
        std::vector<Uuid> setups;
        for (const auto& setup : doc_->project().setups) {
            setups.push_back(setup.id);
        }
        simPanel_->prepareSetups(setups);
    }
    if (jobRunning_ || jobs_.empty()) {
        return;
    }
    runningJob_ = jobs_.front();
    jobs_.pop_front();
    jobRunning_ = true;
    auto cancel = std::make_shared<CancelToken>();
    jobCancel_ = cancel;

    struct Result {
        std::shared_ptr<const toolpath::Toolpath> path;
        QString error;
    };
    auto result = std::make_shared<Result>();
    auto snapshot = std::make_shared<const document::Document>(doc_->snapshot());
    const ToolpathJob job = runningJob_;
    runInBackground(
        this,
        [result, snapshot, job, cancel] {
            try {
                const document::Setup* setup = snapshot->project().findSetup(job.setupId);
                const document::Operation* op = snapshot->project().findOperation(job.operationId);
                if (!setup || !op) {
                    return;
                }
                NullProgress progress;
                // The snapshot shares the document's cache: toolpaths saved with the project
                // or generated before come back at once.
                result->path = document::cachedToolpath(*snapshot, *setup, *op, *cancel, progress);
            } catch (const OperationCancelled&) {
                // superseded
            } catch (const std::exception& e) {
                result->error = QString::fromUtf8(e.what());
            }
        },
        [this, result, job] {
            jobRunning_ = false;
            auto it = toolpaths_.find(job.operationId);
            if (it != toolpaths_.end() && it->second.key == job.key) {
                it->second.pending = false;
                it->second.path = result->path;
                it->second.error = result->error;
                refreshToolpathDisplay();
                panel_->refreshToolpathStatus();
                tree_->refreshStates();
                refreshToolpathData();
            }
            startNextToolpathJob();
        });
}

void MainWindow::refreshToolpathDisplay() {
    std::vector<OcctView::ToolpathView> views;
    const auto setupId = currentSetupId();
    const document::Setup* setup = setupId ? doc_->project().findSetup(*setupId) : nullptr;
    if (setup) {
        const Selection selected = tree_->selection();
        const Transform3 wcsToProject = setup->wcs.toParent();
        for (const auto& op : setup->operations) {
            auto it = toolpaths_.find(op.id);
            const bool isSelected =
                selected.kind == Selection::Kind::Operation && selected.id == op.id;
            if (toolpathMode_ == 2 || (toolpathMode_ == 1 && !isSelected)) {
                continue;
            }
            if (op.enabled && it != toolpaths_.end() && it->second.path) {
                const bool emphasized = isSelected;
                views.push_back({op.id, it->second.path, wcsToProject, emphasized});
            }
        }
    }
    view_->setToolpaths(views);
}

ProjectTree::OperationState MainWindow::toolpathState(const Uuid& operationId) const {
    using State = ProjectTree::OperationState;
    auto it = toolpaths_.find(operationId);
    if (it == toolpaths_.end()) {
        return State::None; // disabled or not generated
    }
    if (it->second.pending) {
        return State::Generating;
    }
    if (!it->second.error.isEmpty()) {
        return State::Error;
    }
    if (!it->second.path) {
        return State::None;
    }
    return it->second.path->warnings.empty() ? State::Ready : State::Warning;
}

QString MainWindow::toolpathStatus(const Uuid& operationId) const {
    auto it = toolpaths_.find(operationId);
    if (it == toolpaths_.end()) {
        return tr("Disabled.");
    }
    const ToolpathEntry& e = it->second;
    if (e.pending) {
        return tr("Generating…");
    }
    if (!e.error.isEmpty()) {
        return QStringLiteral("⚠ ") + e.error;
    }
    if (!e.path) {
        return {};
    }
    const toolpath::Stats stats = toolpath::computeStats(*e.path);
    const int seconds = static_cast<int>(std::lround(stats.totalTime));
    QString text = tr("%1 moves, feed length %2, estimated %3:%4")
                       .arg(stats.moveCount)
                       .arg(formatLength(stats.cutLength, doc_->project().displayUnit))
                       .arg(seconds / 60)
                       .arg(seconds % 60, 2, 10, QLatin1Char('0'));
    for (const auto& w : e.path->warnings) {
        text += QStringLiteral("\n• ") + QString::fromStdString(w);
    }
    return text;
}

void MainWindow::showToolpathData(const Uuid& operationId) {
    QPointer<ToolpathDataDialog>& window = toolpathData_[operationId];
    if (!window) {
        window = new ToolpathDataDialog(this);
    }
    QPointer<ToolpathDataDialog> shown = window; // the refresh may tidy the map
    refreshToolpathData();
    if (shown) {
        shown->show();
        shown->raise();
        shown->activateWindow();
    }
}

void MainWindow::refreshToolpathData() {
    for (auto it = toolpathData_.begin(); it != toolpathData_.end();) {
        ToolpathDataDialog* window = it->second;
        const document::Operation* op = window ? doc_->project().findOperation(it->first) : nullptr;
        if (!op) {
            if (window) {
                window->close(); // the operation is gone (deleted, or another project opened)
            }
            it = toolpathData_.erase(it);
            continue;
        }
        window->setOperationName(QString::fromStdString(op->name));
        const auto entry = toolpaths_.find(op->id);
        if (!op->enabled) {
            window->showMessage(tr("This operation is disabled, so it has no toolpath. Turn on "
                                   "Enabled in its right-click menu to generate one."));
        } else if (entry == toolpaths_.end() || entry->second.pending) {
            window->showMessage(
                tr("Generating the toolpath… Its moves show here when it is done."));
        } else if (!entry->second.error.isEmpty()) {
            window->showMessage(tr("This operation has no toolpath, because it could not be "
                                   "generated:\n\n%1")
                                    .arg(entry->second.error));
        } else if (!entry->second.path) {
            window->showMessage(tr("This operation has no toolpath yet. Use Regenerate Toolpath "
                                   "in its right-click menu."));
        } else if (entry->second.path->isEmpty()) {
            QString text = tr("This operation's toolpath has no moves.");
            for (const auto& w : entry->second.path->warnings) {
                text += QStringLiteral("\n• ") + QString::fromStdString(w);
            }
            window->showMessage(text);
        } else {
            window->showPath(entry->second.path, doc_->project().displayUnit);
        }
        ++it;
    }
}

void MainWindow::updateTitle() {
    const QString name = doc_->path() ? QFileInfo(fromPath(*doc_->path())).fileName()
                                      : QString::fromStdString(doc_->project().name);
    setWindowTitle(
        QStringLiteral("%1%2 - OxCAM %3")
            .arg(name, doc_->isModified() ? QStringLiteral(" *") : QString(), version()));
}

std::optional<Uuid> MainWindow::currentSetupId() const {
    const Selection s = tree_->selection();
    const document::Project& p = doc_->project();
    if (s.kind == Selection::Kind::Setup && p.findSetup(s.id)) {
        return s.id;
    }
    if (s.kind == Selection::Kind::Operation) {
        const document::Setup* owner = nullptr;
        if (p.findOperation(s.id, &owner)) {
            return owner->id;
        }
    }
    if (!p.setups.empty()) {
        return p.setups.front().id;
    }
    return std::nullopt;
}

void MainWindow::openFile(const QString& path) {
    if (path.endsWith(QStringLiteral(".ocam"), Qt::CaseInsensitive)) {
        openProject(path);
    } else {
        importStep(path);
    }
}

std::unique_ptr<document::Document> MainWindow::newDocument() const {
    document::Project project;
    project.displayUnit = preferredUnit();
    return std::make_unique<document::Document>(std::move(project));
}

void MainWindow::newProject() {
    if (maybeSave()) {
        setDocument(newDocument());
    }
}

void MainWindow::openProjectDialog() {
    if (!maybeSave()) {
        return;
    }
    const QString path = QFileDialog::getOpenFileName(this, tr("Open project"), QString(),
                                                      tr("OxCAM projects (*.ocam)"));
    if (!path.isEmpty()) {
        openProject(path);
    }
}

void MainWindow::importStepDialog() {
    const QString path = QFileDialog::getOpenFileName(this, tr("Import STEP model"), QString(),
                                                      tr("STEP files (*.step *.stp *.STEP *.STP)"));
    if (!path.isEmpty()) {
        importStep(path);
    }
}

void MainWindow::importFixtureDialog(std::optional<Uuid> setupId) {
    if (!setupId) {
        setupId = currentSetupId();
    }
    const document::Setup* setup = setupId ? doc_->project().findSetup(*setupId) : nullptr;
    if (!setup) {
        QMessageBox::information(this, tr("Import fixture"),
                                 tr("Add a setup first: a fixture holds the part in a setup."));
        return;
    }
    const QString path = QFileDialog::getOpenFileName(
        this, tr("Import fixture into %1").arg(QString::fromStdString(setup->name)), QString(),
        tr("STEP files (*.step *.stp *.STEP *.STP)"));
    if (!path.isEmpty()) {
        importStep(path, setup->id);
    }
}

void MainWindow::importStep(const QString& path, std::optional<Uuid> fixtureFor) {
    if (loadCancel_) {
        loadCancel_->requestCancel();
    }
    auto cancel = std::make_shared<CancelToken>();
    loadCancel_ = cancel;
    statusBar()->showMessage(tr("Importing %1...").arg(QFileInfo(path).fileName()));
    QApplication::setOverrideCursor(Qt::WaitCursor);

    struct Result {
        std::optional<document::ImportedModel> model;
        QString error;
    };
    auto result = std::make_shared<Result>();
    const std::filesystem::path file = toPath(path);
    runInBackground(
        this,
        [result, file, cancel] {
            try {
                NullProgress progress;
                result->model = document::importModelFile(file, *cancel, progress);
            } catch (const std::exception& e) {
                result->error = QString::fromUtf8(e.what());
            }
        },
        [this, result, cancel, path, fixtureFor] {
            QApplication::restoreOverrideCursor();
            if (loadCancel_ != cancel) {
                return; // superseded
            }
            if (!result->model) {
                statusBar()->showMessage(tr("Import failed"));
                QMessageBox::warning(this, tr("Import failed"),
                                     tr("Could not import %1:\n%2").arg(path, result->error));
                emit loadFinished(false);
                return;
            }
            if (fixtureFor && doc_->project().findSetup(*fixtureFor)) {
                doc_->addImportedFixture(std::move(*result->model), *fixtureFor);
                refresh();
                statusBar()->showMessage(
                    tr("Imported fixture %1 into %2")
                        .arg(QFileInfo(path).fileName(),
                             QString::fromStdString(doc_->project().findSetup(*fixtureFor)->name)));
                emit loadFinished(true);
                return;
            }
            const Uuid modelId = doc_->addImportedModel(std::move(*result->model));
            if (doc_->project().setups.empty()) {
                doc_->addSetup(modelId);
            }
            refresh();
            view_->fitAll();
            statusBar()->showMessage(tr("Imported %1").arg(QFileInfo(path).fileName()));
            emit loadFinished(true);
        });
}

void MainWindow::openProject(const QString& path) {
    if (loadCancel_) {
        loadCancel_->requestCancel();
    }
    auto cancel = std::make_shared<CancelToken>();
    loadCancel_ = cancel;
    statusBar()->showMessage(tr("Opening %1...").arg(QFileInfo(path).fileName()));
    QApplication::setOverrideCursor(Qt::WaitCursor);

    struct Result {
        std::unique_ptr<document::Document> doc;
        QString error;
    };
    auto result = std::make_shared<Result>();
    const std::filesystem::path file = toPath(path);
    runInBackground(
        this,
        [result, file, cancel] {
            try {
                NullProgress progress;
                result->doc = std::make_unique<document::Document>(
                    document::Document::open(file, *cancel, progress));
            } catch (const std::exception& e) {
                result->error = QString::fromUtf8(e.what());
            }
        },
        [this, result, cancel, path] {
            QApplication::restoreOverrideCursor();
            if (loadCancel_ != cancel) {
                return;
            }
            if (!result->doc) {
                statusBar()->showMessage(tr("Open failed"));
                QMessageBox::warning(this, tr("Open failed"),
                                     tr("Could not open %1:\n%2").arg(path, result->error));
                emit loadFinished(false);
                return;
            }
            setDocument(std::move(result->doc));
            view_->fitAll();
            RecentProjects::add(path);
            statusBar()->showMessage(tr("Opened %1").arg(QFileInfo(path).fileName()));
            emit loadFinished(true);
        });
}

bool MainWindow::save() {
    if (!doc_->path()) {
        return saveAs();
    }
    try {
        doc_->save(*doc_->path());
        RecentProjects::add(fromPath(*doc_->path()));
        statusBar()->showMessage(tr("Saved"));
        return true;
    } catch (const Error& e) {
        QMessageBox::warning(this, tr("Save failed"), QString::fromUtf8(e.what()));
        return false;
    }
}

bool MainWindow::saveAs() {
    QString path = QFileDialog::getSaveFileName(this, tr("Save project"),
                                                QString::fromStdString(doc_->project().name) +
                                                    QStringLiteral(".ocam"),
                                                tr("OxCAM projects (*.ocam)"));
    if (path.isEmpty()) {
        return false;
    }
    if (!path.endsWith(QStringLiteral(".ocam"), Qt::CaseInsensitive)) {
        path += QStringLiteral(".ocam");
    }
    try {
        doc_->save(toPath(path));
        RecentProjects::add(path);
        statusBar()->showMessage(tr("Saved %1").arg(QFileInfo(path).fileName()));
        return true;
    } catch (const Error& e) {
        QMessageBox::warning(this, tr("Save failed"), QString::fromUtf8(e.what()));
        return false;
    }
}

bool MainWindow::maybeSave() {
    if (!doc_ || !doc_->isModified()) {
        return true;
    }
    const auto answer = QMessageBox::question(
        this, tr("Unsaved changes"),
        tr("Save changes to %1?").arg(QString::fromStdString(doc_->project().name)),
        QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
    if (answer == QMessageBox::Save) {
        return save();
    }
    return answer == QMessageBox::Discard;
}

void MainWindow::simulateCurrentSetup() {
    const auto setupId = currentSetupId();
    if (!setupId) {
        statusBar()->showMessage(tr("Add a setup to simulate."));
        return;
    }
    simDock_->show();
    simDock_->raise();
    simPanel_->simulate(*setupId);
}

bool MainWindow::simulationBusy() const noexcept {
    return simPanel_->busy();
}

void MainWindow::closeEvent(QCloseEvent* event) {
    if (maybeSave()) {
        event->accept();
    } else {
        event->ignore();
    }
}

void MainWindow::addSetup() {
    const document::Project& p = doc_->project();
    std::optional<Uuid> modelId;
    const Selection s = tree_->selection();
    if (s.kind == Selection::Kind::Model) {
        modelId = s.id;
    } else if (!p.models.empty()) {
        modelId = p.models.front().id;
    }
    const Uuid id = doc_->addSetup(modelId);
    tree_->rebuild(doc_->project());
    tree_->select({Selection::Kind::Setup, id});
}

void MainWindow::addOperation(const QString& type, const QString& name) {
    std::optional<Uuid> setupId = currentSetupId();
    if (!setupId) {
        addSetup();
        setupId = currentSetupId();
        if (!setupId) {
            return;
        }
    }
    document::Operation op;
    op.type = type.toStdString();
    op.name = name.toStdString();
    const auto& tools = doc_->project().tools.tools;
    if (!tools.empty() && op.type != "spindle_transfer") { // a transfer has no tool (ADR-0007)
        using tooling::ToolType;
        // Probing operations take probes, and nothing else does (DL-0090).
        const bool probing = op.type.rfind("probe_", 0) == 0;
        auto suits = [&](ToolType t) {
            if (probing || t == ToolType::Probe) {
                return probing && t == ToolType::Probe;
            }
            if (op.type == "chamfer2d") {
                return t == ToolType::Chamfer || t == ToolType::Engraver ||
                       t == ToolType::SpotDrill;
            }
            if (op.type == "trace2d") {
                return t == ToolType::Engraver || t == ToolType::Chamfer ||
                       t == ToolType::SpotDrill || t == ToolType::BallEndMill ||
                       t == ToolType::FlatEndMill;
            }
            if (op.type == "edgebreak3d") {
                return t == ToolType::Chamfer || t == ToolType::BallEndMill;
            }
            if (op.type == "backbore") {
                return t == ToolType::BoringBar || t == ToolType::Lollipop ||
                       t == ToolType::SlotCutter;
            }
            if (op.type == "tap") {
                return t == ToolType::Tap;
            }
            if (op.type == "threadmill") {
                return t == ToolType::ThreadMill;
            }
            if (op.type == "drill") {
                return t == ToolType::Drill || t == ToolType::SpotDrill ||
                       t == ToolType::CenterDrill;
            }
            if (op.type == "turn_face" || op.type == "turn_rough" || op.type == "turn_finish") {
                return t == ToolType::TurningInsert;
            }
            if (op.type == "turn_groove") {
                return t == ToolType::GroovingInsert;
            }
            if (op.type == "turn_thread") {
                return t == ToolType::ThreadingInsert;
            }
            if (op.type == "turn_partoff") {
                return t == ToolType::PartingInsert;
            }
            if (op.type == "bar_pull") {
                return t == ToolType::BarPuller;
            }
            if (op.type == "parallel3d" || op.type == "waterline3d" || op.type == "contour3d" ||
                op.type == "steep_shallow3d" || op.type == "pencil3d") {
                return t == ToolType::BallEndMill || t == ToolType::BullNoseEndMill;
            }
            if (op.type == "flat3d") {
                return t == ToolType::FlatEndMill || t == ToolType::BullNoseEndMill;
            }
            if (op.type == "multiaxis_finish") {
                return t == ToolType::BallEndMill;
            }
            if (op.type == "swarf5x") {
                return t == ToolType::FlatEndMill || t == ToolType::BullNoseEndMill ||
                       t == ToolType::BallEndMill;
            }
            if (op.type == "bore") {
                return t == ToolType::BoringBar || t == ToolType::Reamer ||
                       t == ToolType::FlatEndMill;
            }
            return t == ToolType::FlatEndMill || t == ToolType::BullNoseEndMill;
        };
        // The first tool that suits the type, else the first of the right kind (turning or
        // milling), else none: a turning operation never starts with a mill, nor the reverse.
        const bool turning = op.type.rfind("turn_", 0) == 0;
        auto it = std::find_if(tools.begin(), tools.end(),
                               [&](const tooling::Tool& t) { return suits(t.type); });
        if (it == tools.end() && op.type != "bar_pull") { // only a puller pulls a bar
            it = std::find_if(tools.begin(), tools.end(), [&](const tooling::Tool& t) {
                return tooling::isTurningTool(t.type) == turning &&
                       (t.type == ToolType::Probe) == probing;
            });
        }
        if (it != tools.end()) {
            op.toolId = it->id;
            if (!it->presets.empty()) {
                op.presetId = it->presets.front().id; // until another is chosen
            }
        }
    }
    // The user's own starting values for this type (Edit > Preferences > Operation Defaults).
    const Json defaults = operationDefaults(op.type);
    for (const auto& [key, value] : defaults.items()) {
        op.parameters[key] = value;
    }
    // Its heights as they are measured, not "default" (0.2" above the stock, and so on).
    op.heights = document::defaultHeights(op);
    const Uuid opId = op.id;
    doc_->modify("Add " + op.name, [&](document::Project& p) {
        if (document::Setup* s = p.findSetup(*setupId)) {
            s->operations.push_back(op);
        }
    });
    tree_->rebuild(doc_->project());
    tree_->select({Selection::Kind::Operation, opId});
}

void MainWindow::deleteSelected() {
    const Selection s = tree_->selection();
    const document::Project& p = doc_->project();
    switch (s.kind) {
    case Selection::Kind::Operation:
        doc_->modify("Delete operation", [&](document::Project& pr) {
            for (auto& setup : pr.setups) {
                std::erase_if(setup.operations,
                              [&](const document::Operation& o) { return o.id == s.id; });
            }
        });
        break;
    case Selection::Kind::Setup:
        doc_->modify("Delete setup", [&](document::Project& pr) {
            std::erase_if(pr.setups, [&](const document::Setup& st) { return st.id == s.id; });
        });
        break;
    case Selection::Kind::Tool: {
        const bool used =
            std::any_of(p.setups.begin(), p.setups.end(), [&](const document::Setup& st) {
                return std::any_of(st.operations.begin(), st.operations.end(),
                                   [&](const document::Operation& o) { return o.toolId == s.id; });
            });
        if (used && QMessageBox::question(this, tr("Delete tool"),
                                          tr("Operations use this tool. Delete it anyway?")) !=
                        QMessageBox::Yes) {
            return;
        }
        doc_->modify("Delete tool", [&](document::Project& pr) {
            std::erase_if(pr.tools.tools, [&](const tooling::Tool& t) { return t.id == s.id; });
        });
        break;
    }
    case Selection::Kind::Model:
        if (QMessageBox::question(this, tr("Remove model"),
                                  tr("Remove this model from the project? Setups and operations "
                                     "that use it lose it too.")) == QMessageBox::Yes) {
            doc_->removeModel(s.id);
        }
        break;
    default:
        break;
    }
}

void MainWindow::postProcessSetup() {
    const auto setupId = currentSetupId();
    if (!setupId) {
        QMessageBox::information(this, tr("Post process"),
                                 tr("Add a setup with operations first."));
        return;
    }
    // Choose the post and set its options first; they are stored with the setup (one undo step)
    // so the project posts the same anywhere.
    if (!configurePost(*setupId, true)) {
        return;
    }
    const document::Setup* setup = doc_->project().findSetup(*setupId);
    const QString name = QString::fromStdString(doc_->project().name + "_" + setup->name)
                             .replace(QLatin1Char(' '), QLatin1Char('_'));
    statusBar()->showMessage(tr("Post processing %1...").arg(QString::fromStdString(setup->name)));

    struct Result {
        document::PostOutput output;
        QString error;
        bool cancelled = false;
    };
    auto result = std::make_shared<Result>();
    auto snapshot = std::make_shared<const document::Document>(doc_->snapshot());
    const Uuid id = *setupId;
    const machine::Machine machine = document::machineFor(doc_->project(), *setup);
    // Operations whose toolpaths are not generated yet are generated first: that can take a
    // while, so the progress shows and the post can be cancelled.
    auto cancel = std::make_shared<CancelToken>();
    auto fraction = std::make_shared<std::atomic<double>>(0.0);
    auto* progressDialog =
        new QProgressDialog(tr("Post processing %1...").arg(QString::fromStdString(setup->name)),
                            tr("Cancel"), 0, 1000, this);
    progressDialog->setWindowTitle(tr("Post process"));
    progressDialog->setWindowModality(Qt::WindowModal);
    progressDialog->setMinimumDuration(300);
    progressDialog->setAutoClose(false);
    progressDialog->setAutoReset(false);
    progressDialog->setValue(0); // shows after the minimum duration unless finished by then
    auto* poll = new QTimer(progressDialog);
    connect(poll, &QTimer::timeout, progressDialog, [progressDialog, fraction] {
        progressDialog->setValue(static_cast<int>(1000.0 * fraction->load()));
    });
    poll->start(100);
    // Cancel hides the dialog; it stays hidden (no more progress) until the post stops.
    connect(progressDialog, &QProgressDialog::canceled, this, [cancel, poll] {
        cancel->requestCancel();
        poll->stop();
    });
    runInBackground(
        this,
        [result, snapshot, id, machine, cancel, fraction] {
            try {
                AtomicProgress progress(fraction);
                // The setup's post and option values.
                result->output = document::postSetup(*snapshot, id, machine, *cancel, progress);
            } catch (const OperationCancelled&) {
                result->cancelled = true;
            } catch (const std::exception& e) {
                result->error = QString::fromUtf8(e.what());
            }
        },
        [this, result, name, progressDialog] {
            // Closed now, with its timer: deleteLater would wait for the main event loop, and the
            // timer would show it again during the program dialog's own loop.
            delete progressDialog;
            if (result->cancelled) {
                statusBar()->showMessage(tr("Post processing cancelled"));
                return;
            }
            const document::PostOutput& out = result->output;
            if (!result->error.isEmpty() || !out.ok()) {
                QStringList errors;
                for (const auto& e : out.errors) {
                    errors << QString::fromStdString(e);
                }
                if (!result->error.isEmpty()) {
                    errors << result->error;
                }
                statusBar()->showMessage(tr("Post processing failed"));
                QMessageBox::warning(
                    this, tr("Post process"),
                    tr("The program was not posted:\n\n%1").arg(errors.join(QLatin1Char('\n'))));
                return;
            }
            QStringList warnings;
            for (const auto& w : out.warnings) {
                warnings << QString::fromStdString(w);
            }
            statusBar()->showMessage(tr("Posted %1").arg(name));
            PostDialog dialog(QString::fromStdString(out.gcode), warnings, name,
                              QString::fromStdString(out.extension), this);
            dialog.exec();
        });
}

bool MainWindow::configurePost(const Uuid& setupId, bool forPosting) {
    const document::Setup* setup = doc_->project().findSetup(setupId);
    if (!setup) {
        return false;
    }
    if (const std::string problem = document::machineProblem(doc_->project(), *setup);
        !problem.empty()) {
        QMessageBox::warning(this, tr("Post process"), QString::fromStdString(problem));
        return false;
    }
    PostProcessDialog settings(
        doc_->project(), setupId,
        forPosting ? PostProcessDialog::Mode::Post : PostProcessDialog::Mode::Configure, this);
    if (settings.exec() != QDialog::Accepted) {
        return false;
    }
    document::Project changed = doc_->project();
    if (settings.apply(changed)) {
        const std::string setupName = doc_->project().findSetup(setupId)->name;
        doc_->modify("Post settings of " + setupName,
                     [&](document::Project& p) { settings.apply(p); });
    }
    return true;
}

void MainWindow::selectFirstSetup() {
    if (doc_ && !doc_->project().setups.empty()) {
        tree_->select({Selection::Kind::Setup, doc_->project().setups.front().id});
    }
}

void MainWindow::selectLastSetup() {
    if (doc_ && !doc_->project().setups.empty()) {
        tree_->select({Selection::Kind::Setup, doc_->project().setups.back().id});
    }
}

void MainWindow::selectOperation(int index) {
    if (doc_ && !doc_->project().setups.empty()) {
        const auto& ops = doc_->project().setups.front().operations;
        if (!ops.empty()) {
            const auto& op = ops[static_cast<std::size_t>(
                std::clamp(index, 0, static_cast<int>(ops.size()) - 1))];
            tree_->select({Selection::Kind::Operation, op.id});
        }
    }
}

void MainWindow::updateStockPreview() {
    const Selection s = tree_->selection();
    const document::Setup* owner = nullptr;
    const bool on = previewAction_ && previewAction_->isChecked() && doc_ &&
                    s.kind == Selection::Kind::Operation &&
                    doc_->project().findOperation(s.id, &owner) && owner;
    if (!on) {
        simPanel_->stopPreview();
        return;
    }
    simPanel_->previewOperation(owner->id, s.id);
    if (simPanel_->busy()) {
        statusBar()->showMessage(
            tr("Simulating the stock up to %1…")
                .arg(QString::fromStdString(doc_->project().findOperation(s.id)->name)));
    }
}

void MainWindow::showToolLibraries() {
    ToolLibrariesDialog dialog([this](const std::filesystem::path& file) { editToolLibrary(file); },
                               this);
    dialog.exec();
    panel_->showSelection(tree_->selection()); // library tools in the tool pickers
}

void MainWindow::editToolLibrary(const std::filesystem::path& file) {
    tooling::ToolLibrary lib;
    lib.name = tr("My tools").toStdString();
    try {
        if (std::filesystem::exists(file)) {
            lib = tooling::loadToolLibrary(file);
        }
    } catch (const Error& e) {
        QMessageBox::warning(this, tr("Tool library"), QString::fromUtf8(e.what()));
        return;
    }
    ToolLibraryDialog dialog(lib, ToolLibraryDialog::Mode::LibraryFile, this);
    dialog.setFile(file);
    dialog.setAddToProjectHandler([this](const std::vector<tooling::Tool>& tools,
                                         const std::vector<tooling::Holder>& holders) {
        doc_->modify("Add tools to project", [&](document::Project& p) {
            for (const auto& h : holders) {
                if (!p.tools.findHolder(h.id)) {
                    p.tools.holders.push_back(h);
                }
            }
            for (const auto& t : tools) {
                if (!p.tools.findTool(t.id)) {
                    p.tools.tools.push_back(t);
                }
            }
        });
        statusBar()->showMessage(
            tr("Added %n tool(s) to the project", nullptr, static_cast<int>(tools.size())));
    });
    dialog.exec();
    if (dialog.file() && *dialog.file() != file) {
        ToolLibraries::add(*dialog.file()); // saved under a new name
    }
    panel_->showSelection(tree_->selection());
}

void MainWindow::setSimulationMode(bool on) {
    // While simulating the program cannot change: what is played back stays what was simulated.
    simulationMode_ = on;
    panel_->setEnabled(!on);
    turretWindow_->panel()->setEnabled(!on);
    tree_->setReorderEnabled(!on);
    for (QAction* a : editActions_) {
        a->setEnabled(!on);
    }
    for (QWidget* w : editWidgets_) {
        w->setEnabled(!on);
    }
    undo_->setEnabled(!on && doc_ && doc_->canUndo());
    redo_->setEnabled(!on && doc_ && doc_->canRedo());
    simulationBanner_->setVisible(on);
}

void MainWindow::showContextMenu(const Selection& selection, const QString& group,
                                 const QPoint& pos) {
    if (simulationMode_) {
        statusBar()->showMessage(tr("Close the simulation to edit the program."), 4000);
        return;
    }
    const document::Project& p = doc_->project();
    QMenu menu(this);
    if (group == QStringLiteral("setups")) {
        menu.addAction(tr("Add Setup"), this, &MainWindow::addSetup);
    } else if (group == QStringLiteral("models")) {
        menu.addAction(tr("Import STEP"), this, &MainWindow::importStepDialog);
        menu.addAction(tr("Import Fixture"), this, [this] { importFixtureDialog(); });
    } else if (group == QStringLiteral("tools")) {
        menu.addAction(tr("Project Tools"), this, &MainWindow::editProjectTools);
        menu.addAction(tr("Tool Libraries"), this, &MainWindow::showToolLibraries);
    } else {
        switch (selection.kind) {
        case Selection::Kind::Project:
            menu.addAction(tr("Add Setup"), this, &MainWindow::addSetup);
            menu.addAction(tr("Rename"), this, &MainWindow::renameSelected);
            break;
        case Selection::Kind::Model:
            menu.addAction(tr("Add Setup for This Model"), this, &MainWindow::addSetup);
            menu.addAction(tr("Rename"), this, &MainWindow::renameSelected);
            menu.addSeparator();
            menu.addAction(tr("Remove"), this, &MainWindow::deleteSelected);
            break;
        case Selection::Kind::Tool:
            menu.addAction(tr("Edit Project Tools"), this, &MainWindow::editProjectTools);
            menu.addSeparator();
            menu.addAction(tr("Delete"), this, &MainWindow::deleteSelected);
            break;
        case Selection::Kind::Setup: {
            const Uuid id = selection.id;
            populateOperationMenu(menu.addMenu(tr("Add Operation")));
            menu.addAction(tr("Import Fixture"), this, [this, id] { importFixtureDialog(id); });
            menu.addSeparator();
            // The commands themselves, so the menu shows the user's shortcuts.
            menu.addAction(simulateAction_);
            menu.addAction(postAction_);
            menu.addAction(tr("Regenerate Toolpaths"), this,
                           [this, id] { regenerateToolpaths(id); });
            menu.addSeparator();
            menu.addAction(tr("Machine"), this, &MainWindow::showMachineLibrary);
            menu.addAction(tr("Workholding"), this, &MainWindow::showWorkholdingLibrary);
            menu.addAction(renameAction_);
            menu.addAction(tr("Duplicate"), this, [this, id] { duplicateSetup(id); });
            menu.addSeparator();
            menu.addAction(deleteAction_);
            break;
        }
        case Selection::Kind::Operation: {
            const Uuid id = selection.id;
            const document::Setup* owner = nullptr;
            const document::Operation* op = p.findOperation(id, &owner);
            if (!op) {
                return;
            }
            QAction* enabled = menu.addAction(tr("Enabled"));
            enabled->setCheckable(true);
            enabled->setChecked(op->enabled);
            enabled->setObjectName(QStringLiteral("operationEnabled"));
            connect(enabled, &QAction::toggled, this, [this, id](bool on) {
                doc_->modify(on ? "Enable operation" : "Disable operation",
                             [&](document::Project& pr) {
                                 if (document::Operation* o = pr.findOperation(id)) {
                                     o->enabled = on;
                                 }
                             });
            });
            const Uuid setupId = owner->id;
            menu.addAction(tr("Regenerate Toolpath"), this, [this, id, setupId] {
                toolpaths_.erase(id);
                regenerateToolpaths(setupId);
            });
            menu.addAction(tr("Show Toolpath Data"), this, [this, id] { showToolpathData(id); })
                ->setObjectName(QStringLiteral("showToolpathData"));
            menu.addSeparator();
            menu.addAction(renameAction_);
            menu.addAction(tr("Duplicate"), this, [this, id] { duplicateOperation(id); });
            menu.addSeparator();
            menu.addAction(simulateAction_);
            menu.addSeparator();
            menu.addAction(deleteAction_);
            break;
        }
        case Selection::Kind::None:
            return;
        }
    }
    menu.exec(pos);
}

void MainWindow::renameSelected() {
    const Selection s = tree_->selection();
    const document::Project& p = doc_->project();
    QString current;
    switch (s.kind) {
    case Selection::Kind::Project:
        current = QString::fromStdString(p.name);
        break;
    case Selection::Kind::Model:
        if (const auto* m = p.findModel(s.id)) {
            current = QString::fromStdString(m->name);
        }
        break;
    case Selection::Kind::Setup:
        if (const auto* st = p.findSetup(s.id)) {
            current = QString::fromStdString(st->name);
        }
        break;
    case Selection::Kind::Operation:
        if (const auto* op = p.findOperation(s.id)) {
            current = QString::fromStdString(op->name);
        }
        break;
    default:
        return;
    }
    bool ok = false;
    const QString name =
        QInputDialog::getText(this, tr("Rename"), tr("Name:"), QLineEdit::Normal, current, &ok)
            .trimmed();
    if (!ok || name.isEmpty() || name == current) {
        return;
    }
    const std::string n = name.toStdString();
    doc_->modify("Rename", [&](document::Project& pr) {
        switch (s.kind) {
        case Selection::Kind::Project:
            pr.name = n;
            break;
        case Selection::Kind::Model:
            for (auto& m : pr.models) {
                if (m.id == s.id) {
                    m.name = n;
                }
            }
            break;
        case Selection::Kind::Setup:
            if (auto* st = pr.findSetup(s.id)) {
                st->name = n;
            }
            break;
        case Selection::Kind::Operation:
            if (auto* op = pr.findOperation(s.id)) {
                op->name = n;
            }
            break;
        default:
            break;
        }
    });
}

void MainWindow::duplicateSetup(const Uuid& setupId) {
    const document::Setup* source = doc_->project().findSetup(setupId);
    if (!source) {
        return;
    }
    document::Setup copy = *source;
    copy.id = Uuid::generate();
    copy.name = source->name + " (copy)";
    for (auto& op : copy.operations) {
        op.id = Uuid::generate();
    }
    const Uuid id = copy.id;
    doc_->modify("Duplicate setup", [&](document::Project& pr) {
        auto it = std::find_if(pr.setups.begin(), pr.setups.end(),
                               [&](const document::Setup& s) { return s.id == setupId; });
        pr.setups.insert(it == pr.setups.end() ? it : std::next(it), copy);
    });
    tree_->rebuild(doc_->project());
    tree_->select({Selection::Kind::Setup, id});
}

void MainWindow::duplicateOperation(const Uuid& operationId) {
    document::Setup* owner = nullptr;
    document::Project probe = doc_->project();
    const document::Operation* source = probe.findOperation(operationId, &owner);
    if (!source || !owner) {
        return;
    }
    document::Operation copy = *source;
    copy.id = Uuid::generate();
    copy.name = source->name + " (copy)";
    const Uuid setupId = owner->id;
    const Uuid id = copy.id;
    doc_->modify("Duplicate operation", [&](document::Project& pr) {
        if (document::Setup* s = pr.findSetup(setupId)) {
            auto it =
                std::find_if(s->operations.begin(), s->operations.end(),
                             [&](const document::Operation& o) { return o.id == operationId; });
            s->operations.insert(it == s->operations.end() ? it : std::next(it), copy);
        }
    });
    tree_->rebuild(doc_->project());
    tree_->select({Selection::Kind::Operation, id});
}

void MainWindow::regenerateToolpaths(const Uuid& setupId) {
    if (const document::Setup* s = doc_->project().findSetup(setupId)) {
        for (const auto& op : s->operations) {
            toolpaths_.erase(op.id); // the cache key no longer matches: regenerate
        }
    }
    updateToolpaths();
    refreshToolpathDisplay();
    tree_->rebuild(doc_->project());
}

void MainWindow::updateRecentMenu() {
    recentMenu_->clear();
    const QStringList recent = RecentProjects::list();
    for (const QString& path : recent) {
        QAction* a = recentMenu_->addAction(QFileInfo(path).fileName(), this, [this, path] {
            if (maybeSave()) {
                openProject(path);
            }
        });
        a->setToolTip(path);
        a->setEnabled(QFileInfo::exists(path));
    }
    if (recent.isEmpty()) {
        recentMenu_->addAction(tr("(none)"))->setEnabled(false);
    } else {
        recentMenu_->addSeparator();
        recentMenu_->addAction(tr("Clear List"), this,
                               [] { QSettings().remove(QStringLiteral("recentProjects")); });
    }
}

void MainWindow::showPreferences(int page) {
    PreferencesDialog dialog(commands_, this);
    dialog.showPage(static_cast<PreferencesDialog::Page>(page));
    // Opens on the type of the selected operation.
    const Selection s = tree_->selection();
    if (const document::Operation* op =
            s.kind == Selection::Kind::Operation ? doc_->project().findOperation(s.id) : nullptr) {
        dialog.showOperationType(op->type);
    }
    if (dialog.exec() == QDialog::Accepted) {
        applyColors();
        commands_.applyShortcuts();
        updatePinnedSection();
        panel_->showSelection(tree_->selection());
    }
}

void MainWindow::applyColors() {
    const ViewColors colors = ViewColors::load();
    view_->setColors(colors);
    simPanel_->setStockColour(colors.stock);
}

void MainWindow::showStartHub() {
    StartHub hub(this);
    hub.exec();
    switch (hub.action()) {
    case StartHub::Action::NewProject:
        newProject();
        break;
    case StartHub::Action::OpenProject:
        openProjectDialog();
        break;
    case StartHub::Action::ImportStep:
        if (maybeSave()) {
            setDocument(newDocument());
            importStepDialog();
        }
        break;
    case StartHub::Action::OpenRecent:
        if (maybeSave()) {
            openProject(hub.path());
        }
        break;
    case StartHub::Action::ToolLibraries:
        showToolLibraries();
        break;
    case StartHub::Action::MachineLibrary:
        showMachineLibrary();
        break;
    case StartHub::Action::None:
        break;
    }
}

void MainWindow::showMachineLibrary() {
    MachineLibraryDialog dialog(this);
    const auto setupId = currentSetupId();
    const document::Setup* setup = setupId ? doc_->project().findSetup(*setupId) : nullptr;
    if (setup) {
        const Uuid id = setup->id;
        dialog.setUseHandler(QString::fromStdString(setup->name), [this,
                                                                   id](const machine::Machine& m) {
            doc_->modify("Use machine " + m.name, [&](document::Project& p) {
                if (!machine::builtinMachine(m.id) || p.findMachine(m.id)) {
                    auto it = std::find_if(p.machines.begin(), p.machines.end(),
                                           [&](const machine::Machine& x) { return x.id == m.id; });
                    if (it == p.machines.end()) {
                        p.machines.push_back(m);
                    } else {
                        *it = m;
                    }
                }
                if (document::Setup* s = p.findSetup(id)) {
                    s->machine = m.id;
                }
            });
        });
        if (const machine::Machine* m = document::findMachineFor(doc_->project(), *setup)) {
            dialog.selectMachine(m->id);
        }
    }
    dialog.exec();
    panel_->showSelection(tree_->selection()); // the machine list may have changed
}

SetupPlacementDialog* MainWindow::placeOnMachine(std::optional<Uuid> setupId, bool modal) {
    if (!setupId) {
        setupId = currentSetupId();
    }
    const document::Setup* setup = setupId ? doc_->project().findSetup(*setupId) : nullptr;
    if (!setup) {
        return nullptr;
    }
    if (const std::string problem = document::machineProblem(doc_->project(), *setup);
        !problem.empty()) {
        QMessageBox::warning(this, tr("Place on machine"), QString::fromStdString(problem));
        return nullptr;
    }
    auto* dialog = new SetupPlacementDialog(*doc_, setup->id, doc_->project().displayUnit, this);
    dialog->setAttribute(Qt::WA_DeleteOnClose);
    const Uuid id = setup->id;
    connect(dialog, &QDialog::accepted, this, [this, dialog, id] {
        const auto placement = dialog->placement();
        doc_->modify("Place setup on machine", [&](document::Project& p) {
            if (document::Setup* s = p.findSetup(id)) {
                s->placement = placement;
            }
        });
    });
    if (modal) {
        dialog->exec();
        return nullptr;
    }
    dialog->show();
    return dialog;
}

WorkholdingSetupDialog* MainWindow::setUpWorkholding(std::optional<Uuid> setupId, bool modal) {
    if (!setupId) {
        setupId = currentSetupId();
    }
    const document::Setup* setup = setupId ? doc_->project().findSetup(*setupId) : nullptr;
    if (!setup) {
        return nullptr;
    }
    auto* dialog = new WorkholdingSetupDialog(*doc_, setup->id, doc_->project().displayUnit, this);
    dialog->setAttribute(Qt::WA_DeleteOnClose);
    connect(dialog, &QDialog::accepted, this, [this, dialog] {
        try {
            doc_->takeWorkholding(dialog->edited(), dialog->setupId());
        } catch (const Error& e) {
            QMessageBox::warning(this, tr("Workholding"), QString::fromUtf8(e.what()));
        }
    });
    if (modal) {
        dialog->exec();
        return nullptr;
    }
    dialog->show();
    return dialog;
}

void MainWindow::showFittingsLibrary() {
    LatheFittingsDialog dialog(this);
    dialog.exec();
    // The setup page lists what the library has.
    refresh();
}

void MainWindow::showWorkholdingLibrary() {
    WorkholdingLibraryDialog dialog(this);
    const auto setupId = currentSetupId();
    const document::Setup* setup = setupId ? doc_->project().findSetup(*setupId) : nullptr;
    if (setup) {
        const Uuid id = setup->id;
        dialog.setUseHandler(
            QString::fromStdString(setup->name),
            [this, id](const document::WorkholdingPackage& pkg) {
                QApplication::setOverrideCursor(Qt::WaitCursor);
                try {
                    CancelToken cancel;
                    NullProgress progress;
                    doc_->useWorkholding(
                        pkg, document::importWorkholdingBodies(pkg, cancel, progress), id);
                    QApplication::restoreOverrideCursor();
                } catch (const Error& e) {
                    QApplication::restoreOverrideCursor();
                    QMessageBox::warning(this, tr("Workholding"), QString::fromUtf8(e.what()));
                }
            });
        if (setup->workholding) {
            dialog.selectTemplate(setup->workholding->templateId);
        }
    }
    dialog.exec();
    panel_->showSelection(tree_->selection()); // the library may have changed
}

void MainWindow::editProjectTools() {
    ToolLibraryDialog dialog(doc_->project().tools, ToolLibraryDialog::Mode::ProjectTools, this);
    if (dialog.exec() == QDialog::Accepted) {
        const tooling::ToolLibrary edited = dialog.library();
        doc_->modify("Edit project tools", [&](document::Project& p) { p.tools = edited; });
    }
}

} // namespace ocam::gui
