// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// oxcam-gui [file.ocam | file.step]
//
// Test hooks (used by CI and for screenshots in reviews):
//   --smoke-test <file>             open the file, render, and exit (0 on success)
//   --screenshot <png>              with --smoke-test: save the 3D view
//   --screenshot-window <png>       with --smoke-test: save the whole window (docks included)
//   --screenshot-tools <png> <lib>  save the tool library dialog showing <lib>, then exit
//   --screenshot-sim <png>          with --smoke-test: simulate the first setup, save the 3D view
//   --sim-last-setup                with --screenshot-sim: simulate the last setup instead
//   --sim-position <op>:<move>      with --screenshot-sim: show this playback position (0-based op)
//   --sim-play <seconds>            with --screenshot-sim: play from there for this long, pause,
//                                   print where playback got to (exit 8 when it did not move)
//   --machine-part <name>=<0|1>     with --screenshot-sim: hide or show a machine part (repeat)
//   --screenshot-closed-sim <png>   with --screenshot-sim: close the simulation, save the 3D view
//   --new-tool <type>               with --screenshot-tools: add a tool of this type first
//   --show-holders                  with --screenshot-tools: show the holders tab
//   --new-turning-holder <shank|boring|plain>  with --screenshot-tools: add a turning holder first
//   --press-enter                   with --screenshot-tools: press Enter in the tool's name,
//                                   exit 5 when that changes the tool list
//   --select-tool <n>               with --screenshot-tools: select tool number <n>
//   --tool-search <text>            with --screenshot-tools: filter the tool list by text
//   --tool-type <id>                with --screenshot-tools: filter by a family (milling,
//                                   hole_making, turning) or a tool type (e.g. drill)
//   --tool-diameter <min>:<max>     with --screenshot-tools: filter by diameter [mm], 0 = any
//   --check-holder-editor <dir>     drive the tool library's holder widgets, check the library,
//                                   save a screenshot of each step, then exit (HolderEditorCheck)
//   --screenshot-presets <png>      with --screenshot-tools: also save the cutting preset
//                                   editor of the first tool
//   --screenshot-holder-import <png> <step>  save the holder import dialog for <step>, then exit
//   --select-setup                  with --smoke-test: select the first setup before screenshots
//   --screenshot-placement <png>    with --smoke-test: place the first setup on its machine
//                                   (--place-work <x,y,z> clicks that WCS point of the work,
//                                   --place-on <x,y,z> that machine-zero point of the machine,
//                                   --place-snap face|axis|point), print, save the dialog, exit
//   --set-choice <key>=<value>      with --select-operation: choose it in that parameter's list
//   --select-operation <n>          with --smoke-test: select operation n of the first setup
//   --reselect-test                 with --smoke-test: show the in-process stock of operation 0
//                                   at a new resolution, pick operation 1 and 0 again while it
//                                   is simulated, fail if the simulation starts over
//   --pick-test                     with --smoke-test: click the view centre, fail without a face
//   --wheel-test                    with --select-setup: turn the wheel over a field of the setup
//                                   page, fail if it changes unfocused or not when focused
//   --wcs-pick-test                 with --select-setup: origin from a picked point, press Pick,
//                                   click the view centre from the top, fail unless the origin
//                                   is then on a face
//   --wcs-axis-test <x,y,z>         with --select-setup: X and Z from picks, press X's Pick,
//                                   click that point (an edge) from the top, print WCS X
//   --trace-pick-test <x,y,z/...>   with --smoke-test: add a 2D trace, pick its edges at those
//                                   project points in the top view; fail unless one open chain
//   --hover-test <png>              with --select-operation (a 2D contour): start picking its
//                                   geometry, hover an edge in the top view, save the 3D view
//   --screenshot-hub <png>          save the start hub, then exit
//   --screenshot-machines <png>     save the machine library dialog, then exit
//   --screenshot-workholding <png>  save the workholding library dialog, then exit
//   --workholding-folder <dir>      use <dir> as the workholding library for this run
//   --workholding-up-test <png>     a Y-up vise (--workholding-step): the wizard's up, the
//                                   editor's warning for up +X, up picked on the jaw tops
//   --fittings-folder <dir>         use <dir> as the turret block and centre library for this run
//   --machine <id>                  with --screenshot-machines: select this machine
//   --jog <axis>=<value>            with --screenshot-machines: put a preview axis there (repeat)
//   --show-model                    with --screenshot-machines: scroll to the machine model
//   --select-part <name>            with --screenshot-machines: select a part (highlighted)
//   --isolate                       with --select-part: show only it in the preview
//   --pick-axis <axis>@<x>,<y>,<z>   with --screenshot-machines: pick a rotary axis on the round
//                                   face at a machine-zero point (clicking where it shows), print
//   --pick-gauge <x,y,z>;<x,y,z>    with --screenshot-machines: pick the spindle gauge point on a
//                                   round face then the seat face (machine zero), print
//   --machine-stands <up>,<front>   with --screenshot-machines: choose how the machine stands
//                                   (+Y,+Z), print how the preview stands before and after
//   --maximized                     with --screenshot-machines: show the dialog maximized
//   --pick-preview                  with --screenshot-machines: click the preview's centre, print
//                                   the part selected; fail unless it is --select-part's
//   --screenshot-parts-import <png> <file>  save the machine parts import dialog for a STEP or
//                                   STL file (--merge-level <n>), then exit
//   --screenshot-preferences <png>  save the preferences dialog, then exit
//   --preferences-page <n>          with --screenshot-preferences: show page n (0-based)
//   --preferences-type <type>       with --screenshot-preferences: show this operation type's
//                                   defaults
//   --settings-dir <dir>            keep the settings in <dir> (INI), not the user's
//   --list-commands                 print the commands (id, shortcuts), then exit
//   --press-shortcut <key>          press a key three times, print the toolpath display, exit
//   --screenshot-tooltip <png>      save the tooltip of a command (--tooltip-command <id>), exit
//   --run-command <id>              with --smoke-test: run this command before screenshots
//   --toolpath-data-test <png>      with --select-operation: open Show Toolpath Data from the
//                                   operation's right-click menu, check it against the toolpath
//   --toolpath-data-expect <text>   with --toolpath-data-test: a move type that must be listed,
//                                   or "refused" for an operation generation refuses
//   --toolpath-data-long <png>      with --smoke-test: list a toolpath of 500000 moves
//   --write-library-strings <file> write the libraries' texts for lupdate, then exit
//
// Without a file (and without test hooks) the start hub opens.

#include "ComboLists.hpp"
#include "CuttingPresetDialog.hpp"
#include "HolderEditorCheck.hpp"
#include "HolderImportDialog.hpp"
#include "LatheFittingsDialog.hpp"
#include "MachineLibrary.hpp"
#include "MachineLibraryDialog.hpp"
#include "MachinePartsImportDialog.hpp"
#include "MainWindow.hpp"
#include "OcctView.hpp"
#include "OpenCamSettings.hpp"
#include "OperationDefaults.hpp"
#include "PostDialog.hpp"
#include "PostProcessDialog.hpp"
#include "PreferencesDialog.hpp"
#include "ProjectTree.hpp"
#include "PropertyPanel.hpp"
#include "SetupPlacementDialog.hpp"
#include "SimulationPanel.hpp"
#include "StartHub.hpp"
#include "ToolLibraryDialog.hpp"
#include "ToolTips.hpp"
#include "ToolpathDataDialog.hpp"
#include "Translations.hpp"
#include "TurretPreview.hpp"
#include "TurretWindow.hpp"
#include "Units.hpp"
#include "WheelGuard.hpp"
#include "WorkholdingLibrary.hpp"
#include "WorkholdingLibraryDialog.hpp"
#include "WorkholdingSetupDialog.hpp"
#include "WorkholdingWizard.hpp"

#include <ocam/cad/Assembly.hpp>
#include <ocam/cad/Datum.hpp>
#include <ocam/cad/Section.hpp>
#include <ocam/cad/SubShape.hpp>
#include <ocam/document/Generate.hpp>
#include <ocam/document/LatheFitting.hpp>
#include <ocam/document/LatheMachine.hpp>
#include <ocam/document/PostSetup.hpp>
#include <ocam/document/StepBodies.hpp>
#include <ocam/foundation/Cancel.hpp>
#include <ocam/foundation/Error.hpp>
#include <ocam/foundation/Progress.hpp>
#include <ocam/foundation/Version.hpp>

#include <QAbstractItemView>
#include <QAction>
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QCommandLineParser>
#include <QContextMenuEvent>
#include <QDir>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QFileInfo>
#include <QGroupBox>
#include <QImage>
#include <QKeyEvent>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMenu>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPainter>
#include <QPointer>
#include <QProgressDialog>
#include <QProxyStyle>
#include <QPushButton>
#include <QScrollArea>
#include <QScrollBar>
#include <QSettings>
#include <QSpinBox>
#include <QStandardPaths>
#include <QStyleOption>
#include <QTableView>
#include <QTemporaryDir>
#include <QThread>
#include <QTimer>
#include <QToolButton>
#include <QToolTip>
#include <QTreeWidget>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <map>
#include <memory>
#include <set>

namespace {

/// The test chuck taking jaw sets (DL-0069), and a hard and a soft jaw set made of one of its jaws:
/// placeholder shapes, as in the unit tests. The chuck is changed in place; the sets come back.
std::pair<ocam::document::WorkholdingTemplate, ocam::document::WorkholdingTemplate>
testJawSets(ocam::document::WorkholdingTemplate& chuck) {
    namespace doc = ocam::document;
    const auto typed = [](const char* name, const ocam::Vec3& at, const ocam::Vec3& dir,
                          const char* member = "") {
        doc::WorkholdingDatum d;
        d.name = name;
        d.kind = doc::DatumKind::Typed;
        d.member = member;
        d.point = at;
        d.direction = dir;
        return d;
    };
    chuck.datums.push_back(typed("Jaw seat", {50, 0, 100}, ocam::Vec3::UnitZ(), "Jaw 1"));
    chuck.jawMount = doc::JawMount{"Test jaws", "Jaw seat"};
    for (doc::WorkholdingPart& part : chuck.parts) {
        part.topJaw = !part.member.empty();
    }
    doc::WorkholdingPart jaw = chuck.parts[1];
    jaw.member.clear();
    jaw.topJaw = false;
    doc::WorkholdingTemplate hard;
    hard.name = "Step jaws (test)";
    hard.parts = {jaw};
    hard.datums = {typed("Seat", {47.5, 0, 60}, ocam::Vec3::UnitZ()),
                   typed("Grip", {25, 0, 80}, -ocam::Vec3::UnitX()),
                   typed("Stop", {30, 0, 70}, ocam::Vec3::UnitZ()),
                   typed("Front", {47.5, 0, 100}, ocam::Vec3::UnitZ())};
    // Two ways on, as a vise's holdings: the step grips an outside diameter one way and, the jaw
    // turned round on its seat, an inside one the other.
    hard.jawSet =
        doc::JawSet{"Test jaws",
                    doc::JawSet::Kind::Hard,
                    "Front",
                    {{"OD", "Seat", -ocam::Vec3::UnitX(), {{"Inner", "Grip", "Stop"}}},
                     {"Reversed (ID)", "Seat", ocam::Vec3::UnitX(), {{"Inner", "Grip", "Stop"}}}}};
    doc::WorkholdingTemplate soft = hard;
    soft.id = ocam::Uuid::generate();
    soft.name = "Soft jaws (test)";
    soft.jawSet->kind = doc::JawSet::Kind::Soft;
    soft.jawSet->options = {{"Normal", "Seat", -ocam::Vec3::UnitX(), {{"Clamp", "Grip", "Stop"}}}};
    return {hard, soft};
}

/// Right-clicks the current item of the project tree and chooses an item of the menu that opens,
/// as the user would. Blocks while the menu is open, as the menu does; false when it has no such
/// item.
bool chooseFromTreeMenu(ocam::gui::MainWindow& window, const QString& item) {
    auto* tree = window.findChild<ocam::gui::ProjectTree*>();
    QTreeWidgetItem* current = tree ? tree->currentItem() : nullptr;
    if (!current) {
        return false;
    }
    auto chosen = std::make_shared<bool>(false);
    QTimer::singleShot(0, &window, [chosen, item] {
        auto* menu = qobject_cast<QMenu*>(QApplication::activePopupWidget());
        if (!menu) {
            return;
        }
        for (QAction* action : menu->actions()) {
            if (action->objectName() == item) {
                *chosen = true;
                menu->setActiveAction(action);
                QKeyEvent enter(QEvent::KeyPress, Qt::Key_Return, Qt::NoModifier);
                QApplication::sendEvent(menu, &enter);
                return;
            }
        }
        menu->close();
    });
    const QPoint at = tree->visualItemRect(current).center();
    QContextMenuEvent click(QContextMenuEvent::Mouse, at, tree->viewport()->mapToGlobal(at));
    QApplication::sendEvent(tree->viewport(), &click);
    return *chosen;
}

/// The first entry under a list's "In your library" heading, or -1 when it has none.
int libraryEntry(const QComboBox* box) {
    bool under = false;
    for (int i = 0; i < box->count(); ++i) {
        if (ocam::gui::isListHeading(box, i)) {
            under = box->itemText(i) == QStringLiteral("In your library");
        } else if (box->itemText(i).isEmpty()) {
            if (under) {
                return -1; // a separator ends the group
            }
        } else if (under) {
            return i;
        }
    }
    return -1;
}

/// Whether an entry of the list has a tooltip holding `text`.
bool anyToolTipHolds(const QComboBox* box, const QString& text) {
    for (int i = 0; i < box->count(); ++i) {
        if (box->itemData(i, Qt::ToolTipRole).toString().contains(text)) {
            return true;
        }
    }
    return false;
}

/// Opens the list's pop-up and clicks the entry holding `text`, as a user would; false when the
/// list has no such entry or the click chose nothing. The list may be gone when it returns: a
/// choice that changes the project rebuilds the page it is on.
bool chooseInPopup(QComboBox* box, const QString& text) {
    const auto settle = [] {
        for (int i = 0; i < 5; ++i) {
            QApplication::processEvents();
        }
    };
    const int i = box ? box->findText(text, Qt::MatchContains) : -1;
    if (i < 0) {
        return false;
    }
    box->showPopup();
    settle();
    // A press this soon after the pop-up opened would be taken as the click that opened it.
    QThread::msleep(static_cast<unsigned long>(QApplication::doubleClickInterval()) + 50);
    settle();
    QAbstractItemView* view = box->view();
    const QModelIndex index = view->model()->index(i, box->modelColumn(), view->rootIndex());
    view->scrollTo(index);
    settle();
    const QPointF at = view->visualRect(index).center();
    const QPointF global = view->viewport()->mapToGlobal(at);
    QMouseEvent move(QEvent::MouseMove, at, global, Qt::NoButton, Qt::NoButton, Qt::NoModifier);
    QApplication::sendEvent(view->viewport(), &move);
    QMouseEvent press(QEvent::MouseButtonPress, at, global, Qt::LeftButton, Qt::LeftButton,
                      Qt::NoModifier);
    QApplication::sendEvent(view->viewport(), &press);
    QMouseEvent release(QEvent::MouseButtonRelease, at, global, Qt::LeftButton, Qt::NoButton,
                        Qt::NoModifier);
    const QPointer<QComboBox> still(box);
    QApplication::sendEvent(view->viewport(), &release);
    settle();
    if (still && still->view()->isVisible()) {
        still->hidePopup(); // the click missed: nothing was chosen
        settle();
        return false;
    }
    return true;
}

/// Lets the window work (a refresh, then its toolpath jobs) until no toolpath is being generated.
void settleToolpaths(const ocam::gui::MainWindow& window) {
    do {
        for (int i = 0; i < 5; ++i) {
            QApplication::processEvents();
            QThread::msleep(10);
        }
    } while (window.toolpathsBusy());
}

/// Every row of the list against the toolpath: in its order, with its numbers and its feeds.
QString checkToolpathRows(const ocam::gui::ToolpathDataModel& model,
                          const ocam::toolpath::Toolpath& path, const QString& expect) {
    using Model = ocam::gui::ToolpathDataModel;
    using ocam::toolpath::MoveKind;
    if (model.rowCount() != static_cast<int>(path.moves.size())) {
        return QStringLiteral("%1 rows for %2 moves").arg(model.rowCount()).arg(path.moves.size());
    }
    const ocam::LengthUnit unit = model.unit();
    const double shownTo = 0.51 * std::pow(10.0, -ocam::gui::lengthDecimals(unit));
    bool expected = false;
    for (int row = 0; row < model.rowCount(); ++row) {
        const ocam::toolpath::Move& m = path.moves[static_cast<std::size_t>(row)];
        auto wrong = [&](const char* what) {
            return QStringLiteral("move %1: %2").arg(row + 1).arg(QLatin1String(what));
        };
        auto same = [&](Model::Column column, double value) {
            return std::abs(model.text(row, column).toDouble() - value) <= shownTo;
        };
        if (model.text(row, Model::Number).toInt() != row + 1) {
            return wrong("number");
        }
        if (!same(Model::X, ocam::fromMm(m.end.x(), unit)) ||
            !same(Model::Y, ocam::fromMm(m.end.y(), unit)) ||
            !same(Model::Z, ocam::fromMm(m.end.z(), unit))) {
            return wrong("position");
        }
        const QString type = model.text(row, Model::Type);
        expected = expected || type.contains(expect);
        if (m.kind == MoveKind::Rapid || m.kind == MoveKind::Dwell) {
            if (!model.text(row, Model::Feed).isEmpty()) {
                return wrong("a feed on a move without one");
            }
        } else {
            const double feed = m.kind == MoveKind::Cycle
                                    ? path.cycles.at(static_cast<std::size_t>(m.cycle)).feedRate
                                    : m.feedRate;
            if (model.text(row, Model::Feed).isEmpty() ||
                !same(Model::Feed, ocam::mmPerSecToFeedPerMin(feed, unit))) {
                return wrong("feed");
            }
        }
        if (m.isArc() != !model.text(row, Model::Radius).isEmpty() ||
            (m.isArc() && (!same(Model::CenterX, ocam::fromMm(m.center.x(), unit)) ||
                           !same(Model::CenterY, ocam::fromMm(m.center.y(), unit)) ||
                           !same(Model::CenterZ, ocam::fromMm(m.center.z(), unit))))) {
            return wrong("arc");
        }
        if (m.kind == MoveKind::Cycle &&
            (!type.endsWith(QLatin1String("cycle")) ||
             !model.text(row, Model::Details).contains(QLatin1String("Bottom")))) {
            return wrong("cycle");
        }
    }
    return expected ? QString() : QStringLiteral("no %1 move listed").arg(expect);
}

/// Show Toolpath Data, from the menu to the list; the exit code of the test.
int toolpathDataTest(ocam::gui::MainWindow& window, const QString& png, const QString& expect) {
    auto fail = [](const QString& why) {
        std::fprintf(stderr, "%s\n", qPrintable(why));
        return 1;
    };
    settleToolpaths(window);
    if (!chooseFromTreeMenu(window, QStringLiteral("showToolpathData"))) {
        return fail(QStringLiteral("the operation's menu has no Show Toolpath Data"));
    }
    QPointer dialog = window.findChild<ocam::gui::ToolpathDataDialog*>();
    if (!dialog || !dialog->isVisible()) {
        return fail(QStringLiteral("no window opened"));
    }
    if (expect == QLatin1String("refused")) {
        if (!dialog->message().contains(QLatin1String("could not be generated")) ||
            dialog->table()->isVisible()) {
            return fail(QStringLiteral("not told why: ") + dialog->message());
        }
    } else {
        const auto* tree = window.findChild<ocam::gui::ProjectTree*>();
        const ocam::document::Document& doc = *window.document();
        const ocam::document::Setup* setup = nullptr;
        const ocam::document::Operation* op =
            doc.project().findOperation(tree->selection().id, &setup);
        ocam::CancelToken cancel;
        ocam::NullProgress progress;
        const auto path = ocam::document::cachedToolpath(doc, *setup, *op, cancel, progress);
        if (!dialog->message().isEmpty() || !dialog->model()->path() ||
            !(*dialog->model()->path() == *path)) {
            return fail(QStringLiteral("not the operation's toolpath: ") + dialog->message());
        }
        if (const QString wrong = checkToolpathRows(*dialog->model(), *path, expect);
            !wrong.isEmpty()) {
            return fail(wrong);
        }
        // Choosing it again raises the same window.
        if (!chooseFromTreeMenu(window, QStringLiteral("showToolpathData")) ||
            window.findChildren<ocam::gui::ToolpathDataDialog*>().size() != 1) {
            return fail(QStringLiteral("a second window"));
        }
        // Disabled there is nothing to list, and the window says so; enabled again, the moves
        // come back.
        if (!chooseFromTreeMenu(window, QStringLiteral("operationEnabled"))) {
            return fail(QStringLiteral("the operation's menu has no Enabled"));
        }
        settleToolpaths(window);
        if (!dialog->message().contains(QLatin1String("disabled")) ||
            dialog->model()->rowCount() != 0) {
            return fail(QStringLiteral("disabled: ") + dialog->message());
        }
        if (!chooseFromTreeMenu(window, QStringLiteral("operationEnabled"))) {
            return fail(QStringLiteral("the operation's menu has no Enabled"));
        }
        // It says so while generating, if the generation takes long enough to be seen.
        bool generating = false;
        for (int i = 0; i < 50 || window.toolpathsBusy(); ++i) {
            QApplication::processEvents();
            generating = generating || dialog->message().contains(QLatin1String("Generating"));
            QThread::msleep(1);
        }
        std::printf("generating shown: %s\n", generating ? "yes" : "no");
        std::fflush(stdout);
        if (dialog->model()->rowCount() != static_cast<int>(path->moves.size())) {
            return fail(QStringLiteral("enabled: ") + dialog->message());
        }
    }
    QApplication::processEvents();
    if (!dialog->grab().save(png)) {
        return 4;
    }
    // Without its operation the window closes.
    const ocam::gui::Command* remove = window.commands().find(QStringLiteral("edit.delete"));
    if (!remove) {
        return fail(QStringLiteral("no delete command"));
    }
    remove->action->trigger();
    settleToolpaths(window);
    return dialog && dialog->isVisible() ? fail(QStringLiteral("still open without its operation"))
                                         : 0;
}

/// Waits `ms` while the window works (the view's animations run).
void waitFor(int ms) {
    QElapsedTimer t;
    t.start();
    while (t.elapsed() < ms) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
    }
}

/// The workholding dialog (DL-0078) on the chuck project, its chuck given a jaw mount and jaw sets,
/// with a changed copy of the chuck in the (throwaway) library: the lists say what is where, Update
/// from library takes the library's copy, a jaw set is chosen; Cancel leaves the project as it was
/// and OK takes it as one undo step. The exit code of the test.
int workholdingSetupTestRun(ocam::gui::MainWindow& window, const QString& png) {
    namespace doc = ocam::document;
    auto* changing = const_cast<doc::Document*>(window.document());
    ocam::Uuid chuckId;
    ocam::Uuid hardId;
    changing->modify("Jaws", [&](doc::Project& p) {
        const auto sets = testJawSets(p.workholdings.front());
        chuckId = p.workholdings.front().id;
        hardId = sets.first.id;
        p.workholdings.push_back(sets.first);
        p.workholdings.push_back(sets.second);
    });
    // The library's copy of the chuck differs from the project's: its name.
    auto package = changing->workholdingPackage(chuckId);
    if (!package) {
        std::fprintf(stderr, "the project's chuck has no package\n");
        return 2;
    }
    const std::string oldName = package->workholding.name;
    package->workholding.name += " (library)";
    const QString libraryName = QString::fromStdString(package->workholding.name);
    try {
        // The folder is the test's own (--workholding-folder): emptied first, as the sets'
        // ids differ from run to run.
        std::filesystem::remove_all(ocam::gui::WorkholdingLibrary::folder());
        std::filesystem::create_directories(ocam::gui::WorkholdingLibrary::folder());
        ocam::gui::WorkholdingLibrary::save(*package);
        // The hard set, as the project has it: put on a library chuck chosen without jaws.
        if (const auto hard = changing->workholdingPackage(hardId)) {
            ocam::gui::WorkholdingLibrary::save(*hard);
        }
    } catch (const std::exception& e) {
        std::fprintf(stderr, "the library copy was not written: %s\n", e.what());
        return 2;
    }
    QApplication::processEvents();
    const auto setup = [&]() -> const doc::Setup& {
        return window.document()->project().setups.front();
    };
    int failed = 0;
    const auto expect = [&](bool ok, const char* what) {
        std::printf("%s: %s\n", what, ok ? "ok" : "WRONG");
        failed += ok ? 0 : 1;
    };
    const auto rows = [](const char* what, const QStringList& list) {
        std::printf("%s:\n", what);
        for (const QString& row : list) {
            std::printf("  %s\n", row.toUtf8().constData());
        }
    };
    const auto has = [](const QStringList& list, const QString& row) {
        return list.contains(row);
    };
    // A row starting and ending so (the kind names the chuck's interface as the test chuck has it).
    const auto hasRow = [](const QStringList& list, const QString& start, const QString& end) {
        return std::any_of(list.begin(), list.end(), [&](const QString& row) {
            return row.startsWith(start) && row.endsWith(end);
        });
    };
    const QString oldQ = QString::fromStdString(oldName);
    const std::string before = window.document()->undoLabel();

    auto* dialog = window.setUpWorkholding(setup().id, false);
    if (!dialog) {
        std::fprintf(stderr, "no workholding dialog\n");
        return 3;
    }
    QApplication::processEvents();
    rows("holds the stock", dialog->holdingRows());
    rows("jaws", dialog->jawRows());
    rows("copies", dialog->copyRows());
    expect(has(dialog->holdingRows(), QStringLiteral("(none) |  | ")) &&
               hasRow(dialog->holdingRows(), oldQ + QStringLiteral(" | Chuck ("),
                      QStringLiteral(") | This project (the library's differs)")),
           "the chuck is the project's, the library's differs");
    // Titled as the maintainer asked (2026-11-07): Workholding, Jaw Selection, Name.
    QStringList titles;
    for (const QGroupBox* box : dialog->findChildren<QGroupBox*>()) {
        titles << box->title();
    }
    const auto* holdingList =
        dialog->findChild<QTreeWidget*>(QStringLiteral("workholdingSetupHoldings"));
    expect(titles.contains(QStringLiteral("Workholding")) &&
               titles.contains(QStringLiteral("Jaw Selection")) && holdingList &&
               holdingList->headerItem()->text(0) == QStringLiteral("Name"),
           "titles: Workholding, Jaw Selection, Name");
    // No entry for the top jaws modelled with the chuck: it holds by a jaw set, and says so.
    expect(dialog->jawRows().size() == 3 &&
               has(dialog->jawRows(),
                   QStringLiteral("Step jaws (test) (OD) | Hard jaws | This project")) &&
               has(dialog->jawRows(),
                   QStringLiteral("Soft jaws (test) | Soft jaws | This project only")),
           "every way each set goes on");
    expect(dialog->problemText() == QStringLiteral("Choose the chuck's jaws in Jaw Selection."),
           "no jaws chosen yet: said");
    expect(
        has(dialog->copyRows(),
            QStringLiteral("%1 | %2 | Differs").arg(oldQ, QString::fromStdString(setup().name))) &&
            has(dialog->copyRows(), QStringLiteral("Step jaws (test) | (none) | Same")),
        "the project's copies, who uses them, the library's");
    expect(dialog->chooseJaws(QStringLiteral("Step jaws (test) (OD)")) && dialog->updateCopy(oldQ),
           "a jaw set chosen, the chuck updated");
    rows("copies after", dialog->copyRows());
    expect(has(dialog->copyRows(), QStringLiteral("%1 | %2 | Same")
                                       .arg(libraryName, QString::fromStdString(setup().name))) &&
               has(dialog->copyRows(), QStringLiteral("Step jaws (test) | %1 | Same")
                                           .arg(QString::fromStdString(setup().name))) &&
               dialog->problemText().isEmpty(),
           "in the dialog: the library's chuck, the set used, held");
    expect(!setup().workholding->jawSet &&
               window.document()->project().findWorkholding(chuckId)->name == oldName,
           "the project as it was while the dialog is open");
    if (!png.isEmpty()) {
        for (int i = 0; i < 50; ++i) {
            QApplication::processEvents();
            QThread::msleep(10);
        }
        dialog->snapshot().save(png);
    }
    // A chuck that takes jaw sets, chosen from the library: the library's set that fits goes on.
    const bool rechosen =
        dialog->chooseHolding(QStringLiteral("(none)")) && dialog->chooseHolding(libraryName);
    const auto& apart = dialog->edited().project().setups.front();
    rows("jaws for the library's chuck", dialog->jawRows());
    expect(rechosen && apart.workholding && apart.workholding->jawSet == hardId &&
               apart.workholding->jawOption == "OD" && dialog->problemText().isEmpty(),
           "a library chuck gets the jaw set that fits");
    dialog->reject();
    QApplication::processEvents();
    expect(!setup().workholding->jawSet &&
               window.document()->project().findWorkholding(chuckId)->name == oldName &&
               window.document()->undoLabel() == before,
           "Cancel: nothing changed");

    dialog = window.setUpWorkholding(setup().id, false);
    QApplication::processEvents();
    const bool chosen = dialog && dialog->chooseJaws(QStringLiteral("Step jaws (test) (OD)")) &&
                        dialog->updateCopy(oldQ);
    if (dialog) {
        dialog->accept();
    }
    QApplication::processEvents();
    const doc::WorkholdingTemplate* chuck = window.document()->project().findWorkholding(chuckId);
    expect(chosen && setup().workholding->jawSet == hardId &&
               setup().workholding->jawOption == "OD" && chuck &&
               chuck->name == package->workholding.name &&
               window.document()->undoLabel() == "Set up workholding",
           "OK: the set and the library's chuck, one undo step");
    const auto* named = window.findChild<QLabel*>(QStringLiteral("setupWorkholdingName"));
    const auto* jaws = window.findChild<QLabel*>(QStringLiteral("setupJawSetName"));
    const auto* setUp = window.findChild<QPushButton*>(QStringLiteral("setupWorkholdingNameSetUp"));
    expect(setUp && setUp->width() >= setUp->fontMetrics().horizontalAdvance(setUp->text()),
           "Set up... shows its text");
    expect(named && named->text() == libraryName && jaws &&
               jaws->text() == QStringLiteral("Step jaws (test) (OD)") &&
               !window.findChild<QLabel*>(QStringLiteral("setupWorkholdingNewer")),
           "the page names them, nothing differs");
    std::printf("workholding setup test: %s\n", failed == 0 ? "all ok" : "WRONG");
    std::fflush(stdout);
    return failed == 0 ? 0 : 6;
}

/// How the view follows the setup's machine as it stands (DL-0077), on the turret project (a
/// horizontal lathe): the cube's names, the standard views, a click on the cube, the machine made a
/// vertical lathe and then none. Saves the 3D view as a horizontal and as a vertical lathe. The
/// exit code of the test.
int orientationTest(ocam::gui::MainWindow& window, const QString& png) {
    namespace doc = ocam::document;
    using ocam::Mat3;
    using ocam::Vec3;
    ocam::gui::OcctView* view = window.view();
    int failed = 0;
    const auto expect = [&](bool ok, const QString& what) {
        std::printf("%s: %s\n", what.toUtf8().constData(), ok ? "ok" : "FAILED");
        if (!ok) {
            ++failed;
        }
    };
    const auto vec = [](const Vec3& v) {
        return QStringLiteral("(%1, %2, %3)")
            .arg(v.x(), 0, 'f', 3)
            .arg(v.y(), 0, 'f', 3)
            .arg(v.z(), 0, 'f', 3);
    };
    const auto along = [&](const Vec3& got, const Vec3& want, const QString& what) {
        expect((got - want.normalized()).norm() < 1e-3,
               QStringLiteral("%1 %2, wanted %3").arg(what, vec(got), vec(want.normalized())));
    };
    // The cube stands as the machine does; its triad names the machine's axes along the world's
    // right, back (away from the operator) and up.
    const auto cube = [&](const char* rightBackUp) {
        expect(view->cubeAxes() == QString::fromLatin1(rightBackUp),
               QStringLiteral("cube axes right, back, up: %1, wanted %2")
                   .arg(view->cubeAxes(), QString::fromLatin1(rightBackUp)));
    };
    const doc::Setup& setup = window.document()->project().setups.front();
    const Mat3 wcs = setup.wcs.normalized().toParent().linear();
    waitFor(800);

    // A horizontal lathe: Y up, the operator at -X, the spindle's Z to the right.
    cube("Z X Y");
    expect(view->viewUp().dot(wcs * Vec3::UnitY()) > 0.7,
           QStringLiteral("opened with Y up: %1").arg(vec(view->viewUp())));
    if (!png.isEmpty()) {
        view->dumpImage(png); // the 3D view: a grab of the window leaves it black
    }
    const ocam::gui::Command* top = window.commands().find(QStringLiteral("view.top"));
    if (!top) {
        std::fprintf(stderr, "no Top view command\n");
        return 2;
    }
    top->action->trigger(); // View > Standard Views > Top, as Alt+1 does
    waitFor(800);
    along(view->viewDirection(), wcs * -Vec3::UnitY(), QStringLiteral("Top looks down"));
    along(view->viewUp(), wcs * Vec3::UnitX(), QStringLiteral("Top has the turret up"));
    view->snapView(V3d_Yneg, false);
    along(view->viewDirection(), wcs * Vec3::UnitX(), QStringLiteral("Front looks from the door"));
    along(view->viewUp(), wcs * Vec3::UnitY(), QStringLiteral("Front has Y up"));

    // A click on the cube's edge above its front face: the view from the front and above.
    view->update();
    waitFor(300);
    const QPoint edge(90, view->height() - 90 - 30);
    const QPointF global = view->mapToGlobal(QPointF(edge));
    QMouseEvent move(QEvent::MouseMove, QPointF(edge), global, Qt::NoButton, Qt::NoButton,
                     Qt::NoModifier);
    QApplication::sendEvent(view, &move);
    waitFor(200);
    QMouseEvent press(QEvent::MouseButtonPress, QPointF(edge), global, Qt::LeftButton,
                      Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(view, &press);
    QMouseEvent release(QEvent::MouseButtonRelease, QPointF(edge), global, Qt::LeftButton,
                        Qt::NoButton, Qt::NoModifier);
    QApplication::sendEvent(view, &release);
    waitFor(1000);
    along(view->viewDirection(), wcs * Vec3(1.0, -1.0, 0.0),
          QStringLiteral("the cube's front top edge"));

    // The same machine as a vertical lathe: Z up. The view turns to it.
    auto* changing = const_cast<doc::Document*>(window.document());
    changing->modify("Stand it up", [&](doc::Project& p) {
        ocam::machine::Machine vtl = *ocam::machine::builtinMachine(p.setups.front().machine);
        vtl.orientation = ocam::machine::MachineOrientation{ocam::machine::AxisSense::PlusZ,
                                                            ocam::machine::AxisSense::MinusX};
        p.machines.push_back(vtl);
    });
    waitFor(1000);
    cube("-Y X Z");
    expect(view->viewUp().dot(wcs * Vec3::UnitZ()) > 0.7,
           QStringLiteral("turned to Z up: %1").arg(vec(view->viewUp())));
    if (!png.isEmpty()) {
        QString vertical = png;
        vertical.insert(vertical.lastIndexOf(QLatin1Char('.')), QStringLiteral("_vertical"));
        view->dumpImage(vertical);
    }
    view->snapView(V3d_Yneg, false);
    along(view->viewDirection(), wcs * Vec3::UnitX(),
          QStringLiteral("a vertical lathe's Front from the door"));
    along(view->viewUp(), wcs * Vec3::UnitZ(), QStringLiteral("a vertical lathe's Front, Z up"));

    // No machine of its own: the generic mill, as before.
    changing->modify("No machine", [&](doc::Project& p) {
        p.machines.clear();
        p.setups.front().machine.clear();
    });
    waitFor(1000);
    cube("X Y Z");
    view->snapView(V3d_Zpos, false);
    along(view->viewDirection(), wcs * -Vec3::UnitZ(), QStringLiteral("a mill's Top looks down Z"));
    along(view->viewUp(), wcs * Vec3::UnitY(), QStringLiteral("a mill's Top has Y up"));
    return failed == 0 ? 0 : 6;
}

/// A long toolpath, built here rather than generated: the window has to open and scroll in about
/// the time of a short one. The exit code of the test.
int toolpathDataLongTest(ocam::gui::MainWindow& window, const QString& png) {
    constexpr int kMoves = 500000;
    auto path = std::make_shared<ocam::toolpath::Toolpath>();
    path->moves.reserve(kMoves);
    for (int i = 0; i < kMoves; ++i) {
        ocam::toolpath::Move m;
        m.kind = i % 2 == 0 ? ocam::toolpath::MoveKind::Linear : ocam::toolpath::MoveKind::ArcCW;
        m.end = ocam::Vec3(i % 1000, (i + 1) % 2, -1.0);
        m.center = ocam::Vec3(i % 1000, 0.0, -1.0);
        m.feedRate = 10.0;
        path->moves.push_back(m);
    }
    QElapsedTimer timer;
    timer.start();
    auto* dialog = new ocam::gui::ToolpathDataDialog(&window);
    dialog->setOperationName(QStringLiteral("Long"));
    dialog->showPath(path, ocam::LengthUnit::Millimeter);
    dialog->show();
    QApplication::processEvents();
    const qint64 opened = timer.restart();
    dialog->table()->scrollToBottom();
    QApplication::processEvents();
    const int bottom = dialog->table()->rowAt(dialog->table()->viewport()->height() - 1);
    const qint64 scrolled = timer.elapsed();
    std::printf("%d rows: opened in %lld ms, scrolled to the end in %lld ms\n",
                dialog->model()->rowCount(), static_cast<long long>(opened),
                static_cast<long long>(scrolled));
    std::fflush(stdout);
    if (dialog->model()->rowCount() != kMoves || bottom < kMoves - 2 || opened > 5000 ||
        scrolled > 5000) {
        std::fprintf(stderr, "too slow or short of rows (last row shown %d)\n", bottom);
        return 1;
    }
    return dialog->grab().save(png) ? 0 : 4;
}

/// The default style with dock title-bar buttons (close, float) drawn in the palette's text
/// colour, so they stay visible on dark themes, and tooltips that wait a little longer. A combo
/// box too narrow for its choice shows the start of it, cut at the end, never in the middle.
class AppStyle : public QProxyStyle {
public:
    void drawControl(ControlElement element, const QStyleOption* option, QPainter* painter,
                     const QWidget* widget) const override {
        const auto* box = qstyleoption_cast<const QStyleOptionComboBox*>(option);
        if (element != CE_ComboBoxLabel || !box || box->editable) {
            QProxyStyle::drawControl(element, option, painter, widget);
            return;
        }
        QStyleOptionComboBox cut = *box;
        const QRect field = subControlRect(CC_ComboBox, box, SC_ComboBoxEditField, widget);
        int room = field.width() - 6;
        if (!box->currentIcon.isNull()) {
            room -= box->iconSize.width() + 4;
        }
        cut.currentText =
            box->fontMetrics.elidedText(box->currentText, Qt::ElideRight, std::max(room, 0));
        QProxyStyle::drawControl(element, &cut, painter, widget);
    }

    int styleHint(StyleHint hint, const QStyleOption* option, const QWidget* widget,
                  QStyleHintReturn* returnData) const override {
        return hint == SH_ToolTip_WakeUpDelay
                   ? ocam::gui::kToolTipDelay
                   : QProxyStyle::styleHint(hint, option, widget, returnData);
    }

    QIcon standardIcon(StandardPixmap icon, const QStyleOption* option,
                       const QWidget* widget) const override {
        switch (icon) {
        case SP_TitleBarCloseButton:
        case SP_DockWidgetCloseButton:
            return drawn(true);
        case SP_TitleBarNormalButton:
            return drawn(false);
        default:
            return QProxyStyle::standardIcon(icon, option, widget);
        }
    }

private:
    static QIcon drawn(bool close) {
        QIcon icon;
        const QColor normal = QApplication::palette().color(QPalette::WindowText);
        const QColor active = QApplication::palette().color(QPalette::Highlight).lighter(150);
        for (const int size : {12, 16, 20, 24, 32}) {
            for (const auto& [mode, colour] :
                 {std::pair{QIcon::Normal, normal}, std::pair{QIcon::Active, active}}) {
                QPixmap pixmap(size, size);
                pixmap.fill(Qt::transparent);
                QPainter painter(&pixmap);
                painter.setRenderHint(QPainter::Antialiasing);
                painter.setPen(
                    QPen(colour, std::max(1.4, size / 9.0), Qt::SolidLine, Qt::RoundCap));
                const double m = size * 0.25;
                if (close) {
                    painter.drawLine(QPointF(m, m), QPointF(size - m, size - m));
                    painter.drawLine(QPointF(size - m, m), QPointF(m, size - m));
                } else {
                    // Two overlapping windows: detach / float.
                    const double w = size * 0.42;
                    painter.drawRect(QRectF(m + size * 0.12, m, w, w));
                    painter.fillRect(QRectF(m, m + size * 0.12, w, w), Qt::transparent);
                    painter.drawRect(QRectF(m, m + size * 0.12, w, w));
                }
                painter.end();
                icon.addPixmap(pixmap, mode);
            }
        }
        return icon;
    }
};

} // namespace

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    QApplication::setStyle(new AppStyle);
    // The wheel changes a field only when it has the focus: scrolling a page does not.
    ocam::gui::WheelGuard::install();
    QApplication::setApplicationName(QStringLiteral("OxCAM"));
    QApplication::setOrganizationName(QStringLiteral("OxCAM"));
    QApplication::setApplicationVersion(QString::fromUtf8(
        ocam::kVersionString.data(), static_cast<int>(ocam::kVersionString.size())));

    QCommandLineParser parser;
    parser.setApplicationDescription(QStringLiteral("OxCAM desktop application"));
    parser.addHelpOption();
    parser.addVersionOption();
    const QCommandLineOption smokeTest(QStringLiteral("smoke-test"),
                                       QStringLiteral("Open the file, render, and exit."));
    const QCommandLineOption screenshot(
        QStringLiteral("screenshot"),
        QStringLiteral("With --smoke-test, save the 3D view to <file>."), QStringLiteral("file"));
    const QCommandLineOption screenshotWindow(
        QStringLiteral("screenshot-window"),
        QStringLiteral("With --smoke-test, save the whole window to <file>."),
        QStringLiteral("file"));
    const QCommandLineOption screenshotTools(
        QStringLiteral("screenshot-tools"),
        QStringLiteral("Save the tool library dialog for the given library to <file> and exit."),
        QStringLiteral("file"));
    parser.addOption(smokeTest);
    parser.addOption(screenshot);
    parser.addOption(screenshotWindow);
    const QCommandLineOption screenshotPost(
        QStringLiteral("screenshot-post"),
        QStringLiteral(
            "With --smoke-test, post process the first setup and save the dialog to <file>."),
        QStringLiteral("file"));
    parser.addOption(screenshotPost);
    const QCommandLineOption screenshotPostSettings(
        QStringLiteral("screenshot-post-settings"),
        QStringLiteral("With --screenshot-post, also save the post settings dialog (shown before "
                       "posting, then accepted) to <file>."),
        QStringLiteral("file"));
    parser.addOption(screenshotPostSettings);
    const QCommandLineOption screenshotSim(
        QStringLiteral("screenshot-sim"),
        QStringLiteral(
            "With --smoke-test, simulate the current setup and save the 3D view to <file>."),
        QStringLiteral("file"));
    parser.addOption(screenshotSim);
    const QCommandLineOption simLastSetup(
        QStringLiteral("sim-last-setup"),
        QStringLiteral("With --screenshot-sim, simulate the last setup instead of the first."));
    parser.addOption(simLastSetup);
    const QCommandLineOption simPosition(
        QStringLiteral("sim-position"),
        QStringLiteral("With --screenshot-sim, show operation <op> after <move> moves."),
        QStringLiteral("op:move"));
    parser.addOption(simPosition);
    const QCommandLineOption simPlay(
        QStringLiteral("sim-play"),
        QStringLiteral("With --screenshot-sim, play for <seconds> (after --sim-position), pause "
                       "and print how far playback got."),
        QStringLiteral("seconds"));
    parser.addOption(simPlay);
    const QCommandLineOption machinePart(
        QStringLiteral("machine-part"),
        QStringLiteral("With --screenshot-sim, hide (0) or show (1) a part of the machine."),
        QStringLiteral("name=0|1"));
    parser.addOption(machinePart);
    const QCommandLineOption screenshotClosedSim(
        QStringLiteral("screenshot-closed-sim"),
        QStringLiteral("With --screenshot-sim, close the simulation and save the 3D view to "
                       "<file>."),
        QStringLiteral("file"));
    parser.addOption(screenshotClosedSim);
    const QCommandLineOption newTool(
        QStringLiteral("new-tool"),
        QStringLiteral("With --screenshot-tools, add a tool of <type> (e.g. bull_nose_end_mill) "
                       "first, as the Add button does."),
        QStringLiteral("type"));
    parser.addOption(newTool);
    const QCommandLineOption showHolders(
        QStringLiteral("show-holders"),
        QStringLiteral("With --screenshot-tools, show the holders tab."));
    parser.addOption(showHolders);
    const QCommandLineOption newTurningHolder(
        QStringLiteral("new-turning-holder"),
        QStringLiteral("With --screenshot-tools, add an ISO turning holder (shank or boring)."),
        QStringLiteral("kind"));
    parser.addOption(newTurningHolder);
    const QCommandLineOption pressEnter(
        QStringLiteral("press-enter"),
        QStringLiteral("With --screenshot-tools, press Enter in the tool's name; exit 5 when that "
                       "changes the tool list."));
    parser.addOption(pressEnter);
    const QCommandLineOption selectTool(
        QStringLiteral("select-tool"),
        QStringLiteral("With --screenshot-tools, select the tool numbered <n>."),
        QStringLiteral("n"));
    parser.addOption(selectTool);
    const QCommandLineOption toolSearch(
        QStringLiteral("tool-search"),
        QStringLiteral("With --screenshot-tools, filter the tool list by <text>."),
        QStringLiteral("text"));
    parser.addOption(toolSearch);
    const QCommandLineOption toolTypeFilter(
        QStringLiteral("tool-type"),
        QStringLiteral("With --screenshot-tools, show one family (milling, hole_making, turning) "
                       "or one tool type (e.g. drill)."),
        QStringLiteral("id"));
    parser.addOption(toolTypeFilter);
    const QCommandLineOption toolDiameter(
        QStringLiteral("tool-diameter"),
        QStringLiteral("With --screenshot-tools, show diameters from <min> to <max> mm (0 = no "
                       "limit)."),
        QStringLiteral("min:max"));
    parser.addOption(toolDiameter);
    const QCommandLineOption screenshotPresets(
        QStringLiteral("screenshot-presets"),
        QStringLiteral("With --screenshot-tools, also save the first tool's preset editor to "
                       "<file>."),
        QStringLiteral("file"));
    parser.addOption(screenshotPresets);
    const QCommandLineOption screenshotHolderImport(
        QStringLiteral("screenshot-holder-import"),
        QStringLiteral("Save the holder import dialog for the STEP file to <file>, exit."),
        QStringLiteral("file"));
    parser.addOption(screenshotHolderImport);
    const QCommandLineOption selectSetup(
        QStringLiteral("select-setup"),
        QStringLiteral("With --smoke-test, select the first setup before taking screenshots."));
    parser.addOption(selectSetup);
    const QCommandLineOption screenshotPlacement(
        QStringLiteral("screenshot-placement"),
        QStringLiteral("With --smoke-test, place the first setup on its machine, save the dialog "
                       "to <png>, exit."),
        QStringLiteral("png"));
    parser.addOption(screenshotPlacement);
    const QCommandLineOption placeWork(
        QStringLiteral("place-work"),
        QStringLiteral("With --screenshot-placement, click this WCS point of the work first."),
        QStringLiteral("x,y,z"));
    parser.addOption(placeWork);
    const QCommandLineOption placeOn(
        QStringLiteral("place-on"),
        QStringLiteral("With --screenshot-placement, click this machine-zero point."),
        QStringLiteral("x,y,z"));
    parser.addOption(placeOn);
    const QCommandLineOption placeSnap(
        QStringLiteral("place-snap"),
        QStringLiteral("With --screenshot-placement, snap to the face's centre (face), the "
                       "rotary axis on it (axis) or the point (point)."),
        QStringLiteral("mode"));
    parser.addOption(placeSnap);
    const QCommandLineOption selectOperation(
        QStringLiteral("select-operation"),
        QStringLiteral("With --smoke-test, select operation <n> (0-based) of the first setup."),
        QStringLiteral("n"));
    parser.addOption(selectOperation);
    const QCommandLineOption toolpathData(
        QStringLiteral("toolpath-data-test"),
        QStringLiteral("With --select-operation, open Show Toolpath Data from the operation's "
                       "right-click menu, check it against the toolpath and save it to <file>."),
        QStringLiteral("file"));
    parser.addOption(toolpathData);
    const QCommandLineOption toolpathDataExpect(
        QStringLiteral("toolpath-data-expect"),
        QStringLiteral(
            "With --toolpath-data-test, a move type the list must have, or \"refused\"."),
        QStringLiteral("text"));
    parser.addOption(toolpathDataExpect);
    const QCommandLineOption toolpathDataLong(
        QStringLiteral("toolpath-data-long"),
        QStringLiteral("With --smoke-test, list 500000 moves and save the window to <file>."),
        QStringLiteral("file"));
    parser.addOption(toolpathDataLong);
    const QCommandLineOption setChoice(
        QStringLiteral("set-choice"),
        QStringLiteral("With --select-operation, choose <key>=<value> in a parameter's list."),
        QStringLiteral("key=value"));
    parser.addOption(setChoice);
    const QCommandLineOption setFlag(
        QStringLiteral("set-flag"),
        QStringLiteral("With --select-operation, tick (on) or clear (off) a parameter's box: "
                       "<key>=on|off."),
        QStringLiteral("key=on|off"));
    parser.addOption(setFlag);
    const QCommandLineOption expectParams(
        QStringLiteral("expect-params"),
        QStringLiteral("With --select-operation, exit 0 when the operation's page shows these "
                       "parameters (comma-separated keys) and not those of --expect-no-params, "
                       "else 11."),
        QStringLiteral("keys"));
    parser.addOption(expectParams);
    const QCommandLineOption expectNoParams(
        QStringLiteral("expect-no-params"),
        QStringLiteral("With --expect-params: parameters the page must not show."),
        QStringLiteral("keys"));
    parser.addOption(expectNoParams);
    const QCommandLineOption reselectTest(
        QStringLiteral("reselect-test"),
        QStringLiteral("With --smoke-test, show the in-process stock of operation 0 at a new "
                       "resolution, pick operation 1 and 0 again while it is simulated and fail "
                       "if the simulation starts over."));
    parser.addOption(reselectTest);
    const QCommandLineOption pickTest(
        QStringLiteral("pick-test"),
        QStringLiteral("With --smoke-test, click the centre of the 3D view in the top view and "
                       "fail unless a face is selected."));
    parser.addOption(pickTest);
    const QCommandLineOption wheelTest(
        QStringLiteral("wheel-test"),
        QStringLiteral("With --select-setup, turn the mouse wheel over a spin box of the setup "
                       "page: fail if it changes while not focused, or does not while focused."));
    parser.addOption(wheelTest);
    const QCommandLineOption turretEditTest(
        QStringLiteral("turret-edit-test"),
        QStringLiteral("With --smoke-test and --select-setup: work the setup page's turret: add a "
                       "tool that rides along, move a station, set a tool's tip, take the tool "
                       "out; print what the project has after each."));
    parser.addOption(turretEditTest);
    const QCommandLineOption stationToolsTest(
        QStringLiteral("station-tools-test"),
        QStringLiteral("With --smoke-test of the turret project, --select-setup and "
                       "--fittings-folder <empty dir>: odd stations taking milling units only, "
                       "put tools and blocks on stations through the turret's lists' pop-ups."));
    parser.addOption(stationToolsTest);
    const QCommandLineOption setupMachineListTest(
        QStringLiteral("setup-machine-list-test"),
        QStringLiteral("With --smoke-test of the turret project, --select-setup, --settings-dir "
                       "of its own and --fittings-folder: open the libraries from the setup "
                       "page's lists (Manage machine library..., Manage blocks...), use a machine "
                       "for the setup, close them."));
    parser.addOption(setupMachineListTest);
    const QCommandLineOption turretPreviewTest(
        QStringLiteral("turret-preview-test"),
        QStringLiteral("With --smoke-test of the turret project, --select-setup and "
                       "--fittings-folder <empty dir>: show the turret preview from the setup "
                       "page, choose stations from the table and the preview, check where the "
                       "turret stands. With --screenshot-window, also save the preview's view "
                       "beside it (_preview)."));
    parser.addOption(turretPreviewTest);
    const QCommandLineOption turretWindowShot(
        QStringLiteral("turret-window"),
        QStringLiteral("With --smoke-test, --select-setup and --screenshot-window: open the "
                       "setup's turret window on <station> and save it, the preview's view "
                       "beside it (_preview)."),
        QStringLiteral("station"));
    parser.addOption(turretWindowShot);
    const QCommandLineOption setupMachineListClose(
        QStringLiteral("setup-machine-list-close"),
        QStringLiteral("With --setup-machine-list-test: close the machine library instead of "
                       "using a machine, then open and close the blocks and centres libraries."));
    parser.addOption(setupMachineListClose);
    const QCommandLineOption fittingsFolder(
        QStringLiteral("fittings-folder"),
        QStringLiteral("Use <dir> as the turret block and centre library for this run."),
        QStringLiteral("dir"));
    parser.addOption(fittingsFolder);
    const QCommandLineOption showBuiltinsTest(
        QStringLiteral("show-builtins-test"),
        QStringLiteral("With --settings-dir of its own: the machine and block libraries and the "
                       "post choice of the project's first setup, their Show built-ins box off "
                       "then ticked; print, exit."),
        QStringLiteral("project"));
    parser.addOption(showBuiltinsTest);
    const QCommandLineOption orientationTestOption(
        QStringLiteral("orientation-test"),
        QStringLiteral("With --smoke-test of the turret project and --select-setup: check the view "
                       "follows how the setup's machine stands (the cube, the standard views, a "
                       "click on the cube), make it a vertical lathe and then none; with "
                       "--screenshot-window, save the 3D view as each lathe."));
    parser.addOption(orientationTestOption);
    const QCommandLineOption jawsTest(
        QStringLiteral("jaws-test"),
        QStringLiteral("With --smoke-test of the chuck project and --select-setup: give its chuck "
                       "a jaw mount and two jaw sets, and work the setup page's jaw choices; "
                       "print what the project has."));
    parser.addOption(jawsTest);
    const QCommandLineOption workholdingSetupTest(
        QStringLiteral("workholding-setup-test"),
        QStringLiteral("With --smoke-test of the chuck project, --select-setup and "
                       "--workholding-folder <empty dir>: put a changed copy of its chuck in the "
                       "library, and work the workholding dialog: its lists, Update from library, "
                       "a jaw set, Cancel and OK; with --screenshot-window, save the dialog."));
    parser.addOption(workholdingSetupTest);
    const QCommandLineOption workholdingJawsTest(
        QStringLiteral("workholding-jaws-test"),
        QStringLiteral("With --workholding-folder <dir> holding chuck.ocamhold's copy: write a "
                       "chuck with a jaw mount and a jaw set there, read them in the library's "
                       "Jaws section, change it, and save the dialog to <file>."),
        QStringLiteral("file"));
    parser.addOption(workholdingJawsTest);
    const QCommandLineOption wcsPickTest(
        QStringLiteral("wcs-pick-test"),
        QStringLiteral("With --smoke-test and --select-setup: set the setup's WCS origin from a "
                       "picked point, press Pick, click the centre of the 3D view in the top view "
                       "and fail unless the origin is then on a face."));
    parser.addOption(wcsPickTest);
    const QCommandLineOption wcsAxisTest(
        QStringLiteral("wcs-axis-test"),
        QStringLiteral("With --smoke-test and --select-setup: orient the setup by X and Z picks, "
                       "press X's Pick, click the project point <x,y,z> (on an edge) in the top "
                       "view and print the WCS X axis then."),
        QStringLiteral("x,y,z"));
    parser.addOption(wcsAxisTest);
    const QCommandLineOption tracePickTest(
        QStringLiteral("trace-pick-test"),
        QStringLiteral("With --smoke-test: add a 2D trace to the first setup, press its geometry's "
                       "Select, click the project points <x,y,z/x,y,z/...> (on edges) one after "
                       "another in the top view, print its edges and chains, save the window to "
                       "<file> when given (--screenshot-window); fail unless they make one open "
                       "chain."),
        QStringLiteral("points"));
    parser.addOption(tracePickTest);
    const QCommandLineOption hoverTest(
        QStringLiteral("hover-test"),
        QStringLiteral("With --select-operation of a 2D contour: pick geometry, hover an edge in "
                       "the top view, save the 3D view to <file>; fail unless the whole outline "
                       "lights up."),
        QStringLiteral("file"));
    parser.addOption(hoverTest);
    const QCommandLineOption screenshotFittings(
        QStringLiteral("screenshot-fittings"),
        QStringLiteral("Save the turret blocks and centres dialog to <file>, exit."),
        QStringLiteral("file"));
    parser.addOption(screenshotFittings);
    const QCommandLineOption fittingName(
        QStringLiteral("fitting"),
        QStringLiteral("With --screenshot-fittings, show the fitting of this name."),
        QStringLiteral("name"));
    parser.addOption(fittingName);
    const QCommandLineOption fittingOnLathe(
        QStringLiteral("fitting-on-lathe"),
        QStringLiteral("With --screenshot-fittings, show the fitting on the generic lathe."));
    parser.addOption(fittingOnLathe);
    const QCommandLineOption fittingsEditTest(
        QStringLiteral("fittings-edit-test"),
        QStringLiteral("Make a fitting from this STEP file in a throwaway library: picks on its "
                       "faces, edges and a vertex, a click on the preview, save and load back. "
                       "Prints each and exits 0 when all hold."),
        QStringLiteral("file"));
    parser.addOption(fittingsEditTest);
    const QCommandLineOption quietWindows(
        QStringLiteral("quiet-windows"),
        QStringLiteral("With a screenshot option: windows open off the screen and do not take the "
                       "focus, to leave the desktop alone."));
    parser.addOption(quietWindows);
    const QCommandLineOption screenshotHub(QStringLiteral("screenshot-hub"),
                                           QStringLiteral("Save the start hub to <file>, exit."),
                                           QStringLiteral("file"));
    parser.addOption(screenshotHub);
    const QCommandLineOption screenshotMachines(
        QStringLiteral("screenshot-machines"),
        QStringLiteral("Save the machine library dialog to <file>, exit."), QStringLiteral("file"));
    parser.addOption(screenshotMachines);
    const QCommandLineOption machineStands(
        QStringLiteral("machine-stands"),
        QStringLiteral("With --screenshot-machines: choose the machine's Up and Front (such as "
                       "+Y,+Z) and print how the preview stands before and after."),
        QStringLiteral("up,front"));
    parser.addOption(machineStands);
    const QCommandLineOption screenshotWorkholding(
        QStringLiteral("screenshot-workholding"),
        QStringLiteral("Save the workholding library dialog to <file>, exit."),
        QStringLiteral("file"));
    parser.addOption(screenshotWorkholding);
    const QCommandLineOption workholdingFolder(
        QStringLiteral("workholding-folder"),
        QStringLiteral("Use <dir> as the workholding library for this run."),
        QStringLiteral("dir"));
    parser.addOption(workholdingFolder);
    const QCommandLineOption workholdingStep(
        QStringLiteral("workholding-step"),
        QStringLiteral("With --screenshot-workholding, make a new template of the bodies of the "
                       "STEP <file> in the editor and pick a datum on its first part."),
        QStringLiteral("file"));
    parser.addOption(workholdingStep);
    const QCommandLineOption screenshotWizard(
        QStringLiteral("screenshot-wizard"),
        QStringLiteral("Set up the bodies of --workholding-step in the setup wizard: three "
                       "bodies as a self-centring vise (its first jaw on -Y, its second on +Y), "
                       "four as a 3-jaw chuck (a body with a bore, the first jaw on +X), five as "
                       "a self-centring vise whose first jaw is two bodies and a riser to remove "
                       "(merged and removed in the wizard first); close it on a trial stock, save "
                       "the wizard to <file>, finish, exit."),
        QStringLiteral("file"));
    parser.addOption(screenshotWizard);
    const QCommandLineOption workholdingUpTest(
        QStringLiteral("workholding-up-test"),
        QStringLiteral("Set up the Y-up self-centring vise of --workholding-step in the wizard "
                       "(its up must be the model's +Y), save it to --workholding-folder with up "
                       "+X, check the editor's warning, save the editor to <file>, then use the "
                       "jaw tops' direction and pick up on the jaw tops (<file>_picked.png)."),
        QStringLiteral("file"));
    parser.addOption(workholdingUpTest);
    const QCommandLineOption machineId(
        QStringLiteral("machine"),
        QStringLiteral("With --screenshot-machines, select the machine <id>."),
        QStringLiteral("id"));
    parser.addOption(machineId);
    const QCommandLineOption jog(
        QStringLiteral("jog"),
        QStringLiteral("With --screenshot-machines, put a preview axis at a value (A=30)."),
        QStringLiteral("axis=value"));
    parser.addOption(jog);
    const QCommandLineOption showModel(
        QStringLiteral("show-model"),
        QStringLiteral("With --screenshot-machines, scroll the editor to the machine model."));
    parser.addOption(showModel);
    const QCommandLineOption selectPart(
        QStringLiteral("select-part"),
        QStringLiteral("With --screenshot-machines, select the part <name>."),
        QStringLiteral("name"));
    parser.addOption(selectPart);
    const QCommandLineOption isolatePart(
        QStringLiteral("isolate"),
        QStringLiteral("With --select-part, show only the selected part in the preview."));
    parser.addOption(isolatePart);
    const QCommandLineOption pickPreview(
        QStringLiteral("pick-preview"),
        QStringLiteral("With --screenshot-machines, click the preview's centre and print the "
                       "part selected (fails unless it is --select-part's)."));
    parser.addOption(pickPreview);
    const QCommandLineOption pickAxis(
        QStringLiteral("pick-axis"),
        QStringLiteral("With --screenshot-machines, pick rotary axis <axis> on the round face at "
                       "a machine-zero point and print the result."),
        QStringLiteral("axis@x,y,z"));
    parser.addOption(pickAxis);
    const QCommandLineOption pickGauge(
        QStringLiteral("pick-gauge"),
        QStringLiteral("With --screenshot-machines, pick the spindle gauge point: a round face of "
                       "the spindle at the first machine-zero point, the seat face at the second."),
        QStringLiteral("x,y,z;x,y,z"));
    parser.addOption(pickGauge);
    const QCommandLineOption maximized(
        QStringLiteral("maximized"),
        QStringLiteral("With --screenshot-machines, show the dialog maximized."));
    parser.addOption(maximized);
    const QCommandLineOption panelSweep(
        QStringLiteral("panel-sweep"),
        QStringLiteral("With --smoke-test, resize the Properties dock through a range of widths "
                       "and fail when the page is wider than the dock at any of them, saving "
                       "a screenshot of each such width to <folder>."),
        QStringLiteral("folder"));
    parser.addOption(panelSweep);
    const QCommandLineOption screenshotPartsImport(
        QStringLiteral("screenshot-parts-import"),
        QStringLiteral("Save the machine parts import dialog for <file> (the positional argument) "
                       "to <png>, exit."),
        QStringLiteral("png"));
    parser.addOption(screenshotPartsImport);
    const QCommandLineOption mergeLevel(
        QStringLiteral("merge-level"),
        QStringLiteral("With --screenshot-parts-import, merge at assembly level <n>."),
        QStringLiteral("n"));
    parser.addOption(mergeLevel);
    const QCommandLineOption screenshotPreferences(
        QStringLiteral("screenshot-preferences"),
        QStringLiteral("Save the preferences dialog to <file>, exit."), QStringLiteral("file"));
    parser.addOption(screenshotPreferences);
    const QCommandLineOption preferencesPage(
        QStringLiteral("preferences-page"),
        QStringLiteral("With --screenshot-preferences, show page <n> (0-based)."),
        QStringLiteral("n"));
    parser.addOption(preferencesPage);
    const QCommandLineOption preferencesType(
        QStringLiteral("preferences-type"),
        QStringLiteral("With --screenshot-preferences, show the defaults of operation <type>."),
        QStringLiteral("type"));
    parser.addOption(preferencesType);
    const QCommandLineOption settingsDir(
        QStringLiteral("settings-dir"),
        QStringLiteral("Keep the settings in <dir> (INI files) instead of the user's."),
        QStringLiteral("dir"));
    parser.addOption(settingsDir);
    const QCommandLineOption listCommands(
        QStringLiteral("list-commands"),
        QStringLiteral("Print the commands with their shortcuts, one per line, and exit."));
    parser.addOption(listCommands);
    const QCommandLineOption pressShortcut(
        QStringLiteral("press-shortcut"),
        QStringLiteral("Press <key> three times, print the toolpath display after each, exit."),
        QStringLiteral("key"));
    parser.addOption(pressShortcut);
    const QCommandLineOption screenshotToolTip(
        QStringLiteral("screenshot-tooltip"),
        QStringLiteral("Show the tooltip of command --tooltip-command, save it to <file>, exit."),
        QStringLiteral("file"));
    parser.addOption(screenshotToolTip);
    const QCommandLineOption toolTipCommand(
        QStringLiteral("tooltip-command"),
        QStringLiteral("With --screenshot-tooltip, the command (default operation.pocket2d)."),
        QStringLiteral("id"), QStringLiteral("operation.pocket2d"));
    parser.addOption(toolTipCommand);
    const QCommandLineOption runCommand(
        QStringLiteral("run-command"),
        QStringLiteral("With --smoke-test, run command <id> before taking screenshots."),
        QStringLiteral("id"));
    parser.addOption(runCommand);
    const QCommandLineOption writeLibraryStrings(
        QStringLiteral("write-library-strings"),
        QStringLiteral("Write the libraries' texts for lupdate to <file>, exit."),
        QStringLiteral("file"));
    parser.addOption(writeLibraryStrings);
    parser.addOption(screenshotTools);
    const QCommandLineOption checkHolderEditor(
        QStringLiteral("check-holder-editor"),
        QStringLiteral("Drive the holder editor's widgets, check the result, then exit"),
        QStringLiteral("dir"));
    parser.addOption(checkHolderEditor);
    parser.addPositionalArgument(QStringLiteral("file"),
                                 QStringLiteral("Project (.ocam) or STEP file."));
    parser.process(app);
    const QStringList files = parser.positionalArguments();
    if (parser.isSet(writeLibraryStrings)) {
        return ocam::gui::writeLibraryStrings(parser.value(writeLibraryStrings)) ? 0 : 4;
    }
    if (parser.isSet(settingsDir)) {
        QSettings::setDefaultFormat(QSettings::IniFormat);
        QSettings::setPath(QSettings::IniFormat, QSettings::UserScope, parser.value(settingsDir));
    }
    // Settings of their own (tests) leave the user's data folders alone.
    ocam::gui::importOpenCamSettings(!parser.isSet(settingsDir));
    // Meshes made from STEP parts, kept between sessions (a real machine loads in about 2 s
    // from STEP, 0.1 s from these). Always safe to delete.
    {
        const QString cache =
            parser.isSet(settingsDir)
                ? QDir(parser.value(settingsDir)).filePath(QStringLiteral("step-meshes"))
                : QDir(QStandardPaths::writableLocation(QStandardPaths::CacheLocation))
                      .filePath(QStringLiteral("step-meshes"));
        ocam::document::setStepMeshCacheFolder(std::filesystem::path(cache.toStdU16String()));
    }
    ocam::gui::migrateOperationDefaults();
    ocam::gui::installTranslations();
    ocam::gui::installToolTipFormatting(app);

    if (parser.isSet(checkHolderEditor)) {
        return ocam::gui::checkHolderEditor(parser.value(checkHolderEditor));
    }
    if (parser.isSet(screenshotTools)) {
        if (files.isEmpty()) {
            std::fprintf(stderr, "--screenshot-tools requires a tool library file\n");
            return 2;
        }
        try {
            const auto lib = ocam::tooling::loadToolLibrary(
                std::filesystem::path(files.front().toStdU16String()));
            ocam::gui::ToolLibraryDialog dialog(lib,
                                                ocam::gui::ToolLibraryDialog::Mode::LibraryFile);
            if (parser.value(newTool) == QStringLiteral("solid_boring_bar")) {
                dialog.addSolidBoringBar();
            } else if (parser.isSet(newTool)) {
                const auto type = ocam::tooling::parseToolType(parser.value(newTool).toStdString());
                if (!type) {
                    std::fprintf(stderr, "unknown tool type\n");
                    return 2;
                }
                dialog.addTool(*type);
            }
            if (parser.isSet(toolSearch) || parser.isSet(toolTypeFilter) ||
                parser.isSet(toolDiameter)) {
                ocam::tooling::ToolFilter filter;
                filter.text = parser.value(toolSearch).toStdString();
                const std::string id = parser.value(toolTypeFilter).toStdString();
                if (id == "milling") {
                    filter.family = ocam::tooling::ToolFamily::Milling;
                } else if (id == "hole_making") {
                    filter.family = ocam::tooling::ToolFamily::HoleMaking;
                } else if (id == "turning") {
                    filter.family = ocam::tooling::ToolFamily::Turning;
                } else if (id == "probing") {
                    filter.family = ocam::tooling::ToolFamily::Probing;
                } else if (!id.empty()) {
                    filter.type = ocam::tooling::parseToolType(id);
                    if (!filter.type) {
                        std::fprintf(stderr, "unknown tool type\n");
                        return 2;
                    }
                }
                const QStringList range = parser.value(toolDiameter).split(QLatin1Char(':'));
                filter.minDiameter = range.value(0).toDouble();
                filter.maxDiameter = range.value(1).toDouble();
                dialog.setToolFilter(filter);
            }
            if (parser.isSet(selectTool) &&
                !dialog.selectToolNumber(parser.value(selectTool).toInt())) {
                std::fprintf(stderr, "no tool %s shown\n", qPrintable(parser.value(selectTool)));
                return 2;
            }
            if (parser.value(newTurningHolder) == QStringLiteral("plain")) {
                dialog.addTurningStick(false);
            } else if (parser.isSet(newTurningHolder)) {
                dialog.addTurningHolder(parser.value(newTurningHolder) == QStringLiteral("boring"));
            }
            if (parser.isSet(showHolders)) {
                dialog.showHolderTab();
            }
            if (parser.isSet(screenshotPresets) && !lib.tools.empty()) {
                ocam::gui::CuttingPresetDialog presets(lib.tools.front(), 0);
                presets.show();
                QTimer::singleShot(500, &app, [&] {
                    QApplication::exit(presets.grab().save(parser.value(screenshotPresets)) ? 0
                                                                                            : 4);
                });
                if (QApplication::exec() != 0) {
                    return 4;
                }
            }
            dialog.show();
            QTimer::singleShot(500, &app, [&] {
                if (parser.isSet(pressEnter)) {
                    auto* name = dialog.findChild<QLineEdit*>(QStringLiteral("toolName"));
                    const std::size_t before = dialog.library().tools.size();
                    if (name) {
                        name->setFocus();
                        QKeyEvent enter(QEvent::KeyPress, Qt::Key_Return, Qt::NoModifier);
                        QApplication::sendEvent(name, &enter);
                    }
                    if (!name || dialog.library().tools.size() != before) {
                        std::fprintf(stderr, "Enter in the tool's name changed the tool list\n");
                        QApplication::exit(5);
                        return;
                    }
                }
                QApplication::exit(dialog.grab().save(parser.value(screenshotTools)) ? 0 : 4);
            });
            return QApplication::exec();
        } catch (const ocam::Error& e) {
            std::fprintf(stderr, "%s\n", e.what());
            return 3;
        }
    }

    if (parser.isSet(screenshotHolderImport)) {
        if (files.isEmpty()) {
            std::fprintf(stderr, "--screenshot-holder-import requires a STEP file\n");
            return 2;
        }
        try {
            ocam::gui::HolderImportDialog dialog(
                ocam::cad::Model::importStep(std::filesystem::path(files.front().toStdU16String())),
                QFileInfo(files.front()).completeBaseName());
            dialog.show();
            QTimer::singleShot(500, &app, [&] {
                QApplication::exit(dialog.grab().save(parser.value(screenshotHolderImport)) ? 0
                                                                                            : 4);
            });
            return QApplication::exec();
        } catch (const ocam::Error& e) {
            std::fprintf(stderr, "%s\n", e.what());
            return 3;
        }
    }

    if (parser.isSet(workholdingFolder)) {
        ocam::gui::WorkholdingLibrary::setFolderOverride(
            std::filesystem::path(parser.value(workholdingFolder).toStdU16String()));
    }
    if (parser.isSet(fittingsFolder)) {
        ocam::gui::LatheFittingLibrary::setFolderOverride(
            std::filesystem::path(parser.value(fittingsFolder).toStdU16String()));
    }
    if (parser.isSet(screenshotWizard)) {
        if (!parser.isSet(workholdingStep)) {
            std::fprintf(stderr, "--screenshot-wizard needs --workholding-step\n");
            return 2;
        }
        try {
            ocam::CancelToken cancel;
            ocam::NullProgress progress;
            ocam::document::WorkholdingPackage start;
            start.workholding.name = "Wizard test";
            std::map<std::string, ocam::cad::Model> models;
            std::vector<std::string> names;
            for (auto& body : ocam::cad::importStepAssembly(
                     std::filesystem::path(parser.value(workholdingStep).toStdU16String()), cancel,
                     progress)) {
                const ocam::document::WorkholdingPart part =
                    ocam::document::addWorkholdingPart(start, body.name, body.model);
                models.emplace(part.sha256,
                               ocam::cad::Model::importStepData(start.files.at(part.sha256),
                                                                body.name, cancel, progress));
                names.push_back(body.name);
            }
            const bool chuck = names.size() == 4;
            const bool extras = names.size() == 5;
            if (names.size() != 3 && !chuck && !extras) {
                std::fprintf(stderr, "expected a base and two jaws, or a chuck and three jaws\n");
                return 3;
            }
            ocam::gui::WorkholdingWizard wizard(start, models);
            wizard.show();
            if (extras) {
                // Leave the riser out and make the first jaw's two bodies one part.
                if (!wizard.removePart(names[4]) || !wizard.mergeParts({names[1], names[2]})) {
                    std::fprintf(stderr, "removing or merging parts failed\n");
                    return 8;
                }
                names = wizard.partNames();
                if (names.size() != 3 || wizard.partModel(names[1])->info().solids != 2) {
                    std::fprintf(stderr, "the parts are not a base and two jaws\n");
                    return 8;
                }
            }
            // The face of part `i` whose normal is `n` (exact geometry).
            auto face = [&](std::size_t i, const ocam::Vec3& n) {
                const auto& m = *wizard.partModel(names[i]);
                for (int f = 1; f <= ocam::cad::subShapeCount(m, ocam::cad::SubShapeType::Face);
                     ++f) {
                    const auto g = ocam::cad::datumGeometry(m, ocam::cad::SubShapeType::Face, f);
                    if (g.shape == ocam::cad::DatumGeometry::Shape::Plane &&
                        g.direction.isApprox(n, 1e-6)) {
                        return f;
                    }
                }
                return 0;
            };
            // The smallest round face of part `i` (a chuck body's bore).
            auto bore = [&](std::size_t i) {
                const auto& m = *wizard.partModel(names[i]);
                int best = 0;
                double radius = 1e300;
                for (int f = 1; f <= ocam::cad::subShapeCount(m, ocam::cad::SubShapeType::Face);
                     ++f) {
                    const auto g = ocam::cad::subShapeGeometry(m, ocam::cad::SubShapeType::Face, f);
                    if (g.kind == ocam::cad::SubShapeGeometry::Kind::Cylinder &&
                        g.radius < radius) {
                        best = f;
                        radius = g.radius;
                    }
                }
                return best;
            };
            bool ok = true;
            auto clickFace = [&](std::size_t i, int f) {
                ok = ok && wizard.clickShape(names[i], ocam::cad::SubShapeType::Face, f) &&
                     wizard.next();
            };
            if (chuck) {
                wizard.setType(ocam::document::FixtureType::Chuck);
                ok = wizard.next(); // the kind
                for (std::size_t i = 1; i < 4; ++i) {
                    ok = ok && wizard.clickPart(names[i]);
                }
                ok = ok && wizard.next();
                clickFace(0, bore(0));                       // the spindle axis
                clickFace(1, face(1, -ocam::Vec3::UnitX())); // the first jaw's gripping face
                clickFace(1, face(1, ocam::Vec3::UnitZ()));  // the jaw front
            } else {
                wizard.setType(ocam::document::FixtureType::SelfCentringVise);
                ok = wizard.next();                                     // the kind
                ok = ok && wizard.clickPart(names[1]) && wizard.next(); // first jaw
                ok = ok && wizard.clickPart(names[2]) && wizard.next(); // second jaw
                clickFace(1, face(1, ocam::Vec3::UnitY()));
                clickFace(2, face(2, -ocam::Vec3::UnitY()));
                clickFace(2, face(2, ocam::Vec3::UnitZ()));
            }
            ok = ok && wizard.next(); // what it stands on: the bottom centre
            if (!ok) {
                std::fprintf(stderr, "the wizard stopped at '%s': %s\n",
                             wizard.stepTitle().toUtf8().constData(),
                             wizard.problemText().toUtf8().constData());
                return 5;
            }
            wizard.setTrialStock(chuck ? ocam::Vec3(32, 32, 80) : ocam::Vec3(70, 80, 30));
            int code = 0;
            QTimer::singleShot(500, &app, [&] {
                code = wizard.snapshot().save(parser.value(screenshotWizard)) ? 0 : 4;
                if (code == 0 && !wizard.finish()) {
                    std::fprintf(stderr, "finish failed: %s\n",
                                 wizard.problemText().toUtf8().constData());
                    code = 6;
                }
                const auto& w = wizard.result().workholding;
                if (code == 0 &&
                    (w.members.size() != (chuck ? 3u : 2u) || w.members[1].linkedTo.empty() ||
                     w.parts.size() != (chuck ? 4u : 3u) ||
                     wizard.result().files.size() != w.parts.size())) {
                    std::fprintf(stderr, "the jaws are not linked, or the parts are wrong\n");
                    code = 7;
                }
                QApplication::exit(code);
            });
            QApplication::exec();
            return code;
        } catch (const ocam::Error& e) {
            std::fprintf(stderr, "%s\n", e.what());
            return 3;
        }
    }
    if (parser.isSet(workholdingUpTest)) {
        namespace doc = ocam::document;
        using ocam::Vec3;
        if (!parser.isSet(workholdingFolder) || !parser.isSet(workholdingStep)) {
            // It empties the library folder: never the user's.
            std::fprintf(stderr, "--workholding-up-test needs --workholding-folder and "
                                 "--workholding-step\n");
            return 2;
        }
        try {
            ocam::CancelToken cancel;
            ocam::NullProgress progress;
            doc::WorkholdingPackage start;
            start.workholding.name = "Y-up vise";
            std::map<std::string, ocam::cad::Model> models;
            std::vector<std::string> names;
            for (auto& body : ocam::cad::importStepAssembly(
                     std::filesystem::path(parser.value(workholdingStep).toStdU16String()), cancel,
                     progress)) {
                const doc::WorkholdingPart part =
                    doc::addWorkholdingPart(start, body.name, body.model);
                models.emplace(part.sha256,
                               ocam::cad::Model::importStepData(start.files.at(part.sha256),
                                                                body.name, cancel, progress));
                names.push_back(body.name);
            }
            if (names.size() != 3) {
                std::fprintf(stderr, "expected a base and two jaws\n");
                return 3;
            }
            int failed = 0;
            const auto check = [&](const char* what, bool ok) {
                std::printf("%s: %s\n", what, ok ? "ok" : "WRONG");
                failed += ok ? 0 : 1;
            };
            // The face of a part whose outward normal is `n` (exact geometry).
            auto face = [&](const ocam::cad::Model& m, const Vec3& n) {
                for (int f = 1; f <= ocam::cad::subShapeCount(m, ocam::cad::SubShapeType::Face);
                     ++f) {
                    const auto g = ocam::cad::datumGeometry(m, ocam::cad::SubShapeType::Face, f);
                    if (g.shape == ocam::cad::DatumGeometry::Shape::Plane &&
                        g.direction.isApprox(n, 1e-6)) {
                        return f;
                    }
                }
                return 0;
            };
            doc::WorkholdingPackage built;
            {
                ocam::gui::WorkholdingWizard wizard(start, models);
                wizard.show();
                wizard.setType(doc::FixtureType::SelfCentringVise);
                bool ok = wizard.next();
                ok = ok && wizard.clickPart(names[1]) && wizard.next();
                ok = ok && wizard.clickPart(names[2]) && wizard.next();
                auto click = [&](std::size_t i, const Vec3& n) {
                    ok = ok &&
                         wizard.clickShape(names[i], ocam::cad::SubShapeType::Face,
                                           face(*wizard.partModel(names[i]), n)) &&
                         wizard.next();
                };
                click(1, -Vec3::UnitZ()); // the first jaw's clamping face
                click(2, Vec3::UnitZ());  // the second jaw's
                click(2, Vec3::UnitY());  // the jaw tops: the model's +Y
                ok = ok && wizard.next() && wizard.finish();
                if (!ok) {
                    std::fprintf(stderr, "the wizard stopped at '%s': %s\n",
                                 wizard.stepTitle().toUtf8().constData(),
                                 wizard.problemText().toUtf8().constData());
                    return 5;
                }
                built = wizard.result();
            }
            check("the wizard's up is the jaw tops' direction, the model's +Y",
                  built.workholding.up.isApprox(Vec3::UnitY(), 1e-9) &&
                      !doc::upMismatch(built.workholding));
            // Saved upright on screen but with up +X, as the user had it.
            built.workholding.up = Vec3::UnitX();
            std::filesystem::remove_all(ocam::gui::WorkholdingLibrary::folder());
            std::filesystem::create_directories(ocam::gui::WorkholdingLibrary::folder());
            ocam::gui::WorkholdingLibrary::save(built);
            ocam::gui::WorkholdingLibraryDialog dialog;
            dialog.show();
            dialog.selectTemplate(built.workholding.id);
            QApplication::processEvents();
            const QString warning = dialog.upWarningText();
            std::printf("warning: %s\n", warning.toUtf8().constData());
            check("up +X is warned about against the jaw tops (+Y)",
                  warning.contains(QStringLiteral("Model +X")) &&
                      warning.contains(QStringLiteral("Model +Y")));
            auto* up = dialog.findChild<QComboBox*>(QStringLiteral("workholdingUp"));
            check("the up field reads as the model's axis",
                  up && up->currentText() == QStringLiteral("Model +X") && up->isVisible());
            const QString png = parser.value(workholdingUpTest);
            int code = 0;
            QTimer::singleShot(500, &app, [&] {
                // The warning and the sideways arrow.
                const bool saved = dialog.snapshot().save(png);
                check("use the jaw tops' direction",
                      dialog.useSeatUp() &&
                          dialog.editedPackage().workholding.up.isApprox(Vec3::UnitY(), 1e-9) &&
                          dialog.upWarningText().isEmpty());
                if (up) {
                    up->setCurrentIndex(0); // +X again, by hand
                }
                check("the warning is back for +X", !dialog.upWarningText().isEmpty());
                const auto& jaw = built.workholding.parts[2];
                const QString note = dialog.pickUp(jaw.name, ocam::cad::SubShapeType::Face,
                                                   face(models.at(jaw.sha256), Vec3::UnitY()));
                std::printf("picked: %s\n", note.toUtf8().constData());
                check("picking the jaw-top face sets the model's +Y",
                      dialog.editedPackage().workholding.up.isApprox(Vec3::UnitY(), 1e-9) &&
                          dialog.upWarningText().isEmpty());
                QString picked = png;
                picked.replace(QStringLiteral(".png"), QStringLiteral("_picked.png"));
                QTimer::singleShot(300, &app, [&, saved, picked] {
                    const bool savedPicked = dialog.snapshot().save(picked);
                    if (failed == 0 && saved && savedPicked) {
                        std::printf("workholding up test: all ok\n");
                    }
                    std::fflush(stdout);
                    code = failed == 0 && saved && savedPicked ? 0 : 6;
                    QApplication::exit(code);
                });
            });
            QApplication::exec();
            return code;
        } catch (const ocam::Error& e) {
            std::fprintf(stderr, "%s\n", e.what());
            return 3;
        }
    }
    if (parser.isSet(workholdingJawsTest)) {
        namespace doc = ocam::document;
        try {
            const std::filesystem::path folder(parser.value(workholdingFolder).toStdU16String());
            doc::WorkholdingPackage chuck = doc::loadWorkholdingFile(folder / "chuck.ocamhold");
            const auto [hard, soft] = testJawSets(chuck.workholding);
            doc::saveWorkholdingFile(folder / "chuck.ocamhold", chuck);
            doc::WorkholdingPackage jaws;
            jaws.workholding = hard;
            jaws.files = {*chuck.files.find(hard.parts.front().sha256)};
            doc::saveWorkholdingFile(folder / "jaws.ocamhold", jaws);
            jaws.workholding = soft;
            doc::saveWorkholdingFile(folder / "soft-jaws.ocamhold", jaws);
            ocam::gui::WorkholdingLibraryDialog dialog;
            dialog.show();
            dialog.showAdvanced(true);
            int failed = 0;
            const auto check = [&](const char* what, bool ok) {
                std::printf("%s: %s\n", what, ok ? "ok" : "WRONG");
                failed += ok ? 0 : 1;
            };
            dialog.selectTemplate(hard.id);
            QApplication::processEvents();
            check("jaw set read and given back",
                  dialog.editedPackage().workholding.jawSet == hard.jawSet);
            // Its two options: the second shown, then a third added from it.
            auto* options = dialog.findChild<QComboBox*>(QStringLiteral("workholdingJawOption"));
            auto* optionName =
                dialog.findChild<QLineEdit*>(QStringLiteral("workholdingJawOptionName"));
            if (options && options->count() == 2) {
                options->setCurrentIndex(1);
                QApplication::processEvents();
            }
            check("second option shown, both given back",
                  optionName && optionName->text() == QStringLiteral("Reversed (ID)") &&
                      dialog.editedPackage().workholding.jawSet == hard.jawSet);
            if (auto* add =
                    dialog.findChild<QPushButton*>(QStringLiteral("workholdingJawAddOption"))) {
                add->click();
                QApplication::processEvents();
            }
            const auto added = dialog.editedPackage().workholding.jawSet;
            check("an option added as a copy of the shown one",
                  added && added->options.size() == 3 &&
                      added->options[2].inward == hard.jawSet->options[1].inward &&
                      added->options[2].steps == hard.jawSet->options[1].steps);
            // Leaving it asks whether to save the change: Discard, as a user would.
            QTimer::singleShot(200, &app, [] {
                if (auto* box = qobject_cast<QMessageBox*>(QApplication::activeModalWidget())) {
                    if (QAbstractButton* discard = box->button(QMessageBox::Discard)) {
                        std::printf("asked to save the change: Discard\n");
                        discard->click();
                    }
                }
            });
            dialog.selectTemplate(soft.id);
            QApplication::processEvents();
            auto* kind = dialog.findChild<QComboBox*>(QStringLiteral("workholdingJawKind"));
            check("soft jaw set read as soft",
                  kind && kind->currentIndex() == 1 &&
                      dialog.editedPackage().workholding.jawSet == soft.jawSet);
            dialog.selectTemplate(chuck.workholding.id);
            QApplication::processEvents();
            const doc::WorkholdingTemplate edited = dialog.editedPackage().workholding;
            check("jaw mount read and given back", edited.jawMount == chuck.workholding.jawMount);
            bool tops = edited.parts.size() == chuck.workholding.parts.size();
            for (std::size_t i = 0; tops && i < edited.parts.size(); ++i) {
                tops = edited.parts[i].topJaw == chuck.workholding.parts[i].topJaw;
            }
            check("top jaws kept", tops);
            if (auto* list = dialog.findChild<QListWidget*>(QStringLiteral("workholdingTopJaws"));
                list && list->count() > 1) {
                list->item(1)->setCheckState(Qt::Unchecked);
            }
            check("a top jaw unticked", !dialog.editedPackage().workholding.parts[1].topJaw);
            std::fflush(stdout);
            QTimer::singleShot(500, &app, [&] {
                const bool saved = dialog.snapshot().save(parser.value(workholdingJawsTest));
                QApplication::exit(failed == 0 && saved ? 0 : 6);
            });
            return QApplication::exec();
        } catch (const ocam::Error& e) {
            std::fprintf(stderr, "%s\n", e.what());
            return 3;
        }
    }
    if (parser.isSet(screenshotWorkholding)) {
        ocam::gui::WorkholdingLibraryDialog dialog;
        dialog.show();
        if (parser.isSet(workholdingStep)) {
            // Every body of the file a part, as the import would give them, and a datum picked.
            try {
                ocam::CancelToken cancel;
                ocam::NullProgress progress;
                std::vector<ocam::document::SolidPart> parts;
                for (auto& body : ocam::cad::importStepAssembly(
                         std::filesystem::path(parser.value(workholdingStep).toStdU16String()),
                         cancel, progress)) {
                    parts.push_back({body.name, body.model, body.colour});
                }
                if (parts.empty()) {
                    std::fprintf(stderr, "no bodies\n");
                    return 3;
                }
                const std::string first = parts.front().name;
                dialog.newFromParts("From STEP", std::move(parts));
                if (dialog.pickDatum(first, ocam::cad::SubShapeType::Face, 1).empty()) {
                    std::fprintf(stderr, "no datum picked\n");
                    return 5;
                }
            } catch (const ocam::Error& e) {
                std::fprintf(stderr, "%s\n", e.what());
                return 3;
            }
        }
        QTimer::singleShot(500, &app, [&] {
            QApplication::exit(dialog.snapshot().save(parser.value(screenshotWorkholding)) ? 0 : 4);
        });
        return QApplication::exec();
    }

    if (parser.isSet(screenshotPartsImport)) {
        if (parser.positionalArguments().isEmpty()) {
            std::fprintf(stderr, "--screenshot-parts-import requires a STEP or STL file\n");
            return 2;
        }
        try {
            ocam::CancelToken cancel;
            ocam::NullProgress progress;
            auto parts = ocam::document::readMachinePartFile(
                std::filesystem::path(parser.positionalArguments().front().toStdU16String()), 1.0,
                cancel, progress);
            ocam::gui::MachinePartsImportDialog dialog(std::move(parts));
            if (parser.isSet(mergeLevel)) {
                dialog.setLevel(parser.value(mergeLevel).toInt());
            }
            dialog.show();
            QTimer::singleShot(500, &app, [&] {
                QApplication::exit(dialog.grab().save(parser.value(screenshotPartsImport)) ? 0 : 4);
            });
            return QApplication::exec();
        } catch (const ocam::Error& e) {
            std::fprintf(stderr, "%s\n", e.what());
            return 3;
        }
    }

    const auto keepAside = [&](QWidget& widget) {
        if (parser.isSet(quietWindows)) {
            widget.setAttribute(Qt::WA_ShowWithoutActivating);
            widget.move(-20000, -20000);
        }
    };
    // Opens the setup's turret window as a user does: the setup page's turret section, then its
    // Turret setup... button (with --quiet-windows, off the screen). Null if it did not open.
    const auto openTurretWindow = [&](QWidget& main) -> ocam::gui::TurretWindow* {
        if (auto* head = main.findChild<QToolButton*>(QStringLiteral("sectionHead_turret"))) {
            if (!head->isChecked()) {
                head->click();
            }
            QApplication::processEvents();
        }
        auto* turret = main.findChild<ocam::gui::TurretWindow*>();
        auto* open = main.findChild<QPushButton*>(QStringLiteral("setupTurretWindow"));
        if (!turret || !open) {
            return nullptr;
        }
        if (!turret->isVisible()) {
            keepAside(*turret);
        }
        open->click();
        for (int i = 0; i < 20; ++i) {
            QApplication::processEvents();
            QThread::msleep(5);
        }
        return turret->isVisible() ? turret : nullptr;
    };
    if (parser.isSet(screenshotFittings)) {
        ocam::gui::LatheFittingsDialog dialog;
        keepAside(dialog);
        dialog.show();
        if (parser.isSet(fittingName) && !dialog.selectByName(parser.value(fittingName))) {
            std::fprintf(stderr, "no such fitting\n");
            return 3;
        }
        if (parser.isSet(fittingOnLathe)) {
            if (auto* on = dialog.findChild<QCheckBox*>(QStringLiteral("fittingOnLathe"))) {
                on->setChecked(true);
            }
        }
        for (int i = 0; i < 50; ++i) {
            QApplication::processEvents();
            QThread::msleep(10);
        }
        return dialog.snapshot().save(parser.value(screenshotFittings)) ? 0 : 4;
    }
    if (parser.isSet(showBuiltinsTest)) {
        // Every dialog listing OxCAM's built-in examples next to the user's own: one Show
        // built-ins box, off at first; ticked, they are listed; one opened on a built-in ticks it.
        // Its own libraries, in throwaway folders (the test has a settings folder of its own).
        namespace doc = ocam::document;
        const QTemporaryDir machines;
        const QTemporaryDir fittings;
        const QTemporaryDir posts;
        {
            QSettings settings;
            settings.setValue(QStringLiteral("machineLibraryFolder"), machines.path());
            settings.setValue(QStringLiteral("postLibraryFolder"), posts.path());
        }
        ocam::gui::LatheFittingLibrary::setFolderOverride(
            std::filesystem::path(fittings.path().toStdU16String()));
        const ocam::machine::Machine* example = ocam::machine::builtinMachines().front();
        ocam::machine::Machine mine = *example;
        mine.id = "my-test-machine";
        mine.name = "My machine (test)";
        ocam::gui::MachineLibrary::save(mine);
        doc::FittingPackage block = doc::builtinFittings().front();
        block.builtin = false;
        block.fitting.id = ocam::Uuid::generate();
        block.fitting.name = "My block (test)";
        doc::saveFitting(std::filesystem::path(fittings.path().toStdU16String()), block);

        int failed = 0;
        const auto check = [&](const char* what, bool ok) {
            std::printf("%s: %s\n", what, ok ? "ok" : "WRONG");
            std::fflush(stdout);
            failed += ok ? 0 : 1;
        };
        const auto settle = [] {
            for (int i = 0; i < 5; ++i) {
                QApplication::processEvents();
            }
        };
        const auto shown = [](QListWidget* list) {
            QStringList rows;
            for (int i = 0; list && i < list->count(); ++i) {
                if (!list->item(i)->isHidden()) {
                    rows << list->item(i)->text();
                }
            }
            return rows;
        };
        const auto box = [](QWidget& dialog) {
            return dialog.findChild<QCheckBox*>(QStringLiteral("showBuiltins"));
        };
        {
            ocam::gui::MachineLibraryDialog dialog;
            keepAside(dialog);
            dialog.show();
            settle();
            auto* list = dialog.findChild<QListWidget*>(QStringLiteral("machineList"));
            std::printf("machines: %s\n", shown(list).join(QStringLiteral(" | ")).toUtf8().data());
            check("machines: the user's own only, the box off",
                  box(dialog) && box(dialog)->text() == QStringLiteral("Show built-ins") &&
                      !box(dialog)->isChecked() &&
                      shown(list) == QStringList{QStringLiteral("My machine (test)")} &&
                      list->currentItem() &&
                      list->currentItem()->text() == QStringLiteral("My machine (test)"));
            box(dialog)->click();
            settle();
            check("machines: ticked, the built-ins too",
                  shown(list).size() ==
                      static_cast<qsizetype>(ocam::machine::builtinMachines().size() + 1));
            box(dialog)->click();
            settle();
            check("machines: unticked, the user's own again", shown(list).size() == 1 &&
                                                                  list->currentItem() &&
                                                                  !list->currentItem()->isHidden());
            dialog.selectMachine(example->id);
            settle();
            check(
                "machines: opened on a built-in, it is shown",
                box(dialog)->isChecked() && list->currentItem() &&
                    list->currentItem()->text().startsWith(QString::fromStdString(example->name)));
        }
        {
            ocam::gui::LatheFittingsDialog dialog;
            keepAside(dialog);
            dialog.show();
            settle();
            auto* list = dialog.findChild<QListWidget*>(QStringLiteral("fittingList"));
            std::printf("fittings: %s\n", shown(list).join(QStringLiteral(" | ")).toUtf8().data());
            check("fittings: the user's own only, the box off",
                  box(dialog) && !box(dialog)->isChecked() &&
                      shown(list) == QStringList{QStringLiteral("My block (test)")} &&
                      dialog.current() && dialog.current()->name == "My block (test)");
            box(dialog)->click();
            settle();
            check("fittings: ticked, the built-ins too",
                  shown(list).size() == static_cast<qsizetype>(doc::builtinFittings().size() + 1));
            box(dialog)->click();
            settle();
            const QString builtinName =
                QString::fromStdString(doc::builtinFittings().front().fitting.name);
            check("fittings: opened on a built-in, it is shown",
                  shown(list).size() == 1 && dialog.selectByName(builtinName) &&
                      box(dialog)->isChecked() && dialog.current() &&
                      QString::fromStdString(dialog.current()->name) == builtinName);
        }
        {
            const doc::Document project = doc::Document::open(
                std::filesystem::path(parser.value(showBuiltinsTest).toStdU16String()));
            ocam::gui::PostProcessDialog dialog(project.project(),
                                                project.project().setups.front().id,
                                                ocam::gui::PostProcessDialog::Mode::Configure);
            keepAside(dialog);
            dialog.show();
            settle();
            auto* choice = dialog.findChild<QComboBox*>(QStringLiteral("postChoice"));
            const auto items = [&] {
                QStringList all;
                for (int i = 0; choice && i < choice->count(); ++i) {
                    // Entries, not the separators and headings between their groups.
                    if (!choice->itemText(i).isEmpty() && !ocam::gui::isListHeading(choice, i)) {
                        all << choice->itemText(i);
                    }
                }
                return all;
            };
            std::printf("posts: %s\n", items().join(QStringLiteral(" | ")).toUtf8().data());
            const qsizetype hidden = items().size();
            check("posts: the machine's default and adding a file, the box off",
                  box(dialog) && !box(dialog)->isChecked() && hidden == 2 &&
                      items().front().startsWith(QStringLiteral("The machine's default")));
            box(dialog)->click();
            settle();
            std::printf("posts shown: %s\n", items().join(QStringLiteral(" | ")).toUtf8().data());
            check("posts: ticked, the built-ins too",
                  items().size() > hidden && choice->currentIndex() == 0);
            // A built-in chosen stays listed when the box is unticked.
            choice->setCurrentIndex(choice->findText(items().at(1)));
            settle();
            const QString chosen = choice->currentText();
            box(dialog)->click();
            settle();
            check("posts: a built-in chosen stays listed",
                  choice->currentText() == chosen && items().size() == hidden + 1);
            if (parser.isSet(screenshotWindow)) {
                box(dialog)->click();
                settle();
                dialog.grab().save(parser.value(screenshotWindow));
            }
        }
        std::printf("show built-ins test: %s\n", failed == 0 ? "all ok" : "WRONG");
        std::fflush(stdout);
        ocam::gui::LatheFittingLibrary::setFolderOverride({});
        return failed == 0 ? 0 : 6;
    }
    if (parser.isSet(fittingsEditTest)) {
        namespace cad = ocam::cad;
        using Kind = cad::SubShapeGeometry::Kind;
        const QTemporaryDir folder;
        ocam::gui::LatheFittingLibrary::setFolderOverride(
            std::filesystem::path(folder.path().toStdU16String()));
        const std::filesystem::path file(parser.value(fittingsEditTest).toStdU16String());
        const cad::Model model = cad::Model::importStep(file);
        ocam::gui::LatheFittingsDialog dialog;
        keepAside(dialog);
        dialog.show();
        dialog.addFromFile(file);
        const ocam::document::LatheFitting* f = dialog.current();
        if (!f || f->sha256.empty()) {
            std::fprintf(stderr, "new: no fitting from the file\n");
            return 3;
        }
        std::printf("new: %s\n", f->name.c_str());

        int failed = 0;
        const auto check = [&](const char* what, bool held,
                               const ocam::document::FittingPoint& at) {
            std::printf("%s: %s at %.3f %.3f %.3f dir %.3f %.3f %.3f (%s)\n", what,
                        held ? "ok" : "WRONG", at.point.x(), at.point.y(), at.point.z(),
                        at.direction.x(), at.direction.y(), at.direction.z(), at.on.c_str());
            failed += held ? 0 : 1;
        };
        const auto same = [](const ocam::Vec3& a, const ocam::Vec3& b) {
            return (a - b).norm() < 1e-6;
        };
        const auto firstOf = [&](cad::SubShapeType type, Kind kind) {
            for (int i = 1; i <= cad::subShapeCount(model, type); ++i) {
                if (cad::subShapeGeometry(model, type, i).kind == kind) {
                    return i;
                }
            }
            return 0;
        };

        const int flat = firstOf(cad::SubShapeType::Face, Kind::Plane);
        const cad::SubShapeGeometry plane =
            cad::subShapeGeometry(model, cad::SubShapeType::Face, flat);
        dialog.pickForTest(true, static_cast<int>(cad::SubShapeType::Face), flat);
        check("seat on a face",
              same(f->seat.point, plane.point) && same(f->seat.direction, -plane.direction),
              f->seat);
        dialog.pickForTest(false, static_cast<int>(cad::SubShapeType::Face), flat);
        check("mount on a face",
              same(f->mount.point, plane.point) && same(f->mount.direction, plane.direction),
              f->mount);

        // An axis has no sense: it goes away from the middle of the fitting.
        const ocam::geom3d::Aabb3 box = model.info().bounds;
        const auto outwards = [&](const ocam::document::FittingPoint& at) {
            return at.direction.dot(at.point - 0.5 * (box.min + box.max)) >= 0.0;
        };
        const int round = firstOf(cad::SubShapeType::Edge, Kind::Circle);
        if (round > 0) {
            const cad::SubShapeGeometry circle =
                cad::subShapeGeometry(model, cad::SubShapeType::Edge, round);
            dialog.pickForTest(false, static_cast<int>(cad::SubShapeType::Edge), round);
            check("mount on a circular edge",
                  same(f->mount.point, circle.point) && outwards(f->mount) &&
                      std::abs(f->mount.direction.dot(circle.direction)) > 1.0 - 1e-6,
                  f->mount);
        } else {
            std::printf("mount on a circular edge: no such edge in this model\n");
        }
        // A round face (the bore): the centre of an end, not its middle. With no click, the end
        // away from the fitting's middle; with one, the end nearest it.
        const int bore = firstOf(cad::SubShapeType::Face, Kind::Cylinder);
        if (bore > 0) {
            const cad::SubShapeGeometry g =
                cad::subShapeGeometry(model, cad::SubShapeType::Face, bore);
            dialog.pickForTest(false, static_cast<int>(cad::SubShapeType::Face), bore);
            check("mount on a round face, at its outer end",
                  (same(f->mount.point, g.axisStart) || same(f->mount.point, g.axisEnd)) &&
                      outwards(f->mount) && !same(g.axisStart, g.axisEnd),
                  f->mount);
            const ocam::Vec3 outer = f->mount.point;
            const ocam::Vec3 other = same(outer, g.axisStart) ? g.axisEnd : g.axisStart;
            dialog.pickForTest(false, static_cast<int>(cad::SubShapeType::Face), bore,
                               other + 0.1 * (outer - other));
            check("mount on a round face, at the end clicked near", same(f->mount.point, other),
                  f->mount);
        } else {
            std::printf("mount on a round face: no such face in this model\n");
        }
        // A vertex says where, and leaves the direction.
        const ocam::Vec3 kept = f->seat.direction;
        const cad::SubShapeGeometry vertex =
            cad::subShapeGeometry(model, cad::SubShapeType::Vertex, 1);
        dialog.pickForTest(true, static_cast<int>(cad::SubShapeType::Vertex), 1);
        check("seat on a vertex",
              same(f->seat.point, vertex.point) && same(f->seat.direction, kept), f->seat);

        // A real click: the button, then the mouse on the preview, whatever is at its middle.
        auto* pick = dialog.findChild<QPushButton*>(QStringLiteral("fittingPickSeat"));
        auto* view = dialog.findChild<ocam::gui::OcctView*>();
        const auto settle = [] {
            for (int i = 0; i < 50; ++i) {
                QApplication::processEvents();
                QThread::msleep(10);
            }
        };
        settle();
        if (!pick || !view) {
            std::fprintf(stderr, "click: no button or preview\n");
            return 3;
        }
        pick->click();
        const QPointF middle(view->width() / 2.0, view->height() / 2.0);
        const QPointF global = view->mapToGlobal(middle);
        QMouseEvent move(QEvent::MouseMove, middle, global, Qt::NoButton, Qt::NoButton,
                         Qt::NoModifier);
        QApplication::sendEvent(view, &move);
        settle();
        QMouseEvent press(QEvent::MouseButtonPress, middle, global, Qt::LeftButton, Qt::LeftButton,
                          Qt::NoModifier);
        QApplication::sendEvent(view, &press);
        QMouseEvent release(QEvent::MouseButtonRelease, middle, global, Qt::LeftButton,
                            Qt::NoButton, Qt::NoModifier);
        QApplication::sendEvent(view, &release);
        settle();
        // The button lets go when a click has reached the fitting.
        check("seat by a click", !pick->isChecked() && !same(f->seat.point, vertex.point), f->seat);

        // Picking is on the fitting alone: it leaves the view on a lathe.
        auto* onLathe = dialog.findChild<QCheckBox*>(QStringLiteral("fittingOnLathe"));
        if (!onLathe) {
            std::fprintf(stderr, "lathe view: no check box\n");
            return 3;
        }
        onLathe->setChecked(true);
        settle();
        pick->click();
        const bool leftLathe = !onLathe->isChecked();
        pick->click(); // and stop picking
        std::printf("picking leaves the lathe view: %s\n", leftLathe ? "ok" : "WRONG");
        failed += leftLathe ? 0 : 1;

        // What it takes and how it mounts, through the fields.
        auto* mountType = dialog.findChild<QComboBox*>(QStringLiteral("fittingMountType"));
        auto* takes = dialog.findChild<QComboBox*>(QStringLiteral("fittingTakes"));
        auto* shankStated = dialog.findChild<QCheckBox*>(QStringLiteral("fittingShankStated"));
        if (!mountType || !takes || !shankStated) {
            std::fprintf(stderr, "takes: no fields\n");
            return 3;
        }
        mountType->setEditText(QStringLiteral("  VDI   40 "));
        takes->setCurrentIndex(2);
        takes->activated(2);
        shankStated->setChecked(true);
        settle();
        const bool fields = f->mountType == "VDI 40" &&
                            f->takes == ocam::document::LatheFitting::Takes::Both &&
                            f->squareShank.has_value() && !f->roundBar.has_value();
        std::printf("takes both, square stated, mounts by '%s': %s\n", f->mountType.c_str(),
                    fields ? "ok" : "WRONG");
        failed += fields ? 0 : 1;

        QString why;
        const bool saved = dialog.saveCurrent(&why);
        const std::vector<ocam::document::FittingPackage> back =
            ocam::gui::LatheFittingLibrary::load();
        const auto found = std::find_if(back.begin(), back.end(),
                                        [&](const auto& one) { return one.fitting.id == f->id; });
        const bool held =
            saved && found != back.end() && same(found->fitting.seat.point, f->seat.point) &&
            same(found->fitting.seat.direction, f->seat.direction) &&
            same(found->fitting.mount.point, f->mount.point) &&
            same(found->fitting.mount.direction, f->mount.direction) &&
            found->fitting.mountType == f->mountType && found->fitting.takes == f->takes &&
            found->fitting.squareShank == f->squareShank && found->fitting.roundBar == f->roundBar;
        std::printf("saved and loaded back: %s %s\n", held ? "ok" : "WRONG",
                    why.toUtf8().constData());
        failed += held ? 0 : 1;
        std::fflush(stdout);
        ocam::gui::LatheFittingLibrary::setFolderOverride({});
        return failed == 0 ? 0 : 4;
    }
    if (parser.isSet(screenshotHub) || parser.isSet(screenshotMachines)) {
        std::unique_ptr<QDialog> dialog;
        ocam::gui::MachineLibraryDialog* machines = nullptr;
        QString target;
        if (parser.isSet(screenshotHub)) {
            dialog = std::make_unique<ocam::gui::StartHub>();
            target = parser.value(screenshotHub);
        } else {
            auto library = std::make_unique<ocam::gui::MachineLibraryDialog>();
            library->selectMachine(parser.isSet(machineId)
                                       ? parser.value(machineId).toStdString()
                                       : std::string("generic-5axis-trunnion-ac"));
            machines = library.get();
            dialog = std::move(library);
            target = parser.value(screenshotMachines);
        }
        if (machines && parser.isSet(maximized)) {
            dialog->showMaximized();
        } else {
            dialog->show();
        }
        QTimer::singleShot(800, &app, [&] {
            if (machines) {
                if (parser.isSet(showModel)) {
                    machines->showModelSection();
                }
                for (const QString& j : parser.values(jog)) {
                    const QStringList kv = j.split(QLatin1Char('='));
                    if (kv.size() == 2) {
                        machines->jogPreview(kv[0].trimmed().toUpper().toStdString(),
                                             kv[1].toDouble());
                    }
                }
            }
            if (machines && parser.isSet(selectPart)) {
                machines->selectPartByName(parser.value(selectPart).toStdString());
                if (parser.isSet(isolatePart)) {
                    machines->isolateSelected();
                }
            }
            QTimer::singleShot(300, &app, [&] {
                int code = 0;
                if (machines && parser.isSet(pickGauge)) {
                    const QStringList points = parser.value(pickGauge).split(QLatin1Char(';'));
                    auto point = [&](int i) {
                        const QStringList xyz = points.value(i).split(QLatin1Char(','));
                        return ocam::Vec3(xyz.value(0).toDouble(), xyz.value(1).toDouble(),
                                          xyz.value(2).toDouble());
                    };
                    const std::string result = machines->pickGaugeAt(point(0), point(1));
                    std::printf("gauge pick: %s\n", result.c_str());
                    std::fflush(stdout);
                }
                if (machines && parser.isSet(pickAxis)) {
                    const QStringList spec = parser.value(pickAxis).split(QLatin1Char('@'));
                    const QStringList xyz = spec.value(1).split(QLatin1Char(','));
                    const std::string result = machines->pickAxisAt(
                        spec.value(0).trimmed().toUpper().toStdString(),
                        ocam::Vec3(xyz.value(0).toDouble(), xyz.value(1).toDouble(),
                                   xyz.value(2).toDouble()));
                    std::printf("axis pick: %s\n", result.c_str());
                    std::fflush(stdout);
                }
                if (machines && parser.isSet(machineStands)) {
                    const QStringList sides = parser.value(machineStands).split(QLatin1Char(','));
                    std::printf("before: %s\n", machines->standingReport().c_str());
                    machines->chooseStanding(sides.value(0), sides.value(1));
                    std::printf("after: %s\n", machines->standingReport().c_str());
                    std::fflush(stdout);
                }
                if (machines && parser.isSet(pickPreview)) {
                    const QSize size = machines->previewSize();
                    const std::string picked =
                        machines->pickPreview(QPoint(size.width() / 2, size.height() / 2));
                    std::printf("picked: %s\n", picked.c_str());
                    std::fflush(stdout);
                    if (picked.empty() || (parser.isSet(selectPart) &&
                                           picked != parser.value(selectPart).toStdString())) {
                        code = 5;
                    }
                }
                const QImage image = machines ? machines->snapshot() : dialog->grab().toImage();
                QApplication::exit(image.save(target) ? code : 4);
            });
        });
        return QApplication::exec();
    }

    ocam::gui::MainWindow window;
    if (parser.isSet(listCommands)) {
        for (const ocam::gui::Command& c : window.commands().commands()) {
            std::printf("%s\t%s\n", c.id.toUtf8().constData(),
                        QKeySequence::listToString(window.commands().shortcutsOf(c),
                                                   QKeySequence::PortableText)
                            .toUtf8()
                            .constData());
        }
        return 0;
    }
    if (parser.isSet(pressShortcut)) {
        // Presses a key three times as the window's shortcuts take it, printing the toolpath
        // display after each press (a key the toolpath modes share steps through them).
        const QKeySequence key =
            QKeySequence::fromString(parser.value(pressShortcut), QKeySequence::PortableText);
        for (int press = 0; press < 3; ++press) {
            bool pressed = false;
            for (QAction* a : window.actions()) {
                if (a->shortcuts().contains(key) && a->isEnabled()) {
                    a->trigger();
                    pressed = true;
                    break;
                }
            }
            if (!pressed) {
                std::fprintf(stderr, "no action takes the key\n");
                return 7;
            }
            for (const ocam::gui::Command& c : window.commands().commands()) {
                if (c.optionGroup == QStringLiteral("toolpaths") && c.action->isChecked()) {
                    std::printf("%s\n", c.id.toUtf8().constData());
                }
            }
        }
        return 0;
    }
    if (parser.isSet(screenshotToolTip)) {
        // The tooltip of a command (its toolbar button, its menu entry), as it shows.
        keepAside(window);
        window.show();
        const ocam::gui::Command* c = window.commands().find(parser.value(toolTipCommand));
        if (!c) {
            std::fprintf(stderr, "unknown command\n");
            return 7;
        }
        // Once the window is on screen. Another window taking the focus hides a tooltip (tests
        // run side by side): show it again until it stays, for a few seconds at most.
        int attempts = 0;
        auto* poll = new QTimer(&app);
        QObject::connect(poll, &QTimer::timeout, &app, [&, c, poll] {
            for (QWidget* w : QApplication::topLevelWidgets()) {
                if (w->objectName() == QStringLiteral("qtooltip_label") && w->isVisible()) {
                    poll->stop();
                    QApplication::exit(w->grab().save(parser.value(screenshotToolTip)) ? 0 : 4);
                    return;
                }
            }
            if (++attempts > 20) {
                poll->stop();
                std::fprintf(stderr, "no tooltip shown\n");
                QApplication::exit(5);
                return;
            }
            window.activateWindow();
            QToolTip::showText(window.mapToGlobal(QPoint(100, 100)), c->action->toolTip(), &window);
        });
        poll->start(300);
        return QApplication::exec();
    }
    if (parser.isSet(screenshotPreferences)) {
        ocam::gui::PreferencesDialog dialog(window.commands());
        dialog.showPage(
            static_cast<ocam::gui::PreferencesDialog::Page>(parser.value(preferencesPage).toInt()));
        if (parser.isSet(preferencesType)) {
            dialog.showOperationType(parser.value(preferencesType).toStdString());
        }
        dialog.show();
        QTimer::singleShot(500, &app, [&] {
            QApplication::exit(dialog.grab().save(parser.value(screenshotPreferences)) ? 0 : 4);
        });
        return QApplication::exec();
    }
    keepAside(window); // with --quiet-windows: off the screen
    window.show();

    bool simulated = false;
    bool positioned = false;
    bool played = false;
    std::pair<int, std::size_t> playFrom;
    if (parser.isSet(smokeTest)) {
        if (files.isEmpty()) {
            std::fprintf(stderr, "--smoke-test requires a file\n");
            return 2;
        }
        QObject::connect(&window, &ocam::gui::MainWindow::loadFinished, &app, [&](bool ok) {
            if (!ok) {
                QApplication::exit(3);
                return;
            }
            // Wait for background toolpath generation, let the window paint, then capture.
            auto* poll = new QTimer(&app);
            QObject::connect(poll, &QTimer::timeout, &app, [&, poll] {
                if (window.toolpathsBusy()) {
                    return;
                }
                if (parser.isSet(screenshotSim) && !simulated) {
                    // Simulate first, then capture once playback has drawn the final stock.
                    simulated = true;
                    if (parser.isSet(simLastSetup)) {
                        window.selectLastSetup();
                    }
                    window.simulateCurrentSetup();
                    return;
                }
                if (window.simulationBusy()) {
                    return;
                }
                if (parser.isSet(simPosition) && !positioned) {
                    positioned = true;
                    for (const QString& p : parser.values(machinePart)) {
                        const qsizetype eq = p.lastIndexOf(QLatin1Char('='));
                        window.simulationPanel()->setMachinePartShown(p.left(eq).toStdString(),
                                                                      p.mid(eq + 1).toInt() != 0);
                    }
                    const QStringList parts = parser.value(simPosition).split(QLatin1Char(':'));
                    window.simulationPanel()->showPosition(
                        parts.value(0).toInt(), parts.value(1, QStringLiteral("0")).toULongLong());
                    return;
                }
                ocam::gui::SimulationPanel* panel = window.simulationPanel();
                if (parser.isSet(simPlay) && !played) {
                    // Play from the position shown, then pause where playback got to.
                    played = true;
                    playFrom = panel->position();
                    panel->resetFrameCount();
                    panel->play();
                    QTimer::singleShot(static_cast<int>(parser.value(simPlay).toDouble() * 1000.0),
                                       &app, [panel] { panel->pause(); });
                    return;
                }
                if (panel->playing()) {
                    return;
                }
                if (parser.isSet(simPlay)) {
                    const auto at = panel->position();
                    std::printf("playback: from %d:%zu to %d:%zu, %d frames\n", playFrom.first,
                                playFrom.second, at.first, at.second, panel->frameCount());
                    std::fflush(stdout);
                    if (at <= playFrom || panel->frameCount() < 2) {
                        std::fprintf(stderr, "playback did not advance\n");
                        poll->stop();
                        QApplication::exit(8);
                        return;
                    }
                }
                poll->stop();
                if (parser.isSet(reselectTest)) {
                    // A new resolution: nothing simulated (or being prepared) matches, so the
                    // stock of operation 0 is simulated afresh.
                    auto* resolution =
                        panel->findChild<QDoubleSpinBox*>(QStringLiteral("simResolution"));
                    const double fine = resolution->singleStep() * 2.0;
                    resolution->setValue(resolution->value() == fine ? fine * 1.5 : fine);
                    window.commands()
                        .find(QStringLiteral("view.inProcessStock"))
                        ->action->setChecked(true);
                    window.selectOperation(0);
                    if (!panel->busy()) {
                        std::fprintf(stderr, "operation 0 is not being simulated\n");
                        QApplication::exit(10);
                        return;
                    }
                    auto* wait = new QTimer(&app);
                    QObject::connect(wait, &QTimer::timeout, &app, [&app, &window, panel, wait] {
                        const double reached = panel->simulationProgress();
                        if (!panel->busy()) {
                            wait->stop();
                            std::fprintf(stderr, "the simulation ended before it was checked\n");
                            QApplication::exit(10);
                            return;
                        }
                        if (reached < 0.05) {
                            return;
                        }
                        wait->stop();
                        window.selectOperation(1);
                        window.selectOperation(0);
                        const double after = panel->simulationProgress();
                        std::printf("progress: %.3f before picking, %.3f after\n", reached, after);
                        std::fflush(stdout);
                        if (after < reached) {
                            std::fprintf(stderr, "the simulation started over\n");
                            QApplication::exit(10);
                            return;
                        }
                        auto* done = new QTimer(&app);
                        QObject::connect(done, &QTimer::timeout, &app, [panel, done] {
                            if (!panel->busy()) {
                                done->stop();
                                QApplication::exit(panel->hasSimulation() ? 0 : 10);
                            }
                        });
                        done->start(50);
                    });
                    wait->start(10);
                    return;
                }
                if (parser.isSet(selectSetup)) {
                    window.selectFirstSetup();
                }
                if (parser.isSet(selectOperation)) {
                    window.selectOperation(parser.value(selectOperation).toInt());
                }
                if (parser.isSet(setFlag)) {
                    // As clicking it: through the check box's own signals.
                    const QStringList kv = parser.value(setFlag).split(QLatin1Char('='));
                    auto* check =
                        window.findChild<QCheckBox*>(QStringLiteral("param_") + kv.value(0));
                    if (!check) {
                        std::fprintf(stderr, "no box %s\n", qPrintable(parser.value(setFlag)));
                        QApplication::exit(2);
                        return;
                    }
                    check->setChecked(kv.value(1) == QStringLiteral("on"));
                    QApplication::processEvents();
                }
                if (parser.isSet(setChoice)) {
                    // As choosing it from the list: through the combo box's own signals.
                    const QStringList kv = parser.value(setChoice).split(QLatin1Char('='));
                    auto* combo =
                        window.findChild<QComboBox*>(QStringLiteral("param_") + kv.value(0));
                    const int at = combo ? combo->findData(kv.value(1)) : -1;
                    if (at < 0) {
                        std::fprintf(stderr, "no choice %s\n", qPrintable(parser.value(setChoice)));
                        QApplication::exit(2);
                        return;
                    }
                    combo->setCurrentIndex(at);
                    QApplication::processEvents();
                }
                if (parser.isSet(expectParams)) {
                    // What the operation's page shows once it has redrawn for the change.
                    QTimer::singleShot(500, &app, [&] {
                        QCoreApplication::sendPostedEvents(nullptr, QEvent::DeferredDelete);
                        int failures = 0;
                        auto shown = [&](const QString& key) {
                            const auto* w =
                                window.findChild<QWidget*>(QStringLiteral("param_") + key);
                            return w && !w->isHidden();
                        };
                        for (const QString& key :
                             parser.value(expectParams)
                                 .split(QLatin1Char(','), Qt::SkipEmptyParts)) {
                            if (!shown(key)) {
                                std::fprintf(stderr, "%s is not shown\n", qPrintable(key));
                                ++failures;
                            }
                        }
                        for (const QString& key :
                             parser.value(expectNoParams)
                                 .split(QLatin1Char(','), Qt::SkipEmptyParts)) {
                            if (shown(key)) {
                                std::fprintf(stderr, "%s is shown\n", qPrintable(key));
                                ++failures;
                            }
                        }
                        QApplication::exit(failures == 0 ? 0 : 11);
                    });
                    return;
                }
                if (parser.isSet(toolpathData)) {
                    QApplication::exit(toolpathDataTest(window, parser.value(toolpathData),
                                                        parser.value(toolpathDataExpect)));
                    return;
                }
                if (parser.isSet(toolpathDataLong)) {
                    QApplication::exit(
                        toolpathDataLongTest(window, parser.value(toolpathDataLong)));
                    return;
                }
                if (parser.isSet(runCommand)) {
                    const ocam::gui::Command* c = window.commands().find(parser.value(runCommand));
                    if (!c) {
                        std::fprintf(stderr, "unknown command\n");
                        QApplication::exit(7);
                        return;
                    }
                    c->action->trigger();
                }
                if (parser.isSet(screenshotPlacement)) {
                    auto* dialog = window.placeOnMachine(std::nullopt, false);
                    if (!dialog) {
                        std::fprintf(stderr, "no setup to place\n");
                        QApplication::exit(7);
                        return;
                    }
                    QTimer::singleShot(500, &app, [&, dialog] {
                        auto point = [](const QString& text) {
                            const QStringList xyz = text.split(QLatin1Char(','));
                            return ocam::Vec3(xyz.value(0).toDouble(), xyz.value(1).toDouble(),
                                              xyz.value(2).toDouble());
                        };
                        const QString mode = parser.value(placeSnap);
                        const ocam::machine::Snap snap =
                            mode == QStringLiteral("axis")    ? ocam::machine::Snap::AxisOnFace
                            : mode == QStringLiteral("point") ? ocam::machine::Snap::Point
                                                              : ocam::machine::Snap::FaceCentre;
                        if (parser.isSet(placeWork)) {
                            std::printf(
                                "work pick: %s\n",
                                dialog->clickWork(point(parser.value(placeWork)), snap).c_str());
                        }
                        if (parser.isSet(placeOn)) {
                            std::printf(
                                "machine pick: %s\n",
                                dialog->clickMachine(point(parser.value(placeOn)), snap).c_str());
                        }
                        std::printf("placement: %s\n", dialog->statusText().c_str());
                        std::fflush(stdout);
                        const bool saved =
                            dialog->snapshot().save(parser.value(screenshotPlacement));
                        dialog->accept();
                        QApplication::exit(saved ? 0 : 4);
                    });
                    return;
                }
                if (parser.isSet(tracePickTest)) {
                    const auto* add = window.commands().find(QStringLiteral("operation.trace2d"));
                    if (!add) {
                        std::fprintf(stderr, "no command to add a trace\n");
                        QApplication::exit(7);
                        return;
                    }
                    add->action->trigger();
                    QTimer::singleShot(1000, &app, [&] {
                        const QString selectText =
                            QCoreApplication::translate("ocam::gui::PropertyPanel", "Select");
                        for (QPushButton* b : window.findChildren<QPushButton*>()) {
                            if (b->text() == selectText && b->isVisible()) {
                                b->click();
                                break;
                            }
                        }
                        ocam::gui::OcctView* view = window.view();
                        view->snapView(V3d_Zpos, false);
                        for (const QString& text :
                             parser.value(tracePickTest).split(QLatin1Char('/'))) {
                            const QStringList c = text.split(QLatin1Char(','));
                            const ocam::Vec3 at(c.value(0).toDouble(), c.value(1).toDouble(),
                                                c.value(2).toDouble());
                            const QString picked = view->pickAt(view->widgetPoint(at), true);
                            std::printf("picked: %s\n", picked.toUtf8().constData());
                            emit view->selectionChanged(picked);
                            QCoreApplication::processEvents();
                        }
                        QTimer::singleShot(1000, &app, [&] {
                            const auto& setup = window.document()->project().setups.front();
                            if (setup.operations.empty() ||
                                setup.operations.back().type != "trace2d") {
                                std::fprintf(stderr, "no trace was added\n");
                                QApplication::exit(7);
                                return;
                            }
                            const auto& op = setup.operations.back();
                            std::map<ocam::Uuid, std::vector<int>> edges;
                            for (const auto& ref : op.geometry) {
                                if (ref.type == ocam::cad::SubShapeType::Edge) {
                                    edges[ref.modelId].push_back(ref.index);
                                }
                            }
                            int chains = 0;
                            int open = 0;
                            for (const auto& [model, indices] : edges) {
                                if (const auto* m = window.document()->model(model)) {
                                    for (const auto& chain : ocam::cad::edgeChains(
                                             *m, indices, ocam::Transform3::Identity())) {
                                        ++chains;
                                        open += chain.path.isClosed() ? 0 : 1;
                                    }
                                }
                            }
                            std::printf("trace edges: %zu, chains: %d, open: %d\n",
                                        op.geometry.size(), chains, open);
                            std::fflush(stdout);
                            if (parser.isSet(screenshotWindow)) {
                                window.grab().save(parser.value(screenshotWindow));
                            }
                            QApplication::exit(
                                op.geometry.size() >= 2 && chains == 1 && open == 1 ? 0 : 8);
                        });
                    });
                    return;
                }
                QTimer::singleShot(500, &app, [&] {
                    if (parser.isSet(hoverTest)) {
                        for (QPushButton* b : window.findChildren<QPushButton*>()) {
                            if (b->text() ==
                                QCoreApplication::translate("ocam::gui::PropertyPanel", "Select")) {
                                b->click();
                                break;
                            }
                        }
                        ocam::gui::OcctView* view = window.view();
                        view->snapView(V3d_Zpos, false);
                        // Sweep along the middle row until an edge's outline lights up.
                        int lit = 0;
                        const int y = view->height() / 2;
                        for (int x = 0; x < view->width() && lit == 0; x += 2) {
                            lit = view->hoverAt(QPoint(x, y));
                        }
                        std::printf("outline edges lit: %d\n", lit);
                        view->dumpImage(parser.value(hoverTest));
                        if (lit == 0) {
                            QApplication::exit(6);
                            return;
                        }
                    }
                    if (parser.isSet(wcsAxisTest)) {
                        const QStringList c = parser.value(wcsAxisTest).split(QLatin1Char(','));
                        if (c.size() != 3) {
                            QApplication::exit(2);
                            return;
                        }
                        const ocam::Vec3 at(c[0].toDouble(), c[1].toDouble(), c[2].toDouble());
                        const QString xz = QCoreApplication::translate("ocam::gui::PropertyPanel",
                                                                       "X and Z from picks");
                        for (QComboBox* combo : window.findChildren<QComboBox*>()) {
                            if (const int i = combo->findText(xz); i >= 0) {
                                combo->setCurrentIndex(i);
                                break;
                            }
                        }
                        QTimer::singleShot(1000, &app, [&, at] {
                            const QString pickText =
                                QCoreApplication::translate("ocam::gui::PropertyPanel", "Pick");
                            for (QPushButton* b : window.findChildren<QPushButton*>()) {
                                if (b->text() == pickText && b->isVisible()) {
                                    b->click(); // the first: X from
                                    break;
                                }
                            }
                            ocam::gui::OcctView* view = window.view();
                            view->snapView(V3d_Zpos, false);
                            const QString picked = view->pickAt(view->widgetPoint(at));
                            emit view->selectionChanged(picked);
                            std::printf("picked: %s\n", picked.toUtf8().constData());
                            QTimer::singleShot(1000, &app, [&] {
                                const auto& setup = window.document()->project().setups.front();
                                const ocam::Vec3 x = setup.wcs.xAxis.normalized();
                                std::printf("WCS X: %.6f %.6f %.6f\n", x.x(), x.y(), x.z());
                                QApplication::exit(0);
                            });
                        });
                        return;
                    }
                    if (parser.isSet(wcsPickTest)) {
                        // Origin from: Picked point (the page is rebuilt after each edit).
                        const QString point =
                            QCoreApplication::translate("ocam::gui::PropertyPanel", "Picked point");
                        bool chosen = false;
                        for (QComboBox* c : window.findChildren<QComboBox*>()) {
                            if (const int i = c->findText(point); i >= 0) {
                                c->setCurrentIndex(i);
                                chosen = true;
                                break;
                            }
                        }
                        if (!chosen) {
                            std::fprintf(stderr, "no Origin from choice\n");
                            QApplication::exit(7);
                            return;
                        }
                        QTimer::singleShot(1000, &app, [&] {
                            const QString pickText =
                                QCoreApplication::translate("ocam::gui::PropertyPanel", "Pick");
                            for (QPushButton* b : window.findChildren<QPushButton*>()) {
                                if (b->text() == pickText && b->isVisible()) {
                                    b->click();
                                    break;
                                }
                            }
                            ocam::gui::OcctView* view = window.view();
                            view->snapView(V3d_Zpos, false);
                            const QString picked = view->pickAt(view->rect().center());
                            emit view->selectionChanged(picked); // as a click in the view does
                            std::printf("picked: %s\n", picked.toUtf8().constData());
                            QTimer::singleShot(1500, &app, [&] {
                                const auto& o =
                                    window.document()->project().setups.front().originFrom;
                                const bool shown =
                                    o && o->from == ocam::document::OriginFrom::Point && o->point &&
                                    o->point->type == ocam::cad::SubShapeType::Face;
                                if (parser.isSet(screenshotWindow)) {
                                    window.grab().save(parser.value(screenshotWindow));
                                }
                                QApplication::exit(shown ? 0 : 8);
                            });
                        });
                        return;
                    }
                    if (parser.isSet(orientationTestOption)) {
                        QApplication::exit(orientationTest(window, parser.value(screenshotWindow)));
                        return;
                    }
                    if (parser.isSet(workholdingSetupTest)) {
                        if (!parser.isSet(workholdingFolder)) {
                            // It empties the library folder: never the user's.
                            std::fprintf(stderr, "--workholding-setup-test needs "
                                                 "--workholding-folder\n");
                            QApplication::exit(2);
                            return;
                        }
                        QApplication::exit(
                            workholdingSetupTestRun(window, parser.value(screenshotWindow)));
                        return;
                    }
                    if (parser.isSet(jawsTest)) {
                        namespace doc = ocam::document;
                        auto* changing = const_cast<doc::Document*>(window.document());
                        ocam::Uuid hardId;
                        ocam::Uuid softId;
                        doc::WorkholdingTemplate soft;
                        changing->modify("Jaws", [&](doc::Project& p) {
                            const auto sets = testJawSets(p.workholdings.front());
                            hardId = sets.first.id;
                            softId = sets.second.id;
                            soft = sets.second;
                            p.workholdings.push_back(sets.first);
                            p.workholdings.push_back(sets.second);
                        });
                        QApplication::processEvents();
                        const auto& setup = [&]() -> const doc::Setup& {
                            return window.document()->project().setups.front();
                        };
                        // The jaws are chosen and set in the workholding dialog (DL-0078), which
                        // edits a snapshot: OK takes it.
                        ocam::gui::WorkholdingSetupDialog* dialog = nullptr;
                        const auto settle = [] {
                            for (int i = 0; i < 3; ++i) {
                                QApplication::processEvents();
                            }
                        };
                        const auto edited = [&]() -> const doc::Setup& {
                            return dialog->edited().project().setups.front();
                        };
                        const auto close = [&] {
                            dialog->accept();
                            dialog = nullptr;
                            settle();
                            QCoreApplication::sendPostedEvents(nullptr, QEvent::DeferredDelete);
                            settle();
                        };
                        const auto combo = [&](const char* name) {
                            return dialog->findChild<QComboBox*>(QString::fromLatin1(name));
                        };
                        const auto length = [&](const char* name) {
                            return dialog->findChild<ocam::gui::LengthSpinBox*>(
                                QString::fromLatin1(name));
                        };
                        const auto note = [&] {
                            auto* label = dialog ? dialog->findChild<QLabel*>(
                                                       QStringLiteral("setupJawPlacement"))
                                                 : nullptr;
                            return label ? label->text().toStdString() : std::string("(none)");
                        };
                        int failed = 0;
                        const auto check = [&](const char* what, bool passed) {
                            std::printf("%s: %s | %s\n", what, passed ? "ok" : "WRONG",
                                        note().c_str());
                            failed += passed ? 0 : 1;
                        };
                        const auto entry = [&](const QString& text) {
                            const bool found = dialog->chooseJaws(text);
                            settle();
                            return found;
                        };
                        // Every label in the dialog saying a problem: one, under the view.
                        const auto problems = [&] {
                            QStringList all;
                            for (QLabel* l : dialog->findChildren<QLabel*>()) {
                                if ((l->objectName() == QStringLiteral("workholdingSetupProblem") &&
                                     !l->text().isEmpty()) ||
                                    l->text().contains(QStringLiteral("can't reach")) ||
                                    l->text().contains(QStringLiteral("grips an outside"))) {
                                    all << l->text();
                                }
                            }
                            return all;
                        };
                        const auto labelled = [&](const QString& text) {
                            const auto labels = dialog->findChildren<QLabel*>();
                            return std::any_of(labels.begin(), labels.end(),
                                               [&](QLabel* l) { return l->text() == text; });
                        };
                        const auto section = [&](const char* key) {
                            return window.findChild<QToolButton*>(QStringLiteral("sectionHead_") +
                                                                  QString::fromLatin1(key));
                        };
                        const auto open = [&](const char* key) {
                            auto* head = section(key);
                            return head && head->isChecked();
                        };
                        check("usual sections open, no jaws section",
                              open("wcs") && open("stock") && open("workholding") &&
                                  !section("jaws") && !open("fixtures"));

                        dialog = window.setUpWorkholding(setup().id, false);
                        settle();
                        check("hard jaws chosen",
                              dialog && entry(QStringLiteral("Step jaws (test) (OD)")) &&
                                  edited().workholding->jawSet == hardId);
                        check(
                            "no option field, no serration, no reverse box",
                            !combo("setupJawOption") &&
                                !dialog->findChild<QSpinBox*>(
                                    QStringLiteral("setupJawSerration")) &&
                                !dialog->findChild<QCheckBox*>(QStringLiteral("setupJawReversed")));
                        auto* grip = combo("setupJawGrip");
                        check("Part gripping: OD / ID",
                              grip && grip->itemText(0) == QStringLiteral("OD") &&
                                  grip->itemText(1) == QStringLiteral("ID"));
                        check("jaw position and first jaw along in the dialog",
                              combo("setupJawPosition") && combo("setupJawAlong") &&
                                  labelled(QStringLiteral("First jaw along")));
                        // A set's ways on are entries of the one list: reversed, its step grips
                        // an inside diameter, so an outside grip is refused, said once.
                        check("reversed entry, said once why not",
                              entry(QStringLiteral("Step jaws (test) (Reversed (ID))")) &&
                                  edited().workholding->jawOption == "Reversed (ID)" &&
                                  problems().size() == 1 &&
                                  problems().front().contains(QStringLiteral("outside diameter")));
                        check("OD entry holds the stock",
                              entry(QStringLiteral("Step jaws (test) (OD)")) &&
                                  edited().workholding->jawOption == "OD" && problems().isEmpty() &&
                                  note().find("holds") != std::string::npos);
                        auto* offset = length("setupJawStopOffset");
                        check("Jaw face offset, no helper text",
                              offset && offset->toolTip().isEmpty() &&
                                  labelled(QStringLiteral("Jaw face offset")) &&
                                  !labelled(QStringLiteral("Out from stop")));
                        if (offset) {
                            offset->setMm(5.0);
                            settle();
                        }
                        check("stock 5 off the jaw face",
                              edited().workholding->stopOffset == 5.0 && problems().isEmpty() &&
                                  note().find("holds") != std::string::npos);
                        // Jaw position Manual, opened beyond the jaws' reach: drawn as far as they
                        // go, said once and briefly.
                        if (auto* position = combo("setupJawPosition")) {
                            position->setCurrentIndex(1); // Manual
                            settle();
                        }
                        if (auto* opening = length("setupJawOpening")) {
                            opening->setMm(200.0);
                            settle();
                        }
                        std::printf("out of reach: %s\n",
                                    problems().join(QStringLiteral(" | ")).toUtf8().constData());
                        check("out of reach: said once, briefly",
                              problems().size() == 1 &&
                                  problems().front().startsWith(
                                      QStringLiteral("The jaws can't reach Ø200.000 mm: on "
                                                     "step 'Inner' they grip Ø")) &&
                                  !problems().front().contains(QStringLiteral("travel")));
                        if (parser.isSet(screenshotWindow)) {
                            QString reach = parser.value(screenshotWindow);
                            reach.insert(reach.lastIndexOf(QLatin1Char('.')),
                                         QStringLiteral("_out_of_reach"));
                            for (int i = 0; i < 50; ++i) {
                                QApplication::processEvents();
                                QThread::msleep(10);
                            }
                            dialog->snapshot().save(reach);
                        }
                        if (auto* position = combo("setupJawPosition")) {
                            position->setCurrentIndex(0); // Auto
                            settle();
                        }
                        check("the project unchanged until OK", !setup().workholding->jawSet);
                        close();
                        check("OK takes the set and its offset",
                              setup().workholding->jawSet == hardId &&
                                  setup().workholding->jawOption == "OD" &&
                                  setup().workholding->stopOffset == 5.0 &&
                                  setup().workholding->jaws == doc::JawSetting::OnStock);
                        const auto* named =
                            window.findChild<QLabel*>(QStringLiteral("setupJawSetName"));
                        check(
                            "the page names the set, and no jaw fields",
                            named && named->text() == QStringLiteral("Step jaws (test) (OD)") &&
                                !section("jaws") &&
                                !window.findChild<QComboBox*>(QStringLiteral("setupJawSet")) &&
                                !window.findChild<QComboBox*>(QStringLiteral("setupJawPosition")) &&
                                !window.findChild<QComboBox*>(QStringLiteral("setupJawAlong")) &&
                                !window.findChild<QComboBox*>(QStringLiteral("setupJawGrip")) &&
                                !window.findChild<ocam::gui::LengthSpinBox*>(
                                    QStringLiteral("setupJawStopOffset")));
                        if (parser.isSet(screenshotWindow)) {
                            QString panelPath = parser.value(screenshotWindow);
                            panelPath.insert(panelPath.lastIndexOf(QLatin1Char('.')),
                                             QStringLiteral("_panel"));
                            for (int i = 0; i < 50; ++i) {
                                QApplication::processEvents();
                                QThread::msleep(10);
                            }
                            if (auto* label =
                                    window.findChild<QWidget*>(QStringLiteral("setupJawSetName"))) {
                                for (QWidget* w = label->parentWidget(); w; w = w->parentWidget()) {
                                    if (auto* area = qobject_cast<QScrollArea*>(w)) {
                                        area->ensureWidgetVisible(label, 0, 250);
                                        break;
                                    }
                                }
                                QApplication::processEvents();
                            }
                            window.grab().save(panelPath);
                        }

                        // A set no setup uses leaves the project: the soft one is put back.
                        if (!window.document()->project().findWorkholding(softId)) {
                            changing->modify("Soft jaws", [&](doc::Project& p) {
                                p.workholdings.push_back(soft);
                            });
                            settle();
                        }
                        dialog = window.setUpWorkholding(setup().id, false);
                        settle();
                        entry(QStringLiteral("Soft jaws (test)"));
                        for (const char* name : {"setupSoftBoreDepth", "setupSoftBoreClamp"}) {
                            if (auto* box = length(name)) {
                                box->setMm(std::string_view(name) == "setupSoftBoreDepth" ? 12.0
                                                                                          : 30.0);
                                settle();
                            }
                        }
                        check("soft jaws bored", edited().workholding->jawSet == softId &&
                                                     edited().workholding->softBore &&
                                                     note().find("Bored to") != std::string::npos);
                        close();
                        if (auto* add = window.findChild<QPushButton*>(
                                QStringLiteral("setupAddSoftJawOp"))) {
                            add->click();
                            settle();
                        }
                        check("boring operation added from the page",
                              setup().workholding->jawSet == softId &&
                                  !setup().operations.empty() &&
                                  setup().operations.front().type == "turn_soft_jaws");
                        std::printf("jaws test: %s\n", failed == 0 ? "all ok" : "WRONG");
                        std::fflush(stdout);
                        if (parser.isSet(screenshotWindow)) {
                            for (int i = 0; i < 50; ++i) {
                                QApplication::processEvents();
                                QThread::msleep(10);
                            }
                            window.grab().save(parser.value(screenshotWindow));
                            // The 3D view (a grab of the window leaves it black): the soft set on
                            // each master jaw.
                            QString view = parser.value(screenshotWindow);
                            view.insert(view.lastIndexOf(QLatin1Char('.')),
                                        QStringLiteral("_view"));
                            window.view()->snapView(V3d_XposYnegZpos, false);
                            window.view()->dumpImage(view);
                        }
                        QApplication::exit(failed == 0 ? 0 : 6);
                        return;
                    }
                    if (parser.isSet(setupMachineListTest)) {
                        // The libraries opened from the setup page's lists, as a user does: the
                        // list's own "Manage..." entry, clicked in its pop-up. Using a machine
                        // there, or closing, rebuilds the page and the list with it.
                        namespace doc = ocam::document;
                        if (!parser.isSet(settingsDir) || !parser.isSet(fittingsFolder)) {
                            // It points the machine library at a folder of its own.
                            std::fprintf(stderr, "--setup-machine-list-test needs --settings-dir "
                                                 "and --fittings-folder\n");
                            QApplication::exit(2);
                            return;
                        }
                        std::filesystem::create_directories(
                            std::filesystem::path(parser.value(fittingsFolder).toStdU16String()));
                        const QTemporaryDir machines;
                        QSettings().setValue(QStringLiteral("machineLibraryFolder"),
                                             machines.path());
                        const auto& project = [&]() -> const doc::Project& {
                            return window.document()->project();
                        };
                        const auto setup = [&]() -> const doc::Setup& {
                            return project().setups.front();
                        };
                        ocam::machine::Machine mine = doc::machineFor(project(), setup());
                        mine.id = "my-test-lathe";
                        mine.name = "My lathe (test)";
                        ocam::gui::MachineLibrary::save(mine);
                        const auto settle = [] {
                            for (int i = 0; i < 10; ++i) {
                                QApplication::processEvents();
                                QThread::msleep(10);
                            }
                        };
                        int failed = 0;
                        const auto check = [&](const char* what, bool passed) {
                            std::printf("%s: %s\n", what, passed ? "ok" : "WRONG");
                            std::fflush(stdout);
                            failed += passed ? 0 : 1;
                        };
                        // Does `act` to the dialog of that class once the list has opened it
                        // (modal: it opens in an event loop of its own).
                        const auto onOpen = [&app](const char* className,
                                                   std::function<void(QDialog*)> act) {
                            auto* watch = new QTimer(&app);
                            auto tries = std::make_shared<int>(0);
                            QObject::connect(
                                watch, &QTimer::timeout, &app, [watch, tries, className, act] {
                                    auto* d =
                                        qobject_cast<QDialog*>(QApplication::activeModalWidget());
                                    if (d && d->inherits(className)) {
                                        watch->stop();
                                        watch->deleteLater();
                                        act(d);
                                    } else if (++*tries > 500) {
                                        watch->stop();
                                        watch->deleteLater();
                                    }
                                });
                            watch->start(20);
                        };
                        const auto machineList = [&] {
                            return window.findChild<QComboBox*>(QStringLiteral("setupMachine"));
                        };
                        const std::string before = setup().machine;
                        const QString setupName = QString::fromStdString(setup().name);

                        bool opened = false;
                        if (parser.isSet(setupMachineListClose)) {
                            // Manage machine library..., then Close: nothing changes.
                            const std::string undoBefore = window.document()->undoLabel();
                            onOpen("ocam::gui::MachineLibraryDialog", [&](QDialog* d) {
                                opened = true;
                                d->reject();
                            });
                            check("the machine list has Manage machine library...",
                                  chooseInPopup(machineList(),
                                                QStringLiteral("Manage machine library...")));
                            settle();
                            check("the library opened from the list and closed, nothing changed",
                                  opened && setup().machine == before && machineList() &&
                                      window.document()->undoLabel() == undoBefore);
                        } else {
                            // Manage machine library..., then Use for the setup.
                            QString useText;
                            onOpen("ocam::gui::MachineLibraryDialog", [&](QDialog* d) {
                                opened = true;
                                auto* list =
                                    d->findChild<QListWidget*>(QStringLiteral("machineList"));
                                const auto rows =
                                    list ? list->findItems(QStringLiteral("My lathe (test)"),
                                                           Qt::MatchStartsWith)
                                         : QList<QListWidgetItem*>();
                                QPushButton* use = nullptr;
                                for (QPushButton* b : d->findChildren<QPushButton*>()) {
                                    if (b->text().startsWith(QStringLiteral("Use for"))) {
                                        use = b;
                                    }
                                }
                                if (rows.isEmpty() || !use || !use->isVisible()) {
                                    d->reject();
                                    return;
                                }
                                list->setCurrentItem(rows.front());
                                QApplication::processEvents();
                                useText = use->text();
                                use->click();
                            });
                            check("the machine list has Manage machine library...",
                                  chooseInPopup(machineList(),
                                                QStringLiteral("Manage machine library...")));
                            settle();
                            std::printf("use button: %s\n", qPrintable(useText));
                            check("the library opened from the list, its Use for the setup",
                                  opened && useText == QStringLiteral("Use for ") + setupName);
                            check("the setup uses the machine chosen",
                                  setup().machine == "my-test-lathe" &&
                                      project().findMachine("my-test-lathe") != nullptr);
                            check("one undo step",
                                  window.document()->undoLabel() == "Use machine My lathe (test)");
                            std::printf("machine list shows: %s\n",
                                        machineList() ? qPrintable(machineList()->currentText())
                                                      : "(no list)");
                            check("the page shows it",
                                  machineList() && machineList()->currentText() ==
                                                       QStringLiteral("My lathe (test)"));

                            // The library's copy changed after the project took it (the user
                            // gave odd stations another kind of block): the page says so beside
                            // the machine list, and Update gives the project the library's.
                            const auto newerHint = [&] {
                                return window.findChild<QLabel*>(
                                    QStringLiteral("setupMachineNewer"));
                            };
                            const auto updateButton = [&] {
                                return window.findChild<QPushButton*>(
                                    QStringLiteral("setupMachineUpdate"));
                            };
                            // The page built again from the project, as after any change.
                            const auto rebuild = [&] {
                                if (auto* props = window.findChild<ocam::gui::PropertyPanel*>()) {
                                    props->showSelection(
                                        {ocam::gui::Selection::Kind::Setup, setup().id});
                                }
                                settle();
                            };
                            const ocam::machine::Machine* own =
                                project().findMachine("my-test-lathe");
                            std::printf("revision: the project's copy %d\n",
                                        own ? own->revision : -1);
                            check("the library counted its first save, the project took it",
                                  own && own->revision == 1);
                            check("no Update while the copies are the same",
                                  newerHint() == nullptr && updateButton() == nullptr);
                            // Saved again unchanged: the same revision.
                            ocam::gui::MachineLibrary::save(*own);
                            rebuild();
                            check("saved unchanged, still no Update",
                                  newerHint() == nullptr && updateButton() == nullptr &&
                                      ocam::gui::MachineLibrary::load().front().machine.revision ==
                                          1);
                            ocam::machine::Machine changed = *own;
                            check("the machine is a lathe", changed.lathe.has_value());
                            if (changed.lathe) {
                                changed.lathe->turret.stationMounts = {
                                    {{1, 3}, {"Milling unit (test)", "Bolt-on (test)"}}};
                            }
                            ocam::gui::MachineLibrary::save(changed);
                            rebuild();
                            std::printf("note: %s\n",
                                        newerHint() ? qPrintable(newerHint()->text()) : "(none)");
                            QPushButton* update = updateButton();
                            check("Update beside the machine list, a note saying the revisions",
                                  newerHint() &&
                                      newerHint()->text().contains(
                                          QStringLiteral("My lathe (test)")) &&
                                      newerHint()->text().contains(QStringLiteral("revision 2")) &&
                                      update && machineList() &&
                                      update->parentWidget() == machineList()->parentWidget());
                            if (update) {
                                update->click();
                                settle();
                            }
                            own = project().findMachine("my-test-lathe");
                            check("the project's copy is the library's",
                                  own && own->lathe && changed.lathe && own->revision == 2 &&
                                      own->lathe->turret.stationMounts ==
                                          changed.lathe->turret.stationMounts &&
                                      std::count_if(project().machines.begin(),
                                                    project().machines.end(),
                                                    [](const ocam::machine::Machine& m) {
                                                        return m.id == "my-test-lathe";
                                                    }) == 1);
                            check("its own undo step",
                                  window.document()->undoLabel() ==
                                      "Update machine My lathe (test) from the library");
                            check("the note is gone", newerHint() == nullptr &&
                                                          updateButton() == nullptr &&
                                                          setup().machine == "my-test-lathe");

                            // The library's file changed outside the library (the revision not
                            // counted): it differs at the same revision, and Update is offered.
                            ocam::machine::Machine edited = *own;
                            if (edited.lathe) {
                                edited.lathe->turret.stationMounts.front().stations = {1, 3, 5};
                            }
                            ocam::machine::saveMachine(edited, ocam::gui::MachineLibrary::folder() /
                                                                   "my-test-lathe.json");
                            rebuild();
                            check("a file changed outside the library: Update, said so",
                                  updateButton() && newerHint() &&
                                      newerHint()->text().contains(
                                          QStringLiteral("changed outside")));
                            // A project copy newer than the library's is left as it is.
                            ocam::machine::Machine older = edited;
                            older.revision = 1;
                            ocam::machine::saveMachine(older, ocam::gui::MachineLibrary::folder() /
                                                                  "my-test-lathe.json");
                            rebuild();
                            check("a library copy of a lower revision: no Update",
                                  updateButton() == nullptr && newerHint() == nullptr);
                        }

                        // The turret's Manage blocks... and the tailstock's Manage centres..., then
                        // Close: in the run that closes the machine library (one opening of it a
                        // run).
                        if (parser.isSet(setupMachineListClose)) {
                            // The turret's Manage blocks..., then Close: in the turret window.
                            check("the turret window opens from the setup page",
                                  openTurretWindow(window) != nullptr);
                            settle();
                            opened = false;
                            onOpen("ocam::gui::LatheFittingsDialog", [&](QDialog* d) {
                                opened = true;
                                d->reject();
                            });
                            const bool blocks = chooseInPopup(
                                window.findChild<QComboBox*>(QStringLiteral("setupStationBlock1")),
                                QStringLiteral("Manage blocks..."));
                            settle();
                            check("the blocks library opened from a station's list and closed",
                                  blocks && opened &&
                                      window.findChild<QComboBox*>(
                                          QStringLiteral("setupStationBlock1")) != nullptr);

                            // The tailstock's Manage centres...: the setup given a tailstock first.
                            for (QCheckBox* box : window.findChildren<QCheckBox*>()) {
                                if (box->text() == QStringLiteral("Tailstock") &&
                                    !box->isChecked()) {
                                    box->click();
                                    settle();
                                    break;
                                }
                            }
                            opened = false;
                            onOpen("ocam::gui::LatheFittingsDialog", [&](QDialog* d) {
                                opened = true;
                                d->reject();
                            });
                            const bool centres =
                                chooseInPopup(window.findChild<QComboBox*>(
                                                  QStringLiteral("setupTailstockCentre")),
                                              QStringLiteral("Manage centres..."));
                            settle();
                            check("the centres library opened from the centre list and closed",
                                  centres && opened && setup().tailstock.has_value() &&
                                      window.findChild<QComboBox*>(
                                          QStringLiteral("setupTailstockCentre")) != nullptr);
                        }
                        std::printf("before: %s\n", before.c_str());
                        std::printf("setup machine list test: %s\n",
                                    failed == 0 ? "all ok" : "WRONG");
                        std::fflush(stdout);
                        if (parser.isSet(screenshotWindow)) {
                            window.grab().save(parser.value(screenshotWindow));
                        }
                        QApplication::exit(failed == 0 ? 0 : 6);
                        return;
                    }
                    if (parser.isSet(turretWindowShot)) {
                        // A look at a project's turret: its window, on a station.
                        ocam::gui::TurretWindow* turret = openTurretWindow(window);
                        if (!turret) {
                            std::fprintf(stderr, "the turret window did not open\n");
                            QApplication::exit(1);
                            return;
                        }
                        turret->preview()->setStation(parser.value(turretWindowShot).toInt());
                        // Large, for close looks cut from it (off the screen, any size goes).
                        turret->resize(2560, 1600);
                        for (int i = 0; i < 60; ++i) {
                            QApplication::processEvents();
                            QThread::msleep(10);
                        }
                        std::printf("turret window: %s\n", qPrintable(turret->preview()->note()));
                        for (const auto& [name, colour] : turret->preview()->partColours()) {
                            if (name.rfind("T0", 0) == 0 || name.rfind("T1", 0) == 0) {
                                std::printf("  %s %06X\n", name.c_str(), colour);
                            }
                        }
                        if (parser.isSet(screenshotWindow)) {
                            const QString file = parser.value(screenshotWindow);
                            QString view = file;
                            view.insert(view.lastIndexOf(QLatin1Char('.')),
                                        QStringLiteral("_preview"));
                            turret->preview()->view()->dumpImage(view);
                            turret->grab().save(file);
                        }
                        QApplication::exit(0);
                        return;
                    }
                    if (parser.isSet(turretPreviewTest)) {
                        // The turret preview beside the setup page: opened from the turret
                        // section, following the row chosen in the table and the project, its
                        // station turned to the work.
                        namespace doc = ocam::document;
                        if (!parser.isSet(fittingsFolder)) {
                            std::fprintf(stderr, "--turret-preview-test needs --fittings-folder\n");
                            QApplication::exit(2);
                            return;
                        }
                        const std::filesystem::path library(
                            parser.value(fittingsFolder).toStdU16String());
                        std::filesystem::remove_all(library);
                        std::filesystem::create_directories(library);
                        auto* changing = const_cast<doc::Document*>(window.document());
                        doc::FittingPackage holder = doc::builtinFittings()[0];
                        holder.builtin = false;
                        holder.fitting.id = ocam::Uuid::generate();
                        holder.fitting.name = "OD holder (test)";
                        changing->useFitting(holder);
                        const auto& project = [&]() -> const doc::Project& {
                            return window.document()->project();
                        };
                        // The stations that hold a tool, by the project.
                        std::vector<int> withTools;
                        for (const auto& s :
                             doc::turretLoad(*window.document(), project().setups.front(),
                                             doc::machineFor(project(), project().setups.front()))
                                 .stations) {
                            withTools.push_back(s.station);
                        }
                        if (withTools.size() < 2) {
                            std::fprintf(stderr, "the project needs two tools in its turret\n");
                            QApplication::exit(2);
                            return;
                        }
                        const int first = withTools[0];
                        const int second = withTools[1];
                        changing->modify("Block", [&](doc::Project& p) {
                            p.setups.front().turretBlocks.push_back({first, holder.fitting.id});
                        });
                        const auto settle = [] {
                            for (int i = 0; i < 20; ++i) {
                                QApplication::processEvents();
                                QThread::msleep(5);
                            }
                        };
                        settle();
                        int failed = 0;
                        const auto check = [&](const char* what, bool passed) {
                            std::printf("%s: %s\n", what, passed ? "ok" : "WRONG");
                            std::fflush(stdout);
                            failed += passed ? 0 : 1;
                        };
                        auto* turret = window.findChild<ocam::gui::TurretWindow*>();
                        check("the turret window waits to be asked for",
                              turret && !turret->isVisible() &&
                                  window.findChild<QToolButton*>(
                                      QStringLiteral("setupStationRow1")) == nullptr);
                        // The setup page says what is on the stations.
                        auto* summary =
                            window.findChild<QLabel*>(QStringLiteral("setupTurretSummary"));
                        if (auto* head = window.findChild<QToolButton*>(
                                QStringLiteral("sectionHead_turret"))) {
                            if (!head->isChecked()) {
                                head->click();
                            }
                            settle();
                            summary =
                                window.findChild<QLabel*>(QStringLiteral("setupTurretSummary"));
                        }
                        std::printf("summary: %s\n",
                                    summary ? qPrintable(summary->text()) : "(none)");
                        check("the setup page sums up the stations",
                              summary &&
                                  summary->text().contains(QStringLiteral("%1: ").arg(first)));
                        if (!turret || openTurretWindow(window) != turret) {
                            check("the turret window opens from the setup page", false);
                            QApplication::exit(6);
                            return;
                        }
                        auto* preview = turret->preview();
                        auto* turretPanel = turret->panel();
                        const auto has = [](const std::vector<std::string>& names,
                                            const std::string& start) {
                            return std::any_of(
                                names.begin(), names.end(),
                                [&](const std::string& n) { return n.rfind(start, 0) == 0; });
                        };
                        const auto print = [&](const char* when) {
                            const auto& at = preview->position();
                            std::printf("%s: station %d, X %.3f Z %.3f T %.4f; %s\n", when,
                                        preview->station(), at.linear.x(), at.linear.z(), at.t,
                                        preview->note().toUtf8().constData());
                            std::string lit;
                            for (const auto& n : preview->highlightedParts()) {
                                lit += " [" + n + "]";
                            }
                            std::printf("  highlighted:%s\n", lit.c_str());
                            std::fflush(stdout);
                        };
                        const auto reaches = [&] {
                            const auto tip = preview->tipNow();
                            const auto work = preview->workPoint();
                            if (tip && work) {
                                std::printf("  tip %.4f %.4f %.4f, work %.4f %.4f %.4f\n", tip->x(),
                                            tip->y(), tip->z(), work->x(), work->y(), work->z());
                            }
                            return tip && work && (*tip - *work).norm() < 1e-3;
                        };
                        const auto row = [&](int station) {
                            return window.findChild<QToolButton*>(
                                QStringLiteral("setupStationRow%1").arg(station));
                        };
                        // The row chosen in the window's table.
                        if (QToolButton* r = row(first)) {
                            r->click();
                        }
                        for (int i = 0; i < 30; ++i) {
                            settle();
                        }
                        print("opened");
                        const std::string firstTool =
                            QStringLiteral("T%1").arg(first, 2, 10, QLatin1Char('0')).toStdString();
                        const std::string secondTool = QStringLiteral("T%1")
                                                           .arg(second, 2, 10, QLatin1Char('0'))
                                                           .toStdString();
                        const std::string firstBlock =
                            QStringLiteral("Station %1 block").arg(first).toStdString();
                        check("the preview shows beside the table, on the row chosen",
                              turret->isVisible() && preview->isVisible() &&
                                  !preview->shownParts().empty() && preview->station() == first);
                        check("its station's block highlighted, its tool not, nothing else",
                              has(preview->highlightedParts(), firstBlock) &&
                                  !has(preview->highlightedParts(), firstTool) &&
                                  has(preview->shownParts(), firstTool) &&
                                  !has(preview->highlightedParts(), secondTool));
                        // The tool in its own colours: what cuts gold, the rest black.
                        int toolParts = 0;
                        bool coloured = true;
                        for (const auto& [name, colour] : preview->partColours()) {
                            if (name.rfind(firstTool, 0) != 0) {
                                continue;
                            }
                            ++toolParts;
                            const bool cuts =
                                name.ends_with(", insert") || name.ends_with(", tool");
                            coloured = coloured && colour == (cuts ? 0xD4A82Au : 0x26282Bu);
                            std::printf("  %s: %06X\n", name.c_str(), colour);
                        }
                        check("the tool black, what cuts gold", toolParts > 0 && coloured);
                        const auto& machine = doc::machineFor(project(), project().setups.front());
                        check("the turret turned to it",
                              std::abs(preview->position().t -
                                       ocam::machine::stationAngle(machine, first)) < 1e-9);
                        check("its tool at the work", reaches());
                        QString file;
                        if (parser.isSet(screenshotWindow)) {
                            file = parser.value(screenshotWindow);
                            QString view = file;
                            view.insert(view.lastIndexOf(QLatin1Char('.')),
                                        QStringLiteral("_preview"));
                            preview->view()->dumpImage(view);
                        }
                        // Another row chosen in the table: the preview follows.
                        if (QToolButton* r = row(second)) {
                            r->click();
                        }
                        settle();
                        print("second row");
                        check("a row chosen in the table chooses the station",
                              preview->station() == second &&
                                  !has(preview->highlightedParts(), secondTool) &&
                                  !has(preview->highlightedParts(), firstBlock) &&
                                  std::abs(preview->position().t -
                                           ocam::machine::stationAngle(machine, second)) < 1e-9 &&
                                  reaches());
                        // The preview's own station: the table marks its row.
                        if (auto* box = preview->findChild<QSpinBox*>(
                                QStringLiteral("turretPreviewStation"))) {
                            box->setValue(first);
                        }
                        settle();
                        check("a station chosen in the preview marks its row",
                              turretPanel->turretStation() == first && row(first) &&
                                  row(first)->isChecked() && !row(second)->isChecked());
                        // At home: the axes at zero, the turret still on the station.
                        preview->setAtWork(false);
                        settle();
                        print("at home");
                        check("at home",
                              preview->position().linear.norm() < 1e-9 &&
                                  std::abs(preview->position().t -
                                           ocam::machine::stationAngle(machine, first)) < 1e-9);
                        preview->setAtWork(true);
                        // Only what is near the work.
                        const std::size_t all = preview->shownParts().size();
                        if (auto* whole = preview->findChild<QCheckBox*>(
                                QStringLiteral("turretPreviewWholeMachine"))) {
                            whole->setChecked(false);
                        }
                        settle();
                        std::string nearby;
                        for (const auto& n : preview->shownParts()) {
                            nearby += " [" + n + "]";
                        }
                        std::printf("near the work:%s\n", nearby.c_str());
                        check("Whole machine off leaves the turret, the spindle and the work",
                              preview->shownParts().size() < all &&
                                  has(preview->shownParts(), firstBlock) &&
                                  has(preview->shownParts(), "Stock"));
                        if (auto* whole = preview->findChild<QCheckBox*>(
                                QStringLiteral("turretPreviewWholeMachine"))) {
                            whole->setChecked(true);
                        }
                        settle();
                        // The enclosure hidden: the rest of the machine stays, and the choice
                        // is remembered.
                        {
                            auto* enclosure = preview->findChild<QCheckBox*>(
                                QStringLiteral("turretPreviewEnclosure"));
                            const bool walls = has(preview->shownParts(), "Enclosure, ");
                            if (enclosure) {
                                enclosure->setChecked(false);
                            }
                            settle();
                            std::string without;
                            for (const auto& n : preview->shownParts()) {
                                without += " [" + n + "]";
                            }
                            std::printf("enclosure off:%s\n", without.c_str());
                            check("Enclosure off hides its walls and keeps the rest of the machine",
                                  enclosure && walls &&
                                      !has(preview->shownParts(), "Enclosure, ") &&
                                      has(preview->shownParts(), "Turret") &&
                                      has(preview->shownParts(), "Headstock") &&
                                      !QSettings()
                                           .value(QStringLiteral("turretPreviewEnclosure"), true)
                                           .toBool());
                            if (enclosure) {
                                enclosure->setChecked(true);
                            }
                        }
                        // The project changes: a block on the second station shows at once.
                        changing->modify("Block", [&](doc::Project& p) {
                            p.setups.front().turretBlocks.push_back({second, holder.fitting.id});
                        });
                        for (int i = 0; i < 10; ++i) {
                            settle();
                        }
                        preview->setStation(second);
                        settle();
                        print("block added");
                        check("a block put on a station shows on it",
                              has(preview->highlightedParts(),
                                  QStringLiteral("Station %1 block").arg(second).toStdString()));
                        // Closed and opened again, on other stations: the view draws each time,
                        // with one note under it (the maintainer: reopened, the preview was a
                        // white box with the notes painted over each other).
                        {
                            const QTemporaryDir shots;
                            const auto draws = [&](const QString& name) {
                                const QString path = shots.filePath(name);
                                if (!preview->view()->dumpImage(path)) {
                                    return false;
                                }
                                const QImage image(path);
                                std::set<QRgb> colours;
                                for (int y = 0; y < image.height(); y += 7) {
                                    for (int x = 0; x < image.width(); x += 7) {
                                        colours.insert(image.pixel(x, y));
                                    }
                                }
                                std::printf("  %s: %dx%d, %zu colours\n", qPrintable(name),
                                            image.width(), image.height(), colours.size());
                                return colours.size() > 20;
                            };
                            const auto handle = [&] {
                                return static_cast<unsigned long long>(preview->view()->winId());
                            };
                            const unsigned long long before = handle();
                            bool everyTime = true;
                            for (int round = 0; round < 3; ++round) {
                                turret->close();
                                settle();
                                const bool closed = !turret->isVisible();
                                const bool reopened = openTurretWindow(window) == turret;
                                if (QToolButton* r = row(round % 2 == 0 ? second : first)) {
                                    r->click();
                                }
                                for (int i = 0; i < 20; ++i) {
                                    settle();
                                }
                                std::printf("reopened %d: view handle %llx (first %llx), %s\n",
                                            round + 1, handle(), before,
                                            preview->view()->drawsIntoItsWindow()
                                                ? "drawn into"
                                                : "NOT drawn into");
                                everyTime = everyTime && closed && reopened &&
                                            preview->view()->drawsIntoItsWindow() &&
                                            handle() == before &&
                                            draws(QStringLiteral("reopen%1.png").arg(round));
                            }
                            check("closed and opened again, the preview draws each time",
                                  everyTime);
                            check(
                                "one note under the preview",
                                preview->findChildren<QLabel*>(QStringLiteral("turretPreviewNote"))
                                        .size() == 1);
                        }
                        std::printf("turret preview test: %s\n", failed == 0 ? "all ok" : "WRONG");
                        std::fflush(stdout);
                        if (!file.isEmpty()) {
                            preview->setStation(first);
                            // The turret table beside the preview.
                            if (QToolButton* r = row(first)) {
                                for (QWidget* w = r->parentWidget(); w; w = w->parentWidget()) {
                                    if (auto* area = qobject_cast<QScrollArea*>(w)) {
                                        area->ensureWidgetVisible(r, 0, 200);
                                        break;
                                    }
                                }
                            }
                            for (int i = 0; i < 10; ++i) {
                                settle();
                            }
                            turret->grab().save(file);
                        }
                        QApplication::exit(failed == 0 ? 0 : 6);
                        return;
                    }
                    if (parser.isSet(stationToolsTest)) {
                        // The turret set up as on a turret whose odd stations take only milling
                        // units (DL-0068), its blocks in a throwaway library: an OD holder put
                        // under a turning tool, a milling unit given a drill, by clicking in the
                        // lists' pop-ups as a user would.
                        namespace doc = ocam::document;
                        if (!parser.isSet(fittingsFolder)) {
                            // It empties the library folder: never the user's.
                            std::fprintf(stderr, "--station-tools-test needs --fittings-folder\n");
                            QApplication::exit(2);
                            return;
                        }
                        const std::filesystem::path library(
                            parser.value(fittingsFolder).toStdU16String());
                        std::filesystem::remove_all(library);
                        std::filesystem::create_directories(library);
                        doc::FittingPackage holder = doc::builtinFittings()[0];
                        holder.builtin = false;
                        holder.fitting.id = ocam::Uuid::generate();
                        holder.fitting.name = "OD holder (test)";
                        holder.fitting.mountType = "Bolt-on";
                        doc::saveFitting(library, holder);
                        doc::FittingPackage unit = doc::builtinFittings()[1];
                        unit.builtin = false;
                        unit.fitting.id = ocam::Uuid::generate();
                        unit.fitting.name = "Milling unit (test)";
                        unit.fitting.mountType = "Unit";
                        doc::saveFitting(library, unit);
                        // A name longer than the panel's lists are wide.
                        doc::FittingPackage longNamed = holder;
                        longNamed.fitting.id = ocam::Uuid::generate();
                        longNamed.fitting.name =
                            "JBH2 - Bolt-on OD and face holder, double, long reach (test)";
                        doc::saveFitting(library, longNamed);
                        auto* changing = const_cast<doc::Document*>(window.document());
                        changing->modify("Stations", [&](doc::Project& p) {
                            doc::Setup& st = p.setups.front();
                            ocam::machine::Machine m = doc::machineFor(p, st);
                            m.id += "-stations-test";
                            m.lathe->turret.blockMounts = {"Bolt-on"};
                            m.lathe->turret.stationMounts = {{{1, 3, 5, 7, 9, 11}, {"Unit"}}};
                            st.machine = m.id;
                            p.machines.push_back(std::move(m));
                        });
                        const auto settle = [] {
                            for (int i = 0; i < 5; ++i) {
                                QApplication::processEvents();
                            }
                        };
                        settle();
                        if (!openTurretWindow(window)) {
                            std::fprintf(stderr, "the turret window did not open\n");
                            QApplication::exit(1);
                            return;
                        }
                        settle();
                        const auto& project = [&]() -> const doc::Project& {
                            return window.document()->project();
                        };
                        const auto numberOf = [&](const char* name) {
                            for (const auto& t : project().tools.tools) {
                                if (t.name == name) {
                                    return t.number;
                                }
                            }
                            return -1;
                        };
                        const auto blockOn = [&](int station) -> std::string {
                            for (const auto& b : project().setups.front().turretBlocks) {
                                if (b.station == station) {
                                    const auto* f = project().findFitting(b.block);
                                    return f ? f->name : "?";
                                }
                            }
                            return "";
                        };
                        const auto list = [&](const char* kind, int station) {
                            return window.findChild<QComboBox*>(QStringLiteral("setupStation%1%2")
                                                                    .arg(QLatin1String(kind))
                                                                    .arg(station));
                        };
                        const auto items = [](QComboBox* box) {
                            QStringList all;
                            for (int i = 0; box && i < box->count(); ++i) {
                                all << box->itemText(i);
                            }
                            return all.join(QStringLiteral(" | "));
                        };
                        const auto click = [](QComboBox* box, const QString& text) {
                            return chooseInPopup(box, text);
                        };
                        int failed = 0;
                        const auto check = [&](const char* what, bool passed) {
                            std::printf("%s: %s\n", what, passed ? "ok" : "WRONG");
                            std::fflush(stdout);
                            failed += passed ? 0 : 1;
                        };
                        std::printf("station 1 blocks: %s\nstation 1 tools: %s\n",
                                    items(list("Block", 1)).toUtf8().constData(),
                                    items(list("Tool", 1)).toUtf8().constData());
                        check("the tool's row holds no number box",
                              window.findChild<QWidget*>(QStringLiteral("setupTurretTable")) &&
                                  window.findChild<QWidget*>(QStringLiteral("setupTurretTable"))
                                      ->findChildren<QSpinBox*>()
                                      .isEmpty());
                        // T1, a turning tool, on station 1: a station for milling units only.
                        check(
                            "station 1 shows its tool, offers no holder, and says why",
                            list("Tool", 1) &&
                                list("Tool", 1)->currentText().contains(QStringLiteral("CNMG")) &&
                                list("Block", 1)
                                        ->findText(QStringLiteral("OD holder"), Qt::MatchContains) <
                                    0 &&
                                anyToolTipHolds(list("Block", 1), QStringLiteral("it takes Unit")));
                        // The library's blocks under a heading, each name first and whole in
                        // the open list, however narrow the list is.
                        if (QComboBox* two = list("Block", 2)) {
                            const int first = libraryEntry(two);
                            const int heading = first - 1;
                            const QString longName = QStringLiteral(
                                "JBH2 - Bolt-on OD and face holder, double, long reach (test)");
                            const int at = two->findText(longName);
                            check("the library's blocks under a heading, without a prefix",
                                  first > 0 && ocam::gui::isListHeading(two, heading) &&
                                      !(two->model()->flags(two->model()->index(heading, 0)) &
                                        Qt::ItemIsEnabled) &&
                                      at >= first &&
                                      two->findText(QStringLiteral("Library: "),
                                                    Qt::MatchStartsWith) < 0);
                            check("a library block's tooltip names it and where it is from",
                                  at >= 0 && two->itemData(at, Qt::ToolTipRole)
                                                 .toString()
                                                 .startsWith(longName + QStringLiteral("\nIn your "
                                                                                       "library")));
                            check("the note on blocks left out is short, said in full in its "
                                  "tooltip",
                                  two->findText(QStringLiteral("more that do not fit"),
                                                Qt::MatchContains) < 0 ||
                                      anyToolTipHolds(two, QStringLiteral("it takes")));
                            two->showPopup();
                            settle();
                            // Its width, as the pop-up opened: off the screen on Windows
                            // (--quiet-windows) it does not stay open.
                            QWidget* popup = two->view()->window();
                            const int needs =
                                two->view()->fontMetrics().horizontalAdvance(longName);
                            std::printf("block list %d px wide, open %d px (%s), the long name "
                                        "%d px, cut %s\n",
                                        two->width(), popup->width(),
                                        popup->isVisible() ? "shown" : "not shown", needs,
                                        two->view()->textElideMode() == Qt::ElideRight
                                            ? "at the end"
                                            : "elsewhere");
                            check("the open list shows the long name whole",
                                  popup->width() > needs &&
                                      two->view()->textElideMode() == Qt::ElideRight);
                            check("the closed list narrower than the name (what is checked)",
                                  two->width() < needs);
                            if (parser.isSet(screenshotWindow)) {
                                QString file = parser.value(screenshotWindow);
                                file.insert(file.lastIndexOf(QLatin1Char('.')),
                                            QStringLiteral("_list"));
                                popup->grab().save(file);
                            }
                            two->hidePopup();
                            settle();
                        }
                        check("the turning tool put on station 2",
                              click(list("Tool", 2), QStringLiteral("CNMG")) &&
                                  numberOf("CNMG 120404") == 2);
                        check("the OD holder put under it",
                              click(list("Block", 2), QStringLiteral("OD holder")) &&
                                  blockOn(2) == "OD holder (test)" &&
                                  list("Tool", 2)->currentText().contains(QStringLiteral("CNMG")));
                        const auto load = [&] {
                            return doc::turretLoad(
                                *window.document(), project().setups.front(),
                                doc::machineFor(project(), project().setups.front()));
                        };
                        const auto heldBy = [&](int station) -> std::string {
                            for (const auto& s : load().stations) {
                                if (s.station == station && s.block) {
                                    const auto* f = project().findFitting(*s.block);
                                    return s.name + " in " + (f ? f->name : "?");
                                }
                            }
                            return "";
                        };
                        check("the turning tool sits in the holder",
                              heldBy(2) == "CNMG 120404 in OD holder (test)");
                        check("a milling unit put on station 3",
                              click(list("Block", 3), QStringLiteral("Milling unit")) &&
                                  blockOn(3) == "Milling unit (test)");
                        check("the drill put in it",
                              click(list("Tool", 3), QStringLiteral("Drill 16")) &&
                                  numberOf("Drill 16") == 3 &&
                                  heldBy(3) == "Drill 16 in Milling unit (test)");
                        // The turning tool to station 3: the drill takes its old number.
                        check("two tools swapped", click(list("Tool", 3), QStringLiteral("CNMG")) &&
                                                       numberOf("CNMG 120404") == 3 &&
                                                       numberOf("Drill 16") == 2);
                        // A tool an operation uses can go to another station, not out.
                        click(list("Tool", 3), QStringLiteral("(no tool)"));
                        check("a used tool not taken out", numberOf("CNMG 120404") == 3);
                        check("one undo step each",
                              window.document()->undoLabel() == "Put a tool on a station");
                        std::printf("station tools test: %s\n", failed == 0 ? "all ok" : "WRONG");
                        std::fflush(stdout);
                        if (parser.isSet(screenshotWindow)) {
                            for (int i = 0; i < 50; ++i) {
                                QApplication::processEvents();
                                QThread::msleep(10);
                            }
                            if (auto* rows = window.findChild<QWidget*>(
                                    QStringLiteral("setupTurretTable"))) {
                                for (QWidget* w = rows->parentWidget(); w; w = w->parentWidget()) {
                                    if (auto* area = qobject_cast<QScrollArea*>(w)) {
                                        area->ensureWidgetVisible(rows, 0, 0);
                                        break;
                                    }
                                }
                                QApplication::processEvents();
                            }
                            window.grab().save(parser.value(screenshotWindow));
                        }
                        QApplication::exit(failed == 0 ? 0 : 6);
                        return;
                    }
                    if (parser.isSet(turretEditTest)) {
                        // The turret's window, from the setup page, as a user would open it.
                        if (!openTurretWindow(window)) {
                            std::fprintf(stderr, "the turret window did not open\n");
                            QApplication::exit(1);
                            return;
                        }
                        // Finds the page's widgets again after each change: it is rebuilt.
                        const auto table = [&] {
                            auto* found =
                                window.findChild<QWidget*>(QStringLiteral("setupTurretTable"));
                            if (!found) {
                                std::fprintf(stderr, "no turret on the setup page\n");
                                QApplication::exit(1);
                            }
                            return found;
                        };
                        const auto report = [&](const char* what) {
                            const auto& project = window.document()->project();
                            std::printf("%s:", what);
                            for (const auto& entry : project.setups.front().turret) {
                                const auto* tool = project.tools.findTool(entry.tool);
                                std::printf(" T%d%s", tool ? tool->number : 0,
                                            entry.tip ? "(tip)" : "");
                            }
                            std::printf(" | blocks %zu, fittings %zu, centre %s",
                                        project.setups.front().turretBlocks.size(),
                                        project.fittings.size(),
                                        project.setups.front().tailstock &&
                                                project.setups.front().tailstock->centre
                                            ? "fitted"
                                            : "none");
                            std::printf(" | tools:");
                            for (const auto& tool : project.tools.tools) {
                                std::printf(" %d", tool.number);
                            }
                            std::printf("\n");
                            std::fflush(stdout);
                        };
                        report("start");
                        auto* add = window.findChild<QComboBox*>(QStringLiteral("setupTurretAdd"));
                        const QVariant rider =
                            add && add->count() > 1 ? add->itemData(1) : QVariant();
                        if (add && add->count() > 1) {
                            add->setCurrentIndex(1);
                            add->activated(1);
                            QApplication::processEvents();
                        }
                        report("added");
                        // The tool that rides along, put on station 9 from that station's list.
                        if (auto* nine = table()->findChild<QComboBox*>(
                                QStringLiteral("setupStationTool9"))) {
                            const int i = rider.isValid() ? nine->findData(rider) : -1;
                            if (i >= 0) {
                                nine->setCurrentIndex(i);
                                nine->activated(i);
                                QApplication::processEvents();
                            }
                        }
                        report("station");
                        const auto tips = table()->findChildren<ocam::gui::LengthSpinBox*>();
                        if (!tips.empty()) {
                            tips.front()->setMm(200.0);
                            QApplication::processEvents();
                        }
                        report("tip");
                        const auto buttons = table()->findChildren<QPushButton*>();
                        if (!buttons.isEmpty()) {
                            buttons.front()->click();
                            QApplication::processEvents();
                        }
                        report("reset");
                        // A block from the library on the first station, the tailstock's centre.
                        for (QComboBox* box : table()->findChildren<QComboBox*>()) {
                            const int library = libraryEntry(box);
                            if (library >= 0) {
                                box->setCurrentIndex(library);
                                box->activated(library);
                                QApplication::processEvents();
                                break;
                            }
                        }
                        report("block");
                        if (auto* centre = window.findChild<QComboBox*>(
                                QStringLiteral("setupTailstockCentre"))) {
                            const int library = libraryEntry(centre);
                            if (library >= 0) {
                                centre->setCurrentIndex(library);
                                centre->activated(library);
                                QApplication::processEvents();
                            }
                        }
                        report("centre");
                        const auto& before = window.document()->project();
                        const bool centreOk = before.setups.front().turretBlocks.size() == 1 &&
                                              before.fittings.size() == 2 &&
                                              before.setups.front().tailstock &&
                                              before.setups.front().tailstock->centre;
                        // A machine whose station 2 takes only milling units (DL-0068).
                        namespace doc = ocam::document;
                        doc::FittingPackage unit = doc::builtinFittings()[1];
                        unit.builtin = false;
                        unit.fitting.id = ocam::Uuid::generate();
                        unit.fitting.name = "Milling unit (test)";
                        unit.fitting.mountType = "Unit";
                        // The window follows the document's listeners; the test changes it.
                        auto* changing = const_cast<doc::Document*>(window.document());
                        changing->useFitting(unit);
                        changing->modify("Station test", [&](doc::Project& p) {
                            doc::Setup& st = p.setups.front();
                            ocam::machine::Machine m = doc::machineFor(p, st);
                            m.id += "-station-test";
                            m.lathe->turret.stationMounts = {{{2}, {"Unit"}}};
                            st.machine = m.id;
                            p.machines.push_back(std::move(m));
                        });
                        QApplication::processEvents();
                        bool stationOk = false;
                        if (auto* two = window.findChild<QComboBox*>(
                                QStringLiteral("setupStationBlock2"))) {
                            QStringList items;
                            for (int i = 0; i < two->count(); ++i) {
                                items << two->itemText(i);
                            }
                            std::printf("station 2 offers: %s\n",
                                        items.join(QStringLiteral(" | ")).toUtf8().constData());
                            stationOk = two->findText(QStringLiteral("Milling unit (test)")) >= 0 &&
                                        libraryEntry(two) < 0 &&
                                        anyToolTipHolds(two, QStringLiteral("it takes Unit"));
                            const int i = two->findText(QStringLiteral("Milling unit (test)"));
                            if (i >= 0) {
                                two->setCurrentIndex(i);
                                two->activated(i);
                                QApplication::processEvents();
                            }
                            const auto& blocks =
                                window.document()->project().setups.front().turretBlocks;
                            stationOk =
                                stationOk && std::any_of(blocks.begin(), blocks.end(),
                                                         [&](const doc::StationBlock& b) {
                                                             return b.station == 2 &&
                                                                    b.block == unit.fitting.id;
                                                         });
                        }
                        bool otherOk = false;
                        if (auto* one = window.findChild<QComboBox*>(
                                QStringLiteral("setupStationBlock1"))) {
                            otherOk = one->findText(QStringLiteral("Milling unit (test)")) < 0;
                        }
                        report("station");
                        std::printf("station test: centre %s, station 2 %s, station 1 %s\n",
                                    centreOk ? "ok" : "WRONG", stationOk ? "ok" : "WRONG",
                                    otherOk ? "ok" : "WRONG");
                        std::fflush(stdout);
                        if (parser.isSet(screenshotWindow)) {
                            // The page is rebuilt after the last change: let it, and show the
                            // turret's rows.
                            for (int i = 0; i < 50; ++i) {
                                QApplication::processEvents();
                                QThread::msleep(10);
                            }
                            if (auto* rows = window.findChild<QWidget*>(
                                    QStringLiteral("setupStationBlock2"))) {
                                for (QWidget* w = rows->parentWidget(); w; w = w->parentWidget()) {
                                    if (auto* area = qobject_cast<QScrollArea*>(w)) {
                                        area->ensureWidgetVisible(rows, 0, 200);
                                        break;
                                    }
                                }
                                QApplication::processEvents();
                            }
                            window.grab().save(parser.value(screenshotWindow));
                        }
                        QApplication::exit(0);
                        return;
                    }
                    if (parser.isSet(wheelTest)) {
                        // A spin box of the setup page that can go up.
                        QPointer<QDoubleSpinBox> spin;
                        for (QDoubleSpinBox* s : window.findChildren<QDoubleSpinBox*>()) {
                            if (s->isVisible() && s->isEnabled() && s->value() < s->maximum()) {
                                spin = s;
                                break;
                            }
                        }
                        if (!spin) {
                            std::fprintf(stderr, "wheel test: no spin box\n");
                            QApplication::exit(9);
                            return;
                        }
                        bool changed = false;
                        QObject::connect(spin, &QDoubleSpinBox::valueChanged, &app,
                                         [&changed] { changed = true; });
                        auto turn = [&] {
                            const QPointF at(spin->rect().center());
                            QWheelEvent wheel(at, spin->mapToGlobal(at), QPoint(), QPoint(0, 120),
                                              Qt::NoButton, Qt::NoModifier, Qt::NoScrollPhase,
                                              false);
                            QApplication::sendEvent(spin, &wheel);
                            QApplication::processEvents();
                        };
                        window.view()->setFocus();
                        turn();
                        std::printf("wheel unfocused: %s\n", changed ? "changed" : "unchanged");
                        if (changed) {
                            QApplication::exit(9);
                            return;
                        }
                        window.activateWindow();
                        spin->setFocus(Qt::OtherFocusReason);
                        QApplication::processEvents();
                        if (spin && spin->hasFocus()) {
                            turn();
                            std::printf("wheel focused: %s\n", changed ? "changed" : "unchanged");
                            if (!changed) {
                                QApplication::exit(9);
                                return;
                            }
                        } else {
                            std::printf("wheel focused: not checked (no window focus)\n");
                        }
                    }
                    if (parser.isSet(pickTest)) {
                        window.view()->snapView(V3d_Zpos, false);
                        const QString picked =
                            window.view()->pickAt(window.view()->rect().center());
                        std::printf("picked: %s\n", picked.toUtf8().constData());
                        if (!picked.startsWith(QStringLiteral("Face"))) {
                            QApplication::exit(5);
                            return;
                        }
                    }
                    if (parser.isSet(screenshotSim) &&
                        !window.view()->dumpImage(parser.value(screenshotSim))) {
                        std::fprintf(stderr, "failed to save simulation screenshot\n");
                        QApplication::exit(4);
                        return;
                    }
                    if (parser.isSet(screenshotSim) && parser.isSet(screenshotClosedSim)) {
                        window.simulationPanel()->closeSimulation();
                        if (!window.view()->dumpImage(parser.value(screenshotClosedSim))) {
                            std::fprintf(stderr, "failed to save screenshot\n");
                            QApplication::exit(4);
                            return;
                        }
                    }
                    if (parser.isSet(screenshot) &&
                        !window.view()->dumpImage(parser.value(screenshot))) {
                        std::fprintf(stderr, "failed to save screenshot\n");
                        QApplication::exit(4);
                        return;
                    }
                    if (parser.isSet(panelSweep)) {
                        // Every width the dock can take: the fields get the width the dock
                        // gains (never squeezed by a label taking it, as a long label did at
                        // one width), and a page wider than the dock scrolls rather than being
                        // cut off.
                        auto* dock =
                            window.findChild<QDockWidget*>(QStringLiteral("propertiesDock"));
                        auto* area = dock ? qobject_cast<QScrollArea*>(dock->widget()) : nullptr;
                        if (!area || !area->widget()) {
                            QApplication::exit(9);
                            return;
                        }
                        const QDir dir(parser.value(panelSweep));
                        QDir().mkpath(dir.absolutePath());
                        int problems = 0;
                        int lastField = -1;
                        // About a laptop screen's height, so the page scrolls.
                        window.showNormal();
                        window.resize(window.width(), 900);
                        for (int w = 220; w <= 640; ++w) {
                            window.resizeDocks({dock}, {w}, Qt::Horizontal);
                            for (int k = 0; k < 4; ++k) {
                                QApplication::processEvents();
                            }
                            const int page = area->widget()->width();
                            const int view = area->viewport()->width();
                            // The first line edit (the name): the field column's width.
                            const auto edits = area->widget()->findChildren<QLineEdit*>();
                            const int field = edits.isEmpty() ? -1 : edits.front()->width();
                            QString problem;
                            // A label whose text needs more lines than its row shows.
                            for (QLabel* l : area->widget()->findChildren<QLabel*>()) {
                                if (l->isVisible() && l->wordWrap() && !l->text().isEmpty() &&
                                    l->heightForWidth(l->width()) > l->height() + 1) {
                                    problem = QStringLiteral("label '%1' cut off (%2 of %3 px)")
                                                  .arg(l->text().left(30))
                                                  .arg(l->height())
                                                  .arg(l->heightForWidth(l->width()));
                                    break;
                                }
                            }
                            if (!problem.isEmpty()) {
                                // reported below
                            } else if (page > view + 1 &&
                                       !area->horizontalScrollBar()->isVisible()) {
                                problem =
                                    QStringLiteral("page %1 cut off at %2").arg(page).arg(view);
                            } else if (lastField >= 0 && field < lastField - 4) {
                                problem = QStringLiteral("fields squeezed from %1 to %2")
                                              .arg(lastField)
                                              .arg(field);
                            }
                            lastField = field;
                            if (w == 260 || w == 360 || w == 480) { // to look at
                                dock->grab().save(dir.filePath(QStringLiteral("at_%1.png").arg(w)));
                            }
                            if (!problem.isEmpty()) {
                                std::printf("width %d: %s\n", dock->width(),
                                            problem.toUtf8().constData());
                                if (problems++ < 6) {
                                    dock->grab().save(dir.filePath(
                                        QStringLiteral("panel_%1.png").arg(dock->width())));
                                }
                            }
                        }
                        std::printf("%d widths with layout problems\n", problems);
                        QApplication::exit(problems == 0 ? 0 : 7);
                        return;
                    }
                    if (parser.isSet(screenshotWindow) &&
                        !window.grab().save(parser.value(screenshotWindow))) {
                        std::fprintf(stderr, "failed to save window screenshot\n");
                        QApplication::exit(4);
                        return;
                    }
                    if (parser.isSet(screenshotPost)) {
                        // The post dialog is modal: capture it from a timer while it is open.
                        // The progress dialog must be gone by then (left alive, its timer shows
                        // it again over the program).
                        auto* capture = new QTimer(&app);
                        QObject::connect(capture, &QTimer::timeout, &app, [&, capture] {
                            // First the post settings: captured if asked, then accepted.
                            if (auto* settings = qobject_cast<ocam::gui::PostProcessDialog*>(
                                    QApplication::activeModalWidget())) {
                                if (parser.isSet(screenshotPostSettings) &&
                                    !settings->grab().save(parser.value(screenshotPostSettings))) {
                                    std::fprintf(stderr, "failed to save the post settings\n");
                                }
                                settings->accept();
                                return;
                            }
                            auto* dialog = qobject_cast<ocam::gui::PostDialog*>(
                                QApplication::activeModalWidget());
                            if (!dialog) {
                                return;
                            }
                            capture->stop();
                            const auto tops = QApplication::topLevelWidgets();
                            const bool progressLeft =
                                std::any_of(tops.begin(), tops.end(), [](const QWidget* w) {
                                    return qobject_cast<const QProgressDialog*>(w) != nullptr;
                                });
                            if (progressLeft) {
                                std::fprintf(stderr, "the progress dialog is still there\n");
                            }
                            const bool saved = dialog->grab().save(parser.value(screenshotPost));
                            dialog->close();
                            QApplication::exit(progressLeft ? 5 : saved ? 0 : 4);
                        });
                        capture->start(200);
                        window.postProcessSetup();
                        return;
                    }
                    QApplication::exit(0);
                });
            });
            poll->start(100);
        });
    }
    if (!files.isEmpty()) {
        window.openFile(files.front());
    } else if (!parser.isSet(smokeTest) && ocam::gui::StartHub::showAtStartup()) {
        QTimer::singleShot(0, &window, [&window] { window.showStartHub(); });
    }
    return QApplication::exec();
}
