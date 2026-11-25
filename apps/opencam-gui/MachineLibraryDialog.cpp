// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "MachineLibraryDialog.hpp"

#include "MachineLibrary.hpp"
#include "MachinePartsImportDialog.hpp"
#include "OcctView.hpp"
#include "PostLibrary.hpp"
#include "PostOptionsForm.hpp"
#include "ShowBuiltins.hpp"

#include <ocam/document/PostSetup.hpp>
#include <ocam/document/StepBodies.hpp>
#include <ocam/foundation/Error.hpp>
#include <ocam/foundation/Units.hpp>
#include <ocam/geom3d/Bvh.hpp>
#include <ocam/machine/Model.hpp>
#include <ocam/machine/Turret.hpp>

#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QInputDialog>
#include <QKeyEvent>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPainter>
#include <QPushButton>
#include <QRegularExpression>
#include <QScrollArea>
#include <QScrollBar>
#include <QSettings>
#include <QSlider>
#include <QSpinBox>
#include <QSplitter>
#include <QTableWidget>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>

namespace ocam::gui {
namespace {

constexpr double kPerMinute = 60.0;

QString qs(const std::string& s) {
    return QString::fromStdString(s);
}
std::string ss(const QString& s) {
    return s.trimmed().toStdString();
}

struct KinematicsChoice {
    machine::Kinematics kinematics;
    const char* label;
};
const KinematicsChoice kKinematics[] = {
    {machine::Kinematics::Mill3, "3-axis mill (X Y Z)"},
    {machine::Kinematics::Mill4A, "4-axis mill, rotary table A about X"},
    {machine::Kinematics::Mill5TableAC, "5-axis trunnion: A tilt, C table"},
    {machine::Kinematics::Mill5TableBC, "5-axis trunnion: B tilt, C table"},
    {machine::Kinematics::Mill5HeadTableBC, "5-axis head-table: B head, C table"},
    {machine::Kinematics::Mill5HeadBC, "5-axis head-head: C and B in the head"},
    {machine::Kinematics::Lathe2, "2-axis lathe (X Z)"},
};

const char* kAttach[] = {"head", "table", "cradle"};

QDoubleSpinBox* spin(double min, double max, double value, int decimals, const QString& suffix) {
    auto* s = new QDoubleSpinBox;
    s->setRange(min, max);
    s->setDecimals(decimals);
    s->setValue(value);
    s->setSuffix(suffix);
    s->setKeyboardTracking(false);
    return s;
}

double cellNumber(const QTableWidget* t, int row, int col) {
    const QTableWidgetItem* item = t->item(row, col);
    return item ? item->text().toDouble() : 0.0;
}

QTableWidgetItem* numberItem(double v) {
    return new QTableWidgetItem(QString::number(v, 'g', 10));
}

} // namespace

MachineLibraryDialog::MachineLibraryDialog(QWidget* parent) : QDialog(parent) {
    setWindowTitle(tr("Machine Library"));
    // A normal window: maximize and minimize buttons next to the close button.
    setWindowFlags(windowFlags() | Qt::WindowMinMaxButtonsHint);
    resize(1560, 860);
    buildUi();
    reloadList();
    const QByteArray geometry =
        QSettings().value(QStringLiteral("machineLibraryGeometry")).toByteArray();
    if (!geometry.isEmpty()) {
        restoreGeometry(geometry); // size, position and maximized state of the last opening
    }
}

void MachineLibraryDialog::done(int result) {
    QSettings().setValue(QStringLiteral("machineLibraryGeometry"), saveGeometry());
    // Closed: no preview to draw. One drawn after the window was hidden crashed (Mesa, X11).
    if (previewTimer_) {
        previewTimer_->stop();
    }
    QDialog::done(result);
}

void MachineLibraryDialog::buildUi() {
    auto* root = new QVBoxLayout(this);
    auto* splitter = new QSplitter(this);
    root->addWidget(splitter, 1);

    // Left: the machines and library actions.
    auto* left = new QWidget;
    auto* leftLayout = new QVBoxLayout(left);
    leftLayout->setContentsMargins(0, 0, 0, 0);
    list_ = new QListWidget;
    list_->setObjectName(QStringLiteral("machineList"));
    leftLayout->addWidget(list_, 1);
    showBuiltins_ = newShowBuiltinsBox();
    leftLayout->addWidget(showBuiltins_);
    connect(showBuiltins_, &QCheckBox::toggled, this, [this] { applyShowBuiltins(); });
    auto* buttons = new QGridLayout;
    auto* add = new QPushButton(tr("New..."));
    add->setToolTip(tr("A new machine, starting from a built-in one"));
    auto* duplicate = new QPushButton(tr("Duplicate"));
    delete_ = new QPushButton(tr("Delete"));
    auto* importButton = new QPushButton(tr("Import..."));
    auto* exportButton = new QPushButton(tr("Export..."));
    buttons->addWidget(add, 0, 0);
    buttons->addWidget(duplicate, 0, 1);
    buttons->addWidget(delete_, 0, 2);
    buttons->addWidget(importButton, 1, 0);
    buttons->addWidget(exportButton, 1, 1);
    leftLayout->addLayout(buttons);
    auto* folder =
        new QLabel(tr("Library folder: %1")
                       .arg(QString::fromStdU16String(MachineLibrary::folder().u16string())));
    folder->setWordWrap(true);
    folder->setTextInteractionFlags(Qt::TextSelectableByMouse);
    folder->setStyleSheet(QStringLiteral("color: gray"));
    leftLayout->addWidget(folder);
    splitter->addWidget(left);

    // Right: the editor.
    auto* scroll = new QScrollArea;
    scroll_ = scroll;
    scroll->setWidgetResizable(true);
    editor_ = new QWidget;
    auto* editorLayout = new QVBoxLayout(editor_);
    readOnlyNote_ =
        new QLabel(tr("Built-in machines are read-only. Duplicate one to customize it."));
    readOnlyNote_->setStyleSheet(QStringLiteral("color: #c08000"));
    editorLayout->addWidget(readOnlyNote_);

    auto* identity = new QGroupBox(tr("Machine"));
    auto* identityForm = new QFormLayout(identity);
    id_ = new QLineEdit;
    id_->setToolTip(tr("Lower case letters, digits, '.', '_' and '-'; setups refer to it"));
    name_ = new QLineEdit;
    vendor_ = new QLineEdit;
    kinematics_ = new QComboBox;
    for (const auto& k : kKinematics) {
        kinematics_->addItem(tr(k.label), static_cast<int>(k.kinematics));
    }
    identityForm->addRow(tr("Id"), id_);
    identityForm->addRow(tr("Name"), name_);
    identityForm->addRow(tr("Vendor"), vendor_);
    identityForm->addRow(tr("Kinematics"), kinematics_);
    // How it stands (DL-0077): the views' Top, Front and the view cube follow it.
    standUp_ = new QComboBox;
    standUp_->setObjectName(QStringLiteral("machineStandUp"));
    standUp_->setToolTip(tr("The machine axis that points up: Z on a mill or a vertical lathe, "
                            "while a horizontal lathe's spindle (Z) lies across. The views (Top, "
                            "Front, the view cube) and the simulation follow it"));
    standFront_ = new QComboBox;
    standFront_->setObjectName(QStringLiteral("machineStandFront"));
    standFront_->setToolTip(tr("The machine axis that points out of its front, towards where the "
                               "operator stands: the Front view looks from there"));
    standUp_->addItem(QString(), -1);
    for (const machine::AxisSense a :
         {machine::AxisSense::PlusX, machine::AxisSense::MinusX, machine::AxisSense::PlusY,
          machine::AxisSense::MinusY, machine::AxisSense::PlusZ, machine::AxisSense::MinusZ}) {
        standUp_->addItem(qs(std::string(machine::toString(a))), static_cast<int>(a));
        standFront_->addItem(qs(std::string(machine::toString(a))), static_cast<int>(a));
    }
    auto* standRow = new QHBoxLayout;
    standRow->addWidget(new QLabel(tr("Up")));
    standRow->addWidget(standUp_, 1);
    standRow->addWidget(new QLabel(tr("Front")));
    standRow->addWidget(standFront_, 1);
    identityForm->addRow(tr("Stands"), standRow);
    post_ = new QComboBox;
    post_->setToolTip(tr("The post processor setups on this machine use unless they choose "
                         "another (Setup > Post Process). A post from your post library is "
                         "copied into a project when a setup there uses it"));
    postOptionsButton_ = new QPushButton(tr("Configure..."));
    postOptionsButton_->setToolTip(tr("The values of the default post's options that setups on "
                                      "this machine start from"));
    auto* postRow = new QHBoxLayout;
    postRow->addWidget(post_, 1);
    postRow->addWidget(postOptionsButton_);
    identityForm->addRow(tr("Default post"), postRow);
    editorLayout->addWidget(identity);

    latheBox_ = new QGroupBox(tr("Lathe turret"));
    latheBox_->setObjectName(QStringLiteral("machineLatheBox"));
    auto* latheForm = new QFormLayout(latheBox_);
    liveTooling_ = new QCheckBox(tr("Live tooling"));
    liveTooling_->setObjectName(QStringLiteral("machineLiveTooling"));
    liveTooling_->setToolTip(tr("Driven tools in the turret, with the spindle as a C axis. Setups "
                                "on this machine can then drill with a live tool, on or off the "
                                "spindle's axis."));
    liveMaxRpm_ = spin(0, 200000, 6000, 0, tr(" rpm"));
    liveMaxRpm_->setObjectName(QStringLiteral("machineLiveMaxRpm"));
    liveMaxRpm_->setToolTip(tr("The fastest a live tool turns: a tool set faster is refused"));
    yAxis_ = new QCheckBox(tr("Y axis"));
    yAxis_->setObjectName(QStringLiteral("machineYAxis"));
    yAxis_->setToolTip(tr("Lifts the tool off the spindle's centre, for holes across the part that "
                          "do not cross its axis. Without it those are refused. It is a row in "
                          "the axes above."));
    latheForm->addRow(QString(), liveTooling_);
    latheForm->addRow(tr("Live tool max"), liveMaxRpm_);
    latheForm->addRow(QString(), yAxis_);
    buildLatheLayoutUi(latheForm);
    editorLayout->addWidget(latheBox_);
    connect(liveTooling_, &QCheckBox::toggled, this, [this] {
        updateLatheBox();
        if (!loading_) {
            markDirty();
        }
    });
    connect(liveMaxRpm_, &QDoubleSpinBox::valueChanged, this, &MachineLibraryDialog::markDirty);
    connect(yAxis_, &QCheckBox::toggled, this, [this](bool on) {
        if (!loading_) {
            setYAxis(on);
            markDirty();
        }
    });

    // The machine's collision geometry from its CAD comes first.
    buildModelUi(editorLayout);

    auto* axesBox = new QGroupBox(tr("Axes (travel: mm, rotary: degrees; rates per minute)"));
    auto* axesLayout = new QVBoxLayout(axesBox);
    axes_ = new QTableWidget(0, 4);
    axes_->setHorizontalHeaderLabels({tr("Axis"), tr("Min"), tr("Max"), tr("Max rate")});
    axes_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    axes_->verticalHeader()->hide();
    axes_->setMinimumHeight(170);
    axesLayout->addWidget(axes_);
    auto* axisButtons = new QHBoxLayout;
    auto* addAxis = new QPushButton(tr("Add axis"));
    auto* removeAxis = new QPushButton(tr("Remove axis"));
    axisButtons->addWidget(addAxis);
    axisButtons->addWidget(removeAxis);
    axisButtons->addStretch();
    axesLayout->addLayout(axisButtons);
    editorLayout->addWidget(axesBox);

    auto* limits = new QGroupBox(tr("Spindle, feeds and tool changer"));
    auto* limitsForm = new QFormLayout(limits);
    minRpm_ = spin(0, 200000, 0, 0, tr(" rpm"));
    maxRpm_ = spin(1, 200000, 10000, 0, tr(" rpm"));
    maxFeed_ = spin(1, 200000, 10000, 0, tr(" mm/min"));
    rapid_ = spin(1, 500000, 30000, 0, tr(" mm/min"));
    toolCapacity_ = new QSpinBox;
    toolCapacity_->setRange(0, 999);
    connect(toolCapacity_, &QSpinBox::valueChanged, this, [this](int n) {
        const QSignalBlocker block(stations_);
        stations_->setValue(std::max(n, 1));
    });
    toolChangeTime_ = spin(0, 600, 5, 1, tr(" s"));
    workOffsets_ = new QSpinBox;
    workOffsets_->setRange(1, 999);
    pivot_ = spin(0, 5000, 0, 3, tr(" mm"));
    pivot_->setToolTip(tr("Head machines: from the head's rotation centre to the spindle gauge "
                          "line. Used for machine positions without tool centre point control."));
    limitsForm->addRow(tr("Spindle min"), minRpm_);
    limitsForm->addRow(tr("Spindle max"), maxRpm_);
    limitsForm->addRow(tr("Max feed"), maxFeed_);
    limitsForm->addRow(tr("Rapid"), rapid_);
    limitsForm->addRow(tr("Tool capacity"), toolCapacity_);
    limitsForm->addRow(tr("Tool change time"), toolChangeTime_);
    limitsForm->addRow(tr("Work offsets"), workOffsets_);
    limitsForm->addRow(tr("Pivot length"), pivot_);
    editorLayout->addWidget(limits);

    // Boxes and cylinders: a quick stand-in where there is no CAD, folded away by default.
    simpleToggle_ = new QToolButton;
    simpleToggle_->setText(tr("Boxes and cylinders instead of CAD (optional)"));
    simpleToggle_->setCheckable(true);
    simpleToggle_->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    simpleToggle_->setArrowType(Qt::RightArrow);
    simpleToggle_->setAutoRaise(true);
    editorLayout->addWidget(simpleToggle_);
    auto* compBox = new QGroupBox(tr("Boxes and cylinders"));
    simpleShapes_ = compBox;
    auto* compLayout = new QVBoxLayout(compBox);
    auto* compHelp = new QLabel(
        tr("Simple solids for machines without CAD; imported parts above are exact. Boxes "
           "(P1 = min corner, P2 = max corner) and cylinders (P1 = base centre, P2 = axis, "
           "radius, length) in the frame of what carries them: <i>head</i> (origin at the spindle "
           "gauge point, +Z up the tool axis), <i>table</i> (the machine's axes, origin at the "
           "table centre, where the table's rotary axes meet; turns with them) or <i>cradle</i> "
           "(turns with the outer table axis only)."));
    compHelp->setWordWrap(true);
    compLayout->addWidget(compHelp);
    components_ = new QTableWidget(0, 11);
    components_->setHorizontalHeaderLabels({tr("Name"), tr("Carrier"), tr("Shape"), tr("P1 x"),
                                            tr("P1 y"), tr("P1 z"), tr("P2 x"), tr("P2 y"),
                                            tr("P2 z"), tr("Radius"), tr("Length")});
    components_->horizontalHeader()->setSectionResizeMode(QHeaderView::Interactive);
    components_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
    components_->verticalHeader()->hide();
    components_->setMinimumHeight(180);
    compLayout->addWidget(components_);
    auto* compButtons = new QHBoxLayout;
    auto* addBox = new QPushButton(tr("Add box"));
    auto* addCylinder = new QPushButton(tr("Add cylinder"));
    auto* removeComp = new QPushButton(tr("Remove"));
    compButtons->addWidget(addBox);
    compButtons->addWidget(addCylinder);
    compButtons->addWidget(removeComp);
    compButtons->addStretch();
    compLayout->addLayout(compButtons);
    editorLayout->addWidget(compBox);
    compBox->setVisible(false);
    connect(simpleToggle_, &QToolButton::toggled, this, [this](bool on) {
        simpleToggle_->setArrowType(on ? Qt::DownArrow : Qt::RightArrow);
        simpleShapes_->setVisible(on);
    });

    issues_ = new QLabel;
    issues_->setWordWrap(true);
    issues_->setStyleSheet(QStringLiteral("color: #d04040"));
    editorLayout->addWidget(issues_);
    scroll->setWidget(editor_);
    splitter->addWidget(scroll);
    splitter->setStretchFactor(1, 3);

    // Right: the preview, with sliders that jog the axes.
    auto* previewPane = new QWidget;
    auto* previewLayout = new QVBoxLayout(previewPane);
    previewLayout->setContentsMargins(0, 0, 0, 0);
    preview_ = new OcctView;
    preview_->setMinimumSize(360, 360);
    previewLayout->addWidget(preview_, 1);
    previewNote_ = new QLabel;
    previewNote_->setWordWrap(true);
    previewNote_->setStyleSheet(QStringLiteral("color: gray"));
    previewLayout->addWidget(previewNote_);
    auto* viewRow = new QHBoxLayout;
    auto* hideSelected = new QPushButton(tr("Hide selected"));
    auto* isolate = new QPushButton(tr("Isolate selected"));
    isolate->setToolTip(tr("Show only the selected parts, and frame them"));
    auto* showAll = new QPushButton(tr("Show all"));
    showAll->setToolTip(tr("Show every part, also those hidden by default"));
    viewRow->addWidget(hideSelected);
    viewRow->addWidget(isolate);
    viewRow->addWidget(showAll);
    enclosureBox_ = new QCheckBox(tr("Enclosure"));
    enclosureBox_->setObjectName(QStringLiteral("machinePreviewEnclosure"));
    enclosureBox_->setToolTip(tr("Show the machine's enclosure (its walls, sheet metal and way "
                                 "covers, as the machine library marks them); off: the rest of the "
                                 "machine without it. It is still checked for collisions."));
    enclosureBox_->setChecked(
        QSettings().value(QStringLiteral("machineLibraryEnclosure"), true).toBool());
    viewRow->addWidget(enclosureBox_);
    auto* viewOnly = new QLabel(tr("View only, not saved"));
    viewOnly->setStyleSheet(QStringLiteral("color: gray"));
    viewOnly->setToolTip(tr("These buttons change what the preview shows. The Shown column sets "
                            "what the machine shows by default, saved with it."));
    viewRow->addWidget(viewOnly, 1);
    previewLayout->addLayout(viewRow);
    connect(hideSelected, &QPushButton::clicked, this, [this] {
        for (const int i : selectedSceneParts()) {
            setViewShown(i, false);
        }
        schedulePreview();
    });
    connect(isolate, &QPushButton::clicked, this, &MachineLibraryDialog::isolateSelected);
    connect(enclosureBox_, &QCheckBox::toggled, this, [this](bool on) {
        QSettings().setValue(QStringLiteral("machineLibraryEnclosure"), on);
        schedulePreview();
    });
    connect(showAll, &QPushButton::clicked, this, [this] {
        std::fill(viewShown_.begin(), viewShown_.end(), 1);
        schedulePreview();
    });
    connect(preview_, &OcctView::machineShapeClicked, this,
            [this](int index, int type, int subIndex, double, double, double) {
                exactPick_ = ExactPick{index, static_cast<cad::SubShapeType>(type), subIndex};
            });
    connect(preview_, &OcctView::machinePartClicked, this, &MachineLibraryDialog::previewClicked);
    pickNote_ = new QLabel;
    pickNote_->setWordWrap(true);
    pickNote_->setStyleSheet(QStringLiteral("color: #e0a030"));
    pickNote_->hide();
    previewLayout->insertWidget(previewLayout->indexOf(previewNote_) + 1, pickNote_);
    auto* jogBox = new QGroupBox(tr("Jog the axes (preview only)"));
    jogForm_ = new QFormLayout(jogBox);
    previewLayout->addWidget(jogBox);
    splitter->addWidget(previewPane);
    splitter->setStretchFactor(2, 2);
    splitter->setSizes({300, 760, 500});
    previewTimer_ = new QTimer(this);
    previewTimer_->setSingleShot(true);
    previewTimer_->setInterval(30);
    connect(previewTimer_, &QTimer::timeout, this, &MachineLibraryDialog::updatePreview);
    connect(parts_, &QTableWidget::itemSelectionChanged, this,
            &MachineLibraryDialog::schedulePreview);
    connect(components_, &QTableWidget::itemSelectionChanged, this,
            &MachineLibraryDialog::schedulePreview);

    // Bottom row.
    auto* bottom = new QHBoxLayout;
    save_ = new QPushButton(tr("Save to library"));
    use_ = new QPushButton;
    use_->hide();
    auto* close = new QPushButton(tr("Close"));
    bottom->addWidget(save_);
    bottom->addStretch();
    bottom->addWidget(use_);
    bottom->addWidget(close);
    root->addLayout(bottom);

    connect(list_, &QListWidget::currentRowChanged, this,
            &MachineLibraryDialog::onCurrentRowChanged);
    connect(add, &QPushButton::clicked, this, &MachineLibraryDialog::addFromTemplate);
    connect(duplicate, &QPushButton::clicked, this, &MachineLibraryDialog::duplicateCurrent);
    connect(delete_, &QPushButton::clicked, this, &MachineLibraryDialog::deleteCurrent);
    connect(importButton, &QPushButton::clicked, this, &MachineLibraryDialog::importMachine);
    connect(exportButton, &QPushButton::clicked, this, &MachineLibraryDialog::exportCurrent);
    connect(save_, &QPushButton::clicked, this, [this] { saveCurrent(); });
    connect(close, &QPushButton::clicked, this, &MachineLibraryDialog::reject);
    connect(use_, &QPushButton::clicked, this, [this] {
        if (dirty_ && !saveCurrent()) {
            return;
        }
        if (const Item* item = currentItem(); item && useHandler_) {
            useHandler_(item->machine);
            accept();
        }
    });
    connect(addAxis, &QPushButton::clicked, this, [this] {
        const int row = axes_->rowCount();
        axes_->insertRow(row);
        axes_->setItem(row, 0, new QTableWidgetItem(QStringLiteral("B")));
        axes_->setItem(row, 1, numberItem(-120));
        axes_->setItem(row, 2, numberItem(120));
        axes_->setItem(row, 3, numberItem(10000));
        markDirty();
    });
    connect(removeAxis, &QPushButton::clicked, this, [this] {
        if (axes_->currentRow() >= 0) {
            axes_->removeRow(axes_->currentRow());
            markDirty();
        }
    });
    auto addComponent = [this](bool cylinder) {
        const int row = components_->rowCount();
        components_->insertRow(row);
        components_->setItem(row, 0, new QTableWidgetItem(cylinder ? tr("cylinder") : tr("box")));
        auto* attach = new QComboBox;
        for (const char* a : kAttach) {
            attach->addItem(QString::fromLatin1(a));
        }
        attach->setCurrentIndex(1);
        connect(attach, &QComboBox::currentIndexChanged, this, &MachineLibraryDialog::markDirty);
        components_->setCellWidget(row, 1, attach);
        components_->setItem(
            row, 2,
            new QTableWidgetItem(cylinder ? QStringLiteral("cylinder") : QStringLiteral("box")));
        components_->item(row, 2)->setFlags(Qt::ItemIsEnabled);
        const double values[] = {0, 0, 0, cylinder ? 0.0 : 10.0, cylinder ? 0.0 : 10.0, 10, 10, 10};
        for (int c = 0; c < 8; ++c) {
            components_->setItem(row, 3 + c, numberItem(values[c]));
        }
        markDirty();
    };
    connect(addBox, &QPushButton::clicked, this, [addComponent] { addComponent(false); });
    connect(addCylinder, &QPushButton::clicked, this, [addComponent] { addComponent(true); });
    connect(removeComp, &QPushButton::clicked, this, [this] {
        if (components_->currentRow() >= 0) {
            components_->removeRow(components_->currentRow());
            markDirty();
        }
    });

    for (QLineEdit* e : {id_, name_, vendor_}) {
        connect(e, &QLineEdit::textEdited, this, &MachineLibraryDialog::markDirty);
    }
    connect(kinematics_, &QComboBox::currentIndexChanged, this, [this] {
        if (!loading_) {
            ensureRotaryAxes();
            fillPosts(post_->currentData().toString().toStdString());
            updateLatheBox();
            updateStanding();
            markDirty();
            schedulePreview();
        }
    });
    for (QComboBox* c : {standUp_, standFront_}) {
        connect(c, &QComboBox::currentIndexChanged, this, [this] {
            if (!loading_) {
                updateStanding();
                markDirty();
                schedulePreview();
            }
        });
    }
    connect(post_, &QComboBox::currentIndexChanged, this, [this] {
        if (!loading_) {
            postOptions_ = Json::object(); // the values were the other post's
            markDirty();
        }
    });
    connect(postOptionsButton_, &QPushButton::clicked, this,
            &MachineLibraryDialog::configurePostOptions);
    for (QDoubleSpinBox* s : {minRpm_, maxRpm_, maxFeed_, rapid_, toolChangeTime_, pivot_}) {
        connect(s, &QDoubleSpinBox::valueChanged, this, &MachineLibraryDialog::markDirty);
    }
    for (QSpinBox* s : {toolCapacity_, workOffsets_}) {
        connect(s, &QSpinBox::valueChanged, this, &MachineLibraryDialog::markDirty);
    }
    connect(axes_, &QTableWidget::itemChanged, this, &MachineLibraryDialog::markDirty);
    connect(components_, &QTableWidget::itemChanged, this, &MachineLibraryDialog::markDirty);
}

void MachineLibraryDialog::fillPosts(const std::string& postId) {
    machine::Machine probe;
    probe.kinematics = static_cast<machine::Kinematics>(kinematics_->currentData().toInt());
    const std::string kinematics(machine::toString(probe.kinematics));
    auto fits = [&](const post::PostScript& p) {
        return std::ranges::find(p.manifest().machines, kinematics) != p.manifest().machines.end();
    };
    const QSignalBlocker block(post_);
    post_->clear();
    post_->addItem(tr("Built-in for its kinematics (%1)")
                       .arg(qs(document::defaultPostFor(probe).manifest().name)),
                   QString());
    for (const std::string& id : post::builtinPostIds()) {
        if (fits(post::builtinPost(id))) {
            post_->addItem(qs(post::builtinPost(id).manifest().name), qs(id));
        }
    }
    bool separated = false;
    for (const auto& entry : PostLibrary::load()) {
        const std::string& id = entry.post.manifest().id;
        if (fits(entry.post) && post_->findData(qs(id)) < 0) {
            if (!separated) {
                post_->insertSeparator(post_->count());
                separated = true;
            }
            post_->addItem(tr("Library: %1").arg(qs(entry.post.manifest().name)), qs(id));
        }
    }
    if (!postId.empty() && post_->findData(qs(postId)) < 0) {
        // Not a post for these kinematics, or not in the library: kept until changed.
        post_->addItem(tr("%1 (not available for these kinematics)").arg(qs(postId)), qs(postId));
    }
    post_->setCurrentIndex(std::max(0, post_->findData(qs(postId))));
}

void MachineLibraryDialog::configurePostOptions() {
    // The default post: the built-in post for the kinematics, a built-in post, or a library post.
    machine::Machine probe;
    probe.kinematics = static_cast<machine::Kinematics>(kinematics_->currentData().toInt());
    const std::string id = ss(post_->currentData().toString());
    std::optional<post::PostScript> post;
    const std::vector<std::string> builtins = post::builtinPostIds();
    if (id.empty()) {
        post = document::defaultPostFor(probe);
    } else if (std::ranges::find(builtins, id) != builtins.end()) {
        post = post::builtinPost(id);
    } else {
        for (auto& entry : PostLibrary::load()) {
            if (entry.post.manifest().id == id) {
                post = std::move(entry.post);
            }
        }
    }
    if (!post) {
        QMessageBox::information(this, tr("Post options"),
                                 tr("The post '%1' is not in your post library.").arg(qs(id)));
        return;
    }
    QDialog dialog(this);
    dialog.setWindowTitle(tr("%1: options of %2").arg(name_->text(), qs(post->manifest().name)));
    dialog.setMinimumWidth(560);
    auto* layout = new QVBoxLayout(&dialog);
    auto* note = new QLabel(tr("Setups on this machine start from these values when posted with "
                               "its default post; a setup's own values (Post Process) come "
                               "first."));
    note->setWordWrap(true);
    layout->addWidget(note);
    auto* form = new PostOptionsForm;
    const PostOptionsForm::Layer machineLayer{tr("machine"), tr("This machine's value"),
                                              postOptions_};
    const PostOptionsForm::Layer userLayer{tr("your default"),
                                           tr("Your saved default for this post"),
                                           PostLibrary::userDefaults(post->manifest().id)};
    form->show(&*post, {machineLayer, userLayer});
    layout->addWidget(form);
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    auto* reset = buttons->addButton(tr("Use defaults"), QDialogButtonBox::ResetRole);
    reset->setToolTip(tr("Drop the machine's values: your defaults, else the post's"));
    connect(reset, &QPushButton::clicked, &dialog, [&] { form->show(&*post, {userLayer}); });
    connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    layout->addWidget(buttons);
    if (dialog.exec() == QDialog::Accepted && form->values() != postOptions_) {
        postOptions_ = form->values();
        markDirty();
    }
}

void MachineLibraryDialog::setUseHandler(const QString& setupName,
                                         std::function<void(const machine::Machine&)> handler) {
    useHandler_ = std::move(handler);
    use_->setText(tr("Use for %1").arg(setupName));
    use_->setVisible(static_cast<bool>(useHandler_));
}

void MachineLibraryDialog::reloadList(const std::string& selectId) {
    items_.clear();
    for (const machine::Machine* m : machine::builtinMachines()) {
        items_.push_back({*m, true});
    }
    std::vector<std::string> problems;
    for (const auto& entry : MachineLibrary::load(&problems)) {
        items_.push_back({entry.machine, false});
    }
    {
        const QSignalBlocker block(list_);
        list_->clear();
        for (const Item& item : items_) {
            auto* row =
                new QListWidgetItem(item.builtin ? tr("%1  (built-in)").arg(qs(item.machine.name))
                                                 : qs(item.machine.name));
            row->setToolTip(qs(item.machine.id + " - " +
                               std::string(machine::toString(item.machine.kinematics))));
            if (item.builtin) {
                row->setForeground(Qt::gray);
            }
            list_->addItem(row);
        }
    }
    if (!problems.empty()) {
        QStringList text;
        for (const auto& p : problems) {
            text << qs(p);
        }
        QMessageBox::warning(
            this, tr("Machine library"),
            tr("Some machine files could not be read:\n\n%1").arg(text.join(QLatin1Char('\n'))));
    }
    int row = -1;
    for (std::size_t i = 0; i < items_.size(); ++i) {
        if (!selectId.empty() && items_[i].machine.id == selectId) {
            row = static_cast<int>(i);
            if (items_[i].builtin) {
                const QSignalBlocker block(showBuiltins_);
                showBuiltins_->setChecked(true); // what it is opened on is shown
            }
        }
    }
    for (std::size_t i = 0; i < items_.size(); ++i) {
        list_->item(static_cast<int>(i))
            ->setHidden(items_[i].builtin && !showBuiltins_->isChecked());
        if (row < 0 && !list_->item(static_cast<int>(i))->isHidden()) {
            row = static_cast<int>(i);
        }
    }
    currentRow_ = -1;
    dirty_ = false;
    list_->setCurrentRow(row);
    if (row >= 0 && currentRow_ != row) {
        onCurrentRowChanged(row);
    }
}

void MachineLibraryDialog::applyShowBuiltins() {
    const bool show = showBuiltins_->isChecked();
    int first = -1;
    for (std::size_t i = 0; i < items_.size(); ++i) {
        QListWidgetItem* row = list_->item(static_cast<int>(i));
        row->setHidden(items_[i].builtin && !show);
        if (first < 0 && !row->isHidden()) {
            first = static_cast<int>(i);
        }
    }
    QListWidgetItem* current = list_->currentItem();
    if ((!current || current->isHidden()) && first >= 0) {
        list_->setCurrentRow(first);
    }
}

void MachineLibraryDialog::selectMachine(const std::string& id) {
    for (std::size_t i = 0; i < items_.size(); ++i) {
        if (items_[i].machine.id == id) {
            if (items_[i].builtin && !showBuiltins_->isChecked()) {
                showBuiltins_->setChecked(true);
            }
            list_->setCurrentRow(static_cast<int>(i));
            return;
        }
    }
}

const MachineLibraryDialog::Item* MachineLibraryDialog::currentItem() const {
    return currentRow_ >= 0 && currentRow_ < static_cast<int>(items_.size())
               ? &items_[static_cast<std::size_t>(currentRow_)]
               : nullptr;
}

void MachineLibraryDialog::onCurrentRowChanged(int row) {
    if (row == currentRow_) {
        return;
    }
    if (dirty_ && !confirmDiscard()) {
        const QSignalBlocker block(list_);
        list_->setCurrentRow(currentRow_);
        return;
    }
    currentRow_ = row;
    if (const Item* item = currentItem()) {
        loadEditor(*item);
    }
    // No machine of the user's and the built-ins hidden: nothing to edit.
    scroll_->setEnabled(currentItem() != nullptr);
    if (!currentItem()) {
        updateIssues();
    }
}

void MachineLibraryDialog::loadEditor(const Item& item) {
    loading_ = true;
    const machine::Machine& m = item.machine;
    loadedId_ = item.builtin ? std::string() : m.id;
    id_->setText(qs(m.id));
    name_->setText(qs(m.name));
    vendor_->setText(qs(m.vendor));
    kinematics_->setCurrentIndex(
        std::max(0, kinematics_->findData(static_cast<int>(m.kinematics))));
    standUp_->setCurrentIndex(
        m.orientation ? std::max(0, standUp_->findData(static_cast<int>(m.orientation->up))) : 0);
    standFront_->setCurrentIndex(
        std::max(0, standFront_->findData(static_cast<int>(m.standing().front))));
    updateStanding();
    fillPosts(m.post);
    postOptions_ = m.postOptions.is_object() ? m.postOptions : Json::object();
    {
        const QSignalBlocker block(axes_);
        axes_->setRowCount(0);
        for (const auto& a : m.axes) {
            const int row = axes_->rowCount();
            axes_->insertRow(row);
            axes_->setItem(row, 0, new QTableWidgetItem(qs(a.name)));
            axes_->setItem(row, 1, numberItem(a.min));
            axes_->setItem(row, 2, numberItem(a.max));
            axes_->setItem(row, 3, numberItem(a.maxRate * kPerMinute));
        }
    }
    minRpm_->setValue(m.minSpindle * kPerMinute);
    maxRpm_->setValue(m.maxSpindle * kPerMinute);
    maxFeed_->setValue(m.maxFeed * kPerMinute);
    rapid_->setValue(m.rapidRate * kPerMinute);
    toolCapacity_->setValue(m.toolCapacity);
    toolChangeTime_->setValue(m.toolChangeTime);
    workOffsets_->setValue(m.workOffsets);
    pivot_->setValue(m.pivotLength);
    liveTooling_->setChecked(m.liveTooling);
    liveMaxRpm_->setValue(m.liveTooling ? m.liveMaxSpindle * kPerMinute : 6000.0);
    yAxis_->setChecked(m.hasYAxis());
    loadLatheLayout(m);
    updateLatheBox();
    {
        const QSignalBlocker block(components_);
        components_->setRowCount(0);
        for (const auto& c : m.components) {
            const int row = components_->rowCount();
            components_->insertRow(row);
            components_->setItem(row, 0, new QTableWidgetItem(qs(c.name)));
            auto* attach = new QComboBox;
            for (const char* a : kAttach) {
                attach->addItem(QString::fromLatin1(a));
            }
            attach->setCurrentIndex(static_cast<int>(c.attach));
            connect(attach, &QComboBox::currentIndexChanged, this,
                    &MachineLibraryDialog::markDirty);
            components_->setCellWidget(row, 1, attach);
            const bool cyl = c.shape == machine::Component::Shape::Cylinder;
            auto* shape =
                new QTableWidgetItem(cyl ? QStringLiteral("cylinder") : QStringLiteral("box"));
            shape->setFlags(Qt::ItemIsEnabled);
            components_->setItem(row, 2, shape);
            const Vec3 p1 = cyl ? c.base : c.min;
            const Vec3 p2 = cyl ? c.axis : c.max;
            for (int k = 0; k < 3; ++k) {
                components_->setItem(row, 3 + k, numberItem(p1[k]));
                components_->setItem(row, 6 + k, numberItem(p2[k]));
            }
            components_->setItem(row, 9, numberItem(cyl ? c.radius : 0.0));
            components_->setItem(row, 10, numberItem(cyl ? c.length : 0.0));
        }
    }
    // A built-in without a model of its own shows the axis tree it moves with.
    machine::Machine shown = m;
    if (item.builtin && shown.model.empty()) {
        shown.model = machine::defaultModel(shown);
    }
    loadModel(shown);
    simpleToggle_->setChecked(!m.components.empty());
    const bool editable = !item.builtin;
    readOnlyNote_->setVisible(!editable);
    for (QWidget* w : editor_->findChildren<QGroupBox*>()) {
        w->setEnabled(editable);
    }
    delete_->setEnabled(editable);
    loading_ = false;
    dirty_ = false;
    updateIssues();
    previewShown_ = false;
    viewShown_.clear();
    rebuildJog();
    schedulePreview();
}

void MachineLibraryDialog::updateStanding() {
    const auto kinematics = static_cast<machine::Kinematics>(kinematics_->currentData().toInt());
    const machine::MachineOrientation def = machine::defaultOrientation(kinematics);
    standUp_->setItemText(
        0, tr("As its kinematics (%1)").arg(qs(std::string(machine::toString(def.up)))));
    const bool own = standUp_->currentData().toInt() >= 0;
    if (!own) {
        // The kinematics' front goes with its up.
        const QSignalBlocker block(standFront_);
        standFront_->setCurrentIndex(
            std::max(0, standFront_->findData(static_cast<int>(def.front))));
    }
    standFront_->setEnabled(own);
}

machine::Machine MachineLibraryDialog::editedMachine() const {
    machine::Machine m;
    m.id = ss(id_->text());
    m.name = ss(name_->text());
    m.vendor = ss(vendor_->text());
    m.kinematics = static_cast<machine::Kinematics>(kinematics_->currentData().toInt());
    if (const int up = standUp_->currentData().toInt(); up >= 0) {
        m.orientation = machine::MachineOrientation{
            static_cast<machine::AxisSense>(up),
            static_cast<machine::AxisSense>(standFront_->currentData().toInt())};
    }
    m.post = ss(post_->currentData().toString());
    m.postOptions = postOptions_;
    for (int r = 0; r < axes_->rowCount(); ++r) {
        const QTableWidgetItem* nameItem = axes_->item(r, 0);
        m.axes.push_back({nameItem ? ss(nameItem->text().toUpper()) : std::string(),
                          cellNumber(axes_, r, 1), cellNumber(axes_, r, 2),
                          cellNumber(axes_, r, 3) / kPerMinute});
    }
    m.minSpindle = minRpm_->value() / kPerMinute;
    m.maxSpindle = maxRpm_->value() / kPerMinute;
    m.maxFeed = maxFeed_->value() / kPerMinute;
    m.rapidRate = rapid_->value() / kPerMinute;
    m.toolCapacity = toolCapacity_->value();
    m.toolChangeTime = toolChangeTime_->value();
    m.workOffsets = workOffsets_->value();
    m.pivotLength = pivot_->value();
    if (m.isLathe() && liveTooling_->isChecked()) {
        m.liveTooling = true;
        m.liveMaxSpindle = liveMaxRpm_->value() / kPerMinute;
    }
    if (m.isLathe()) {
        m.lathe = editedLatheLayout();
    }
    m.model = editedModel();
    for (int r = 0; r < components_->rowCount(); ++r) {
        machine::Component c;
        const QTableWidgetItem* nameItem = components_->item(r, 0);
        c.name = nameItem ? ss(nameItem->text()) : std::string();
        if (const auto* attach = qobject_cast<QComboBox*>(components_->cellWidget(r, 1))) {
            c.attach = static_cast<machine::Component::Attach>(attach->currentIndex());
        }
        const bool cyl = components_->item(r, 2) &&
                         components_->item(r, 2)->text() == QStringLiteral("cylinder");
        const Vec3 p1(cellNumber(components_, r, 3), cellNumber(components_, r, 4),
                      cellNumber(components_, r, 5));
        const Vec3 p2(cellNumber(components_, r, 6), cellNumber(components_, r, 7),
                      cellNumber(components_, r, 8));
        if (cyl) {
            c.shape = machine::Component::Shape::Cylinder;
            c.base = p1;
            c.axis = p2.norm() > 1e-12 ? Vec3(p2.normalized()) : Vec3::UnitZ();
            c.radius = cellNumber(components_, r, 9);
            c.length = cellNumber(components_, r, 10);
        } else {
            c.shape = machine::Component::Shape::Box;
            c.min = p1.cwiseMin(p2);
            c.max = p1.cwiseMax(p2);
        }
        m.components.push_back(std::move(c));
    }
    return m;
}

void MachineLibraryDialog::markDirty() {
    if (loading_) {
        return;
    }
    const Item* item = currentItem();
    if (!item || item->builtin) {
        return;
    }
    dirty_ = true;
    updateIssues();
    schedulePreview();
}

void MachineLibraryDialog::updateIssues() {
    const Item* item = currentItem();
    if (!item) {
        issues_->clear();
        save_->setEnabled(false);
        return;
    }
    if (item->builtin) {
        issues_->clear();
        save_->setEnabled(false);
        return;
    }
    const machine::Machine m = editedMachine();
    QStringList issues;
    if (!MachineLibrary::isUsableId(m.id)) {
        issues << tr("The id must be lower case letters, digits, '.', '_' or '-' and differ from "
                     "the built-in machines.");
    }
    if (m.name.empty()) {
        issues << tr("Give the machine a name.");
    }
    for (const auto& issue : m.validate()) {
        issues << qs(issue);
    }
    for (const auto& a : m.axes) {
        if (a.name.size() != 1 || std::string("XYZABC").find(a.name) == std::string::npos) {
            issues << tr("Axis names are X, Y, Z, A, B or C.");
            break;
        }
    }
    issues_->setText(issues.join(QLatin1Char('\n')));
    save_->setEnabled(dirty_ && issues.isEmpty());
    save_->setText(dirty_ ? tr("Save to library *") : tr("Save to library"));
}

bool MachineLibraryDialog::saveCurrent() {
    const Item* item = currentItem();
    if (!item || item->builtin) {
        return false;
    }
    const machine::Machine m = editedMachine();
    if (m.id != loadedId_) {
        const auto clash = std::find_if(items_.begin(), items_.end(), [&](const Item& i) {
            return !i.builtin && i.machine.id == m.id;
        });
        if (clash != items_.end()) {
            QMessageBox::warning(this, tr("Machine library"),
                                 tr("Another machine already uses the id '%1'.").arg(qs(m.id)));
            return false;
        }
    }
    try {
        MachineLibrary::save(m);
        if (!loadedId_.empty() && loadedId_ != m.id) {
            MachineLibrary::remove(loadedId_);
        }
    } catch (const Error& e) {
        QMessageBox::warning(this, tr("Machine library"), QString::fromUtf8(e.what()));
        return false;
    }
    dirty_ = false;
    reloadList(m.id);
    return true;
}

bool MachineLibraryDialog::confirmDiscard() {
    const auto answer =
        QMessageBox::question(this, tr("Machine library"), tr("Save the changes to this machine?"),
                              QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
    if (answer == QMessageBox::Save) {
        return saveCurrent();
    }
    if (answer == QMessageBox::Discard) {
        dirty_ = false;
        return true;
    }
    return false;
}

void MachineLibraryDialog::reject() {
    if (dirty_ && !confirmDiscard()) {
        return;
    }
    QDialog::reject();
}

std::string MachineLibraryDialog::uniqueId(const std::string& base) const {
    std::string id = base;
    for (int n = 2; !MachineLibrary::isUsableId(id) ||
                    std::any_of(items_.begin(), items_.end(),
                                [&](const Item& i) { return i.machine.id == id; });
         ++n) {
        id = base + "-" + std::to_string(n);
    }
    return id;
}

void MachineLibraryDialog::addFromTemplate() {
    if (dirty_ && !confirmDiscard()) {
        return;
    }
    QStringList names;
    std::vector<const machine::Machine*> templates = machine::builtinMachines();
    for (const machine::Machine* m : templates) {
        names << qs(m->name);
    }
    bool ok = false;
    const QString choice =
        QInputDialog::getItem(this, tr("New machine"), tr("Start from:"), names, 0, false, &ok);
    if (!ok) {
        return;
    }
    machine::Machine m = *templates[static_cast<std::size_t>(names.indexOf(choice))];
    m.id = uniqueId("my-" + m.id);
    m.name = "My " + m.name;
    try {
        MachineLibrary::save(m);
    } catch (const Error& e) {
        QMessageBox::warning(this, tr("Machine library"), QString::fromUtf8(e.what()));
        return;
    }
    reloadList(m.id);
}

void MachineLibraryDialog::duplicateCurrent() {
    const Item* item = currentItem();
    if (!item) {
        return;
    }
    if (dirty_ && !confirmDiscard()) {
        return;
    }
    machine::Machine m = item->machine;
    m.id = uniqueId(item->builtin ? "my-" + m.id : m.id + "-copy");
    m.name = item->builtin ? "My " + m.name : m.name + " (copy)";
    try {
        MachineLibrary::save(m);
    } catch (const Error& e) {
        QMessageBox::warning(this, tr("Machine library"), QString::fromUtf8(e.what()));
        return;
    }
    reloadList(m.id);
}

void MachineLibraryDialog::deleteCurrent() {
    const Item* item = currentItem();
    if (!item || item->builtin) {
        return;
    }
    if (QMessageBox::question(this, tr("Delete machine"),
                              tr("Delete '%1' from the library? Projects keep their own copies.")
                                  .arg(qs(item->machine.name))) != QMessageBox::Yes) {
        return;
    }
    MachineLibrary::remove(item->machine.id);
    dirty_ = false;
    reloadList();
}

void MachineLibraryDialog::importMachine() {
    if (dirty_ && !confirmDiscard()) {
        return;
    }
    const QString path = QFileDialog::getOpenFileName(this, tr("Import machine"), QString(),
                                                      tr("Machine definitions (*.json)"));
    if (path.isEmpty()) {
        return;
    }
    try {
        machine::Machine m =
            document::loadMachineFile(std::filesystem::path(path.toStdU16String()));
        if (!MachineLibrary::isUsableId(m.id) ||
            std::any_of(items_.begin(), items_.end(),
                        [&](const Item& i) { return i.machine.id == m.id; })) {
            m.id = uniqueId(MachineLibrary::isUsableId(m.id) ? m.id : "imported-" + m.id);
        }
        MachineLibrary::save(m);
        reloadList(m.id);
    } catch (const Error& e) {
        QMessageBox::warning(this, tr("Import machine"), QString::fromUtf8(e.what()));
    }
}

void MachineLibraryDialog::exportCurrent() {
    const Item* item = currentItem();
    if (!item) {
        return;
    }
    const machine::Machine m = item->builtin ? item->machine : editedMachine();
    const QString path =
        QFileDialog::getSaveFileName(this, tr("Export machine"), qs(m.id) + QStringLiteral(".json"),
                                     tr("Machine definitions (*.json)"));
    if (path.isEmpty()) {
        return;
    }
    try {
        // The part files (STEP, or STL for parts without one) go next to it (<name>.meshes).
        machine::saveMachine(m, std::filesystem::path(path.toStdU16String()));
    } catch (const Error& e) {
        QMessageBox::warning(this, tr("Export machine"), QString::fromUtf8(e.what()));
    }
}

void MachineLibraryDialog::updateLatheBox() {
    machine::Machine probe;
    probe.kinematics = static_cast<machine::Kinematics>(kinematics_->currentData().toInt());
    latheBox_->setVisible(probe.isLathe());
    liveMaxRpm_->setEnabled(liveTooling_->isChecked());
    layoutFields_->setVisible(layoutOn_->isChecked());
}

void MachineLibraryDialog::buildLatheLayoutUi(QFormLayout* latheForm) {
    layoutOn_ = new QCheckBox(tr("Simulate the machine"));
    layoutOn_->setObjectName(QStringLiteral("machineLatheLayout"));
    layoutOn_->setToolTip(
        tr("Show this lathe moving in the simulation and check its turret and every "
           "tool in it against the enclosure, chuck, work and tailstock. Sizes "
           "are enough: a part from the machine's CAD can stand in for any of "
           "them (set its size here to 0)."));
    latheForm->addRow(QString(), layoutOn_);
    layoutFields_ = new QWidget;
    auto* form = new QFormLayout(layoutFields_);
    form->setContentsMargins(0, 0, 0, 0);
    auto number = [&](const QString& label, const QString& tip,
                      std::function<double&(machine::LatheLayout&)> field, double lo = 0.0) {
        auto* box = spin(lo, 100000.0, 0.0, 3, tr(" mm"));
        box->setToolTip(tip);
        connect(box, &QDoubleSpinBox::valueChanged, this, &MachineLibraryDialog::markDirty);
        layoutNumbers_.emplace_back(std::move(field), box);
        form->addRow(label, box);
    };
    using L = machine::LatheLayout;
    const double any = -100000.0;
    form->addRow(new QLabel(tr("<b>Turret</b>")));
    stations_ = new QSpinBox;
    stations_->setObjectName(QStringLiteral("machineTurretStations"));
    stations_->setRange(1, 999);
    stations_->setToolTip(tr("How many stations it has, evenly spaced. A tool's number is its "
                             "station, station 1 working with the turret at 0. It is the machine's "
                             "tool capacity."));
    connect(stations_, &QSpinBox::valueChanged, this, [this](int n) {
        if (toolCapacity_->value() != n) {
            toolCapacity_->setValue(n);
        }
    });
    form->addRow(tr("Stations"), stations_);
    number(
        tr("Centre X at home"), tr("From the spindle's centre line to the index line, with X at 0"),
        [](L& l) -> double& { return l.turret.centre.x(); }, any);
    number(
        tr("Centre Y"), tr("Off the plane of the spindle, with Y at 0"),
        [](L& l) -> double& { return l.turret.centre.y(); }, any);
    number(
        tr("Centre Z at home"), tr("From the spindle's nose to the disc's middle, with Z at 0"),
        [](L& l) -> double& { return l.turret.centre.z(); }, any);
    number(tr("Disc diameter"), tr("0: a part of the machine's model is the disc"),
           [](L& l) -> double& { return l.turret.diameter; });
    number(tr("Disc width"), tr("Along the index line"),
           [](L& l) -> double& { return l.turret.width; });
    number(tr("Reach of a tool across"),
           tr("From the index line to the tip of a tool lying across the spindle"),
           [](L& l) -> double& { return l.turret.radialReach; });
    number(tr("Pitch of a tool along"),
           tr("From the index line to the centre line of a tool along the spindle"),
           [](L& l) -> double& { return l.turret.axialPitch; });
    number(tr("Reach of a tool along"),
           tr("From the disc's face to the tip of a tool along the spindle"),
           [](L& l) -> double& { return l.turret.axialReach; });
    number(tr("Block seat from the index line"),
           tr("Where a tooling block seats on a station: from the index line towards the spindle, "
              "in the working position. Every station has the same seat, turned about the line. "
              "Marked in the preview."),
           [](L& l) -> double& { return l.turret.blockRadius; });
    number(
        tr("Block seat across"),
        tr("Where a tooling block seats on a station, sideways: along the working station's Y "
           "from the index line's plane, for a turret whose station faces are shifted off the line "
           "through its centre (its centre off the spindle's plane). Turned with the station. A "
           "tool without a block stands as far sideways."),
        [](L& l) -> double& { return l.turret.blockAcross; }, any);
    number(
        tr("Block seat along Z"),
        tr("Where a tooling block seats on a station: along the index line from the turret's "
           "centre"),
        [](L& l) -> double& { return l.turret.blockAlong; }, any);
    blockMounts_ = new QLineEdit;
    blockMounts_->setObjectName(QStringLiteral("machineBlockMounts"));
    blockMounts_->setPlaceholderText(tr("Not stated: every block is offered"));
    blockMounts_->setToolTip(
        tr("The kinds of tooling block the turret takes, by name, separated by commas: VDI 40, "
           "BMT 55, or the maker's own. A setup offers the blocks that mount by one of them. "
           "Case and spacing do not matter."));
    connect(blockMounts_, &QLineEdit::textEdited, this, &MachineLibraryDialog::markDirty);
    form->addRow(tr("Blocks it takes"), blockMounts_);
    // A small table with rows added and removed by buttons.
    const auto table = [this](QTableWidget*& to, const QString& name, const QStringList& heads,
                              const QString& tip, const QString& addText) {
        auto* w = new QWidget;
        auto* v = new QVBoxLayout(w);
        v->setContentsMargins(0, 0, 0, 0);
        auto* t = new QTableWidget(0, static_cast<int>(heads.size()));
        to = t;
        t->setObjectName(name);
        t->setHorizontalHeaderLabels(heads);
        t->horizontalHeader()->setStretchLastSection(true);
        t->verticalHeader()->hide();
        t->setToolTip(tip);
        t->setMaximumHeight(130);
        connect(t, &QTableWidget::itemChanged, this, &MachineLibraryDialog::markDirty);
        v->addWidget(t);
        auto* buttons = new QHBoxLayout;
        auto* add = new QPushButton(addText);
        add->setObjectName(name + QStringLiteral("Add"));
        auto* remove = new QPushButton(tr("Remove"));
        connect(add, &QPushButton::clicked, this, [this, t] {
            const QSignalBlocker block(t);
            const int r = t->rowCount();
            t->insertRow(r);
            for (int c = 0; c < t->columnCount(); ++c) {
                t->setItem(r, c, new QTableWidgetItem);
            }
            markDirty();
        });
        connect(remove, &QPushButton::clicked, this, [this, t] {
            if (t->currentRow() >= 0) {
                t->removeRow(t->currentRow());
                markDirty();
            }
        });
        buttons->addWidget(add);
        buttons->addWidget(remove);
        buttons->addStretch(1);
        v->addLayout(buttons);
        return w;
    };
    form->addRow(tr("Stations that differ"),
                 table(stationMounts_, QStringLiteral("machineStationMounts"),
                       {tr("Stations"), tr("Blocks they take")},
                       tr("Stations that take other kinds of block than the rest, such as the "
                          "milling stations of a turret with turning ones: the stations by number "
                          "(2, 4, 6) and the kinds they take, with commas. A station in no row "
                          "takes what Blocks it takes says."),
                       tr("Add stations")));
    indexAtHome_ = new QCheckBox(tr("Index at home"));
    indexAtHome_->setToolTip(tr("The turret goes home (machine zero) before it indexes, and the "
                                "simulation checks it there. Off: it indexes where the last "
                                "operation ended. The post must do the same: the generic lathe "
                                "posts index where they are."));
    connect(indexAtHome_, &QCheckBox::toggled, this, &MachineLibraryDialog::markDirty);
    form->addRow(QString(), indexAtHome_);
    number(tr("Clearance"), tr("What the turret and its tools must keep from anything"),
           [](L& l) -> double& { return l.clearance; });
    form->addRow(new QLabel(tr("<b>Spindle</b> (its nose at Z 0, 0 for what is not here)")));
    number(tr("Headstock diameter"), QString(),
           [](L& l) -> double& { return l.headstockDiameter; });
    number(tr("Headstock length"), tr("Behind the nose"),
           [](L& l) -> double& { return l.headstockLength; });
    number(
        tr("Workholding mount"),
        tr("Where a setup's workholding seats on the spindle: its back this far along Z from the "
           "nose (the front of a back plate), on the spindle's axis, turning with C. Chucks come "
           "from the workholding library, so another chuck is another setup choice. The chuck "
           "below stands here too."),
        [](L& l) -> double& { return l.workholdingMount; }, any);
    number(tr("Chuck diameter"), tr("For setups with no chuck of their own"),
           [](L& l) -> double& { return l.chuckDiameter; });
    number(tr("Chuck length"), tr("In front of the nose"),
           [](L& l) -> double& { return l.chuckLength; });
    number(tr("Chuck grip"), tr("How far the bar goes into the chuck"),
           [](L& l) -> double& { return l.chuckGrip; });
    form->addRow(new QLabel(tr("<b>Enclosure</b> (its inside; all 0 for none)")));
    number(tr("X, door side"), QString(), [](L& l) -> double& { return l.enclosureMin.x(); }, any);
    number(tr("X, back"), QString(), [](L& l) -> double& { return l.enclosureMax.x(); }, any);
    number(tr("Y, floor"), QString(), [](L& l) -> double& { return l.enclosureMin.y(); }, any);
    number(tr("Y, roof"), QString(), [](L& l) -> double& { return l.enclosureMax.y(); }, any);
    number(
        tr("Z, headstock end"), QString(), [](L& l) -> double& { return l.enclosureMin.z(); }, any);
    number(tr("Z, far end"), QString(), [](L& l) -> double& { return l.enclosureMax.z(); }, any);
    number(tr("Wall"), tr("How thick the walls are drawn"), [](L& l) -> double& { return l.wall; });
    tailstockOn_ = new QCheckBox(tr("Tailstock"));
    connect(tailstockOn_, &QCheckBox::toggled, this, &MachineLibraryDialog::markDirty);
    form->addRow(QString(), tailstockOn_);
    // Kept in the layout's first tailstock and sub-spindle while editing: see editedLatheLayout.
    number(
        tr("Tailstock front"), tr("At W 0: home, in the model frame"),
        [](L& l) -> double& { return l.tailstock->z; }, any);
    number(tr("Tailstock diameter"), QString(),
           [](L& l) -> double& { return l.tailstock->diameter; });
    number(tr("Tailstock length"), QString(), [](L& l) -> double& { return l.tailstock->length; });
    quillInterface_ = new QLineEdit;
    quillInterface_->setObjectName(QStringLiteral("machineQuillInterface"));
    quillInterface_->setPlaceholderText(tr("Not stated: every centre is offered"));
    quillInterface_->setToolTip(tr("What the quill takes, by name: MT 4, or the maker's own. A "
                                   "setup offers the centres that fit it."));
    connect(quillInterface_, &QLineEdit::textEdited, this, &MachineLibraryDialog::markDirty);
    form->addRow(tr("Quill takes"), quillInterface_);
    number(tr("Quill travel"),
           tr("How far the quill goes out of the tailstock, its stroke. A setup whose quill is out "
              "further is said. 0: not stated, and nothing is said."),
           [](L& l) -> double& { return l.tailstock->quillTravel; });
    layoutNumbers_.back().second->setObjectName(QStringLiteral("machineQuillTravel"));
    layoutNumbers_.back().second->setSpecialValueText(tr("Not stated"));
    subOn_ = new QCheckBox(tr("Sub-spindle"));
    connect(subOn_, &QCheckBox::toggled, this, &MachineLibraryDialog::markDirty);
    form->addRow(QString(), subOn_);
    number(
        tr("Sub-spindle nose"), tr("At Z2 0: home, facing the main spindle"),
        [](L& l) -> double& { return l.subSpindle->nose; }, any);
    number(tr("Sub-spindle diameter"), QString(),
           [](L& l) -> double& { return l.subSpindle->diameter; });
    number(tr("Sub-spindle length"), tr("Behind its nose"),
           [](L& l) -> double& { return l.subSpindle->length; });
    number(tr("Sub-spindle chuck diameter"), QString(),
           [](L& l) -> double& { return l.subSpindle->chuckDiameter; });
    number(tr("Sub-spindle chuck length"), QString(),
           [](L& l) -> double& { return l.subSpindle->chuckLength; });
    number(tr("Sub-spindle grip"), QString(), [](L& l) -> double& { return l.subSpindle->grip; });
    number(
        tr("Z2 while it cuts"), QString(), [](L& l) -> double& { return l.subSpindle->workZ2; },
        any);
    // Its speed range: a transfer's speed outside it is refused, never changed (ADR-0007).
    subMinRpm_ = spin(0.0, 100000.0, 0.0, 0, tr(" rpm"));
    subMinRpm_->setObjectName(QStringLiteral("machineSubMinRpm"));
    subMinRpm_->setToolTip(tr("The sub-spindle's lowest speed"));
    connect(subMinRpm_, &QDoubleSpinBox::valueChanged, this, &MachineLibraryDialog::markDirty);
    form->addRow(tr("Sub-spindle lowest speed"), subMinRpm_);
    subMaxRpm_ = spin(0.0, 100000.0, 0.0, 0, tr(" rpm"));
    subMaxRpm_->setObjectName(QStringLiteral("machineSubMaxRpm"));
    subMaxRpm_->setSpecialValueText(tr("Not stated"));
    subMaxRpm_->setToolTip(tr("The sub-spindle's highest speed. 0: not stated, and nothing is "
                              "checked against it"));
    connect(subMaxRpm_, &QDoubleSpinBox::valueChanged, this, &MachineLibraryDialog::markDirty);
    form->addRow(tr("Sub-spindle highest speed"), subMaxRpm_);
    latheForm->addRow(layoutFields_);
    connect(layoutOn_, &QCheckBox::toggled, this, [this](bool on) {
        if (on && !loading_ && layoutNumbers_.front().second->value() == 0.0) {
            // A place to start: the generic lathe's numbers, a placeholder like its others.
            const bool wasLoading = loading_;
            loading_ = true;
            machine::Machine start = machine::genericLatheTurret();
            loadLatheLayout(start);
            loading_ = wasLoading;
            layoutOn_->setChecked(true);
        }
        updateLatheBox();
        if (!loading_) {
            markDirty();
        }
    });
}

void MachineLibraryDialog::loadLatheLayout(const machine::Machine& m) {
    machine::LatheLayout l = m.lathe.value_or(machine::LatheLayout{});
    layoutOn_->setChecked(m.lathe.has_value());
    indexAtHome_->setChecked(l.turret.indexAtHome);
    tailstockOn_->setChecked(l.tailstock.has_value());
    subOn_->setChecked(l.subSpindle.has_value());
    l.tailstock = l.tailstock.value_or(machine::LatheTailstockBody{});
    l.subSpindle = l.subSpindle.value_or(machine::LatheSubSpindle{});
    {
        QStringList names;
        for (const std::string& name : l.turret.blockMounts) {
            names << qs(name);
        }
        blockMounts_->setText(names.join(QStringLiteral(", ")));
        quillInterface_->setText(qs(l.tailstock->quillInterface));
        const QSignalBlocker b1(stationMounts_);
        stationMounts_->setRowCount(0);
        for (const machine::StationMounts& g : l.turret.stationMounts) {
            QStringList stations;
            for (const int s : g.stations) {
                stations << QString::number(s);
            }
            QStringList kinds;
            for (const std::string& k : g.blockMounts) {
                kinds << qs(k);
            }
            const int r = stationMounts_->rowCount();
            stationMounts_->insertRow(r);
            stationMounts_->setItem(r, 0,
                                    new QTableWidgetItem(stations.join(QStringLiteral(", "))));
            stationMounts_->setItem(r, 1, new QTableWidgetItem(kinds.join(QStringLiteral(", "))));
        }
    }
    for (auto& [field, box] : layoutNumbers_) {
        const QSignalBlocker block(box);
        box->setValue(field(l));
    }
    const QSignalBlocker b2(subMinRpm_);
    const QSignalBlocker b3(subMaxRpm_);
    subMinRpm_->setValue(l.subSpindle->minSpindle * 60.0);
    subMaxRpm_->setValue(l.subSpindle->maxSpindle * 60.0);
}

std::optional<machine::LatheLayout> MachineLibraryDialog::editedLatheLayout() const {
    if (!layoutOn_->isChecked()) {
        return std::nullopt;
    }
    machine::LatheLayout l;
    l.tailstock = machine::LatheTailstockBody{};
    l.subSpindle = machine::LatheSubSpindle{};
    for (const auto& [field, box] : layoutNumbers_) {
        field(l) = box->value();
    }
    l.turret.indexAtHome = indexAtHome_->isChecked();
    for (const QString& name : blockMounts_->text().split(QLatin1Char(','))) {
        if (const QString clean = name.simplified(); !clean.isEmpty()) {
            l.turret.blockMounts.push_back(ss(clean));
        }
    }
    l.tailstock->quillInterface = ss(quillInterface_->text().simplified());
    const auto text = [](const QTableWidget* t, int r, int c) {
        const QTableWidgetItem* item = t->item(r, c);
        return item ? item->text() : QString();
    };
    for (int r = 0; r < stationMounts_->rowCount(); ++r) {
        machine::StationMounts g;
        static const QRegularExpression separators(QStringLiteral("[,;\\s]+"));
        for (const QString& s : text(stationMounts_, r, 0).split(separators, Qt::SkipEmptyParts)) {
            g.stations.push_back(s.toInt()); // not a number: 0, which the machine's check says
        }
        for (const QString& k : text(stationMounts_, r, 1).split(QLatin1Char(','))) {
            if (const QString clean = k.simplified(); !clean.isEmpty()) {
                g.blockMounts.push_back(ss(clean));
            }
        }
        if (!g.stations.empty() || !g.blockMounts.empty()) {
            l.turret.stationMounts.push_back(std::move(g));
        }
    }
    if (!tailstockOn_->isChecked()) {
        l.tailstock.reset();
    }
    l.subSpindle->minSpindle = subMinRpm_->value() / 60.0;
    l.subSpindle->maxSpindle = subMaxRpm_->value() / 60.0;
    if (!subOn_->isChecked()) {
        l.subSpindle.reset();
    }
    return l;
}

void MachineLibraryDialog::setYAxis(bool on) {
    const QSignalBlocker block(axes_);
    int row = -1;
    for (int r = 0; r < axes_->rowCount(); ++r) {
        if (axes_->item(r, 0) && ss(axes_->item(r, 0)->text()) == "Y") {
            row = r;
        }
    }
    if (on && row < 0) {
        row = axes_->rowCount();
        axes_->insertRow(row);
        axes_->setItem(row, 0, new QTableWidgetItem(QStringLiteral("Y")));
        axes_->setItem(row, 1, numberItem(-50));
        axes_->setItem(row, 2, numberItem(50));
        axes_->setItem(row, 3, numberItem(10000));
    } else if (!on && row >= 0) {
        axes_->removeRow(row);
    }
}

void MachineLibraryDialog::ensureRotaryAxes() {
    // Add the axes the kinematics type needs (with typical limits) if they are missing.
    machine::Machine probe;
    probe.kinematics = static_cast<machine::Kinematics>(kinematics_->currentData().toInt());
    std::vector<std::string> wanted = probe.isLathe() ? std::vector<std::string>{"X", "Z"}
                                                      : std::vector<std::string>{"X", "Y", "Z"};
    for (const char c : probe.rotaryAxes()) {
        wanted.emplace_back(1, c);
    }
    const QSignalBlocker block(axes_);
    for (const auto& name : wanted) {
        bool present = false;
        for (int r = 0; r < axes_->rowCount(); ++r) {
            present = present || (axes_->item(r, 0) && ss(axes_->item(r, 0)->text()) == name);
        }
        if (present) {
            continue;
        }
        const bool rotary = name == "A" || name == "B" || name == "C";
        const std::string rotaries = probe.rotaryAxes();
        const bool rotating = !rotaries.empty() && name == rotaries.substr(rotaries.size() - 1);
        const int row = axes_->rowCount();
        axes_->insertRow(row);
        axes_->setItem(row, 0, new QTableWidgetItem(qs(name)));
        axes_->setItem(row, 1, numberItem(rotary ? (rotating ? -9999 : -120) : -300));
        axes_->setItem(row, 2, numberItem(rotary ? (rotating ? 9999 : 120) : 300));
        axes_->setItem(row, 3, numberItem(rotary ? 10000 : 30000));
    }
}

// -- Machine model (DL-0037) -------------------------------------------------------------------

namespace {

const std::string kJogOrder[] = {"X", "Y", "Z", "A", "B", "C", "T", "W", "Z2", "C2"};
const char* kAxisNames[] = {"X", "Y", "Z", "A", "B", "C", "T", "W", "Z2", "C2"};

enum AxisColumn { kAxisName, kAxisParent, kDirX, kDirY, kDirZ, kPivotX, kPivotY, kPivotZ };
enum PartColumn {
    kPartName,
    kPartCarrier,
    kOffX,
    kOffY,
    kOffZ,
    kRotX,
    kRotY,
    kRotZ,
    kPartColour,
    kPartCollides,
    kPartShown,
    kPartEnclosure,
    kPartTriangles
};

QComboBox* carrierCombo(const std::vector<std::string>& axes, const std::string& current) {
    auto* combo = new QComboBox;
    combo->addItem(QObject::tr("Base"), QString());
    for (const auto& a : axes) {
        combo->addItem(QString::fromStdString(a), QString::fromStdString(a));
    }
    combo->setCurrentIndex(std::max(0, combo->findData(QString::fromStdString(current))));
    return combo;
}

std::string comboData(const QComboBox* combo) {
    return combo ? combo->currentData().toString().toStdString() : std::string();
}

QString colourText(std::uint32_t rgb) {
    return QStringLiteral("#%1").arg(rgb & 0xFFFFFFu, 6, 16, QLatin1Char('0'));
}

QTableWidgetItem* colourItem(std::uint32_t rgb) {
    auto* item = new QTableWidgetItem(colourText(rgb));
    item->setBackground(QColor::fromRgb(rgb & 0xFFFFFFu));
    item->setForeground(qGray(static_cast<QRgb>(rgb)) > 128 ? Qt::black : Qt::white);
    return item;
}

QDoubleSpinBox* coordinate() {
    auto* s = spin(-1e5, 1e5, 0, 3, QString());
    s->setMinimumWidth(80);
    s->setMaximumWidth(110);
    return s;
}

/// A check box centred in a table cell. A cell widget, so that clicking it keeps the rows that
/// are selected (several rows change together).
QWidget* checkCell(bool checked) {
    auto* holder = new QWidget;
    auto* layout = new QHBoxLayout(holder);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setAlignment(Qt::AlignCenter);
    auto* box = new QCheckBox;
    box->setChecked(checked);
    layout->addWidget(box);
    return holder;
}

QCheckBox* checkIn(const QTableWidget* table, int row, int column) {
    const QWidget* cell = table->cellWidget(row, column);
    return cell ? cell->findChild<QCheckBox*>() : nullptr;
}

} // namespace

void MachineLibraryDialog::buildModelUi(QVBoxLayout* editorLayout) {
    // Outside the group so it also works on the read-only built-ins (it duplicates them first).
    auto* importRow = new QHBoxLayout;
    auto* importButton = new QPushButton(tr("Import STEP / STL parts..."));
    QFont bold = importButton->font();
    bold.setBold(true);
    importButton->setFont(bold);
    importButton->setMinimumHeight(32);
    importButton->setToolTip(tr("The machine's CAD as its collision geometry: each body of the "
                                "file (base, enclosure, tables, spindle head...) becomes a part, "
                                "placed as the file places it"));
    importRow->addWidget(importButton);
    auto* importHelp = new QLabel(
        tr("Build the machine from its maker's CAD: import its STEP file (every body becomes a "
           "part in its place), then say which axis carries each part."));
    importHelp->setWordWrap(true);
    importRow->addWidget(importHelp, 1);
    editorLayout->addLayout(importRow);

    auto* box =
        new QGroupBox(tr("Machine model: parts and axes (collision geometry and simulation)"));
    modelBox_ = box;
    auto* layout = new QVBoxLayout(box);

    parts_ = new QTableWidget(0, 13);
    parts_->setHorizontalHeaderLabels({tr("Part"), tr("Carried by"), tr("X"), tr("Y"), tr("Z"),
                                       tr("Rot X°"), tr("Rot Y°"), tr("Rot Z°"), tr("Colour"),
                                       tr("Collides"), tr("Shown"), tr("Enclosure"),
                                       tr("Triangles")});
    parts_->horizontalHeaderItem(kPartEnclosure)
        ->setToolTip(tr("Of the machine's enclosure (sheet metal, way covers, walls): the views' "
                        "Enclosure box hides these together, keeping the rest of the machine. "
                        "They are still checked for collisions. A layout's enclosure walls are "
                        "always enclosure."));
    parts_->horizontalHeaderItem(kPartShown)
        ->setToolTip(tr("Saved with the machine: drawn in the views by default (hide an enclosure "
                        "to see inside); hidden parts are still checked for collisions. The "
                        "preview's Hide / Isolate / Show all and the simulation's part list change "
                        "the view only."));
    parts_->horizontalHeader()->setSectionResizeMode(QHeaderView::Interactive);
    parts_->horizontalHeader()->setSectionResizeMode(kPartName, QHeaderView::Stretch);
    parts_->verticalHeader()->hide();
    parts_->setSelectionBehavior(QAbstractItemView::SelectRows);
    parts_->setSelectionMode(QAbstractItemView::ExtendedSelection);
    parts_->setMinimumHeight(220);
    parts_->setToolTip(tr("Placement at machine zero: the mesh turned about X, then Y, then Z, "
                          "then moved by X Y Z (imported assemblies need none)"));
    layout->addWidget(parts_, 1);
    auto* partButtons = new QHBoxLayout;
    auto* merge = new QPushButton(tr("Merge selected"));
    merge->setToolTip(tr("One part of the selected ones (things that move together)"));
    partButtons->addWidget(merge);
    auto* removePart = new QPushButton(tr("Remove selected"));
    partButtons->addWidget(removePart);
    partButtons->addWidget(new QLabel(tr("Mesh tolerance")));
    meshTolerance_ = new QDoubleSpinBox;
    meshTolerance_->setRange(0.01, 5.0);
    meshTolerance_->setDecimals(2);
    meshTolerance_->setSingleStep(0.05);
    meshTolerance_->setSuffix(tr(" mm"));
    meshTolerance_->setValue(0.2);
    meshTolerance_->setKeyboardTracking(false);
    meshTolerance_->setToolTip(
        tr("Largest distance of the meshes from the STEP parts' exact surfaces: the meshes the "
           "views and the collision checks use, made from the STEP files when the machine is "
           "loaded. Coarser meshes check faster; 0.2 mm suits most machines. Parts imported from "
           "STL keep their own mesh."));
    partButtons->addWidget(meshTolerance_);
    connect(meshTolerance_, &QDoubleSpinBox::valueChanged, this, [this](double tolerance) {
        for (machine::ModelPart& p : partData_) {
            if (p.exact()) {
                try {
                    p.geometry = document::stepMesh({p.step, p.stepFile}, tolerance, p.name);
                } catch (const Error&) {
                    // Keeps its mesh.
                }
            }
        }
        markDirty();
        schedulePreview();
    });
    auto* multiHint = new QLabel(tr("With several rows selected, a change to Carried by, Collides "
                                    "or Shown in one of them applies to all of them."));
    multiHint->setWordWrap(true);
    multiHint->setStyleSheet(QStringLiteral("color: gray"));
    partButtons->addWidget(multiHint, 1);
    layout->addLayout(partButtons);

    // The kinematic tree.
    auto* axesBox = new QGroupBox(tr("Kinematic tree: what each axis carries"));
    auto* axesLayout = new QVBoxLayout(axesBox);
    auto* help = new QLabel(
        tr("Parts stand as they do with every axis at 0 (machine zero). Each axis moves what it "
           "carries: a linear axis along its direction, a rotary axis right-handed about its "
           "direction through its pivot. An axis that moves the table points against its machine "
           "axis (X moving the table: direction -X). <i>Default axes</i> fills in a tree that fits "
           "the kinematics."));
    help->setWordWrap(true);
    axesLayout->addWidget(help);

    modelAxes_ = new QTableWidget(0, 8);
    modelAxes_->setHorizontalHeaderLabels({tr("Axis"), tr("Carried by"), tr("Dir x"), tr("Dir y"),
                                           tr("Dir z"), tr("Pivot x"), tr("Pivot y"),
                                           tr("Pivot z")});
    modelAxes_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    modelAxes_->verticalHeader()->hide();
    modelAxes_->setMinimumHeight(170);
    axesLayout->addWidget(modelAxes_);
    auto* axisButtons = new QHBoxLayout;
    auto* defaults = new QPushButton(tr("Default axes"));
    defaults->setToolTip(tr("Replace the axes, spindle and table with a tree that fits the "
                            "kinematics (the parts stay)"));
    auto* addAxis = new QPushButton(tr("Add axis"));
    auto* removeAxis = new QPushButton(tr("Remove axis"));
    auto* clear = new QPushButton(tr("No model"));
    clear->setToolTip(tr("Remove the axes and parts: the machine moves by its kinematics only"));
    axisButtons->addWidget(defaults);
    axisButtons->addWidget(addAxis);
    axisButtons->addWidget(removeAxis);
    axisButtons->addWidget(clear);
    pickFace_ = new QPushButton(tr("Pick axis on a round face..."));
    pickFace_->setCheckable(true);
    pickFace_->setToolTip(tr("For the selected rotary axis: click a cylindrical or conical face "
                             "of a part in the preview (a faceplate's rim, a chuck body). The "
                             "axis's pivot and direction become that face's centre line, at "
                             "machine zero."));
    pickRim_ = new QPushButton(tr("Pick 3 rim points..."));
    pickRim_->setCheckable(true);
    pickRim_->setToolTip(tr("For the selected rotary axis: click three points on a circular rim "
                            "in the preview; the axis goes through the circle's centre, square "
                            "to it."));
    axisButtons->addWidget(pickFace_);
    axisButtons->addWidget(pickRim_);
    axisButtons->addStretch();
    axesLayout->addLayout(axisButtons);
    layout->addWidget(axesBox);

    // Where the tool is on the machine (where the work sits is each setup's placement).
    auto* pointsBox = new QGroupBox(tr("Spindle"));
    auto* grid = new QGridLayout(pointsBox);
    grid->setColumnStretch(9, 1);
    auto pointRow = [&](int row, const QString& title, QComboBox*& carrier,
                        std::array<QDoubleSpinBox*, 3>& point, QPushButton*& pick,
                        const QString& pickText, const QString& explanation) {
        auto* name = new QLabel(QStringLiteral("<b>%1</b>").arg(title));
        grid->addWidget(name, row, 0);
        grid->addWidget(new QLabel(tr("carried by")), row, 1);
        carrier = carrierCombo({}, {});
        grid->addWidget(carrier, row, 2);
        for (int k = 0; k < 3; ++k) {
            point[static_cast<std::size_t>(k)] = coordinate();
            grid->addWidget(new QLabel(QStringLiteral("XYZ").mid(k, 1)), row, 3 + 2 * k,
                            Qt::AlignRight);
            grid->addWidget(point[static_cast<std::size_t>(k)], row, 4 + 2 * k);
            connect(point[static_cast<std::size_t>(k)], &QDoubleSpinBox::valueChanged, this,
                    &MachineLibraryDialog::markDirty);
        }
        pick = new QPushButton(pickText);
        pick->setCheckable(true);
        grid->addWidget(pick, row, 9, Qt::AlignLeft);
        auto* note = new QLabel(explanation);
        note->setWordWrap(true);
        note->setStyleSheet(QStringLiteral("color: gray"));
        grid->addWidget(note, row + 1, 0, 1, 10);
        connect(carrier, &QComboBox::currentIndexChanged, this, &MachineLibraryDialog::markDirty);
    };
    pointRow(0, tr("Spindle gauge point"), spindleCarrier_, spindlePoint_, pickGauge_,
             tr("Pick gauge point..."),
             tr("Where the tool holder seats in the spindle, with every axis at 0. The tool hangs "
                "below it along the spindle axis (+Z points up the tool)."));
    pickGauge_->setToolTip(tr("Click a round face of the spindle in the preview (the nose's bore, "
                              "taper or outside) for its centre line, then the face the holder "
                              "seats against for the height: the gauge point, at machine zero."));
    layout->addWidget(pointsBox);
    editorLayout->addWidget(box, 1);

    connect(defaults, &QPushButton::clicked, this, &MachineLibraryDialog::fillDefaultAxes);
    connect(pickFace_, &QPushButton::clicked, this,
            [this](bool on) { on ? startAxisPick(AxisPick::Face) : stopAxisPick(); });
    connect(pickGauge_, &QPushButton::clicked, this,
            [this](bool on) { on ? startGaugePick() : stopAxisPick(); });
    connect(pickRim_, &QPushButton::clicked, this,
            [this](bool on) { on ? startAxisPick(AxisPick::Rim) : stopAxisPick(); });
    connect(modelAxes_, &QTableWidget::itemSelectionChanged, this,
            &MachineLibraryDialog::schedulePreview);
    connect(addAxis, &QPushButton::clicked, this, [this] {
        const auto names = modelAxisNames();
        std::string next = "X";
        for (const char* n : kAxisNames) {
            if (std::find(names.begin(), names.end(), n) == names.end()) {
                next = n;
                break;
            }
        }
        machine::ModelAxis a;
        a.name = next;
        addModelAxisRow(a);
        refreshCarriers();
        markDirty();
    });
    connect(removeAxis, &QPushButton::clicked, this, [this] {
        if (modelAxes_->currentRow() >= 0) {
            modelAxes_->removeRow(modelAxes_->currentRow());
            refreshCarriers();
            markDirty();
        }
    });
    connect(clear, &QPushButton::clicked, this, [this] {
        modelAxes_->setRowCount(0);
        parts_->setRowCount(0);
        partData_.clear();
        refreshCarriers();
        markDirty();
    });
    connect(importButton, &QPushButton::clicked, this, &MachineLibraryDialog::importParts);
    connect(merge, &QPushButton::clicked, this, &MachineLibraryDialog::mergeSelectedParts);
    connect(removePart, &QPushButton::clicked, this, [this] {
        const std::vector<int> rows = selectedPartRows();
        for (auto it = rows.rbegin(); it != rows.rend(); ++it) {
            parts_->removeRow(*it);
            partData_.erase(partData_.begin() + *it);
        }
        if (!rows.empty()) {
            markDirty();
        }
    });
    connect(modelAxes_, &QTableWidget::itemChanged, this, &MachineLibraryDialog::markDirty);
    connect(parts_, &QTableWidget::itemChanged, this, [this](QTableWidgetItem* item) {
        if (item && item->column() == kPartColour) {
            const QColor c(item->text().trimmed());
            if (c.isValid()) {
                const QSignalBlocker block(parts_);
                item->setBackground(c);
                item->setForeground(qGray(c.rgb()) > 128 ? Qt::black : Qt::white);
            }
        }
        markDirty();
    });
}

std::vector<int> MachineLibraryDialog::selectedPartRows() const {
    std::vector<int> rows;
    for (const QModelIndex& index : parts_->selectionModel()->selectedRows()) {
        if (index.row() < static_cast<int>(partData_.size())) {
            rows.push_back(index.row());
        }
    }
    std::sort(rows.begin(), rows.end());
    return rows;
}

void MachineLibraryDialog::mergeSelectedParts() {
    const std::vector<int> rows = selectedPartRows();
    if (rows.size() < 2) {
        QMessageBox::information(this, tr("Merge parts"),
                                 tr("Select two or more parts that move together."));
        return;
    }
    const machine::MachineModel model = editedModel();
    bool ok = false;
    const QString name = QInputDialog::getText(
        this, tr("Merge parts"), tr("Name of the merged part:"), QLineEdit::Normal,
        qs(model.parts[static_cast<std::size_t>(rows.front())].name), &ok);
    if (!ok || name.trimmed().isEmpty()) {
        return;
    }
    // Each part where it stands, as one part in the model frame: exact when they all are.
    std::vector<machine::ModelPart> merging;
    for (const int r : rows) {
        merging.push_back(model.parts[static_cast<std::size_t>(r)]);
    }
    machine::ModelPart part;
    try {
        part = document::mergeParts(merging, ss(name), model.meshTolerance);
    } catch (const Error& e) {
        QMessageBox::warning(this, tr("Merge parts"), QString::fromUtf8(e.what()));
        return;
    }
    for (auto it = rows.rbegin(); it != rows.rend(); ++it) {
        parts_->removeRow(*it);
        partData_.erase(partData_.begin() + *it);
    }
    addPartRow(part);
    refreshCarriers();
    markDirty();
}

std::vector<std::string> MachineLibraryDialog::modelAxisNames() const {
    std::vector<std::string> names;
    for (int r = 0; r < modelAxes_->rowCount(); ++r) {
        names.push_back(comboData(qobject_cast<QComboBox*>(modelAxes_->cellWidget(r, kAxisName))));
    }
    return names;
}

void MachineLibraryDialog::addModelAxisRow(const machine::ModelAxis& a) {
    const QSignalBlocker block(modelAxes_);
    const int row = modelAxes_->rowCount();
    modelAxes_->insertRow(row);
    auto* name = new QComboBox;
    for (const char* n : kAxisNames) {
        name->addItem(QString::fromLatin1(n), QString::fromLatin1(n));
    }
    name->setCurrentIndex(std::max(0, name->findData(qs(a.name))));
    connect(name, &QComboBox::currentIndexChanged, this, [this] {
        refreshCarriers();
        markDirty();
    });
    modelAxes_->setCellWidget(row, kAxisName, name);
    auto* parent = carrierCombo(modelAxisNames(), a.parent);
    connect(parent, &QComboBox::currentIndexChanged, this, &MachineLibraryDialog::markDirty);
    modelAxes_->setCellWidget(row, kAxisParent, parent);
    for (int k = 0; k < 3; ++k) {
        modelAxes_->setItem(row, kDirX + k, numberItem(a.direction[k]));
        modelAxes_->setItem(row, kPivotX + k, numberItem(a.pivot[k]));
    }
}

void MachineLibraryDialog::addPartRow(const machine::ModelPart& p) {
    const QSignalBlocker block(parts_);
    const int row = parts_->rowCount();
    parts_->insertRow(row);
    parts_->setItem(row, kPartName, new QTableWidgetItem(qs(p.name)));
    auto* carrier = carrierCombo(modelAxisNames(), p.carrier);
    connect(carrier, &QComboBox::currentIndexChanged, this, [this, carrier] {
        // Several rows selected: the change applies to all of them.
        const std::vector<int> rows = selectedPartRows();
        const QString value = carrier->currentData().toString();
        bool inSelection = false;
        for (const int r : rows) {
            inSelection = inSelection || parts_->cellWidget(r, kPartCarrier) == carrier;
        }
        if (inSelection) {
            for (const int r : rows) {
                if (auto* other = qobject_cast<QComboBox*>(parts_->cellWidget(r, kPartCarrier));
                    other && other != carrier) {
                    const QSignalBlocker block(other);
                    other->setCurrentIndex(std::max(0, other->findData(value)));
                }
            }
        }
        markDirty();
    });
    parts_->setCellWidget(row, kPartCarrier, carrier);
    for (int k = 0; k < 3; ++k) {
        parts_->setItem(row, kOffX + k, numberItem(p.offset[k]));
        parts_->setItem(row, kRotX + k, numberItem(p.rotation[k]));
    }
    parts_->setItem(row, kPartColour, colourItem(p.colour));
    for (const auto& [column, checked] :
         {std::pair{kPartCollides, p.collides}, std::pair{kPartShown, p.shown},
          std::pair{kPartEnclosure, p.enclosure}}) {
        QWidget* cell = checkCell(checked);
        parts_->setCellWidget(row, column, cell);
        QCheckBox* box = cell->findChild<QCheckBox*>();
        const int col = column;
        connect(box, &QCheckBox::toggled, this, [this, box, col](bool on) {
            const std::vector<int> rows = selectedPartRows();
            bool inSelection = false;
            for (const int r : rows) {
                inSelection = inSelection || checkIn(parts_, r, col) == box;
            }
            if (inSelection) {
                for (const int r : rows) {
                    if (QCheckBox* other = checkIn(parts_, r, col); other && other != box) {
                        const QSignalBlocker block(other);
                        other->setChecked(on);
                    }
                }
            }
            markDirty();
        });
    }
    auto* triangles = new QTableWidgetItem(
        p.geometry ? QString::number(p.geometry->triangles.size()) : tr("missing"));
    triangles->setFlags(Qt::ItemIsEnabled);
    parts_->setItem(row, kPartTriangles, triangles);
    partData_.push_back(p);
}

void MachineLibraryDialog::refreshCarriers() {
    const std::vector<std::string> names = modelAxisNames();
    auto refill = [&](QComboBox* combo, const std::string& self) {
        if (!combo) {
            return;
        }
        const QSignalBlocker block(combo);
        const QString current = combo->currentData().toString();
        combo->clear();
        combo->addItem(tr("Base"), QString());
        for (const auto& n : names) {
            if (n != self) {
                combo->addItem(qs(n), qs(n));
            }
        }
        combo->setCurrentIndex(std::max(0, combo->findData(current)));
    };
    for (int r = 0; r < modelAxes_->rowCount(); ++r) {
        refill(qobject_cast<QComboBox*>(modelAxes_->cellWidget(r, kAxisParent)),
               names[static_cast<std::size_t>(r)]);
    }
    for (int r = 0; r < parts_->rowCount(); ++r) {
        refill(qobject_cast<QComboBox*>(parts_->cellWidget(r, kPartCarrier)), {});
    }
    refill(spindleCarrier_, {});
}

void MachineLibraryDialog::loadModel(const machine::Machine& m) {
    const QSignalBlocker blockAxes(modelAxes_);
    const QSignalBlocker blockParts(parts_);
    modelAxes_->setRowCount(0);
    parts_->setRowCount(0);
    partData_.clear();
    for (const auto& a : m.model.axes) {
        addModelAxisRow(a);
    }
    refreshCarriers();
    for (const auto& p : m.model.parts) {
        addPartRow(p);
    }
    for (int k = 0; k < 3; ++k) {
        const QSignalBlocker b1(spindlePoint_[static_cast<std::size_t>(k)]);
        spindlePoint_[static_cast<std::size_t>(k)]->setValue(m.model.spindlePoint[k]);
    }
    const QSignalBlocker b3(spindleCarrier_);
    spindleCarrier_->setCurrentIndex(
        std::max(0, spindleCarrier_->findData(qs(m.model.spindleCarrier))));
    const QSignalBlocker b4(meshTolerance_);
    meshTolerance_->setValue(m.model.meshTolerance);
}

machine::MachineModel MachineLibraryDialog::editedModel() const {
    machine::MachineModel model;
    for (int r = 0; r < modelAxes_->rowCount(); ++r) {
        machine::ModelAxis a;
        a.name = comboData(qobject_cast<QComboBox*>(modelAxes_->cellWidget(r, kAxisName)));
        a.parent = comboData(qobject_cast<QComboBox*>(modelAxes_->cellWidget(r, kAxisParent)));
        a.direction = Vec3(cellNumber(modelAxes_, r, kDirX), cellNumber(modelAxes_, r, kDirY),
                           cellNumber(modelAxes_, r, kDirZ));
        if (a.direction.norm() > 1e-12) {
            a.direction.normalize();
        }
        a.pivot = Vec3(cellNumber(modelAxes_, r, kPivotX), cellNumber(modelAxes_, r, kPivotY),
                       cellNumber(modelAxes_, r, kPivotZ));
        model.axes.push_back(std::move(a));
    }
    for (int r = 0; r < parts_->rowCount() && r < static_cast<int>(partData_.size()); ++r) {
        machine::ModelPart p = partData_[static_cast<std::size_t>(r)];
        const QTableWidgetItem* name = parts_->item(r, kPartName);
        p.name = name ? ss(name->text()) : std::string();
        p.carrier = comboData(qobject_cast<QComboBox*>(parts_->cellWidget(r, kPartCarrier)));
        p.offset = Vec3(cellNumber(parts_, r, kOffX), cellNumber(parts_, r, kOffY),
                        cellNumber(parts_, r, kOffZ));
        p.rotation = Vec3(cellNumber(parts_, r, kRotX), cellNumber(parts_, r, kRotY),
                          cellNumber(parts_, r, kRotZ));
        if (const QTableWidgetItem* c = parts_->item(r, kPartColour)) {
            const QColor colour(c->text().trimmed());
            if (colour.isValid()) {
                p.colour = static_cast<std::uint32_t>(colour.rgb()) & 0xFFFFFFu;
            }
        }
        if (const QCheckBox* c = checkIn(parts_, r, kPartCollides)) {
            p.collides = c->isChecked();
        }
        if (const QCheckBox* c = checkIn(parts_, r, kPartShown)) {
            p.shown = c->isChecked();
        }
        if (const QCheckBox* c = checkIn(parts_, r, kPartEnclosure)) {
            p.enclosure = c->isChecked();
        }
        model.parts.push_back(std::move(p));
    }
    if (model.empty()) {
        return model;
    }
    model.spindleCarrier = comboData(spindleCarrier_);
    model.meshTolerance = meshTolerance_->value();
    for (int k = 0; k < 3; ++k) {
        model.spindlePoint[k] = spindlePoint_[static_cast<std::size_t>(k)]->value();
    }
    return model;
}

void MachineLibraryDialog::fillDefaultAxes() {
    machine::Machine m = editedMachine();
    if (m.isLathe() && !m.lathe) {
        QMessageBox::information(
            this, tr("Machine model"),
            tr("A lathe's model needs its layout: tick Simulate the machine."));
        return;
    }
    const machine::MachineModel model = machine::defaultModel(m);
    {
        const QSignalBlocker block(modelAxes_);
        modelAxes_->setRowCount(0);
        for (const auto& a : model.axes) {
            addModelAxisRow(a);
        }
    }
    refreshCarriers();
    for (int k = 0; k < 3; ++k) {
        const QSignalBlocker b1(spindlePoint_[static_cast<std::size_t>(k)]);
        spindlePoint_[static_cast<std::size_t>(k)]->setValue(model.spindlePoint[k]);
    }
    {
        const QSignalBlocker b3(spindleCarrier_);
        spindleCarrier_->setCurrentIndex(
            std::max(0, spindleCarrier_->findData(qs(model.spindleCarrier))));
    }
    markDirty();
}

void MachineLibraryDialog::importParts() {
    const Item* item = currentItem();
    if (!item) {
        return;
    }
    if (item->builtin) {
        if (QMessageBox::question(this, tr("Import machine parts"),
                                  tr("Built-in machines are read-only. Duplicate '%1' into your "
                                     "library and import the parts there?")
                                      .arg(qs(item->machine.name))) != QMessageBox::Yes) {
            return;
        }
        duplicateCurrent();
        item = currentItem();
        if (!item || item->builtin) {
            return;
        }
    }
    QSettings settings;
    const QString folder = settings.value(QStringLiteral("machinePartsFolder")).toString();
    const QStringList files = QFileDialog::getOpenFileNames(
        this, tr("Import machine parts"), folder,
        tr("Machine CAD (*.step *.stp *.STEP *.STP *.stl *.STL);;STEP files (*.step *.stp);;"
           "STL files (*.stl)"));
    if (files.isEmpty()) {
        return;
    }
    settings.setValue(QStringLiteral("machinePartsFolder"),
                      QFileInfo(files.front()).absolutePath());
    std::vector<machine::ModelPart> parts =
        MachinePartsImportDialog::run(files, meshTolerance_->value(), this);
    if (parts.empty()) {
        return;
    }
    if (modelAxes_->rowCount() == 0) {
        fillDefaultAxes(); // the parts need axes to sit on
    }
    for (const auto& p : parts) {
        addPartRow(p);
    }
    refreshCarriers();
    markDirty();
    parts_->scrollToBottom();
}

void MachineLibraryDialog::schedulePreview() {
    if (previewTimer_) {
        previewTimer_->start();
    }
}

void MachineLibraryDialog::rebuildJog() {
    const Item* item = currentItem();
    std::vector<std::string> names;
    machine::Machine m;
    if (item) {
        m = item->builtin ? item->machine : editedMachine();
        for (const auto& a : machine::effectiveModel(m).axes) {
            names.push_back(a.name);
        }
        std::sort(names.begin(), names.end(), [](const std::string& a, const std::string& b) {
            return std::find(std::begin(kJogOrder), std::end(kJogOrder), a) <
                   std::find(std::begin(kJogOrder), std::end(kJogOrder), b);
        });
    }
    std::map<std::string, int> keep;
    for (const auto& [name, slider] : jog_) {
        keep[name] = slider->value();
    }
    while (jogForm_->rowCount() > 0) {
        jogForm_->removeRow(0);
    }
    jog_.clear();
    jogAxes_ = names;
    for (const auto& name : names) {
        const machine::Axis* axis = m.axis(name);
        const bool rotary = machine::isRotaryAxis(name);
        const double lo = axis ? axis->min : (rotary ? -180.0 : -500.0);
        const double hi = axis ? axis->max : (rotary ? 180.0 : 500.0);
        auto* slider = new QSlider(Qt::Horizontal);
        slider->setRange(static_cast<int>(std::floor(std::max(lo, -36000.0))),
                         static_cast<int>(std::ceil(std::min(hi, 36000.0))));
        const auto it = keep.find(name);
        slider->setValue(it != keep.end() ? it->second
                                          : std::clamp(0, slider->minimum(), slider->maximum()));
        auto* value = new QLabel;
        value->setMinimumWidth(70);
        auto show = [value, rotary](int v) {
            value->setText(rotary ? QStringLiteral("%1°").arg(v) : QStringLiteral("%1 mm").arg(v));
        };
        show(slider->value());
        connect(slider, &QSlider::valueChanged, this, [this, show](int v) {
            show(v);
            schedulePreview();
        });
        auto* row = new QHBoxLayout;
        row->addWidget(slider, 1);
        row->addWidget(value);
        jogForm_->addRow(qs(name), row);
        jog_[name] = slider;
    }
}

void MachineLibraryDialog::updatePreview() {
    const Item* item = currentItem();
    if (!item || !preview_ || !isVisible()) {
        return;
    }
    const machine::Machine m = item->builtin ? item->machine : editedMachine();
    machine::MachineModel model = machine::effectiveModel(m);
    std::vector<std::string> names;
    for (const auto& a : model.axes) {
        names.push_back(a.name);
    }
    std::sort(names.begin(), names.end(), [](const std::string& a, const std::string& b) {
        return std::string("XYZABC").find(a) < std::string("XYZABC").find(b);
    });
    if (names != jogAxes_) {
        rebuildJog();
    }
    machine::AxisPosition position;
    for (const auto& [name, slider] : jog_) {
        const double v = slider->value();
        if (name == "X" || name == "Y" || name == "Z") {
            position.linear[name[0] - 'X'] = v;
        } else if (name == "T") {
            position.t = degToRad(v);
        } else if (name == "W") {
            position.w = v;
        } else if (name == "Z2") {
            position.z2 = v;
        } else if (name == "C2") {
            position.c2 = degToRad(v);
        } else {
            (name == "A"   ? position.rotary.a
             : name == "B" ? position.rotary.b
                           : position.rotary.c) = degToRad(v);
        }
    }
    OcctView::MachineScene scene;
    try {
        if (model.empty()) {
            throw Error(ErrorCode::InvalidArgument, "lathes have no machine model yet");
        }
        const machine::ModelPose pose = machine::poseAt(model, position);
        auto parts = std::make_shared<std::vector<machine::ModelPart>>(m.model.parts);
        for (auto& p : machine::componentParts(m, model)) {
            parts->push_back(std::move(p));
        }
        if (viewShown_.size() != parts->size()) {
            viewShown_.assign(parts->size(), -1); // the parts changed: their defaults again
        }
        std::vector<bool> selected(parts->size(), false);
        for (const int i : selectedSceneParts()) {
            if (i < static_cast<int>(selected.size())) {
                selected[static_cast<std::size_t>(i)] = true;
            }
        }
        if (viewShown_.empty() || previewParts_ == nullptr ||
            previewParts_->size() != parts->size()) {
            geom3d::Aabb3 all;
            for (const auto& p : *parts) {
                if (p.geometry) {
                    all.extend(geom3d::transformed(p.geometry->bounds(), p.placement()));
                }
            }
            axisLineLength_ = all.isEmpty() ? 400.0 : std::max(200.0, 0.6 * all.size().norm());
        }
        previewCarried_.clear();
        for (std::size_t i = 0; i < parts->size(); ++i) {
            const machine::ModelPart& p = (*parts)[i];
            previewCarried_.push_back(pose.carried(model, p.carrier));
            scene.placements.push_back(pose.carried(model, p.carrier) * p.placement());
            scene.hidden.push_back((viewShown_[i] < 0 ? !p.shown : viewShown_[i] == 0) ||
                                   (p.enclosure && !enclosureBox_->isChecked()));
            scene.highlighted.push_back(selected[i]);
        }
        scene.pickable = true;
        // STEP parts drawn and picked as their exact geometry.
        for (const machine::ModelPart& p : *parts) {
            cad::Model solid;
            try {
                solid = document::partModel(p);
            } catch (const Error&) {
                // Its mesh is shown instead.
            }
            scene.solids.push_back(solid);
        }
        // Each rotary axis's centre line, moved by what carries it.
        std::string selectedAxis;
        if (const int row = modelAxes_->currentRow(); row >= 0 && !item->builtin) {
            selectedAxis =
                comboData(qobject_cast<QComboBox*>(modelAxes_->cellWidget(row, kAxisName)));
        }
        for (const machine::ModelAxis& a : model.axes) {
            if (!a.rotary() || a.direction.norm() < 1e-9) {
                continue;
            }
            const Transform3 carried = pose.carried(model, a.parent);
            const Vec3 d = a.direction.normalized() * axisLineLength_;
            scene.lines.push_back(
                {carried * Vec3(a.pivot - d), carried * Vec3(a.pivot + d), a.name == selectedAxis});
        }
        // The spindle's axis through the gauge point, and a stand-in tool hanging from it.
        const Vec3 up = pose.spindle.linear().col(2) * axisLineLength_;
        scene.lines.push_back({pose.spindle.translation() - up, pose.spindle.translation() + up,
                               axisPick_ == AxisPick::GaugeAxis || axisPick_ == AxisPick::GaugeSeat,
                               0x6CD06Cu});
        pickableParts_ = parts->size();
        previewParts_ = parts;
        static const auto standIn = [] {
            machine::Component tool;
            tool.shape = machine::Component::Shape::Cylinder;
            tool.base = Vec3(0, 0, -100);
            tool.axis = Vec3::UnitZ();
            tool.radius = 8;
            tool.length = 100;
            machine::ModelPart part;
            part.name = "stand-in tool";
            part.colour = 0xD4A017;
            part.geometry = std::make_shared<const geom3d::Mesh>(machine::componentMesh(tool));
            return part;
        }();
        auto shown = std::make_shared<std::vector<machine::ModelPart>>(*parts);
        shown->push_back(standIn);
        scene.placements.push_back(pose.spindle);
        scene.hidden.push_back(false);
        scene.highlighted.push_back(false);
        scene.parts = std::move(shown);
        scene.frames = {pose.spindle, pose.table};
        // A lathe's stations: where a block seats on each.
        if (m.lathe && m.toolCapacity > 0) {
            const machine::LatheTurret& t = m.lathe->turret;
            const Transform3 turret = pose.carried(model, model.spindleCarrier);
            for (int k = 1; k <= m.toolCapacity; ++k) {
                const Transform3 at = turret * machine::turretTurn(t, -machine::stationAngle(m, k));
                const Vec3 seat =
                    at * (t.centre + Vec3(-t.blockRadius, t.blockAcross, t.blockAlong));
                const Vec3 out = at.linear() * Vec3(-1.0, 0.0, 0.0);
                scene.lines.push_back({seat, seat + out * 40.0, k == 1});
            }
        }
        previewNote_->setText(
            m.model.empty()
                ? tr("No model of its own: the default axes of its kinematics and its collision "
                     "components. Triads: spindle gauge point and table centre.")
                : tr("Triads: spindle gauge point and table centre (where the table's rotary axes "
                     "meet); a stand-in tool (Ø16 × 100 mm) hangs from the gauge point."));
    } catch (const Error& e) {
        previewNote_->setText(QString::fromUtf8(e.what()));
    }
    // The preview is in the machine's axes, turned as it stands (DL-0077).
    const bool turned = preview_->setReferenceFrame(Mat3::Identity(), m.standing().worldAxes());
    const bool first = !previewShown_;
    preview_->setMachine(scene);
    if ((first || turned) && scene.parts && !scene.parts->empty()) {
        if (m.lathe) {
            preview_->lookFromWorld(Vec3(-0.3, -1.0, 0.45)); // from its door, the spindle left
        } else {
            preview_->lookFromWorld(Vec3(1.0, -1.0, 1.0).normalized()); // isometric
        }
        // The parts shown, not the axis lines that reach far beyond them.
        geom3d::Aabb3 box;
        for (std::size_t i = 0; i < scene.parts->size() && i < scene.placements.size(); ++i) {
            const auto& p = (*scene.parts)[i];
            if (p.geometry && !(i < scene.hidden.size() && scene.hidden[i])) {
                box.extend(geom3d::transformed(p.geometry->bounds(), scene.placements[i]));
            }
        }
        if (box.isEmpty()) {
            preview_->fitAll();
        } else {
            preview_->fitBox(box);
        }
        previewShown_ = true;
    }
}

std::vector<int> MachineLibraryDialog::selectedSceneParts() const {
    // Scene parts: the model's parts (rows of the parts table), then the components.
    std::vector<int> out = selectedPartRows();
    std::vector<int> components;
    for (const QModelIndex& index : components_->selectionModel()->selectedIndexes()) {
        components.push_back(index.row());
    }
    std::sort(components.begin(), components.end());
    components.erase(std::unique(components.begin(), components.end()), components.end());
    for (const int r : components) {
        out.push_back(static_cast<int>(partData_.size()) + r);
    }
    return out;
}

void MachineLibraryDialog::setViewShown(int index, bool shown) {
    if (index >= 0 && index < static_cast<int>(viewShown_.size())) {
        viewShown_[static_cast<std::size_t>(index)] = shown ? 1 : 0;
    }
}

void MachineLibraryDialog::isolateSelected() {
    updatePreview(); // the view overrides cover the parts as they are now
    const std::vector<int> selected = selectedSceneParts();
    if (selected.empty()) {
        return;
    }
    std::fill(viewShown_.begin(), viewShown_.end(), 0);
    for (const int i : selected) {
        setViewShown(i, true);
    }
    updatePreview();
    // Frame the isolated parts (not the axis lines, which reach far beyond them).
    geom3d::Aabb3 box;
    for (const int i : selected) {
        if (previewParts_ && i < static_cast<int>(previewParts_->size()) &&
            (*previewParts_)[static_cast<std::size_t>(i)].geometry) {
            const auto& p = (*previewParts_)[static_cast<std::size_t>(i)];
            box.extend(
                geom3d::transformed(p.geometry->bounds(),
                                    previewCarried_[static_cast<std::size_t>(i)] * p.placement()));
        }
    }
    preview_->fitBox(box);
}

void MachineLibraryDialog::selectScenePart(int index) {
    const int parts = static_cast<int>(partData_.size());
    if (index < 0) {
        return;
    }
    if (index < parts) {
        components_->clearSelection();
        parts_->selectRow(index);
        parts_->scrollToItem(parts_->item(index, 0));
        scroll_->ensureWidgetVisible(parts_);
    } else if (index - parts < components_->rowCount()) {
        parts_->clearSelection();
        simpleToggle_->setChecked(true);
        components_->selectRow(index - parts);
        scroll_->ensureWidgetVisible(components_);
    }
}

void MachineLibraryDialog::selectPartByName(const std::string& name) {
    for (int r = 0; r < parts_->rowCount(); ++r) {
        if (parts_->item(r, 0) && ss(parts_->item(r, 0)->text()) == name) {
            selectScenePart(r);
            return;
        }
    }
    for (int r = 0; r < components_->rowCount(); ++r) {
        if (components_->item(r, 0) && ss(components_->item(r, 0)->text()) == name) {
            selectScenePart(static_cast<int>(partData_.size()) + r);
            return;
        }
    }
}

std::string MachineLibraryDialog::selectedPartName() const {
    const std::vector<int> rows = selectedPartRows();
    if (!rows.empty() && parts_->item(rows.front(), 0)) {
        return ss(parts_->item(rows.front(), 0)->text());
    }
    for (const QModelIndex& index : components_->selectionModel()->selectedIndexes()) {
        if (const QTableWidgetItem* item = components_->item(index.row(), 0)) {
            return ss(item->text());
        }
    }
    return {};
}

void MachineLibraryDialog::chooseStanding(const QString& up, const QString& front) {
    standUp_->setCurrentIndex(std::max(0, standUp_->findText(up)));
    standFront_->setCurrentIndex(std::max(0, standFront_->findText(front)));
}

std::string MachineLibraryDialog::standingReport() {
    updatePreview();
    const Vec3 up = preview_->viewUp();
    const machine::Machine m = editedMachine();
    const auto name = [](machine::AxisSense a) {
        return QString::fromLatin1(machine::toString(a).data(),
                                   static_cast<qsizetype>(machine::toString(a).size()));
    };
    return QStringLiteral("stands: %1 up, front %2; preview up (%3, %4, %5); cube axes right, "
                          "back, up: %6%7")
        .arg(name(m.standing().up), name(m.standing().front))
        .arg(up.x(), 0, 'f', 2)
        .arg(up.y(), 0, 'f', 2)
        .arg(up.z(), 0, 'f', 2)
        .arg(preview_->cubeAxes(),
             m.orientation ? QString() : QStringLiteral(" (as its kinematics)"))
        .toStdString();
}

QSize MachineLibraryDialog::previewSize() const {
    return preview_->size();
}

std::string MachineLibraryDialog::pickPreview(const QPoint& position) {
    updatePreview();
    parts_->clearSelection();
    components_->clearSelection();
    preview_->pickAt(position);
    updatePreview(); // the picked part in the highlight colour
    return selectedPartName();
}

namespace {

QString vecText(const Vec3& v) {
    auto n = [](double x) {
        const double r = std::round(x * 10.0) / 10.0;
        return QString::number(r == 0.0 ? 0.0 : r, 'f', 1);
    };
    return QStringLiteral("(%1, %2, %3)").arg(n(v.x()), n(v.y()), n(v.z()));
}

double tidy(double v) {
    const double r = std::round(v * 1e4) / 1e4;
    return r == 0.0 ? 0.0 : r;
}

} // namespace

void MachineLibraryDialog::startAxisPick(AxisPick mode) {
    const int row = modelAxes_->currentRow();
    const std::string name =
        row >= 0 ? comboData(qobject_cast<QComboBox*>(modelAxes_->cellWidget(row, kAxisName)))
                 : std::string();
    if (!machine::isRotaryAxis(name)) {
        stopAxisPick(tr("Select a rotary axis (A, B or C) in the axes table first."));
        return;
    }
    axisPick_ = mode;
    axisPickRow_ = row;
    rimPoints_.clear();
    pickFace_->setChecked(mode == AxisPick::Face);
    pickRim_->setChecked(mode == AxisPick::Rim);
    pickGauge_->setChecked(false);
    gaugeAxis_.reset();
    pickNote_->setText(mode == AxisPick::Face
                           ? tr("%1 axis: click a round face or a circular edge of a part in the "
                                "preview (Esc cancels).")
                                 .arg(qs(name))
                           : tr("%1 axis: click three points on a circular rim in the preview, or "
                                "the rim's edge of a STEP part (Esc cancels).")
                                 .arg(qs(name)));
    pickNote_->show();
}

void MachineLibraryDialog::startGaugePick() {
    axisPick_ = AxisPick::GaugeAxis;
    axisPickRow_ = -1;
    rimPoints_.clear();
    gaugeAxis_.reset();
    pickFace_->setChecked(false);
    pickRim_->setChecked(false);
    pickGauge_->setChecked(true);
    pickNote_->setText(tr("Spindle, step 1 of 2: click a round face of the spindle (the nose's "
                          "bore, taper or outside) for its centre line, or the circular edge "
                          "where the holder seats for both steps at once (Esc cancels)."));
    pickNote_->show();
    schedulePreview();
}

void MachineLibraryDialog::stopAxisPick(const QString& note) {
    const bool gauge = axisPick_ == AxisPick::GaugeAxis || axisPick_ == AxisPick::GaugeSeat;
    axisPick_ = AxisPick::None;
    axisPickRow_ = -1;
    rimPoints_.clear();
    gaugeAxis_.reset();
    pickFace_->setChecked(false);
    pickRim_->setChecked(false);
    pickGauge_->setChecked(false);
    if (gauge) {
        schedulePreview();
    }
    pickNote_->setText(note);
    pickNote_->setVisible(!note.isEmpty());
}

void MachineLibraryDialog::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Escape && axisPick_ != AxisPick::None) {
        stopAxisPick();
        return; // the dialog stays open
    }
    QDialog::keyPressEvent(event);
}

void MachineLibraryDialog::previewClicked(int index, double x, double y, double z) {
    if (axisPick_ == AxisPick::None) {
        selectScenePart(index);
        return;
    }
    if (!previewParts_ || index < 0 || index >= static_cast<int>(pickableParts_) ||
        index >= static_cast<int>(previewParts_->size()) ||
        index >= static_cast<int>(previewCarried_.size())) {
        return; // the stand-in tool or nothing
    }
    // Back to machine zero: undo what the part's carrier does at the jogged position.
    const Vec3 atZero = previewCarried_[static_cast<std::size_t>(index)].inverse() * Vec3(x, y, z);
    const machine::ModelPart& part = (*previewParts_)[static_cast<std::size_t>(index)];
    // The exact geometry under the click (STEP parts), in the part's own coordinates.
    std::optional<cad::SubShapeGeometry> exact;
    if (exactPick_ && exactPick_->index == index) {
        try {
            const cad::Model solid = document::partModel(part);
            if (!solid.isNull()) {
                exact = cad::subShapeGeometry(solid, exactPick_->type, exactPick_->subIndex);
            }
        } catch (const Error&) {
            exact.reset();
        }
    }
    exactPick_.reset();
    const Transform3 placement = part.placement();
    using Kind = cad::SubShapeGeometry::Kind;
    if (axisPick_ == AxisPick::GaugeAxis && exact && exact->kind == Kind::Circle) {
        // The edge where the holder seats: its centre is the gauge point.
        gaugeAxis_ = document::axisOfShape(*exact, placement);
        axisPick_ = AxisPick::GaugeSeat;
    }
    if (axisPick_ == AxisPick::GaugeSeat && exact && exact->kind == Kind::Circle && gaugeAxis_ &&
        std::abs((placement.linear() * exact->direction).dot(gaugeAxis_->direction)) >
            std::cos(degToRad(5.0))) {
        finishGauge(part, machine::gaugePointOn(*gaugeAxis_, placement * exact->point));
        return;
    }
    if (axisPick_ == AxisPick::GaugeAxis) {
        gaugeAxis_ =
            exact ? document::axisOfShape(*exact, placement) : machine::axisAtPoint(part, atZero);
        if (!gaugeAxis_) {
            pickNote_->setText(tr("'%1' is not round there: click a cylindrical or conical face "
                                  "of the spindle (Esc cancels).")
                                   .arg(qs(part.name)));
            return;
        }
        axisPick_ = AxisPick::GaugeSeat;
        pickNote_->setText(tr("Spindle, step 2 of 2: centre line through %1 (Ø%2 mm). Now click "
                              "the face the holder seats against (turn the view to see it; Esc "
                              "cancels).")
                               .arg(vecText(gaugeAxis_->point))
                               .arg(2.0 * gaugeAxis_->radius, 0, 'f', 1));
        return;
    }
    if (axisPick_ == AxisPick::GaugeSeat && gaugeAxis_) {
        std::optional<Vec3> normal;
        Vec3 seat = atZero;
        if (exact) {
            if (exact->kind == Kind::Plane) {
                normal = placement.linear() * exact->direction;
                seat = placement * exact->point;
            }
        } else {
            normal = machine::normalAtPoint(part, atZero);
        }
        if (!normal ||
            std::abs(normal->dot(gaugeAxis_->direction.normalized())) < std::cos(degToRad(5.0))) {
            pickNote_->setText(tr("That face is not square to the spindle: click the flat face the "
                                  "holder seats against (Esc cancels)."));
            return;
        }
        finishGauge(part, machine::gaugePointOn(*gaugeAxis_, seat));
        return;
    }
    if (axisPick_ == AxisPick::Face) {
        const auto line =
            exact ? document::axisOfShape(*exact, placement) : machine::axisAtPoint(part, atZero);
        if (!line) {
            pickNote_->setText(tr("'%1' is not round there: click a cylindrical or conical face "
                                  "or a circular edge (Esc cancels).")
                                   .arg(qs(part.name)));
            return;
        }
        applyAxisLine(axisPickRow_, *line,
                      tr("%1 of '%2' (radius %3 mm)")
                          .arg(exact && exact->kind == Kind::Circle ? tr("a circular edge")
                                                                    : tr("a round face"),
                               qs(part.name))
                          .arg(line->radius, 0, 'f', 2));
        return;
    }
    if (exact && exact->kind == Kind::Circle) {
        // A STEP part's rim edge is the circle itself.
        const auto line = document::axisOfShape(*exact, placement);
        applyAxisLine(axisPickRow_, *line,
                      tr("a circular edge of '%1' (radius %2 mm)")
                          .arg(qs(part.name))
                          .arg(line->radius, 0, 'f', 2));
        return;
    }
    rimPoints_.push_back(exact && exact->kind == Kind::Point ? Vec3(placement * exact->point)
                                                             : atZero);
    if (rimPoints_.size() < 3) {
        pickNote_->setText(tr("Rim point %1 of 3 at %2; click the next one (Esc cancels).")
                               .arg(rimPoints_.size())
                               .arg(vecText(atZero)));
        return;
    }
    const auto circle = geom3d::circleThrough(rimPoints_[0], rimPoints_[1], rimPoints_[2]);
    if (!circle) {
        rimPoints_.clear();
        pickNote_->setText(tr("The three points lie on a line: click three points spread around "
                              "the rim (Esc cancels)."));
        return;
    }
    applyAxisLine(axisPickRow_, *circle,
                  tr("three rim points (radius %1 mm)").arg(circle->radius, 0, 'f', 2));
}

void MachineLibraryDialog::finishGauge(const machine::ModelPart& part, const Vec3& gauge) {
    for (int k = 0; k < 3; ++k) {
        spindlePoint_[static_cast<std::size_t>(k)]->setValue(tidy(gauge[k]));
    }
    // The model has the tool axis along Z at machine zero.
    const double lean =
        gaugeAxis_ ? std::acos(std::min(1.0, std::abs(gaugeAxis_->direction.normalized().z()))) *
                         180.0 / kPi
                   : 0.0;
    QString note = tr("Spindle from '%1': gauge point %2. Jog Z to check the stand-in tool moves "
                      "with the spindle.")
                       .arg(qs(part.name), vecText(gauge));
    if (lean > 0.5) {
        note += tr(" The spindle's centre line leans %1° from Z, but the machine model needs the "
                   "tool axis along Z at machine zero: check the parts' placement.")
                    .arg(lean, 0, 'f', 1);
    }
    stopAxisPick(note);
    markDirty();
    schedulePreview();
}

void MachineLibraryDialog::applyAxisLine(int row, const geom3d::AxisLine& line,
                                         const QString& from) {
    if (row < 0 || row >= modelAxes_->rowCount()) {
        stopAxisPick();
        return;
    }
    const std::string name =
        comboData(qobject_cast<QComboBox*>(modelAxes_->cellWidget(row, kAxisName)));
    const Vec3 previous(cellNumber(modelAxes_, row, kDirX), cellNumber(modelAxes_, row, kDirY),
                        cellNumber(modelAxes_, row, kDirZ));
    // The line has no sense of its own: keep the one the axis had, or ask.
    std::optional<Vec3> direction = machine::sameSense(line.direction, previous);
    if (!direction) {
        const Vec3 d = line.direction.normalized();
        QMessageBox box(QMessageBox::Question, tr("Rotary axis"),
                        tr("Which way does the %1 axis point? Positive %1 turns what it carries "
                           "right-handed about it (an axis that turns the table points along its "
                           "machine axis, e.g. A along +X).")
                            .arg(qs(name)),
                        QMessageBox::NoButton, this);
        QPushButton* plus = box.addButton(vecText(d), QMessageBox::AcceptRole);
        box.addButton(vecText(-d), QMessageBox::AcceptRole);
        box.addButton(QMessageBox::Cancel);
        box.exec();
        if (box.clickedButton() == box.button(QMessageBox::Cancel)) {
            stopAxisPick();
            return;
        }
        direction = box.clickedButton() == plus ? d : Vec3(-d);
    }
    for (int k = 0; k < 3; ++k) {
        modelAxes_->item(row, kDirX + k)->setText(QString::number(tidy((*direction)[k]), 'g', 10));
        modelAxes_->item(row, kPivotX + k)->setText(QString::number(tidy(line.point[k]), 'g', 10));
    }
    stopAxisPick(tr("%1 axis from %2: pivot %3, direction %4. Jog %1 to check it turns in place.")
                     .arg(qs(name), from, vecText(line.point), vecText(*direction)));
    markDirty();
    schedulePreview();
}

std::string MachineLibraryDialog::pickGaugeAt(const Vec3& round, const Vec3& seat) {
    startGaugePick();
    updatePreview();
    preview_->pickAt(preview_->widgetPoint(round));
    // The seat faces down: look from below at what is shown.
    preview_->snapView(V3d_Zneg, false);
    const AxisPick step = axisPick_;
    isolateSelected();
    axisPick_ = step;
    updatePreview();
    preview_->pickAt(preview_->widgetPoint(seat));
    updatePreview();
    return ss(pickNote_->text());
}

std::string MachineLibraryDialog::pickAxisAt(const std::string& axis, const Vec3& point) {
    for (int r = 0; r < modelAxes_->rowCount(); ++r) {
        if (comboData(qobject_cast<QComboBox*>(modelAxes_->cellWidget(r, kAxisName))) == axis) {
            modelAxes_->setCurrentCell(r, kAxisName);
            break;
        }
    }
    startAxisPick(AxisPick::Face);
    updatePreview();
    preview_->pickAt(preview_->widgetPoint(point));
    updatePreview();
    return ss(pickNote_->text());
}

void MachineLibraryDialog::showModelSection() {
    if (scroll_ && modelBox_) {
        scroll_->ensureWidgetVisible(modelBox_, 0, 0);
        scroll_->verticalScrollBar()->setValue(modelBox_->y());
    }
}

void MachineLibraryDialog::jogPreview(const std::string& axis, double value) {
    updatePreview();
    if (const auto it = jog_.find(axis); it != jog_.end()) {
        it->second->setValue(static_cast<int>(std::lround(value)));
    }
    updatePreview();
}

QImage MachineLibraryDialog::snapshot() {
    QImage image = grab().toImage();
    if (!preview_) {
        return image;
    }
    const QString file = QDir::temp().filePath(QStringLiteral("oxcam-machine-preview.png"));
    if (preview_->dumpImage(file)) {
        const QImage view(file);
        QPainter painter(&image);
        const qreal ratio = image.devicePixelRatio();
        const QPoint at = preview_->mapTo(this, QPoint(0, 0));
        painter.drawImage(QRectF(at.x(), at.y(), preview_->width(), preview_->height()), view);
        Q_UNUSED(ratio);
        QFile::remove(file);
    }
    return image;
}

} // namespace ocam::gui
