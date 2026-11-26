// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ToolLibraryDialog.hpp"

#include "CuttingPresetDialog.hpp"
#include "HolderImportDialog.hpp"
#include "ToolProfileView.hpp"
#include "Translations.hpp"
#include "Units.hpp"

#include <ocam/cad/Model.hpp>
#include <ocam/foundation/Error.hpp>
#include <ocam/tooling/TurningHolder.hpp>

#include <QApplication>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QKeyEvent>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMenu>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollArea>
#include <QSettings>
#include <QSpinBox>
#include <QSplitter>
#include <QStackedWidget>
#include <QTabWidget>
#include <QTableWidget>
#include <QToolButton>
#include <QVBoxLayout>
#include <cmath>
#include <set>

namespace ocam::gui {
namespace {

using tooling::ToolType;

// Geometry fields: id, label, degrees?
struct FieldDef {
    const char* id;
    const char* label;
    bool angle;
};

constexpr FieldDef kMillFields[] = {
    {"diameter", QT_TRANSLATE_NOOP("ocam::gui::ToolLibraryDialog", "Diameter"), false},
    {"cornerRadius", QT_TRANSLATE_NOOP("ocam::gui::ToolLibraryDialog", "Corner radius"), false},
    {"fluteLength", QT_TRANSLATE_NOOP("ocam::gui::ToolLibraryDialog", "Flute length"), false},
    {"overallLength", QT_TRANSLATE_NOOP("ocam::gui::ToolLibraryDialog", "Overall length"), false},
    {"shankDiameter", QT_TRANSLATE_NOOP("ocam::gui::ToolLibraryDialog", "Shank diameter"), false},
    {"tipAngle", QT_TRANSLATE_NOOP("ocam::gui::ToolLibraryDialog", "Point angle"), true},
    {"tipDiameter", QT_TRANSLATE_NOOP("ocam::gui::ToolLibraryDialog", "Tip diameter"), false},
    {"tipLength", QT_TRANSLATE_NOOP("ocam::gui::ToolLibraryDialog", "Pilot length"), false},
    {"threadPitch", QT_TRANSLATE_NOOP("ocam::gui::ToolLibraryDialog", "Thread pitch"), false},
    {"neckDiameter", QT_TRANSLATE_NOOP("ocam::gui::ToolLibraryDialog", "Neck diameter"), false},
    {"neckLength", QT_TRANSLATE_NOOP("ocam::gui::ToolLibraryDialog", "Neck length"), false},
    {"triggerOffset", QT_TRANSLATE_NOOP("ocam::gui::ToolLibraryDialog", "Trigger offset"), false},
};

constexpr FieldDef kInsertFields[] = {
    {"inscribedCircle", QT_TRANSLATE_NOOP("ocam::gui::ToolLibraryDialog", "Inscribed circle"),
     false},
    {"noseRadius", QT_TRANSLATE_NOOP("ocam::gui::ToolLibraryDialog", "Nose radius"), false},
    {"thickness", QT_TRANSLATE_NOOP("ocam::gui::ToolLibraryDialog", "Thickness"), false},
    {"reliefAngle", QT_TRANSLATE_NOOP("ocam::gui::ToolLibraryDialog", "Relief angle"), true},
    {"cuttingEdge", QT_TRANSLATE_NOOP("ocam::gui::ToolLibraryDialog", "Cutting edge angle"), true},
    {"insertWidth", QT_TRANSLATE_NOOP("ocam::gui::ToolLibraryDialog", "Width"), false},
    {"insertPitch", QT_TRANSLATE_NOOP("ocam::gui::ToolLibraryDialog", "Thread pitch"), false},
};

// A solid bar's dimensions, by the names catalogues sell them by.
constexpr FieldDef kSolidFields[] = {
    {"solidMinBore", QT_TRANSLATE_NOOP("ocam::gui::ToolLibraryDialog", "Min. bore (ØD)"), false},
    {"solidShank", QT_TRANSLATE_NOOP("ocam::gui::ToolLibraryDialog", "Shank (ØDMM)"), false},
    {"solidHead", QT_TRANSLATE_NOOP("ocam::gui::ToolLibraryDialog", "Head length (LH)"), false},
    {"solidLength", QT_TRANSLATE_NOOP("ocam::gui::ToolLibraryDialog", "Overall length (OAL)"),
     false},
    {"solidOverhang", QT_TRANSLATE_NOOP("ocam::gui::ToolLibraryDialog", "Max. overhang (OHX)"),
     false},
    {"solidTip", QT_TRANSLATE_NOOP("ocam::gui::ToolLibraryDialog", "Centre to tip (S)"), false},
    {"solidDepth", QT_TRANSLATE_NOOP("ocam::gui::ToolLibraryDialog", "Max. depth (T-MAX)"), false},
};

// Which optional fields apply to a type (always shown: diameter, flute/overall length, shank).
std::set<QString> fieldsFor(ToolType t) {
    switch (t) {
    case ToolType::BullNoseEndMill:
        return {"cornerRadius", "neckDiameter", "neckLength"};
    case ToolType::FlatEndMill:
    case ToolType::BallEndMill:
        return {"neckDiameter", "neckLength"};
    case ToolType::Drill:
    case ToolType::SpotDrill:
        return {"tipAngle"};
    case ToolType::Chamfer:
    case ToolType::Engraver:
        return {"tipAngle", "tipDiameter"};
    case ToolType::CenterDrill:
        return {"tipAngle", "tipDiameter", "tipLength"};
    case ToolType::Tap:
    case ToolType::ThreadMill:
        return {"threadPitch"};
    case ToolType::SlotCutter:
    case ToolType::Lollipop:
        return {"neckDiameter", "neckLength"};
    case ToolType::BoringBar: // with a neck: a back-boring bar; with a point angle, back chamfers
        return {"tipAngle", "neckDiameter", "neckLength"};
    case ToolType::Probe:
        return {"neckDiameter", "triggerOffset"};
    case ToolType::TurningInsert:
        return {"inscribedCircle", "noseRadius", "thickness", "reliefAngle", "cuttingEdge"};
    case ToolType::GroovingInsert:
    case ToolType::PartingInsert:
        return {"noseRadius", "thickness", "reliefAngle", "insertWidth"};
    case ToolType::ThreadingInsert:
        return {"inscribedCircle", "noseRadius", "thickness", "reliefAngle", "insertPitch"};
    case ToolType::BarPuller: // the largest bar it grips (DL-0091)
        return {"tipDiameter"};
    default:
        return {};
    }
}

QDoubleSpinBox* makeSpin(bool angle) {
    auto* s = new QDoubleSpinBox;
    s->setDecimals(angle ? 2 : 4);
    s->setRange(0.0, angle ? 180.0 : 10000.0);
    s->setSuffix(angle ? QStringLiteral(" °") : QStringLiteral(" mm"));
    s->setKeyboardTracking(false);
    return s;
}

QString qs(const std::string& s) {
    return QString::fromStdString(s);
}

std::string ss(const QString& s) {
    return s.toStdString();
}

double unitFactor(LengthUnit unit) {
    return unit == LengthUnit::Inch ? 25.4 : 1.0;
}

// The tool list's type filter, as the filter combo's item data (and the saved setting): "" (all
// types), "family:<id>" or "type:<tool type id>".
QString familyId(tooling::ToolFamily family) {
    switch (family) {
    case tooling::ToolFamily::HoleMaking:
        return QStringLiteral("hole_making");
    case tooling::ToolFamily::Turning:
        return QStringLiteral("turning");
    case tooling::ToolFamily::Probing:
        return QStringLiteral("probing");
    default:
        return QStringLiteral("milling");
    }
}

QString typeId(ToolType type) {
    const std::string_view id = tooling::toString(type);
    return QString::fromUtf8(id.data(), static_cast<qsizetype>(id.size()));
}

const QString kFilterTypeKey = QStringLiteral("toolLibrary/typeFilter");
const QString kFilterMinKey = QStringLiteral("toolLibrary/minDiameter");
const QString kFilterMaxKey = QStringLiteral("toolLibrary/maxDiameter");

} // namespace

ToolLibraryDialog::ToolLibraryDialog(tooling::ToolLibrary library, Mode mode, QWidget* parent)
    : QDialog(parent), lib_(std::move(library)), mode_(mode) {
    saved_ = lib_;
    resize(1100, 700);
    buildUi();
    rebuildTable();
    applyFilter(false);
    rebuildHolderTable();
    currentHolder_ = lib_.holders.empty() ? -1 : 0;
    if (currentHolder_ >= 0) {
        const QSignalBlocker block(holderTable_);
        holderTable_->selectRow(0);
    }
    updateTitle();
}

void ToolLibraryDialog::setFile(const std::optional<std::filesystem::path>& file) {
    file_ = file;
    updateTitle();
}

void ToolLibraryDialog::updateTitle() {
    if (mode_ != Mode::LibraryFile) {
        setWindowTitle(tr("Project Tools"));
        return;
    }
    QString title = lib_.name.empty() ? tr("Tool Library") : QString::fromStdString(lib_.name);
    if (file_) {
        title +=
            QStringLiteral(" - %1").arg(QString::fromStdU16String(file_->filename().u16string()));
    }
    setWindowTitle(isModified() ? title + QStringLiteral(" *") : title);
}

bool ToolLibraryDialog::isModified() const {
    return mode_ == Mode::LibraryFile && !(lib_ == saved_);
}

bool ToolLibraryDialog::maybeSave() {
    if (!isModified()) {
        return true;
    }
    const auto answer = QMessageBox::question(
        this, windowTitle(), tr("Save the changes to this tool library?"),
        QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel, QMessageBox::Save);
    if (answer == QMessageBox::Save) {
        return saveFile(false);
    }
    return answer == QMessageBox::Discard;
}

void ToolLibraryDialog::reject() {
    // Name edits commit when the field loses focus: take a pending one now.
    if (libraryName_ && libraryName_->hasFocus()) {
        lib_.name = libraryName_->text().trimmed().toStdString();
    }
    if (mode_ == Mode::LibraryFile && !maybeSave()) {
        return;
    }
    QDialog::reject();
}

void ToolLibraryDialog::keyPressEvent(QKeyEvent* event) {
    // The field has committed its value (editingFinished) before the key gets here. A focused
    // button handles Enter itself and never passes it on.
    if (event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter) {
        event->accept();
        return;
    }
    QDialog::keyPressEvent(event);
}

void ToolLibraryDialog::setAddToProjectHandler(AddToProjectHandler handler) {
    addToProject_ = std::move(handler);
    if (copyToProject_) {
        copyToProject_->setVisible(static_cast<bool>(addToProject_));
    }
}

void ToolLibraryDialog::buildUi() {
    auto* root = new QVBoxLayout(this);
    if (mode_ == Mode::LibraryFile) {
        auto* nameRow = new QHBoxLayout;
        nameRow->addWidget(new QLabel(tr("Library name")));
        libraryName_ = new QLineEdit(QString::fromStdString(lib_.name));
        libraryName_->setPlaceholderText(tr("For example Mill 1 aluminium"));
        libraryName_->setToolTip(tr("How the library is listed in the tool libraries and in the "
                                    "tool pickers"));
        nameRow->addWidget(libraryName_, 1);
        root->addLayout(nameRow);
        connect(libraryName_, &QLineEdit::editingFinished, this, [this] {
            lib_.name = libraryName_->text().trimmed().toStdString();
            updateTitle();
        });
    }
    auto* splitter = new QSplitter(this);
    root->addWidget(splitter, 1);

    // Left: tool list and list actions.
    auto* left = new QWidget;
    auto* leftLayout = new QVBoxLayout(left);
    leftLayout->setContentsMargins(0, 0, 0, 0);

    // Filters: text, type or family, diameter range. The type and diameter are kept per user.
    filterText_ = new QLineEdit;
    filterText_->setPlaceholderText(tr("Search name, number, vendor or part number"));
    filterText_->setClearButtonEnabled(true);
    leftLayout->addWidget(filterText_);
    auto* filterRow = new QHBoxLayout;
    filterType_ = new QComboBox;
    filterType_->setToolTip(tr("Show one family or one type of tool"));
    filterType_->addItem(tr("All types"), QString());
    filterType_->addItem(tr("Milling"), QStringLiteral("family:milling"));
    filterType_->addItem(tr("Hole making"), QStringLiteral("family:hole_making"));
    filterType_->addItem(tr("Turning"), QStringLiteral("family:turning"));
    filterType_->addItem(tr("Probing"), QStringLiteral("family:probing"));
    filterType_->insertSeparator(filterType_->count());
    for (ToolType t : tooling::allToolTypes()) {
        filterType_->addItem(trToolType(t), QStringLiteral("type:") + typeId(t));
    }
    filterRow->addWidget(filterType_, 1);
    const LengthUnit filterUnit = preferredUnit();
    filterMin_ = new LengthSpinBox(0.0, filterUnit, 0.0, 10000.0);
    filterMax_ = new LengthSpinBox(0.0, filterUnit, 0.0, 10000.0);
    filterMin_->setToolTip(tr("Smallest diameter shown; Any = no limit"));
    filterMax_->setToolTip(tr("Largest diameter shown; Any = no limit"));
    for (LengthSpinBox* spin : {filterMin_, filterMax_}) {
        spin->setSpecialValueText(tr("Any"));
    }
    filterRow->addWidget(new QLabel(tr("Ø")));
    filterRow->addWidget(filterMin_);
    filterRow->addWidget(new QLabel(QStringLiteral("–")));
    filterRow->addWidget(filterMax_);
    auto* clearFilter = new QToolButton;
    clearFilter->setText(tr("Clear"));
    clearFilter->setToolTip(tr("Show every tool"));
    filterRow->addWidget(clearFilter);
    leftLayout->addLayout(filterRow);
    {
        const QSettings settings;
        const int typeIndex = filterType_->findData(settings.value(kFilterTypeKey).toString());
        filterType_->setCurrentIndex(std::max(0, typeIndex));
        filterMin_->setMm(settings.value(kFilterMinKey, 0.0).toDouble());
        filterMax_->setMm(settings.value(kFilterMaxKey, 0.0).toDouble());
    }
    auto saveFilter = [this] {
        QSettings settings;
        settings.setValue(kFilterTypeKey, filterType_->currentData().toString());
        settings.setValue(kFilterMinKey, filterMin_->mm());
        settings.setValue(kFilterMaxKey, filterMax_->mm());
        applyFilter(false);
    };
    connect(filterText_, &QLineEdit::textChanged, this, [this] { applyFilter(false); });
    connect(filterType_, &QComboBox::currentIndexChanged, this, saveFilter);
    connect(filterMin_, &LengthSpinBox::mmChanged, this, saveFilter);
    connect(filterMax_, &LengthSpinBox::mmChanged, this, saveFilter);
    connect(clearFilter, &QToolButton::clicked, this, [this] { setToolFilter({}); });

    table_ = new QTableWidget(0, 4);
    table_->setHorizontalHeaderLabels({tr("T"), tr("Name"), tr("Type"), tr("Ø")});
    table_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    table_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
    table_->verticalHeader()->hide();
    table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    leftLayout->addWidget(table_);
    filterCount_ = new QLabel;
    leftLayout->addWidget(filterCount_);
    connect(table_, &QTableWidget::currentCellChanged, this,
            [this](int row, int, int, int) { selectRow(row); });

    auto* listButtons = new QHBoxLayout;
    auto* addButton = new QToolButton;
    addButton->setText(tr("New tool"));
    addButton->setPopupMode(QToolButton::InstantPopup);
    auto* addMenu = new QMenu(addButton);
    for (ToolType t : tooling::allToolTypes()) {
        addMenu->addAction(trToolType(t), this, [this, t] { addTool(t); });
    }
    addMenu->addAction(tr("Solid boring bar"), this, &ToolLibraryDialog::addSolidBoringBar)
        ->setToolTip(tr("One piece of carbide with no insert or holder, such as a micro boring "
                        "bar, set by the dimensions it is sold by"));
    addButton->setMenu(addMenu);
    listButtons->addWidget(addButton);
    auto* duplicate = new QPushButton(tr("Duplicate"));
    connect(duplicate, &QPushButton::clicked, this, &ToolLibraryDialog::duplicateTool);
    listButtons->addWidget(duplicate);
    auto* remove = new QPushButton(tr("Delete"));
    connect(remove, &QPushButton::clicked, this, &ToolLibraryDialog::deleteTools);
    listButtons->addWidget(remove);
    leftLayout->addLayout(listButtons);
    lists_ = new QTabWidget;
    lists_->addTab(left, tr("Tools"));
    lists_->addTab(buildHolderList(), tr("Holders"));
    splitter->addWidget(lists_);
    connect(lists_, &QTabWidget::currentChanged, this, [this](int tab) { showHolders(tab == 1); });

    // Middle: editor.
    auto* editor = new QWidget;
    form_ = new QFormLayout(editor);
    number_ = new QSpinBox;
    number_->setRange(1, 99999);
    name_ = new QLineEdit;
    name_->setObjectName(QStringLiteral("toolName"));
    type_ = new QComboBox;
    for (ToolType t : tooling::allToolTypes()) {
        type_->addItem(trToolType(t), static_cast<int>(t));
    }
    material_ = new QLineEdit;
    coating_ = new QLineEdit;
    vendor_ = new QLineEdit;
    partNumber_ = new QLineEdit;
    comment_ = new QLineEdit;
    holder_ = new QComboBox;
    holder_->setObjectName(QStringLiteral("toolHolder"));
    holderNote_ = new QLabel;
    holderNote_->setObjectName(QStringLiteral("toolHolderNote"));
    holderNote_->setWordWrap(true);
    locking_ = new QComboBox;
    locking_->setObjectName(QStringLiteral("toolLocking"));
    locking_->addItem(tr("Not set: any clamping"),
                      static_cast<int>(tooling::InsertLocking::NotSet));
    locking_->addItem(tr("Cylindrical hole: lever or top and hole clamping (P, M)"),
                      static_cast<int>(tooling::InsertLocking::CylindricalHole));
    locking_->addItem(tr("No hole: top clamping (C)"),
                      static_cast<int>(tooling::InsertLocking::NoHole));
    locking_->addItem(tr("Countersunk hole: screw (S)"),
                      static_cast<int>(tooling::InsertLocking::CountersunkHole));
    locking_->setToolTip(tr("How the insert is held, by the hole it has: only holders with a "
                            "clamping that takes it are offered"));
    // Long texts show in the open list; the form stays as narrow as the rest of the editor.
    for (QComboBox* combo : {holder_, locking_}) {
        combo->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
        combo->setMinimumContentsLength(14);
    }
    stickout_ = makeSpin(false);
    gauge_ = makeSpin(false);
    flutes_ = new QSpinBox;
    flutes_->setRange(1, 64);
    orientation_ = new QSpinBox;
    orientation_->setRange(1, 9);
    hand_ = new QComboBox;
    hand_->addItems({QStringLiteral("N"), QStringLiteral("L"), QStringLiteral("R")});
    insertShape_ = new QComboBox;
    for (const tooling::InsertShape& s : tooling::insertShapes()) {
        insertShape_->addItem(tr("%1: %2").arg(QChar::fromLatin1(s.letter),
                                               QCoreApplication::translate("Tooling", s.name)),
                              QString(QChar::fromLatin1(s.letter)));
    }
    insertShape_->setObjectName(QStringLiteral("insertShape"));
    insertShape_->setToolTip(tr("The insert's shape letter (ISO 1832): its corner angle and "
                                "outline, for the toolpaths and the simulation"));

    form_->addRow(tr("Number"), number_);
    form_->addRow(tr("Name"), name_);
    form_->addRow(tr("Type"), type_);
    unit_ = new QComboBox;
    unit_->addItem(tr("Millimetres"));
    unit_->addItem(tr("Inches"));
    unit_->setToolTip(tr("The unit this tool's dimensions and cutting data are shown in"));
    form_->addRow(tr("Units"), unit_);
    for (const auto& f : kMillFields) {
        auto* spin = makeSpin(f.angle);
        lengths_.insert(QString::fromLatin1(f.id), spin);
        spin->setObjectName(QString::fromLatin1(f.id));
        form_->addRow(tr(f.label), spin);
    }
    threadHand_ = new QComboBox;
    threadHand_->addItem(tr("Right hand"));
    threadHand_->addItem(tr("Left hand"));
    threadHand_->setObjectName(QStringLiteral("threadHand"));
    threadHand_->setToolTip(tr("The thread this tap cuts: left hand runs the spindle the other way "
                               "(M04) and on the control's left-hand tap cycle"));
    form_->addRow(tr("Thread hand"), threadHand_);
    // 0 means the shank is as wide as the cutter (and follows it).
    lengths_["shankDiameter"]->setSpecialValueText(tr("Same as diameter"));
    lengths_["neckDiameter"]->setSpecialValueText(tr("None"));
    form_->addRow(tr("Flutes"), flutes_);
    form_->addRow(tr("Insert shape"), insertShape_);
    for (const auto& f : kInsertFields) {
        auto* spin = makeSpin(f.angle);
        lengths_.insert(QString::fromLatin1(f.id), spin);
        spin->setObjectName(QString::fromLatin1(f.id));
        form_->addRow(tr(f.label), spin);
    }
    for (const auto& f : kSolidFields) {
        auto* spin = makeSpin(f.angle);
        lengths_.insert(QString::fromLatin1(f.id), spin);
        spin->setObjectName(QString::fromLatin1(f.id));
        form_->addRow(tr(f.label), spin);
    }
    lengths_["solidOverhang"]->setSpecialValueText(tr("Overall length"));
    lengths_["solidOverhang"]->setToolTip(
        tr("How far it may reach out of its sleeve: deeper toolpaths are refused"));
    lengths_["solidDepth"]->setSpecialValueText(tr("Not stated"));
    lengths_["solidDepth"]->setToolTip(
        tr("The largest depth of cut: a rough with more is refused"));
    lengths_["solidMinBore"]->setToolTip(tr("The smallest bore it fits, tip to back of the head"));
    lengths_["solidTip"]->setToolTip(tr("From the shank's axis to the tip"));
    lengths_["cuttingEdge"]->setSpecialValueText(tr("As on a 93-95° holder"));
    lengths_["cuttingEdge"]->setToolTip(
        tr("The angle of the insert's leading edge to the feed towards the chuck, which places "
           "the insert for the toolpaths and the simulation. An ISO holder sets it."));
    form_->addRow(tr("Hand"), hand_);
    form_->addRow(tr("Orientation"), orientation_);
    holder_->setToolTip(tr("The holder this tool sits in, among the library's holders for this "
                           "kind of tool that fit its insert: shape, size, clearance and locking "
                           "style (the Holders tab manages them). An ISO turning holder sets the "
                           "insert's hand, orientation and cutting edge angle."));
    stickout_->setToolTip(tr("How far the tool comes out of the holder: from the holder's face to "
                             "the tool tip; 0 = the overall length"));
    stickout_->setSpecialValueText(tr("Overall length"));
    gauge_->setToolTip(tr("The assembly's length from the spindle gauge line to the tool tip: the "
                          "stickout plus the holder's height (the stickout alone without a "
                          "holder). The machine's tool length, the simulation and the post use "
                          "it; setting it sets the stickout."));
    form_->addRow(tr("Locking style"), locking_);
    form_->addRow(tr("Holder"), holder_);
    form_->addRow(QString(), holderNote_);
    form_->addRow(tr("Stickout"), stickout_);
    form_->addRow(tr("Gauge length"), gauge_);
    form_->addRow(tr("Material"), material_);
    form_->addRow(tr("Coating"), coating_);
    form_->addRow(tr("Vendor"), vendor_);
    form_->addRow(tr("Part number"), partNumber_);
    form_->addRow(tr("Comment"), comment_);

    auto* scroll = new QScrollArea;
    scroll->setWidget(editor);
    scroll->setWidgetResizable(true);
    editors_ = new QStackedWidget;
    editors_->addWidget(scroll);
    editors_->addWidget(buildHolderEditor());
    splitter->addWidget(editors_);

    // Right: preview and presets.
    auto* right = new QWidget;
    auto* rightLayout = new QVBoxLayout(right);
    rightLayout->setContentsMargins(0, 0, 0, 0);
    preview_ = new ToolProfileView;
    rightLayout->addWidget(preview_, 2);
    presetsBox_ = new QGroupBox(tr("Cutting presets"));
    auto* presetsLayout = new QVBoxLayout(presetsBox_);
    presets_ = new QListWidget;
    presets_->setToolTip(tr("Double-click a preset to edit it"));
    presets_->setTextElideMode(Qt::ElideRight); // one line per preset
    presets_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    presetsLayout->addWidget(presets_);
    auto* presetButtons = new QHBoxLayout;
    editPresets_ = new QPushButton(tr("Edit Presets..."));
    auto* addPreset = new QPushButton(tr("Add Preset..."));
    presetButtons->addWidget(editPresets_);
    presetButtons->addWidget(addPreset);
    presetButtons->addStretch();
    presetsLayout->addLayout(presetButtons);
    rightLayout->addWidget(presetsBox_, 1);
    splitter->addWidget(right);
    splitter->setSizes({400, 340, 360});

    connect(editPresets_, &QPushButton::clicked, this,
            [this] { editPresets(std::max(0, presets_->currentRow())); });
    connect(presets_, &QListWidget::itemDoubleClicked, this,
            [this](QListWidgetItem* item) { editPresets(presets_->row(item)); });
    connect(addPreset, &QPushButton::clicked, this, [this] { editPresets(-1); });

    issues_ = new QLabel;
    issues_->setWordWrap(true);
    root->addWidget(issues_);

    // Bottom buttons.
    auto* bottom = new QHBoxLayout;
    if (mode_ == Mode::LibraryFile) {
        auto* open = new QPushButton(tr("Open..."));
        auto* save = new QPushButton(tr("Save"));
        auto* saveAs = new QPushButton(tr("Save As..."));
        connect(open, &QPushButton::clicked, this, &ToolLibraryDialog::openFile);
        connect(save, &QPushButton::clicked, this, [this] { saveFile(false); });
        connect(saveAs, &QPushButton::clicked, this, [this] { saveFile(true); });
        bottom->addWidget(open);
        bottom->addWidget(save);
        bottom->addWidget(saveAs);
        copyToProject_ = new QPushButton(tr("Copy selected to project"));
        copyToProject_->setVisible(false);
        connect(copyToProject_, &QPushButton::clicked, this, [this] {
            std::vector<tooling::Tool> tools;
            std::vector<tooling::Holder> holders;
            for (int r : selectedToolRows()) {
                const tooling::Tool& t = lib_.tools[static_cast<std::size_t>(r)];
                tools.push_back(t);
                if (t.holderId) {
                    if (const tooling::Holder* h = lib_.findHolder(*t.holderId)) {
                        holders.push_back(*h);
                    }
                }
            }
            if (!tools.empty() && addToProject_) {
                addToProject_(tools, holders);
            }
        });
        bottom->addWidget(copyToProject_);
    }
    bottom->addStretch();
    auto* buttons = new QDialogButtonBox(mode_ == Mode::ProjectTools
                                             ? QDialogButtonBox::Ok | QDialogButtonBox::Cancel
                                             : QDialogButtonBox::Close);
    connect(buttons, &QDialogButtonBox::accepted, this, [this] {
        if (!lib_.validate().empty() &&
            QMessageBox::question(this, windowTitle(),
                                  tr("The tool list has problems (see the bottom of the dialog). "
                                     "Apply anyway?")) != QMessageBox::Yes) {
            return;
        }
        accept();
    });
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    bottom->addWidget(buttons);
    root->addLayout(bottom);

    // Every editor commits immediately.
    for (auto* line : {name_, material_, coating_, vendor_, partNumber_, comment_}) {
        connect(line, &QLineEdit::editingFinished, this, &ToolLibraryDialog::commitEditor);
    }
    for (auto* spin : {number_, flutes_, orientation_}) {
        connect(spin, &QSpinBox::valueChanged, this, &ToolLibraryDialog::commitEditor);
    }
    connect(threadHand_, &QComboBox::currentIndexChanged, this, &ToolLibraryDialog::commitEditor);
    for (auto* spin : lengths_) {
        connect(spin, &QDoubleSpinBox::valueChanged, this, &ToolLibraryDialog::commitEditor);
    }
    connect(stickout_, &QDoubleSpinBox::valueChanged, this, &ToolLibraryDialog::commitEditor);
    connect(gauge_, &QDoubleSpinBox::valueChanged, this, &ToolLibraryDialog::commitGauge);
    for (auto* combo : {type_, holder_, hand_, unit_, insertShape_, locking_}) {
        connect(combo, &QComboBox::currentIndexChanged, this, &ToolLibraryDialog::commitEditor);
    }
}

tooling::Tool* ToolLibraryDialog::currentTool() {
    if (current_ < 0 || current_ >= static_cast<int>(lib_.tools.size())) {
        return nullptr;
    }
    return &lib_.tools[static_cast<std::size_t>(current_)];
}

void ToolLibraryDialog::rebuildTable() {
    const QSignalBlocker block(table_);
    table_->setRowCount(static_cast<int>(lib_.tools.size()));
    for (int r = 0; r < table_->rowCount(); ++r) {
        const tooling::Tool& t = lib_.tools[static_cast<std::size_t>(r)];
        const QString type = trToolType(t.type);
        table_->setItem(r, 0, new QTableWidgetItem(QString::number(t.number)));
        table_->setItem(r, 1, new QTableWidgetItem(qs(t.name)));
        table_->setItem(r, 2, new QTableWidgetItem(type));
        table_->setItem(r, 3,
                        new QTableWidgetItem(tooling::isTurningTool(t.type)
                                                 ? QString()
                                                 : formatLength(t.mill.diameter, t.unit)));
    }
    applyFilter(true);
    refreshHolderCombo();
    updateIssues();
}

tooling::ToolFilter ToolLibraryDialog::toolFilter() const {
    tooling::ToolFilter filter;
    filter.text = ss(filterText_->text());
    const QString typeFilter = filterType_->currentData().toString();
    if (typeFilter.startsWith(QStringLiteral("family:"))) {
        for (auto family : {tooling::ToolFamily::Milling, tooling::ToolFamily::HoleMaking,
                            tooling::ToolFamily::Turning, tooling::ToolFamily::Probing}) {
            if (typeFilter.mid(7) == familyId(family)) {
                filter.family = family;
            }
        }
    } else if (typeFilter.startsWith(QStringLiteral("type:"))) {
        filter.type = tooling::parseToolType(ss(typeFilter.mid(5)));
    }
    filter.minDiameter = filterMin_->mm();
    filter.maxDiameter = filterMax_->mm();
    return filter;
}

void ToolLibraryDialog::setToolFilter(const tooling::ToolFilter& filter) {
    {
        const QSignalBlocker b1(filterText_), b2(filterType_), b3(filterMin_), b4(filterMax_);
        filterText_->setText(qs(filter.text));
        QString typeFilter;
        if (filter.type) {
            typeFilter = QStringLiteral("type:") + typeId(*filter.type);
        } else if (filter.family) {
            typeFilter = QStringLiteral("family:") + familyId(*filter.family);
        }
        filterType_->setCurrentIndex(std::max(0, filterType_->findData(typeFilter)));
        filterMin_->setMm(filter.minDiameter);
        filterMax_->setMm(filter.maxDiameter);
    }
    QSettings settings;
    settings.setValue(kFilterTypeKey, filterType_->currentData().toString());
    settings.setValue(kFilterMinKey, filterMin_->mm());
    settings.setValue(kFilterMaxKey, filterMax_->mm());
    applyFilter(false);
}

void ToolLibraryDialog::applyFilter(bool keepCurrent) {
    const tooling::ToolFilter filter = toolFilter();
    const int total = table_->rowCount();
    int shown = 0;
    int first = -1;
    for (int r = 0; r < total; ++r) {
        const bool show = filter.matches(lib_.tools[static_cast<std::size_t>(r)]) ||
                          (keepCurrent && r == current_);
        table_->setRowHidden(r, !show);
        if (show) {
            ++shown;
            first = first < 0 ? r : first;
        }
    }
    filterCount_->setText(shown == total ? tr("%n tool(s)", nullptr, total)
                                         : tr("%1 of %n tool(s) shown", nullptr, total).arg(shown));
    if (!keepCurrent && (current_ < 0 || table_->isRowHidden(current_))) {
        selectRow(first);
    }
}

std::vector<int> ToolLibraryDialog::selectedToolRows() const {
    std::set<int> rows;
    for (const auto& range : table_->selectedRanges()) {
        for (int r = range.topRow(); r <= range.bottomRow(); ++r) {
            if (!table_->isRowHidden(r)) {
                rows.insert(r);
            }
        }
    }
    return {rows.begin(), rows.end()};
}

bool ToolLibraryDialog::selectToolNumber(int number) {
    for (int r = 0; r < static_cast<int>(lib_.tools.size()); ++r) {
        if (lib_.tools[static_cast<std::size_t>(r)].number == number && !table_->isRowHidden(r)) {
            selectRow(r);
            return true;
        }
    }
    return false;
}

void ToolLibraryDialog::refreshCurrentRow() {
    const tooling::Tool* t = currentTool();
    if (!t) {
        return;
    }
    const QSignalBlocker block(table_);
    const QString type = trToolType(t->type);
    table_->item(current_, 0)->setText(QString::number(t->number));
    table_->item(current_, 1)->setText(qs(t->name));
    table_->item(current_, 2)->setText(type);
    table_->item(current_, 3)
        ->setText(tooling::isTurningTool(t->type) ? QString()
                                                  : formatLength(t->mill.diameter, t->unit));
}

void ToolLibraryDialog::selectRow(int row) {
    current_ = row;
    if (row < 0) {
        const QSignalBlocker block(table_);
        table_->clearSelection();
    } else if (table_->currentRow() != row) {
        const QSignalBlocker block(table_);
        table_->selectRow(row);
    }
    loadEditor();
}

void ToolLibraryDialog::loadEditor() {
    const tooling::Tool* t = currentTool();
    form_->parentWidget()->setEnabled(t != nullptr);
    if (!t) {
        loadPresets();
        return;
    }
    loading_ = true;
    const double f = unitFactor(t->unit);
    unit_->setCurrentIndex(t->unit == LengthUnit::Inch ? 1 : 0);
    applyUnitSuffixes(t->unit);
    number_->setValue(t->number);
    name_->setText(qs(t->name));
    type_->setCurrentIndex(type_->findData(static_cast<int>(t->type)));
    material_->setText(qs(t->material));
    coating_->setText(qs(t->coating));
    vendor_->setText(qs(t->vendor));
    partNumber_->setText(qs(t->partNumber));
    comment_->setText(qs(t->comment));
    const tooling::MillGeometry& g = t->mill;
    lengths_["diameter"]->setValue(g.diameter / f);
    lengths_["cornerRadius"]->setValue(g.cornerRadius / f);
    lengths_["fluteLength"]->setValue(g.fluteLength / f);
    lengths_["overallLength"]->setValue(g.overallLength / f);
    lengths_["shankDiameter"]->setValue(g.shankDiameter / f);
    lengths_["tipAngle"]->setValue(radToDeg(g.tipAngle));
    lengths_["tipDiameter"]->setValue(g.tipDiameter / f);
    lengths_["tipLength"]->setValue(g.tipLength / f);
    lengths_["threadPitch"]->setValue(g.threadPitch / f);
    threadHand_->setCurrentIndex(g.leftHand ? 1 : 0);
    lengths_["neckDiameter"]->setValue(g.neckDiameter / f);
    lengths_["neckLength"]->setValue(g.neckLength / f);
    lengths_["triggerOffset"]->setValue(g.triggerOffset / f);
    flutes_->setValue(g.flutes);
    const tooling::TurningInsert& in = t->insert;
    {
        // A letter from an older library that is not in the list stays as it is.
        const QString shape = qs(in.shape).toUpper();
        int at = insertShape_->findData(shape.left(1));
        if (at < 0) {
            insertShape_->addItem(shape, shape);
            at = insertShape_->count() - 1;
        }
        insertShape_->setCurrentIndex(at);
    }
    lengths_["inscribedCircle"]->setValue(in.inscribedCircle / f);
    lengths_["noseRadius"]->setValue(in.noseRadius / f);
    lengths_["thickness"]->setValue(in.thickness / f);
    lengths_["reliefAngle"]->setValue(radToDeg(in.reliefAngle));
    lengths_["cuttingEdge"]->setValue(radToDeg(in.cuttingEdgeAngle));
    lengths_["insertWidth"]->setValue(in.width / f);
    lengths_["insertPitch"]->setValue(in.threadPitch / f);
    if (t->solid) {
        const tooling::SolidBar& solid = *t->solid;
        lengths_["solidMinBore"]->setValue(solid.minBore / f);
        lengths_["solidShank"]->setValue(solid.shankDiameter / f);
        lengths_["solidHead"]->setValue(solid.headLength / f);
        lengths_["solidLength"]->setValue(solid.overallLength / f);
        lengths_["solidOverhang"]->setValue(solid.maxOverhang / f);
        lengths_["solidTip"]->setValue(solid.centreToTip / f);
        lengths_["solidDepth"]->setValue(solid.maxDepth / f);
    }
    hand_->setCurrentText(qs(in.hand));
    orientation_->setValue(in.orientation);
    locking_->setCurrentIndex(locking_->findData(static_cast<int>(in.locking)));
    refreshHolderCombo();
    stickout_->setValue(t->stickout / f);
    loading_ = false;

    updateFieldVisibility();
    updateGauge();
    loadPresets();
    preview_->setTool(*t, t->holderId ? lib_.findHolder(*t->holderId) : nullptr);
}

void ToolLibraryDialog::applyUnitSuffixes(LengthUnit unit) {
    const bool inch = unit == LengthUnit::Inch;
    auto apply = [&](QDoubleSpinBox* spin) {
        spin->setSuffix(lengthSuffix(unit));
        spin->setDecimals(inch ? 5 : 4);
        spin->setMaximum(inch ? 400.0 : 10000.0);
    };
    for (const auto& f : kMillFields) {
        if (!f.angle) {
            apply(lengths_[QString::fromLatin1(f.id)]);
        }
    }
    for (const auto& f : kInsertFields) {
        if (!f.angle) {
            apply(lengths_[QString::fromLatin1(f.id)]);
        }
    }
    for (const auto& f : kSolidFields) {
        apply(lengths_[QString::fromLatin1(f.id)]);
    }
    apply(stickout_);
    apply(gauge_);
}

void ToolLibraryDialog::updateFieldVisibility() {
    const tooling::Tool* t = currentTool();
    if (!t) {
        return;
    }
    const bool turning = tooling::isTurningTool(t->type);
    const std::set<QString> optional = fieldsFor(t->type);
    for (const auto& f : kMillFields) {
        const QString id = QString::fromLatin1(f.id);
        const bool always = id == "diameter" || id == "fluteLength" || id == "overallLength" ||
                            id == "shankDiameter";
        form_->setRowVisible(lengths_[id], !turning && (always || optional.contains(id)));
    }
    const bool probe = t->type == ToolType::Probe;
    // A bar puller (DL-0091) says what bar it takes and how deep, not what it cuts.
    const bool puller = t->type == ToolType::BarPuller;
    const auto relabel = [&](const char* id, const QString& text) {
        if (auto* label = qobject_cast<QLabel*>(form_->labelForField(lengths_[id]))) {
            label->setText(text);
        }
    };
    relabel("diameter", puller ? tr("Body diameter") : tr("Diameter"));
    relabel("fluteLength", puller ? tr("Grip (depth of its jaws)") : tr("Flute length"));
    relabel("tipDiameter", puller ? tr("Largest bar") : tr("Tip diameter"));
    form_->setRowVisible(flutes_, !turning && !probe && !puller);
    form_->setRowVisible(threadHand_, t->type == ToolType::Tap);
    // A probe's sizes are its stylus's and its body's.
    const std::pair<const char*, QString> probeLabels[] = {
        {"diameter", probe ? tr("Stylus diameter") : tr("Diameter")},
        {"fluteLength", probe ? tr("Stylus length") : tr("Flute length")},
        {"neckDiameter", probe ? tr("Stem diameter") : tr("Neck diameter")},
        {"shankDiameter", probe ? tr("Body diameter") : tr("Shank diameter")},
    };
    for (const auto& [id, label] : probeLabels) {
        if (auto* rowLabel = qobject_cast<QLabel*>(form_->labelForField(lengths_[id]))) {
            rowLabel->setText(label);
        }
    }
    lengths_["triggerOffset"]->setToolTip(
        tr("How far the stylus ball travels past first contact before the probe triggers, as "
           "its calibration finds it: the ball's effective radius is its radius less this"));
    for (const auto& f : kInsertFields) {
        const QString id = QString::fromLatin1(f.id);
        form_->setRowVisible(lengths_[id], turning && optional.contains(id));
    }
    form_->setRowVisible(insertShape_, t->type == ToolType::TurningInsert);
    form_->setRowVisible(hand_, turning);
    form_->setRowVisible(orientation_, turning);
    form_->setRowVisible(locking_, t->type == ToolType::TurningInsert && !t->solid);
    // A solid bar has no insert or holder: its tip follows from its dimensions.
    const bool solid = t->solid.has_value();
    for (const auto& f : kSolidFields) {
        form_->setRowVisible(lengths_[QString::fromLatin1(f.id)], solid);
    }
    if (solid) {
        form_->setRowVisible(insertShape_, false);
        form_->setRowVisible(lengths_["inscribedCircle"], false);
        form_->setRowVisible(lengths_["thickness"], false);
        form_->setRowVisible(orientation_, false);
        form_->setRowVisible(holder_, false);
    } else {
        form_->setRowVisible(holder_, true);
    }
    // Turning tools sit in their holder by their insert: no stickout or gauge length.
    form_->setRowVisible(stickout_, !turning);
    form_->setRowVisible(gauge_, !turning);
    // What an ISO holder gives is shown, not edited here.
    const tooling::Holder* holder = t->holderId ? lib_.findHolder(*t->holderId) : nullptr;
    const bool byHolder = holder && holder->iso && t->type == ToolType::TurningInsert;
    for (QWidget* w : std::initializer_list<QWidget*>{
             insertShape_, lengths_["inscribedCircle"], lengths_["reliefAngle"],
             lengths_["cuttingEdge"], hand_, orientation_}) {
        w->setEnabled(!byHolder);
    }
}

void ToolLibraryDialog::updateGauge() {
    const tooling::Tool* t = currentTool();
    if (!t || tooling::isTurningTool(t->type)) {
        return;
    }
    const tooling::Holder* holder = t->holderId ? lib_.findHolder(*t->holderId) : nullptr;
    const double f = unitFactor(t->unit);
    const QSignalBlocker block(gauge_);
    // At least a little longer than the holder: the tool has to come out of it.
    gauge_->setMinimum(holder ? holder->length() / f + std::pow(10.0, -gauge_->decimals()) : 0.0);
    gauge_->setValue(tooling::gaugeLength(*t, holder) / f);
}

void ToolLibraryDialog::commitGauge() {
    const tooling::Tool* t = currentTool();
    if (loading_ || !t) {
        return;
    }
    const tooling::Holder* holder = t->holderId ? lib_.findHolder(*t->holderId) : nullptr;
    const double f = unitFactor(t->unit);
    const double stickout = tooling::stickoutForGauge(gauge_->value() * f, holder);
    if (stickout <= 0.0) {
        updateGauge();
        return;
    }
    {
        const QSignalBlocker block(stickout_);
        stickout_->setValue(stickout / f);
    }
    commitEditor();
}

void ToolLibraryDialog::commitEditor() {
    tooling::Tool* t = currentTool();
    if (loading_ || !t) {
        return;
    }
    const auto oldType = t->type;
    const auto oldHolder = t->holderId;
    const tooling::TurningInsert oldInsert = t->insert;
    const LengthUnit oldUnit = t->unit;
    t->unit = unit_->currentIndex() == 1 ? LengthUnit::Inch : LengthUnit::Millimeter;
    if (t->unit != oldUnit) {
        // Only the presentation changes: show the stored values in the new unit.
        loadEditor();
        refreshCurrentRow();
        return;
    }
    const double f = unitFactor(t->unit);
    t->number = number_->value();
    t->name = ss(name_->text());
    t->type = static_cast<ToolType>(type_->currentData().toInt());
    t->material = ss(material_->text());
    t->coating = ss(coating_->text());
    t->vendor = ss(vendor_->text());
    t->partNumber = ss(partNumber_->text());
    t->comment = ss(comment_->text());
    tooling::MillGeometry& g = t->mill;
    g.diameter = lengths_["diameter"]->value() * f;
    g.cornerRadius = lengths_["cornerRadius"]->value() * f;
    g.fluteLength = lengths_["fluteLength"]->value() * f;
    g.overallLength = lengths_["overallLength"]->value() * f;
    g.shankDiameter = lengths_["shankDiameter"]->value() * f;
    g.tipAngle = degToRad(lengths_["tipAngle"]->value());
    g.tipDiameter = lengths_["tipDiameter"]->value() * f;
    g.tipLength = lengths_["tipLength"]->value() * f;
    g.threadPitch = lengths_["threadPitch"]->value() * f;
    g.leftHand = t->type == ToolType::Tap && threadHand_->currentIndex() == 1;
    g.neckDiameter = lengths_["neckDiameter"]->value() * f;
    g.neckLength = lengths_["neckLength"]->value() * f;
    g.triggerOffset = lengths_["triggerOffset"]->value() * f;
    g.flutes = flutes_->value();
    tooling::TurningInsert& in = t->insert;
    in.shape = ss(insertShape_->currentData().toString());
    in.inscribedCircle = lengths_["inscribedCircle"]->value() * f;
    in.noseRadius = lengths_["noseRadius"]->value() * f;
    in.thickness = lengths_["thickness"]->value() * f;
    in.reliefAngle = degToRad(lengths_["reliefAngle"]->value());
    in.cuttingEdgeAngle = degToRad(lengths_["cuttingEdge"]->value());
    in.width = lengths_["insertWidth"]->value() * f;
    in.threadPitch = lengths_["insertPitch"]->value() * f;
    in.hand = ss(hand_->currentText());
    in.orientation = orientation_->value();
    in.locking = static_cast<tooling::InsertLocking>(locking_->currentData().toInt());
    if (t->solid) {
        t->solid = tooling::SolidBar{
            lengths_["solidMinBore"]->value() * f,  lengths_["solidShank"]->value() * f,
            lengths_["solidHead"]->value() * f,     lengths_["solidLength"]->value() * f,
            lengths_["solidOverhang"]->value() * f, lengths_["solidTip"]->value() * f,
            lengths_["solidDepth"]->value() * f};
        tooling::fitSolidBar(*t);
    }
    const QString holderId = holder_->currentData().toString();
    t->holderId = holderId.isEmpty() ? std::nullopt : Uuid::parse(ss(holderId));
    t->stickout = stickout_->value() * f;
    if (const tooling::Holder* holder = t->holderId ? lib_.findHolder(*t->holderId) : nullptr) {
        if (t->holderId != oldHolder) {
            tooling::fitToHolder(*t, *holder);
        } else if (holder->iso && t->type == ToolType::TurningInsert) {
            // These are not edited while it sits in the holder, and editing something else leaves
            // them as they are, even when they do not fit.
            const tooling::InsertLocking locking = in.locking;
            const double nose = in.noseRadius;
            const double thickness = in.thickness;
            in = oldInsert;
            in.locking = locking;
            in.noseRadius = nose;
            in.thickness = thickness;
        }
    }

    if (t->type != oldType) {
        // Switching type: start from that type's defaults at the same size and number.
        tooling::Tool fresh =
            tooling::makeDefaultTool(t->type, t->number, g.diameter > 0 ? g.diameter : 10.0);
        fresh.id = t->id;
        fresh.unit = t->unit;
        fresh.presets = t->presets;
        // The holder stays when it holds the new type; turning tools have no stickout.
        const tooling::Holder* holder = t->holderId ? lib_.findHolder(*t->holderId) : nullptr;
        fresh.holderId = holder && !tooling::holds(*holder, t->type) ? std::nullopt : t->holderId;
        fresh.stickout = tooling::isTurningTool(t->type) ? 0.0 : t->stickout;
        *t = fresh;
        if (const tooling::Holder* kept = t->holderId ? lib_.findHolder(*t->holderId) : nullptr) {
            tooling::fitToHolder(*t, *kept);
        }
        loadEditor();
    } else if (t->holderId != oldHolder) {
        loadEditor(); // an ISO holder sets the insert
    } else {
        refreshHolderCombo(); // what fits follows the insert
        updateFieldVisibility();
        updateGauge();
        preview_->setTool(*t, t->holderId ? lib_.findHolder(*t->holderId) : nullptr);
    }
    refreshCurrentRow();
    updateIssues();
}

void ToolLibraryDialog::loadPresets() {
    const tooling::Tool* t = currentTool();
    presets_->clear();
    editPresets_->setEnabled(t != nullptr);
    if (!t) {
        return;
    }
    const LengthUnit unit = t->unit;
    const bool turning = tooling::isTurningTool(t->type);
    for (const tooling::CuttingPreset& p : t->presets) {
        QString text = p.name.empty() ? tr("(unnamed)") : qs(p.name);
        if (!p.materialGroup.empty()) {
            text += QStringLiteral("  [%1]").arg(qs(p.materialGroup));
        }
        const QString speed = QStringLiteral("%1%2")
                                  .arg(surfaceSpeedIn(p.surfaceSpeed, unit), 0, 'f', 0)
                                  .arg(surfaceSpeedSuffix(unit));
        QStringList values;
        values << QStringLiteral("vc ") + speed;
        values << (turning ? tr("f %1").arg(formatLength(p.feedPerTooth, unit))
                           : tr("fz %1").arg(formatLength(p.feedPerTooth, unit)));
        if (t->type == tooling::ToolType::TurningInsert) {
            values << tr("lead f %1").arg(formatLength(p.leadFeedPerRev, unit));
        }
        if (t->type == tooling::ToolType::PartingInsert ||
            t->type == tooling::ToolType::GroovingInsert) {
            values << tr("part-off end f %1").arg(formatLength(p.partOffEndFeed, unit));
        }
        if (!turning) {
            values << formatFeed(p.feedRate(t->mill.diameter, t->mill.flutes), unit);
            values << tr("plunge %1").arg(formatFeed(p.plungeFeed, unit));
            values << tr("ramp %1").arg(formatFeed(p.rampFeed, unit));
            values << tr("lead %1").arg(formatFeed(p.leadFeed, unit));
        }
        if (p.axialDepth > 0.0) {
            values << tr("ap %1").arg(formatLength(p.axialDepth, unit));
        }
        if (p.radialDepth > 0.0) {
            values << tr("ae %1").arg(formatLength(p.radialDepth, unit));
        }
        // One line each: the name and the cutting speed; the rest in the tooltip.
        auto* item = new QListWidgetItem(QStringLiteral("%1  -  %2").arg(text, speed), presets_);
        item->setToolTip(text + QLatin1Char('\n') + values.join(QStringLiteral(", ")));
    }
    if (t->presets.empty()) {
        auto* none = new QListWidgetItem(tr("No presets yet"));
        none->setToolTip(tr("Operations take their speeds and feeds from a preset: add one"));
        none->setFlags(Qt::NoItemFlags);
        presets_->addItem(none);
    }
}

bool ToolLibraryDialog::editPresets(int index) {
    tooling::Tool* t = currentTool();
    if (!t) {
        return false;
    }
    CuttingPresetDialog dialog(*t, index, this);
    if (index < 0) {
        dialog.addPreset();
    }
    if (dialog.exec() != QDialog::Accepted) {
        return false;
    }
    t->presets = dialog.presets();
    loadPresets();
    updateIssues();
    return true;
}

void ToolLibraryDialog::updateIssues() {
    updateTitle();
    const auto issues = lib_.validate();
    if (issues.empty()) {
        issues_->setText(
            tr("%n tool(s), ", nullptr, static_cast<int>(lib_.tools.size())) +
            tr("%n holder(s), no problems.", nullptr, static_cast<int>(lib_.holders.size())));
        issues_->setStyleSheet(QString());
        return;
    }
    QStringList lines;
    for (std::size_t i = 0; i < issues.size() && i < 6; ++i) {
        lines << qs(issues[i]);
    }
    if (issues.size() > 6) {
        lines << tr("... and %1 more").arg(issues.size() - 6);
    }
    issues_->setText(lines.join(QLatin1Char('\n')));
    issues_->setStyleSheet(QStringLiteral("color: #c0392b;"));
}

void ToolLibraryDialog::addTool(ToolType type) {
    // A typical size in the preferred unit: 10 mm, or 1/2 inch.
    const LengthUnit unit = preferredUnit();
    const bool inch = unit == LengthUnit::Inch;
    tooling::Tool tool = tooling::makeDefaultTool(type, lib_.nextFreeNumber(), inch ? 12.7 : 10.0);
    tool.unit = unit;
    if (inch && !tooling::isTurningTool(type)) {
        tool.name = ss(QStringLiteral("%1 Ø%2\"")
                           .arg(trToolType(type))
                           .arg(tool.mill.diameter / 25.4, 0, 'g', 6));
    }
    lib_.tools.push_back(std::move(tool));
    current_ = static_cast<int>(lib_.tools.size()) - 1;
    rebuildTable();
    selectRow(current_);
}

void ToolLibraryDialog::addSolidBoringBar() {
    tooling::Tool tool = tooling::makeSolidBoringBar(lib_.nextFreeNumber());
    tool.unit = preferredUnit();
    lib_.tools.push_back(std::move(tool));
    current_ = static_cast<int>(lib_.tools.size()) - 1;
    rebuildTable();
    selectRow(current_);
}

void ToolLibraryDialog::duplicateTool() {
    const tooling::Tool* t = currentTool();
    if (!t) {
        return;
    }
    tooling::Tool copy = *t;
    copy.id = Uuid::generate();
    copy.number = lib_.nextFreeNumber();
    copy.name += " (copy)";
    for (auto& p : copy.presets) {
        p.id = Uuid::generate();
    }
    lib_.tools.push_back(copy);
    rebuildTable();
    selectRow(static_cast<int>(lib_.tools.size()) - 1);
}

void ToolLibraryDialog::deleteTools() {
    const std::vector<int> selected = selectedToolRows();
    if (selected.empty()) {
        return;
    }
    const std::set<int, std::greater<>> rows(selected.begin(), selected.end());
    for (int r : rows) {
        lib_.tools.erase(lib_.tools.begin() + r);
    }
    current_ = -1;
    rebuildTable();
    // The next tool shown after the deleted ones, else the last one shown.
    int next = -1;
    for (int r = 0; r < table_->rowCount(); ++r) {
        if (!table_->isRowHidden(r)) {
            next = r;
            if (r >= selected.front()) {
                break;
            }
        }
    }
    selectRow(next);
}

void ToolLibraryDialog::openFile() {
    if (!maybeSave()) {
        return;
    }
    const QString path = QFileDialog::getOpenFileName(this, tr("Open tool library"), QString(),
                                                      tr("OxCAM tool libraries (*.ocamtools)"));
    if (path.isEmpty()) {
        return;
    }
    try {
        const std::filesystem::path file(path.toStdU16String());
        lib_ = tooling::loadToolLibrary(file);
        saved_ = lib_;
        if (libraryName_) {
            libraryName_->setText(QString::fromStdString(lib_.name));
        }
        setFile(file);
        current_ = -1;
        rebuildTable();
        applyFilter(false);
        currentHolder_ = lib_.holders.empty() ? -1 : 0;
        rebuildHolderTable();
        selectHolder(currentHolder_);
    } catch (const Error& e) {
        QMessageBox::warning(this, tr("Open tool library"), QString::fromUtf8(e.what()));
    }
}

bool ToolLibraryDialog::saveFile(bool askForPath) {
    std::optional<std::filesystem::path> target = askForPath ? std::nullopt : file_;
    if (!target) {
        const QString path = QFileDialog::getSaveFileName(this, tr("Save tool library"), QString(),
                                                          tr("OxCAM tool libraries (*.ocamtools)"));
        if (path.isEmpty()) {
            return false;
        }
        target = std::filesystem::path(path.toStdU16String());
    }
    try {
        if (libraryName_) {
            lib_.name = libraryName_->text().trimmed().toStdString();
        }
        tooling::saveToolLibrary(lib_, *target);
        saved_ = lib_;
        setFile(target);
        return true;
    } catch (const Error& e) {
        QMessageBox::warning(this, tr("Save tool library"), QString::fromUtf8(e.what()));
        return false;
    }
}

// -- Holders ------------------------------------------------------------------------------------

namespace {

double holderFactor() {
    return unitFactor(preferredUnit());
}

enum HolderColumn { HName, HKind, HDiameter, HLength, HTools, HCount };

QString holderKindText(tooling::HolderKind kind) {
    return kind == tooling::HolderKind::Turning
               ? QCoreApplication::translate("ocam::gui::ToolLibraryDialog", "Turning")
               : QCoreApplication::translate("ocam::gui::ToolLibraryDialog", "Milling");
}

} // namespace

QWidget* ToolLibraryDialog::buildHolderList() {
    auto* page = new QWidget;
    auto* layout = new QVBoxLayout(page);
    layout->setContentsMargins(0, 0, 0, 0);
    holderTable_ = new QTableWidget(0, HCount);
    holderTable_->setHorizontalHeaderLabels(
        {tr("Name"), tr("Holds"), tr("Ø"), tr("Length"), tr("Tools")});
    holderTable_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    holderTable_->horizontalHeader()->setSectionResizeMode(HName, QHeaderView::Stretch);
    holderTable_->setObjectName(QStringLiteral("holderTable"));
    holderTable_->verticalHeader()->hide();
    holderTable_->setSelectionBehavior(QAbstractItemView::SelectRows);
    holderTable_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    layout->addWidget(holderTable_);
    connect(holderTable_, &QTableWidget::currentCellChanged, this,
            [this](int row, int, int, int) { selectHolder(row); });

    auto* buttons = new QHBoxLayout;
    auto* add = new QPushButton(tr("New holder"));
    auto* addMenu = new QMenu(add);
    addMenu
        ->addAction(tr("Turning: square shank (ISO 5608)"), this,
                    [this] { addTurningHolder(false); })
        ->setToolTip(tr("A stick tool holder for indexable inserts, set by its designation"));
    addMenu
        ->addAction(tr("Turning: boring bar (ISO 5609)"), this, [this] { addTurningHolder(true); })
        ->setToolTip(tr("A round bar for indexable inserts, set by its designation"));
    addMenu->addAction(tr("Turning: plain shank"), this, [this] { addTurningStick(false); })
        ->setToolTip(tr("A rectangular shank by its sizes, for grooving, threading and parting "
                        "tools, whose holders are the makers' own"));
    addMenu->addAction(tr("Turning: plain bar"), this, [this] { addTurningStick(true); })
        ->setToolTip(tr("A round bar by its sizes, for inside grooving and threading tools"));
    addMenu->addAction(tr("Milling: stepped holder"), this, &ToolLibraryDialog::addHolder)
        ->setToolTip(tr("A stepped holder: set its dimensions or edit its points"));
    addMenu->setToolTipsVisible(true);
    add->setMenu(addMenu);
    auto* import = new QPushButton(tr("Import STEP..."));
    import->setToolTip(tr("A holder from the vendor's solid model: its outline turned about "
                          "its axis"));
    connect(import, &QPushButton::clicked, this, [this] {
        const QString path =
            QFileDialog::getOpenFileName(this, tr("Import holder model"), QString(),
                                         tr("STEP files (*.step *.stp *.STEP *.STP)"));
        if (!path.isEmpty()) {
            importHolder(path);
        }
    });
    auto* duplicate = new QPushButton(tr("Duplicate"));
    connect(duplicate, &QPushButton::clicked, this, &ToolLibraryDialog::duplicateHolder);
    auto* remove = new QPushButton(tr("Delete"));
    connect(remove, &QPushButton::clicked, this, &ToolLibraryDialog::deleteHolders);
    buttons->addWidget(add);
    buttons->addWidget(import);
    buttons->addWidget(duplicate);
    buttons->addWidget(remove);
    layout->addLayout(buttons);
    return page;
}

QWidget* ToolLibraryDialog::buildHolderEditor() {
    holderEditor_ = new QWidget;
    auto* layout = new QVBoxLayout(holderEditor_);
    auto* form = new QFormLayout;
    holderForm_ = form;
    holderName_ = new QLineEdit;
    holderKind_ = new QComboBox;
    holderKind_->addItem(tr("Milling and drilling tools"),
                         static_cast<int>(tooling::HolderKind::Milling));
    holderKind_->addItem(tr("Turning tools"), static_cast<int>(tooling::HolderKind::Turning));
    holderKind_->setToolTip(tr("Which tools the holder takes: a tool's holder is picked among "
                               "the holders of its kind"));
    holderVendor_ = new QLineEdit;
    holderPartNumber_ = new QLineEdit;
    holderComment_ = new QLineEdit;
    form->addRow(tr("Name"), holderName_);
    form->addRow(tr("Holds"), holderKind_);
    form->addRow(tr("Vendor"), holderVendor_);
    form->addRow(tr("Part number"), holderPartNumber_);
    form->addRow(tr("Comment"), holderComment_);
    layout->addLayout(form);
    layout->addWidget(buildIsoHolderBox(), 1);
    layout->addWidget(buildStickHolderBox());
    for (auto* line : {holderName_, holderVendor_, holderPartNumber_, holderComment_}) {
        connect(line, &QLineEdit::editingFinished, this, &ToolLibraryDialog::commitHolderEditor);
    }
    connect(holderKind_, &QComboBox::currentIndexChanged, this,
            &ToolLibraryDialog::commitHolderEditor);

    auto* outlineBox = new QGroupBox(tr("Outline"));
    holderOutline_ = outlineBox;
    auto* outlineLayout = new QVBoxLayout(outlineBox);
    auto* help = new QLabel(tr("Radius and height of each point, from the axis at the tool face "
                               "(height 0) out and up to the axis at the gauge line."));
    help->setWordWrap(true);
    outlineLayout->addWidget(help);
    holderPoints_ = new QTableWidget(0, 2);
    holderPoints_->verticalHeader()->hide();
    holderPoints_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    outlineLayout->addWidget(holderPoints_, 1);
    connect(holderPoints_, &QTableWidget::itemChanged, this,
            &ToolLibraryDialog::commitHolderPoints);
    auto* pointButtons = new QHBoxLayout;
    auto* addPoint = new QPushButton(tr("Add point"));
    auto* removePoint = new QPushButton(tr("Remove point"));
    auto* dimensions = new QPushButton(tr("From dimensions..."));
    dimensions->setToolTip(tr("Replaces the outline by a stepped one: nose, body and flange"));
    pointButtons->addWidget(addPoint);
    pointButtons->addWidget(removePoint);
    pointButtons->addStretch();
    pointButtons->addWidget(dimensions);
    outlineLayout->addLayout(pointButtons);
    layout->addWidget(outlineBox, 1);
    holderInfo_ = new QLabel;
    holderInfo_->setWordWrap(true);
    layout->addWidget(holderInfo_);

    connect(addPoint, &QPushButton::clicked, this, [this] {
        const int row = std::max(0, holderPoints_->currentRow());
        const int at = std::min(row + 1, holderPoints_->rowCount());
        const QSignalBlocker block(holderPoints_);
        holderPoints_->insertRow(at);
        for (int c = 0; c < 2; ++c) {
            const QTableWidgetItem* from = holderPoints_->item(row, c);
            holderPoints_->setItem(at, c,
                                   new QTableWidgetItem(from ? from->text() : QStringLiteral("0")));
        }
        holderPoints_->setCurrentCell(at, 0);
        commitHolderPoints();
    });
    connect(removePoint, &QPushButton::clicked, this, [this] {
        const int row = holderPoints_->currentRow();
        if (row >= 0 && holderPoints_->rowCount() > 2) {
            const QSignalBlocker block(holderPoints_);
            holderPoints_->removeRow(row);
            commitHolderPoints();
        }
    });
    connect(dimensions, &QPushButton::clicked, this, &ToolLibraryDialog::holderFromDimensions);
    return holderEditor_;
}

QWidget* ToolLibraryDialog::buildStickHolderBox() {
    stickBox_ = new QGroupBox(tr("Plain holder"));
    auto* form = new QFormLayout(stickBox_);
    stickType_ = new QComboBox;
    stickType_->setObjectName(QStringLiteral("stickType"));
    stickType_->addItem(tr("Shank, for outside and face cuts"), false);
    stickType_->addItem(tr("Bar, for inside cuts"), true);
    const auto makeLength = [&](const QString& tip) {
        auto* spin = new QDoubleSpinBox;
        spin->setDecimals(3);
        spin->setRange(0.0, 10000.0);
        spin->setToolTip(tip);
        connect(spin, &QDoubleSpinBox::editingFinished, this,
                &ToolLibraryDialog::commitStickHolder);
        return spin;
    };
    stickWidth_ = makeLength(tr("The shank's width, or the bar's diameter"));
    stickF_ = makeLength(tr("Dimension f: how far the tool's tip stands from the back of the "
                            "shank, measured across the shank; on a bar, from the bar's axis"));
    stickLength_ = makeLength(tr("From the tip along the holder"));
    connect(stickType_, &QComboBox::currentIndexChanged, this,
            &ToolLibraryDialog::commitStickHolder);
    form->addRow(tr("Holder"), stickType_);
    form->addRow(tr("Width or diameter"), stickWidth_);
    form->addRow(tr("Tip to back of shank (f)"), stickF_);
    if (auto* label = qobject_cast<QLabel*>(form->labelForField(stickF_))) {
        label->setObjectName(QStringLiteral("stickFLabel"));
    }
    form->addRow(tr("Length"), stickLength_);
    return stickBox_;
}

void ToolLibraryDialog::commitStickHolder() {
    tooling::Holder* h = currentHolder();
    if (loadingHolder_ || !h || !h->stick) {
        return;
    }
    const double f = holderFactor();
    tooling::setTurningStick(
        *h, tooling::TurningStick{stickType_->currentData().toBool(), stickWidth_->value() * f,
                                  stickF_->value() * f, stickLength_->value() * f});
    loadHolderEditor();
    refreshHolderRow();
    refreshHolderCombo();
    updateIssues();
}

QWidget* ToolLibraryDialog::buildIsoHolderBox() {
    isoBox_ = new QGroupBox(tr("ISO designation"));
    auto* scrollContent = new QWidget;
    isoForm_ = new QFormLayout(scrollContent);
    auto* outer = new QVBoxLayout(isoBox_);
    auto* scroll = new QScrollArea;
    scroll->setFrameShape(QFrame::NoFrame);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scroll->setWidget(scrollContent);
    scroll->setWidgetResizable(true);
    outer->addWidget(scroll);

    isoDesignation_ = new QLineEdit;
    isoDesignation_->setObjectName(QStringLiteral("isoDesignation"));
    isoDesignation_->setToolTip(tr("Type or paste a catalogue code, such as PCLNR 2525M 12 or "
                                   "S25T-PCLNR 12, to set everything below"));
    isoType_ = new QComboBox;
    isoType_->setObjectName(QStringLiteral("isoType"));
    isoType_->addItem(tr("Square shank (ISO 5608)"), false);
    isoType_->addItem(tr("Boring bar (ISO 5609)"), true);
    auto letters = [](const std::vector<tooling::IsoLetter>& list, auto text) {
        auto* combo = new QComboBox;
        for (const tooling::IsoLetter& l : list) {
            combo->addItem(tr("%1: %2").arg(QChar::fromLatin1(l.letter), text(l)),
                           QString(QChar::fromLatin1(l.letter)));
        }
        return combo;
    };
    auto named = [](const tooling::IsoLetter& l) {
        return QCoreApplication::translate("Tooling", l.name);
    };
    isoBarType_ = letters(tooling::boringBarTypes(), named);
    isoClamping_ = letters(tooling::holderClampings(), named);
    isoShape_ = new QComboBox;
    for (const tooling::InsertShape& s : tooling::insertShapes()) {
        isoShape_->addItem(tr("%1: %2").arg(QChar::fromLatin1(s.letter),
                                            QCoreApplication::translate("Tooling", s.name)),
                           QString(QChar::fromLatin1(s.letter)));
    }
    isoStyle_ = letters(tooling::holderStyles(), [&](const tooling::IsoLetter& l) {
        return tr("%1°, %2").arg(l.value).arg(named(l));
    });
    isoStyle_->setToolTip(tr("The tool style: the cutting edge angle, which places the insert for "
                             "the toolpaths and the simulation, and how the shank is offset"));
    isoStyle_->setObjectName(QStringLiteral("isoStyle"));
    isoStyle_->addItem(tr("%1: custom").arg(QChar::fromLatin1(tooling::kCustomStyle)),
                       QString(QChar::fromLatin1(tooling::kCustomStyle)));
    isoStyleAngle_ = makeSpin(true);
    isoStyleAngle_->setObjectName(QStringLiteral("isoStyleAngle"));
    isoStyleAngle_->setToolTip(tr("The cutting edge's angle to the feed towards the chuck, which "
                                  "places the insert for the toolpaths and the simulation"));
    isoClearance_ = letters(tooling::insertClearances(),
                            [](const tooling::IsoLetter& l) { return tr("%1°").arg(l.value); });
    isoClearance_->setObjectName(QStringLiteral("isoClearance"));
    isoClearance_->addItem(tr("%1: custom").arg(QChar::fromLatin1(tooling::kCustomClearance)),
                           QString(QChar::fromLatin1(tooling::kCustomClearance)));
    isoClearanceAngle_ = makeSpin(true);
    isoClearanceAngle_->setObjectName(QStringLiteral("isoClearanceAngle"));
    isoClearanceAngle_->setToolTip(tr("The insert's clearance: the relief angle of the inserts "
                                      "this holder takes"));
    isoHand_ = new QComboBox;
    isoHand_->addItem(tr("R: right hand"), QStringLiteral("R"));
    isoHand_->addItem(tr("L: left hand"), QStringLiteral("L"));
    isoHand_->addItem(tr("N: either hand"), QStringLiteral("N"));
    isoHeight_ = makeSpin(false);
    isoWidth_ = makeSpin(false);
    isoLength_ = letters(tooling::holderLengths(),
                         [](const tooling::IsoLetter& l) { return tr("%1 mm").arg(l.value); });
    isoLength_->setObjectName(QStringLiteral("isoLength"));
    isoLength_->addItem(tr("%1: custom").arg(QChar::fromLatin1(tooling::kCustomLength)),
                        QString(QChar::fromLatin1(tooling::kCustomLength)));
    isoLengthValue_ = makeSpin(false);
    isoLengthValue_->setObjectName(QStringLiteral("isoLengthValue"));
    isoLengthValue_->setToolTip(tr("The holder's overall length"));
    isoSize_ = new QSpinBox;
    isoSize_->setRange(1, 99);
    isoSize_->setToolTip(tr("The insert's size symbol: the length of its side in whole "
                            "millimetres. It gives the inscribed circle."));
    isoF_ = makeSpin(false);
    isoF_->setToolTip(tr("Dimension f: how far the tool's tip stands from the back of the shank, "
                         "measured across the shank; on a boring bar, from the bar's axis. The "
                         "designation does not give it: this is the usual value for the size, "
                         "to check against the holder's data sheet."));
    isoForm_->addRow(tr("Designation"), isoDesignation_);
    isoForm_->addRow(tr("Holder"), isoType_);
    isoForm_->addRow(tr("Bar material"), isoBarType_);
    isoForm_->addRow(tr("Clamping"), isoClamping_);
    isoForm_->addRow(tr("Insert shape"), isoShape_);
    isoForm_->addRow(tr("Style"), isoStyle_);
    isoForm_->addRow(tr("Cutting edge angle"), isoStyleAngle_);
    isoForm_->addRow(tr("Insert clearance"), isoClearance_);
    isoForm_->addRow(tr("Clearance angle"), isoClearanceAngle_);
    isoForm_->addRow(tr("Hand"), isoHand_);
    isoForm_->addRow(tr("Height"), isoHeight_);
    isoForm_->addRow(tr("Width"), isoWidth_);
    isoForm_->addRow(tr("Length"), isoLength_);
    isoForm_->addRow(tr("Overall length"), isoLengthValue_);
    isoForm_->addRow(tr("Insert size"), isoSize_);
    isoForm_->addRow(tr("Tip to back of shank (f)"), isoF_);
    if (auto* label = qobject_cast<QLabel*>(isoForm_->labelForField(isoF_))) {
        label->setObjectName(QStringLiteral("isoFLabel"));
    }

    connect(isoDesignation_, &QLineEdit::editingFinished, this,
            &ToolLibraryDialog::commitIsoDesignation);
    for (QComboBox* combo : {isoType_, isoBarType_, isoClamping_, isoShape_, isoStyle_,
                             isoClearance_, isoHand_, isoLength_}) {
        // Long texts show in the open list; the form stays as narrow as the rest of the editor.
        combo->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
        combo->setMinimumContentsLength(14);
        connect(combo, &QComboBox::currentIndexChanged, this, &ToolLibraryDialog::commitIsoHolder);
    }
    for (QDoubleSpinBox* spin :
         {isoHeight_, isoWidth_, isoF_, isoStyleAngle_, isoClearanceAngle_, isoLengthValue_}) {
        connect(spin, &QDoubleSpinBox::valueChanged, this, &ToolLibraryDialog::commitIsoHolder);
    }
    connect(isoSize_, &QSpinBox::valueChanged, this, &ToolLibraryDialog::commitIsoHolder);
    return isoBox_;
}

void ToolLibraryDialog::loadIsoHolder(const tooling::TurningHolderCode& c) {
    const LengthUnit unit = preferredUnit();
    const double f = holderFactor();
    isoDesignation_->setText(qs(tooling::designation(c)));
    isoType_->setCurrentIndex(c.boringBar ? 1 : 0);
    auto choose = [](QComboBox* combo, char letter) {
        combo->setCurrentIndex(combo->findData(QString(QChar::fromLatin1(letter))));
    };
    choose(isoBarType_, c.barType);
    choose(isoClamping_, c.clamping);
    choose(isoShape_, c.insertShape);
    choose(isoStyle_, c.style);
    choose(isoClearance_, c.clearance);
    choose(isoHand_, c.hand);
    choose(isoLength_, c.length);
    for (QDoubleSpinBox* spin : {isoHeight_, isoWidth_, isoF_, isoLengthValue_}) {
        spin->setSuffix(lengthSuffix(unit));
    }
    isoStyleAngle_->setValue(c.customStyle.value_or(0.0));
    isoClearanceAngle_->setValue(c.customClearance.value_or(0.0));
    isoLengthValue_->setValue(c.customLength.value_or(0.0) / f);
    isoForm_->setRowVisible(isoStyleAngle_, c.style == tooling::kCustomStyle);
    isoForm_->setRowVisible(isoClearanceAngle_, c.clearance == tooling::kCustomClearance);
    isoForm_->setRowVisible(isoLengthValue_, c.length == tooling::kCustomLength);
    isoHeight_->setValue(c.height / f);
    isoWidth_->setValue(c.width / f);
    isoF_->setValue(c.f / f);
    isoSize_->setValue(c.insertSize);
    isoForm_->setRowVisible(isoBarType_, c.boringBar);
    isoForm_->setRowVisible(isoWidth_, !c.boringBar);
    if (auto* label = qobject_cast<QLabel*>(isoForm_->labelForField(isoHeight_))) {
        label->setText(c.boringBar ? tr("Diameter") : tr("Height"));
    }
    if (auto* label = qobject_cast<QLabel*>(isoForm_->labelForField(isoF_))) {
        label->setText(c.boringBar ? tr("Tip to bar's axis (f)") : tr("Tip to back of shank (f)"));
    }
}

void ToolLibraryDialog::commitIsoHolder() {
    const tooling::Holder* h = currentHolder();
    if (loadingHolder_ || !h || !h->iso) {
        return;
    }
    const tooling::TurningHolderCode& old = *h->iso;
    const double f = holderFactor();
    auto letter = [](const QComboBox* combo, char fallback) {
        const QString text = combo->currentData().toString();
        return text.isEmpty() ? fallback : text.at(0).toLatin1();
    };
    tooling::TurningHolderCode c = old;
    c.boringBar = isoType_->currentData().toBool();
    c.barType = letter(isoBarType_, c.barType);
    c.clamping = letter(isoClamping_, c.clamping);
    c.insertShape = letter(isoShape_, c.insertShape);
    c.style = letter(isoStyle_, c.style);
    c.clearance = letter(isoClearance_, c.clearance);
    c.hand = letter(isoHand_, c.hand);
    c.length = letter(isoLength_, c.length);
    c.height = isoHeight_->value() * f;
    c.width = c.boringBar ? c.height : isoWidth_->value() * f;
    c.insertSize = isoSize_->value();
    c.f = isoF_->value() * f;
    // A custom value starts from the one its letter stood for, and goes with the letter.
    c.customStyle = c.style != tooling::kCustomStyle ? std::nullopt
                    : old.style != c.style           ? tooling::holderStyleAngle(old)
                                                     : std::optional(isoStyleAngle_->value());
    c.customClearance = c.clearance != tooling::kCustomClearance ? std::nullopt
                        : old.clearance != c.clearance ? tooling::holderClearanceAngle(old)
                                                       : std::optional(isoClearanceAngle_->value());
    c.customLength = c.length != tooling::kCustomLength ? std::nullopt
                     : old.length != c.length           ? std::optional(tooling::holderLength(old))
                                              : std::optional(isoLengthValue_->value() * f);
    // A value the user has not set follows the size and the hand.
    if (old.f == tooling::standardF(old)) {
        c.f = tooling::standardF(c);
    }
    applyIsoHolder(c);
}

void ToolLibraryDialog::commitIsoDesignation() {
    const tooling::Holder* h = currentHolder();
    if (loadingHolder_ || !h || !h->iso) {
        return;
    }
    if (const QString text = isoDesignation_->text(); text != qs(tooling::designation(*h->iso))) {
        if (const auto c = tooling::parseDesignation(ss(text), &*h->iso)) {
            applyIsoHolder(*c);
        } else {
            holderInfo_->setText(tr("%1 is not an ISO designation, such as PCLNR 2525M 12 or "
                                    "S25T-PCLNR 12.")
                                     .arg(text));
            holderInfo_->setStyleSheet(QStringLiteral("color: #c0392b;"));
        }
    }
}

void ToolLibraryDialog::applyIsoHolder(const tooling::TurningHolderCode& code) {
    tooling::Holder* h = currentHolder();
    if (!h) {
        return;
    }
    // A name that was the designation stays the designation.
    const bool named = h->iso && h->name == tooling::designation(*h->iso);
    tooling::setTurningHolderCode(*h, code);
    if (named) {
        h->name = tooling::designation(code);
    }
    for (tooling::Tool& t : lib_.tools) {
        if (t.holderId == h->id) {
            tooling::fitToHolder(t, *h);
        }
    }
    loadHolderEditor();
    refreshHolderRow();
    refreshHolderCombo();
    updateIssues();
}

tooling::Holder* ToolLibraryDialog::currentHolder() {
    if (currentHolder_ < 0 || currentHolder_ >= static_cast<int>(lib_.holders.size())) {
        return nullptr;
    }
    return &lib_.holders[static_cast<std::size_t>(currentHolder_)];
}

void ToolLibraryDialog::showHolderTab() {
    lists_->setCurrentIndex(1);
}

void ToolLibraryDialog::showHolders(bool on) {
    editors_->setCurrentIndex(on ? 1 : 0);
    presetsBox_->setVisible(!on);
    if (on) {
        rebuildHolderTable(); // tool counts may have changed
        loadHolderEditor();
    } else {
        loadEditor();
    }
}

void ToolLibraryDialog::rebuildHolderTable() {
    const QSignalBlocker block(holderTable_);
    holderTable_->setRowCount(static_cast<int>(lib_.holders.size()));
    for (int r = 0; r < holderTable_->rowCount(); ++r) {
        for (int c = 0; c < HCount; ++c) {
            holderTable_->setItem(r, c, new QTableWidgetItem);
        }
    }
    const int keep = currentHolder_;
    for (int r = 0; r < holderTable_->rowCount(); ++r) {
        currentHolder_ = r;
        refreshHolderRow();
    }
    currentHolder_ = keep;
    refreshHolderCombo();
    updateIssues();
}

void ToolLibraryDialog::refreshHolderRow() {
    const tooling::Holder* h = currentHolder();
    if (!h || currentHolder_ >= holderTable_->rowCount()) {
        return;
    }
    const QSignalBlocker block(holderTable_);
    const LengthUnit unit = preferredUnit();
    const auto users = std::count_if(lib_.tools.begin(), lib_.tools.end(),
                                     [&](const auto& t) { return t.holderId == h->id; });
    holderTable_->item(currentHolder_, HName)->setText(qs(h->name));
    holderTable_->item(currentHolder_, HKind)->setText(holderKindText(h->kind));
    holderTable_->item(currentHolder_, HDiameter)
        ->setText(formatLength(2.0 * h->maxRadius(), unit));
    holderTable_->item(currentHolder_, HLength)->setText(formatLength(h->length(), unit));
    holderTable_->item(currentHolder_, HTools)->setText(QString::number(users));
}

QString ToolLibraryDialog::holderMismatchText(const tooling::Tool& tool,
                                              const tooling::Holder& holder) const {
    QStringList why;
    const tooling::TurningInsert& in = tool.insert;
    for (const tooling::HolderMismatch m : tooling::holderMismatches(tool, holder)) {
        if (!holder.iso) {
            continue;
        }
        const tooling::TurningHolderCode& c = *holder.iso;
        switch (m) {
        case tooling::HolderMismatch::Kind:
            break;
        case tooling::HolderMismatch::Shape:
            why << tr("it takes shape %1, the insert is %2")
                       .arg(QChar::fromLatin1(c.insertShape), qs(in.shape));
            break;
        case tooling::HolderMismatch::Size:
            why << tr("it takes a %1 inscribed circle, the insert has %2")
                       .arg(formatLength(tooling::inscribedCircle(c), tool.unit),
                            formatLength(in.inscribedCircle, tool.unit));
            break;
        case tooling::HolderMismatch::Clearance:
            why << tr("it takes %1 clearance, the insert has %2° relief")
                       .arg(QChar::fromLatin1(c.clearance))
                       .arg(radToDeg(in.reliefAngle), 0, 'g', 4);
            break;
        case tooling::HolderMismatch::Locking:
            why << tr("its clamping %1 does not take this insert's locking style")
                       .arg(QChar::fromLatin1(c.clamping));
            break;
        }
    }
    return why.join(QStringLiteral("; "));
}

void ToolLibraryDialog::refreshHolderCombo() {
    holder_->blockSignals(true);
    holder_->clear();
    holder_->addItem(tr("(none)"));
    const tooling::Tool* t = currentTool();
    // The holders that take the tool, and the one it has even when it does not: an older library's
    // choice, or an insert changed since, stays as it is and says so.
    QStringList hidden;
    QString own;
    for (const auto& h : lib_.holders) {
        const auto misses =
            t ? tooling::holderMismatches(*t, h) : std::vector<tooling::HolderMismatch>{};
        const bool otherKind = !misses.empty() && misses.front() == tooling::HolderMismatch::Kind;
        if (!misses.empty() && t->holderId != h.id) {
            if (!otherKind) {
                hidden << tr("%1: %2").arg(qs(h.name), holderMismatchText(*t, h));
            }
            continue;
        }
        const QString name = misses.empty() ? qs(h.name)
                             : !otherKind   ? tr("%1 (does not fit)").arg(qs(h.name))
                             : h.kind == tooling::HolderKind::Turning
                                 ? tr("%1 (turning holder)").arg(qs(h.name))
                                 : tr("%1 (milling holder)").arg(qs(h.name));
        holder_->addItem(name, QString::fromStdString(h.id.toString()));
        if (otherKind) {
            holder_->setItemData(holder_->count() - 1,
                                 tr("A holder for another kind of tool: listed because this tool "
                                    "uses it"),
                                 Qt::ToolTipRole);
        } else if (!misses.empty()) {
            own = holderMismatchText(*t, h);
            holder_->setItemData(holder_->count() - 1, own, Qt::ToolTipRole);
        }
    }
    holder_->setCurrentIndex(
        t && t->holderId ? std::max(0, holder_->findData(qs(t->holderId->toString()))) : 0);
    holder_->blockSignals(false);
    QStringList note;
    if (!own.isEmpty()) {
        note << tr("This tool keeps its holder, but it does not fit: %1.").arg(own);
    }
    if (!hidden.isEmpty()) {
        note << tr("%n holder(s) hidden, not fitting this insert.", nullptr,
                   static_cast<int>(hidden.size()));
    }
    holderNote_->setText(note.join(QLatin1Char(' ')));
    holderNote_->setToolTip(hidden.join(QLatin1Char('\n')));
    holderNote_->setStyleSheet(own.isEmpty() ? QString() : QStringLiteral("color: #c0392b;"));
    form_->setRowVisible(holderNote_, !note.isEmpty());
}

void ToolLibraryDialog::selectHolder(int row) {
    currentHolder_ = row;
    if (row >= 0 && holderTable_->currentRow() != row) {
        const QSignalBlocker block(holderTable_);
        holderTable_->selectRow(row);
    }
    loadHolderEditor();
}

void ToolLibraryDialog::loadHolderEditor() {
    const tooling::Holder* h = currentHolder();
    holderEditor_->setEnabled(h != nullptr);
    loadingHolder_ = true;
    const QSignalBlocker block(holderPoints_);
    const LengthUnit unit = preferredUnit();
    holderPoints_->setHorizontalHeaderLabels(
        {tr("Radius") + lengthSuffix(unit), tr("Height") + lengthSuffix(unit)});
    if (!h) {
        for (auto* line : {holderName_, holderVendor_, holderPartNumber_, holderComment_}) {
            line->clear();
        }
        holderPoints_->setRowCount(0);
        holderInfo_->setText(lib_.holders.empty()
                                 ? tr("No holders yet: make one or import a STEP model.")
                                 : QString());
        loadingHolder_ = false;
        if (editors_->currentIndex() == 1) {
            preview_->setHolder(tooling::Holder{});
        }
        return;
    }
    holderName_->setText(qs(h->name));
    {
        const QSignalBlocker blockKind(holderKind_);
        holderKind_->setCurrentIndex(holderKind_->findData(static_cast<int>(h->kind)));
    }
    holderVendor_->setText(qs(h->vendor));
    holderPartNumber_->setText(qs(h->partNumber));
    holderComment_->setText(qs(h->comment));
    holderForm_->setRowVisible(holderKind_, !h->iso && !h->stick);
    holderOutline_->setVisible(!h->iso && !h->stick);
    isoBox_->setVisible(h->iso.has_value());
    stickBox_->setVisible(h->stick.has_value());
    if (h->stick) {
        const double unitSize = holderFactor();
        for (QDoubleSpinBox* spin : {stickWidth_, stickF_, stickLength_}) {
            spin->setSuffix(lengthSuffix(unit));
        }
        stickType_->setCurrentIndex(h->stick->bar ? 1 : 0);
        stickWidth_->setValue(h->stick->width / unitSize);
        stickF_->setValue(h->stick->f / unitSize);
        if (auto* label = stickBox_->findChild<QLabel*>(QStringLiteral("stickFLabel"))) {
            label->setText(h->stick->bar ? tr("Tip to bar's axis (f)")
                                         : tr("Tip to back of shank (f)"));
        }
        stickLength_->setValue(h->stick->length / unitSize);
    }
    if (h->iso) {
        loadIsoHolder(*h->iso);
    }
    const auto& v = h->profile.vertices();
    holderPoints_->setRowCount(static_cast<int>(v.size()));
    const double f = holderFactor();
    for (int r = 0; r < holderPoints_->rowCount(); ++r) {
        const Vec2& p = v[static_cast<std::size_t>(r)].p;
        holderPoints_->setItem(r, 0, new QTableWidgetItem(QString::number(p.x() / f, 'g', 7)));
        holderPoints_->setItem(r, 1, new QTableWidgetItem(QString::number(p.y() / f, 'g', 7)));
    }
    loadingHolder_ = false;
    const auto users = std::count_if(lib_.tools.begin(), lib_.tools.end(),
                                     [&](const auto& t) { return t.holderId == h->id; });
    const auto issues = h->validate();
    if (h->iso && issues.empty()) {
        const tooling::TurningHolderCode& c = *h->iso;
        tooling::Tool sample = tooling::makeDefaultTool(ToolType::TurningInsert, 1);
        tooling::fitToHolder(sample, *h);
        QString text = tr("%1 long, for inserts with a %2 inscribed circle; %n tool(s) use it.",
                          nullptr, static_cast<int>(users))
                           .arg(formatLength(tooling::holderLength(c), unit),
                                formatLength(sample.insert.inscribedCircle, unit));
        if (c.boringBar) {
            text += QLatin1Char(' ') + tr("Bores from Ø%1 at the least.")
                                           .arg(formatLength(tooling::smallestBore(c), unit));
        }
        holderInfo_->setText(text);
        holderInfo_->setStyleSheet(QString());
        if (editors_->currentIndex() == 1) {
            preview_->setHolder(*h);
        }
        return;
    }
    if (h->stick && issues.empty()) {
        QString text = tr("%1 long; %n tool(s) use it.", nullptr, static_cast<int>(users))
                           .arg(formatLength(h->stick->length, unit));
        if (h->stick->bar) {
            text += QLatin1Char(' ') +
                    tr("Bores from Ø%1 at the least.")
                        .arg(formatLength(h->stick->f + 0.5 * h->stick->width, unit));
        }
        holderInfo_->setText(text);
        holderInfo_->setStyleSheet(QString());
        if (editors_->currentIndex() == 1) {
            preview_->setHolder(*h);
        }
        return;
    }
    holderInfo_->setText(
        issues.empty()
            ? tr("Ø%1 at the widest, %2 from the tool face to the gauge line; %n tool(s) use it.",
                 nullptr, static_cast<int>(users))
                  .arg(formatLength(2.0 * h->maxRadius(), unit), formatLength(h->length(), unit))
            : qs(issues.front()));
    holderInfo_->setStyleSheet(issues.empty() ? QString() : QStringLiteral("color: #c0392b;"));
    if (editors_->currentIndex() == 1) {
        preview_->setHolder(*h);
    }
}

void ToolLibraryDialog::commitHolderEditor() {
    tooling::Holder* h = currentHolder();
    if (loadingHolder_ || !h) {
        return;
    }
    h->name = ss(holderName_->text());
    h->kind = static_cast<tooling::HolderKind>(holderKind_->currentData().toInt());
    h->vendor = ss(holderVendor_->text());
    h->partNumber = ss(holderPartNumber_->text());
    h->comment = ss(holderComment_->text());
    refreshHolderRow();
    refreshHolderCombo();
    updateIssues();
}

void ToolLibraryDialog::commitHolderPoints() {
    tooling::Holder* h = currentHolder();
    if (loadingHolder_ || !h) {
        return;
    }
    const double f = holderFactor();
    const auto& old = h->profile.vertices();
    std::vector<geom2d::Vertex> v;
    for (int r = 0; r < holderPoints_->rowCount(); ++r) {
        auto number = [&](int c) {
            const QTableWidgetItem* item = holderPoints_->item(r, c);
            bool ok = false;
            const double value = item ? item->text().toDouble(&ok) : 0.0;
            return ok ? value * f : 0.0;
        };
        // Arcs stay on the points that keep their place; edited outlines are straight.
        const double bulge = holderPoints_->rowCount() == static_cast<int>(old.size())
                                 ? old[static_cast<std::size_t>(r)].bulge
                                 : 0.0;
        v.push_back({{number(0), number(1)}, bulge});
    }
    h->profile = geom2d::Polyline(std::move(v), false);
    loadHolderEditor();
    refreshHolderRow();
    updateIssues();
}

void ToolLibraryDialog::addTurningHolder(bool boringBar) {
    lib_.holders.push_back(tooling::makeTurningHolder(
        *tooling::parseDesignation(boringBar ? "S25T-PCLNR 12" : "PCLNR 2525M 12")));
    currentHolder_ = static_cast<int>(lib_.holders.size()) - 1;
    rebuildHolderTable();
    lists_->setCurrentIndex(1);
    selectHolder(currentHolder_);
}

void ToolLibraryDialog::addTurningStick(bool bar) {
    lib_.holders.push_back(tooling::makeTurningStick(
        bar ? tooling::TurningStick{true, 16.0, 11.0, 150.0} : tooling::TurningStick{}));
    currentHolder_ = static_cast<int>(lib_.holders.size()) - 1;
    rebuildHolderTable();
    lists_->setCurrentIndex(1);
    selectHolder(currentHolder_);
}

void ToolLibraryDialog::addHolder() {
    lib_.holders.push_back(tooling::makeHolder(ss(tr("New holder")), {}));
    currentHolder_ = static_cast<int>(lib_.holders.size()) - 1;
    rebuildHolderTable();
    lists_->setCurrentIndex(1);
    selectHolder(currentHolder_);
}

bool ToolLibraryDialog::importHolder(const QString& path) {
    cad::Model model;
    QApplication::setOverrideCursor(Qt::WaitCursor);
    try {
        model = cad::Model::importStep(std::filesystem::path(path.toStdU16String()));
    } catch (const Error& e) {
        QApplication::restoreOverrideCursor();
        QMessageBox::warning(this, tr("Import holder"),
                             tr("Could not import %1:\n%2").arg(path, QString::fromUtf8(e.what())));
        return false;
    }
    QApplication::restoreOverrideCursor();
    HolderImportDialog dialog(std::move(model), QFileInfo(path).completeBaseName(), this);
    if (dialog.exec() != QDialog::Accepted) {
        return false;
    }
    lib_.holders.push_back(dialog.holder());
    currentHolder_ = static_cast<int>(lib_.holders.size()) - 1;
    rebuildHolderTable();
    lists_->setCurrentIndex(1);
    selectHolder(currentHolder_);
    return true;
}

void ToolLibraryDialog::duplicateHolder() {
    const tooling::Holder* h = currentHolder();
    if (!h) {
        return;
    }
    tooling::Holder copy = *h;
    copy.id = Uuid::generate();
    copy.name += " (copy)";
    lib_.holders.push_back(copy);
    currentHolder_ = static_cast<int>(lib_.holders.size()) - 1;
    rebuildHolderTable();
    selectHolder(currentHolder_);
}

void ToolLibraryDialog::deleteHolders() {
    std::set<int, std::greater<>> rows;
    for (const auto& range : holderTable_->selectedRanges()) {
        for (int r = range.topRow(); r <= range.bottomRow(); ++r) {
            rows.insert(r);
        }
    }
    if (rows.empty()) {
        return;
    }
    std::set<Uuid> ids;
    for (int r : rows) {
        ids.insert(lib_.holders[static_cast<std::size_t>(r)].id);
    }
    const auto users = std::count_if(lib_.tools.begin(), lib_.tools.end(), [&](const auto& t) {
        return t.holderId && ids.contains(*t.holderId);
    });
    if (users > 0 &&
        QMessageBox::question(this, tr("Delete holder"),
                              tr("%n tool(s) use it; they will have no holder. Delete anyway?",
                                 nullptr, static_cast<int>(users))) != QMessageBox::Yes) {
        return;
    }
    for (auto& t : lib_.tools) {
        if (t.holderId && ids.contains(*t.holderId)) {
            t.holderId.reset();
        }
    }
    for (int r : rows) {
        lib_.holders.erase(lib_.holders.begin() + r);
    }
    currentHolder_ = std::min(currentHolder_, static_cast<int>(lib_.holders.size()) - 1);
    rebuildHolderTable();
    selectHolder(currentHolder_);
}

void ToolLibraryDialog::holderFromDimensions() {
    tooling::Holder* h = currentHolder();
    if (!h) {
        return;
    }
    QDialog dialog(this);
    dialog.setWindowTitle(tr("Holder dimensions"));
    auto* form = new QFormLayout(&dialog);
    const LengthUnit unit = preferredUnit();
    tooling::HolderDimensions d;
    struct Field {
        QString label;
        double* value;
        QString tip;
    };
    const Field fields[] = {
        {tr("Nose diameter"), &d.noseDiameter, tr("At the tool face")},
        {tr("Nose top diameter"), &d.noseTopDiameter,
         tr("At the top of the nose, for a tapered nose (shrink-fit, collet chucks); 0 = "
            "straight")},
        {tr("Nose length"), &d.noseLength, QString()},
        {tr("Body diameter"), &d.bodyDiameter, tr("0 = no body")},
        {tr("Body length"), &d.bodyLength, QString()},
        {tr("Flange diameter"), &d.flangeDiameter, QString()},
        {tr("Flange length"), &d.flangeLength, tr("Up to the gauge line")},
    };
    std::vector<LengthSpinBox*> spins;
    for (const Field& f : fields) {
        auto* spin = new LengthSpinBox(*f.value, unit, 0.0, 10000.0);
        spin->setToolTip(f.tip);
        form->addRow(f.label, spin);
        spins.push_back(spin);
    }
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    form->addRow(buttons);
    if (dialog.exec() != QDialog::Accepted) {
        return;
    }
    for (std::size_t i = 0; i < spins.size(); ++i) {
        *fields[i].value = spins[i]->mm();
    }
    try {
        h->profile = tooling::makeHolder(h->name, d).profile;
    } catch (const Error& e) {
        QMessageBox::warning(this, tr("Holder dimensions"), QString::fromUtf8(e.what()));
        return;
    }
    loadHolderEditor();
    refreshHolderRow();
    updateIssues();
}

} // namespace ocam::gui
