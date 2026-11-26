// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "WorkholdingLibraryDialog.hpp"

#include "MachinePartsImportDialog.hpp"
#include "OcctView.hpp"
#include "PartTable.hpp"
#include "WorkholdingLibrary.hpp"
#include "WorkholdingWizard.hpp"

#include <ocam/cad/Datum.hpp>
#include <ocam/foundation/Error.hpp>
#include <ocam/foundation/Sha256.hpp>

#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
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
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPainter>
#include <QPushButton>
#include <QScrollArea>
#include <QSettings>
#include <QSlider>
#include <QSpinBox>
#include <QSplitter>
#include <QTableWidget>
#include <QToolButton>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>
#include <set>

namespace ocam::gui {
namespace {

using document::DatumAxis;
using document::DatumKind;
using document::DatumShape;
using document::HoldingKind;
using document::StockHolding;
using document::WorkholdingDatum;
using document::WorkholdingMember;
using document::WorkholdingPart;

constexpr std::uint32_t kHighlight = 0x2EC4FF;

QString qs(const std::string& s) {
    return QString::fromStdString(s);
}
std::string ss(const QString& s) {
    return s.trimmed().toStdString();
}

QDoubleSpinBox* number(double min, double max, int decimals = 3) {
    auto* s = new QDoubleSpinBox;
    s->setRange(min, max);
    s->setDecimals(decimals);
    s->setKeyboardTracking(false);
    s->setMinimumWidth(80);
    return s;
}

QTableWidgetItem* numberItem(double v) {
    return new QTableWidgetItem(QString::number(v, 'g', 10));
}

double cellNumber(const QTableWidget* t, int row, int col) {
    const QTableWidgetItem* item = t->item(row, col);
    return item ? item->text().toDouble() : 0.0;
}

QString vecText(const Vec3& v) {
    auto n = [](double x) {
        const double r = std::round(x * 1000.0) / 1000.0;
        return QString::number(r == 0.0 ? 0.0 : r, 'g', 8);
    };
    return QStringLiteral("%1, %2, %3").arg(n(v.x()), n(v.y()), n(v.z()));
}

std::string comboText(const QTableWidget* t, int row, int col) {
    const auto* c = qobject_cast<const QComboBox*>(t->cellWidget(row, col));
    return c ? c->currentData().toString().toStdString() : std::string();
}

/// The six directions along the template's axes.
const Vec3 kDirections[] = {Vec3::UnitX(),  -Vec3::UnitX(), Vec3::UnitY(),
                            -Vec3::UnitY(), Vec3::UnitZ(),  -Vec3::UnitZ()};
const char* kDirectionNames[] = {"+X", "-X", "+Y", "-Y", "+Z", "-Z"};

enum MemberColumn { kMemberName, kDirX, kDirY, kDirZ, kMin, kMax, kLinked, kRatio };
enum HoldingColumn {
    kHoldingName,
    kHoldingKind,
    kFaceA,
    kFaceB,
    kSeat,
    kXDatum,
    kAxis,
    kGrip,
    kDepth,
    kTenon,
    kGripRange,
    kAnchor
};

/// "X min, Y mid, Z min" labels of the 27 stock points of a seat, index x + 3y + 9z.
QString anchorLabel(int x, int y, int z) {
    const char* names[] = {"min", "middle", "max"};
    return QStringLiteral("X %1, Y %2, Z %3")
        .arg(QLatin1String(names[x]), QLatin1String(names[y]), QLatin1String(names[z]));
}

} // namespace

WorkholdingLibraryDialog::WorkholdingLibraryDialog(QWidget* parent) : QDialog(parent) {
    setWindowTitle(tr("Workholding Library"));
    setWindowFlags(windowFlags() | Qt::WindowMinMaxButtonsHint);
    resize(1560, 900);
    buildUi();
    reloadList();
    const QByteArray geometry =
        QSettings().value(QStringLiteral("workholdingLibraryGeometry")).toByteArray();
    if (!geometry.isEmpty()) {
        restoreGeometry(geometry);
    }
}

void WorkholdingLibraryDialog::done(int result) {
    QSettings().setValue(QStringLiteral("workholdingLibraryGeometry"), saveGeometry());
    QDialog::done(result);
}

void WorkholdingLibraryDialog::buildUi() {
    auto* root = new QVBoxLayout(this);
    auto* splitter = new QSplitter(this);
    root->addWidget(splitter, 1);

    // Left: the templates and library actions.
    auto* left = new QWidget;
    auto* leftLayout = new QVBoxLayout(left);
    leftLayout->setContentsMargins(0, 0, 0, 0);
    list_ = new QListWidget;
    leftLayout->addWidget(list_, 1);
    auto* buttons = new QGridLayout;
    auto* add = new QPushButton(tr("New from STEP..."));
    add->setToolTip(tr("A new workholding from its maker's STEP file: every body becomes a part"));
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
                       .arg(QString::fromStdU16String(WorkholdingLibrary::folder().u16string())));
    folder->setWordWrap(true);
    folder->setTextInteractionFlags(Qt::TextSelectableByMouse);
    folder->setStyleSheet(QStringLiteral("color: gray"));
    leftLayout->addWidget(folder);
    splitter->addWidget(left);

    // Right: the preview over the editor.
    auto* right = new QSplitter(Qt::Vertical);
    auto* previewBox = new QWidget;
    auto* previewLayout = new QVBoxLayout(previewBox);
    previewLayout->setContentsMargins(0, 0, 0, 0);
    preview_ = new OcctView;
    preview_->setMinimumHeight(320);
    previewLayout->addWidget(preview_, 1);
    auto* viewRow = new QHBoxLayout;
    auto* hideSelected = new QPushButton(tr("Hide selected"));
    auto* isolate = new QPushButton(tr("Isolate selected"));
    auto* showAll = new QPushButton(tr("Show all"));
    pickDatum_ = new QPushButton(tr("Pick datum..."));
    pickDatum_->setCheckable(true);
    pickDatum_->setToolTip(tr("Click a face, edge or vertex of a part: a flat face gives its "
                              "centre and normal, a round face or circular edge its axis, a "
                              "straight edge its middle and direction, a vertex its point"));
    pickPoint_ = new QPushButton(tr("Pick point on face..."));
    pickPoint_->setCheckable(true);
    pickPoint_->setToolTip(tr("Click a face: the point clicked, with the face's normal there"));
    for (QPushButton* b : {hideSelected, isolate, showAll}) {
        viewRow->addWidget(b);
    }
    pickNote_ = new QLabel;
    pickNote_->setStyleSheet(QStringLiteral("color: gray"));
    viewRow->addWidget(pickNote_, 1);
    previewLayout->addLayout(viewRow);
    // The jaws moved together by their opening, or closed on a trial stock.
    openingBar_ = new QWidget;
    auto* openingLayout = new QHBoxLayout(openingBar_);
    openingLayout->setContentsMargins(0, 0, 0, 0);
    openingLayout->addWidget(new QLabel(tr("Opening")));
    opening_ = new QSlider(Qt::Horizontal);
    opening_->setToolTip(tr("Moves the jaws together, as a setup closes them (the preview only)"));
    openingLayout->addWidget(opening_, 1);
    openingValue_ = new QLabel;
    openingValue_->setMinimumWidth(80);
    openingLayout->addWidget(openingValue_);
    trial_ = new QCheckBox(tr("Close on a trial stock"));
    trial_->setToolTip(tr("Shows a stock of this size in the jaws, as a setup holds it"));
    openingLayout->addWidget(trial_);
    const char* sizes[] = {QT_TR_NOOP("Width"), QT_TR_NOOP("Length"), QT_TR_NOOP("Height")};
    const char* sizeTips[] = {QT_TR_NOOP("Across the jaws, from one clamping face to the other"),
                              QT_TR_NOOP("Along the jaws"),
                              QT_TR_NOOP("From the stock's bottom to its top")};
    const double sizeValues[] = {60, 100, 40};
    for (int k = 0; k < 3; ++k) {
        trialSize_[k] = number(1, 1e5, 1);
        trialSize_[k]->setValue(sizeValues[k]);
        trialSize_[k]->setSuffix(QStringLiteral(" mm"));
        trialSize_[k]->setToolTip(tr(sizeTips[k]));
        openingLayout->addWidget(new QLabel(tr(sizes[k])));
        openingLayout->addWidget(trialSize_[k]);
    }
    previewLayout->addWidget(openingBar_);
    jogForm_ = new QFormLayout;
    previewLayout->addLayout(jogForm_);
    right->addWidget(previewBox);

    auto* scroll = new QScrollArea;
    scroll->setWidgetResizable(true);
    editor_ = new QWidget;
    auto* editorLayout = new QVBoxLayout(editor_);

    auto* identity = new QGroupBox(tr("Workholding"));
    auto* identityForm = new QFormLayout(identity);
    name_ = new QLineEdit;
    description_ = new QLineEdit;
    up_ = new QComboBox;
    for (int i = 0; i < 6; ++i) {
        up_->addItem(QString::fromLatin1(kDirectionNames[i]), i);
    }
    up_->setToolTip(tr("Up, out of the fixture, in the parts' coordinates"));
    mount_ = new QComboBox;
    mount_->setToolTip(tr("The datum the workholding sits on the machine by (a setup can snap "
                          "it onto the machine): its base's bottom, a zero-point pin"));
    identityForm->addRow(tr("Name"), name_);
    identityForm->addRow(tr("Description"), description_);
    summary_ = new QLabel;
    summary_->setWordWrap(true);
    identityForm->addRow(tr("What it is"), summary_);
    rangeRow_ = new QWidget;
    auto* rangeLayout = new QHBoxLayout(rangeRow_);
    rangeLayout->setContentsMargins(0, 0, 0, 0);
    minOpening_ = number(0, 1e5);
    maxOpening_ = number(0, 1e5);
    for (QDoubleSpinBox* s : {minOpening_, maxOpening_}) {
        s->setSuffix(QStringLiteral(" mm"));
    }
    rangeLayout->addWidget(minOpening_);
    rangeLayout->addWidget(new QLabel(tr("to")));
    rangeLayout->addWidget(maxOpening_);
    rangeLayout->addStretch(1);
    identityForm->addRow(tr("Opening range"), rangeRow_);
    gripDepth_ = number(0, 1e4);
    gripDepth_->setSuffix(QStringLiteral(" mm"));
    gripDepth_->setToolTip(tr("How far the stock reaches below the jaw tops (setups can change "
                              "it)"));
    identityForm->addRow(tr("Grip depth"), gripDepth_);
    auto* wizard = new QPushButton(tr("Set up with the wizard..."));
    wizard->setToolTip(tr("Answer a few questions and click the parts and faces: the jaws, "
                          "their motion and where the stock sits are set up for you"));
    identityForm->addRow(QString(), wizard);
    connect(wizard, &QPushButton::clicked, this, &WorkholdingLibraryDialog::setUpWithWizard);
    editorLayout->addWidget(identity);

    // Everything the wizard sets up, by hand.
    advancedToggle_ = new QToolButton;
    advancedToggle_->setText(tr("Advanced: parts, members, datums and holdings"));
    advancedToggle_->setCheckable(true);
    advancedToggle_->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    advancedToggle_->setArrowType(Qt::RightArrow);
    advancedToggle_->setAutoRaise(true);
    editorLayout->addWidget(advancedToggle_);
    advanced_ = new QWidget;
    auto* advancedLayout = new QVBoxLayout(advanced_);
    advancedLayout->setContentsMargins(0, 0, 0, 0);
    auto* frameBox = new QGroupBox(tr("Orientation and mount"));
    auto* frameForm = new QFormLayout(frameBox);
    frameForm->addRow(tr("Up"), up_);
    frameForm->addRow(tr("Mount datum"), mount_);
    advancedLayout->addWidget(frameBox);

    auto* partsBox = new QGroupBox(tr("Parts"));
    auto* partsLayout = new QVBoxLayout(partsBox);
    auto* partsHelp = new QLabel(
        tr("Each body of the STEP file is a part, where the file puts it. Say what moves each "
           "one: the base (fixed) or a moving member. With several rows selected, a change to "
           "Carried by, Collides or Shown applies to all of them. Parts that do not collide are "
           "left out of the toolpaths' clearance and the collision checks."));
    partsHelp->setWordWrap(true);
    partsLayout->addWidget(partsHelp);
    parts_ = new PartTable(false, tr("Faces"));
    parts_->setMinimumHeight(160);
    partsLayout->addWidget(parts_);
    auto* partButtons = new QHBoxLayout;
    auto* importParts = new QPushButton(tr("Import STEP parts..."));
    auto* merge = new QPushButton(tr("Merge selected"));
    auto* removePart = new QPushButton(tr("Remove selected"));
    for (QPushButton* b : {importParts, merge, removePart}) {
        partButtons->addWidget(b);
    }
    partButtons->addStretch();
    partsLayout->addLayout(partButtons);
    advancedLayout->addWidget(partsBox);

    auto* membersBox = new QGroupBox(tr("Moving members"));
    auto* membersLayout = new QVBoxLayout(membersBox);
    auto* membersHelp = new QLabel(
        tr("A member (a jaw, a clamp) moves its parts and datums along its direction, its travel "
           "measured from where the parts are modelled, within its limits. A linked member "
           "follows another: its travel is the ratio times the other's, along its own direction "
           "(a self-centring vise's second jaw: the opposite direction, ratio 1)."));
    membersHelp->setWordWrap(true);
    membersLayout->addWidget(membersHelp);
    members_ = new QTableWidget(0, 8);
    members_->setHorizontalHeaderLabels({tr("Member"), tr("Dir x"), tr("Dir y"), tr("Dir z"),
                                         tr("Min travel"), tr("Max travel"), tr("Linked to"),
                                         tr("Ratio")});
    members_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    members_->verticalHeader()->hide();
    members_->setMinimumHeight(110);
    membersLayout->addWidget(members_);
    auto* memberButtons = new QHBoxLayout;
    auto* addMember = new QPushButton(tr("Add member"));
    auto* removeMember = new QPushButton(tr("Remove member"));
    auto* directionFromDatum = new QPushButton(tr("Direction from selected datum"));
    for (QPushButton* b : {addMember, removeMember, directionFromDatum}) {
        memberButtons->addWidget(b);
    }
    memberButtons->addStretch();
    membersLayout->addLayout(memberButtons);
    advancedLayout->addWidget(membersBox);

    auto* datumsBox = new QGroupBox(tr("Datums"));
    auto* datumsLayout = new QVBoxLayout(datumsBox);
    auto* datumsHelp = new QLabel(
        tr("Named points with a direction: picked on the parts in the preview (they move with "
           "the part's member), typed, offset from another datum, or midway between two. Jaw "
           "faces, the centre line, a dovetail's flanks and the seat are datums."));
    datumsHelp->setWordWrap(true);
    datumsLayout->addWidget(datumsHelp);
    datumTable_ = new QTableWidget(0, 5);
    datumTable_->setHorizontalHeaderLabels(
        {tr("Datum"), tr("Kind"), tr("On"), tr("Point"), tr("Direction")});
    datumTable_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    datumTable_->verticalHeader()->hide();
    datumTable_->setSelectionBehavior(QAbstractItemView::SelectRows);
    datumTable_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    datumTable_->setMinimumHeight(140);
    datumsLayout->addWidget(datumTable_);
    auto* datumButtons = new QHBoxLayout;
    auto* typed = new QPushButton(tr("Typed"));
    auto* offset = new QPushButton(tr("Offset from selected"));
    auto* midpoint = new QPushButton(tr("Midpoint of two selected"));
    auto* flip = new QPushButton(tr("Flip direction"));
    auto* removeDatum = new QPushButton(tr("Remove"));
    for (QPushButton* b : {pickDatum_, pickPoint_, typed, offset, midpoint, flip, removeDatum}) {
        datumButtons->addWidget(b);
    }
    datumButtons->addStretch();
    datumsLayout->addLayout(datumButtons);
    datumForm_ = new QWidget;
    datumLayout_ = new QFormLayout(datumForm_);
    datumName_ = new QLineEdit;
    datumMember_ = new QComboBox;
    auto* pointRow = new QHBoxLayout;
    auto* dirRow = new QHBoxLayout;
    for (int k = 0; k < 3; ++k) {
        datumPoint_[k] = number(-1e5, 1e5);
        datumDirection_[k] = number(-1, 1, 6);
        datumDirection_[k]->setSingleStep(0.1);
        pointRow->addWidget(datumPoint_[k]);
        dirRow->addWidget(datumDirection_[k]);
    }
    datumFrom_ = new QComboBox;
    datumTo_ = new QComboBox;
    datumAlong_ = new QComboBox;
    datumAlong_->addItem(tr("X"), static_cast<int>(DatumAxis::X));
    datumAlong_->addItem(tr("Y"), static_cast<int>(DatumAxis::Y));
    datumAlong_->addItem(tr("Z"), static_cast<int>(DatumAxis::Z));
    datumAlong_->addItem(tr("Its direction"), static_cast<int>(DatumAxis::Direction));
    datumDistance_ = number(-1e5, 1e5);
    datumLayout_->addRow(tr("Name"), datumName_);
    datumLayout_->addRow(tr("Moves with"), datumMember_);
    datumLayout_->addRow(tr("Point"), pointRow);
    datumLayout_->addRow(tr("Direction"), dirRow);
    datumLayout_->addRow(tr("From"), datumFrom_);
    datumLayout_->addRow(tr("To"), datumTo_);
    datumLayout_->addRow(tr("Along"), datumAlong_);
    datumLayout_->addRow(tr("Distance"), datumDistance_);
    datumsLayout->addWidget(datumForm_);
    advancedLayout->addWidget(datumsBox);

    buildJawsUi(advancedLayout);

    auto* holdingsBox = new QGroupBox(tr("Stock holdings"));
    auto* holdingsLayout = new QVBoxLayout(holdingsBox);
    auto* holdingsHelp = new QLabel(
        tr("Where the stock sits. <b>Jaws</b>: face A and face B clamp the stock's width (its "
           "side on face A, centred when both move), its bottom the grip depth below the jaw tops "
           "(seat). <b>Dovetail</b>: the flanks A and B hold a tenon of the tenon width (0 = the "
           "flanks as modelled) and depth below its shoulder, which sits on the seat. "
           "<b>Seat</b>: the stock's chosen point goes to the seat datum, its Z along the seat's "
           "direction and its X along the X datum's. <b>Chuck</b>: a bar centred on the axis "
           "datum, gripped on its diameter by face A (its distance from the axis the radius "
           "gripped), its end the grip depth behind the seat (the front of the jaws); with no "
           "member moving face A (a collet), bars up to the grip range smaller. In a setup the "
           "members close on the stock and the workholding is placed around it."));
    holdingsHelp->setWordWrap(true);
    holdingsLayout->addWidget(holdingsHelp);
    holdings_ = new QTableWidget(0, 12);
    holdings_->setHorizontalHeaderLabels({tr("Holding"), tr("Kind"), tr("Face A"), tr("Face B"),
                                          tr("Seat / tops"), tr("X datum"), tr("Axis"),
                                          tr("Grip depth"), tr("Tenon depth"), tr("Tenon width"),
                                          tr("Grip range"), tr("Stock point")});
    holdings_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    holdings_->verticalHeader()->hide();
    holdings_->setMinimumHeight(110);
    holdingsLayout->addWidget(holdings_);
    auto* holdingButtons = new QHBoxLayout;
    auto* addJaws = new QPushButton(tr("Add jaws"));
    auto* addDovetail = new QPushButton(tr("Add dovetail"));
    auto* addSeat = new QPushButton(tr("Add seat"));
    auto* addChuck = new QPushButton(tr("Add chuck"));
    auto* removeHolding = new QPushButton(tr("Remove holding"));
    for (QPushButton* b : {addJaws, addDovetail, addSeat, addChuck, removeHolding}) {
        holdingButtons->addWidget(b);
    }
    holdingButtons->addStretch();
    holdingsLayout->addLayout(holdingButtons);
    advancedLayout->addWidget(holdingsBox);

    editorLayout->addWidget(advanced_);
    issues_ = new QLabel;
    issues_->setWordWrap(true);
    issues_->setStyleSheet(QStringLiteral("color: #d04040"));
    editorLayout->addWidget(issues_);
    editorLayout->addStretch();
    scroll->setWidget(editor_);
    right->addWidget(scroll);
    right->setStretchFactor(0, 2);
    right->setStretchFactor(1, 3);
    splitter->addWidget(right);
    splitter->setStretchFactor(1, 4);
    splitter->setSizes({320, 1240});

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

    // Library.
    connect(list_, &QListWidget::currentRowChanged, this,
            &WorkholdingLibraryDialog::onCurrentRowChanged);
    connect(add, &QPushButton::clicked, this, &WorkholdingLibraryDialog::newTemplate);
    connect(duplicate, &QPushButton::clicked, this, &WorkholdingLibraryDialog::duplicateCurrent);
    connect(delete_, &QPushButton::clicked, this, &WorkholdingLibraryDialog::deleteCurrent);
    connect(importButton, &QPushButton::clicked, this, &WorkholdingLibraryDialog::importTemplate);
    connect(exportButton, &QPushButton::clicked, this, &WorkholdingLibraryDialog::exportCurrent);
    connect(save_, &QPushButton::clicked, this, [this] { saveCurrent(); });
    connect(close, &QPushButton::clicked, this, &WorkholdingLibraryDialog::reject);
    connect(use_, &QPushButton::clicked, this, [this] {
        if ((dirty_ || (currentItem() && currentItem()->file.empty())) && !saveCurrent()) {
            return;
        }
        if (const Item* item = currentItem(); item && useHandler_) {
            useHandler_(item->package);
            accept();
        }
    });
    for (QLineEdit* e : {name_, description_}) {
        connect(e, &QLineEdit::textEdited, this, &WorkholdingLibraryDialog::markDirty);
    }
    connect(up_, &QComboBox::currentIndexChanged, this, &WorkholdingLibraryDialog::markDirty);
    connect(mount_, &QComboBox::currentIndexChanged, this, &WorkholdingLibraryDialog::markDirty);

    // Preview.
    connect(preview_, &OcctView::selectionChanged, this, [this] { previewPicked(); });
    connect(pickDatum_, &QPushButton::clicked, this,
            [this](bool on) { startPick(on ? Pick::Datum : Pick::None); });
    connect(pickPoint_, &QPushButton::clicked, this,
            [this](bool on) { startPick(on ? Pick::PointOnFace : Pick::None); });
    connect(hideSelected, &QPushButton::clicked, this, [this] {
        for (const int r : parts_->selectedRows()) {
            viewShown_[static_cast<std::size_t>(r)] = 0;
        }
        updatePreview();
    });
    connect(isolate, &QPushButton::clicked, this, [this] {
        const std::vector<int> rows = parts_->selectedRows();
        if (rows.empty()) {
            return;
        }
        std::fill(viewShown_.begin(), viewShown_.end(), 0);
        for (const int r : rows) {
            viewShown_[static_cast<std::size_t>(r)] = 1;
        }
        updatePreview();
        preview_->fitAll();
    });
    connect(showAll, &QPushButton::clicked, this, [this] {
        std::fill(viewShown_.begin(), viewShown_.end(), 1);
        updatePreview();
    });

    // Parts.
    connect(parts_, &PartTable::edited, this, [this] {
        markDirty();
        updatePreview();
    });
    connect(parts_, &QTableWidget::itemSelectionChanged, this,
            &WorkholdingLibraryDialog::updatePreview);
    connect(importParts, &QPushButton::clicked, this, &WorkholdingLibraryDialog::importParts);
    connect(merge, &QPushButton::clicked, this, &WorkholdingLibraryDialog::mergeSelectedParts);
    connect(removePart, &QPushButton::clicked, this,
            &WorkholdingLibraryDialog::removeSelectedParts);

    // Members.
    connect(members_, &QTableWidget::itemChanged, this, &WorkholdingLibraryDialog::membersEdited);
    connect(addMember, &QPushButton::clicked, this, [this] {
        std::set<std::string> names(memberNames_.begin(), memberNames_.end());
        std::string name = tr("Moving jaw").toStdString();
        for (int n = 2; names.contains(name); ++n) {
            name = tr("Member %1").arg(n).toStdString();
        }
        addMemberRow({name, Vec3::UnitY(), 0.0, 100.0, {}, 1.0});
        membersEdited();
    });
    connect(removeMember, &QPushButton::clicked, this, [this] {
        if (const int r = members_->currentRow(); r >= 0) {
            members_->removeRow(r);
            membersEdited();
        }
    });
    connect(directionFromDatum, &QPushButton::clicked, this, [this] {
        const int m = members_->currentRow();
        const int d = datumTable_->currentRow();
        if (m < 0 || d < 0 || d >= static_cast<int>(datums_.size())) {
            QMessageBox::information(this, tr("Member direction"),
                                     tr("Select a member and a datum with a direction."));
            return;
        }
        try {
            const auto value = document::datumAt(editedPackage().workholding,
                                                 datums_[static_cast<std::size_t>(d)].name, {});
            if (value.direction.isZero()) {
                throw Error(ErrorCode::InvalidArgument, "the datum has no direction");
            }
            const QSignalBlocker block(members_);
            for (int k = 0; k < 3; ++k) {
                members_->setItem(m, kDirX + k, numberItem(value.direction[k]));
            }
            membersEdited();
        } catch (const Error& e) {
            QMessageBox::information(this, tr("Member direction"), QString::fromUtf8(e.what()));
        }
    });

    // Datums.
    connect(datumTable_, &QTableWidget::itemSelectionChanged, this,
            &WorkholdingLibraryDialog::datumSelected);
    connect(typed, &QPushButton::clicked, this, [this] {
        WorkholdingDatum d;
        d.name = uniqueDatumName("Datum");
        d.kind = DatumKind::Typed;
        addDatum(d);
    });
    connect(offset, &QPushButton::clicked, this, [this] {
        const int r = datumTable_->currentRow();
        if (r < 0 || r >= static_cast<int>(datums_.size())) {
            QMessageBox::information(this, tr("Offset datum"),
                                     tr("Select the datum to start from."));
            return;
        }
        WorkholdingDatum d;
        d.kind = DatumKind::Offset;
        d.from = datums_[static_cast<std::size_t>(r)].name;
        d.name = uniqueDatumName(d.from + " offset");
        d.along = DatumAxis::Direction;
        d.distance = 10.0;
        addDatum(d);
    });
    connect(midpoint, &QPushButton::clicked, this, [this] {
        std::vector<int> rows;
        for (const QModelIndex& i : datumTable_->selectionModel()->selectedRows()) {
            rows.push_back(i.row());
        }
        std::sort(rows.begin(), rows.end());
        if (rows.size() != 2) {
            QMessageBox::information(this, tr("Midpoint datum"), tr("Select two datums."));
            return;
        }
        WorkholdingDatum d;
        d.kind = DatumKind::Midpoint;
        d.from = datums_[static_cast<std::size_t>(rows[0])].name;
        d.to = datums_[static_cast<std::size_t>(rows[1])].name;
        d.name = uniqueDatumName("Centre");
        addDatum(d);
    });
    connect(flip, &QPushButton::clicked, this, [this] {
        const int r = datumTable_->currentRow();
        if (r >= 0 && r < static_cast<int>(datums_.size())) {
            auto& d = datums_[static_cast<std::size_t>(r)];
            if (d.kind == DatumKind::Picked || d.kind == DatumKind::Typed) {
                d.direction = -d.direction;
                refreshDatumTable();
                datumSelected();
                markDirty();
            }
        }
    });
    connect(removeDatum, &QPushButton::clicked, this, [this] {
        const int r = datumTable_->currentRow();
        if (r >= 0 && r < static_cast<int>(datums_.size())) {
            datums_.erase(datums_.begin() + r);
            refreshDatumTable();
            refreshDatumChoices();
            markDirty();
        }
    });
    connect(datumName_, &QLineEdit::editingFinished, this,
            &WorkholdingLibraryDialog::datumFormEdited);
    for (QComboBox* c : {datumMember_, datumFrom_, datumTo_, datumAlong_}) {
        connect(c, &QComboBox::currentIndexChanged, this,
                &WorkholdingLibraryDialog::datumFormEdited);
    }
    for (int k = 0; k < 3; ++k) {
        connect(datumPoint_[k], &QDoubleSpinBox::valueChanged, this,
                &WorkholdingLibraryDialog::datumFormEdited);
        connect(datumDirection_[k], &QDoubleSpinBox::valueChanged, this,
                &WorkholdingLibraryDialog::datumFormEdited);
    }
    connect(datumDistance_, &QDoubleSpinBox::valueChanged, this,
            &WorkholdingLibraryDialog::datumFormEdited);

    // Holdings.
    auto addHolding = [this](HoldingKind kind, const QString& base) {
        StockHolding h;
        h.kind = kind;
        std::set<std::string> names;
        for (const auto& x : editedHoldings()) {
            names.insert(x.name);
        }
        h.name = base.toStdString();
        for (int n = 2; names.contains(h.name); ++n) {
            h.name = QStringLiteral("%1 %2").arg(base).arg(n).toStdString();
        }
        addHoldingRow(h);
        markDirty();
    };
    connect(addJaws, &QPushButton::clicked, this,
            [addHolding] { addHolding(HoldingKind::Jaws, tr("Jaws")); });
    connect(addDovetail, &QPushButton::clicked, this,
            [addHolding] { addHolding(HoldingKind::Dovetail, tr("Dovetail")); });
    connect(addSeat, &QPushButton::clicked, this,
            [addHolding] { addHolding(HoldingKind::Seat, tr("Seat")); });
    connect(addChuck, &QPushButton::clicked, this,
            [addHolding] { addHolding(HoldingKind::Chuck, tr("Chuck")); });
    connect(removeHolding, &QPushButton::clicked, this, [this] {
        if (const int r = holdings_->currentRow(); r >= 0) {
            holdings_->removeRow(r);
            markDirty();
        }
    });
    connect(holdings_, &QTableWidget::itemChanged, this, &WorkholdingLibraryDialog::markDirty);

    // The simple part.
    connect(advancedToggle_, &QToolButton::toggled, this, [this](bool on) { showAdvanced(on); });
    for (QDoubleSpinBox* s : {minOpening_, maxOpening_}) {
        connect(s, &QDoubleSpinBox::valueChanged, this,
                &WorkholdingLibraryDialog::openingRangeEdited);
    }
    connect(gripDepth_, &QDoubleSpinBox::valueChanged, this, [this](double v) {
        if (loading_ || holdings_->rowCount() == 0) {
            return;
        }
        holdings_->setItem(0, kGrip, numberItem(v));
    });
    connect(opening_, &QSlider::valueChanged, this, [this](int v) {
        const document::WorkholdingTemplate w = editedPackage().workholding;
        if (w.holdings.empty() || trial_->isChecked()) {
            return;
        }
        try {
            jogTravel_ = document::travelsForOpening(w, w.holdings.front(), v / 10.0, jogTravel_);
            openingValue_->setText(tr("%1 mm").arg(v / 10.0, 0, 'f', 1));
            for (const auto& [name, slider] : jog_) {
                const QSignalBlocker block(slider);
                slider->setValue(static_cast<int>(std::round(jogTravel_[name] * 10.0)));
            }
        } catch (const Error& e) {
            pickNote_->setText(QString::fromUtf8(e.what()));
        }
        updatePreview();
    });
    connect(trial_, &QCheckBox::toggled, this, [this] {
        opening_->setEnabled(!trial_->isChecked());
        updatePreview();
    });
    for (QDoubleSpinBox* s : trialSize_) {
        connect(s, &QDoubleSpinBox::valueChanged, this, &WorkholdingLibraryDialog::updatePreview);
    }
    showAdvanced(QSettings().value(QStringLiteral("workholdingAdvanced"), false).toBool());
}

void WorkholdingLibraryDialog::setUseHandler(
    const QString& setupName, std::function<void(const document::WorkholdingPackage&)> handler) {
    useHandler_ = std::move(handler);
    use_->setText(tr("Use for %1").arg(setupName));
    use_->setVisible(static_cast<bool>(useHandler_));
}

// -- Library ----------------------------------------------------------------------------------

void WorkholdingLibraryDialog::reloadList(const std::optional<Uuid>& selectId) {
    items_.clear();
    std::vector<std::string> problems;
    for (auto& entry : WorkholdingLibrary::load(&problems)) {
        items_.push_back({std::move(entry.package), entry.file});
    }
    {
        const QSignalBlocker block(list_);
        list_->clear();
        for (const Item& item : items_) {
            auto* row = new QListWidgetItem(qs(item.package.workholding.name));
            row->setToolTip(qs(item.package.workholding.description));
            list_->addItem(row);
        }
    }
    if (!problems.empty()) {
        QStringList text;
        for (const auto& p : problems) {
            text << qs(p);
        }
        QMessageBox::warning(this, tr("Workholding library"),
                             tr("Some workholding files could not be read:\n\n%1")
                                 .arg(text.join(QLatin1Char('\n'))));
    }
    int row = items_.empty() ? -1 : 0;
    for (std::size_t i = 0; i < items_.size(); ++i) {
        if (selectId && items_[i].package.workholding.id == *selectId) {
            row = static_cast<int>(i);
        }
    }
    currentRow_ = -1;
    dirty_ = false;
    list_->setCurrentRow(row);
    if (row >= 0 && currentRow_ != row) {
        onCurrentRowChanged(row);
    }
    editor_->setEnabled(row >= 0);
    updateIssues();
}

void WorkholdingLibraryDialog::selectTemplate(const Uuid& id) {
    for (std::size_t i = 0; i < items_.size(); ++i) {
        if (items_[i].package.workholding.id == id) {
            list_->setCurrentRow(static_cast<int>(i));
            return;
        }
    }
}

WorkholdingLibraryDialog::Item* WorkholdingLibraryDialog::currentItem() {
    return currentRow_ >= 0 && currentRow_ < static_cast<int>(items_.size())
               ? &items_[static_cast<std::size_t>(currentRow_)]
               : nullptr;
}

const WorkholdingLibraryDialog::Item* WorkholdingLibraryDialog::currentItem() const {
    return currentRow_ >= 0 && currentRow_ < static_cast<int>(items_.size())
               ? &items_[static_cast<std::size_t>(currentRow_)]
               : nullptr;
}

void WorkholdingLibraryDialog::onCurrentRowChanged(int row) {
    if (row == currentRow_) {
        return;
    }
    if (dirty_ && !confirmDiscard()) {
        const QSignalBlocker block(list_);
        list_->setCurrentRow(currentRow_);
        return;
    }
    // An unsaved new template that is left is dropped.
    if (Item* old = currentItem(); old && old->file.empty()) {
        const int dropped = currentRow_;
        items_.erase(items_.begin() + dropped);
        currentRow_ = -1;
        const QSignalBlocker block(list_);
        delete list_->takeItem(dropped);
        row = row > dropped ? row - 1 : row;
        list_->setCurrentRow(row);
    }
    currentRow_ = row;
    if (Item* item = currentItem()) {
        loadEditor(*item);
    }
}

void WorkholdingLibraryDialog::loadEditor(const Item& item) {
    loading_ = true;
    startPick(Pick::None);
    const document::WorkholdingTemplate& w = item.package.workholding;
    name_->setText(qs(w.name));
    description_->setText(qs(w.description));
    int up = 4;
    for (int i = 0; i < 6; ++i) {
        if (w.up.normalized().isApprox(kDirections[i], 1e-9)) {
            up = i;
        }
    }
    up_->setCurrentIndex(up);
    files_ = item.package.files;
    models_.clear();
    {
        const QSignalBlocker block(members_);
        members_->setRowCount(0);
    }
    memberNames_.clear();
    for (const auto& m : w.members) {
        addMemberRow(m);
    }
    for (int r = 0; r < members_->rowCount(); ++r) {
        memberNames_.push_back(members_->item(r, kMemberName)->text().toStdString());
    }
    std::vector<std::pair<QString, QString>> carriers;
    for (const auto& m : w.members) {
        carriers.emplace_back(qs(m.name), qs(m.name));
    }
    parts_->setRowCount(0);
    parts_->setCarriers(carriers);
    partData_.clear();
    for (const auto& p : w.parts) {
        addPartRow(p);
    }
    viewShown_.assign(partData_.size(), -1);
    datums_ = w.datums;
    {
        const QSignalBlocker block(holdings_);
        holdings_->setRowCount(0);
    }
    for (const auto& h : w.holdings) {
        addHoldingRow(h);
    }
    refreshDatumTable();
    refreshDatumChoices();
    mount_->setCurrentIndex(std::max(0, mount_->findData(qs(w.mount))));
    loadJaws(w);
    jogTravel_.clear();
    rebuildJog();
    editor_->setEnabled(true);
    delete_->setEnabled(true);
    loading_ = false;
    dirty_ = item.file.empty();
    previewFitted_ = false;
    refreshSimple();
    updatePreview();
    updateIssues();
}

document::WorkholdingPackage WorkholdingLibraryDialog::editedPackage() const {
    document::WorkholdingPackage pkg;
    document::WorkholdingTemplate& w = pkg.workholding;
    if (const Item* item = currentItem()) {
        w.id = item->package.workholding.id;
    }
    w.name = ss(name_->text());
    w.description = ss(description_->text());
    w.parts = editedParts();
    w.members = editedMembers();
    w.datums = datums_;
    w.holdings = editedHoldings();
    w.mount = mount_->currentData().toString().toStdString();
    w.up = kDirections[std::clamp(up_->currentIndex(), 0, 5)];
    editedJaws(w);
    for (const auto& p : w.parts) {
        if (const auto it = files_.find(p.sha256); it != files_.end()) {
            pkg.files.emplace(p.sha256, it->second);
        }
    }
    return pkg;
}

void WorkholdingLibraryDialog::markDirty() {
    if (loading_ || !currentItem()) {
        return;
    }
    refreshTopJaws();
    dirty_ = true;
    updateIssues();
    refreshSimple();
}

void WorkholdingLibraryDialog::updateIssues() {
    if (!currentItem()) {
        issues_->clear();
        save_->setEnabled(false);
        use_->setEnabled(false);
        delete_->setEnabled(false);
        return;
    }
    const document::WorkholdingPackage pkg = editedPackage();
    QStringList issues;
    for (const auto& issue : pkg.workholding.validate()) {
        issues << qs(issue);
    }
    issues_->setText(issues.join(QLatin1Char('\n')));
    save_->setEnabled(dirty_ && issues.isEmpty());
    save_->setText(dirty_ ? tr("Save to library *") : tr("Save to library"));
    use_->setEnabled(issues.isEmpty());
}

QString WorkholdingLibraryDialog::issuesText() const {
    return issues_->text();
}

bool WorkholdingLibraryDialog::saveCurrent() {
    if (!currentItem()) {
        return false;
    }
    const document::WorkholdingPackage pkg = editedPackage();
    try {
        WorkholdingLibrary::save(pkg);
    } catch (const Error& e) {
        QMessageBox::warning(this, tr("Workholding library"), QString::fromUtf8(e.what()));
        return false;
    }
    dirty_ = false;
    reloadList(pkg.workholding.id);
    return true;
}

bool WorkholdingLibraryDialog::confirmDiscard() {
    const auto answer = QMessageBox::question(
        this, tr("Workholding library"), tr("Save the changes to this workholding?"),
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

void WorkholdingLibraryDialog::reject() {
    if (dirty_ && !confirmDiscard()) {
        return;
    }
    QDialog::reject();
}

void WorkholdingLibraryDialog::newTemplate() {
    if (dirty_ && !confirmDiscard()) {
        return;
    }
    QSettings settings;
    const QString folder = settings.value(QStringLiteral("workholdingPartsFolder")).toString();
    const QStringList files =
        QFileDialog::getOpenFileNames(this, tr("New workholding from STEP"), folder,
                                      tr("STEP files (*.step *.stp *.STEP *.STP)"));
    if (files.isEmpty()) {
        return;
    }
    settings.setValue(QStringLiteral("workholdingPartsFolder"),
                      QFileInfo(files.front()).absolutePath());
    std::vector<document::SolidPart> parts = MachinePartsImportDialog::runSolids(files, this);
    if (parts.empty()) {
        return;
    }
    // Every body a part, then the wizard asks what it is and which parts and faces do what.
    auto [start, models] = packageFromParts(
        QFileInfo(files.front()).completeBaseName().toStdString(), std::move(parts));
    document::WorkholdingPackage result;
    if (runWizard(start, models, result)) {
        newFromPackage(std::move(result));
    } else if (QMessageBox::question(this, tr("New workholding"),
                                     tr("Keep the parts to set them up by hand?")) ==
               QMessageBox::Yes) {
        newFromPackage(std::move(start));
        showAdvanced(true);
    }
}

void WorkholdingLibraryDialog::newFromParts(const std::string& name,
                                            std::vector<document::SolidPart> parts) {
    Item item;
    item.package.workholding.name = name;
    items_.push_back(std::move(item));
    {
        const QSignalBlocker block(list_);
        list_->addItem(qs(name));
    }
    dirty_ = false;
    list_->setCurrentRow(static_cast<int>(items_.size()) - 1);
    if (currentRow_ != static_cast<int>(items_.size()) - 1) {
        onCurrentRowChanged(static_cast<int>(items_.size()) - 1);
    }
    addParts(std::move(parts));
}

std::pair<document::WorkholdingPackage, std::map<std::string, cad::Model>>
WorkholdingLibraryDialog::packageFromParts(const std::string& name,
                                           std::vector<document::SolidPart> parts) const {
    document::WorkholdingPackage pkg;
    pkg.workholding.name = name;
    std::map<std::string, cad::Model> models;
    static constexpr std::uint32_t kPalette[] = {0x6B809E, 0xA08C6E, 0x6E9E7C,
                                                 0x9E6E8E, 0x8E9E6E, 0x6E9A9E};
    CancelToken cancel;
    NullProgress progress;
    for (auto& solid : parts) {
        try {
            std::string bytes = solid.model.exportStepData();
            WorkholdingPart part;
            part.name = solid.name;
            part.sha256 = sha256Hex(bytes);
            part.colour = solid.colour ? *solid.colour : kPalette[pkg.workholding.parts.size() % 6];
            // Read back from its file: the sub-shape numbers datums refer to.
            models.emplace(part.sha256,
                           cad::Model::importStepData(bytes, solid.name, cancel, progress));
            pkg.files.emplace(part.sha256, std::move(bytes));
            pkg.workholding.parts.push_back(part);
        } catch (const Error&) {
            // a body that does not write: left out (rare: the import read it)
        }
    }
    return {std::move(pkg), std::move(models)};
}

bool WorkholdingLibraryDialog::runWizard(const document::WorkholdingPackage& start,
                                         const std::map<std::string, cad::Model>& models,
                                         document::WorkholdingPackage& result) {
    WorkholdingWizard wizard(start, models, this);
    if (wizard.exec() != QDialog::Accepted) {
        return false;
    }
    result = wizard.result();
    return true;
}

void WorkholdingLibraryDialog::setUpWithWizard() {
    if (!currentItem()) {
        return;
    }
    document::WorkholdingPackage start = editedPackage();
    if (start.workholding.parts.empty()) {
        QMessageBox::information(this, tr("Set up workholding"),
                                 tr("Import the workholding's STEP parts first."));
        return;
    }
    std::map<std::string, cad::Model> models;
    for (const auto& p : start.workholding.parts) {
        if (const cad::Model* m = modelOf(p.sha256)) {
            models.emplace(p.sha256, *m);
        }
    }
    document::WorkholdingPackage result;
    if (!runWizard(start, models, result)) {
        return;
    }
    // In the editor as the edited template (not saved yet).
    Item edited{std::move(result), {}};
    loadEditor(edited);
    dirty_ = true;
    updateIssues();
}

void WorkholdingLibraryDialog::newFromPackage(document::WorkholdingPackage package) {
    const QString name = qs(package.workholding.name);
    items_.push_back({std::move(package), {}});
    {
        const QSignalBlocker block(list_);
        list_->addItem(name);
    }
    dirty_ = false;
    list_->setCurrentRow(static_cast<int>(items_.size()) - 1);
    if (currentRow_ != static_cast<int>(items_.size()) - 1) {
        onCurrentRowChanged(static_cast<int>(items_.size()) - 1);
    }
}

void WorkholdingLibraryDialog::showAdvanced(bool on) {
    {
        const QSignalBlocker block(advancedToggle_);
        advancedToggle_->setChecked(on);
    }
    advancedToggle_->setArrowType(on ? Qt::DownArrow : Qt::RightArrow);
    advanced_->setVisible(on);
    // The members' own travel sliders belong to the hand-made setup.
    for (int r = 0; r < jogForm_->rowCount(); ++r) {
        jogForm_->setRowVisible(r, on);
    }
    QSettings().setValue(QStringLiteral("workholdingAdvanced"), on);
}

void WorkholdingLibraryDialog::setTrialStock(const std::optional<Vec3>& size) {
    if (size) {
        for (int k = 0; k < 3; ++k) {
            const QSignalBlocker block(trialSize_[k]);
            trialSize_[k]->setValue((*size)[k]);
        }
    }
    trial_->setChecked(size.has_value());
    updatePreview();
}

void WorkholdingLibraryDialog::refreshSimple() {
    const document::WorkholdingTemplate w = editedPackage().workholding;
    const bool wasLoading = loading_;
    loading_ = true;
    QString summary;
    std::optional<std::pair<double, double>> range;
    const document::StockHolding* h = w.holdings.empty() ? nullptr : &w.holdings.front();
    if (!h) {
        summary = w.parts.empty() ? tr("No parts yet: start with New from STEP.")
                                  : tr("Not set up yet: run the wizard, or set it up by hand "
                                       "under Advanced.");
    } else {
        try {
            range = h->kind == HoldingKind::Seat ? std::nullopt : document::openingRange(w, *h);
        } catch (const Error&) {
            range.reset();
        }
        const int linked = static_cast<int>(std::count_if(
            w.members.begin(), w.members.end(), [](const auto& m) { return !m.linkedTo.empty(); }));
        switch (h->kind) {
        case HoldingKind::Jaws:
            summary = linked > 0 ? tr("A self-centring vise: the jaws move together.")
                      : range    ? tr("A vise: one jaw moves.")
                                 : tr("Jaws that do not move.");
            break;
        case HoldingKind::Dovetail:
            summary = range ? tr("A dovetail fixture: one flank moves to clamp the tenon.")
                            : tr("A dovetail fixture with fixed flanks.");
            break;
        case HoldingKind::Seat:
            summary = tr("A seat: the stock sits on '%1'.").arg(qs(h->seat));
            break;
        case HoldingKind::Chuck:
            summary =
                range
                    ? tr("A chuck: its %1 jaws grip the bar together.").arg(std::max(1, linked + 1))
                    : tr("A collet: it grips bars up to %1 mm smaller than its bore.")
                          .arg(h->gripRange, 0, 'f', 3);
            break;
        }
        if (w.holdings.size() > 1) {
            summary += tr(" %1 stock holdings (the setup chooses one).").arg(w.holdings.size());
        }
    }
    summary_->setText(summary);
    rangeRow_->setEnabled(range.has_value());
    if (range) {
        minOpening_->setValue(range->first);
        maxOpening_->setValue(range->second);
    }
    gripDepth_->setEnabled(h && (h->kind == HoldingKind::Jaws || h->kind == HoldingKind::Chuck));
    if (h) {
        gripDepth_->setValue(h->gripDepth);
    }
    openingBar_->setVisible(h != nullptr);
    opening_->setEnabled(range.has_value() && !trial_->isChecked());
    if (range && h) {
        const QSignalBlocker block(opening_);
        opening_->setRange(static_cast<int>(std::floor(range->first * 10.0)),
                           static_cast<int>(std::ceil(range->second * 10.0)));
        try {
            const double now =
                document::holdingOpening(w, *h, document::memberTravels(w, jogTravel_));
            opening_->setValue(static_cast<int>(std::round(now * 10.0)));
            openingValue_->setText(tr("%1 mm").arg(now, 0, 'f', 1));
        } catch (const Error&) {
            openingValue_->clear();
        }
    } else {
        openingValue_->clear();
    }
    loading_ = wasLoading;
}

void WorkholdingLibraryDialog::openingRangeEdited() {
    if (loading_) {
        return;
    }
    document::WorkholdingTemplate w = editedPackage().workholding;
    if (w.holdings.empty() || !(maxOpening_->value() > minOpening_->value())) {
        return;
    }
    try {
        document::setOpeningRange(w, w.holdings.front(), minOpening_->value(),
                                  maxOpening_->value());
    } catch (const Error& e) {
        pickNote_->setText(QString::fromUtf8(e.what()));
        return;
    }
    // The members' ranges follow (their table is what is saved).
    {
        const QSignalBlocker block(members_);
        for (int r = 0; r < members_->rowCount(); ++r) {
            const std::string name =
                members_->item(r, kMemberName) ? ss(members_->item(r, kMemberName)->text()) : "";
            if (const WorkholdingMember* m = w.findMember(name)) {
                members_->setItem(r, kMin, numberItem(m->minTravel));
                members_->setItem(r, kMax, numberItem(m->maxTravel));
            }
        }
    }
    rebuildJog();
    markDirty();
    refreshSimple();
}

void WorkholdingLibraryDialog::duplicateCurrent() {
    if (!currentItem() || (dirty_ && !confirmDiscard())) {
        return;
    }
    document::WorkholdingPackage copy = currentItem()->package;
    copy.workholding.id = Uuid::generate();
    copy.workholding.name += " (copy)";
    try {
        WorkholdingLibrary::save(copy);
    } catch (const Error& e) {
        QMessageBox::warning(this, tr("Workholding library"), QString::fromUtf8(e.what()));
        return;
    }
    reloadList(copy.workholding.id);
}

void WorkholdingLibraryDialog::deleteCurrent() {
    Item* item = currentItem();
    if (!item) {
        return;
    }
    if (QMessageBox::question(this, tr("Delete workholding"),
                              tr("Delete '%1' from the library? Projects keep their own copies.")
                                  .arg(qs(item->package.workholding.name))) != QMessageBox::Yes) {
        return;
    }
    if (!item->file.empty()) {
        WorkholdingLibrary::remove(item->package.workholding.id);
    }
    dirty_ = false;
    reloadList();
}

void WorkholdingLibraryDialog::importTemplate() {
    if (dirty_ && !confirmDiscard()) {
        return;
    }
    const QString path = QFileDialog::getOpenFileName(this, tr("Import workholding"), QString(),
                                                      tr("Workholding templates (*.ocamhold)"));
    if (path.isEmpty()) {
        return;
    }
    try {
        document::WorkholdingPackage pkg =
            document::loadWorkholdingFile(std::filesystem::path(path.toStdU16String()));
        if (std::any_of(items_.begin(), items_.end(), [&](const Item& i) {
                return i.package.workholding.id == pkg.workholding.id;
            })) {
            pkg.workholding.id = Uuid::generate(); // keep the library's own one
        }
        WorkholdingLibrary::save(pkg);
        reloadList(pkg.workholding.id);
    } catch (const Error& e) {
        QMessageBox::warning(this, tr("Import workholding"), QString::fromUtf8(e.what()));
    }
}

void WorkholdingLibraryDialog::exportCurrent() {
    if (!currentItem()) {
        return;
    }
    const document::WorkholdingPackage pkg = editedPackage();
    const QString path = QFileDialog::getSaveFileName(
        this, tr("Export workholding"), qs(pkg.workholding.name) + QStringLiteral(".ocamhold"),
        tr("Workholding templates (*.ocamhold)"));
    if (path.isEmpty()) {
        return;
    }
    try {
        document::saveWorkholdingFile(std::filesystem::path(path.toStdU16String()), pkg);
    } catch (const Error& e) {
        QMessageBox::warning(this, tr("Export workholding"), QString::fromUtf8(e.what()));
    }
}

// -- Parts ------------------------------------------------------------------------------------

const cad::Model* WorkholdingLibraryDialog::modelOf(const std::string& sha256) {
    if (const auto it = models_.find(sha256); it != models_.end()) {
        return &it->second;
    }
    const auto file = files_.find(sha256);
    if (file == files_.end()) {
        return nullptr;
    }
    try {
        CancelToken cancel;
        NullProgress progress;
        return &models_
                    .emplace(sha256,
                             cad::Model::importStepData(file->second, "part", cancel, progress))
                    .first->second;
    } catch (const Error&) {
        return nullptr;
    }
}

void WorkholdingLibraryDialog::addPartRow(const WorkholdingPart& part) {
    PartTable::Row row;
    row.name = part.name;
    row.carrier = part.member;
    row.colour = part.colour;
    row.collides = part.collides;
    row.shown = part.shown;
    const cad::Model* model = modelOf(part.sha256);
    row.note = model ? QString::number(model->info().faces) : tr("missing");
    parts_->addRow(row);
    partData_.push_back(part);
    viewShown_.push_back(-1);
}

std::vector<WorkholdingPart> WorkholdingLibraryDialog::editedParts() const {
    std::vector<WorkholdingPart> out;
    for (int r = 0; r < parts_->rowCount() && r < static_cast<int>(partData_.size()); ++r) {
        WorkholdingPart p = partData_[static_cast<std::size_t>(r)];
        const PartTable::Row row = parts_->row(r);
        p.name = row.name;
        p.member = row.carrier;
        p.colour = row.colour;
        p.collides = row.collides;
        p.shown = row.shown;
        out.push_back(std::move(p));
    }
    return out;
}

void WorkholdingLibraryDialog::addParts(std::vector<document::SolidPart> parts) {
    if (!currentItem()) {
        return;
    }
    QApplication::setOverrideCursor(Qt::WaitCursor);
    for (auto& solid : parts) {
        try {
            std::string bytes = solid.model.exportStepData();
            WorkholdingPart part;
            part.name = solid.name;
            part.sha256 = sha256Hex(bytes);
            // The file's colour, else one of a few that tell neighbouring parts apart.
            static constexpr std::uint32_t kPalette[] = {0x6B809E, 0xA08C6E, 0x6E9E7C,
                                                         0x9E6E8E, 0x8E9E6E, 0x6E9A9E};
            part.colour = solid.colour ? *solid.colour : kPalette[partData_.size() % 6];
            files_.emplace(part.sha256, std::move(bytes));
            addPartRow(part);
        } catch (const Error& e) {
            QApplication::restoreOverrideCursor();
            QMessageBox::warning(this, tr("Workholding parts"), QString::fromUtf8(e.what()));
            QApplication::setOverrideCursor(Qt::WaitCursor);
        }
    }
    QApplication::restoreOverrideCursor();
    markDirty();
    previewFitted_ = false;
    updatePreview();
}

void WorkholdingLibraryDialog::importParts() {
    if (!currentItem()) {
        return;
    }
    QSettings settings;
    const QString folder = settings.value(QStringLiteral("workholdingPartsFolder")).toString();
    const QStringList files = QFileDialog::getOpenFileNames(
        this, tr("Import workholding parts"), folder, tr("STEP files (*.step *.stp *.STEP *.STP)"));
    if (files.isEmpty()) {
        return;
    }
    settings.setValue(QStringLiteral("workholdingPartsFolder"),
                      QFileInfo(files.front()).absolutePath());
    addParts(MachinePartsImportDialog::runSolids(files, this));
}

void WorkholdingLibraryDialog::mergeSelectedParts() {
    const std::vector<int> rows = parts_->selectedRows();
    if (rows.size() < 2) {
        QMessageBox::information(this, tr("Merge parts"),
                                 tr("Select two or more parts that move together."));
        return;
    }
    const std::vector<WorkholdingPart> parts = editedParts();
    for (const auto& d : datums_) {
        for (const int r : rows) {
            if (d.part && *d.part == parts[static_cast<std::size_t>(r)].id) {
                QMessageBox::information(
                    this, tr("Merge parts"),
                    tr("The datum '%1' is picked on '%2': remove it first.")
                        .arg(qs(d.name), qs(parts[static_cast<std::size_t>(r)].name)));
                return;
            }
        }
    }
    bool ok = false;
    const QString name = QInputDialog::getText(
        this, tr("Merge parts"), tr("Name of the merged part:"), QLineEdit::Normal,
        qs(parts[static_cast<std::size_t>(rows.front())].name), &ok);
    if (!ok || name.trimmed().isEmpty()) {
        return;
    }
    std::vector<cad::Model> models;
    for (const int r : rows) {
        if (const cad::Model* m = modelOf(parts[static_cast<std::size_t>(r)].sha256)) {
            models.push_back(*m);
        }
    }
    WorkholdingPart merged = parts[static_cast<std::size_t>(rows.front())];
    merged.id = Uuid::generate();
    merged.name = ss(name);
    try {
        std::string bytes = cad::compound(models).exportStepData();
        merged.sha256 = sha256Hex(bytes);
        files_.emplace(merged.sha256, std::move(bytes));
    } catch (const Error& e) {
        QMessageBox::warning(this, tr("Merge parts"), QString::fromUtf8(e.what()));
        return;
    }
    parts_->removeRows(rows);
    for (auto it = rows.rbegin(); it != rows.rend(); ++it) {
        partData_.erase(partData_.begin() + *it);
        viewShown_.erase(viewShown_.begin() + *it);
    }
    addPartRow(merged);
    markDirty();
    updatePreview();
}

void WorkholdingLibraryDialog::removeSelectedParts() {
    const std::vector<int> rows = parts_->selectedRows();
    if (rows.empty()) {
        return;
    }
    const std::vector<WorkholdingPart> parts = editedParts();
    std::set<Uuid> gone;
    for (const int r : rows) {
        gone.insert(parts[static_cast<std::size_t>(r)].id);
    }
    // Datums picked on them go too.
    std::erase_if(datums_,
                  [&](const WorkholdingDatum& d) { return d.part && gone.contains(*d.part); });
    parts_->removeRows(rows);
    for (auto it = rows.rbegin(); it != rows.rend(); ++it) {
        partData_.erase(partData_.begin() + *it);
        viewShown_.erase(viewShown_.begin() + *it);
    }
    refreshDatumTable();
    refreshDatumChoices();
    markDirty();
    updatePreview();
}

// -- Members ----------------------------------------------------------------------------------

void WorkholdingLibraryDialog::addMemberRow(const WorkholdingMember& m) {
    const QSignalBlocker block(members_);
    const int r = members_->rowCount();
    members_->insertRow(r);
    members_->setItem(r, kMemberName, new QTableWidgetItem(qs(m.name)));
    for (int k = 0; k < 3; ++k) {
        members_->setItem(r, kDirX + k, numberItem(m.direction[k]));
    }
    members_->setItem(r, kMin, numberItem(m.minTravel));
    members_->setItem(r, kMax, numberItem(m.maxTravel));
    auto* linked = new QComboBox;
    linked->addItem(tr("(moves on its own)"), QString());
    for (const auto& name : memberNames_) {
        if (name != m.name) {
            linked->addItem(qs(name), qs(name));
        }
    }
    if (!m.linkedTo.empty() && linked->findData(qs(m.linkedTo)) < 0) {
        linked->addItem(qs(m.linkedTo), qs(m.linkedTo));
    }
    linked->setCurrentIndex(std::max(0, linked->findData(qs(m.linkedTo))));
    connect(linked, &QComboBox::currentIndexChanged, this,
            &WorkholdingLibraryDialog::membersEdited);
    members_->setCellWidget(r, kLinked, linked);
    members_->setItem(r, kRatio, numberItem(m.ratio));
}

std::vector<WorkholdingMember> WorkholdingLibraryDialog::editedMembers() const {
    std::vector<WorkholdingMember> out;
    for (int r = 0; r < members_->rowCount(); ++r) {
        WorkholdingMember m;
        if (const QTableWidgetItem* name = members_->item(r, kMemberName)) {
            m.name = ss(name->text());
        }
        m.direction = Vec3(cellNumber(members_, r, kDirX), cellNumber(members_, r, kDirY),
                           cellNumber(members_, r, kDirZ));
        if (m.direction.norm() > 1e-12) {
            m.direction.normalize();
        }
        m.minTravel = cellNumber(members_, r, kMin);
        m.maxTravel = cellNumber(members_, r, kMax);
        m.linkedTo = comboText(members_, r, kLinked);
        m.ratio = cellNumber(members_, r, kRatio);
        out.push_back(std::move(m));
    }
    return out;
}

void WorkholdingLibraryDialog::membersEdited() {
    if (loading_) {
        return;
    }
    // Renamed members: what refers to them follows.
    std::vector<std::string> now;
    for (int r = 0; r < members_->rowCount(); ++r) {
        now.push_back(members_->item(r, kMemberName) ? ss(members_->item(r, kMemberName)->text())
                                                     : std::string());
    }
    std::map<std::string, std::string> renamed;
    if (now.size() == memberNames_.size()) {
        for (std::size_t i = 0; i < now.size(); ++i) {
            if (now[i] != memberNames_[i] && !memberNames_[i].empty()) {
                renamed[memberNames_[i]] = now[i];
            }
        }
    }
    memberNames_ = now;
    std::vector<std::pair<QString, QString>> carriers;
    for (const auto& n : now) {
        carriers.emplace_back(qs(n), qs(n));
    }
    std::vector<WorkholdingPart> parts = editedParts();
    for (auto& d : datums_) {
        if (const auto it = renamed.find(d.member); it != renamed.end()) {
            d.member = it->second;
        }
    }
    parts_->setCarriers(carriers);
    for (int r = 0; r < parts_->rowCount() && r < static_cast<int>(parts.size()); ++r) {
        if (const auto it = renamed.find(parts[static_cast<std::size_t>(r)].member);
            it != renamed.end()) {
            if (auto* combo = qobject_cast<QComboBox*>(parts_->cellWidget(r, 1))) {
                const QSignalBlocker block(combo);
                combo->setCurrentIndex(std::max(0, combo->findData(qs(it->second))));
            }
        }
    }
    // The link choices.
    for (int r = 0; r < members_->rowCount(); ++r) {
        auto* linked = qobject_cast<QComboBox*>(members_->cellWidget(r, kLinked));
        if (!linked) {
            continue;
        }
        std::string current = linked->currentData().toString().toStdString();
        if (const auto it = renamed.find(current); it != renamed.end()) {
            current = it->second;
        }
        const QSignalBlocker block(linked);
        linked->clear();
        linked->addItem(tr("(moves on its own)"), QString());
        for (const auto& n : now) {
            if (n != now[static_cast<std::size_t>(r)]) {
                linked->addItem(qs(n), qs(n));
            }
        }
        linked->setCurrentIndex(std::max(0, linked->findData(qs(current))));
    }
    refreshDatumTable();
    datumSelected();
    rebuildJog();
    markDirty();
    updatePreview();
}

// -- Datums -----------------------------------------------------------------------------------

std::string WorkholdingLibraryDialog::uniqueDatumName(const std::string& base) const {
    auto taken = [&](const std::string& n) {
        return std::any_of(datums_.begin(), datums_.end(),
                           [&](const WorkholdingDatum& d) { return d.name == n; });
    };
    std::string name = base;
    for (int n = 2; taken(name); ++n) {
        name = base + " " + std::to_string(n);
    }
    return name;
}

void WorkholdingLibraryDialog::addDatum(WorkholdingDatum datum) {
    datums_.push_back(std::move(datum));
    refreshDatumTable();
    refreshDatumChoices();
    datumTable_->selectRow(static_cast<int>(datums_.size()) - 1);
    markDirty();
    updatePreview();
}

void WorkholdingLibraryDialog::refreshDatumTable() {
    const document::WorkholdingTemplate w = editedPackage().workholding;
    const int selected = datumTable_->currentRow();
    const QSignalBlocker block(datumTable_);
    datumTable_->setRowCount(0);
    for (const WorkholdingDatum& d : datums_) {
        const int r = datumTable_->rowCount();
        datumTable_->insertRow(r);
        QString kind;
        QString on;
        switch (d.kind) {
        case DatumKind::Picked: {
            const WorkholdingPart* p = d.part ? w.findPart(*d.part) : nullptr;
            const char* what = d.type == cad::SubShapeType::Edge     ? "edge"
                               : d.type == cad::SubShapeType::Vertex ? "vertex"
                                                                     : "face";
            kind =
                tr("picked %1 (%2)").arg(QLatin1String(what), qs(std::string(toString(d.shape))));
            on = p ? qs(p->name) + (p->member.empty() ? QString()
                                                      : QStringLiteral(" (%1)").arg(qs(p->member)))
                   : tr("missing part");
            break;
        }
        case DatumKind::Typed:
            kind = tr("typed");
            on = d.member.empty() ? tr("base") : qs(d.member);
            break;
        case DatumKind::Offset:
            kind = tr("offset");
            on = tr("%1 + %2 mm").arg(qs(d.from)).arg(d.distance);
            break;
        case DatumKind::Midpoint:
            kind = tr("midpoint");
            on = tr("%1 / %2").arg(qs(d.from), qs(d.to));
            break;
        }
        QString point;
        QString direction;
        try {
            const auto v = document::datumAt(w, d.name, {});
            point = vecText(v.point);
            direction = v.direction.isZero() ? tr("none") : vecText(v.direction);
        } catch (const Error& e) {
            point = QString::fromUtf8(e.what());
        }
        const QString cells[] = {qs(d.name), kind, on, point, direction};
        for (int c = 0; c < 5; ++c) {
            datumTable_->setItem(r, c, new QTableWidgetItem(cells[c]));
        }
    }
    if (selected >= 0 && selected < datumTable_->rowCount()) {
        datumTable_->selectRow(selected);
    }
}

void WorkholdingLibraryDialog::refreshDatumChoices() {
    auto refill = [&](QComboBox* combo, const QString& empty) {
        if (!combo) {
            return;
        }
        const QSignalBlocker block(combo);
        const QString current = combo->currentData().toString();
        combo->clear();
        combo->addItem(empty, QString());
        for (const auto& d : datums_) {
            combo->addItem(qs(d.name), qs(d.name));
        }
        combo->setCurrentIndex(std::max(0, combo->findData(current)));
    };
    refill(mount_, tr("(the template origin)"));
    refill(jawSeat_, tr("(choose)"));
    refill(jawFront_, tr("(choose)"));
    for (int r = 0; jawSteps_ && r < jawSteps_->rowCount(); ++r) {
        for (const int c : {1, 2}) {
            refill(qobject_cast<QComboBox*>(jawSteps_->cellWidget(r, c)), tr("(choose)"));
        }
    }
    refill(datumFrom_, tr("(choose)"));
    refill(datumTo_, tr("(choose)"));
    for (int r = 0; r < holdings_->rowCount(); ++r) {
        for (const int c : {kFaceA, kFaceB, kSeat, kXDatum, kAxis}) {
            refill(qobject_cast<QComboBox*>(holdings_->cellWidget(r, c)),
                   c == kXDatum ? tr("(template X)") : tr("(choose)"));
        }
    }
}

void WorkholdingLibraryDialog::datumSelected() {
    const int r = datumTable_->currentRow();
    const bool any = r >= 0 && r < static_cast<int>(datums_.size());
    datumForm_->setEnabled(any);
    if (!any) {
        return;
    }
    const WorkholdingDatum& d = datums_[static_cast<std::size_t>(r)];
    const bool wasLoading = loading_;
    loading_ = true;
    datumName_->setText(qs(d.name));
    {
        const QSignalBlocker block(datumMember_);
        datumMember_->clear();
        datumMember_->addItem(tr("Base"), QString());
        for (const auto& n : memberNames_) {
            datumMember_->addItem(qs(n), qs(n));
        }
        datumMember_->setCurrentIndex(std::max(0, datumMember_->findData(qs(d.member))));
    }
    for (int k = 0; k < 3; ++k) {
        const QSignalBlocker b1(datumPoint_[k]);
        const QSignalBlocker b2(datumDirection_[k]);
        datumPoint_[k]->setValue(d.point[k]);
        datumDirection_[k]->setValue(d.direction[k]);
    }
    {
        const QSignalBlocker b1(datumFrom_);
        const QSignalBlocker b2(datumTo_);
        const QSignalBlocker b3(datumAlong_);
        const QSignalBlocker b4(datumDistance_);
        datumFrom_->setCurrentIndex(std::max(0, datumFrom_->findData(qs(d.from))));
        datumTo_->setCurrentIndex(std::max(0, datumTo_->findData(qs(d.to))));
        datumAlong_->setCurrentIndex(std::max(0, datumAlong_->findData(static_cast<int>(d.along))));
        datumDistance_->setValue(d.distance);
    }
    const bool typed = d.kind == DatumKind::Typed;
    const bool offset = d.kind == DatumKind::Offset;
    const bool midpoint = d.kind == DatumKind::Midpoint;
    // Rows: name, moves with, point, direction, from, to, along, distance.
    const bool rows[] = {true, typed, typed, typed, offset || midpoint, midpoint, offset, offset};
    for (int i = 0; i < 8; ++i) {
        datumLayout_->setRowVisible(i, rows[i]);
    }
    loading_ = wasLoading;
    updatePreview();
}

void WorkholdingLibraryDialog::renameDatum(const std::string& from, const std::string& to) {
    for (auto& d : datums_) {
        if (d.from == from) {
            d.from = to;
        }
        if (d.to == from) {
            d.to = to;
        }
    }
    for (int r = 0; r < holdings_->rowCount(); ++r) {
        for (const int c : {kFaceA, kFaceB, kSeat, kXDatum, kAxis}) {
            if (auto* combo = qobject_cast<QComboBox*>(holdings_->cellWidget(r, c));
                combo && combo->currentData().toString() == qs(from)) {
                const QSignalBlocker block(combo);
                combo->setItemData(combo->currentIndex(), qs(to));
                combo->setItemText(combo->currentIndex(), qs(to));
            }
        }
    }
    if (mount_->currentData().toString() == qs(from)) {
        const QSignalBlocker block(mount_);
        mount_->setItemData(mount_->currentIndex(), qs(to));
    }
}

void WorkholdingLibraryDialog::datumFormEdited() {
    if (loading_) {
        return;
    }
    const int r = datumTable_->currentRow();
    if (r < 0 || r >= static_cast<int>(datums_.size())) {
        return;
    }
    WorkholdingDatum& d = datums_[static_cast<std::size_t>(r)];
    const std::string name = ss(datumName_->text());
    if (!name.empty() && name != d.name) {
        if (std::any_of(datums_.begin(), datums_.end(),
                        [&](const WorkholdingDatum& x) { return x.name == name; })) {
            QMessageBox::information(this, tr("Datum"),
                                     tr("Another datum is called '%1'.").arg(qs(name)));
        } else {
            const std::string old = d.name;
            d.name = name;
            renameDatum(old, name);
        }
    }
    if (d.kind == DatumKind::Typed) {
        d.member = datumMember_->currentData().toString().toStdString();
        for (int k = 0; k < 3; ++k) {
            d.point[k] = datumPoint_[k]->value();
            d.direction[k] = datumDirection_[k]->value();
        }
    }
    if (d.kind == DatumKind::Offset || d.kind == DatumKind::Midpoint) {
        d.from = datumFrom_->currentData().toString().toStdString();
    }
    if (d.kind == DatumKind::Midpoint) {
        d.to = datumTo_->currentData().toString().toStdString();
    }
    if (d.kind == DatumKind::Offset) {
        d.along = static_cast<DatumAxis>(datumAlong_->currentData().toInt());
        d.distance = datumDistance_->value();
    }
    refreshDatumTable();
    refreshDatumChoices();
    markDirty();
    updatePreview();
}

// -- Holdings ---------------------------------------------------------------------------------

void WorkholdingLibraryDialog::addHoldingRow(const StockHolding& h) {
    const QSignalBlocker block(holdings_);
    const int r = holdings_->rowCount();
    holdings_->insertRow(r);
    holdings_->setItem(r, kHoldingName, new QTableWidgetItem(qs(h.name)));
    auto* kind = new QComboBox;
    kind->addItem(tr("Jaws"), static_cast<int>(HoldingKind::Jaws));
    kind->addItem(tr("Dovetail"), static_cast<int>(HoldingKind::Dovetail));
    kind->addItem(tr("Seat"), static_cast<int>(HoldingKind::Seat));
    kind->addItem(tr("Chuck"), static_cast<int>(HoldingKind::Chuck));
    kind->setCurrentIndex(std::max(0, kind->findData(static_cast<int>(h.kind))));
    connect(kind, &QComboBox::currentIndexChanged, this, &WorkholdingLibraryDialog::markDirty);
    holdings_->setCellWidget(r, kHoldingKind, kind);
    for (const auto& [column, value] :
         {std::pair{kFaceA, h.faceA}, std::pair{kFaceB, h.faceB}, std::pair{kSeat, h.seat},
          std::pair{kXDatum, h.xDatum}, std::pair{kAxis, h.axis}}) {
        auto* combo = new QComboBox;
        combo->addItem(column == kXDatum ? tr("(template X)") : tr("(choose)"), QString());
        for (const auto& d : datums_) {
            combo->addItem(qs(d.name), qs(d.name));
        }
        combo->setCurrentIndex(std::max(0, combo->findData(qs(value))));
        connect(combo, &QComboBox::currentIndexChanged, this, &WorkholdingLibraryDialog::markDirty);
        holdings_->setCellWidget(r, column, combo);
    }
    holdings_->setItem(r, kGrip, numberItem(h.gripDepth));
    holdings_->setItem(r, kDepth, numberItem(h.depth));
    holdings_->setItem(r, kTenon, numberItem(h.tenonWidth));
    holdings_->setItem(r, kGripRange, numberItem(h.gripRange));
    auto* anchor = new QComboBox;
    for (int z = 0; z < 3; ++z) {
        for (int y = 0; y < 3; ++y) {
            for (int x = 0; x < 3; ++x) {
                anchor->addItem(anchorLabel(x, y, z), x + 3 * y + 9 * z);
            }
        }
    }
    anchor->setCurrentIndex(
        std::max(0, anchor->findData(h.anchor[0] + 3 * h.anchor[1] + 9 * h.anchor[2])));
    connect(anchor, &QComboBox::currentIndexChanged, this, &WorkholdingLibraryDialog::markDirty);
    holdings_->setCellWidget(r, kAnchor, anchor);
}

std::vector<StockHolding> WorkholdingLibraryDialog::editedHoldings() const {
    std::vector<StockHolding> out;
    for (int r = 0; r < holdings_->rowCount(); ++r) {
        StockHolding h;
        if (const QTableWidgetItem* name = holdings_->item(r, kHoldingName)) {
            h.name = ss(name->text());
        }
        if (const auto* kind =
                qobject_cast<const QComboBox*>(holdings_->cellWidget(r, kHoldingKind))) {
            h.kind = static_cast<HoldingKind>(kind->currentData().toInt());
        }
        h.faceA = comboText(holdings_, r, kFaceA);
        h.faceB = comboText(holdings_, r, kFaceB);
        h.seat = comboText(holdings_, r, kSeat);
        h.xDatum = comboText(holdings_, r, kXDatum);
        h.gripDepth = cellNumber(holdings_, r, kGrip);
        h.depth = cellNumber(holdings_, r, kDepth);
        h.tenonWidth = cellNumber(holdings_, r, kTenon);
        h.axis = comboText(holdings_, r, kAxis);
        h.gripRange = cellNumber(holdings_, r, kGripRange);
        if (const auto* anchor =
                qobject_cast<const QComboBox*>(holdings_->cellWidget(r, kAnchor))) {
            const int i = anchor->currentData().toInt();
            h.anchor = {i % 3, (i / 3) % 3, i / 9};
        }
        out.push_back(std::move(h));
    }
    return out;
}

// -- Preview ----------------------------------------------------------------------------------

void WorkholdingLibraryDialog::startPick(Pick mode) {
    pick_ = mode;
    {
        const QSignalBlocker b1(pickDatum_);
        const QSignalBlocker b2(pickPoint_);
        pickDatum_->setChecked(mode == Pick::Datum);
        pickPoint_->setChecked(mode == Pick::PointOnFace);
    }
    pickNote_->setText(mode == Pick::Datum         ? tr("Click a face, edge or vertex of a part.")
                       : mode == Pick::PointOnFace ? tr("Click a point on a face of a part.")
                                                   : QString());
}

void WorkholdingLibraryDialog::previewPicked() {
    const std::vector<OcctView::PickedShape> shapes = preview_->selectedShapes();
    if (shapes.empty()) {
        return;
    }
    const OcctView::PickedShape s = shapes.front();
    const std::vector<WorkholdingPart> parts = editedParts();
    const auto it = std::find_if(parts.begin(), parts.end(),
                                 [&](const WorkholdingPart& p) { return p.id == s.model; });
    if (it == parts.end()) {
        return;
    }
    const int row = static_cast<int>(it - parts.begin());
    if (pick_ == Pick::None) {
        parts_->selectRow(row);
        return;
    }
    const cad::Model* model = modelOf(it->sha256);
    if (!model) {
        return;
    }
    try {
        std::optional<Vec3> onFace;
        if (pick_ == Pick::PointOnFace) {
            if (s.type != cad::SubShapeType::Face || !preview_->lastPickedPoint()) {
                pickNote_->setText(tr("Click a face."));
                return;
            }
            // Back to where the part is modelled.
            const auto travel = document::memberTravels(editedPackage().workholding, jogTravel_);
            Vec3 moved = Vec3::Zero();
            for (const auto& m : editedMembers()) {
                if (m.name == it->member && travel.contains(m.name)) {
                    moved = m.direction * travel.at(m.name);
                }
            }
            onFace = *preview_->lastPickedPoint() - moved;
        }
        WorkholdingDatum d = document::pickedDatum("", *it, *model, s.type, s.index, onFace);
        const char* base = d.shape == DatumShape::Plane  ? "Face"
                           : d.shape == DatumShape::Axis ? "Axis"
                           : d.shape == DatumShape::Line ? "Edge"
                                                         : "Point";
        d.name = uniqueDatumName(it->name + " " + base);
        startPick(Pick::None);
        addDatum(d);
        pickNote_->setText(tr("Datum '%1' added.").arg(qs(d.name)));
    } catch (const Error& e) {
        pickNote_->setText(QString::fromUtf8(e.what()));
    }
}

std::string WorkholdingLibraryDialog::pickDatum(const std::string& partName, cad::SubShapeType type,
                                                int index) {
    const std::vector<WorkholdingPart> parts = editedParts();
    const auto it = std::find_if(parts.begin(), parts.end(),
                                 [&](const WorkholdingPart& p) { return p.name == partName; });
    if (it == parts.end()) {
        return {};
    }
    const cad::Model* model = modelOf(it->sha256);
    if (!model) {
        return {};
    }
    WorkholdingDatum d = document::pickedDatum("", *it, *model, type, index);
    d.name = uniqueDatumName(it->name + " datum");
    const std::string name = d.name;
    addDatum(d);
    return name;
}

void WorkholdingLibraryDialog::rebuildJog() {
    while (jogForm_->rowCount() > 0) {
        jogForm_->removeRow(0);
    }
    jog_.clear();
    for (const auto& m : editedMembers()) {
        if (!m.linkedTo.empty() || m.name.empty()) {
            continue; // follows another
        }
        auto* slider = new QSlider(Qt::Horizontal);
        slider->setRange(static_cast<int>(std::floor(m.minTravel * 10.0)),
                         static_cast<int>(std::ceil(m.maxTravel * 10.0)));
        slider->setValue(static_cast<int>(
            std::round((jogTravel_.contains(m.name) ? jogTravel_.at(m.name) : 0.0) * 10.0)));
        slider->setToolTip(tr("Moves the member in the preview only"));
        const std::string name = m.name;
        connect(slider, &QSlider::valueChanged, this, [this, name](int v) {
            jogTravel_[name] = v / 10.0;
            updatePreview();
        });
        jogForm_->addRow(tr("%1 travel").arg(qs(m.name)), slider);
        jog_[m.name] = slider;
    }
    // The members' own sliders only with the Advanced section (the Opening slider moves them).
    for (int r = 0; r < jogForm_->rowCount(); ++r) {
        jogForm_->setRowVisible(r, advancedToggle_ && advancedToggle_->isChecked());
    }
}

void WorkholdingLibraryDialog::jog(const std::string& member, double travel) {
    if (const auto it = jog_.find(member); it != jog_.end()) {
        it->second->setValue(static_cast<int>(std::round(travel * 10.0)));
    }
}

void WorkholdingLibraryDialog::updatePreview() {
    if (!preview_ || !currentItem()) {
        return;
    }
    const document::WorkholdingTemplate w = editedPackage().workholding;
    auto travel = document::memberTravels(w, jogTravel_);
    // A trial stock in the jaws: the members close on it, as a setup closes them.
    std::vector<std::pair<Vec3, Vec3>> stockEdges;
    if (trial_->isChecked() && !w.holdings.empty()) {
        document::SetupWorkholding placement;
        placement.templateId = w.id;
        const geom3d::Aabb3 stock{Vec3(0, 0, -trialSize_[2]->value()),
                                  Vec3(trialSize_[0]->value(), trialSize_[1]->value(), 0)};
        try {
            const document::WorkholdingPose pose = document::poseWorkholding(w, placement, stock);
            travel = pose.travel;
            const Transform3 toTemplate = pose.templateToWcs.inverse();
            Vec3 c[8];
            for (int i = 0; i < 8; ++i) {
                c[i] = toTemplate * Vec3((i & 1) ? stock.max.x() : stock.min.x(),
                                         (i & 2) ? stock.max.y() : stock.min.y(),
                                         (i & 4) ? stock.max.z() : stock.min.z());
            }
            for (int i = 0; i < 8; ++i) {
                for (const int bit : {1, 2, 4}) {
                    if (!(i & bit)) {
                        stockEdges.emplace_back(c[i], c[i | bit]);
                    }
                }
            }
            if (w.holdings.front().kind != HoldingKind::Seat) {
                openingValue_->setText(tr("%1 mm").arg(pose.opening, 0, 'f', 1));
            }
            pickNote_->setText(pose.warnings.empty() ? QString() : qs(pose.warnings.front()));
        } catch (const Error& e) {
            pickNote_->setText(QString::fromUtf8(e.what()));
        }
    }
    auto moved = [&](const std::string& member) {
        Transform3 t = Transform3::Identity();
        if (const WorkholdingMember* m = member.empty() ? nullptr : w.findMember(member)) {
            const auto it = travel.find(member);
            t.translation() = m->direction.normalized() * (it == travel.end() ? 0.0 : it->second);
        }
        return t;
    };
    std::set<int> selected;
    for (const int r : parts_->selectedRows()) {
        selected.insert(r);
    }
    std::vector<OcctView::ViewModel> models;
    for (std::size_t i = 0; i < w.parts.size(); ++i) {
        const WorkholdingPart& p = w.parts[i];
        const bool shown =
            i < viewShown_.size() && viewShown_[i] >= 0 ? viewShown_[i] == 1 : p.shown;
        const cad::Model* m = modelOf(p.sha256);
        if (!shown || !m) {
            continue;
        }
        OcctView::ViewModel vm;
        vm.id = p.id;
        vm.model = *m;
        vm.placement = moved(p.member);
        vm.colour = selected.contains(static_cast<int>(i)) ? kHighlight : p.colour;
        models.push_back(std::move(vm));
    }
    preview_->setModels(models);
    // The datums: a cross at each point and a line along its direction, moved with the members.
    OcctView::MachineScene scene;
    const int selectedDatum = datumTable_->currentRow();
    for (std::size_t i = 0; i < w.datums.size(); ++i) {
        try {
            const document::DatumValue v = document::datumAt(w, w.datums[i].name, travel);
            const bool hot = static_cast<int>(i) == selectedDatum;
            const Vec3 axes[] = {Vec3::UnitX(), Vec3::UnitY(), Vec3::UnitZ()};
            for (const Vec3& axis : axes) {
                scene.lines.push_back({v.point - 3.0 * axis, v.point + 3.0 * axis, hot, 0xFF5050u});
            }
            if (!v.direction.isZero()) {
                scene.lines.push_back({v.point, v.point + 25.0 * v.direction, hot, 0xFF5050u});
            }
        } catch (const Error&) {
            // shown in the table
        }
    }
    for (const auto& [from, to] : stockEdges) {
        scene.lines.push_back({from, to, true, 0xFFD040u});
    }
    preview_->setMachine(scene);
    if (!previewFitted_ && !models.empty()) {
        preview_->fitAll();
        previewFitted_ = true;
    }
}

QImage WorkholdingLibraryDialog::snapshot() {
    QImage image = grab().toImage();
    const QString file = QDir::temp().filePath(QStringLiteral("oxcam-workholding-preview.png"));
    if (preview_->dumpImage(file)) {
        const QImage view(file);
        QPainter painter(&image);
        const QPoint at = preview_->mapTo(this, QPoint(0, 0));
        painter.drawImage(QRectF(at.x(), at.y(), preview_->width(), preview_->height()), view);
        QFile::remove(file);
    }
    return image;
}

void WorkholdingLibraryDialog::buildJawsUi(QVBoxLayout* into) {
    auto* box = new QGroupBox(tr("Jaws"));
    box->setObjectName(QStringLiteral("workholdingJaws"));
    auto* layout = new QVBoxLayout(box);
    auto* help = new QLabel(
        tr("A chuck can take jaw sets on its master jaws (members Jaw 1, Jaw 2, ...): say where a "
           "jaw seats on master jaw 1, and which of its parts are its own top jaws (left out when "
           "a setup puts a set on). A jaw set is one jaw, copied onto each master jaw. Its options "
           "are the ways it goes on, as a vise's holdings are its ways of holding: a jaw that goes "
           "on either way round is two options, each with its own seat, way in towards the axis "
           "and steps. Both name their interface (a label); a setup offers the jaw sets whose "
           "interface the chuck takes."));
    help->setWordWrap(true);
    layout->addWidget(help);
    auto* top = new QFormLayout;
    jawRole_ = new QComboBox;
    jawRole_->setObjectName(QStringLiteral("workholdingJawRole"));
    jawRole_->addItem(tr("Neither"));
    jawRole_->addItem(tr("A chuck that takes jaw sets"));
    jawRole_->addItem(tr("A jaw set"));
    top->addRow(tr("This is"), jawRole_);
    jawInterface_ = new QLineEdit;
    jawInterface_->setObjectName(QStringLiteral("workholdingJawInterface"));
    jawInterface_->setPlaceholderText(tr("A name for how jaws go on"));
    jawInterface_->setToolTip(tr("How jaws go on master jaws, by name: compared without regard to "
                                 "case or spacing"));
    top->addRow(tr("Interface"), jawInterface_);
    layout->addLayout(top);

    chuckJawRows_ = new QWidget;
    auto* chuckForm = new QFormLayout(chuckJawRows_);
    chuckForm->setContentsMargins(0, 0, 0, 0);
    jawSeat_ = new QComboBox;
    jawSeat_->setObjectName(QStringLiteral("workholdingJawSeat"));
    jawSeat_->setToolTip(tr("Chuck: a datum on master jaw 1 where a jaw's seat goes, its direction "
                            "out of the chuck. Jaw set: the datum of this option that sits there, "
                            "its direction out of the chuck."));
    chuckForm->addRow(tr("Jaw seat"), jawSeat_);
    topJaws_ = new QListWidget;
    topJaws_->setObjectName(QStringLiteral("workholdingTopJaws"));
    topJaws_->setMaximumHeight(110);
    topJaws_->setToolTip(
        tr("Ticked: the chuck's own top jaws, left out when a setup puts a jaw set "
           "on"));
    chuckForm->addRow(tr("Its own top jaws"), topJaws_);
    layout->addWidget(chuckJawRows_);

    jawSetRows_ = new QWidget;
    auto* setForm = new QFormLayout(jawSetRows_);
    setForm->setContentsMargins(0, 0, 0, 0);
    jawKind_ = new QComboBox;
    jawKind_->setObjectName(QStringLiteral("workholdingJawKind"));
    jawKind_->addItem(tr("Hard: steps to grip with"));
    jawKind_->addItem(tr("Soft: a blank, bored to fit"));
    setForm->addRow(tr("Kind"), jawKind_);
    jawFront_ = new QComboBox;
    jawFront_->setObjectName(QStringLiteral("workholdingJawFront"));
    jawFront_->setToolTip(tr("Its face towards the work: soft jaws are bored back from it"));
    setForm->addRow(tr("Front"), jawFront_);
    auto* optionRow = new QWidget;
    auto* optionLayout = new QHBoxLayout(optionRow);
    optionLayout->setContentsMargins(0, 0, 0, 0);
    jawOption_ = new QComboBox;
    jawOption_->setObjectName(QStringLiteral("workholdingJawOption"));
    jawOption_->setToolTip(tr("The ways the jaw goes on a master jaw, such as OD and reversed for "
                              "ID: the fields below are this one's"));
    auto* addOption = new QPushButton(tr("Add option"));
    addOption->setObjectName(QStringLiteral("workholdingJawAddOption"));
    addOption->setToolTip(tr("Another way on, starting as a copy of this one: give it its seat, "
                             "way in and steps"));
    auto* removeOption = new QPushButton(tr("Remove"));
    removeOption->setObjectName(QStringLiteral("workholdingJawRemoveOption"));
    optionLayout->addWidget(jawOption_, 1);
    optionLayout->addWidget(addOption);
    optionLayout->addWidget(removeOption);
    setForm->addRow(tr("Option"), optionRow);
    jawOptionName_ = new QLineEdit;
    jawOptionName_->setObjectName(QStringLiteral("workholdingJawOptionName"));
    jawOptionName_->setPlaceholderText(tr("OD, Reversed (ID), ..."));
    setForm->addRow(tr("Option name"), jawOptionName_);
    auto* inward = new QWidget;
    auto* inwardLayout = new QHBoxLayout(inward);
    inwardLayout->setContentsMargins(0, 0, 0, 0);
    for (auto*& s : jawInward_) {
        s = number(-1.0, 1.0, 4);
        inwardLayout->addWidget(s);
        connect(s, &QDoubleSpinBox::valueChanged, this, &WorkholdingLibraryDialog::markDirty);
    }
    inward->setToolTip(tr("Towards the chuck's axis with the jaw on this way, in the jaw's own "
                          "coordinates"));
    setForm->addRow(tr("In towards the axis"), inward);
    jawSteps_ = new QTableWidget(0, 3);
    jawSteps_->setObjectName(QStringLiteral("workholdingJawSteps"));
    jawSteps_->setHorizontalHeaderLabels({tr("Step"), tr("Grips on"), tr("Work stops on")});
    jawSteps_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    jawSteps_->verticalHeader()->hide();
    jawSteps_->setMaximumHeight(140);
    jawSteps_->setToolTip(tr("Each step: the datum on the face that grips (its normal facing the "
                             "axis grips an outside diameter, facing away an inside one) and the "
                             "one the work's end stops against. A soft blank's step is what it "
                             "clamps the boring ring or plug by."));
    connect(jawSteps_, &QTableWidget::itemChanged, this, &WorkholdingLibraryDialog::markDirty);
    setForm->addRow(tr("Steps"), jawSteps_);
    auto* stepButtons = new QWidget;
    auto* stepLayout = new QHBoxLayout(stepButtons);
    stepLayout->setContentsMargins(0, 0, 0, 0);
    auto* addStep = new QPushButton(tr("Add step"));
    addStep->setObjectName(QStringLiteral("workholdingJawAddStep"));
    auto* removeStep = new QPushButton(tr("Remove step"));
    stepLayout->addWidget(addStep);
    stepLayout->addWidget(removeStep);
    stepLayout->addStretch(1);
    setForm->addRow(QString(), stepButtons);
    connect(addStep, &QPushButton::clicked, this, [this] {
        addJawStepRow({ss(tr("Step %1").arg(jawSteps_->rowCount() + 1)), {}, {}});
        markDirty();
    });
    connect(removeStep, &QPushButton::clicked, this, [this] {
        if (jawSteps_->currentRow() >= 0) {
            jawSteps_->removeRow(jawSteps_->currentRow());
            markDirty();
        }
    });
    layout->addWidget(jawSetRows_);
    into->addWidget(box);

    connect(jawRole_, &QComboBox::currentIndexChanged, this, [this] {
        updateJawRows();
        markDirty();
    });
    connect(jawInterface_, &QLineEdit::textEdited, this, &WorkholdingLibraryDialog::markDirty);
    for (QComboBox* c : {jawSeat_, jawFront_, jawKind_}) {
        connect(c, &QComboBox::currentIndexChanged, this, &WorkholdingLibraryDialog::markDirty);
    }
    connect(jawOption_, &QComboBox::currentIndexChanged, this, [this](int index) {
        if (!loading_ && index >= 0) {
            showJawOption(index);
        }
    });
    connect(jawOptionName_, &QLineEdit::textEdited, this, [this](const QString& text) {
        if (jawOptionShown_ >= 0) {
            jawOption_->setItemText(jawOptionShown_, text);
            markDirty();
        }
    });
    connect(addOption, &QPushButton::clicked, this, [this] {
        document::JawOption copy = shownJawOption();
        copy.name = ss(tr("Option %1").arg(jawOptionData_.size() + 1));
        if (jawOptionShown_ >= 0) {
            jawOptionData_[static_cast<std::size_t>(jawOptionShown_)] = shownJawOption();
        }
        jawOptionData_.push_back(copy);
        {
            const QSignalBlocker block(jawOption_);
            jawOption_->addItem(qs(copy.name));
        }
        jawOptionShown_ = -1; // what the fields show is kept above
        jawOption_->setCurrentIndex(jawOption_->count() - 1);
        markDirty();
    });
    connect(removeOption, &QPushButton::clicked, this, [this] {
        if (jawOptionShown_ < 0 || jawOptionData_.size() < 2) {
            return; // a jaw set goes on some way
        }
        const int gone = jawOptionShown_;
        jawOptionData_.erase(jawOptionData_.begin() + gone);
        jawOptionShown_ = -1;
        {
            const QSignalBlocker block(jawOption_);
            jawOption_->removeItem(gone);
        }
        showJawOption(std::min(gone, jawOption_->count() - 1));
        markDirty();
    });
    connect(topJaws_, &QListWidget::itemChanged, this, [this](QListWidgetItem* item) {
        const int r = topJaws_->row(item);
        if (!loading_ && r >= 0 && r < static_cast<int>(partData_.size())) {
            partData_[static_cast<std::size_t>(r)].topJaw = item->checkState() == Qt::Checked;
            markDirty();
        }
    });
    updateJawRows();
}

void WorkholdingLibraryDialog::updateJawRows() {
    const int role = jawRole_->currentIndex();
    jawInterface_->setEnabled(role != 0);
    chuckJawRows_->setVisible(role == 1);
    jawSetRows_->setVisible(role == 2);
    // The seat is one field for both: in the chuck's rows, or the jaw set's.
    if (auto* chuckForm = qobject_cast<QFormLayout*>(chuckJawRows_->layout());
        chuckForm && role == 2 && jawSeat_->parentWidget() == chuckJawRows_) {
        chuckForm->takeRow(jawSeat_);
        qobject_cast<QFormLayout*>(jawSetRows_->layout())->insertRow(4, tr("Jaw seat"), jawSeat_);
    } else if (auto* setForm = qobject_cast<QFormLayout*>(jawSetRows_->layout());
               setForm && role == 1 && jawSeat_->parentWidget() == jawSetRows_) {
        setForm->takeRow(jawSeat_);
        qobject_cast<QFormLayout*>(chuckJawRows_->layout())->insertRow(1, tr("Jaw seat"), jawSeat_);
    }
}

void WorkholdingLibraryDialog::addJawStepRow(const document::JawStep& step) {
    const QSignalBlocker block(jawSteps_);
    const int r = jawSteps_->rowCount();
    jawSteps_->insertRow(r);
    jawSteps_->setItem(r, 0, new QTableWidgetItem(qs(step.name)));
    for (const auto& [c, value] : {std::pair{1, step.grip}, std::pair{2, step.stop}}) {
        auto* combo = new QComboBox;
        combo->addItem(tr("(choose)"), QString());
        for (const auto& d : datums_) {
            combo->addItem(qs(d.name), qs(d.name));
        }
        combo->setCurrentIndex(std::max(0, combo->findData(qs(value))));
        connect(combo, &QComboBox::currentIndexChanged, this, &WorkholdingLibraryDialog::markDirty);
        jawSteps_->setCellWidget(r, c, combo);
    }
}

void WorkholdingLibraryDialog::refreshTopJaws() {
    const QSignalBlocker block(topJaws_);
    topJaws_->clear();
    for (std::size_t i = 0; i < partData_.size(); ++i) {
        auto* item = new QListWidgetItem(qs(partData_[i].name));
        item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable);
        item->setCheckState(partData_[i].topJaw ? Qt::Checked : Qt::Unchecked);
        topJaws_->addItem(item);
    }
}

void WorkholdingLibraryDialog::loadJaws(const document::WorkholdingTemplate& w) {
    const QSignalBlocker b1(jawRole_);
    jawRole_->setCurrentIndex(w.jawSet ? 2 : w.jawMount ? 1 : 0);
    updateJawRows();
    jawInterface_->setText(w.jawSet     ? qs(w.jawSet->interfaceName)
                           : w.jawMount ? qs(w.jawMount->interfaceName)
                                        : QString());
    if (const auto& m = w.jawMount) {
        jawSeat_->setCurrentIndex(std::max(0, jawSeat_->findData(qs(m->seat))));
    }
    const document::JawSet set = w.jawSet.value_or(document::JawSet{});
    jawKind_->setCurrentIndex(set.kind == document::JawSet::Kind::Soft ? 1 : 0);
    jawFront_->setCurrentIndex(std::max(0, jawFront_->findData(qs(set.front))));
    jawOptionData_ = set.options;
    if (jawOptionData_.empty()) {
        jawOptionData_.push_back({ss(tr("Normal")), {}, Vec3::UnitX(), {}});
    }
    {
        const QSignalBlocker block(jawOption_);
        jawOption_->clear();
        for (const document::JawOption& o : jawOptionData_) {
            jawOption_->addItem(qs(o.name));
        }
        jawOption_->setCurrentIndex(0);
    }
    jawOptionShown_ = -1;
    showJawOption(0);
    refreshTopJaws();
}

document::JawOption WorkholdingLibraryDialog::shownJawOption() const {
    document::JawOption o;
    o.name = ss(jawOptionName_->text().simplified());
    o.seat = ss(jawSeat_->currentData().toString());
    o.inward = Vec3(jawInward_[0]->value(), jawInward_[1]->value(), jawInward_[2]->value());
    for (int r = 0; r < jawSteps_->rowCount(); ++r) {
        const QTableWidgetItem* name = jawSteps_->item(r, 0);
        const auto datum = [&](int c) {
            const auto* combo = qobject_cast<QComboBox*>(jawSteps_->cellWidget(r, c));
            return combo ? ss(combo->currentData().toString()) : std::string();
        };
        o.steps.push_back(
            {name ? ss(name->text().simplified()) : std::string(), datum(1), datum(2)});
    }
    return o;
}

void WorkholdingLibraryDialog::showJawOption(int index) {
    if (jawOptionShown_ >= 0 && jawOptionShown_ < static_cast<int>(jawOptionData_.size())) {
        jawOptionData_[static_cast<std::size_t>(jawOptionShown_)] = shownJawOption();
    }
    if (index < 0 || index >= static_cast<int>(jawOptionData_.size())) {
        return;
    }
    const bool wasLoading = loading_;
    loading_ = true; // filling the fields changes nothing
    const document::JawOption& o = jawOptionData_[static_cast<std::size_t>(index)];
    jawOptionName_->setText(qs(o.name));
    if (jawRole_->currentIndex() == 2) {
        const QSignalBlocker block(jawSeat_);
        jawSeat_->setCurrentIndex(std::max(0, jawSeat_->findData(qs(o.seat))));
    }
    for (int k = 0; k < 3; ++k) {
        const QSignalBlocker block(jawInward_[k]);
        jawInward_[k]->setValue(o.inward[k]);
    }
    {
        const QSignalBlocker block(jawSteps_);
        jawSteps_->setRowCount(0);
    }
    for (const document::JawStep& step : o.steps) {
        addJawStepRow(step);
    }
    {
        const QSignalBlocker block(jawOption_);
        jawOption_->setCurrentIndex(index);
    }
    jawOptionShown_ = index;
    loading_ = wasLoading;
}

void WorkholdingLibraryDialog::editedJaws(document::WorkholdingTemplate& w) const {
    const int role = jawRole_->currentIndex();
    const std::string mountName = ss(jawInterface_->text().simplified());
    w.jawMount.reset();
    w.jawSet.reset();
    if (role == 1) {
        w.jawMount = document::JawMount{mountName, ss(jawSeat_->currentData().toString())};
    } else if (role == 2) {
        document::JawSet set;
        set.interfaceName = mountName;
        set.kind = jawKind_->currentIndex() == 1 ? document::JawSet::Kind::Soft
                                                 : document::JawSet::Kind::Hard;
        set.front = ss(jawFront_->currentData().toString());
        set.options = jawOptionData_;
        if (jawOptionShown_ >= 0 && jawOptionShown_ < static_cast<int>(set.options.size())) {
            set.options[static_cast<std::size_t>(jawOptionShown_)] = shownJawOption();
        }
        w.jawSet = std::move(set);
    }
    if (role != 1) {
        for (document::WorkholdingPart& p : w.parts) {
            p.topJaw = false; // only a chuck taking jaw sets has top jaws to leave out
        }
    }
}

} // namespace ocam::gui
