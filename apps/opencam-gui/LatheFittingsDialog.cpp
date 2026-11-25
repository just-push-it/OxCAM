// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "LatheFittingsDialog.hpp"

#include "OcctView.hpp"
#include "ShowBuiltins.hpp"

#include <ocam/cad/SubShape.hpp>
#include <ocam/document/LatheMachine.hpp>
#include <ocam/foundation/Error.hpp>
#include <ocam/foundation/Units.hpp>
#include <ocam/machine/Machine.hpp>
#include <ocam/machine/Model.hpp>

#include <QCheckBox>
#include <QComboBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QImage>
#include <QInputDialog>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPainter>
#include <QPushButton>
#include <QScrollArea>
#include <QSettings>
#include <QSplitter>
#include <QStandardPaths>
#include <QVBoxLayout>
#include <array>
#include <cmath>

namespace ocam::gui {
namespace {

QString qs(const std::string& text) {
    return QString::fromStdString(text);
}

std::string ss(const QString& text) {
    return text.toStdString();
}

std::filesystem::path& folderOverride() {
    static std::filesystem::path path;
    return path;
}

QDoubleSpinBox* number(double lo, double hi, const QString& suffix = {}) {
    auto* box = new QDoubleSpinBox;
    box->setRange(lo, hi);
    box->setDecimals(3);
    box->setSuffix(suffix);
    return box;
}

constexpr std::uint32_t kSeatColour = 0xE08A1E;
constexpr std::uint32_t kMountColour = 0x22B45A;

} // namespace

std::filesystem::path LatheFittingLibrary::folder() {
    if (!folderOverride().empty()) {
        return folderOverride();
    }
    const QSettings settings;
    QString dir = settings.value(QStringLiteral("fittingLibraryFolder")).toString();
    if (dir.isEmpty()) {
        dir = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation) +
              QStringLiteral("/fittings");
    }
    std::filesystem::path path(dir.toStdU16String());
    std::error_code ec;
    std::filesystem::create_directories(path, ec);
    return path;
}

void LatheFittingLibrary::setFolderOverride(std::filesystem::path folder) {
    folderOverride() = std::move(folder);
}

std::vector<document::FittingPackage>
LatheFittingLibrary::load(std::vector<std::string>* problems) {
    std::vector<document::FittingPackage> out = document::builtinFittings();
    for (document::FittingPackage& p : document::loadFittings(folder(), problems)) {
        out.push_back(std::move(p));
    }
    return out;
}

LatheFittingsDialog::LatheFittingsDialog(QWidget* parent) : QDialog(parent) {
    setWindowTitle(tr("Turret Blocks and Tailstock Centres"));
    resize(1320, 820);
    auto* root = new QHBoxLayout(this);
    auto* split = new QSplitter;
    root->addWidget(split);

    auto* left = new QWidget;
    auto* leftLayout = new QVBoxLayout(left);
    list_ = new QListWidget;
    list_->setObjectName(QStringLiteral("fittingList"));
    leftLayout->addWidget(list_, 1);
    showBuiltins_ = newShowBuiltinsBox();
    leftLayout->addWidget(showBuiltins_);
    connect(showBuiltins_, &QCheckBox::toggled, this, [this] { applyShowBuiltins(); });
    auto* addButton = new QPushButton(tr("New from STEP..."));
    addButton->setToolTip(tr("A new block or centre from a model of yours. STEP is kept as it is: "
                             "points are picked on its exact faces and edges."));
    auto* duplicate = new QPushButton(tr("Duplicate"));
    remove_ = new QPushButton(tr("Delete"));
    auto* buttons = new QHBoxLayout;
    buttons->addWidget(addButton);
    buttons->addWidget(duplicate);
    buttons->addWidget(remove_);
    leftLayout->addLayout(buttons);
    split->addWidget(left);

    auto* editor = new QWidget;
    editor_ = editor;
    auto* editorLayout = new QVBoxLayout(editor);
    readOnly_ = new QLabel(tr("Built-in examples are read-only, and their sizes are placeholders. "
                              "Duplicate one to change it."));
    readOnly_->setWordWrap(true);
    editorLayout->addWidget(readOnly_);
    auto* form = new QFormLayout;
    name_ = new QLineEdit;
    kind_ = new QComboBox;
    kind_->addItem(tr("Tooling block on a turret station"));
    kind_->addItem(tr("Centre in the tailstock's quill"));
    description_ = new QLineEdit;
    // Names, compared without regard to case or spacing with what the machine says it takes.
    mountType_ = new QComboBox;
    mountType_->setObjectName(QStringLiteral("fittingMountType"));
    mountType_->setEditable(true);
    mountType_->setInsertPolicy(QComboBox::NoInsert);
    mountType_->lineEdit()->setPlaceholderText(
        tr("Not stated: fits any machine that does not say"));
    form->addRow(tr("Name"), name_);
    form->addRow(tr("Kind"), kind_);
    form->addRow(tr("Notes"), description_);
    form->addRow(tr("Mounts by"), mountType_);
    mountTypeLabel_ = qobject_cast<QLabel*>(form->labelForField(mountType_));
    editorLayout->addLayout(form);

    seat_ = buildPoint(tr("Turret mount"), true);
    editorLayout->addWidget(seat_.box);
    mount_ = buildPoint(tr("Tool mount"), false);
    editorLayout->addWidget(mount_.box);

    blockOnly_ = new QWidget;
    auto* blockForm = new QFormLayout(blockOnly_);
    blockForm->setContentsMargins(0, 0, 0, 0);
    seatTurn_ = number(-360.0, 360.0, tr("°"));
    seatTurn_->setToolTip(tr("Turns the block about its seat, when the points do not say which way "
                             "is towards the work"));
    overhang_ = number(0.0, 2000.0, tr(" mm"));
    overhang_->setToolTip(tr("How far a tool's tip stands beyond the tool mount, unless a setup "
                             "says otherwise"));
    takes_ = new QComboBox;
    takes_->setObjectName(QStringLiteral("fittingTakes"));
    takes_->addItem(tr("Square shanks"));
    takes_->addItem(tr("Round shanks and bars"));
    takes_->addItem(tr("Both"));
    takes_->setToolTip(tr("A turning holder has a square shank; a boring bar, a drill or a mill a "
                          "round one. A tool with the other kind is reported."));
    // A size is either stated or not: never a 0 that means anything.
    const auto sizeRow = [&](QCheckBox*& stated, QDoubleSpinBox*& size, const QString& tip) {
        stated = new QCheckBox(tr("States its size"));
        size = number(0.1, 500.0, tr(" mm"));
        size->setToolTip(tip);
        auto* row = new QWidget;
        auto* layout = new QHBoxLayout(row);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->addWidget(stated);
        layout->addWidget(size, 1);
        return row;
    };
    shankRow_ = sizeRow(shankStated_, shank_,
                        tr("The square shank it takes: a holder of another size is reported"));
    boreRow_ = sizeRow(boreStated_, bore_,
                       tr("Its bore: the largest round shank or bar it takes. A smaller one goes "
                          "in through a collet or sleeve; a larger one is "
                          "reported"));
    shankStated_->setObjectName(QStringLiteral("fittingShankStated"));
    boreStated_->setObjectName(QStringLiteral("fittingBoreStated"));
    blockForm->addRow(tr("Turned on the seat"), seatTurn_);
    blockForm->addRow(tr("Tool sticks out"), overhang_);
    blockForm->addRow(tr("Takes"), takes_);
    blockForm->addRow(tr("Square shank"), shankRow_);
    blockForm->addRow(tr("Round bore"), boreRow_);
    editorLayout->addWidget(blockOnly_);

    status_ = new QLabel;
    status_->setObjectName(QStringLiteral("fittingStatus"));
    status_->setWordWrap(true);
    editorLayout->addWidget(status_);
    save_ = new QPushButton(tr("Save to library"));
    editorLayout->addWidget(save_);
    editorLayout->addStretch(1);
    auto* scroll = new QScrollArea;
    scroll->setWidget(editor);
    scroll->setWidgetResizable(true);
    split->addWidget(scroll);

    auto* right = new QWidget;
    auto* rightLayout = new QVBoxLayout(right);
    preview_ = new OcctView;
    rightLayout->addWidget(preview_, 1);
    onLathe_ = new QCheckBox(tr("Show it on a lathe"));
    onLathe_->setObjectName(QStringLiteral("fittingOnLathe"));
    onLathe_->setToolTip(tr("A block on station 1 of the generic lathe's turret, in the working "
                            "position, with an example tool; a centre in the work's centre hole. "
                            "Points are picked with this off."));
    connect(onLathe_, &QCheckBox::toggled, this, [this] { updateScene(true); });
    rightLayout->addWidget(onLathe_);
    auto* legend =
        new QLabel(tr("<span style=\"color:#e08a1e\">Orange</span>: the seat and what "
                      "it sits on. <span style=\"color:#22b45a\">Green</span>: the tool "
                      "mount, or a centre's tip. Each line points the way its label says. "
                      "The tool drawn is only an example."));
    legend->setWordWrap(true);
    rightLayout->addWidget(legend);
    split->addWidget(right);
    split->setStretchFactor(0, 0);
    split->setStretchFactor(1, 0);
    split->setStretchFactor(2, 1);
    split->setSizes({260, 500, 560});

    connect(list_, &QListWidget::currentRowChanged, this, &LatheFittingsDialog::showFitting);
    connect(preview_, &OcctView::machineShapeClicked, this,
            [this](int part, int type, int subIndex, double x, double y, double z) {
                clicked(part, type, subIndex, Vec3(x, y, z));
            });
    for (QLineEdit* edit : {name_, description_}) {
        connect(edit, &QLineEdit::textEdited, this, &LatheFittingsDialog::fieldsChanged);
    }
    connect(kind_, &QComboBox::activated, this, &LatheFittingsDialog::fieldsChanged);
    connect(takes_, &QComboBox::activated, this, &LatheFittingsDialog::fieldsChanged);
    for (QCheckBox* box : {shankStated_, boreStated_}) {
        connect(box, &QCheckBox::toggled, this, &LatheFittingsDialog::fieldsChanged);
    }
    connect(mountType_, &QComboBox::currentTextChanged, this, &LatheFittingsDialog::fieldsChanged);
    for (QDoubleSpinBox* box : {seatTurn_, overhang_, shank_, bore_}) {
        connect(box, &QDoubleSpinBox::valueChanged, this, &LatheFittingsDialog::fieldsChanged);
    }
    connect(save_, &QPushButton::clicked, this, [this] {
        QString why;
        if (!saveCurrent(&why)) {
            QMessageBox::warning(this, windowTitle(), why);
        }
    });
    connect(addButton, &QPushButton::clicked, this, [this] {
        const QString file =
            QFileDialog::getOpenFileName(this, tr("A model for the block or centre"), QString(),
                                         tr("STEP files (*.step *.stp *.STEP *.STP)"));
        if (!file.isEmpty()) {
            addFromFile(std::filesystem::path(file.toStdU16String()));
        }
    });
    connect(duplicate, &QPushButton::clicked, this, [this] {
        if (currentRow_ < 0) {
            return;
        }
        document::FittingPackage copy = fittings_[static_cast<std::size_t>(currentRow_)];
        copy.builtin = false;
        copy.fitting.id = Uuid::generate();
        copy.fitting.name = ss(tr("%1 copy").arg(qs(copy.fitting.name)));
        try {
            document::saveFitting(LatheFittingLibrary::folder(), copy);
        } catch (const Error& e) {
            QMessageBox::warning(this, windowTitle(), QString::fromUtf8(e.what()));
            return;
        }
        reload();
        selectByName(qs(copy.fitting.name));
    });
    connect(remove_, &QPushButton::clicked, this, [this] {
        if (currentRow_ >= 0 && !fittings_[static_cast<std::size_t>(currentRow_)].builtin) {
            document::removeFittingFile(
                LatheFittingLibrary::folder(),
                fittings_[static_cast<std::size_t>(currentRow_)].fitting.id);
            reload();
        }
    });
    reload();
}

LatheFittingsDialog::PointEditors LatheFittingsDialog::buildPoint(const QString& title, bool seat) {
    PointEditors e;
    e.box = new QGroupBox(title);
    auto* grid = new QFormLayout(e.box);
    const char* letters[] = {"X", "Y", "Z"};
    auto* pointRow = new QHBoxLayout;
    auto* directionRow = new QHBoxLayout;
    for (std::size_t k = 0; k < 3; ++k) {
        e.point[k] = number(-100000.0, 100000.0);
        e.point[k]->setButtonSymbols(QAbstractSpinBox::NoButtons);
        e.point[k]->setPrefix(QString::fromLatin1(letters[k]) + QLatin1Char(' '));
        e.direction[k] = number(-1.0, 1.0);
        e.direction[k]->setButtonSymbols(QAbstractSpinBox::NoButtons);
        e.direction[k]->setPrefix(QString::fromLatin1(letters[k]) + QLatin1Char(' '));
        e.direction[k]->setSingleStep(0.1);
        pointRow->addWidget(e.point[k]);
        directionRow->addWidget(e.direction[k]);
        connect(e.point[k], &QDoubleSpinBox::valueChanged, this,
                &LatheFittingsDialog::fieldsChanged);
        connect(e.direction[k], &QDoubleSpinBox::valueChanged, this,
                &LatheFittingsDialog::fieldsChanged);
    }
    grid->addRow(tr("Point"), pointRow);
    grid->addRow(tr("Direction"), directionRow);
    e.hint = new QLabel;
    e.hint->setWordWrap(true);
    grid->insertRow(0, e.hint);
    e.on = new QLabel;
    auto* row = new QHBoxLayout;
    e.pick = new QPushButton(tr("Pick on the model"));
    e.pick->setCheckable(true);
    e.pick->setObjectName(seat ? QStringLiteral("fittingPickSeat")
                               : QStringLiteral("fittingPickMount"));
    auto* reverse = new QPushButton(tr("Reverse"));
    reverse->setToolTip(tr("The direction the other way"));
    row->addWidget(e.pick);
    row->addWidget(reverse);
    row->addWidget(e.on, 1);
    grid->addRow(QString(), row);
    connect(e.pick, &QPushButton::clicked, this,
            [this, seat](bool on) { setPicking(on ? std::optional<bool>(seat) : std::nullopt); });
    connect(reverse, &QPushButton::clicked, this, [this, seat] {
        PointEditors& editors = seat ? seat_ : mount_;
        for (QDoubleSpinBox* box : editors.direction) {
            const QSignalBlocker block(box);
            box->setValue(-box->value());
        }
        fieldsChanged();
    });
    return e;
}

void LatheFittingsDialog::reload() {
    std::vector<std::string> problems;
    fittings_ = LatheFittingLibrary::load(&problems);
    {
        const QSignalBlocker block(list_);
        list_->clear();
        for (const document::FittingPackage& p : fittings_) {
            auto* item = new QListWidgetItem(p.builtin ? tr("%1 (built-in)").arg(qs(p.fitting.name))
                                                       : qs(p.fitting.name),
                                             list_);
            item->setToolTip(qs(p.fitting.description));
        }
    }
    currentRow_ = -1;
    applyShowBuiltins();
    if (!problems.empty()) {
        status_->setText(qs(problems.front()));
    }
}

void LatheFittingsDialog::applyShowBuiltins() {
    const bool show = showBuiltins_->isChecked();
    int first = -1;
    for (std::size_t i = 0; i < fittings_.size(); ++i) {
        QListWidgetItem* row = list_->item(static_cast<int>(i));
        row->setHidden(fittings_[i].builtin && !show);
        if (first < 0 && !row->isHidden()) {
            first = static_cast<int>(i);
        }
    }
    QListWidgetItem* current = list_->currentItem();
    if (!current || current->isHidden()) {
        list_->setCurrentRow(first);
    }
    // None of the user's and the built-ins hidden: nothing to edit.
    editor_->setEnabled(first >= 0);
}

bool LatheFittingsDialog::selectByName(const QString& name) {
    for (std::size_t i = 0; i < fittings_.size(); ++i) {
        if (qs(fittings_[i].fitting.name) == name) {
            if (fittings_[i].builtin && !showBuiltins_->isChecked()) {
                showBuiltins_->setChecked(true); // what it is opened on is shown
            }
            list_->setCurrentRow(static_cast<int>(i));
            return true;
        }
    }
    return false;
}

const document::LatheFitting* LatheFittingsDialog::current() const {
    return currentRow_ >= 0 ? &fittings_[static_cast<std::size_t>(currentRow_)].fitting : nullptr;
}

void LatheFittingsDialog::showFitting(int row) {
    if (row < 0 || row >= static_cast<int>(fittings_.size())) {
        return;
    }
    if (dirty_ && currentRow_ >= 0 && currentRow_ != row &&
        !fittings_[static_cast<std::size_t>(currentRow_)].builtin) {
        const auto answer = QMessageBox::question(
            this, windowTitle(),
            tr("Save the changes to '%1'?")
                .arg(qs(fittings_[static_cast<std::size_t>(currentRow_)].fitting.name)));
        if (answer == QMessageBox::Yes) {
            QString why;
            if (!saveCurrent(&why)) {
                QMessageBox::warning(this, windowTitle(), why);
            }
        }
    }
    currentRow_ = row;
    dirty_ = false;
    setPicking(std::nullopt);
    const document::FittingPackage& p = fittings_[static_cast<std::size_t>(row)];
    loading_ = true;
    name_->setText(qs(p.fitting.name));
    description_->setText(qs(p.fitting.description));
    kind_->setCurrentIndex(p.fitting.kind == document::LatheFitting::Kind::Block ? 0 : 1);
    const auto fill = [](PointEditors& e, const document::FittingPoint& from) {
        for (int k = 0; k < 3; ++k) {
            e.point[static_cast<std::size_t>(k)]->setValue(from.point[k]);
            e.direction[static_cast<std::size_t>(k)]->setValue(from.direction[k]);
        }
        e.on->setText(qs(from.on));
    };
    fill(seat_, p.fitting.seat);
    fill(mount_, p.fitting.mount);
    seatTurn_->setValue(p.fitting.seatTurn);
    overhang_->setValue(p.fitting.overhang);
    takes_->setCurrentIndex(static_cast<int>(p.fitting.takes));
    shankStated_->setChecked(p.fitting.squareShank.has_value());
    shank_->setValue(p.fitting.squareShank.value_or(shank_->value()));
    boreStated_->setChecked(p.fitting.roundBar.has_value());
    bore_->setValue(p.fitting.roundBar.value_or(bore_->value()));
    fillMountTypes(p.fitting.kind == document::LatheFitting::Kind::Block);
    mountType_->setEditText(qs(p.fitting.mountType));
    loading_ = false;
    model_ = cad::Model();
    try {
        model_ = document::stepModel(p.file, p.fitting.name);
    } catch (const Error& e) {
        status_->setText(QString::fromUtf8(e.what()));
    }
    const bool editable = !p.builtin;
    readOnly_->setVisible(!editable);
    for (QWidget* w :
         std::initializer_list<QWidget*>{name_, kind_, description_, mountType_, seat_.box,
                                         mount_.box, blockOnly_, save_, remove_}) {
        w->setEnabled(editable);
    }
    updateLabels();
    updateScene(true);
}

void LatheFittingsDialog::fieldsChanged() {
    if (loading_ || currentRow_ < 0) {
        return;
    }
    dirty_ = true;
    document::LatheFitting& f = fittings_[static_cast<std::size_t>(currentRow_)].fitting;
    f.name = ss(name_->text());
    f.description = ss(description_->text());
    const document::LatheFitting::Kind was = f.kind;
    f.kind = kind_->currentIndex() == 0 ? document::LatheFitting::Kind::Block
                                        : document::LatheFitting::Kind::Centre;
    const auto read = [](const PointEditors& e, document::FittingPoint& to) {
        const Vec3 point(e.point[0]->value(), e.point[1]->value(), e.point[2]->value());
        const Vec3 direction(e.direction[0]->value(), e.direction[1]->value(),
                             e.direction[2]->value());
        if ((point - to.point).norm() > 1e-9 || (direction - to.direction).norm() > 1e-3) {
            to.on.clear(); // typed
        }
        to.point = point;
        if ((direction - to.direction).norm() > 1e-3) {
            to.direction = direction;
        }
    };
    read(seat_, f.seat);
    read(mount_, f.mount);
    f.seatTurn = seatTurn_->value();
    f.overhang = overhang_->value();
    f.takes = static_cast<document::LatheFitting::Takes>(takes_->currentIndex());
    f.squareShank =
        shankStated_->isChecked() ? std::optional<double>(shank_->value()) : std::nullopt;
    f.roundBar = boreStated_->isChecked() ? std::optional<double>(bore_->value()) : std::nullopt;
    f.mountType = ss(mountType_->currentText().simplified());
    if (f.kind != was) {
        loading_ = true;
        fillMountTypes(f.kind == document::LatheFitting::Kind::Block);
        mountType_->setEditText(qs(f.mountType));
        loading_ = false;
    }
    updateLabels();
    updateScene(false);
}

void LatheFittingsDialog::fillMountTypes(bool block) {
    const QSignalBlocker quiet(mountType_);
    const QString text = mountType_->currentText();
    mountType_->clear();
    // Names only, as suggestions: what the user's machine takes is theirs to type.
    const QStringList names =
        block ? QStringList{QStringLiteral("VDI 20"),
                            QStringLiteral("VDI 25"),
                            QStringLiteral("VDI 30"),
                            QStringLiteral("VDI 40"),
                            QStringLiteral("VDI 50"),
                            QStringLiteral("VDI 60"),
                            QStringLiteral("BMT 45"),
                            QStringLiteral("BMT 55"),
                            QStringLiteral("BMT 65"),
                            QStringLiteral("BMT 75"),
                            QString::fromUtf8(document::kGenericBlockMount)}
              : QStringList{
                    QStringLiteral("MT 2"), QStringLiteral("MT 3"),
                    QStringLiteral("MT 4"), QStringLiteral("MT 5"),
                    QStringLiteral("MT 6"), QString::fromUtf8(document::kGenericQuillInterface)};
    mountType_->addItems(names);
    mountType_->setEditText(text);
}

void LatheFittingsDialog::updateLabels() {
    if (currentRow_ < 0) {
        return;
    }
    const document::LatheFitting& f = fittings_[static_cast<std::size_t>(currentRow_)].fitting;
    const bool block = f.kind == document::LatheFitting::Kind::Block;
    seat_.box->setTitle(block ? tr("Turret mount") : tr("Seat in the quill"));
    seat_.hint->setText(
        block ? tr("Where it seats on a station, and the direction out of the turret.")
              : tr("Where it sits in the quill, and the direction towards the tip."));
    mount_.box->setTitle(block ? tr("Tool mount") : tr("Tip"));
    mount_.hint->setText(block ? tr("On the tool's centre line at the block's mouth, at the height "
                                    "of the tool's tip, and the direction towards the tip.")
                               : tr("Its point."));
    blockOnly_->setVisible(block);
    if (mountTypeLabel_) {
        mountTypeLabel_->setText(block ? tr("Mounts by") : tr("Fits a quill taking"));
    }
    mountType_->setToolTip(
        block
            ? tr("The kind of block it is, by name, as the machine's turret lists the kinds it "
                 "takes (such as VDI 40 or BMT 55). The setup offers it on machines that take it.")
            : tr("What its shank fits, by name, as the machine's tailstock says its quill takes "
                 "(such as MT 4). The setup offers it on machines whose quill takes it."));
    shankRow_->setEnabled(f.takesSquare());
    boreRow_->setEnabled(f.takesRound());
    shank_->setEnabled(shankStated_->isChecked());
    bore_->setEnabled(boreStated_->isChecked());
    QString text;
    if (const auto issues = f.validate(); !issues.empty()) {
        text = QStringLiteral("<span style=\"color:#d04040\">%1</span>").arg(qs(issues.front()));
    } else if (block) {
        text = document::blockAcross(f) ? tr("Holds a tool across the spindle.")
                                        : tr("Holds a tool along the spindle.");
    } else {
        text = tr("%1 mm from its seat to its tip.").arg(document::centreLength(f), 0, 'f', 3);
    }
    status_->setText(text);
}

void LatheFittingsDialog::updateScene(bool fit) {
    if (currentRow_ < 0) {
        return;
    }
    const document::FittingPackage& p = fittings_[static_cast<std::size_t>(currentRow_)];
    const document::LatheFitting& f = p.fitting;
    const bool block = f.kind == document::LatheFitting::Kind::Block;
    const bool usable = f.validate().empty();
    // On a lathe: the generic one's turret, or the work and its centre hole, so which way round
    // the points put it shows.
    const bool onLathe = onLathe_->isChecked() && usable;
    const machine::LatheLayout& lathe = *machine::genericLatheTurret().lathe;
    constexpr double kWorkLength = 120.0;
    constexpr double kWorkRadius = 25.0;
    std::optional<document::ExampleTool> tool;
    if (block && usable) {
        tool = document::exampleTool(f);
    }
    // A block on station 1 in the working position, the turret moved so the example tool is at
    // the work's end, as it would be cutting.
    Vec3 shift = Vec3::Zero();
    Transform3 at = Transform3::Identity();
    if (onLathe && block) {
        const Transform3 onStation = document::blockOnStation(f, lathe, false);
        const Vec3 target = tool->round ? Vec3(0.5 * kWorkRadius, 0.0, kWorkLength - 10.0)
                                        : Vec3(kWorkRadius, 0.0, kWorkLength - 10.0);
        shift = target - onStation * tool->tip;
        at = Eigen::Translation3d(shift) * onStation;
    } else if (onLathe) {
        at = document::centreInWcs(f, kWorkLength);
    }

    OcctView::MachineScene scene;
    auto parts = std::make_shared<std::vector<machine::ModelPart>>();
    machine::ModelPart part;
    part.name = f.name;
    part.colour = f.colour;
    geom3d::Aabb3 box;
    try {
        part.geometry = document::stepMesh(p.file, 0.1, f.name);
        box = part.geometry->bounds();
    } catch (const Error&) {
        // the status line has it
    }
    parts->push_back(std::move(part));
    if (tool) {
        const auto meshOf = [](const std::vector<sim::ConvexPrism>& prisms) {
            geom3d::Mesh out;
            for (const sim::ConvexPrism& prism : prisms) {
                const geom3d::Mesh one = sim::prismMesh(prism);
                const auto base = static_cast<std::uint32_t>(out.vertices.size());
                out.vertices.insert(out.vertices.end(), one.vertices.begin(), one.vertices.end());
                for (const auto& t : one.triangles) {
                    out.triangles.push_back({t[0] + base, t[1] + base, t[2] + base});
                }
            }
            return out;
        };
        machine::ModelPart shank = machine::makePart("Example tool", meshOf(tool->shank));
        shank.colour = 0xB8BEC6;
        machine::ModelPart insert = machine::makePart("Example insert", meshOf(tool->insert));
        insert.colour = 0xD9A21B;
        parts->push_back(std::move(shank));
        parts->push_back(std::move(insert));
    }
    scene.placements.assign(parts->size(), at);
    scene.parts = std::move(parts);
    if (!onLathe) {
        scene.solids = {model_};
    }
    scene.pickable = !onLathe;
    scene.transparency = 0.45;

    geom3d::Aabb3 all;
    if (!box.isEmpty()) {
        for (const Vec3& corner : {box.min, box.max, Vec3(box.min.x(), box.max.y(), box.min.z()),
                                   Vec3(box.max.x(), box.min.y(), box.max.z())}) {
            all.extend(at * corner);
        }
    }
    const double size = box.isEmpty() ? 50.0 : std::max(20.0, 0.7 * box.size().norm());
    const auto line = [&](const Vec3& a, const Vec3& b, std::uint32_t colour, bool thick = false) {
        scene.lines.push_back({a, b, thick, colour});
        all.extend(a);
        all.extend(b);
    };
    const auto label = [&](const Vec3& where, const QString& text, std::uint32_t colour) {
        scene.labels.push_back({where, ss(text), colour});
    };
    // A circle about `axis`.
    const auto circle = [&](const Vec3& centre, const Vec3& axis, double radius,
                            std::uint32_t colour) {
        const Vec3 n = axis.normalized();
        const Vec3 u = n.unitOrthogonal();
        const Vec3 v = n.cross(u);
        constexpr int kSides = 48;
        for (int k = 0; k < kSides; ++k) {
            const double a0 = 2.0 * kPi * k / kSides;
            const double a1 = 2.0 * kPi * (k + 1) / kSides;
            line(centre + radius * (std::cos(a0) * u + std::sin(a0) * v),
                 centre + radius * (std::cos(a1) * u + std::sin(a1) * v), colour);
        }
    };
    constexpr std::uint32_t kGuide = 0x9AA4B0;
    constexpr std::uint32_t kSpindle = 0x6FB7FF;
    constexpr std::uint32_t kText = 0xF2F2F2;

    if (!onLathe) {
        // Each point, its direction and the face it sits on, said in the view.
        const auto mark = [&](const document::FittingPoint& point, std::uint32_t colour,
                              const QString& name, const QString& direction) {
            const Vec3 d = point.direction.norm() > 1e-9 ? Vec3(point.direction.normalized())
                                                         : Vec3(Vec3::UnitZ());
            line(point.point, point.point + d * size, colour, true);
            const double tick = 0.08 * size;
            for (const Vec3& axis :
                 {Vec3(Vec3::UnitX()), Vec3(Vec3::UnitY()), Vec3(Vec3::UnitZ())}) {
                line(point.point - axis * tick, point.point + axis * tick, colour, true);
            }
            label(point.point, name, colour);
            if (!direction.isEmpty()) {
                label(point.point + d * size, direction, colour);
            }
        };
        mark(f.seat, kSeatColour, block ? tr("Turret mount") : tr("Seat in the quill"),
             block ? tr("out of the turret") : tr("towards the tip"));
        mark(f.mount, kMountColour, block ? tr("Tool mount") : tr("Tip"),
             block ? tr("towards the tip") : QString());
        // What the seat sits on: a square across its direction.
        if (f.seat.direction.norm() > 1e-9) {
            const Vec3 n = f.seat.direction.normalized();
            const Vec3 u = (block && usable ? document::towardsWork(f) : n.unitOrthogonal());
            const Vec3 v = n.cross(u);
            const double half = 0.45 * size;
            const Vec3 o = f.seat.point;
            const std::array<Vec3, 4> c{o + half * (u + v), o + half * (-u + v),
                                        o + half * (-u - v), o + half * (u - v)};
            for (std::size_t k = 0; k < c.size(); ++k) {
                line(c[k], c[(k + 1) % c.size()], kSeatColour);
            }
            label(c[0], block ? tr("Turret face") : tr("Quill"), kSeatColour);
        }
        if (block && usable) {
            const Vec3 w = document::towardsWork(f);
            line(f.seat.point, f.seat.point + w * (0.6 * size), kGuide, true);
            label(f.seat.point + w * (0.6 * size), tr("towards the work"), kGuide);
        }
    } else if (block) {
        const machine::LatheTurret& t = lathe.turret;
        const Vec3 c = t.centre + shift;
        const double r = t.diameter / 2.0;
        circle(c - t.axis * (t.width / 2.0), t.axis, r, kGuide);
        circle(c + t.axis * (t.width / 2.0), t.axis, r, kGuide);
        const int stations = std::max(1, machine::genericLatheTurret().toolCapacity);
        for (int k = 0; k < stations; ++k) {
            const Vec3 out =
                Eigen::AngleAxisd(2.0 * kPi * k / stations, t.axis) * Vec3(-Vec3::UnitX());
            line(c + out * r, c + out * (r + 15.0), k == 0 ? kSeatColour : kGuide, k == 0);
        }
        label(c - t.axis * (t.width / 2.0) + Vec3::UnitX() * r, tr("Turret"), kText);
        label(at * f.seat.point, tr("Station 1, working"), kSeatColour);
        line(Vec3(0.0, 0.0, -30.0), Vec3(0.0, 0.0, std::max(kWorkLength, c.z()) + 30.0), kSpindle,
             true);
        label(Vec3(0.0, 0.0, -30.0), tr("Spindle's centre line"), kSpindle);
        const double work = kWorkRadius;
        circle(Vec3::Zero(), Vec3::UnitZ(), work, kSpindle);
        circle(Vec3(0.0, 0.0, kWorkLength), Vec3::UnitZ(), work, kSpindle);
        for (const double side : {-work, work}) {
            line(Vec3(side, 0.0, 0.0), Vec3(side, 0.0, kWorkLength), kSpindle);
        }
        label(Vec3(-work, 0.0, 0.4 * kWorkLength), tr("The work"), kSpindle);
    } else {
        line(Vec3(0.0, 0.0, -30.0), Vec3(0.0, 0.0, kWorkLength + 2.5 * size), kSpindle, true);
        label(Vec3(0.0, 0.0, -30.0), tr("Spindle's centre line"), kSpindle);
        const double work = kWorkRadius;
        circle(Vec3::Zero(), Vec3::UnitZ(), work, kSpindle);
        circle(Vec3(0.0, 0.0, kWorkLength), Vec3::UnitZ(), work, kSpindle);
        for (const double side : {-work, work}) {
            line(Vec3(side, 0.0, 0.0), Vec3(side, 0.0, kWorkLength), kSpindle);
        }
        label(Vec3(-work, 0.0, 0.4 * kWorkLength), tr("The work"), kSpindle);
        const Vec3 seat = at * f.seat.point;
        circle(seat, Vec3::UnitZ(), 0.3 * size, kSeatColour);
        label(seat, tr("Quill"), kSeatColour);
    }
    if (tool) {
        // At the back of its shank: the tip has the tool mount's direction on it.
        label(at * tool->shank.front().lower.front(),
              tool->sizeStated ? tr("Example tool, %1 mm %2")
                                     .arg(tool->size, 0, 'f', 1)
                                     .arg(tool->round ? tr("bar") : tr("square shank"))
                               : tr("Example tool, %1 mm %2 (placeholder)")
                                     .arg(tool->size, 0, 'f', 1)
                                     .arg(tool->round ? tr("bar") : tr("square shank")),
              0xD9A21B);
        all.extend(at * tool->tip);
    }
    preview_->setMachine(scene);
    if (fit && !all.isEmpty()) {
        if (onLathe) {
            // From above the lathe, the spindle down the view, the turret to its right.
            preview_->lookFrom(Vec3(0.2, 1.0, 0.25).normalized(), -Vec3::UnitZ());
        }
        // Room for the labels, which run to the right of their points.
        const Vec3 margin = Vec3::Constant((onLathe ? 0.06 : 0.15) * all.size().norm());
        all.extend(all.min - margin);
        all.extend(all.max + margin);
        preview_->fitBox(all);
    }
}

void LatheFittingsDialog::setPicking(std::optional<bool> seat) {
    picking_ = seat;
    if (seat && onLathe_->isChecked()) {
        onLathe_->setChecked(false); // picked on the fitting by itself
    }
    const QSignalBlocker a(seat_.pick);
    const QSignalBlocker b(mount_.pick);
    seat_.pick->setChecked(seat && *seat);
    mount_.pick->setChecked(seat && !*seat);
}

void LatheFittingsDialog::clicked(int part, int type, int subIndex, const Vec3& at) {
    // Only the fitting's own model: not the example tool. Picked on the fitting alone, so the
    // view's frame is the model's.
    if (!picking_ || currentRow_ < 0 || model_.isNull() || part != 0) {
        return;
    }
    pickForTest(*picking_, type, subIndex, at);
    setPicking(std::nullopt);
}

void LatheFittingsDialog::pickForTest(bool seat, int type, int subIndex, std::optional<Vec3> at) {
    if (currentRow_ < 0 || model_.isNull()) {
        return;
    }
    cad::SubShapeGeometry g;
    try {
        g = cad::subShapeGeometry(model_, static_cast<cad::SubShapeType>(type), subIndex);
    } catch (const Error& e) {
        status_->setText(QString::fromUtf8(e.what()));
        return;
    }
    document::LatheFitting& f = fittings_[static_cast<std::size_t>(currentRow_)].fitting;
    document::FittingPoint& to = seat ? f.seat : f.mount;
    const bool block = f.kind == document::LatheFitting::Kind::Block;
    to.point = g.point;
    QString on;
    using Kind = cad::SubShapeGeometry::Kind;
    if (g.kind == Kind::Plane) {
        // A seat's direction is into the fitting, the way out of what it sits on; a mouth's is out
        // of it, the way the tool comes.
        to.direction = seat ? Vec3(-g.direction) : g.direction;
        on = tr("a flat face");
    } else if (g.hasAxis()) {
        // An axis has no sense of its own: away from the middle of the fitting.
        Vec3 d = g.direction.normalized();
        const geom3d::Aabb3 box = model_.info().bounds;
        if (d.dot(g.point - 0.5 * (box.min + box.max)) < 0.0) {
            d = -d;
        }
        to.direction = d;
        if (g.kind == Kind::Circle) {
            on = tr("a circular edge");
        } else {
            // A round face (a bore, a pilot): the centre of one of its ends, not its middle. The
            // end nearest where it was clicked, else the end the direction points to.
            const Vec3 axis = g.direction.normalized();
            const double s = axis.dot(g.axisEnd - g.axisStart);
            const double t = at ? axis.dot(*at - g.axisStart) : (d.dot(axis) > 0.0 ? s : 0.0);
            const bool atEnd = t > 0.5 * s;
            to.point = atEnd ? g.axisEnd : g.axisStart;
            const bool outwards = (atEnd ? 1.0 : -1.0) * d.dot(axis) > 0.0;
            on = outwards ? tr("the outer end of a round face")
                          : tr("the inner end of a round face");
        }
    } else if (g.kind == Kind::Line) {
        to.direction = g.direction;
        on = tr("a straight edge");
    } else if (g.kind == Kind::Point) {
        on = tr("a vertex");
    } else {
        on = tr("a face");
    }
    if (!block && !seat) {
        // A centre's tip: the direction goes from its seat.
        const Vec3 along = f.mount.point - f.seat.point;
        if (along.norm() > 1e-9) {
            f.mount.direction = along.normalized();
            f.seat.direction = f.mount.direction;
        }
    }
    to.on = ss(on);
    dirty_ = true;
    loading_ = true;
    const auto fill = [](PointEditors& e, const document::FittingPoint& from) {
        for (int k = 0; k < 3; ++k) {
            e.point[static_cast<std::size_t>(k)]->setValue(from.point[k]);
            e.direction[static_cast<std::size_t>(k)]->setValue(from.direction[k]);
        }
        e.on->setText(qs(from.on));
    };
    fill(seat_, f.seat);
    fill(mount_, f.mount);
    loading_ = false;
    updateLabels();
    updateScene(false);
}

void LatheFittingsDialog::addFromFile(const std::filesystem::path& file) {
    try {
        const cad::Model model = cad::Model::importStep(file);
        const std::u8string stem = file.stem().u8string();
        document::FittingPackage p = document::makeFitting(
            std::string(stem.begin(), stem.end()), document::LatheFitting::Kind::Block, model);
        const geom3d::Aabb3 box = model.info().bounds;
        p.fitting.seat.point = 0.5 * (box.min + box.max);
        p.fitting.seat.point.z() = box.min.z();
        p.fitting.mount.point = 0.5 * (box.min + box.max);
        p.fitting.mount.point.z() = box.max.z();
        p.fitting.seat.on.clear();
        document::saveFitting(LatheFittingLibrary::folder(), p);
        reload();
        selectByName(qs(p.fitting.name));
    } catch (const Error& e) {
        QMessageBox::warning(this, windowTitle(), QString::fromUtf8(e.what()));
    }
}

bool LatheFittingsDialog::saveCurrent(QString* why) {
    if (currentRow_ < 0) {
        return false;
    }
    try {
        document::saveFitting(LatheFittingLibrary::folder(),
                              fittings_[static_cast<std::size_t>(currentRow_)]);
        dirty_ = false;
        if (QListWidgetItem* item = list_->item(currentRow_)) {
            item->setText(qs(fittings_[static_cast<std::size_t>(currentRow_)].fitting.name));
        }
        return true;
    } catch (const Error& e) {
        if (why) {
            *why = QString::fromUtf8(e.what());
        }
        return false;
    }
}

QImage LatheFittingsDialog::snapshot() {
    QImage image = grab().toImage();
    const QString file = QDir::temp().filePath(QStringLiteral("oxcam-fitting-preview.png"));
    if (preview_->dumpImage(file)) {
        const QImage view(file);
        QPainter painter(&image);
        const QPoint at = preview_->mapTo(this, QPoint(0, 0));
        painter.drawImage(QRectF(at.x(), at.y(), preview_->width(), preview_->height()), view);
        QFile::remove(file);
    }
    return image;
}

} // namespace ocam::gui
