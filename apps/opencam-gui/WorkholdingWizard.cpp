// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "WorkholdingWizard.hpp"

#include "OcctView.hpp"

#include <ocam/cad/Datum.hpp>
#include <ocam/foundation/Cancel.hpp>
#include <ocam/foundation/Error.hpp>
#include <ocam/foundation/Progress.hpp>

#include <QAction>
#include <QButtonGroup>
#include <QCheckBox>
#include <QComboBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPainter>
#include <QPushButton>
#include <QRadioButton>
#include <QSlider>
#include <QSplitter>
#include <QTimer>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>

namespace ocam::gui {
namespace {

using document::FixtureType;
using document::WorkholdingPart;
using document::WorkholdingPick;

constexpr std::uint32_t kAsked = 0xF28C28;    ///< Parts answering the current question.
constexpr std::uint32_t kListed = 0x5AB4F0;   ///< Parts selected in the parts list.
constexpr std::uint32_t kAnswered = 0xE8B07A; ///< Parts answering another one.
constexpr std::uint32_t kDatum = 0x50D080;    ///< Picked faces, edges and the mount.
constexpr std::uint32_t kStock = 0xFFD040;    ///< The trial stock.

QString qs(const std::string& s) {
    return QString::fromStdString(s);
}

QDoubleSpinBox* length(double min, double max, double value) {
    auto* s = new QDoubleSpinBox;
    s->setRange(min, max);
    s->setDecimals(2);
    s->setSuffix(QStringLiteral(" mm"));
    s->setValue(value);
    s->setKeyboardTracking(false);
    return s;
}

QString vecText(const Vec3& v) {
    auto n = [](double x) {
        const double r = std::round(x * 100.0) / 100.0;
        return QString::number(r == 0.0 ? 0.0 : r, 'g', 8);
    };
    return QStringLiteral("(%1, %2, %3)").arg(n(v.x()), n(v.y()), n(v.z()));
}

} // namespace

WorkholdingWizard::WorkholdingWizard(document::WorkholdingPackage start,
                                     std::map<std::string, cad::Model> models, QWidget* parent)
    : QDialog(parent), start_(std::move(start)), models_(std::move(models)) {
    setWindowTitle(tr("Set up workholding"));
    setWindowFlags(windowFlags() | Qt::WindowMinMaxButtonsHint);
    resize(1400, 820);
    answers_.name = start_.workholding.name;
    answers_.description = start_.workholding.description;

    auto* root = new QHBoxLayout(this);
    auto* splitter = new QSplitter;
    root->addWidget(splitter);
    view_ = new OcctView;
    view_->setMinimumSize(520, 420);
    splitter->addWidget(view_);

    auto* side = new QWidget;
    auto* layout = new QVBoxLayout(side);
    stepList_ = new QListWidget;
    stepList_->setMaximumHeight(170);
    stepList_->setToolTip(tr("The questions; click an answered one to change it"));
    layout->addWidget(stepList_);
    title_ = new QLabel;
    QFont big = title_->font();
    big.setPointSizeF(big.pointSizeF() * 1.35);
    big.setBold(true);
    title_->setFont(big);
    title_->setWordWrap(true);
    layout->addWidget(title_);
    help_ = new QLabel;
    help_->setWordWrap(true);
    layout->addWidget(help_);

    // What kind of workholding.
    typePage_ = new QWidget;
    auto* typeLayout = new QVBoxLayout(typePage_);
    types_ = new QButtonGroup(this);
    const std::pair<FixtureType, const char*> kinds[] = {
        {FixtureType::Vise, QT_TR_NOOP("Standard vise: a fixed jaw and a moving jaw")},
        {FixtureType::SelfCentringVise,
         QT_TR_NOOP("Self-centring vise: both jaws move together, the stock stays centred")},
        {FixtureType::Dovetail,
         QT_TR_NOOP("Dovetail fixture: flanks hold a dovetail tenon on the stock's bottom")},
        {FixtureType::Seat,
         QT_TR_NOOP("Plate or custom fixture: the stock sits on a face (no moving jaws)")},
        {FixtureType::Chuck,
         QT_TR_NOOP("Chuck: 3, 4, 6 or more jaws move together on a bar's diameter (lathe)")},
        {FixtureType::Collet, QT_TR_NOOP("Collet: a bore that grips bars a little smaller than "
                                         "itself (lathe)")},
    };
    for (const auto& [kind, label] : kinds) {
        auto* button = new QRadioButton(tr(label));
        types_->addButton(button, static_cast<int>(kind));
        typeLayout->addWidget(button);
    }
    types_->button(static_cast<int>(FixtureType::Vise))->setChecked(true);
    auto* typeNote = new QLabel(tr("Anything else (clamps, several stations, jaws that move on "
                                   "their own) can be set up by hand in the editor's Advanced "
                                   "section."));
    typeNote->setWordWrap(true);
    typeNote->setStyleSheet(QStringLiteral("color: gray"));
    typeLayout->addWidget(typeNote);
    layout->addWidget(typePage_);

    answer_ = new QLabel;
    answer_->setWordWrap(true);
    answer_->setStyleSheet(QStringLiteral("color: #2e8b57"));
    layout->addWidget(answer_);

    // Name it and try it.
    finishPage_ = new QWidget;
    auto* form = new QFormLayout(finishPage_);
    name_ = new QLineEdit;
    description_ = new QLineEdit;
    form->addRow(tr("Name"), name_);
    form->addRow(tr("Description"), description_);
    openingRow_ = new QWidget;
    auto* openingLayout = new QHBoxLayout(openingRow_);
    openingLayout->setContentsMargins(0, 0, 0, 0);
    minOpening_ = length(0, 1e5, 0);
    maxOpening_ = length(0, 1e5, 150);
    openingLayout->addWidget(minOpening_);
    openingLayout->addWidget(new QLabel(tr("to")));
    openingLayout->addWidget(maxOpening_);
    form->addRow(tr("Opening range"), openingRow_);
    gripDepth_ = length(0, 1e4, 5);
    gripDepth_->setToolTip(tr("How far the stock reaches below the jaw tops (setups can change "
                              "it)"));
    form->addRow(tr("Grip depth"), gripDepth_);
    tenonDepth_ = length(0, 1e4, 5);
    tenonDepth_->setToolTip(tr("The tenon's height below the stock's shoulder"));
    form->addRow(tr("Tenon depth"), tenonDepth_);
    gripRange_ = length(0, 100, 0.5);
    gripRange_->setDecimals(3);
    gripRange_->setToolTip(tr("How much smaller than its bore a bar may be and still be gripped"));
    form->addRow(tr("Grips bars smaller by up to"), gripRange_);
    anchor_ = new QComboBox;
    anchor_->addItem(tr("Its bottom centre"), QStringLiteral("1,1,0"));
    anchor_->addItem(tr("Its bottom corner (min X, min Y)"), QStringLiteral("0,0,0"));
    anchor_->addItem(tr("Its bottom corner (max X, min Y)"), QStringLiteral("2,0,0"));
    anchor_->addItem(tr("Its bottom corner (min X, max Y)"), QStringLiteral("0,2,0"));
    anchor_->addItem(tr("Its bottom corner (max X, max Y)"), QStringLiteral("2,2,0"));
    form->addRow(tr("The stock's point on the face"), anchor_);
    auto* tryLabel = new QLabel(QStringLiteral("<b>%1</b>").arg(tr("Try it")));
    form->addRow(tryLabel);
    auto* sliderRow = new QHBoxLayout;
    opening_ = new QSlider(Qt::Horizontal);
    openingValue_ = new QLabel;
    openingValue_->setMinimumWidth(80);
    sliderRow->addWidget(opening_, 1);
    sliderRow->addWidget(openingValue_);
    form->addRow(tr("Opening"), sliderRow);
    trial_ = new QCheckBox(tr("Close on a trial stock"));
    form->addRow(trial_);
    const char* sizes[] = {QT_TR_NOOP("Stock width"), QT_TR_NOOP("Stock length"),
                           QT_TR_NOOP("Stock height")};
    const char* sizeTips[] = {QT_TR_NOOP("Across the jaws, from one clamping face to the other"),
                              QT_TR_NOOP("Along the jaws"),
                              QT_TR_NOOP("From the stock's bottom to its top")};
    const double sizeValues[] = {60, 100, 40};
    for (int k = 0; k < 3; ++k) {
        trialSize_[k] = length(1, 1e5, sizeValues[k]);
        trialSize_[k]->setToolTip(tr(sizeTips[k]));
        form->addRow(tr(sizes[k]), trialSize_[k]);
    }
    layout->addWidget(finishPage_);

    problem_ = new QLabel;
    problem_->setWordWrap(true);
    problem_->setStyleSheet(QStringLiteral("color: #d04040"));
    layout->addWidget(problem_);
    layout->addStretch(1);

    // The parts: remove what the fixture does not need, merge what is one part.
    auto* partsBox = new QGroupBox(tr("Parts"));
    auto* partsLayout = new QVBoxLayout(partsBox);
    parts_ = new QListWidget;
    parts_->setSelectionMode(QAbstractItemView::ExtendedSelection);
    parts_->setMaximumHeight(130);
    parts_->setToolTip(tr("Select parts here to see them in blue. Remove the ones this fixture "
                          "does not have (another jaw set, a riser); merge the ones that are one "
                          "part (Ctrl+click to select several)."));
    partsLayout->addWidget(parts_);
    auto* partButtons = new QHBoxLayout;
    removePart_ = new QPushButton(tr("Remove"));
    removePart_->setToolTip(tr("Leave the selected parts out of the workholding"));
    mergeParts_ = new QPushButton(tr("Merge"));
    mergeParts_->setToolTip(tr("Make the selected parts one part"));
    partButtons->addWidget(removePart_);
    partButtons->addWidget(mergeParts_);
    partButtons->addStretch(1);
    partsLayout->addLayout(partButtons);
    layout->addWidget(partsBox);
    auto* removeKey = new QAction(parts_);
    removeKey->setShortcut(QKeySequence::Delete);
    removeKey->setShortcutContext(Qt::WidgetShortcut);
    parts_->addAction(removeKey);

    auto* buttons = new QHBoxLayout;
    back_ = new QPushButton(tr("< Back"));
    skip_ = new QPushButton(tr("Skip"));
    next_ = new QPushButton(tr("Next >"));
    finish_ = new QPushButton(tr("Finish"));
    auto* cancel = new QPushButton(tr("Cancel"));
    next_->setDefault(true);
    for (QPushButton* b : {back_, skip_, next_, finish_, cancel}) {
        buttons->addWidget(b);
    }
    layout->addLayout(buttons);
    splitter->addWidget(side);
    splitter->setStretchFactor(0, 3);
    splitter->setStretchFactor(1, 2);

    connect(view_, &OcctView::selectionChanged, this, [this] { picked(); });
    connect(parts_, &QListWidget::itemSelectionChanged, this, [this] {
        const std::size_t n = selectedParts().size();
        removePart_->setEnabled(n > 0 && n < start_.workholding.parts.size());
        mergeParts_->setEnabled(n > 1);
        updateView();
    });
    connect(removePart_, &QPushButton::clicked, this, [this] { removeParts(selectedParts()); });
    connect(removeKey, &QAction::triggered, this, [this] { removeParts(selectedParts()); });
    connect(mergeParts_, &QPushButton::clicked, this, [this] { mergeParts(selectedParts()); });
    connect(types_, &QButtonGroup::idClicked, this,
            [this](int id) { setType(static_cast<FixtureType>(id)); });
    connect(back_, &QPushButton::clicked, this, [this] { showStep(std::max(0, current_ - 1)); });
    connect(skip_, &QPushButton::clicked, this, [this] { showStep(current_ + 1); });
    connect(next_, &QPushButton::clicked, this, [this] { next(); });
    connect(finish_, &QPushButton::clicked, this, [this] { finish(); });
    connect(cancel, &QPushButton::clicked, this, &QDialog::reject);
    connect(stepList_, &QListWidget::itemClicked, this, [this](QListWidgetItem* item) {
        const int row = stepList_->row(item);
        // Earlier questions, or later ones once everything before them is answered.
        for (int i = 0; i < row; ++i) {
            if (!steps_[static_cast<std::size_t>(i)].optional &&
                !answered(steps_[static_cast<std::size_t>(i)])) {
                refreshSteps();
                return;
            }
        }
        showStep(row);
    });
    for (QDoubleSpinBox* s : {minOpening_, maxOpening_, gripDepth_, tenonDepth_, gripRange_}) {
        connect(s, &QDoubleSpinBox::valueChanged, this, [this] { rebuild(); });
    }
    for (QLineEdit* e : {name_, description_}) {
        connect(e, &QLineEdit::textEdited, this, [this] { rebuild(); });
    }
    connect(anchor_, &QComboBox::currentIndexChanged, this, [this] { rebuild(); });
    connect(trial_, &QCheckBox::toggled, this, [this] { rebuild(); });
    for (QDoubleSpinBox* s : trialSize_) {
        connect(s, &QDoubleSpinBox::valueChanged, this, [this] { rebuild(); });
    }
    connect(opening_, &QSlider::valueChanged, this, [this](int v) {
        if (!built_ || trial_->isChecked()) {
            return;
        }
        try {
            travel_ = document::travelsForOpening(*built_, built_->holdings.front(), v / 10.0);
            openingValue_->setText(tr("%1 mm").arg(v / 10.0, 0, 'f', 1));
            updateView();
        } catch (const Error& e) {
            problem_->setText(QString::fromUtf8(e.what()));
        }
    });

    refreshParts();
    buildSteps();
    showStep(0);
}

void WorkholdingWizard::refreshParts() {
    const std::vector<Uuid> selected = selectedParts();
    {
        const QSignalBlocker block(parts_);
        parts_->clear();
        for (const WorkholdingPart& p : start_.workholding.parts) {
            auto* item = new QListWidgetItem(qs(p.name));
            item->setData(Qt::UserRole, QString::fromStdString(p.id.toString()));
            parts_->addItem(item);
            item->setSelected(std::find(selected.begin(), selected.end(), p.id) != selected.end());
        }
    }
    const std::size_t n = selectedParts().size();
    removePart_->setEnabled(n > 0 && n < start_.workholding.parts.size());
    mergeParts_->setEnabled(n > 1);
}

std::vector<Uuid> WorkholdingWizard::selectedParts() const {
    std::vector<Uuid> out;
    for (int r = 0; r < parts_->count(); ++r) {
        if (parts_->item(r)->isSelected()) {
            if (const auto id =
                    Uuid::parse(parts_->item(r)->data(Qt::UserRole).toString().toStdString())) {
                out.push_back(*id);
            }
        }
    }
    return out;
}

void WorkholdingWizard::removeParts(const std::vector<Uuid>& ids) {
    if (ids.empty() || ids.size() >= start_.workholding.parts.size()) {
        return; // a fixture keeps at least one part
    }
    for (const Uuid& id : ids) {
        document::removeWorkholdingPart(start_, id);
    }
    forgetParts(ids, std::nullopt);
}

void WorkholdingWizard::mergeParts(const std::vector<Uuid>& ids) {
    if (ids.size() < 2) {
        return;
    }
    QStringList names;
    for (const Uuid& id : ids) {
        if (const WorkholdingPart* p = partById(id)) {
            names << qs(p->name);
        }
    }
    try {
        const WorkholdingPart merged = document::mergeWorkholdingParts(
            start_, ids, names.join(QStringLiteral(" + ")).toStdString(),
            [this](const WorkholdingPart& p) { return modelOf(p); });
        // Read back from its file: the sub-shape numbers picks refer to.
        CancelToken cancel;
        NullProgress progress;
        models_.emplace(merged.sha256, cad::Model::importStepData(start_.files.at(merged.sha256),
                                                                  merged.name, cancel, progress));
        forgetParts(ids, merged.id);
    } catch (const Error& e) {
        problem_->setText(QString::fromUtf8(e.what()));
    }
}

void WorkholdingWizard::forgetParts(const std::vector<Uuid>& gone,
                                    const std::optional<Uuid>& merged) {
    auto isGone = [&](const Uuid& id) {
        return std::find(gone.begin(), gone.end(), id) != gone.end();
    };
    for (std::vector<Uuid>* list : {&answers_.moving, &answers_.opposite}) {
        const bool had = std::any_of(list->begin(), list->end(), isGone);
        std::erase_if(*list, isGone);
        if (had && merged) {
            list->push_back(*merged);
        }
    }
    for (std::optional<WorkholdingPick>* pick : {&answers_.axis, &answers_.faceA, &answers_.faceB,
                                                 &answers_.top, &answers_.xEdge, &answers_.mount}) {
        if (*pick && isGone((*pick)->part)) {
            pick->reset(); // its face numbers belonged to the old part
        }
    }
    built_.reset();
    base_.reset();
    travel_.clear();
    refreshParts();
    showStep(current_);
}

const WorkholdingPart* WorkholdingWizard::partByName(const std::string& name) const {
    for (const auto& p : start_.workholding.parts) {
        if (p.name == name) {
            return &p;
        }
    }
    return nullptr;
}

const WorkholdingPart* WorkholdingWizard::partById(const Uuid& id) const {
    for (const auto& p : start_.workholding.parts) {
        if (p.id == id) {
            return &p;
        }
    }
    return nullptr;
}

const cad::Model* WorkholdingWizard::modelOf(const WorkholdingPart& part) const {
    const auto it = models_.find(part.sha256);
    return it == models_.end() ? nullptr : &it->second;
}

void WorkholdingWizard::setType(FixtureType type) {
    if (type != answers_.type) {
        const std::string name = answers_.name;
        const std::string description = answers_.description;
        answers_ = {};
        answers_.type = type;
        answers_.name = name;
        answers_.description = description;
        defaultsSet_ = false;
        built_.reset();
        base_.reset();
        travel_.clear();
    }
    if (QAbstractButton* b = types_->button(static_cast<int>(type))) {
        const QSignalBlocker block(types_);
        b->setChecked(true);
    }
    buildSteps();
    showStep(0);
}

void WorkholdingWizard::buildSteps() {
    steps_.clear();
    steps_.push_back({Ask::Type, Slot::None, tr("What kind of workholding is it?"),
                      tr("The kind decides how the jaws move. The next questions ask you to click "
                         "the parts and faces in the view."),
                      false});
    QString mount = tr("Click what it stands on");
    QString mountHelp = tr("The face it sits on the machine table with: a setup can snap this "
                           "onto the machine. Skip to use the bottom centre of all the parts.");
    switch (answers_.type) {
    case FixtureType::Vise:
        steps_.push_back({Ask::Parts, Slot::Moving, tr("Click the moving jaw"),
                          tr("Click every part that moves when the vise opens: the jaw and "
                             "anything that slides with it. They turn orange; click one again to "
                             "take it out. Then press Next."),
                          false});
        steps_.push_back({Ask::Face, Slot::FaceA, tr("Click the fixed jaw's clamping face"),
                          tr("The face of the fixed jaw that touches the stock. The stock's side "
                             "rests against it."),
                          false});
        steps_.push_back({Ask::Face, Slot::FaceB, tr("Click the moving jaw's clamping face"),
                          tr("The face of the moving jaw that touches the stock."), false});
        steps_.push_back({Ask::Face, Slot::Top, tr("Click the top of the jaws"),
                          tr("A face at the height of the jaw tops: the stock's bottom sits the "
                             "grip depth below it, and up is this face's normal."),
                          false});
        break;
    case FixtureType::SelfCentringVise:
        steps_.push_back({Ask::Parts, Slot::Opposite, tr("Click the first jaw"),
                          tr("Click every part of one jaw (they turn orange; click again to take "
                             "one out), then press Next."),
                          false});
        steps_.push_back({Ask::Parts, Slot::Moving, tr("Click the second jaw"),
                          tr("Click every part of the other jaw. The two jaws will move together, "
                             "keeping the stock centred."),
                          false});
        steps_.push_back({Ask::Face, Slot::FaceA, tr("Click the first jaw's clamping face"),
                          tr("The face of the first jaw that touches the stock."), false});
        steps_.push_back({Ask::Face, Slot::FaceB, tr("Click the second jaw's clamping face"),
                          tr("The face of the second jaw that touches the stock."), false});
        steps_.push_back({Ask::Face, Slot::Top, tr("Click the top of the jaws"),
                          tr("A face at the height of the jaw tops: the stock's bottom sits the "
                             "grip depth below it."),
                          false});
        break;
    case FixtureType::Dovetail:
        steps_.push_back({Ask::Parts, Slot::Moving, tr("Click the moving flank"),
                          tr("Click the parts that move to clamp the tenon. Skip when both "
                             "flanks are fixed."),
                          true});
        steps_.push_back({Ask::Face, Slot::FaceA, tr("Click the fixed flank"),
                          tr("The leaning face of the fixed side of the dovetail."), false});
        steps_.push_back({Ask::Face, Slot::FaceB, tr("Click the other flank"),
                          tr("The leaning face of the other side."), false});
        steps_.push_back({Ask::Face, Slot::Top, tr("Click the seat"),
                          tr("The face the stock's shoulder sits on, at the top of the flanks."),
                          false});
        break;
    case FixtureType::Seat:
        steps_.push_back({Ask::Face, Slot::Top, tr("Click the face the stock sits on"),
                          tr("The stock's bottom goes onto this face."), false});
        steps_.push_back({Ask::Edge, Slot::XEdge, tr("Click an edge along the stock's X"),
                          tr("A straight edge the stock's X axis runs along. Skip to use the "
                             "parts' X axis."),
                          true});
        break;
    case FixtureType::Chuck:
        steps_.push_back({Ask::Parts, Slot::Moving, tr("Click every jaw"),
                          tr("Click every part of all the jaws (they turn orange; click again to "
                             "take one out), then press Next. Parts at the same angle about the "
                             "axis move as one jaw; all the jaws move together."),
                          false});
        steps_.push_back({Ask::Face, Slot::Axis, tr("Click a round face on the spindle axis"),
                          tr("The chuck body's outside or its bore: its axis is the spindle's."),
                          false});
        steps_.push_back({Ask::Face, Slot::FaceA, tr("Click a jaw's gripping face"),
                          tr("The face of one jaw that touches the bar: its distance from the axis "
                             "is the radius gripped."),
                          false});
        steps_.push_back({Ask::Face, Slot::Top, tr("Click the front of the jaws"),
                          tr("The jaws' front face, or the step the bar's end sits against: the "
                             "bar reaches the grip length behind it."),
                          false});
        break;
    case FixtureType::Collet:
        steps_.push_back({Ask::Face, Slot::Axis, tr("Click the collet's bore"),
                          tr("The bore that grips the bar: its axis is the spindle's, its "
                             "diameter the bar's."),
                          false});
        steps_.push_back({Ask::Face, Slot::Top, tr("Click the front of the collet"),
                          tr("Its front face: the bar reaches the grip length behind it."), false});
        break;
    }
    if (chuck()) {
        mount = tr("Click the back of the chuck");
        mountHelp = tr("The face it mounts on the spindle by. Skip to use the centre of its back.");
    }
    steps_.push_back({Ask::Face, Slot::Mount, mount, mountHelp, true});
    steps_.push_back({Ask::Finish, Slot::None, tr("Name it and try it"),
                      tr("Set the range of openings, then move the slider or close the jaws on a "
                         "trial stock to check it. Finish puts it in the editor, where it can be "
                         "saved to the library."),
                      false});
}

bool WorkholdingWizard::answered(const Step& step) const {
    switch (step.slot) {
    case Slot::None:
        return step.ask == Ask::Type;
    case Slot::Moving:
        return !answers_.moving.empty();
    case Slot::Opposite:
        return !answers_.opposite.empty();
    case Slot::Axis:
        return answers_.axis.has_value();
    case Slot::FaceA:
        return answers_.faceA.has_value();
    case Slot::FaceB:
        return answers_.faceB.has_value();
    case Slot::Top:
        return answers_.top.has_value();
    case Slot::XEdge:
        return answers_.xEdge.has_value();
    case Slot::Mount:
        return answers_.mount.has_value();
    }
    return false;
}

QString WorkholdingWizard::answerText(const Step& step) const {
    auto pickText = [this](const std::optional<WorkholdingPick>& p) -> QString {
        if (!p) {
            return {};
        }
        const WorkholdingPart* part = partById(p->part);
        const cad::Model* model = part ? modelOf(*part) : nullptr;
        if (!model) {
            return {};
        }
        try {
            const cad::DatumGeometry g = cad::datumGeometry(*model, p->type, p->index);
            const char* what = g.shape == cad::DatumGeometry::Shape::Plane  ? "a flat face"
                               : g.shape == cad::DatumGeometry::Shape::Axis ? "a round face"
                               : g.shape == cad::DatumGeometry::Shape::Line ? "a straight edge"
                                                                            : "a point";
            return tr("Picked %1 of '%2' at %3").arg(tr(what), qs(part->name), vecText(g.point));
        } catch (const Error&) {
            return {};
        }
    };
    auto partsText = [this](const std::vector<Uuid>& ids) {
        QStringList names;
        for (const Uuid& id : ids) {
            if (const WorkholdingPart* p = partById(id)) {
                names << qs(p->name);
            }
        }
        return names.isEmpty() ? QString() : tr("Moving: %1").arg(names.join(QStringLiteral(", ")));
    };
    switch (step.slot) {
    case Slot::Moving:
        return partsText(answers_.moving);
    case Slot::Opposite:
        return partsText(answers_.opposite);
    case Slot::Axis:
        return pickText(answers_.axis);
    case Slot::FaceA:
        return pickText(answers_.faceA);
    case Slot::FaceB:
        return pickText(answers_.faceB);
    case Slot::Top:
        return pickText(answers_.top);
    case Slot::XEdge:
        return pickText(answers_.xEdge);
    case Slot::Mount:
        return pickText(answers_.mount);
    case Slot::None:
        break;
    }
    return {};
}

void WorkholdingWizard::refreshSteps() {
    const QSignalBlocker block(stepList_);
    stepList_->clear();
    for (std::size_t i = 0; i < steps_.size(); ++i) {
        const Step& s = steps_[i];
        const bool done = answered(s) && s.ask != Ask::Finish;
        auto* item = new QListWidgetItem(
            QStringLiteral("%1  %2").arg(done ? QStringLiteral("✓") : QStringLiteral("•"),
                                         s.title + (s.optional ? tr(" (optional)") : QString())));
        if (static_cast<int>(i) == current_) {
            QFont f = item->font();
            f.setBold(true);
            item->setFont(f);
        }
        stepList_->addItem(item);
    }
    stepList_->setCurrentRow(current_);
}

void WorkholdingWizard::showStep(int index) {
    current_ = std::clamp(index, 0, static_cast<int>(steps_.size()) - 1);
    const Step& step = steps_[static_cast<std::size_t>(current_)];
    title_->setText(QStringLiteral("%1. %2").arg(current_ + 1).arg(step.title));
    help_->setText(step.help);
    typePage_->setVisible(step.ask == Ask::Type);
    finishPage_->setVisible(step.ask == Ask::Finish);
    answer_->setText(answerText(step));
    problem_->clear();
    back_->setEnabled(current_ > 0);
    skip_->setVisible(step.optional && !answered(step) && step.ask != Ask::Finish);
    next_->setVisible(step.ask != Ask::Finish);
    next_->setEnabled(answered(step) || step.optional);
    finish_->setVisible(step.ask == Ask::Finish);
    // The face picked for this question shows selected; parts questions clear the selection.
    std::optional<WorkholdingPick> shown;
    switch (step.slot) {
    case Slot::Axis:
        shown = answers_.axis;
        break;
    case Slot::FaceA:
        shown = answers_.faceA;
        break;
    case Slot::FaceB:
        shown = answers_.faceB;
        break;
    case Slot::Top:
        shown = answers_.top;
        break;
    case Slot::XEdge:
        shown = answers_.xEdge;
        break;
    case Slot::Mount:
        shown = answers_.mount;
        break;
    default:
        break;
    }
    {
        const QSignalBlocker block(view_);
        view_->selectShapes(
            shown ? std::vector<OcctView::PickedShape>{{shown->part, shown->type, shown->index}}
                  : std::vector<OcctView::PickedShape>{});
    }
    if (step.ask == Ask::Finish) {
        const bool seat = answers_.type == FixtureType::Seat;
        const bool moving = !(answers_.type == FixtureType::Dovetail && answers_.moving.empty());
        const bool collet = answers_.type == FixtureType::Collet;
        openingRow_->setVisible(!seat && moving && !collet);
        gripDepth_->setVisible(answers_.type == FixtureType::Vise ||
                               answers_.type == FixtureType::SelfCentringVise || chuck());
        tenonDepth_->setVisible(answers_.type == FixtureType::Dovetail);
        gripRange_->setVisible(collet);
        anchor_->setVisible(seat);
        trial_->setText(chuck() ? tr("Grip a trial bar") : tr("Close on a trial stock"));
        gripDepth_->setToolTip(chuck() ? tr("How far the bar reaches into the jaws behind their "
                                            "front (setups can change it)")
                                       : tr("How far the stock reaches below the jaw tops "
                                            "(setups can change it)"));
        auto* form = qobject_cast<QFormLayout*>(finishPage_->layout());
        for (QWidget* w : std::initializer_list<QWidget*>{openingRow_, gripDepth_, tenonDepth_,
                                                          gripRange_, anchor_}) {
            if (QWidget* label = form->labelForField(w)) {
                label->setVisible(w->isVisibleTo(finishPage_));
            }
        }
        if (auto* label = qobject_cast<QLabel*>(form->labelForField(openingRow_))) {
            label->setText(chuck() ? tr("Diameters gripped") : tr("Opening range"));
        }
        if (auto* label = qobject_cast<QLabel*>(form->labelForField(gripDepth_))) {
            label->setText(chuck() ? tr("Grip length") : tr("Grip depth"));
        }
        // A trial bar has a diameter and a length.
        form->setRowVisible(trialSize_[1], !chuck());
        if (auto* label = qobject_cast<QLabel*>(form->labelForField(trialSize_[0]))) {
            label->setText(chuck() ? tr("Bar diameter") : tr("Stock width"));
        }
        if (auto* label = qobject_cast<QLabel*>(form->labelForField(trialSize_[2]))) {
            label->setText(chuck() ? tr("Bar length") : tr("Stock height"));
        }
        trialSize_[0]->setToolTip(chuck() ? tr("The diameter the jaws close on")
                                          : tr("Across the jaws, from one clamping face to the "
                                               "other"));
        trialSize_[2]->setToolTip(chuck() ? tr("From the bar's end in the jaws to its front")
                                          : tr("From the stock's bottom to its top"));
        setFinishDefaults();
        rebuild();
    } else {
        built_.reset();
        base_.reset();
        travel_.clear();
    }
    refreshSteps();
    updateView();
}

bool WorkholdingWizard::next() {
    const Step& step = steps_[static_cast<std::size_t>(current_)];
    if (step.ask == Ask::Finish) {
        return false;
    }
    if (!answered(step) && !step.optional) {
        problem_->setText(tr("Answer this question first."));
        return false;
    }
    showStep(current_ + 1);
    return true;
}

void WorkholdingWizard::setTrialStock(const std::optional<Vec3>& size) {
    if (size) {
        for (int k = 0; k < 3; ++k) {
            const QSignalBlocker block(trialSize_[k]);
            trialSize_[k]->setValue((*size)[k]);
        }
    }
    trial_->setChecked(size.has_value()); // rebuilds
    rebuild();
}

QString WorkholdingWizard::stepTitle() const {
    return steps_[static_cast<std::size_t>(current_)].title;
}

QString WorkholdingWizard::problemText() const {
    return problem_->text();
}

void WorkholdingWizard::picked() {
    const std::vector<OcctView::PickedShape> shapes = view_->selectedShapes();
    if (shapes.empty()) {
        return;
    }
    const OcctView::PickedShape s = shapes.front();
    const WorkholdingPart* part = partById(s.model);
    if (!part) {
        return;
    }
    const Step& step = steps_[static_cast<std::size_t>(current_)];
    if (step.ask == Ask::Parts) {
        answerPart(*part);
    } else if (step.ask == Ask::Face || step.ask == Ask::Edge) {
        answerShape(*part, s.type, s.index);
    }
}

void WorkholdingWizard::answerPart(const WorkholdingPart& part) {
    const Step& step = steps_[static_cast<std::size_t>(current_)];
    std::vector<Uuid>& list = step.slot == Slot::Opposite ? answers_.opposite : answers_.moving;
    std::vector<Uuid>& other = step.slot == Slot::Opposite ? answers_.moving : answers_.opposite;
    const auto it = std::find(list.begin(), list.end(), part.id);
    if (it != list.end()) {
        list.erase(it);
    } else {
        list.push_back(part.id);
        std::erase(other, part.id); // a part moves with one jaw
    }
    {
        const QSignalBlocker block(view_);
        view_->selectShapes({});
    }
    answer_->setText(answerText(step));
    next_->setEnabled(answered(step) || step.optional);
    skip_->setVisible(step.optional && !answered(step));
    problem_->clear();
    refreshSteps();
    updateView();
}

void WorkholdingWizard::answerShape(const WorkholdingPart& part, cad::SubShapeType type,
                                    int index) {
    const Step& step = steps_[static_cast<std::size_t>(current_)];
    if (step.ask == Ask::Face && type != cad::SubShapeType::Face) {
        problem_->setText(tr("Click a face (not an edge or a corner)."));
        return;
    }
    if (step.ask == Ask::Edge && type != cad::SubShapeType::Edge) {
        problem_->setText(tr("Click a straight edge."));
        return;
    }
    const WorkholdingPick pick{part.id, type, index, std::nullopt};
    switch (step.slot) {
    case Slot::Axis:
        answers_.axis = pick;
        if (answers_.type == FixtureType::Collet) {
            answers_.faceA = pick; // the bore grips too
        }
        break;
    case Slot::FaceA:
        answers_.faceA = pick;
        break;
    case Slot::FaceB:
        answers_.faceB = pick;
        break;
    case Slot::Top:
        answers_.top = pick;
        break;
    case Slot::XEdge:
        answers_.xEdge = pick;
        break;
    case Slot::Mount:
        answers_.mount = pick;
        break;
    default:
        return;
    }
    problem_->clear();
    answer_->setText(answerText(step));
    next_->setEnabled(true);
    skip_->setVisible(false);
    refreshSteps();
    updateView();
    // On to the next question after a moment (the pick shows first).
    const int at = current_;
    QTimer::singleShot(400, this, [this, at] {
        if (current_ == at) {
            showStep(at + 1);
        }
    });
}

void WorkholdingWizard::setFinishDefaults() {
    if (defaultsSet_) {
        return;
    }
    defaultsSet_ = true;
    // One rebuild after the defaults (showStep), not one per value set.
    const QSignalBlocker blockMin(minOpening_);
    const QSignalBlocker blockMax(maxOpening_);
    const QSignalBlocker blockGrip(gripDepth_);
    const QSignalBlocker blockAcross(trialSize_[0]);
    const QSignalBlocker blockHigh(trialSize_[2]);
    if (name_->text().isEmpty()) {
        name_->setText(qs(answers_.name));
    }
    description_->setText(qs(answers_.description));
    // The gap between the clamping faces as modelled, for the range and the trial stock.
    double gap = 50.0;
    auto geometry =
        [this](const std::optional<WorkholdingPick>& p) -> std::optional<cad::DatumGeometry> {
        const WorkholdingPart* part = p ? partById(p->part) : nullptr;
        const cad::Model* model = part ? modelOf(*part) : nullptr;
        if (!model) {
            return std::nullopt;
        }
        try {
            return cad::datumGeometry(*model, p->type, p->index);
        } catch (const Error&) {
            return std::nullopt;
        }
    };
    const auto a = geometry(answers_.faceA);
    const auto b = geometry(answers_.faceB);
    const auto top = geometry(answers_.top);
    if (chuck()) {
        // The diameter gripped as modelled: twice the gripping face's distance from the axis.
        const WorkholdingPart* part = answers_.faceA ? partById(answers_.faceA->part) : nullptr;
        const cad::Model* model = part ? modelOf(*part) : nullptr;
        const auto axis = geometry(answers_.axis);
        if (model && axis && axis->direction.norm() > 1e-9) {
            try {
                const cad::SubShapeGeometry g =
                    cad::subShapeGeometry(*model, answers_.faceA->type, answers_.faceA->index);
                const Vec3 d = axis->direction.normalized();
                Vec3 r = g.point - axis->point;
                r -= r.dot(d) * d;
                gap = std::max(
                    1.0,
                    2.0 * (g.kind == cad::SubShapeGeometry::Kind::Cylinder ? g.radius : r.norm()));
            } catch (const Error&) {
            }
        }
        minOpening_->setValue(0.0);
        maxOpening_->setValue(std::max(100.0, std::ceil(2.0 * gap / 10.0) * 10.0));
        gripDepth_->setValue(answers_.type == FixtureType::Collet ? 20.0 : 15.0);
        trialSize_[0]->setValue(std::round(gap * 10.0) / 10.0);
        trialSize_[2]->setValue(80.0);
        return;
    }
    if (a && b) {
        const Vec3 up = top && top->direction.norm() > 1e-9 ? Vec3(top->direction.normalized())
                                                            : Vec3(Vec3::UnitZ());
        Vec3 d = b->point - a->point;
        d -= d.dot(up) * up;
        gap = std::max(1.0, d.norm());
    }
    const bool dovetail = answers_.type == FixtureType::Dovetail;
    minOpening_->setValue(dovetail ? std::max(0.0, std::floor(gap - 10.0)) : 0.0);
    maxOpening_->setValue(dovetail ? std::ceil(gap + 20.0)
                                   : std::max(100.0, std::ceil(2.0 * gap / 10.0) * 10.0));
    trialSize_[0]->setValue(std::round(gap));
}

void WorkholdingWizard::rebuild() {
    answers_.name = name_->text().trimmed().toStdString();
    answers_.description = description_->text().trimmed().toStdString();
    answers_.minOpening = minOpening_->value();
    answers_.maxOpening = maxOpening_->value();
    answers_.gripDepth = gripDepth_->value();
    answers_.tenonDepth = tenonDepth_->value();
    answers_.gripRange = gripRange_->value();
    const QStringList anchor = anchor_->currentData().toString().split(QLatin1Char(','));
    for (int k = 0; k < 3 && k < anchor.size(); ++k) {
        answers_.anchor[static_cast<std::size_t>(k)] = anchor[k].toInt();
    }
    problem_->clear();
    try {
        // The clicked answers make the template once (picking datums on the exact geometry);
        // the values typed on this page go onto a copy, as buildWorkholding would set them.
        if (!base_) {
            document::WorkholdingAnswers clicks = answers_;
            clicks.minOpening = 0.0;
            clicks.maxOpening = 1.0;
            base_ =
                document::buildWorkholding(clicks, start_.workholding.parts,
                                           [this](const WorkholdingPart& p) { return modelOf(p); });
        }
        document::WorkholdingTemplate w = *base_;
        w.name = answers_.name.empty() ? std::string("Workholding") : answers_.name;
        w.description = answers_.description;
        document::StockHolding& h = w.holdings.front();
        switch (h.kind) {
        case document::HoldingKind::Jaws:
        case document::HoldingKind::Dovetail:
            h.gripDepth = answers_.gripDepth;
            h.depth = answers_.tenonDepth;
            break;
        case document::HoldingKind::Chuck:
            h.gripDepth = answers_.gripDepth;
            h.gripRange = answers_.type == FixtureType::Collet ? answers_.gripRange : 0.0;
            break;
        case document::HoldingKind::Seat:
            h.anchor = answers_.anchor;
            break;
        }
        if (!w.members.empty()) {
            if (!(answers_.maxOpening > answers_.minOpening)) {
                throw Error(ErrorCode::InvalidArgument,
                            "the largest opening must be larger than the smallest");
            }
            document::setOpeningRange(w, h, answers_.minOpening, answers_.maxOpening);
        }
        built_ = std::move(w);
    } catch (const Error& e) {
        built_.reset();
        travel_.clear();
        problem_->setText(QString::fromUtf8(e.what()));
        finish_->setEnabled(false);
        updateView();
        return;
    }
    finish_->setEnabled(true);
    const document::StockHolding& h = built_->holdings.front();
    const auto range =
        h.kind == document::HoldingKind::Seat ? std::nullopt : document::openingRange(*built_, h);
    opening_->setEnabled(range.has_value() && !trial_->isChecked());
    if (range) {
        const QSignalBlocker block(opening_);
        opening_->setRange(static_cast<int>(std::floor(range->first * 10.0)),
                           static_cast<int>(std::ceil(range->second * 10.0)));
    }
    if (trial_->isChecked()) {
        // The jaws closed on a trial stock (its X across the jaws, its top at Z 0).
        document::SetupWorkholding placement;
        placement.templateId = built_->id;
        const geom3d::Aabb3 stock = trialStock();
        try {
            const document::WorkholdingPose pose =
                document::poseWorkholding(*built_, placement, stock);
            travel_ = pose.travel;
            if (range) {
                const QSignalBlocker block(opening_);
                opening_->setValue(static_cast<int>(std::round(pose.opening * 10.0)));
            }
            openingValue_->setText(tr("%1 mm").arg(pose.opening, 0, 'f', 1));
            for (const auto& w : pose.warnings) {
                problem_->setText(qs(w));
            }
        } catch (const Error& e) {
            problem_->setText(QString::fromUtf8(e.what()));
        }
    } else if (range) {
        try {
            const double now = document::holdingOpening(*built_, h, travel_);
            const double v = std::clamp(now, range->first, range->second);
            travel_ = document::travelsForOpening(*built_, h, v);
            const QSignalBlocker block(opening_);
            opening_->setValue(static_cast<int>(std::round(v * 10.0)));
            openingValue_->setText(tr("%1 mm").arg(v, 0, 'f', 1));
        } catch (const Error& e) {
            problem_->setText(QString::fromUtf8(e.what()));
        }
    } else {
        openingValue_->clear();
    }
    updateView();
}

void WorkholdingWizard::updateView() {
    const Step& step = steps_[static_cast<std::size_t>(current_)];
    auto memberOf = [this](const WorkholdingPart& p) -> std::string {
        if (built_) {
            if (const WorkholdingPart* q = built_->findPart(p.id)) {
                return q->member;
            }
        }
        return {};
    };
    auto moved = [&](const std::string& member) {
        Transform3 t = Transform3::Identity();
        if (built_ && !member.empty()) {
            if (const auto* m = built_->findMember(member); m && travel_.contains(member)) {
                t.translation() = m->direction.normalized() * travel_.at(member);
            }
        }
        return t;
    };
    std::vector<OcctView::ViewModel> models;
    for (const WorkholdingPart& p : start_.workholding.parts) {
        const cad::Model* m = modelOf(p);
        if (!m) {
            continue;
        }
        const auto in = [&](const std::vector<Uuid>& ids) {
            return std::find(ids.begin(), ids.end(), p.id) != ids.end();
        };
        const bool asked = (step.slot == Slot::Moving && in(answers_.moving)) ||
                           (step.slot == Slot::Opposite && in(answers_.opposite));
        const bool answeredElsewhere = in(answers_.moving) || in(answers_.opposite);
        OcctView::ViewModel vm;
        vm.id = p.id;
        vm.model = *m;
        vm.placement = moved(memberOf(p));
        const std::vector<Uuid> listed = selectedParts();
        vm.colour = in(listed)          ? kListed
                    : asked             ? kAsked
                    : answeredElsewhere ? kAnswered
                                        : p.colour;
        models.push_back(std::move(vm));
    }
    view_->setModels(models);
    // The picked faces and edges (a cross and their direction), and the trial stock.
    OcctView::MachineScene scene;
    for (const auto* pick : {&answers_.axis, &answers_.faceA, &answers_.faceB, &answers_.top,
                             &answers_.xEdge, &answers_.mount}) {
        if (!*pick) {
            continue;
        }
        const WorkholdingPart* part = partById((*pick)->part);
        const cad::Model* model = part ? modelOf(*part) : nullptr;
        if (!model) {
            continue;
        }
        try {
            // Worked out once per pick: the view is redrawn on every slider step.
            const std::string key = part->id.toString() + "/" +
                                    std::to_string(static_cast<int>((*pick)->type)) + "/" +
                                    std::to_string((*pick)->index);
            auto known = pickGeometry_.find(key);
            if (known == pickGeometry_.end()) {
                known = pickGeometry_
                            .emplace(key, cad::datumGeometry(*model, (*pick)->type, (*pick)->index))
                            .first;
            }
            const cad::DatumGeometry& g = known->second;
            const Transform3 at = moved(memberOf(*part));
            const Vec3 p = at * g.point;
            for (const Vec3& axis : {Vec3::UnitX(), Vec3::UnitY(), Vec3::UnitZ()}) {
                scene.lines.push_back({p - 4.0 * axis, p + 4.0 * axis, false, kDatum});
            }
            if (!g.direction.isZero()) {
                scene.lines.push_back({p, p + 25.0 * (at.linear() * g.direction), false, kDatum});
            }
        } catch (const Error&) {
        }
    }
    if (built_ && trial_->isChecked() && step.ask == Ask::Finish) {
        document::SetupWorkholding placement;
        placement.templateId = built_->id;
        const geom3d::Aabb3 stock = trialStock();
        try {
            const Transform3 toTemplate =
                document::poseWorkholding(*built_, placement, stock).templateToWcs.inverse();
            Vec3 c[8];
            for (int i = 0; i < 8; ++i) {
                c[i] = toTemplate * Vec3((i & 1) ? stock.max.x() : stock.min.x(),
                                         (i & 2) ? stock.max.y() : stock.min.y(),
                                         (i & 4) ? stock.max.z() : stock.min.z());
            }
            for (int i = 0; i < 8; ++i) {
                for (const int bit : {1, 2, 4}) {
                    if (!(i & bit)) {
                        scene.lines.push_back({c[i], c[i | bit], true, kStock});
                    }
                }
            }
        } catch (const Error&) {
            // the problem line says why
        }
    }
    view_->setMachine(scene);
    if (!fitted_ && !models.empty()) {
        view_->fitAll();
        fitted_ = true;
    }
}

bool WorkholdingWizard::chuck() const noexcept {
    return answers_.type == FixtureType::Chuck || answers_.type == FixtureType::Collet;
}

geom3d::Aabb3 WorkholdingWizard::trialStock() const {
    if (chuck()) {
        const double r = trialSize_[0]->value() / 2.0;
        return {Vec3(-r, -r, -trialSize_[2]->value()), Vec3(r, r, 0)};
    }
    return {Vec3(0, 0, -trialSize_[2]->value()),
            Vec3(trialSize_[0]->value(), trialSize_[1]->value(), 0)};
}

bool WorkholdingWizard::removePart(const std::string& partName) {
    const WorkholdingPart* part = partByName(partName);
    if (!part || start_.workholding.parts.size() < 2) {
        return false;
    }
    removeParts({part->id});
    return true;
}

bool WorkholdingWizard::mergeParts(const std::vector<std::string>& partNames) {
    std::vector<Uuid> ids;
    for (const std::string& name : partNames) {
        const WorkholdingPart* part = partByName(name);
        if (!part) {
            return false;
        }
        ids.push_back(part->id);
    }
    const std::size_t before = start_.workholding.parts.size();
    mergeParts(ids);
    return start_.workholding.parts.size() + ids.size() - 1 == before;
}

std::vector<std::string> WorkholdingWizard::partNames() const {
    std::vector<std::string> out;
    for (const WorkholdingPart& p : start_.workholding.parts) {
        out.push_back(p.name);
    }
    return out;
}

const cad::Model* WorkholdingWizard::partModel(const std::string& partName) const {
    const WorkholdingPart* part = partByName(partName);
    return part ? modelOf(*part) : nullptr;
}

bool WorkholdingWizard::clickPart(const std::string& partName) {
    const WorkholdingPart* part = partByName(partName);
    if (!part || steps_[static_cast<std::size_t>(current_)].ask != Ask::Parts) {
        return false;
    }
    answerPart(*part);
    return true;
}

bool WorkholdingWizard::clickShape(const std::string& partName, cad::SubShapeType type, int index) {
    const WorkholdingPart* part = partByName(partName);
    const Ask ask = steps_[static_cast<std::size_t>(current_)].ask;
    if (!part || (ask != Ask::Face && ask != Ask::Edge)) {
        return false;
    }
    answerShape(*part, type, index);
    return answered(steps_[static_cast<std::size_t>(current_)]);
}

bool WorkholdingWizard::finish() {
    if (steps_[static_cast<std::size_t>(current_)].ask != Ask::Finish) {
        showStep(static_cast<int>(steps_.size()) - 1);
    }
    rebuild();
    if (!built_) {
        return false;
    }
    result_.workholding = *built_;
    result_.workholding.id = start_.workholding.id;
    result_.files.clear();
    for (const auto& p : result_.workholding.parts) {
        if (const auto it = start_.files.find(p.sha256); it != start_.files.end()) {
            result_.files.emplace(p.sha256, it->second);
        }
    }
    accept();
    return true;
}

QImage WorkholdingWizard::snapshot() {
    QImage image = grab().toImage();
    const QString file = QDir::temp().filePath(QStringLiteral("oxcam-workholding-wizard.png"));
    if (view_->dumpImage(file)) {
        const QImage view(file);
        QPainter painter(&image);
        const QPoint at = view_->mapTo(this, QPoint(0, 0));
        painter.drawImage(QRectF(at.x(), at.y(), view_->width(), view_->height()), view);
        QFile::remove(file);
    }
    return image;
}

} // namespace ocam::gui
