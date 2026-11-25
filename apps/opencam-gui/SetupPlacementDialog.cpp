// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "SetupPlacementDialog.hpp"

#include "OcctView.hpp"
#include "Units.hpp"

#include <ocam/cad/Tessellation.hpp>
#include <ocam/document/Generate.hpp>
#include <ocam/document/MachineImport.hpp>
#include <ocam/document/PostSetup.hpp>
#include <ocam/document/Simulate.hpp>
#include <ocam/document/Workholding.hpp>
#include <ocam/foundation/Error.hpp>
#include <ocam/geom3d/Bvh.hpp>

#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QPushButton>
#include <QRadioButton>
#include <QSplitter>
#include <QVBoxLayout>
#include <cmath>

namespace ocam::gui {
namespace {

QString qs(const std::string& s) {
    return QString::fromStdString(s);
}

std::string ss(const QString& s) {
    return s.trimmed().toStdString();
}

QString pointText(const Vec3& v, LengthUnit unit) {
    // No "-0.000" for the rounding of exact geometry.
    auto c = [](double x) {
        return std::abs(x) < 5e-4 ? 0.0 : x;
    };
    return QStringLiteral("(%1, %2, %3)")
        .arg(formatLength(c(v.x()), unit, false), formatLength(c(v.y()), unit, false),
             formatLength(c(v.z()), unit));
}

QString carrierText(const std::string& carrier) {
    return carrier.empty() ? QObject::tr("the machine base")
                           : QObject::tr("the %1 axis").arg(qs(carrier));
}

constexpr std::uint32_t kPartColour = 0xD9A659;
constexpr std::uint32_t kStockColour = 0xE8D9B8;
constexpr std::uint32_t kFixtureColour = 0x6B809E;

} // namespace

SetupPlacementDialog::SetupPlacementDialog(const document::Document& doc, const Uuid& setupId,
                                           LengthUnit unit, QWidget* parent)
    : QDialog(parent), unit_(unit) {
    doc_ = &doc;
    const document::Project& project = doc.project();
    if (const document::Setup* s = project.findSetup(setupId)) {
        setup_ = *s;
    }
    machine_ = document::machineFor(project, setup_);
    model_ = machine::effectiveModel(machine_);
    placement_ = setup_.placement;
    setWindowTitle(tr("Place %1 on %2").arg(qs(setup_.name), qs(machine_.name)));
    setWindowFlag(Qt::WindowMaximizeButtonHint, true);

    // The machine at machine zero (STEP parts as their exact geometry), then the work in the
    // WCS (the part, fixtures and workholding exact, the stock as its mesh).
    parts_ = std::make_shared<std::vector<machine::ModelPart>>(model_.parts);
    for (const machine::ModelPart& p : model_.parts) {
        cad::Model solid;
        try {
            solid = document::partModel(p);
        } catch (const Error&) {
            // Its mesh is shown and picked instead.
        }
        solids_.push_back(solid);
    }
    for (auto& p : machine::componentParts(machine_, model_)) {
        parts_->push_back(std::move(p));
        solids_.emplace_back();
    }
    machineParts_ = parts_->size();
    auto addWork = [&](std::string name, const cad::Model& solid, geom3d::Mesh mesh,
                       std::uint32_t colour) {
        if (mesh.isEmpty()) {
            return;
        }
        machine::ModelPart p;
        p.name = std::move(name);
        p.colour = colour;
        p.geometry = std::make_shared<const geom3d::Mesh>(std::move(mesh));
        parts_->push_back(std::move(p));
        solids_.push_back(solid);
    };
    auto addBody = [&](std::string name, const cad::Model& model, const Transform3& toWcs,
                       std::uint32_t colour) {
        try {
            const cad::Model inWcs = cad::transformed(model, toWcs);
            addWork(std::move(name), inWcs, cad::tessellate(inWcs, {0.1, 0.35}), colour);
        } catch (const Error&) {
            // Shown without it.
        }
    };
    bool hasPart = false;
    if (setup_.modelId) {
        const document::ModelEntry* entry = project.findModel(*setup_.modelId);
        const cad::Model* model = doc.model(*setup_.modelId);
        if (entry && model) {
            addBody(ss(tr("the part")), *model, document::modelToWcs(*entry, setup_), kPartColour);
            hasPart = parts_->size() > machineParts_;
        }
    }
    stockPart_ = parts_->size();
    if (auto stock = document::stockMeshInWcs(doc, setup_)) {
        addWork(ss(tr("the stock")), {}, std::move(*stock), kStockColour);
    } else {
        stockPart_ = parts_->size() + 1;
    }
    for (const Uuid& id : setup_.fixtures) {
        const document::ModelEntry* entry = project.findModel(id);
        const cad::Model* model = doc.model(id);
        if (entry && model) {
            addBody(entry->name, *model, document::modelToWcs(*entry, setup_), kFixtureColour);
        }
    }
    try {
        if (const auto pose = document::shownWorkholdingPose(doc, setup_)) {
            for (const document::PosedPart& body : pose->parts) {
                if (const cad::Model* model = document::posedPartModel(doc, body)) {
                    addBody(body.name, *model, body.toWcs, kFixtureColour);
                }
            }
        }
    } catch (const Error&) {
        // Shown without it.
    }

    auto* layout = new QHBoxLayout(this);
    auto* splitter = new QSplitter;
    layout->addWidget(splitter);

    auto* viewPane = new QWidget;
    auto* viewLayout = new QVBoxLayout(viewPane);
    viewLayout->setContentsMargins(0, 0, 0, 0);
    view_ = new OcctView;
    view_->setMinimumSize(420, 380);
    viewLayout->addWidget(view_, 1);
    auto* frameRow = new QHBoxLayout;
    auto* frameWork = new QPushButton(tr("Frame the work"));
    auto* frameAll = new QPushButton(tr("Frame all"));
    frameRow->addWidget(frameWork);
    frameRow->addWidget(frameAll);
    frameRow->addStretch(1);
    viewLayout->addLayout(frameRow);
    connect(frameWork, &QPushButton::clicked, this, [this] { frame(true); });
    connect(frameAll, &QPushButton::clicked, this, [this] { frame(false); });
    note_ = new QLabel(tr("Click a face of the machine to put the work there. To put another point "
                          "of the work there than its WCS origin, click it on the work (orange) "
                          "first."));
    note_->setWordWrap(true);
    note_->setStyleSheet(QStringLiteral("color: #e0a030"));
    viewLayout->addWidget(note_);
    status_ = new QLabel;
    status_->setWordWrap(true);
    viewLayout->addWidget(status_);
    problem_ = new QLabel;
    problem_->setWordWrap(true);
    problem_->setStyleSheet(QStringLiteral("color: #c0392b;"));
    viewLayout->addWidget(problem_);
    splitter->addWidget(viewPane);

    auto* side = new QWidget;
    auto* sideLayout = new QVBoxLayout(side);
    auto* snapBox = new QGroupBox(tr("A click on a face snaps to"));
    auto* snapLayout = new QVBoxLayout(snapBox);
    snapFace_ = new QRadioButton(tr("Its centre (a zero-point plate's or a vise base's top)"));
    snapAxis_ = new QRadioButton(tr("Where the rotary axis carrying it meets it (a rotary "
                                    "table's or a faceplate's centre)"));
    snapPoint_ = new QRadioButton(tr("The point clicked"));
    for (QRadioButton* b : {snapFace_, snapAxis_, snapPoint_}) {
        snapLayout->addWidget(b);
    }
    snapFace_->setChecked(true);
    showStock_ = new QCheckBox(tr("Show the stock"));
    showStock_->setChecked(!hasPart);
    snapLayout->addWidget(showStock_);
    sideLayout->addWidget(snapBox);

    auto* placeBox = new QGroupBox(tr("Placement (machine zero, machine axes)"));
    auto* form = new QFormLayout(placeBox);
    carrier_ = new QComboBox;
    carrier_->addItem(tr("Base (fixed)"), QString());
    for (const machine::ModelAxis& a : model_.axes) {
        carrier_->addItem(tr("%1 axis").arg(qs(a.name)), qs(a.name));
    }
    carrier_->setToolTip(tr("The axis that carries the work: it moves and turns with it. A click "
                            "on a part sets it to the part's."));
    form->addRow(tr("Carried by"), carrier_);
    auto lengths = [&](std::array<LengthSpinBox*, 3>& spins) {
        auto* row = new QHBoxLayout;
        for (std::size_t k = 0; k < 3; ++k) {
            spins[k] = new LengthSpinBox(0.0, unit_);
            row->addWidget(spins[k]);
            connect(spins[k], &LengthSpinBox::mmChanged, this, &SetupPlacementDialog::fieldsEdited);
        }
        return row;
    };
    form->addRow(tr("Machine point"), lengths(target_));
    targetOn_ = new QLabel;
    targetOn_->setWordWrap(true);
    targetOn_->setStyleSheet(QStringLiteral("color: gray"));
    form->addRow(QString(), targetOn_);
    workPoint_ = new QLabel;
    workPoint_->setWordWrap(true);
    auto* useOrigin = new QPushButton(tr("Use the WCS origin"));
    useOrigin->setToolTip(tr("Put the setup's WCS origin on the machine point"));
    auto* useWorkholding = new QPushButton(tr("Use the workholding's mount datum"));
    useWorkholding->setToolTip(
        tr("Put the workholding's mount datum on the machine point: the workholding stays there "
           "and the stock, part and WCS follow it (a larger stock moves the work, not the "
           "fixture)"));
    useWorkholding->setVisible(setup_.workholding.has_value());
    auto* workRow = new QVBoxLayout;
    workRow->addWidget(workPoint_);
    auto* workButtons = new QHBoxLayout;
    workButtons->addWidget(useOrigin);
    workButtons->addWidget(useWorkholding);
    workButtons->addStretch(1);
    workRow->addLayout(workButtons);
    form->addRow(tr("Work point"), workRow);
    form->addRow(tr("Offset"), lengths(offset_));
    auto* rotationRow = new QHBoxLayout;
    for (std::size_t k = 0; k < 3; ++k) {
        rotation_[k] = new QDoubleSpinBox;
        rotation_[k]->setRange(-360.0, 360.0);
        rotation_[k]->setDecimals(3);
        rotation_[k]->setSuffix(QStringLiteral("°"));
        rotation_[k]->setKeyboardTracking(false);
        rotationRow->addWidget(rotation_[k]);
        connect(rotation_[k], &QDoubleSpinBox::valueChanged, this,
                &SetupPlacementDialog::fieldsEdited);
    }
    form->addRow(tr("Rotation X Y Z"), rotationRow);
    auto* help = new QLabel(
        tr("The work point sits on the machine point plus the offset. The rotation turns the work "
           "about machine X, then Y, then Z (0: the part upright, as modelled). Solve, the post "
           "and the simulation use this: rotary positions without tool centre point control are "
           "measured from where the work sits on the rotary axes."));
    help->setWordWrap(true);
    help->setStyleSheet(QStringLiteral("color: gray"));
    form->addRow(help);
    sideLayout->addWidget(placeBox);
    sideLayout->addStretch(1);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    auto* useDefault = buttons->addButton(tr("Default placement"), QDialogButtonBox::ResetRole);
    useDefault->setToolTip(tr("No placement of its own: the part's origin at the table centre, on "
                              "the end of the table chain, upright"));
    sideLayout->addWidget(buttons);
    splitter->addWidget(side);
    splitter->setStretchFactor(0, 3);
    splitter->setStretchFactor(1, 2);
    resize(1200, 720);

    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    connect(useDefault, &QPushButton::clicked, this, [this] {
        placement_.reset();
        refresh();
    });
    connect(useOrigin, &QPushButton::clicked, this, [this] {
        document::SetupPlacement p = current();
        p.workPoint.reset();
        p.workPointOn.clear();
        p.onWorkholding = false;
        placement_ = p;
        refresh();
    });
    connect(useWorkholding, &QPushButton::clicked, this, [this] {
        document::SetupPlacement p = current();
        p.workPoint.reset();
        p.workPointOn.clear();
        p.onWorkholding = true;
        placement_ = p;
        refresh();
    });
    connect(carrier_, &QComboBox::currentIndexChanged, this, &SetupPlacementDialog::fieldsEdited);
    connect(showStock_, &QCheckBox::toggled, this, &SetupPlacementDialog::updateScene);
    connect(view_, &OcctView::machineShapeClicked, this,
            [this](int index, int type, int subIndex, double, double, double) {
                exactPick_ = ExactPick{index, static_cast<cad::SubShapeType>(type), subIndex};
            });
    connect(view_, &OcctView::machinePartClicked, this, &SetupPlacementDialog::clicked);
    refresh();
}

document::SetupPlacement SetupPlacementDialog::current() const {
    return placement_ ? *placement_ : document::defaultPlacement(machine_);
}

document::Setup SetupPlacementDialog::placedSetup() const {
    document::Setup s = setup_;
    s.placement = placement_;
    // On the workholding: its mount datum where the stock puts the workholding.
    try {
        return document::withPlacedWorkholding(*doc_, s);
    } catch (const Error&) {
        return s; // the status line says why
    }
}

machine::Snap SetupPlacementDialog::snapMode() const {
    if (snapAxis_->isChecked()) {
        return machine::Snap::AxisOnFace;
    }
    return snapPoint_->isChecked() ? machine::Snap::Point : machine::Snap::FaceCentre;
}

void SetupPlacementDialog::setSnapMode(machine::Snap snap) {
    (snap == machine::Snap::AxisOnFace ? snapAxis_
     : snap == machine::Snap::Point    ? snapPoint_
                                       : snapFace_)
        ->setChecked(true);
}

void SetupPlacementDialog::refresh() {
    loading_ = true;
    const document::SetupPlacement p = current();
    carrier_->setCurrentIndex(std::max(0, carrier_->findData(qs(p.carrier))));
    for (std::size_t k = 0; k < 3; ++k) {
        target_[k]->setMm(p.target[static_cast<int>(k)]);
        offset_[k]->setMm(p.offset[static_cast<int>(k)]);
        rotation_[k]->setValue(p.rotation[static_cast<int>(k)]);
    }
    targetOn_->setText(placement_
                           ? (p.targetOn.empty() ? tr("Typed") : tr("On %1").arg(qs(p.targetOn)))
                           : tr("The table centre (default)"));
    if (p.onWorkholding) {
        workPoint_->setText(tr("The workholding's mount datum (the stock, part and WCS follow "
                               "the workholding)"));
    } else if (p.workPoint && !(placement_ == std::nullopt && p.workPoint->isZero())) {
        workPoint_->setText(
            tr("%1 (project)%2")
                .arg(pointText(*p.workPoint, unit_),
                     p.workPointOn.empty() ? QString() : tr(", %1").arg(qs(p.workPointOn))));
    } else if (p.workPoint) {
        workPoint_->setText(tr("The project origin (default)"));
    } else {
        workPoint_->setText(tr("The WCS origin"));
    }
    loading_ = false;
    updateScene();
}

void SetupPlacementDialog::fieldsEdited() {
    if (loading_) {
        return;
    }
    document::SetupPlacement p = current();
    const std::string carrier = ss(carrier_->currentData().toString());
    Vec3 target;
    for (std::size_t k = 0; k < 3; ++k) {
        target[static_cast<int>(k)] = target_[k]->mm();
        p.offset[static_cast<int>(k)] = offset_[k]->mm();
        p.rotation[static_cast<int>(k)] = rotation_[k]->value();
    }
    if ((target - p.target).norm() > 1e-9) {
        p.targetOn.clear(); // typed over the snap
    }
    p.target = target;
    p.carrier = carrier;
    placement_ = p;
    refresh();
}

void SetupPlacementDialog::updateScene() {
    const document::Setup placed = placedSetup();
    const machine::WorkMount mount = document::workMount(placed, machine_);
    OcctView::MachineScene scene;
    scene.parts = parts_;
    scene.solids = solids_;
    scene.pickable = true;
    for (std::size_t i = 0; i < parts_->size(); ++i) {
        const machine::ModelPart& p = (*parts_)[i];
        if (i < machineParts_) {
            scene.placements.push_back(p.placement());
            scene.hidden.push_back(!p.shown);
        } else {
            scene.placements.push_back(mount.wcs);
            scene.hidden.push_back(i == stockPart_ && !showStock_->isChecked());
        }
        scene.highlighted.push_back(false);
    }
    geom3d::Aabb3 all;
    for (std::size_t i = 0; i < parts_->size(); ++i) {
        const auto& p = (*parts_)[i];
        if (p.geometry && !scene.hidden[i]) {
            all.extend(geom3d::transformed(p.geometry->bounds(), scene.placements[i]));
        }
    }
    const double length = all.isEmpty() ? 400.0 : std::max(200.0, 0.6 * all.size().norm());
    for (const machine::ModelAxis& a : model_.axes) {
        if (a.rotary() && a.direction.norm() > 1e-9) {
            const Vec3 d = a.direction.normalized() * length;
            scene.lines.push_back({a.pivot - d, a.pivot + d, false});
        }
    }
    scene.frames = {mount.wcs};
    view_->setMachine(scene);
    if (!fitted_) {
        if (all.isEmpty()) {
            view_->fitAll();
        } else {
            view_->fitBox(all);
        }
        fitted_ = true;
    }

    const document::SetupPlacement p = current();
    const QString where =
        tr("The WCS origin is at %1, carried by %2.")
            .arg(pointText(mount.wcs.translation(), unit_), carrierText(p.carrier));
    status_->setText(placement_ ? where
                                : tr("Default placement (none of its own): the part's origin at "
                                     "the table centre, upright. %1")
                                      .arg(where));
    problem_->setText(qs(document::placementProblem(placed, machine_)));
}

void SetupPlacementDialog::clicked(int index, double x, double y, double z) {
    if (index < 0 || index >= static_cast<int>(parts_->size())) {
        return;
    }
    const machine::ModelPart& part = (*parts_)[static_cast<std::size_t>(index)];
    const Vec3 point(x, y, z);
    // The exact geometry under the click, for bodies drawn as B-rep (their own coordinates).
    std::optional<cad::SubShapeGeometry> exact;
    const auto pick = exactPick_;
    exactPick_.reset();
    document::SetupPlacement p = current();
    try {
        const std::size_t i = static_cast<std::size_t>(index);
        if (pick && pick->index == index && i < solids_.size() && !solids_[i].isNull()) {
            exact = cad::subShapeGeometry(solids_[i], pick->type, pick->subIndex);
        }
        if (i < machineParts_) {
            const machine::Snap snap = snapMode();
            p.target = exact ? document::snapOnShape(model_, part, *exact, point, snap)
                             : machine::snapOnPart(model_, part, point, snap);
            if (!placement_) {
                p.workPoint.reset(); // the WCS origin, not the default's project origin
            }
            p.carrier = part.carrier;
            p.targetOn = ss(snap == machine::Snap::FaceCentre
                                ? tr("the centre of a face of '%1'").arg(qs(part.name))
                            : snap == machine::Snap::AxisOnFace
                                ? tr("the rotary axis on a face of '%1'").arg(qs(part.name))
                                : tr("a point of '%1'").arg(qs(part.name)));
            placement_ = p;
            note_->setText(tr("Snapped to %1; the work is carried by %2.")
                               .arg(qs(p.targetOn), carrierText(p.carrier)));
        } else {
            // A point of the work, in the WCS: the work moves to put it on the machine point.
            const machine::WorkMount mount = document::workMount(placedSetup(), machine_);
            const Vec3 atWcs = mount.wcs.inverse() * point;
            machine::ModelPart inWcs = part; // its mesh is in the WCS
            const machine::Snap snap = snapMode() == machine::Snap::Point
                                           ? machine::Snap::Point
                                           : machine::Snap::FaceCentre;
            const Vec3 chosen = exact ? document::snapOnShape(model_, inWcs, *exact, atWcs, snap)
                                      : machine::snapOnPart(model_, inWcs, atWcs, snap);
            p.workPoint = setup_.wcs.toParent() * chosen;
            p.onWorkholding = false;
            p.workPointOn = ss(snap == machine::Snap::FaceCentre
                                   ? tr("the centre of a face of %1").arg(qs(part.name))
                                   : tr("a point of %1").arg(qs(part.name)));
            placement_ = p;
            note_->setText(tr("Work point: %1. Now click where it goes on the machine.")
                               .arg(qs(p.workPointOn)));
        }
    } catch (const Error& e) {
        note_->setText(qs(e.what()));
        return;
    }
    refresh();
}

void SetupPlacementDialog::frame(bool work) {
    const machine::WorkMount mount = document::workMount(placedSetup(), machine_);
    geom3d::Aabb3 box;
    for (std::size_t i = work ? machineParts_ : 0; i < parts_->size(); ++i) {
        const machine::ModelPart& p = (*parts_)[i];
        const bool onWork = i >= machineParts_;
        const bool hidden = onWork ? i == stockPart_ && !showStock_->isChecked() : !p.shown;
        if (p.geometry && !hidden) {
            box.extend(
                geom3d::transformed(p.geometry->bounds(), onWork ? mount.wcs : p.placement()));
        }
    }
    if (box.isEmpty()) {
        view_->fitAll();
    } else {
        view_->fitBox(box);
    }
}

std::string SetupPlacementDialog::clickMachine(const Vec3& pointAtZero, machine::Snap snap) {
    setSnapMode(snap);
    updateScene();
    view_->pickAt(view_->widgetPoint(pointAtZero));
    return ss(note_->text());
}

std::string SetupPlacementDialog::clickWork(const Vec3& pointWcs, machine::Snap snap) {
    setSnapMode(snap);
    updateScene();
    frame(true); // as a user zooms in on a small part
    const machine::WorkMount mount = document::workMount(placedSetup(), machine_);
    view_->pickAt(view_->widgetPoint(mount.wcs * pointWcs));
    frame(false);
    return ss(note_->text());
}

std::string SetupPlacementDialog::statusText() const {
    return ss(status_->text() + QLatin1Char(' ') + problem_->text());
}

QImage SetupPlacementDialog::snapshot() {
    QImage image = grab().toImage();
    const QString file = QDir::temp().filePath(QStringLiteral("oxcam-placement-view.png"));
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
