// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "TurretPreview.hpp"

#include "OcctView.hpp"

#include <ocam/document/Document.hpp>
#include <ocam/document/Generate.hpp>
#include <ocam/document/PostSetup.hpp>
#include <ocam/document/ProjectIo.hpp>
#include <ocam/foundation/Cancel.hpp>
#include <ocam/foundation/Error.hpp>
#include <ocam/geom3d/Bvh.hpp>
#include <ocam/machine/Turret.hpp>
#include <ocam/sim/Surface.hpp>
#include <ocam/toolpath/Toolpath.hpp>

#include <QCheckBox>
#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QSettings>
#include <QShowEvent>
#include <QSpinBox>
#include <QVBoxLayout>
#include <algorithm>
#include <limits>

namespace ocam::gui {

namespace {

QString qs(const std::string& s) {
    return QString::fromStdString(s);
}

/// A profile turned about its axis (u = Z, v = radius), placed by `frame`.
std::shared_ptr<const geom3d::Mesh> turnedMesh(const geom2d::Regions& area,
                                               const Transform3& frame) {
    const sim::DisplayMesh shown = sim::revolvedMesh(area, 0.25);
    auto mesh = std::make_shared<geom3d::Mesh>();
    mesh->vertices.reserve(shown.positions.size());
    for (const auto& p : shown.positions) {
        mesh->vertices.push_back(frame * Vec3(static_cast<double>(p[0]), static_cast<double>(p[1]),
                                              static_cast<double>(p[2])));
    }
    for (std::size_t i = 0; i + 2 < shown.indices.size(); i += 3) {
        mesh->triangles.push_back({shown.indices[i], shown.indices[i + 1], shown.indices[i + 2]});
    }
    return mesh;
}

} // namespace

TurretPreview::TurretPreview(QWidget* parent) : QWidget(parent) {
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(4, 4, 4, 4);
    auto* bar = new QHBoxLayout;
    auto* stationLabel = new QLabel(tr("Station"));
    stationBox_ = new QSpinBox;
    stationBox_->setObjectName(QStringLiteral("turretPreviewStation"));
    stationBox_->setRange(1, 1);
    stationBox_->setToolTip(tr("The station highlighted, the turret turned to it. Choosing a row "
                               "of the setup's turret table chooses it too."));
    stationLabel->setBuddy(stationBox_);
    atBox_ = new QComboBox;
    atBox_->setObjectName(QStringLiteral("turretPreviewAt"));
    atBox_->addItem(tr("At the work"));
    atBox_->addItem(tr("At home"));
    atBox_->setToolTip(tr("At the work: the station's tool touches the front of the stock, at its "
                          "largest diameter (a tool along the spindle, at its centre), to see "
                          "what else comes close there. At home: the axes at machine zero."));
    wholeMachine_ = new QCheckBox(tr("Whole machine"));
    wholeMachine_->setObjectName(QStringLiteral("turretPreviewWholeMachine"));
    wholeMachine_->setChecked(true);
    wholeMachine_->setToolTip(tr("Off: only the turret, the spindle and the work"));
    bar->addWidget(stationLabel);
    bar->addWidget(stationBox_);
    bar->addWidget(atBox_);
    enclosure_ = new QCheckBox(tr("Enclosure"));
    enclosure_->setObjectName(QStringLiteral("turretPreviewEnclosure"));
    enclosure_->setToolTip(tr("Show the machine's enclosure (its walls, sheet metal and way "
                              "covers, as the machine library marks them); off: the rest of the "
                              "machine without it. It is still checked for collisions."));
    enclosure_->setChecked(
        QSettings().value(QStringLiteral("turretPreviewEnclosure"), true).toBool());
    bar->addWidget(wholeMachine_);
    bar->addWidget(enclosure_);
    bar->addStretch();
    layout->addLayout(bar);
    view_ = new OcctView(this);
    view_->setMinimumSize(240, 240);
    layout->addWidget(view_, 1);
    note_ = new QLabel;
    note_->setObjectName(QStringLiteral("turretPreviewNote"));
    note_->setWordWrap(true);
    note_->setTextFormat(Qt::PlainText);
    layout->addWidget(note_);

    connect(stationBox_, &QSpinBox::valueChanged, this, [this](int n) {
        if (n != station_) {
            station_ = n;
            redraw();
            emit stationChanged(n);
        }
    });
    connect(atBox_, &QComboBox::currentIndexChanged, this, [this] { redraw(); });
    connect(wholeMachine_, &QCheckBox::toggled, this, [this] { redraw(); });
    connect(enclosure_, &QCheckBox::toggled, this, [this](bool on) {
        QSettings().setValue(QStringLiteral("turretPreviewEnclosure"), on);
        redraw();
    });
    clear(tr("Choose a setup on a lathe with a turret."));
}

TurretPreview::~TurretPreview() = default;

void TurretPreview::clear(const QString& why) {
    key_.clear();
    machine_.reset();
    parts_.reset();
    stations_.clear();
    nearWork_.clear();
    held_.clear();
    workEdge_.reset();
    workCentre_.reset();
    problems_.clear();
    note_->setText(why);
    if (isVisible()) {
        view_->setMachine(std::nullopt);
    }
}

void TurretPreview::showSetup(const document::Document& doc, const std::optional<Uuid>& setupId) {
    const document::Project& project = doc.project();
    const document::Setup* setup = setupId ? project.findSetup(*setupId) : nullptr;
    if (!setup) {
        clear(tr("Choose a setup on a lathe with a turret."));
        return;
    }
    // The whole project: tool numbers, blocks and the stock all count.
    std::string key = setupId->toString() + document::toJson(project).dump();
    if (key == key_) {
        return;
    }
    const machine::Machine& machine = document::machineFor(project, *setup);
    if (!machine.lathe) {
        clear(tr("Setup '%1' is on machine '%2', which has no turret layout: there is no "
                 "turret to show.")
                  .arg(qs(setup->name), qs(machine.name)));
        key_ = std::move(key);
        return;
    }
    std::vector<std::string> problems;
    machine_ = machine;
    model_ = machine::effectiveModel(machine);
    const document::TurretLoad load = document::turretLoad(doc, *setup, machine);
    problems.insert(problems.end(), load.problems.begin(), load.problems.end());
    const std::vector<sim::TurretBlock> blocks =
        document::turretBlocks(doc, *setup, machine, &problems);

    // The work: the setup's stock where the machine holds it.
    geom2d::Regions stock;
    try {
        CancelToken never;
        stock = document::latheGeometry(doc, *setup, never).stock;
    } catch (const Error& e) {
        problems.push_back(std::string("the stock is not shown: ") + e.what());
    }
    try {
        mount_ = document::latheWorkMount(doc, *setup, machine, stock);
    } catch (const Error& e) {
        mount_ = machine::WorkMount{};
        problems.push_back(e.what());
    }
    workEdge_.reset();
    workCentre_.reset();
    geom2d::Aabb2 box;
    for (const geom2d::Region& r : stock) {
        box.extend(r.bounds().min);
        box.extend(r.bounds().max);
    }
    if (!stock.empty()) {
        workEdge_ = Vec3(box.max.y(), 0.0, box.max.x());
        workCentre_ = Vec3(0.0, 0.0, box.max.x());
    }
    const std::vector<sim::TurretTool> tools = document::turretTools(doc, load, mount_);
    const std::vector<sim::LatheWorkBody> bodies =
        document::latheWorkBodies(doc, *setup, machine, mount_, &tailstockW_, &problems);
    document::LatheMachineParts all = document::latheMachineParts(
        machine, model_, blocks, tools,
        document::usesMachineChuck(*setup) || document::onSubSpindle(*setup));
    stations_ = std::move(all.stations);
    toolsFrom_ = all.toolsFrom;
    auto parts = std::make_shared<std::vector<machine::ModelPart>>(std::move(all.parts));
    const std::size_t machineParts = all.blocksFrom;
    if (!stock.empty()) {
        machine::ModelPart p;
        p.name = "Stock";
        p.carrier = mount_.carrier;
        p.colour = 0xC4B296;
        p.geometry = turnedMesh(stock, mount_.wcs);
        parts->push_back(std::move(p));
        stations_.push_back(0);
    }
    for (const sim::LatheWorkBody& body : bodies) {
        machine::ModelPart p;
        p.name = body.name;
        p.carrier = body.carrier;
        p.colour = 0x6E7F99;
        p.geometry = turnedMesh(body.area, body.frame);
        parts->push_back(std::move(p));
        stations_.push_back(0);
    }
    // Near the work: what the turret carries, what turns the work, and the work.
    nearWork_.assign(parts->size(), true);
    for (std::size_t i = 0; i < machineParts; ++i) {
        const std::string& c = (*parts)[i].carrier;
        nearWork_[i] = c == model_.spindleCarrier || c == mount_.carrier ||
                       (*parts)[i].name == machine::kLatheChuckPart;
    }
    parts_ = std::move(parts);

    held_.clear();
    for (const document::TurretStation& s : load.stations) {
        Held h;
        const auto tool = std::find_if(tools.begin(), tools.end(), [&](const sim::TurretTool& t) {
            return t.station == s.station;
        });
        h.tool = tool != tools.end() ? tool->name : s.name;
        h.across = s.across;
        h.tip = s.tipPoint;
        held_[s.station] = h;
    }
    for (const document::StationBlock& b : setup->turretBlocks) {
        if (const document::LatheFitting* f = project.findFitting(b.block)) {
            held_[b.station].block = f->name;
        }
    }
    problems_.clear();
    for (const std::string& p : problems) {
        problems_ += QStringLiteral("\n") + qs(p);
    }
    {
        const QSignalBlocker block(stationBox_);
        stationBox_->setRange(1, std::max(1, machine.toolCapacity));
        station_ = std::clamp(station_, 1, std::max(1, machine.toolCapacity));
        stationBox_->setValue(station_);
    }
    key_ = std::move(key);
    redraw();
}

void TurretPreview::setStation(int station) {
    if (station == station_) {
        return;
    }
    station_ = station;
    {
        const QSignalBlocker block(stationBox_);
        stationBox_->setValue(station);
    }
    redraw();
}

void TurretPreview::setAtWork(bool atWork) {
    atBox_->setCurrentIndex(atWork ? 0 : 1);
}

QString TurretPreview::note() const {
    return note_->text();
}

bool TurretPreview::highlighted(std::size_t part) const {
    // The station's block; its tool keeps its own colours (black, the insert gold), to be seen in
    // the block.
    return stations_[part] == station_ && part < toolsFrom_;
}

std::vector<std::pair<std::string, std::uint32_t>> TurretPreview::partColours() const {
    std::vector<std::pair<std::string, std::uint32_t>> out;
    for (std::size_t i = 0; parts_ && i < parts_->size(); ++i) {
        out.emplace_back((*parts_)[i].name, (*parts_)[i].colour);
    }
    return out;
}

std::vector<std::string> TurretPreview::highlightedParts() const {
    std::vector<std::string> out;
    for (std::size_t i = 0; parts_ && i < parts_->size(); ++i) {
        if (highlighted(i)) {
            out.push_back((*parts_)[i].name);
        }
    }
    return out;
}

bool TurretPreview::drawn(std::size_t part) const {
    const machine::ModelPart& p = (*parts_)[part];
    return p.shown && (wholeMachine_->isChecked() || nearWork_[part]) &&
           (!p.enclosure || enclosure_->isChecked());
}

std::vector<std::string> TurretPreview::shownParts() const {
    std::vector<std::string> out;
    for (std::size_t i = 0; parts_ && i < parts_->size(); ++i) {
        if (drawn(i)) {
            out.push_back((*parts_)[i].name);
        }
    }
    return out;
}

std::optional<Vec3> TurretPreview::tipNow() const {
    const auto it = held_.find(station_);
    if (!machine_ || it == held_.end() || it->second.tool.empty()) {
        return std::nullopt;
    }
    // The tip goes with the slides, as latheAxes moves it.
    machine::AxisPosition slides;
    slides.linear = position_.linear;
    const Vec3 moved =
        machine::poseAt(model_, slides).carried(model_, model_.spindleCarrier).translation();
    const Vec3 still = machine::poseAt(model_, machine::AxisPosition{})
                           .carried(model_, model_.spindleCarrier)
                           .translation();
    return Vec3(it->second.tip + moved - still);
}

std::optional<Vec3> TurretPreview::workPoint() const {
    const auto it = held_.find(station_);
    const std::optional<Vec3>& at =
        it != held_.end() && !it->second.across ? workCentre_ : workEdge_;
    if (!machine_ || !at) {
        return std::nullopt;
    }
    return Vec3(machine::poseAt(model_, position_).carried(model_, mount_.carrier) * mount_.wcs *
                *at);
}

void TurretPreview::showEvent(QShowEvent* event) {
    QWidget::showEvent(event);
    if (drawPending_) {
        redraw();
    }
}

void TurretPreview::redraw() {
    if (!machine_ || !parts_) {
        return;
    }
    // Where the turret stands: the station working, at home or its tool at the work.
    position_ = machine::AxisPosition{};
    position_.t = machine::stationAngle(*machine_, station_);
    position_.w = tailstockW_;
    const auto it = held_.find(station_);
    const bool holds = it != held_.end() && !it->second.tool.empty();
    if (atBox_->currentIndex() == 0) {
        const std::optional<Vec3>& at = holds && !it->second.across ? workCentre_ : workEdge_;
        if (!holds) {
            positionNote_ = tr("No tool on station %1: the turret at home.").arg(station_);
        } else if (!at) {
            positionNote_ = tr("No stock to bring the tool to: the turret at home.");
        } else {
            machine::LatheMotion motion;
            motion.station = station_;
            motion.tip = it->second.tip;
            motion.held = position_;
            if (const auto solved =
                    machine::latheAxes(model_, motion, mount_, toolpath::Toolpath{}, *at)) {
                position_ = *solved;
                position_.t = motion.held.t;
                position_.w = motion.held.w;
                position_.rotary = {};
                positionNote_ =
                    tr("The tool's tip on the front of the stock, at %1.")
                        .arg(it->second.across ? tr("its largest diameter") : tr("its centre"));
            } else {
                positionNote_ = tr("The machine cannot bring the tool to the work: the turret "
                                   "at home.");
            }
        }
    } else {
        positionNote_ = tr("The turret at home (machine zero).");
    }
    // Axes beyond their travel are said, not drawn otherwise.
    for (const char* name : {"X", "Y", "Z"}) {
        const machine::Axis* axis = machine_->axis(name);
        const double v = position_.value(name);
        if (axis && (v < axis->min - 1e-6 || v > axis->max + 1e-6)) {
            positionNote_ +=
                QStringLiteral(" ") +
                tr("%1 %2 is beyond its travel.").arg(QString::fromLatin1(name)).arg(v, 0, 'f', 3);
        }
    }

    QString what = tr("Station %1: ").arg(station_);
    if (it == held_.end() || (it->second.tool.empty() && it->second.block.empty())) {
        what += tr("empty.");
    } else {
        QStringList words;
        if (!it->second.block.empty()) {
            words << tr("block '%1'").arg(qs(it->second.block));
        }
        if (!it->second.tool.empty()) {
            words << qs(it->second.tool);
        }
        what += words.join(QStringLiteral(", ")) + QStringLiteral(".");
    }
    note_->setText(what + QStringLiteral(" ") + positionNote_ + problems_);

    if (!isVisible()) {
        drawPending_ = true; // drawn when shown: never into a hidden window
        return;
    }
    drawPending_ = false;
    const machine::ModelPose pose = machine::poseAt(model_, position_);
    OcctView::MachineScene scene;
    scene.parts = parts_;
    for (std::size_t i = 0; i < parts_->size(); ++i) {
        const machine::ModelPart& p = (*parts_)[i];
        scene.placements.push_back(pose.carried(model_, p.carrier) * p.placement());
        scene.hidden.push_back(!drawn(i));
        scene.highlighted.push_back(highlighted(i));
    }
    view_->setReferenceFrame(Mat3::Identity(), machine_->standing().worldAxes());
    view_->setMachine(scene);
    if (fittedFor_ != machine_->id) {
        fittedFor_ = machine_->id;
        fit();
    }
}

void TurretPreview::fit() {
    // From the door, the spindle on the left (as the simulation looks at a lathe), the turret and
    // the work filling the view.
    view_->lookFromWorld(Vec3(-0.3, -1.0, 0.45));
    const machine::ModelPose pose = machine::poseAt(model_, position_);
    geom3d::Aabb3 box;
    for (std::size_t i = 0; i < parts_->size(); ++i) {
        const machine::ModelPart& p = (*parts_)[i];
        if (p.geometry && nearWork_[i] && p.shown) {
            box.extend(geom3d::transformed(p.geometry->bounds(),
                                           pose.carried(model_, p.carrier) * p.placement()));
        }
    }
    if (box.isEmpty()) {
        view_->fitAll();
    } else {
        view_->fitBox(box);
    }
}

} // namespace ocam::gui
