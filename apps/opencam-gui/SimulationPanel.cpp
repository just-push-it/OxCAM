// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "SimulationPanel.hpp"

#include "OcctView.hpp"
#include "Units.hpp"

#include <ocam/document/LatheMachine.hpp>
#include <ocam/document/SimulationCache.hpp>
#include <ocam/foundation/Error.hpp>
#include <ocam/foundation/Json.hpp>
#include <ocam/machine/Turret.hpp>
#include <ocam/sim/LatheMachine.hpp>
#include <ocam/sim/MachineCheck.hpp>
#include <ocam/toolpath/Toolpath.hpp>

#include <QCheckBox>
#include <QColor>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QProgressBar>
#include <QPushButton>
#include <QSettings>
#include <QSlider>
#include <QStandardPaths>
#include <QThread>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <filesystem>
#include <mutex>
#include <tuple>

namespace ocam::gui {
namespace {

/// The local simulation cache (ADR-0003): whole-setup simulations by input key, on this machine
/// only (never in a project). OXCAM_SIMULATION_CACHE sets another folder; "none" turns the
/// cache off (the GUI tests).
std::filesystem::path simulationCacheDir() {
    if (const char* dir = std::getenv("OXCAM_SIMULATION_CACHE"); dir && *dir) {
        return std::string_view(dir) == "none" ? std::filesystem::path()
                                               : std::filesystem::path(dir);
    }
    const QString base = QStandardPaths::writableLocation(QStandardPaths::CacheLocation);
    if (base.isEmpty()) {
        return {};
    }
    return std::filesystem::path(base.toStdU16String()) / "simulations";
}

/// The cache's size limit: the least recently used simulations go beyond it.
constexpr std::uintmax_t kSimulationCacheBytes = 4ULL * 1024 * 1024 * 1024;

std::filesystem::path simulationCacheFile(const std::string& key) {
    const std::filesystem::path dir = simulationCacheDir();
    return dir.empty() ? dir : dir / (key + ".oxsim");
}

/// A whole setup's simulation: read from the cache file when it holds this key, else simulated
/// and written there (`file` empty: no cache). Runs on a worker thread.
std::shared_ptr<const document::SetupSimulation>
simulateOrRead(const document::Document& doc, const Uuid& setupId,
               const document::SimulationOptions& options, const std::string& key,
               const std::filesystem::path& file, CancelToken& cancel, ProgressSink& progress) {
    namespace fs = std::filesystem;
    const document::Setup* setup = doc.project().findSetup(setupId);
    std::error_code ec;
    if (!file.empty() && setup && fs::exists(file, ec)) {
        try {
            if (auto read = document::decodeSimulation(readFileBytes(file), key, doc, *setup)) {
                fs::last_write_time(file, fs::file_time_type::clock::now(), ec); // recently used
                progress.report(1.0, "Read");
                return std::make_shared<const document::SetupSimulation>(std::move(*read));
            }
        } catch (const std::exception&) {
            // unreadable: simulated again and written over
        }
    }
    auto sim = std::make_shared<const document::SetupSimulation>(
        document::simulateSetup(doc, setupId, options, cancel, progress));
    if (!file.empty()) {
        try {
            fs::create_directories(file.parent_path(), ec);
            writeFileAtomic(file, document::encodeSimulation(*sim, key));
        } catch (const std::exception&) {
            // a full disk or a read-only folder: just not cached
        }
    }
    return sim;
}

} // namespace

/// Whole-setup simulations being worked out, by input key: a second request for one waits for
/// it (showing its progress) instead of working it out again. When the one working it out stops
/// (cancelled or failed), a request waiting works it out itself. Thread-safe.
class InFlightSimulations {
public:
    using Result = std::shared_ptr<const document::SetupSimulation>;
    using Compute = std::function<Result(CancelToken&, ProgressSink&)>;

    Result run(const std::string& key, const Compute& compute, CancelToken& cancel,
               ProgressSink& progress) {
        for (;;) {
            std::shared_ptr<Entry> entry;
            bool mine = false;
            {
                std::lock_guard lock(mutex_);
                auto& slot = entries_[key];
                if (!slot) {
                    slot = std::make_shared<Entry>();
                    mine = true;
                }
                entry = slot;
            }
            if (mine) {
                Forward forward(progress, entry->progress);
                Result result;
                try {
                    result = compute(cancel, forward);
                } catch (...) {
                    finish(key, entry, nullptr);
                    throw;
                }
                finish(key, entry, result);
                return result;
            }
            std::unique_lock lock(entry->mutex);
            while (!entry->done) {
                entry->changed.wait_for(lock, std::chrono::milliseconds(50));
                progress.report(entry->progress.load(), "Simulating");
                cancel.throwIfCancelled();
            }
            if (entry->result) {
                return entry->result;
            }
        }
    }

private:
    struct Entry {
        std::mutex mutex;
        std::condition_variable changed;
        bool done = false;
        Result result;
        std::atomic<double> progress{0.0};
    };
    /// Reports to the caller's progress and to the entry's, for those waiting.
    class Forward final : public ProgressSink {
    public:
        Forward(ProgressSink& parent, std::atomic<double>& value)
            : parent_(parent), value_(value) {}
        void report(double fraction, std::string_view stage) override {
            value_.store(fraction);
            parent_.report(fraction, stage);
        }

    private:
        ProgressSink& parent_;
        std::atomic<double>& value_;
    };
    void finish(const std::string& key, const std::shared_ptr<Entry>& entry, Result result) {
        {
            std::lock_guard lock(mutex_);
            if (auto it = entries_.find(key); it != entries_.end() && it->second == entry) {
                entries_.erase(it);
            }
        }
        std::lock_guard lock(entry->mutex);
        entry->done = true;
        entry->result = std::move(result);
        entry->changed.notify_all();
    }

    std::mutex mutex_;
    std::map<std::string, std::shared_ptr<Entry>> entries_;
};

namespace {

using ToolpathMap = std::map<Uuid, std::shared_ptr<const toolpath::Toolpath>>;

/// The axisymmetric stock with `move` moves of a turning operation done and `fraction` of the
/// next, from `before`: the stock with `from` moves done and `fromFraction` of the next.
sim::LatheStock turnedTo(const document::OperationSimulation& op, const sim::LatheStock& before,
                         std::size_t from, double fromFraction, std::size_t move, double fraction) {
    sim::LatheStock lathe = before;
    if (!op.holeProfile.empty()) {
        // A live tool's holes: as far as the moves go, and the move it is in as far as it got.
        sim::cutHoles(lathe, op.holeProfile, op.toolpath, from, move);
        if (fraction > 0.0 && move < op.toolpath.moves.size()) {
            toolpath::Toolpath piece;
            piece.start = sim::toolPositionAt(op.toolpath, move);
            if (const auto part =
                    toolpath::partialMove(piece.start, toolpath::axisBefore(op.toolpath, move),
                                          op.toolpath.moves[move], fraction)) {
                piece.moves.push_back(*part);
                sim::cutHoles(lathe, op.holeProfile, piece);
            }
        }
        return lathe;
    }
    // Cuts move `k` from fraction t0 of it to t1.
    auto cutPart = [&](std::size_t k, double t0, double t1) {
        const toolpath::Move& m = op.toolpath.moves[k];
        toolpath::Toolpath piece;
        piece.start = sim::toolPositionAt(op.toolpath, k);
        if (m.pitch > 0.0 && op.toolpath.isTurning()) {
            lathe.cutThread(op.insert, Vec2(piece.start.z(), piece.start.x()),
                            Vec2(m.end.z(), m.end.x()), m.pitch, t0, t1);
            return;
        }
        // The rest of a threading toolpath takes nothing off.
        if (std::any_of(op.toolpath.moves.begin(), op.toolpath.moves.end(),
                        [](const toolpath::Move& t) { return t.pitch > 0.0; })) {
            return;
        }
        Vec3 axis = toolpath::axisBefore(op.toolpath, k);
        if (t0 > 0.0) {
            const auto head = toolpath::partialMove(piece.start, axis, m, t0);
            if (!head) {
                return;
            }
            piece.start = head->end;
            axis = head->axis;
        }
        // The rest of a move is the same move from where it got to.
        if (const auto rest = toolpath::partialMove(piece.start, axis, m, (t1 - t0) / (1.0 - t0))) {
            piece.moves.push_back(*rest);
            sim::simulateTurning(lathe, op.insert, piece);
        }
    };
    std::size_t next = from;
    if (fromFraction > 0.0 && next < move) {
        cutPart(next, fromFraction, 1.0);
        ++next;
    }
    if (move > next) {
        sim::simulateTurning(lathe, op.insert, op.toolpath, next, move);
    }
    if (fraction > 0.0) {
        cutPart(move, next == move ? fromFraction : 0.0, fraction);
    }
    return lathe;
}

std::function<std::optional<toolpath::Toolpath>(const document::Operation&)>
toolpathsFrom(ToolpathMap paths) {
    return [paths = std::move(paths)](
               const document::Operation& op) -> std::optional<toolpath::Toolpath> {
        const auto it = paths.find(op.id);
        if (it == paths.end() || !it->second) {
            return std::nullopt;
        }
        return *it->second;
    };
}

} // namespace

/// What a simulation needs of the setups its stock comes from, gathered on the GUI thread:
/// their recent whole simulations and the GUI's toolpaths. `simulated` collects the ones worked
/// out on the way.
struct StockSourceContext {
    std::shared_ptr<const document::Document> doc;
    std::shared_ptr<InFlightSimulations> inFlight;
    std::vector<std::pair<std::string, std::shared_ptr<const document::SetupSimulation>>> recent;
    std::map<Uuid, ToolpathMap> toolpaths;
    std::mutex mutex;
    std::vector<std::tuple<Uuid, std::string, std::shared_ptr<const document::SetupSimulation>>>
        simulated;
};

namespace {

/// A setup whose stock comes from an earlier one gets that setup's whole simulation: a recent
/// one, else from the cache or the simulations in flight, else simulated (and cached).
void useStockSources(document::SimulationOptions& options,
                     const std::shared_ptr<StockSourceContext>& context) {
    document::SimulationOptions base = options;
    base.toolpathFor = nullptr;
    base.sourceSimulation = nullptr;
    options.sourceSimulation = [context, base](const document::Setup& source, CancelToken& cancel,
                                               ProgressSink& progress) {
        document::SimulationOptions o = document::sourceOptions(base);
        if (const auto it = context->toolpaths.find(source.id); it != context->toolpaths.end()) {
            o.toolpathFor = toolpathsFrom(it->second);
        }
        useStockSources(o, context);
        const std::string key = document::simulationInputKey(*context->doc, source, o);
        for (const auto& [k, sim] : context->recent) {
            if (k == key) {
                return sim;
            }
        }
        auto sim = context->inFlight->run(
            key,
            [&](CancelToken& c, ProgressSink& p) {
                return simulateOrRead(*context->doc, source.id, o, key, simulationCacheFile(key), c,
                                      p);
            },
            cancel, progress);
        std::lock_guard lock(context->mutex);
        context->simulated.emplace_back(source.id, key, sim);
        return sim;
    };
}

/// Removes the least recently used cached simulations beyond the size limit.
void trimSimulationCache() {
    namespace fs = std::filesystem;
    const fs::path dir = simulationCacheDir();
    std::error_code ec;
    if (dir.empty() || !fs::is_directory(dir, ec)) {
        return;
    }
    std::vector<std::tuple<fs::file_time_type, std::uintmax_t, fs::path>> files;
    for (const auto& entry : fs::directory_iterator(dir, ec)) {
        if (entry.path().extension() == ".oxsim") {
            files.emplace_back(entry.last_write_time(ec), entry.file_size(ec), entry.path());
        }
    }
    std::sort(files.begin(), files.end(),
              [](const auto& a, const auto& b) { return std::get<0>(a) > std::get<0>(b); });
    std::uintmax_t total = 0;
    for (const auto& [time, size, path] : files) {
        total += size;
        if (total > kSimulationCacheBytes) {
            fs::remove(path, ec);
        }
    }
}

class AtomicProgress final : public ProgressSink {
public:
    explicit AtomicProgress(std::shared_ptr<std::atomic<double>> value)
        : value_(std::move(value)) {}
    void report(double fraction, std::string_view /*stage*/) override { value_->store(fraction); }

private:
    std::shared_ptr<std::atomic<double>> value_;
};

QString swatch(const sim::Rgb& c) {
    return QStringLiteral(
               "<span style='background-color:rgb(%1,%2,%3)'>&nbsp;&nbsp;&nbsp;&nbsp;</span>")
        .arg(c[0])
        .arg(c[1])
        .arg(c[2]);
}

} // namespace

template <typename Work, typename Done>
void SimulationPanel::runInBackground(Work work, Done done) {
    QThread* worker = QThread::create(std::move(work));
    threads_.push_back(worker);
    connect(worker, &QThread::finished, this, [this, worker, done = std::move(done)] {
        std::erase(threads_, worker);
        worker->deleteLater();
        done();
    });
    worker->start();
}

SimulationPanel::SimulationPanel(OcctView* view, SnapshotFn snapshot, ToolpathsFn toolpaths,
                                 QWidget* parent)
    : QWidget(parent), view_(view), snapshot_(std::move(snapshot)),
      toolpaths_(std::move(toolpaths)), inFlight_(std::make_shared<InFlightSimulations>()) {
    auto* layout = new QVBoxLayout(this);

    auto* runRow = new QHBoxLayout;
    runButton_ = new QPushButton(tr("Simulate"), this);
    runButton_->setToolTip(tr("Simulate the current setup (F5)"));
    cancelButton_ = new QPushButton(tr("Cancel"), this);
    closeButton_ = new QPushButton(tr("Close"), this);
    closeButton_->setToolTip(tr("Remove the simulation from the view"));
    runRow->addWidget(runButton_);
    runRow->addWidget(cancelButton_);
    runRow->addWidget(closeButton_);
    layout->addLayout(runRow);

    progress_ = new QProgressBar(this);
    progress_->setRange(0, 1000);
    progress_->setTextVisible(false);
    progress_->setMaximumHeight(8);
    layout->addWidget(progress_);
    status_ = new QLabel(tr("No simulation."), this);
    status_->setWordWrap(true);
    layout->addWidget(status_);

    auto* form = new QFormLayout;
    opCombo_ = new QComboBox(this);
    form->addRow(tr("Operation"), opCombo_);
    slider_ = new QSlider(Qt::Horizontal, this);
    form->addRow(tr("Move"), slider_);
    moveLabel_ = new QLabel(this);
    form->addRow(QString(), moveLabel_);
    layout->addLayout(form);

    auto* playRow = new QHBoxLayout;
    auto* back = new QToolButton(this);
    back->setText(QStringLiteral("⏮"));
    back->setToolTip(tr("Previous operation"));
    playButton_ = new QToolButton(this);
    playButton_->setText(QStringLiteral("▶"));
    playButton_->setToolTip(tr("Play / pause"));
    auto* forward = new QToolButton(this);
    forward->setText(QStringLiteral("⏭"));
    forward->setToolTip(tr("Next operation"));
    speedCombo_ = new QComboBox(this);
    for (const double speed : {0.25, 0.5, 1.0, 2.0, 5.0, 10.0, 25.0, 100.0, 500.0}) {
        speedCombo_->addItem(tr("%1×").arg(speed), speed);
    }
    // The last speed chosen stays for the next session.
    const int savedSpeed = speedCombo_->findData(
        QSettings().value(QStringLiteral("simulationSpeed"), 10.0).toDouble());
    speedCombo_->setCurrentIndex(savedSpeed >= 0 ? savedSpeed : speedCombo_->findData(10.0));
    connect(speedCombo_, &QComboBox::currentIndexChanged, this, [this] {
        QSettings().setValue(QStringLiteral("simulationSpeed"), speedCombo_->currentData());
    });
    speedCombo_->setToolTip(tr("Playback speed against machine time (programmed feeds; rapids "
                               "at 10 m/min)"));
    auto* stepBack = new QToolButton(this);
    stepBack->setText(QStringLiteral("|◀"));
    stepBack->setToolTip(tr("Previous move"));
    stepBack->setAutoRepeat(true);
    auto* stepForward = new QToolButton(this);
    stepForward->setText(QStringLiteral("▶|"));
    stepForward->setToolTip(tr("Next move"));
    stepForward->setAutoRepeat(true);
    connect(stepBack, &QToolButton::clicked, this, [this] { stepMove(-1); });
    connect(stepForward, &QToolButton::clicked, this, [this] { stepMove(1); });
    playRow->addWidget(back);
    playRow->addWidget(stepBack);
    playRow->addWidget(playButton_);
    playRow->addWidget(stepForward);
    playRow->addWidget(forward);
    playRow->addWidget(speedCombo_, 1);
    layout->addLayout(playRow);

    auto* options = new QHBoxLayout;
    showTool_ = new QCheckBox(tr("Tool"), this);
    showTool_->setChecked(true);
    showPart_ = new QCheckBox(tr("Part"), this);
    showPart_->setChecked(false);
    showFixtures_ = new QCheckBox(tr("Workholding"), this);
    showFixtures_->setToolTip(tr("Show the setup's fixtures and vise"));
    showFixtures_->setChecked(true);
    showPart_->setToolTip(tr("Show the part: see-through over the stock (inside a transparent "
                             "stock as it is)"));
    compare_ = new QCheckBox(tr("Compare"), this);
    compare_->setToolTip(tr("Colour the stock by its deviation from the part"));
    compare_->setChecked(true);
    options->addWidget(showTool_);
    options->addWidget(showPart_);
    options->addWidget(showFixtures_);
    options->addWidget(compare_);
    options->addStretch(1);
    layout->addLayout(options);

    // The toolpaths over the stock: none, all, or only the trail behind the tool.
    auto* pathsRow = new QHBoxLayout;
    pathsRow->addWidget(new QLabel(tr("Paths"), this));
    pathsCombo_ = new QComboBox(this);
    pathsCombo_->setObjectName(QStringLiteral("simulationPaths"));
    pathsCombo_->addItem(tr("None"), static_cast<int>(PathsShown::None));
    pathsCombo_->addItem(tr("All"), static_cast<int>(PathsShown::All));
    pathsCombo_->addItem(tr("Trail"), static_cast<int>(PathsShown::Trail));
    pathsCombo_->setToolTip(tr("Toolpaths shown over the simulated stock: none, all of them, or "
                               "only the trail: the path the tool has just come along"));
    const int savedPaths =
        pathsCombo_->findData(QSettings().value(QStringLiteral("simulationPaths"), 0).toInt());
    pathsCombo_->setCurrentIndex(std::max(0, savedPaths));
    trailLength_ = new QDoubleSpinBox(this);
    trailLength_->setObjectName(QStringLiteral("simulationTrailLength"));
    trailLength_->setSpecialValueText(tr("All of the operation"));
    trailLength_->setKeyboardTracking(false);
    trailLength_->setToolTip(tr("How much of the path behind the tool the trail shows, measured "
                                "along the path (rapids included). At 0 it shows all of the "
                                "operation's path cut so far."));
    trailLengthMm_ =
        std::max(0.0, QSettings().value(QStringLiteral("simulationTrailLength"), 50.0).toDouble());
    pathsRow->addWidget(pathsCombo_);
    pathsRow->addWidget(new QLabel(tr("Trail"), this));
    pathsRow->addWidget(trailLength_, 1);
    layout->addLayout(pathsRow);
    connect(pathsCombo_, &QComboBox::currentIndexChanged, this, [this] {
        QSettings().setValue(QStringLiteral("simulationPaths"), pathsCombo_->currentData());
        trailLength_->setEnabled(sim_ && !simRunning_ && pathsShown() == PathsShown::Trail);
        requestFrame(lastFrame_);
    });
    connect(trailLength_, &QDoubleSpinBox::valueChanged, this, [this](double v) {
        trailLengthMm_ = v * (unit_ == LengthUnit::Inch ? 25.4 : 1.0);
        QSettings().setValue(QStringLiteral("simulationTrailLength"), trailLengthMm_);
        requestFrame(lastFrame_);
    });

    // The machine moving with its axes.
    auto* machineRow = new QHBoxLayout;
    showMachine_ = new QCheckBox(tr("Machine"), this);
    showMachine_->setToolTip(tr("Show the machine moving with its axes"));
    showMachine_->setChecked(QSettings().value(QStringLiteral("simulationMachine"), true).toBool());
    machineFixed_ = new QCheckBox(tr("Machine fixed"), this);
    machineFixed_->setToolTip(tr("Keep the machine still and move the part with the table, as on "
                                 "the machine (off: the part stays still and the machine moves "
                                 "around it)"));
    machineFixed_->setChecked(
        QSettings().value(QStringLiteral("simulationMachineFixed"), true).toBool());
    showEnclosure_ = new QCheckBox(tr("Enclosure"), this);
    showEnclosure_->setObjectName(QStringLiteral("simulationEnclosure"));
    showEnclosure_->setToolTip(tr("Show the machine's enclosure (its walls, sheet metal and way "
                                  "covers, as the machine library marks them); off: the rest of "
                                  "the machine without it. It is still checked for collisions."));
    showEnclosure_->setChecked(
        QSettings().value(QStringLiteral("simulationEnclosure"), true).toBool());
    machineRow->addWidget(showMachine_);
    machineRow->addWidget(showEnclosure_);
    machineRow->addWidget(machineFixed_);
    machineRow->addStretch(1);
    layout->addLayout(machineRow);
    axes_ = new QLabel(this);
    axes_->setTextFormat(Qt::RichText);
    axes_->setToolTip(tr("Machine axis positions (from the machine model), red beyond their "
                         "travel"));
    layout->addWidget(axes_);
    partList_ = new QListWidget(this);
    partList_->setToolTip(tr("Machine parts shown in the view (for this session; the machine "
                             "library sets the defaults). Hidden parts are still checked for "
                             "collisions."));
    partList_->setMaximumHeight(120);
    partList_->hide();
    layout->addWidget(partList_);
    connect(partList_, &QListWidget::itemChanged, this, [this](QListWidgetItem* item) {
        if (!sim_ || !sim_->machine) {
            return;
        }
        partShown_[sim_->machine->id][item->data(Qt::UserRole).toString().toStdString()] =
            item->checkState() == Qt::Checked;
        requestFrame();
    });

    // Ray spacing of the simulated stock (kept between sessions).
    auto* resolutionRow = new QHBoxLayout;
    resolutionRow->addWidget(new QLabel(tr("Resolution"), this));
    resolution_ = new QDoubleSpinBox(this);
    resolution_->setObjectName(QStringLiteral("simResolution"));
    resolution_->setSpecialValueText(tr("Auto"));
    resolution_->setKeyboardTracking(false);
    resolution_->setToolTip(tr("Ray spacing of the simulated stock. Auto is about 1/800 of the "
                               "stock's size; finer shows walls more smoothly and takes longer. "
                               "Simulate again to apply."));
    resolutionMm_ =
        std::max(0.0, QSettings().value(QStringLiteral("simResolution"), 0.0).toDouble());
    resolutionRow->addWidget(resolution_, 1);
    layout->addLayout(resolutionRow);
    setDisplayUnit(unit_);
    connect(resolution_, &QDoubleSpinBox::valueChanged, this, [this](double v) {
        resolutionMm_ = v * (unit_ == LengthUnit::Inch ? 25.4 : 1.0);
        QSettings().setValue(QStringLiteral("simResolution"), resolutionMm_);
        markStale();
    });

    legend_ = new QLabel(this);
    legend_->setTextFormat(Qt::RichText);
    legend_->setWordWrap(true);
    layout->addWidget(legend_);

    stats_ = new QLabel(this);
    stats_->setWordWrap(true);
    stats_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    layout->addWidget(stats_);

    layout->addWidget(new QLabel(tr("Collisions"), this));
    collisions_ = new QListWidget(this);
    layout->addWidget(collisions_, 1);

    playTimer_ = new QTimer(this);
    playTimer_->setInterval(16); // the tool moves every display frame
    progressTimer_ = new QTimer(this);
    progressTimer_->setInterval(100);

    connect(runButton_, &QPushButton::clicked, this, [this] {
        if (lastSetup_) {
            simulate(*lastSetup_);
        }
    });
    connect(cancelButton_, &QPushButton::clicked, this, &SimulationPanel::cancelSimulation);
    connect(closeButton_, &QPushButton::clicked, this, [this] {
        closeSimulation();
        emit closed();
    });
    connect(opCombo_, &QComboBox::currentIndexChanged, this, &SimulationPanel::operationChanged);
    connect(slider_, &QSlider::valueChanged, this, &SimulationPanel::sliderChanged);
    connect(playButton_, &QToolButton::clicked, this, &SimulationPanel::togglePlay);
    connect(back, &QToolButton::clicked, this, [this] { step(-1); });
    connect(forward, &QToolButton::clicked, this, [this] { step(1); });
    connect(showTool_, &QCheckBox::toggled, this, [this] { requestFrame(); });
    connect(showPart_, &QCheckBox::toggled, this, [this] { requestFrame(); });
    connect(showFixtures_, &QCheckBox::toggled, this, [this] { requestFrame(); });
    connect(showMachine_, &QCheckBox::toggled, this, [this](bool on) {
        QSettings().setValue(QStringLiteral("simulationMachine"), on);
        fitMachine_ = on;
        rebuildControls();
        requestFrame();
    });
    connect(showEnclosure_, &QCheckBox::toggled, this, [this](bool on) {
        QSettings().setValue(QStringLiteral("simulationEnclosure"), on);
        requestFrame();
    });
    connect(machineFixed_, &QCheckBox::toggled, this, [this](bool on) {
        QSettings().setValue(QStringLiteral("simulationMachineFixed"), on);
        requestFrame();
    });
    connect(compare_, &QCheckBox::toggled, this, [this] {
        cached_.reset(); // cached frames carry no comparison
        requestFrame();
    });
    connect(collisions_, &QListWidget::itemActivated, this, [this](QListWidgetItem* item) {
        const int op = item->data(Qt::UserRole).toInt();
        const auto move = item->data(Qt::UserRole + 1).toULongLong();
        playTimer_->stop();
        playButton_->setText(QStringLiteral("▶"));
        opCombo_->setCurrentIndex(op);
        slider_->setValue(static_cast<int>(std::min<std::size_t>(move + 1, movesOf(op))));
    });
    connect(playTimer_, &QTimer::timeout, this, &SimulationPanel::playTick);
    connect(progressTimer_, &QTimer::timeout, this, [this] {
        if (simProgress_) {
            progress_->setValue(static_cast<int>(simProgress_->load() * 1000.0));
        }
        updatePreviewBusy();
    });
    rebuildControls();
}

SimulationPanel::~SimulationPanel() {
    if (simCancel_) {
        simCancel_->requestCancel();
    }
    if (prepareCancel_) {
        prepareCancel_->requestCancel(); // closing does not wait for a background simulation
    }
    for (QThread* t : threads_) {
        disconnect(t, nullptr, this, nullptr);
        t->wait();
        t->deleteLater();
    }
}

void SimulationPanel::simulate(const Uuid& setupId, std::optional<Uuid> lastOperation) {
    lastSetup_ = setupId;
    auto doc = snapshot_();
    if (!doc) {
        return;
    }
    const document::Setup* setup = doc->project().findSetup(setupId);
    if (!setup) {
        return;
    }
    document::SimulationOptions options = baseOptions();
    options.lastOperation = lastOperation;
    const std::string key = document::simulationInputKey(*doc, *setup, options);
    if (simRunning_ && simRunSetup_ == setupId && simRunKey_ == key) {
        // The same simulation is running (another operation of the setup was picked meanwhile):
        // it goes on, and shows the operation picked last when done.
        return;
    }
    if (prepareAdopted_) {
        // Waiting for a preparation: it goes on in the background, no longer shown when done.
        prepareAdopted_ = false;
        simRunning_ = false;
        simCancel_.reset();
        progressTimer_->stop();
    }
    cancelSimulation();
    playTimer_->stop();
    playButton_->setText(QStringLiteral("▶"));
    stale_ = false;
    // Nothing that affects the result changed: show the simulation already computed (this
    // setup's, even after looking at another).
    if (showRecent(*doc, *setup, lastOperation, std::nullopt)) {
        return;
    }
    if (prepareRunning_ && !lastOperation && prepareSetup_ == setupId && prepareKey_ == key) {
        // Being prepared: wait for it instead of starting it again.
        prepareAdopted_ = true;
        prepareWcs_ = setup->wcs.toParent();
        simRunning_ = true;
        simRunSetup_ = setupId;
        simRunKey_ = key;
        simCancel_ = prepareCancel_;
        simProgress_ = prepareProgress_;
        progressTimer_->start();
        status_->setText(tr("Simulating %1…").arg(QString::fromStdString(setup->name)));
        rebuildControls();
        updatePreviewBusy();
        return;
    }
    const std::vector<const document::Setup*> sources =
        document::stockSources(doc->project(), *setup);
    const bool prepareNeeded =
        std::any_of(sources.begin(), sources.end(),
                    [&](const document::Setup* s) { return s->id == prepareSetup_; });
    if (prepareRunning_ && prepareCancel_ && !prepareNeeded) {
        // The user's simulation comes first; the preparation runs again afterwards. One of a
        // setup this stock comes from goes on: the simulation waits for it.
        prepareQueue_.push_front(prepareSetup_);
        prepareCancel_->requestCancel();
    }
    simRunning_ = true;
    simRunSetup_ = setupId;
    simRunKey_ = key;
    auto cancel = std::make_shared<CancelToken>();
    simCancel_ = cancel;
    simProgress_ = std::make_shared<std::atomic<double>>(0.0);
    progress_->setValue(0);
    progressTimer_->start();
    status_->setText(tr("Simulating %1…").arg(QString::fromStdString(setup->name)));
    rebuildControls();
    updatePreviewBusy();

    struct Result {
        std::shared_ptr<const document::SetupSimulation> sim;
        QString error;
    };
    auto result = std::make_shared<Result>();
    auto progressValue = simProgress_;
    const Transform3 wcsToProject = setup->wcs.toParent();
    std::shared_ptr<StockSourceContext> context;
    options = simulationOptions(doc, *setup, lastOperation, context);
    // A whole setup's simulation is kept in the local cache for the next time.
    const std::filesystem::path cacheFile =
        lastOperation ? std::filesystem::path() : simulationCacheFile(key);
    runInBackground(
        [doc, setupId, options, cancel, result, progressValue, key, cacheFile, inFlight = inFlight_,
         whole = !lastOperation] {
            try {
                AtomicProgress progress(progressValue);
                auto compute = [&](CancelToken& c, ProgressSink& p) {
                    return simulateOrRead(*doc, setupId, options, key, cacheFile, c, p);
                };
                result->sim = whole ? inFlight->run(key, compute, *cancel, progress)
                                    : compute(*cancel, progress);
            } catch (const OperationCancelled&) {
                // superseded or cancelled
            } catch (const std::exception& e) {
                result->error = QString::fromUtf8(e.what());
            }
        },
        [this, result, cancel, wcsToProject, key, lastOperation, setupId, context] {
            rememberSources(*context);
            if (cancel != simCancel_) {
                return; // superseded by a newer run
            }
            if (result->sim) {
                remember({setupId, lastOperation, key, result->sim});
            }
            simRunning_ = false;
            progressTimer_->stop();
            updatePreviewBusy();
            wcsToProject_ = wcsToProject;
            simKey_ = result->sim ? key : std::string();
            simLastOperation_ = lastOperation;
            simulationDone(result->sim, cancel->isCancelled() ? tr("Cancelled.") : result->error);
            if (previewActive_ && previewOperation_) {
                showOperationEnd(*previewOperation_);
            }
            trimSimulationCache();
            startNextPrepare();
        });
}

document::SimulationOptions
SimulationPanel::simulationOptions(const std::shared_ptr<const document::Document>& doc,
                                   const document::Setup& setup, std::optional<Uuid> lastOperation,
                                   std::shared_ptr<StockSourceContext>& context) const {
    document::SimulationOptions options = baseOptions();
    options.lastOperation = lastOperation;
    if (toolpaths_) {
        options.toolpathFor = toolpathsFrom(toolpaths_(*doc, setup.id));
    }
    // The setup's latest simulation: what has not changed since is not simulated again.
    for (const Recent& r : recent_) {
        if (r.setupId == setup.id) {
            options.resumeFrom = r.sim;
            break;
        }
    }
    context = std::make_shared<StockSourceContext>();
    context->doc = doc;
    context->inFlight = inFlight_;
    const std::vector<const document::Setup*> sources =
        document::stockSources(doc->project(), setup);
    if (!sources.empty()) {
        for (const Recent& r : recent_) {
            if (!r.lastOperation) {
                context->recent.emplace_back(r.key, r.sim);
            }
        }
        if (toolpaths_) {
            for (const document::Setup* s : sources) {
                context->toolpaths[s->id] = toolpaths_(*doc, s->id);
            }
        }
        useStockSources(options, context);
    }
    return options;
}

void SimulationPanel::rememberSources(StockSourceContext& context) {
    std::lock_guard lock(context.mutex);
    for (auto& [setupId, key, sim] : context.simulated) {
        if (sim) {
            remember({setupId, std::nullopt, key, sim});
        }
    }
    context.simulated.clear();
}

void SimulationPanel::prepareSetups(const std::vector<Uuid>& setupIds) {
    for (const Uuid& id : setupIds) {
        if (std::find(prepareQueue_.begin(), prepareQueue_.end(), id) == prepareQueue_.end() &&
            !(prepareRunning_ && prepareSetup_ == id)) {
            prepareQueue_.push_back(id);
        }
    }
    startNextPrepare();
}

bool SimulationPanel::hasValidWhole(const document::Document& doc,
                                    const document::Setup& setup) const {
    document::SimulationOptions options = baseOptions();
    const std::string key = document::simulationInputKey(doc, setup, options);
    return std::any_of(recent_.begin(), recent_.end(), [&](const Recent& r) {
        return r.setupId == setup.id && !r.lastOperation && r.key == key;
    });
}

void SimulationPanel::startNextPrepare() {
    if (simRunning_ || prepareRunning_) {
        return;
    }
    auto doc = snapshot_();
    if (!doc) {
        prepareQueue_.clear();
        return;
    }
    while (!prepareQueue_.empty()) {
        const Uuid setupId = prepareQueue_.front();
        prepareQueue_.pop_front();
        const document::Setup* setup = doc->project().findSetup(setupId);
        if (!setup || setup->operations.empty() || hasValidWhole(*doc, *setup)) {
            continue; // gone, nothing to simulate, or already worked out
        }
        std::shared_ptr<StockSourceContext> context;
        const document::SimulationOptions options =
            simulationOptions(doc, *setup, std::nullopt, context);
        const std::string key = document::simulationInputKey(*doc, *setup, options);
        prepareRunning_ = true;
        prepareSetup_ = setupId;
        prepareKey_ = key;
        prepareAdopted_ = false;
        auto cancel = std::make_shared<CancelToken>();
        prepareCancel_ = cancel;
        prepareProgress_ = std::make_shared<std::atomic<double>>(0.0);
        auto progressValue = prepareProgress_;
        struct Result {
            std::shared_ptr<const document::SetupSimulation> sim;
        };
        auto result = std::make_shared<Result>();
        const std::filesystem::path cacheFile = simulationCacheFile(key);
        runInBackground(
            [doc, setupId, options, cancel, result, progressValue, key, cacheFile,
             inFlight = inFlight_] {
                try {
                    AtomicProgress progress(progressValue);
                    result->sim = inFlight->run(
                        key,
                        [&](CancelToken& c, ProgressSink& p) {
                            return simulateOrRead(*doc, setupId, options, key, cacheFile, c, p);
                        },
                        *cancel, progress);
                } catch (const std::exception&) {
                    // cancelled or failed: shown, if asked for, by its own simulation
                }
            },
            [this, result, cancel, key, setupId, context] {
                rememberSources(*context);
                if (cancel != prepareCancel_) {
                    return;
                }
                prepareRunning_ = false;
                if (result->sim) {
                    remember({setupId, std::nullopt, key, result->sim});
                }
                if (prepareAdopted_) {
                    // The user asked for this setup meanwhile: show it as their simulation.
                    prepareAdopted_ = false;
                    simRunning_ = false;
                    progressTimer_->stop();
                    updatePreviewBusy();
                    wcsToProject_ = prepareWcs_;
                    simKey_ = result->sim ? key : std::string();
                    simLastOperation_.reset();
                    simulationDone(result->sim,
                                   cancel->isCancelled() ? tr("Cancelled.") : QString());
                    if (previewActive_ && previewOperation_) {
                        showOperationEnd(*previewOperation_);
                    }
                }
                trimSimulationCache();
                startNextPrepare();
            });
        return;
    }
}

void SimulationPanel::setCurrentSetup(const std::optional<Uuid>& setupId) {
    lastSetup_ = setupId;
    runButton_->setEnabled(!simRunning_ && lastSetup_.has_value());
}

void SimulationPanel::cancelSimulation() {
    if (simRunning_ && simCancel_) {
        simCancel_->requestCancel();
    }
}

void SimulationPanel::simulationDone(std::shared_ptr<const document::SetupSimulation> sim,
                                     const QString& error) {
    if (!sim) {
        status_->setText(error.isEmpty() ? tr("Simulation failed.") : error);
        progress_->setValue(0);
        rebuildControls();
        emit simulationFinished(false);
        return;
    }
    fitMachine_ = !sim_ || sim_->setupId != sim->setupId;
    sim_ = std::move(sim);
    machineModel_ = {};
    machineParts_.reset();
    if (sim_->machine) {
        machineModel_ = machine::effectiveModel(*sim_->machine);
        // A lathe's blocks and tools ride on the turret, listed with the machine's parts (as the
        // turret preview has them).
        document::LatheMachineParts all =
            document::latheMachineParts(*sim_->machine, machineModel_, sim_->turretBlocks,
                                        sim_->turretTools, sim_->machineChuck);
        auto parts = std::make_shared<std::vector<machine::ModelPart>>(std::move(all.parts));
        toolPartsFrom_ = all.toolsFrom;
        if (!machineModel_.empty() && !parts->empty()) {
            machineParts_ = std::move(parts);
        }
    }
    rebuildPartList();
    cached_.reset();
    display_ = makeDisplay();
    lastFrame_.reset();
    cutBase_.clear();
    updateLegend();
    startTimes_.clear();
    startTips_.clear();
    startLengths_.clear();
    for (const auto& op : sim_->operations) {
        // How far along the path each move starts, for the trail.
        std::vector<double> lengths{0.0};
        Vec3 at = op.toolpath.start;
        for (const toolpath::Move& m : op.toolpath.moves) {
            double length = 0.0;
            if (m.isArc()) {
                const std::vector<Vec3> points = toolpath::sampleArc(at, m, 0.01);
                for (std::size_t k = 1; k < points.size(); ++k) {
                    length += (points[k] - points[k - 1]).norm();
                }
            } else if (m.kind != toolpath::MoveKind::Dwell) {
                length = (m.end - at).norm();
            }
            lengths.push_back(lengths.back() + length);
            if (m.kind != toolpath::MoveKind::Dwell) {
                at = m.end;
            }
        }
        startLengths_.push_back(std::move(lengths));
        const std::vector<double> durations = toolpath::moveDurations(op.toolpath);
        // A lathe's machine moves between operations before the first move.
        double first = 0.0;
        if (op.latheMotion) {
            for (const machine::LatheStep& step : op.latheMotion->steps) {
                first += step.duration;
            }
        }
        std::vector<double> times{first};
        std::vector<Vec3> tips{op.toolpath.start};
        for (std::size_t k = 0; k < durations.size(); ++k) {
            times.push_back(times.back() + durations[k]);
            const auto& m = op.toolpath.moves[k];
            tips.push_back(m.kind == toolpath::MoveKind::Dwell ? tips.back() : m.end);
        }
        startTimes_.push_back(std::move(times));
        startTips_.push_back(std::move(tips));
    }
    progress_->setValue(1000);
    collisions_->clear();
    for (std::size_t i = 0; i < sim_->operations.size(); ++i) {
        const auto& op = sim_->operations[i];
        for (const auto& c : op.collisions) {
            auto* item = new QListWidgetItem(
                tr("%1: %2 collision, moves %3–%4")
                    .arg(QString::fromStdString(op.name),
                         QString::fromUtf8(sim::toString(c.kind).data(),
                                           static_cast<qsizetype>(sim::toString(c.kind).size())))
                    .arg(c.firstMove + 1)
                    .arg(c.lastMove + 1),
                collisions_);
            item->setData(Qt::UserRole, static_cast<int>(i));
            item->setData(Qt::UserRole + 1, static_cast<qulonglong>(c.firstMove));
            item->setForeground(QColor(200, 40, 40));
        }
        for (const auto& c : op.machineCollisions) {
            auto* item = new QListWidgetItem(
                c.during.empty()
                    ? tr("%1: machine %2 / %3, moves %4–%5")
                          .arg(QString::fromStdString(op.name), QString::fromStdString(c.first),
                               QString::fromStdString(c.second))
                          .arg(c.firstMove + 1)
                          .arg(c.lastMove + 1)
                    : tr("%1: machine %2 / %3, in %4 before its first move")
                          .arg(QString::fromStdString(op.name), QString::fromStdString(c.first),
                               QString::fromStdString(c.second), QString::fromStdString(c.during)),
                collisions_);
            item->setData(Qt::UserRole, static_cast<int>(i));
            item->setData(Qt::UserRole + 1, static_cast<qulonglong>(c.firstMove));
            item->setForeground(QColor(200, 110, 0));
        }
        for (const auto& c : op.fixtureCollisions) {
            auto* item = new QListWidgetItem(tr("%1: %2 hits %3, moves %4–%5")
                                                 .arg(QString::fromStdString(op.name),
                                                      QString::fromStdString(c.toolPart),
                                                      QString::fromStdString(c.body))
                                                 .arg(c.firstMove + 1)
                                                 .arg(c.lastMove + 1),
                                             collisions_);
            item->setData(Qt::UserRole, static_cast<int>(i));
            item->setData(Qt::UserRole + 1, static_cast<qulonglong>(c.firstMove));
            item->setForeground(QColor(200, 40, 40));
        }
        if (!op.error.empty()) {
            auto* item = new QListWidgetItem(
                tr("%1: not simulated (%2)")
                    .arg(QString::fromStdString(op.name), QString::fromStdString(op.error)),
                collisions_);
            item->setData(Qt::UserRole, static_cast<int>(i));
            item->setData(Qt::UserRole + 1, 0);
        }
    }
    rebuildControls();
    showFinal();
    updateStatus();
    emit simulationFinished(true);
}

void SimulationPanel::rebuildControls() {
    const bool has = static_cast<bool>(sim_);
    runButton_->setEnabled(!simRunning_ && lastSetup_.has_value());
    cancelButton_->setEnabled(simRunning_);
    // Close leaves simulation mode even with no simulation shown.
    closeButton_->setEnabled(!simRunning_);
    for (QWidget* w :
         std::initializer_list<QWidget*>{opCombo_, slider_, playButton_, speedCombo_, showTool_,
                                         showPart_, showFixtures_, pathsCombo_, compare_}) {
        w->setEnabled(has && !simRunning_);
    }
    trailLength_->setEnabled(has && !simRunning_ && pathsShown() == PathsShown::Trail);
    compare_->setEnabled(has && !simRunning_ && !sim_->part.isEmpty());
    showMachine_->setEnabled(has && !simRunning_ && machineParts_);
    partList_->setVisible(has && machineParts_ && showMachine_->isChecked());
    machineFixed_->setEnabled(has && !simRunning_ && machineParts_ && showMachine_->isChecked());
    showEnclosure_->setEnabled(has && machineParts_ && showMachine_->isChecked());
    if (!has) {
        axes_->clear();
    }
    legend_->setVisible(has);
    const QSignalBlocker block(opCombo_);
    opCombo_->clear();
    if (has) {
        for (std::size_t i = 0; i < sim_->operations.size(); ++i) {
            opCombo_->addItem(QStringLiteral("%1. %2").arg(i + 1).arg(
                QString::fromStdString(sim_->operations[i].name)));
        }
    }
}

std::size_t SimulationPanel::movesOf(int op) const {
    if (!sim_ || op < 0 || op >= static_cast<int>(sim_->operations.size())) {
        return 0;
    }
    return sim_->operations[static_cast<std::size_t>(op)].toolpath.moves.size();
}

void SimulationPanel::showFinal() {
    if (!sim_ || sim_->operations.empty()) {
        if (sim_) {
            startFrame({0, 0});
        }
        return;
    }
    const int last = static_cast<int>(sim_->operations.size()) - 1;
    {
        const QSignalBlocker block(opCombo_);
        opCombo_->setCurrentIndex(last);
    }
    {
        const QSignalBlocker block(slider_);
        slider_->setRange(0, static_cast<int>(movesOf(last)));
        slider_->setValue(static_cast<int>(movesOf(last)));
    }
    updateMoveLabel();
    requestFrame();
}

void SimulationPanel::showPosition(int op, std::size_t move) {
    if (!sim_ || sim_->operations.empty()) {
        return;
    }
    op = std::clamp(op, 0, static_cast<int>(sim_->operations.size()) - 1);
    {
        const QSignalBlocker block(opCombo_);
        opCombo_->setCurrentIndex(op);
    }
    {
        const QSignalBlocker block(slider_);
        slider_->setRange(0, static_cast<int>(movesOf(op)));
        slider_->setValue(static_cast<int>(std::min(move, movesOf(op))));
    }
    updateMoveLabel();
    requestFrame();
}

void SimulationPanel::operationChanged(int index) {
    if (!sim_ || index < 0) {
        return;
    }
    const QSignalBlocker block(slider_);
    slider_->setRange(0, static_cast<int>(movesOf(index)));
    slider_->setValue(playTimer_->isActive() ? 0 : static_cast<int>(movesOf(index)));
    syncPlayTime();
    updateMoveLabel();
    requestFrame();
}

void SimulationPanel::sliderChanged(int /*value*/) {
    syncPlayTime();
    updateMoveLabel();
    requestFrame();
}

void SimulationPanel::syncPlayTime() {
    const int op = opCombo_->currentIndex();
    if (!sim_ || op < 0 || op >= static_cast<int>(startTimes_.size())) {
        playTime_ = 0.0;
        return;
    }
    const auto& times = startTimes_[static_cast<std::size_t>(op)];
    // From an operation's start, playing shows what the machine does before its first move.
    playTime_ = slider_->value() <= 0
                    ? 0.0
                    : times[std::min(static_cast<std::size_t>(slider_->value()), times.size() - 1)];
    playAt_ = {op, static_cast<std::size_t>(std::max(0, slider_->value())), 0.0};
}

double SimulationPanel::durationOf(int op) const {
    return op >= 0 && op < static_cast<int>(startTimes_.size())
               ? startTimes_[static_cast<std::size_t>(op)].back()
               : 0.0;
}

SimulationPanel::Position SimulationPanel::positionAt(int op, double t) const {
    Position at{op, 0, 0.0};
    if (op < 0 || op >= static_cast<int>(startTimes_.size())) {
        return at;
    }
    const auto& times = startTimes_[static_cast<std::size_t>(op)];
    if (t < times.front()) {
        at.between = t;
        return at;
    }
    const std::size_t moves = times.size() - 1;
    // The move running at time t: the last start time at or before t.
    const auto it = std::upper_bound(times.begin(), times.end(), t);
    const std::size_t k = std::min(
        static_cast<std::size_t>(std::max<std::ptrdiff_t>(it - times.begin() - 1, 0)), moves);
    at.move = k;
    if (k < moves) {
        const double duration = times[k + 1] - times[k];
        at.fraction = duration > 0.0 ? std::clamp((t - times[k]) / duration, 0.0, 1.0) : 0.0;
        if (at.fraction >= 1.0) {
            at.move = k + 1;
            at.fraction = 0.0;
        }
    }
    return at;
}

double SimulationPanel::betweenDuration(int op) const {
    return op >= 0 && op < static_cast<int>(startTimes_.size())
               ? startTimes_[static_cast<std::size_t>(op)].front()
               : 0.0;
}

std::optional<SimulationPanel::MachineAt> SimulationPanel::latheMachineAt(const Position& at,
                                                                          QString* axes) const {
    const auto& op = sim_->operations[static_cast<std::size_t>(at.op)];
    if (!op.latheMotion || !op.mount) {
        return std::nullopt;
    }
    const machine::LatheMotion& motion = *op.latheMotion;
    machine::AxisPosition position;
    double workZ2 = 0.0;
    bool workHeld = false;
    if (at.between >= 0.0) {
        // Along the moves before the first one.
        double left = at.between;
        position = motion.start;
        for (const machine::LatheStep& step : motion.steps) {
            if (left < step.duration && step.duration > 0.0) {
                if (step.workStays) {
                    workZ2 = step.to.z2;
                    workHeld = true;
                }
                position = machine::lerpAxes(position, step.to, left / step.duration);
                break;
            }
            left -= step.duration;
            position = step.to;
        }
    } else {
        const auto solved =
            machine::latheAxes(machineModel_, motion, *op.mount, op.toolpath, toolPose(at).first);
        if (!solved) {
            return std::nullopt;
        }
        position = *solved;
    }
    const machine::ModelPose pose = machine::poseAt(machineModel_, position);
    if (axes) {
        QStringList parts;
        for (const char* name : {"X", "Y", "Z"}) {
            const machine::Axis* axis = sim_->machine->axis(name);
            if (!axis) {
                continue;
            }
            const double value = position.value(name);
            const bool over = value < axis->min - 1e-6 || value > axis->max + 1e-6;
            const QString text =
                QStringLiteral("%1 %2").arg(QString::fromLatin1(name), formatLength(value, unit_));
            parts << (over ? QStringLiteral("<span style=\"color:#d04040\">%1</span>").arg(text)
                           : text);
        }
        if (std::abs(position.rotary.c) > 1e-9) {
            parts << QStringLiteral("C %1°").arg(position.rotary.c * 180.0 / kPi, 0, 'f', 3);
        }
        parts << tr("Station %1").arg(motion.station);
        if (machineModel_.axis("W")) {
            parts << QStringLiteral("W %1").arg(formatLength(position.w, unit_));
        }
        if (machineModel_.axis("Z2")) {
            parts << QStringLiteral("Z2 %1").arg(formatLength(position.z2, unit_));
        }
        *axes = parts.join(QStringLiteral(" &nbsp; "));
    }
    if (!machineParts_ || !showMachine_->isChecked()) {
        return std::nullopt;
    }
    machine::AxisPosition workAt = position;
    if (workHeld) {
        workAt.z2 = workZ2;
    }
    const Transform3 machineFromProject =
        machine::poseAt(machineModel_, workAt).carried(machineModel_, op.mount->carrier) *
        op.mount->wcs * wcsToProject_.inverse();
    const bool fixed = machineFixed_->isChecked();
    const Transform3 viewFromMachine =
        fixed ? Transform3::Identity() : Transform3(machineFromProject.inverse());
    MachineAt out;
    out.scene.parts = machineParts_;
    for (std::size_t i = 0; i < machineParts_->size(); ++i) {
        const machine::ModelPart& p = (*machineParts_)[i];
        out.scene.placements.push_back(viewFromMachine * pose.carried(machineModel_, p.carrier) *
                                       p.placement());
        out.scene.hidden.push_back(i < toolPartsFrom_ ? !partShown(p) : !showTool_->isChecked());
    }
    out.viewFromProject = fixed ? machineFromProject : Transform3::Identity();
    out.viewFromMachine = viewFromMachine;
    if (std::abs(position.w - motion.held.w) > 1e-9) {
        // The setup's quill is a model of the setup, standing where W has it to cut.
        Transform3 move = Transform3::Identity();
        move.translation() = Vec3(0.0, 0.0, position.w - motion.held.w);
        out.modelMoves[document::tailstockSupportId()] =
            viewFromMachine * move * viewFromMachine.inverse();
        out.modelMoves[document::tailstockCentreSupportId()] =
            viewFromMachine * move * viewFromMachine.inverse();
    }
    return out;
}

std::optional<SimulationPanel::MachineAt> SimulationPanel::machineAt(const Position& at,
                                                                     QString* axes) const {
    if (!sim_ || sim_->operations.empty() || machineModel_.empty() || !sim_->machine) {
        return std::nullopt;
    }
    if (sim_->machine->lathe) {
        return latheMachineAt(at, axes);
    }
    const auto& op = sim_->operations[static_cast<std::size_t>(at.op)];
    const auto& angles = op.rotary.angles;
    if (angles.size() != op.toolpath.moves.size() || !op.mount) {
        return std::nullopt;
    }
    // Rotary positions between the move's ends, the tip where the tool is.
    const machine::IndexAngles before = at.move == 0 || angles.empty()
                                            ? op.rotary.start
                                            : angles[std::min(at.move, angles.size()) - 1];
    const machine::IndexAngles after = at.move < angles.size() ? angles[at.move] : before;
    const double t = at.fraction;
    const machine::IndexAngles q{before.a + (after.a - before.a) * t,
                                 before.b + (after.b - before.b) * t,
                                 before.c + (after.c - before.c) * t};
    const Vec3 tip = toolPose(at).first;
    const double gauge = op.tool ? sim::gaugeLength(*op.tool) : 0.0;
    const auto position =
        machine::positionFor(machineModel_, q, op.mount->carrier, op.mount->wcs * tip, gauge);
    if (!position) {
        return std::nullopt;
    }
    const machine::ModelPose pose = machine::poseAt(machineModel_, *position);
    if (axes && !sim_->machine->model.empty()) {
        QStringList parts;
        std::vector<std::string> names{"X", "Y", "Z"};
        for (const char c : sim_->machine->rotaryAxes()) {
            names.emplace_back(1, c);
        }
        for (const std::string& name : names) {
            const machine::Axis* axis = sim_->machine->axis(name);
            const bool rotary = name == "A" || name == "B" || name == "C";
            const double value = position->value(name) * (rotary ? 180.0 / kPi : 1.0);
            const bool over = axis && (value < axis->min - 1e-6 || value > axis->max + 1e-6);
            const QString text = rotary ? QStringLiteral("%1 %2°")
                                              .arg(QString::fromStdString(name))
                                              .arg(value, 0, 'f', 3)
                                        : QStringLiteral("%1 %2").arg(QString::fromStdString(name),
                                                                      formatLength(value, unit_));
            parts << (over ? QStringLiteral("<span style=\"color:#d04040\">%1</span>").arg(text)
                           : text);
        }
        *axes = parts.join(QStringLiteral(" &nbsp; "));
    }
    if (!machineParts_ || !showMachine_->isChecked()) {
        return std::nullopt;
    }
    // The work moves with the axis its placement puts it on (the WCS squared by mountedWcs).
    const Transform3 machineFromProject =
        pose.carried(machineModel_, op.mount->carrier) * op.mount->wcs * wcsToProject_.inverse();
    const bool fixed = machineFixed_->isChecked();
    const Transform3 viewFromMachine =
        fixed ? Transform3::Identity() : Transform3(machineFromProject.inverse());
    MachineAt out;
    out.scene.parts = machineParts_;
    for (const machine::ModelPart& p : *machineParts_) {
        out.scene.placements.push_back(viewFromMachine * pose.carried(machineModel_, p.carrier) *
                                       p.placement());
        out.scene.hidden.push_back(!partShown(p));
    }
    out.viewFromProject = fixed ? machineFromProject : Transform3::Identity();
    out.viewFromMachine = viewFromMachine;
    return out;
}

bool SimulationPanel::partShown(const machine::ModelPart& part) const {
    if (part.enclosure && !showEnclosure_->isChecked()) {
        return false; // the enclosure hidden as one
    }
    if (sim_ && sim_->machine) {
        if (const auto m = partShown_.find(sim_->machine->id); m != partShown_.end()) {
            if (const auto it = m->second.find(part.name); it != m->second.end()) {
                return it->second;
            }
        }
    }
    return part.shown;
}

void SimulationPanel::rebuildPartList() {
    const QSignalBlocker block(partList_);
    partList_->clear();
    if (!machineParts_) {
        return;
    }
    for (std::size_t i = 0; i < std::min(toolPartsFrom_, machineParts_->size()); ++i) {
        const machine::ModelPart& p = (*machineParts_)[i];
        auto* item = new QListWidgetItem(QString::fromStdString(p.name), partList_);
        item->setData(Qt::UserRole, QString::fromStdString(p.name));
        item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable);
        item->setCheckState(partShown(p) ? Qt::Checked : Qt::Unchecked);
    }
}

void SimulationPanel::setMachinePartShown(const std::string& name, bool shown) {
    if (!sim_ || !sim_->machine) {
        return;
    }
    partShown_[sim_->machine->id][name] = shown;
    rebuildPartList();
    requestFrame();
}

std::pair<Vec3, Vec3> SimulationPanel::toolPose(const Position& at) const {
    const auto& op = sim_->operations[static_cast<std::size_t>(at.op)];
    const auto& tips = startTips_[static_cast<std::size_t>(at.op)];
    Vec3 tip = tips[std::min(at.move, tips.size() - 1)];
    Vec3 axis = toolpath::axisBefore(op.toolpath, at.move);
    if (at.fraction > 0.0 && at.move < op.toolpath.moves.size()) {
        if (const auto part =
                toolpath::partialMove(tip, axis, op.toolpath.moves[at.move], at.fraction)) {
            tip = part->end;
            axis = part->axis;
        }
    }
    return {tip, axis};
}

SimulationPanel::PathsShown SimulationPanel::pathsShown() const {
    return static_cast<PathsShown>(pathsCombo_->currentData().toInt());
}

std::shared_ptr<const toolpath::Toolpath> SimulationPanel::trailAt(const Position& at) const {
    if (!sim_ || at.op < 0 || at.op >= static_cast<int>(sim_->operations.size()) ||
        at.op >= static_cast<int>(startLengths_.size()) || at.between >= 0.0) {
        return nullptr; // a lathe's machine between operations: the tool has come along none
    }
    const auto index = static_cast<std::size_t>(at.op);
    const toolpath::Toolpath& path = sim_->operations[index].toolpath;
    const std::vector<double>& lengths = startLengths_[index];
    const std::vector<Vec3>& tips = startTips_[index];
    const std::size_t n = path.moves.size();
    if (n == 0 || lengths.size() != n + 1 || tips.size() != n + 1) {
        return nullptr;
    }
    // From where the trail starts (move `first`, `firstFraction` of it) to the tool.
    const std::size_t end = std::min(at.move, n);
    const double endFraction = end < n ? std::clamp(at.fraction, 0.0, 1.0) : 0.0;
    const double here =
        lengths[end] + (end < n ? endFraction * (lengths[end + 1] - lengths[end]) : 0.0);
    std::size_t first = 0;
    double firstFraction = 0.0;
    if (trailLengthMm_ > 0.0 && here > trailLengthMm_) {
        const double from = here - trailLengthMm_;
        const auto it = std::upper_bound(
            lengths.begin(), lengths.begin() + static_cast<std::ptrdiff_t>(end) + 1, from);
        first = static_cast<std::size_t>(it - lengths.begin()) - 1;
        const double length = lengths[first + 1] - lengths[first];
        firstFraction = length > 0.0 ? std::clamp((from - lengths[first]) / length, 0.0, 1.0) : 0.0;
    }
    auto trail = std::make_shared<toolpath::Toolpath>();
    trail->cycles = path.cycles; // the moves that are cycles index them
    trail->probes = path.probes;
    // Move `k` from fraction t0 of it to t1 (cycles and dwells are not split: whole or none).
    auto add = [&](std::size_t k, double t0, double t1) {
        if (k >= n || t1 <= t0) {
            return;
        }
        const toolpath::Move& m = path.moves[k];
        Vec3 start = tips[k];
        Vec3 axis = toolpath::axisBefore(path, k);
        if (t0 > 0.0) {
            const auto head = toolpath::partialMove(start, axis, m, t0);
            if (!head) {
                return;
            }
            start = head->end;
            axis = head->axis;
        }
        // The rest of a move is the same move from where it got to.
        const std::optional<toolpath::Move> piece =
            t0 <= 0.0 && t1 >= 1.0 ? std::optional<toolpath::Move>(m)
                                   : toolpath::partialMove(start, axis, m, (t1 - t0) / (1.0 - t0));
        if (!piece) {
            return;
        }
        if (trail->moves.empty()) {
            trail->start = start;
        }
        trail->moves.push_back(*piece);
    };
    if (first == end) {
        add(first, firstFraction, endFraction);
    } else {
        add(first, firstFraction, 1.0);
        for (std::size_t k = first + 1; k < end; ++k) {
            add(k, 0.0, 1.0);
        }
        add(end, 0.0, endFraction);
    }
    if (trail->moves.empty()) {
        return nullptr;
    }
    return trail;
}

void SimulationPanel::updateMoveLabel() {
    const int op = opCombo_->currentIndex();
    if (sim_ && playTimer_->isActive() && playAt_.op == op && playAt_.between >= 0.0) {
        moveLabel_->setText(tr("Before the first move"));
        return;
    }
    moveLabel_->setText(sim_ ? tr("%1 of %2").arg(slider_->value()).arg(movesOf(op)) : QString());
}

void SimulationPanel::play() {
    if (!playTimer_->isActive()) {
        togglePlay();
    }
}

void SimulationPanel::pause() {
    if (playTimer_->isActive()) {
        togglePlay();
    }
}

bool SimulationPanel::playing() const {
    return playTimer_->isActive();
}

std::pair<int, std::size_t> SimulationPanel::position() const {
    return {opCombo_->currentIndex(), static_cast<std::size_t>(std::max(0, slider_->value()))};
}

void SimulationPanel::togglePlay() {
    if (playTimer_->isActive()) {
        playTimer_->stop();
        playButton_->setText(QStringLiteral("▶"));
        return;
    }
    if (!sim_ || sim_->operations.empty()) {
        return;
    }
    const int op = opCombo_->currentIndex();
    if (op == static_cast<int>(sim_->operations.size()) - 1 &&
        static_cast<std::size_t>(slider_->value()) >= movesOf(op)) {
        // At the end: restart from the beginning.
        const QSignalBlocker blockCombo(opCombo_);
        const QSignalBlocker blockSlider(slider_);
        opCombo_->setCurrentIndex(0);
        slider_->setRange(0, static_cast<int>(movesOf(0)));
        slider_->setValue(0);
    }
    syncPlayTime();
    playClock_.start();
    playTimer_->start();
    playButton_->setText(QStringLiteral("⏸"));
}

void SimulationPanel::step(int direction) {
    if (!sim_ || sim_->operations.empty()) {
        return;
    }
    const int count = static_cast<int>(sim_->operations.size());
    const int op = opCombo_->currentIndex();
    if (direction < 0) {
        if (slider_->value() > 0) {
            slider_->setValue(0);
        } else if (op > 0) {
            opCombo_->setCurrentIndex(op - 1);
            slider_->setValue(0);
        }
    } else if (static_cast<std::size_t>(slider_->value()) < movesOf(op)) {
        slider_->setValue(static_cast<int>(movesOf(op)));
    } else if (op + 1 < count) {
        opCombo_->setCurrentIndex(op + 1);
    }
}

void SimulationPanel::stepMove(int direction) {
    if (!sim_ || sim_->operations.empty()) {
        return;
    }
    if (playTimer_->isActive()) {
        togglePlay(); // pause
    }
    int op = std::max(0, opCombo_->currentIndex());
    int move = slider_->value() + direction;
    if (move < 0 && op > 0) {
        --op; // the end of the previous operation
        move = static_cast<int>(movesOf(op)) - 1;
    } else if (move > static_cast<int>(movesOf(op)) &&
               op + 1 < static_cast<int>(sim_->operations.size())) {
        ++op; // the first move of the next operation
        move = 1;
    }
    showPosition(op, static_cast<std::size_t>(std::max(0, move)));
}

void SimulationPanel::playTick() {
    if (!sim_ || sim_->operations.empty()) {
        return;
    }
    // The tool is shown with the stock it has cut: while a frame is being computed the clock
    // waits, so slow frames slow the playback down instead of letting the tool run ahead.
    if (frameRunning_) {
        return;
    }
    const double dt = std::min(static_cast<double>(playClock_.restart()) / 1000.0, 0.05);
    double t = playTime_ + dt * speedCombo_->currentData().toDouble();
    int op = std::max(0, opCombo_->currentIndex());
    while (t >= durationOf(op)) {
        if (op + 1 >= static_cast<int>(sim_->operations.size())) {
            // The end: show the final state.
            playTimer_->stop();
            playButton_->setText(QStringLiteral("▶"));
            {
                const QSignalBlocker block(slider_);
                slider_->setValue(static_cast<int>(movesOf(op)));
            }
            syncPlayTime();
            updateMoveLabel();
            requestFrame();
            return;
        }
        t -= durationOf(op);
        ++op;
        const QSignalBlocker blockCombo(opCombo_);
        const QSignalBlocker blockSlider(slider_);
        opCombo_->setCurrentIndex(op);
        slider_->setRange(0, static_cast<int>(movesOf(op)));
    }
    playTime_ = t;
    playAt_ = positionAt(op, t);
    {
        const QSignalBlocker block(slider_);
        slider_->setValue(static_cast<int>(playAt_.move));
    }
    updateMoveLabel();
    requestFrame(playAt_); // the frame shows the tool and the stock together
}

void SimulationPanel::setDockShown(bool shown) {
    dockShown_ = shown;
    updateSceneShown();
}

void SimulationPanel::setSuppressed(bool suppressed) {
    suppressed_ = suppressed;
    updateSceneShown();
}

void SimulationPanel::updatePreviewBusy() {
    QString text;
    if (previewActive_ && !dockShown_ && !suppressed_ && (simRunning_ || frameRunning_)) {
        text = simRunning_ && simProgress_
                   ? tr("Updating in-process stock… %1%").arg(qRound(simProgress_->load() * 100.0))
                   : tr("Updating in-process stock…");
    }
    if (text != previewBusyText_) {
        previewBusyText_ = text;
        emit previewBusyChanged(text);
    }
}

void SimulationPanel::updateSceneShown() {
    updatePreviewBusy();
    const bool shown = (dockShown_ || previewActive_) && !suppressed_;
    if (shown == sceneShown_) {
        return;
    }
    sceneShown_ = shown;
    if (!shown) {
        view_->setSimulation(std::nullopt); // the model and toolpaths come back
    } else if (sim_) {
        cached_.reset();
        requestFrame();
    }
}

document::SimulationOptions SimulationPanel::baseOptions() const {
    document::SimulationOptions options;
    options.resolution = resolutionMm_;
    return options;
}

std::shared_ptr<sim::StockDisplay> SimulationPanel::makeDisplay() const {
    sim::DisplayOptions displayOptions;
    displayOptions.maxCells = 800; // fine rays show as a fine lattice
    displayOptions.stockColour = stockColour_;
    return std::make_shared<sim::StockDisplay>(displayOptions);
}

void SimulationPanel::setStockColour(const QColor& colour) {
    const sim::Rgb rgb{static_cast<std::uint8_t>(colour.red()),
                       static_cast<std::uint8_t>(colour.green()),
                       static_cast<std::uint8_t>(colour.blue())};
    if (rgb == stockColour_) {
        return;
    }
    stockColour_ = rgb;
    updateLegend();
    if (sim_) {
        // A new display rebuilds the stock shown now in the new colour.
        display_ = makeDisplay();
        const std::optional<Position> at = lastFrame_;
        lastFrame_.reset();
        requestFrame(at);
    }
}

void SimulationPanel::requestFrame(std::optional<Position> at) {
    if (!sim_) {
        return;
    }
    const Position target =
        at ? *at
           : Position{std::max(0, opCombo_->currentIndex()),
                      static_cast<std::size_t>(std::max(0, slider_->value())), 0.0};
    if (frameRunning_) {
        pending_ = target;
        return;
    }
    startFrame(target);
}

void SimulationPanel::startFrame(Position target) {
    if (!sim_) {
        return;
    }
    frameRunning_ = true;
    updatePreviewBusy();
    pending_.reset();
    auto sim = sim_;
    const bool hasOps = !sim->operations.empty();
    target.op = hasOps ? std::clamp(target.op, 0, static_cast<int>(sim->operations.size()) - 1) : 0;
    target.move = std::min(target.move, movesOf(target.op));
    if (target.move >= movesOf(target.op)) {
        target.fraction = 0.0;
    }
    const bool compare = compare_->isChecked() && !sim->part.isEmpty();
    // The display continues from the previous frame when moving forward within the same
    // operation with the same colouring; anything else starts it again.
    const bool continuing =
        lastFrame_ && lastFrame_->op == target.op &&
        (lastFrame_->move < target.move ||
         (lastFrame_->move == target.move && lastFrame_->fraction <= target.fraction)) &&
        lastCompare_ == compare;
    lastFrame_ = target;
    lastCompare_ = compare;
    auto display = display_;

    // Start from the cached frame when moving forward within the same operation.
    std::shared_ptr<const sim::Stock> base;
    std::shared_ptr<const sim::LatheStock> baseLathe;
    std::size_t from = 0;
    double fromFraction = 0.0; // turning frames are cached part-way through a move too
    if (cached_ && cached_->at.op == target.op &&
        (cached_->at.move < target.move ||
         (cached_->at.move == target.move && cached_->at.fraction <= target.fraction))) {
        base = cached_->stock;
        baseLathe = cached_->lathe;
        from = cached_->at.move;
        fromFraction = cached_->at.fraction;
    }
    std::shared_ptr<const std::vector<std::uint8_t>> cutBase;
    if (const auto it = cutBase_.find(target.op); it != cutBase_.end()) {
        cutBase = it->second;
    }
    struct Result {
        std::shared_ptr<const std::vector<std::uint8_t>> cutBase; ///< Computed for this op.
        /// The cache: after whole moves (milling), or the frame itself (turning).
        std::shared_ptr<const sim::Stock> base;
        std::shared_ptr<const sim::LatheStock> baseLathe; ///< Turning: the same, axisymmetric.
        bool done = false;
        std::vector<std::shared_ptr<const sim::DisplayMesh>> chunks;
        double volume = 0.0;
        double initialVolume = 0.0;
        std::optional<sim::Comparison> comparison;
        QString error;
    };
    auto result = std::make_shared<Result>();
    runInBackground(
        [sim, target, base, baseLathe, from, fromFraction, compare, hasOps, result, cutBase,
         display, continuing, stockColour = stockColour_] {
            try {
                if (sim->playsOnProfile(hasOps ? static_cast<std::size_t>(target.op) : 0)) {
                    // Turning, drawn as this frame's exact profile revolved and compared in the
                    // lathe plane: the dexel stock plays no part.
                    std::shared_ptr<const sim::LatheStock> lathe = sim->latheInitial;
                    if (hasOps) {
                        const auto& op = sim->operations[static_cast<std::size_t>(target.op)];
                        if (target.move >= op.toolpath.moves.size() ||
                            (op.insert.isEmpty() && op.holeProfile.empty())) {
                            lathe = op.latheAfter;
                        } else {
                            const auto before =
                                baseLathe ? baseLathe
                                          : sim->latheBefore(static_cast<std::size_t>(target.op));
                            lathe = std::make_shared<const sim::LatheStock>(turnedTo(
                                op, *before, baseLathe ? from : 0, baseLathe ? fromFraction : 0.0,
                                target.move, target.fraction));
                        }
                    }
                    result->baseLathe = lathe;
                    result->chunks = {std::make_shared<const sim::DisplayMesh>(
                        document::latheDisplayMesh(*sim, static_cast<std::size_t>(target.op),
                                                   *lathe, compare, stockColour))};
                    if (compare) {
                        result->comparison = document::latheComparison(*sim, *lathe);
                    }
                    result->volume = lathe->volume();
                    result->initialVolume = sim->latheInitial->volume();
                    result->done = true;
                    return;
                }
                sim::Stock stock;
                if (!hasOps) {
                    stock = sim->initialStock;
                } else {
                    const auto& op = sim->operations[static_cast<std::size_t>(target.op)];
                    if (target.move >= op.toolpath.moves.size()) {
                        stock = op.stockAfter;
                    } else {
                        stock =
                            base ? *base : sim->stockBefore(static_cast<std::size_t>(target.op));
                        // Turning what a live tool has milled: the stock before it, clipped to
                        // this frame's profile.
                        if (!op.insert.isEmpty()) {
                            const auto index = static_cast<std::size_t>(target.op);
                            stock = sim->stockBefore(index);
                            sim::clipToLathe(stock, turnedTo(op, *sim->latheBefore(index), 0, 0.0,
                                                             target.move, target.fraction));
                        } else if (op.tool && target.move > from && op.toolpath.strategy != "tap") {
                            sim::SimulateOptions options;
                            options.checkCollisions = false;
                            sim::simulateMoves(stock, *op.tool, op.toolpath, base ? from : 0,
                                               target.move, options);
                        }
                    }
                }
                result->base = std::make_shared<const sim::Stock>(stock);
                if (hasOps && target.fraction > 0.0) {
                    // Part of the next move, so the stock follows the tool between moves.
                    const auto& op = sim->operations[static_cast<std::size_t>(target.op)];
                    if (op.tool && target.move < op.toolpath.moves.size() &&
                        op.toolpath.strategy != "tap") {
                        toolpath::Toolpath piece;
                        piece.start = sim::toolPositionAt(op.toolpath, target.move);
                        if (const auto part = toolpath::partialMove(
                                piece.start, toolpath::axisBefore(op.toolpath, target.move),
                                op.toolpath.moves[target.move], target.fraction)) {
                            piece.moves.push_back(*part);
                            sim::SimulateOptions options;
                            options.checkCollisions = false;
                            sim::simulateMoves(stock, *op.tool, piece, 0, 1, options);
                        }
                    }
                }
                // The part was rasterised on the stock grid once by the simulation.
                const sim::PartReference* part =
                    compare && sim->partReference && sim->partReference->matches(stock.grid(2))
                        ? sim->partReference.get()
                        : nullptr;
                const int cutting = hasOps ? target.op : -1;
                // The display colours each point by the operation that cut it, from
                // the stocks before each operation.
                std::vector<const sim::Stock*> starts;
                for (int j = 0; hasOps && j <= target.op; ++j) {
                    starts.push_back(&sim->stockBefore(static_cast<std::size_t>(j)));
                }
                if (continuing) {
                    // Forward from the previous frame: only what was cut since.
                    display->update(stock, part, cutting, false, nullptr, &starts);
                } else {
                    // A new starting point: which operation last cut each ray, from scratch.
                    std::vector<std::uint8_t> cutBy;
                    if (hasOps) {
                        std::shared_ptr<const std::vector<std::uint8_t>> earlier = cutBase;
                        if (!earlier) {
                            auto made = std::make_shared<std::vector<std::uint8_t>>();
                            for (std::size_t j = 0; j < static_cast<std::size_t>(target.op); ++j) {
                                const auto changed = sim::changedRays(
                                    sim->stockBefore(j), sim->operations[j].stockAfter, true);
                                if (made->empty()) {
                                    made->assign(changed.size(), 0);
                                }
                                if (changed.size() == made->size()) {
                                    for (std::size_t k = 0; k < changed.size(); ++k) {
                                        if (changed[k]) {
                                            (*made)[k] = static_cast<std::uint8_t>(
                                                std::min<std::size_t>(j + 1, 255));
                                        }
                                    }
                                }
                            }
                            earlier = made;
                            result->cutBase = made;
                        }
                        const auto changed = sim::changedRays(
                            sim->stockBefore(static_cast<std::size_t>(target.op)), stock, true);
                        cutBy = earlier->size() == changed.size()
                                    ? *earlier
                                    : std::vector<std::uint8_t>(changed.size(), 0);
                        for (std::size_t k = 0; k < changed.size(); ++k) {
                            if (changed[k]) {
                                cutBy[k] = static_cast<std::uint8_t>(std::min(target.op + 1, 255));
                            }
                        }
                    }
                    display->update(stock, part, cutting, true, hasOps ? &cutBy : nullptr, &starts);
                }
                result->chunks = display->chunks();
                result->comparison = display->comparison();
                result->volume = stock.volume();
                result->initialVolume = sim->initialStock.volume();
                result->done = true;
            } catch (const std::exception& e) {
                result->error = QString::fromUtf8(e.what());
            }
        },
        [this, sim, target, result] {
            frameRunning_ = false;
            updatePreviewBusy();
            if (sim != sim_) {
                return; // a newer simulation replaced this one
            }
            if (result->cutBase) {
                cutBase_[target.op] = result->cutBase;
            }
            if (result->done) {
                ++frameCount_;
                // Turning frames are cached as they are, part of a move included.
                cached_ = Cached{
                    Position{target.op, target.move, result->baseLathe ? target.fraction : 0.0},
                    result->base, result->baseLathe};
                OcctView::SimulationScene scene;
                scene.stock = result->chunks;
                scene.wcsToProject = wcsToProject_;
                scene.showModels = showPart_->isChecked();
                scene.showFixtures = showFixtures_->isChecked();
                // The in-process stock preview always keeps the toolpaths; the simulation's
                // Paths choice applies to the simulation.
                scene.showToolpaths = !dockShown_ || pathsShown() == PathsShown::All;
                if (dockShown_ && pathsShown() == PathsShown::Trail) {
                    scene.trail = trailAt(target);
                }
                if (!sim->operations.empty()) {
                    const auto& op = sim->operations[static_cast<std::size_t>(target.op)];
                    if (showTool_->isChecked() && dockShown_) {
                        if (op.tool) {
                            scene.tool = op.tool;
                        } else if (!op.insert.isEmpty()) {
                            // Turning: the insert, kept alive (and its identity kept) by the
                            // simulation.
                            scene.insert = std::shared_ptr<const geom2d::Polyline>(sim, &op.insert);
                            scene.insertThickness = op.insertThickness;
                            scene.toolParts =
                                std::shared_ptr<const std::vector<tooling::TurningToolPart>>(
                                    sim, &op.toolParts);
                        }
                        std::tie(scene.toolTip, scene.toolAxis) = toolPose(target);
                    }
                }
                for (const auto& op : sim->operations) {
                    for (const auto& c : op.collisions) {
                        scene.collisions.push_back(c.position);
                    }
                    for (const auto& c : op.fixtureCollisions) {
                        scene.collisions.push_back(c.position);
                    }
                    for (const auto& c : op.machineCollisions) {
                        scene.collisions.push_back(c.position);
                    }
                }
                QString axesText;
                Transform3 viewFromMachine = Transform3::Identity();
                if (const auto m = machineAt(target, &axesText); m && dockShown_) {
                    scene.machine = m->scene;
                    scene.viewFromProject = m->viewFromProject;
                    viewFromMachine = m->viewFromMachine;
                    scene.modelMoves = m->modelMoves;
                    if (sim->machine && sim->machine->lathe) {
                        // Every tool is on the turret, the one cutting too.
                        scene.tool.reset();
                        scene.insert.reset();
                        scene.toolParts.reset();
                    }
                }
                axes_->setText(axesText);
                if (sceneShown_) {
                    const bool fit = fitMachine_ && scene.machine;
                    view_->setSimulation(scene);
                    if (fit) {
                        // The machine came into the view: show all of it once. A lathe from
                        // its door, the spindle on the left (DL-0077: as the machine stands).
                        if (sim->machine && sim->machine->lathe) {
                            const Mat3 world =
                                viewFromMachine.linear() * sim->machine->standing().worldAxes();
                            view_->lookFrom(world * Vec3(-0.3, -1.0, 0.45), world.col(2));
                        }
                        view_->fitAll();
                        fitMachine_ = false;
                    }
                }

                auto volumeText = [this](double mm3) {
                    return formatVolume(mm3, unit_);
                };
                QString text = tr("Stock: %1").arg(volumeText(result->volume));
                text += tr("\nRemoved so far: %1")
                            .arg(volumeText(result->initialVolume - result->volume));
                if (!sim->operations.empty()) {
                    const auto& op = sim->operations[static_cast<std::size_t>(target.op)];
                    text += tr("\n%1 removes %2")
                                .arg(QString::fromStdString(op.name), volumeText(op.removedVolume));
                }
                if (result->comparison) {
                    const auto& c = *result->comparison;
                    text += c.hasGouges() ? tr("\nGouges: %1 (deepest %2)")
                                                .arg(volumeText(c.gougeVolume),
                                                     formatLength(c.maxGouge, unit_))
                                          : tr("\nNo gouges");
                    text +=
                        c.excessRays > 0
                            ? tr("\nStock left: %1 (thickest %2)")
                                  .arg(volumeText(c.excessVolume), formatLength(c.maxExcess, unit_))
                            : tr("\nNo stock left beyond %1").arg(formatLength(c.tolerance, unit_));
                }
                stats_->setText(text);
                legend_->setVisible(true);
            } else if (!result->error.isEmpty()) {
                stats_->setText(result->error);
            }
            if (pending_ && *pending_ != target) {
                startFrame(*pending_);
            } else {
                pending_.reset();
            }
        });
}

void SimulationPanel::updateLegend() {
    // One colour per operation for the material it cut; green and red for the comparison.
    QStringList parts;
    if (sim_) {
        for (std::size_t i = 0; i < sim_->operations.size(); ++i) {
            parts << QStringLiteral("%1 %2").arg(
                swatch(sim::operationColour(static_cast<int>(i))),
                QString::fromStdString(sim_->operations[i].name).toHtmlEscaped());
        }
    }
    parts << tr("%1 at size").arg(swatch(sim::comparisonColour(0, 0, 0.02, 2)))
          << tr("%1 gouge").arg(swatch(sim::comparisonColour(0, 1, 0.02, 2)))
          << tr("%1 uncut stock").arg(swatch(stockColour_));
    legend_->setText(parts.join(QStringLiteral(" &nbsp; ")));
}

void SimulationPanel::updateStatus() {
    if (!sim_) {
        status_->setText(tr("No simulation."));
        return;
    }
    QString text = tr("%1 operation(s) at %2 rays, %3 collision(s)")
                       .arg(sim_->operations.size())
                       .arg(formatLength(sim_->resolution, unit_))
                       .arg(sim_->collisionCount());
    if (sim_->machineCollisionCount() > 0) {
        text += tr(", %1 machine collision(s)").arg(sim_->machineCollisionCount());
    }
    if (sim_->fixtureCollisionCount() > 0) {
        text += tr(", %1 workholding collision(s)").arg(sim_->fixtureCollisionCount());
    }
    for (const auto& w : sim_->warnings) {
        text += QStringLiteral("\n") + QString::fromStdString(w);
    }
    if (stale_) {
        text += tr("\nThe project changed: simulate again to update.");
    }
    status_->setText(text);
}

void SimulationPanel::setTrailLengthMm(double mm) {
    trailLength_->setValue(mm / (unit_ == LengthUnit::Inch ? 25.4 : 1.0));
}

void SimulationPanel::setDisplayUnit(LengthUnit unit) {
    unit_ = unit;
    if (!resolution_) {
        return;
    }
    const bool inch = unit == LengthUnit::Inch;
    const double factor = inch ? 25.4 : 1.0;
    const QSignalBlocker block(resolution_);
    resolution_->setDecimals(inch ? 4 : 3);
    resolution_->setRange(0.0, 5.0 / factor);
    resolution_->setSingleStep(inch ? 0.001 : 0.025);
    resolution_->setSuffix(lengthSuffix(unit));
    resolution_->setValue(resolutionMm_ / factor);
    if (trailLength_) {
        const QSignalBlocker blockTrail(trailLength_);
        trailLength_->setDecimals(inch ? 3 : 1);
        trailLength_->setRange(0.0, 10000.0 / factor);
        trailLength_->setSingleStep(inch ? 0.5 : 10.0);
        trailLength_->setSuffix(lengthSuffix(unit));
        trailLength_->setValue(trailLengthMm_ / factor);
    }
}

void SimulationPanel::markStale() {
    if (sim_ && !stale_) {
        stale_ = true;
        updateStatus();
    }
}

bool SimulationPanel::simulationValid(const document::Document& doc,
                                      const document::Setup& setup) const {
    if (!sim_ || sim_->setupId != setup.id || simRunning_) {
        return false;
    }
    document::SimulationOptions options = baseOptions();
    options.lastOperation = simLastOperation_;
    return document::simulationInputKey(doc, setup, options) == simKey_;
}

bool SimulationPanel::showOperationEnd(const Uuid& operationId) {
    if (!sim_) {
        return false;
    }
    for (std::size_t i = 0; i < sim_->operations.size(); ++i) {
        if (sim_->operations[i].operationId == operationId) {
            playTimer_->stop();
            playButton_->setText(QStringLiteral("▶"));
            showPosition(static_cast<int>(i), movesOf(static_cast<int>(i)));
            return true;
        }
    }
    return false;
}

void SimulationPanel::previewOperation(const Uuid& setupId, const Uuid& operationId) {
    previewActive_ = true;
    previewOperation_ = operationId;
    updateSceneShown();
    auto doc = snapshot_();
    const document::Setup* setup = doc ? doc->project().findSetup(setupId) : nullptr;
    if (!setup) {
        return;
    }
    // A simulation that is still valid and reaches this operation shows it at once (the one
    // shown, or a recent one of this setup).
    if (simulationValid(*doc, *setup) && showOperationEnd(operationId)) {
        return;
    }
    if (showRecent(*doc, *setup, std::nullopt, operationId)) {
        return;
    }
    // The whole setup, once for all its operations (and kept in the local cache); a disabled
    // operation shows the stock of the enabled ones before it.
    const document::Operation* op = doc->project().findOperation(operationId);
    simulate(setupId, op && op->enabled ? std::nullopt : std::optional<Uuid>(operationId));
}

void SimulationPanel::remember(Recent entry) {
    std::erase_if(recent_, [&](const Recent& r) {
        return r.setupId == entry.setupId && r.lastOperation == entry.lastOperation;
    });
    recent_.push_front(std::move(entry));
    if (recent_.size() > kRecent) {
        recent_.pop_back();
    }
}

bool SimulationPanel::showRecent(const document::Document& doc, const document::Setup& setup,
                                 const std::optional<Uuid>& lastOperation,
                                 const std::optional<Uuid>& reaching) {
    if (simRunning_) {
        return false;
    }
    for (auto it = recent_.begin(); it != recent_.end(); ++it) {
        if (it->setupId != setup.id) {
            continue;
        }
        if (reaching) {
            const auto& ops = it->sim->operations;
            if (std::none_of(ops.begin(), ops.end(), [&](const document::OperationSimulation& o) {
                    return o.operationId == *reaching;
                })) {
                continue;
            }
        } else if (it->lastOperation != lastOperation) {
            continue;
        }
        document::SimulationOptions options = baseOptions();
        options.lastOperation = it->lastOperation;
        if (document::simulationInputKey(doc, setup, options) != it->key) {
            continue; // something it depends on changed
        }
        Recent entry = *it;
        recent_.erase(it);
        recent_.push_front(entry);
        wcsToProject_ = setup.wcs.toParent();
        simKey_ = entry.key;
        simLastOperation_ = entry.lastOperation;
        simulationDone(entry.sim, {});
        if (reaching) {
            showOperationEnd(*reaching);
        }
        return true;
    }
    return false;
}

void SimulationPanel::stopPreview() {
    if (!previewActive_) {
        return;
    }
    previewActive_ = false;
    previewOperation_.reset();
    updateSceneShown();
}

void SimulationPanel::closeSimulation() {
    cancelSimulation();
    playTimer_->stop();
    playButton_->setText(QStringLiteral("▶"));
    sim_.reset();
    cached_.reset();
    collisions_->clear();
    stats_->clear();
    progress_->setValue(0);
    view_->setSimulation(std::nullopt);
    rebuildControls();
    updateStatus();
}

} // namespace ocam::gui
