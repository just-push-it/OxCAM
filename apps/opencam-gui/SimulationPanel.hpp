// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Simulation dock: runs the stock simulation of a setup in the background and plays it back
// (operation checkpoints plus a move slider), with the part comparison colour map, the tool,
// statistics, the list of collisions and the machine moving with its axes (DL-0037).

#pragma once

#include "OcctView.hpp"

#include <ocam/document/Simulate.hpp>
#include <ocam/machine/Model.hpp>

#include <QElapsedTimer>
#include <QWidget>
#include <atomic>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <vector>

class QCheckBox;
class QColor;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QListWidget;
class QProgressBar;
class QPushButton;
class QSlider;
class QThread;
class QTimer;
class QToolButton;

namespace ocam::gui {

class InFlightSimulations;
struct StockSourceContext;

class SimulationPanel : public QWidget {
    Q_OBJECT

public:
    using SnapshotFn = std::function<std::shared_ptr<const document::Document>()>;
    /// Up-to-date generated toolpaths of a setup's operations (by operation id), so the
    /// simulation does not generate them again.
    using ToolpathsFn = std::function<std::map<Uuid, std::shared_ptr<const toolpath::Toolpath>>(
        const document::Document&, const Uuid& setupId)>;

    SimulationPanel(OcctView* view, SnapshotFn snapshot, ToolpathsFn toolpaths,
                    QWidget* parent = nullptr);
    ~SimulationPanel() override;

    /// Simulates a setup of the current document (replacing any shown simulation), or only up to
    /// and including `lastOperation`.
    void simulate(const Uuid& setupId, std::optional<Uuid> lastOperation = std::nullopt);
    /// Shows the in-process stock after an operation (and everything before it), using a
    /// simulation already computed when it is still valid. The stock shows even with the dock
    /// hidden until stopPreview().
    void previewOperation(const Uuid& setupId, const Uuid& operationId);
    void stopPreview();
    bool previewing() const noexcept { return previewActive_; }
    /// The setup that the Simulate button runs.
    void setCurrentSetup(const std::optional<Uuid>& setupId);
    /// Works out these setups' whole simulations in the background, one at a time (from the
    /// local simulation cache when it has them), so their operations' in-process stock shows at
    /// once. A simulation the user asks for comes first; one of a setup being prepared is not
    /// started again but waited for.
    void prepareSetups(const std::vector<Uuid>& setupIds);
    /// True while setups are being prepared.
    bool preparing() const noexcept { return prepareRunning_ || !prepareQueue_.empty(); }
    /// The document changed: the shown simulation may no longer match it.
    void markStale();
    /// Removes the simulation from the view.
    void closeSimulation();
    /// Shows the end of the last operation with the comparison colours.
    void showFinal();
    /// Shows the stock after `move` moves of operation `op` (clamped).
    void showPosition(int op, std::size_t move);
    /// Playback, as the play button starts and pauses it.
    void play();
    void pause();
    bool playing() const;
    /// The position shown on the controls: the operation and the moves done in it.
    std::pair<int, std::size_t> position() const;
    /// Stock frames shown since the count was reset (tests: playback moves on).
    int frameCount() const noexcept { return frameCount_; }
    void resetFrameCount() noexcept { frameCount_ = 0; }

    /// Unit of the lengths and volumes shown.
    void setDisplayUnit(LengthUnit unit);
    /// The trail's length in millimetres (0: all of the operation), shown in the display unit.
    void setTrailLengthMm(double mm);
    /// The colour of uncut material; the stock shown now is drawn again in it.
    void setStockColour(const QColor& colour);
    /// The simulation dock was shown or hidden: the simulation leaves the 3D view (kept) or comes
    /// back. Previews keep showing either way.
    void setDockShown(bool shown);

    /// True while the simulation or a playback frame is being computed.
    bool busy() const noexcept { return simRunning_ || frameRunning_; }
    /// How far the simulation running has got, 0 to 1 (0 when none runs).
    double simulationProgress() const noexcept {
        return simRunning_ && simProgress_ ? simProgress_->load() : 0.0;
    }
    bool hasSimulation() const noexcept { return static_cast<bool>(sim_); }

    /// While suppressed (picking geometry) the simulated stock leaves the view.
    void setSuppressed(bool suppressed);

    /// Shows or hides a part of the simulated machine in the view for this session (it stays in
    /// the collision checks). Parts start as their machine has them (ModelPart::shown).
    void setMachinePartShown(const std::string& name, bool shown);

signals:
    void simulationFinished(bool success);
    /// Close was pressed: leave simulation mode.
    void closed();
    /// The in-process stock preview is (non-empty text, for the view) or is no longer being
    /// computed (empty).
    void previewBusyChanged(const QString& text);

private:
    /// A playback position: `move` moves of operation `op` done, and `fraction` of the next.
    struct Position {
        int op = 0;
        std::size_t move = 0;
        double fraction = 0.0;
        /// A lathe's machine between operations (DL-0067): seconds into the moves before this
        /// operation's first, when not negative.
        double between = -1.0;
        bool operator==(const Position&) const = default;
    };

    template <typename Work, typename Done> void runInBackground(Work work, Done done);

    void cancelSimulation();
    void simulationDone(std::shared_ptr<const document::SetupSimulation> sim, const QString& error);
    void rebuildControls();
    void operationChanged(int index);
    void sliderChanged(int value);
    void togglePlay();
    void step(int direction);
    /// One move forward or back (crossing into the neighbouring operation), paused.
    void stepMove(int direction);
    void playTick();
    /// Starts a stock frame for `at`, or the slider position.
    void requestFrame(std::optional<Position> at = std::nullopt);
    void startFrame(Position target);
    std::size_t movesOf(int op) const;
    /// Machine time of an operation [s] and the position at time `t` into it.
    double durationOf(int op) const;
    Position positionAt(int op, double t) const;
    /// The tool tip and axis at a position (WCS), interpolated within the move.
    std::pair<Vec3, Vec3> toolPose(const Position& at) const;
    /// The machine at a position: the scene of its parts and the project frame in the view
    /// (nullopt without a machine to show); `axes` gets the axis positions (machines with their
    /// own model).
    struct MachineAt {
        OcctView::MachineScene scene;
        Transform3 viewFromProject = Transform3::Identity();
        Transform3 viewFromMachine = Transform3::Identity(); ///< The machine's frame in the view.
        std::map<Uuid, Transform3> modelMoves;               ///< A lathe's tailstock on its way in.
    };
    std::optional<MachineAt> machineAt(const Position& at, QString* axes) const;
    /// The same on a lathe with a layout: the axes come from the turret's working tool.
    std::optional<MachineAt> latheMachineAt(const Position& at, QString* axes) const;
    /// How long the machine takes between operations before this one [s].
    double betweenDuration(int op) const;
    /// Puts the playback clock at the slider position.
    void syncPlayTime();
    void updateMoveLabel();
    void updateStatus();

    /// The machine's parts from this index on are the tools in a lathe's turret (toggled with the
    /// Tool box, not listed with the machine's parts).
    std::size_t toolPartsFrom_ = 0;
    OcctView* view_;
    SnapshotFn snapshot_;
    ToolpathsFn toolpaths_;
    /// simulationInputKey of `sim_`: simulating an unchanged setup again reuses it.
    std::string simKey_;
    LengthUnit unit_ = LengthUnit::Millimeter;
    bool sceneShown_ = false; // the dock starts hidden
    bool dockShown_ = false;
    bool suppressed_ = false;
    bool previewActive_ = false;
    std::optional<Uuid> previewOperation_;
    std::optional<Uuid> simLastOperation_; ///< The `lastOperation` sim_ was computed with.
    void updateSceneShown();
    void updatePreviewBusy();
    QString previewBusyText_;
    /// sim_ still matches the document (with the options it was computed with).
    bool simulationValid(const document::Document& doc, const document::Setup& setup) const;
    /// Shows the end of operation `operationId` of sim_ (false when sim_ does not have it).
    bool showOperationEnd(const Uuid& operationId);

    std::shared_ptr<const document::SetupSimulation> sim_;
    /// Recent simulations, latest first: going back to a setup (or an operation it reaches)
    /// shows its simulation at once while nothing it depends on changed.
    struct Recent {
        Uuid setupId;
        std::optional<Uuid> lastOperation; ///< As simulated (nullopt: the whole setup).
        std::string key;                   ///< simulationInputKey when simulated.
        std::shared_ptr<const document::SetupSimulation> sim;
    };
    std::deque<Recent> recent_;
    /// Whole-setup simulations being worked out (the user's, preparations, and the setups a
    /// stock comes from), so each is worked out once.
    std::shared_ptr<InFlightSimulations> inFlight_;
    /// Options of a simulation of the setup on the GUI's toolpaths, with the setups its stock
    /// comes from taken from recent_, the cache or the simulations in flight. `context` collects
    /// the ones worked out on the way (rememberSources when done).
    document::SimulationOptions
    simulationOptions(const std::shared_ptr<const document::Document>& doc,
                      const document::Setup& setup, std::optional<Uuid> lastOperation,
                      std::shared_ptr<StockSourceContext>& context) const;
    void rememberSources(StockSourceContext& context);
    static constexpr std::size_t kRecent = 6;
    // Background preparation (prepareSetups): the setups still to do, the one running and its
    // cancel token and progress. When the user asks for the setup being prepared, the panel
    // takes it over (adopted) and shows it when it is done.
    std::deque<Uuid> prepareQueue_;
    bool prepareRunning_ = false;
    Uuid prepareSetup_;
    std::string prepareKey_;
    std::shared_ptr<CancelToken> prepareCancel_;
    std::shared_ptr<std::atomic<double>> prepareProgress_;
    bool prepareAdopted_ = false;
    Transform3 prepareWcs_ = Transform3::Identity();
    void startNextPrepare();
    /// The recent whole simulation of the setup, when still valid.
    bool hasValidWhole(const document::Document& doc, const document::Setup& setup) const;
    void remember(Recent entry);
    /// Makes a recent simulation of `setup` that is still valid the current one: the one
    /// computed up to `lastOperation`, or with `reaching` set any that has that operation (then
    /// shown at its end). False when there is none.
    bool showRecent(const document::Document& doc, const document::Setup& setup,
                    const std::optional<Uuid>& lastOperation, const std::optional<Uuid>& reaching);
    Transform3 wcsToProject_ = Transform3::Identity();
    bool stale_ = false;

    // Background work.
    std::vector<QThread*> threads_;
    bool simRunning_ = false;
    // The setup and input key of the simulation running (or the preparation waited for): asking
    // for the same simulation again waits for it instead of starting over.
    Uuid simRunSetup_;
    std::string simRunKey_;
    std::shared_ptr<CancelToken> simCancel_;
    std::shared_ptr<std::atomic<double>> simProgress_;
    bool frameRunning_ = false;
    std::optional<Position> pending_;
    int frameCount_ = 0;
    struct Cached {
        Position at;
        std::shared_ptr<const sim::Stock> stock;
        std::shared_ptr<const sim::LatheStock> lathe; ///< Turning: the axisymmetric stock.
    };
    std::optional<Cached> cached_;
    /// The stock mesh, updated incrementally by consecutive frames (one frame at a time).
    std::shared_ptr<sim::StockDisplay> display_;
    sim::Rgb stockColour_ = sim::kStockColour;
    /// A display of the simulated stock with the current options (cut marks start afresh).
    /// What every simulation is asked with: the ray spacing, and what the stock display needs.
    document::SimulationOptions baseOptions() const;
    std::shared_ptr<sim::StockDisplay> makeDisplay() const;
    std::optional<Position> lastFrame_; ///< The frame `display_` shows (or is computing).
    bool lastCompare_ = false;
    /// Per operation index: which earlier operation last cut each Z ray (see
    /// sim::DisplayOptions::cutBy), filled in by frames as they need it.
    std::map<int, std::shared_ptr<const std::vector<std::uint8_t>>> cutBase_;
    void updateLegend();
    /// Per operation: the machine time at the start of each move (moves + 1 entries, the last
    /// is the operation's duration) and the tool tip before each move.
    std::vector<std::vector<double>> startTimes_;
    std::vector<std::vector<Vec3>> startTips_;
    double playTime_ = 0.0; ///< Machine time into the current operation [s].
    Position playAt_;
    QElapsedTimer playClock_;

    QPushButton* runButton_ = nullptr;
    QPushButton* cancelButton_ = nullptr;
    QPushButton* closeButton_ = nullptr;
    QProgressBar* progress_ = nullptr;
    QLabel* status_ = nullptr;
    QComboBox* opCombo_ = nullptr;
    QSlider* slider_ = nullptr;
    QLabel* moveLabel_ = nullptr;
    QToolButton* playButton_ = nullptr;
    QComboBox* speedCombo_ = nullptr;
    QDoubleSpinBox* resolution_ = nullptr;
    double resolutionMm_ = 0.0; ///< Ray spacing for new simulations; 0 = automatic.
    QCheckBox* showTool_ = nullptr;
    QCheckBox* showPart_ = nullptr;
    QCheckBox* showFixtures_ = nullptr;
    /// Which toolpaths show over the simulated stock: none, all of them, or the trail.
    enum class PathsShown { None = 0, All = 1, Trail = 2 };
    QComboBox* pathsCombo_ = nullptr;
    PathsShown pathsShown() const;
    /// Trail: how much of the path behind the tool shows [mm along the path]; 0 = all of the
    /// current operation's path done so far.
    QDoubleSpinBox* trailLength_ = nullptr;
    double trailLengthMm_ = 50.0;
    /// The path the tool has come along in the current operation up to `at`, the last
    /// trailLengthMm_ of it (nullptr when there is none).
    std::shared_ptr<const toolpath::Toolpath> trailAt(const Position& at) const;
    /// Per operation: the path length at the start of each move (moves + 1 entries) [mm].
    std::vector<std::vector<double>> startLengths_;
    QCheckBox* compare_ = nullptr;
    QCheckBox* showMachine_ = nullptr;
    QCheckBox* machineFixed_ = nullptr;
    QCheckBox* showEnclosure_ = nullptr; ///< Draws the parts marked enclosure.
    QLabel* axes_ = nullptr;
    /// The simulated machine's model and what is drawn of it (its parts and components).
    machine::MachineModel machineModel_;
    std::shared_ptr<const std::vector<machine::ModelPart>> machineParts_;
    /// The machine's parts with a check box each: shown in the view or not.
    QListWidget* partList_ = nullptr;
    /// Shown or hidden for this session, by machine id and part name (else the part's default).
    std::map<std::string, std::map<std::string, bool>> partShown_;
    bool partShown(const machine::ModelPart& part) const;
    void rebuildPartList();
    bool fitMachine_ = false; ///< Fit the view when the machine next shows.
    QLabel* legend_ = nullptr;
    QLabel* stats_ = nullptr;
    QListWidget* collisions_ = nullptr;
    QTimer* playTimer_ = nullptr;
    QTimer* progressTimer_ = nullptr;
    std::optional<Uuid> lastSetup_;
};

} // namespace ocam::gui
