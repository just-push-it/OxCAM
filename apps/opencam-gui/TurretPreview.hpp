// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// A lathe setup's machine with its turret loaded as the setup has it (DL-0068): each station's
// block and tool where the simulation puts them (document::latheMachineParts), the work on the
// spindle, and one station highlighted and turned to the working position, at home or with its
// tool at the work. It stands beside the setup page's turret table, to see where tools come close
// before simulating.

#pragma once

#include <ocam/document/LatheMachine.hpp>
#include <ocam/foundation/Uuid.hpp>
#include <ocam/machine/Machine.hpp>

#include <QWidget>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

class QCheckBox;
class QComboBox;
class QLabel;
class QSpinBox;

namespace ocam::document {
class Document;
}

namespace ocam::gui {

class OcctView;

class TurretPreview : public QWidget {
    Q_OBJECT

public:
    explicit TurretPreview(QWidget* parent = nullptr);
    ~TurretPreview() override;

    /// Shows the setup's machine and turret as `doc` has them now (nothing to show when the setup
    /// is not on a lathe with a layout). Unchanged inputs are not rebuilt.
    void showSetup(const document::Document& doc, const std::optional<Uuid>& setupId);
    /// Highlights a station and turns the turret to it.
    void setStation(int station);
    int station() const noexcept { return station_; }
    /// Where the turret stands: at home (machine zero) or with the station's tool at the work.
    void setAtWork(bool atWork);

    // Test hooks.
    OcctView* view() const noexcept { return view_; }
    QString note() const;
    /// The names of the parts drawn highlighted, and of those drawn.
    std::vector<std::string> highlightedParts() const;
    /// Each part's name and colour (0xRRGGBB), for the tests.
    std::vector<std::pair<std::string, std::uint32_t>> partColours() const;
    std::vector<std::string> shownParts() const;
    /// The axes the turret stands at now.
    const machine::AxisPosition& position() const noexcept { return position_; }
    /// Where the highlighted station's tool tip is now (model frame), if it holds one.
    std::optional<Vec3> tipNow() const;
    /// Where the work's front edge is (model frame at the work's carrier's zero), if known.
    std::optional<Vec3> workPoint() const;

signals:
    /// The station was changed here (not by setStation).
    void stationChanged(int station);

protected:
    void showEvent(QShowEvent* event) override;

private:
    /// What a station holds, as the setup loads it.
    struct Held {
        std::string tool; ///< "T03 Drill 16", empty for none.
        std::string block;
        bool across = true;
        Vec3 tip = Vec3::Zero(); ///< Working, at machine zero, model frame.
    };
    void clear(const QString& why);
    void redraw();
    void fit();

    OcctView* view_ = nullptr;
    QSpinBox* stationBox_ = nullptr;
    QComboBox* atBox_ = nullptr;
    QCheckBox* wholeMachine_ = nullptr;
    QCheckBox* enclosure_ = nullptr;
    QLabel* note_ = nullptr;

    std::string key_; ///< What the shown scene was made from.
    std::optional<machine::Machine> machine_;
    machine::MachineModel model_;
    std::shared_ptr<std::vector<machine::ModelPart>> parts_;
    std::vector<int> stations_; ///< Per part: its station, 0 for none.
    std::size_t toolsFrom_ = 0; ///< The tools' parts, after the blocks'.
    /// Whether a part is drawn highlighted: the chosen station's block.
    bool highlighted(std::size_t part) const;
    /// Whether a part is drawn: shown by default, near the work unless the whole machine is, and
    /// not of the enclosure while it is hidden.
    bool drawn(std::size_t part) const;
    std::vector<bool> nearWork_; ///< Per part: the turret, the spindle and the work.
    std::map<int, Held> held_;
    machine::WorkMount mount_;
    std::optional<Vec3> workEdge_;   ///< WCS: the stock's front face at its largest radius.
    std::optional<Vec3> workCentre_; ///< WCS: the stock's front face on the axis.
    double tailstockW_ = 0.0;
    QString problems_;
    int station_ = 1;
    machine::AxisPosition position_;
    QString positionNote_;
    std::string fittedFor_; ///< The machine the camera was last fitted to.
    bool drawPending_ = false;
};

} // namespace ocam::gui
