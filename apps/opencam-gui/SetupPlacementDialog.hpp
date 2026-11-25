// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Places a setup's work on its machine (DL-0037): the machine at machine zero with the work (the
// part, stock, fixtures and workholding) where the placement puts it. Clicking a face of the
// machine snaps the work there (the face's centre, where the rotary axis carrying it meets the
// face, or the point clicked) and makes the axis carrying that part carry the work; clicking the
// work first picks the point of it that goes there (the WCS origin otherwise). Offsets and
// rotations are typed on top of the snap.

#pragma once

#include <ocam/cad/SubShape.hpp>
#include <ocam/document/Document.hpp>
#include <ocam/foundation/Units.hpp>
#include <ocam/machine/Model.hpp>

#include <QDialog>
#include <QImage>
#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QRadioButton;

namespace ocam::gui {

class LengthSpinBox;
class OcctView;

class SetupPlacementDialog : public QDialog {
    Q_OBJECT

public:
    SetupPlacementDialog(const document::Document& doc, const Uuid& setupId, LengthUnit unit,
                         QWidget* parent = nullptr);

    /// The placement as edited; unset = none of its own (the default placement).
    const std::optional<document::SetupPlacement>& placement() const noexcept { return placement_; }

    /// Test hooks: click where a point shows, with a snap mode, and return the note shown. A
    /// machine point is in the model frame at machine zero; a work point in the WCS.
    std::string clickMachine(const Vec3& pointAtZero, machine::Snap snap);
    std::string clickWork(const Vec3& pointWcs, machine::Snap snap);
    /// The text under the view: where the work sits and why it cannot sit there.
    std::string statusText() const;
    /// The dialog with its 3D view (grab() leaves the view out).
    QImage snapshot();

private:
    void refresh();
    void updateScene();
    void fieldsEdited();
    void clicked(int index, double x, double y, double z);
    /// Fits the view to the work (`work`) or to everything shown.
    void frame(bool work);
    machine::Snap snapMode() const;
    void setSnapMode(machine::Snap snap);
    document::SetupPlacement current() const;
    document::Setup placedSetup() const;

    const document::Document* doc_ = nullptr;
    document::Setup setup_;
    machine::Machine machine_;
    machine::MachineModel model_;
    LengthUnit unit_;
    std::optional<document::SetupPlacement> placement_;
    /// The machine's parts at machine zero, then the work's meshes in the WCS.
    std::shared_ptr<std::vector<machine::ModelPart>> parts_;
    /// Per part, its exact geometry (null for meshes): machine STEP parts in their own
    /// coordinates, the work's bodies in the WCS.
    std::vector<cad::Model> solids_;
    struct ExactPick {
        int index = -1;
        cad::SubShapeType type = cad::SubShapeType::Face;
        int subIndex = 0;
    };
    std::optional<ExactPick> exactPick_; ///< The sub-shape under the last click on a B-rep.
    std::size_t machineParts_ = 0;
    std::size_t stockPart_ = 0; ///< Index of the stock among parts_ (== size when none).
    bool fitted_ = false;
    bool loading_ = false;

    OcctView* view_ = nullptr;
    QLabel* note_ = nullptr;
    QLabel* status_ = nullptr;
    QLabel* problem_ = nullptr;
    QRadioButton* snapFace_ = nullptr;
    QRadioButton* snapAxis_ = nullptr;
    QRadioButton* snapPoint_ = nullptr;
    QCheckBox* showStock_ = nullptr;
    QComboBox* carrier_ = nullptr;
    std::array<LengthSpinBox*, 3> target_{};
    QLabel* targetOn_ = nullptr;
    QLabel* workPoint_ = nullptr;
    std::array<LengthSpinBox*, 3> offset_{};
    std::array<QDoubleSpinBox*, 3> rotation_{};
};

} // namespace ocam::gui
