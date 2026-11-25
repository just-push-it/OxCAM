// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// OCCT presentations for the stock simulation: the coloured stock mesh, the tool and holder
// bodies, the turning insert, collision markers and the parts of the machine.

#pragma once

#include <ocam/geom2d/Polyline.hpp>
#include <ocam/geom3d/Mesh.hpp>
#include <ocam/sim/Compare.hpp>
#include <ocam/sim/ToolShape.hpp>
#include <ocam/tooling/TurningHolder.hpp>

#include <AIS_InteractiveObject.hxx>
#include <memory>
#include <vector>

namespace ocam::gui {

/// Stock mesh with per-vertex colours, transformed into the project frame.
class StockMeshPrs : public AIS_InteractiveObject {
    DEFINE_STANDARD_RTTI_INLINE(StockMeshPrs, AIS_InteractiveObject)

public:
    StockMeshPrs(std::shared_ptr<const sim::DisplayMesh> mesh, const Transform3& wcsToProject,
                 double transparency = 0.0);
    /// Takes effect when the presentation is recomputed (Redisplay).
    void setStockTransparency(double transparency) { transparency_ = transparency; }

    bool AcceptDisplayMode(const int mode) const override { return mode == 0; }

protected:
    void Compute(const occ::handle<PrsMgr_PresentationManager>& manager,
                 const occ::handle<Prs3d_Presentation>& prs, const int mode) override;
    void ComputeSelection(const occ::handle<SelectMgr_Selection>& /*selection*/,
                          const int /*mode*/) override {}

private:
    std::shared_ptr<const sim::DisplayMesh> mesh_;
    Transform3 toProject_;
    double transparency_ = 0.0;
};

/// Tool (cutting part gold, shank grey) and holder (translucent) around the +Z axis with the
/// tip at the origin; place it with a location.
class ToolBodyPrs : public AIS_InteractiveObject {
    DEFINE_STANDARD_RTTI_INLINE(ToolBodyPrs, AIS_InteractiveObject)

public:
    explicit ToolBodyPrs(std::shared_ptr<const sim::ToolShape> tool);

    bool AcceptDisplayMode(const int mode) const override { return mode == 0; }

protected:
    void Compute(const occ::handle<PrsMgr_PresentationManager>& manager,
                 const occ::handle<Prs3d_Presentation>& prs, const int mode) override;
    void ComputeSelection(const occ::handle<SelectMgr_Selection>& /*selection*/,
                          const int /*mode*/) override {}

private:
    std::shared_ptr<const sim::ToolShape> tool_;
};

/// Turning insert: its outline in the lathe plane (u = Z, v = radius, about the programmed point)
/// as a plate of the given thickness on the XZ plane (cutting part gold), the programmed point at
/// the origin; place it with a location. A thickness of 0 draws it a quarter of its size thick.
/// `parts` are the tool's holder, placed the same way (translucent, as a milling tool's holder).
class InsertPrs : public AIS_InteractiveObject {
    DEFINE_STANDARD_RTTI_INLINE(InsertPrs, AIS_InteractiveObject)

public:
    InsertPrs(std::shared_ptr<const geom2d::Polyline> outline, double thickness,
              std::shared_ptr<const std::vector<tooling::TurningToolPart>> parts);

    bool AcceptDisplayMode(const int mode) const override { return mode == 0; }

protected:
    void Compute(const occ::handle<PrsMgr_PresentationManager>& manager,
                 const occ::handle<Prs3d_Presentation>& prs, const int mode) override;
    void ComputeSelection(const occ::handle<SelectMgr_Selection>& /*selection*/,
                          const int /*mode*/) override {}

private:
    std::shared_ptr<const geom2d::Polyline> outline_;
    double thickness_ = 0.0;
    std::shared_ptr<const std::vector<tooling::TurningToolPart>> parts_;
};

/// A machine part: a flat-shaded mesh in one colour, in its own coordinates; place it with a
/// location. Pickable by its triangles when activated (selection mode 0); `index` tells which
/// part of the machine scene it is.
class MachinePartPrs : public AIS_InteractiveObject {
    DEFINE_STANDARD_RTTI_INLINE(MachinePartPrs, AIS_InteractiveObject)

public:
    MachinePartPrs(std::shared_ptr<const geom3d::Mesh> mesh, std::uint32_t rgb,
                   double transparency);

    bool AcceptDisplayMode(const int mode) const override { return mode == 0; }
    void setIndex(std::size_t index) noexcept { index_ = index; }
    std::size_t index() const noexcept { return index_; }

protected:
    void Compute(const occ::handle<PrsMgr_PresentationManager>& manager,
                 const occ::handle<Prs3d_Presentation>& prs, const int mode) override;
    void ComputeSelection(const occ::handle<SelectMgr_Selection>& selection,
                          const int mode) override;

private:
    std::size_t index_ = 0;
    std::shared_ptr<const geom3d::Mesh> mesh_;
    std::uint32_t rgb_ = 0;
    double transparency_ = 0.0;
};

/// Line segments in one colour and width (machine axes in the machine library's preview).
class LinesPrs : public AIS_InteractiveObject {
    DEFINE_STANDARD_RTTI_INLINE(LinesPrs, AIS_InteractiveObject)

public:
    LinesPrs(std::vector<std::pair<Vec3, Vec3>> segments, std::uint32_t rgb, double width);

    bool AcceptDisplayMode(const int mode) const override { return mode == 0; }

protected:
    void Compute(const occ::handle<PrsMgr_PresentationManager>& manager,
                 const occ::handle<Prs3d_Presentation>& prs, const int mode) override;
    void ComputeSelection(const occ::handle<SelectMgr_Selection>& /*selection*/,
                          const int /*mode*/) override {}

private:
    std::vector<std::pair<Vec3, Vec3>> segments_;
    std::uint32_t rgb_ = 0;
    double width_ = 1.0;
};

/// Point markers (project frame).
class MarkersPrs : public AIS_InteractiveObject {
    DEFINE_STANDARD_RTTI_INLINE(MarkersPrs, AIS_InteractiveObject)

public:
    explicit MarkersPrs(std::vector<Vec3> points);

    bool AcceptDisplayMode(const int mode) const override { return mode == 0; }

protected:
    void Compute(const occ::handle<PrsMgr_PresentationManager>& manager,
                 const occ::handle<Prs3d_Presentation>& prs, const int mode) override;
    void ComputeSelection(const occ::handle<SelectMgr_Selection>& /*selection*/,
                          const int /*mode*/) override {}

private:
    std::vector<Vec3> points_;
};

} // namespace ocam::gui
