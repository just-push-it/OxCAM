// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// OCCT presentation of a toolpath: polylines coloured by feed class (rapids dashed).

#pragma once

#include "ViewColors.hpp"

#include <ocam/toolpath/Toolpath.hpp>

#include <AIS_InteractiveObject.hxx>
#include <vector>

namespace ocam::gui {

class ToolpathPrs : public AIS_InteractiveObject {
    DEFINE_STANDARD_RTTI_INLINE(ToolpathPrs, AIS_InteractiveObject)

public:
    /// `wcsToProject` maps the toolpath's WCS coordinates into the viewer's project frame.
    ToolpathPrs(const toolpath::Toolpath& path, const Transform3& wcsToProject, bool emphasized,
                const ViewColors& colors);

    bool AcceptDisplayMode(const int mode) const override { return mode == 0; }

protected:
    void Compute(const occ::handle<PrsMgr_PresentationManager>& manager,
                 const occ::handle<Prs3d_Presentation>& prs, const int mode) override;
    void ComputeSelection(const occ::handle<SelectMgr_Selection>& /*selection*/,
                          const int /*mode*/) override {}

private:
    std::vector<toolpath::PathRun> runs_;
    bool emphasized_;
    ViewColors colors_;
};

} // namespace ocam::gui
