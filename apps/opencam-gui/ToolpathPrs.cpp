// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ToolpathPrs.hpp"

#include <Graphic3d_ArrayOfSegments.hxx>
#include <Graphic3d_AspectLine3d.hxx>
#include <Graphic3d_Group.hxx>
#include <Prs3d_Presentation.hxx>
#include <map>

namespace ocam::gui {

ToolpathPrs::ToolpathPrs(const toolpath::Toolpath& path, const Transform3& wcsToProject,
                         bool emphasized, const ViewColors& colors)
    : emphasized_(emphasized), colors_(colors) {
    runs_ = toolpath::sampleRuns(path, 0.01);
    for (auto& run : runs_) {
        for (Vec3& p : run.points) {
            p = wcsToProject * p;
        }
    }
}

void ToolpathPrs::Compute(const occ::handle<PrsMgr_PresentationManager>& /*manager*/,
                          const occ::handle<Prs3d_Presentation>& prs, const int /*mode*/) {
    // One primitive array per feed class.
    std::map<toolpath::FeedClass, std::vector<const toolpath::PathRun*>> byClass;
    for (const auto& run : runs_) {
        if (run.points.size() >= 2) {
            byClass[run.feedClass].push_back(&run);
        }
    }
    for (const auto& [feedClass, runs] : byClass) {
        int vertices = 0;
        for (const auto* run : runs) {
            vertices += 2 * (static_cast<int>(run->points.size()) - 1);
        }
        occ::handle<Graphic3d_ArrayOfSegments> segments = new Graphic3d_ArrayOfSegments(vertices);
        for (const auto* run : runs) {
            for (std::size_t i = 0; i + 1 < run->points.size(); ++i) {
                const Vec3& a = run->points[i];
                const Vec3& b = run->points[i + 1];
                segments->AddVertex(a.x(), a.y(), a.z());
                segments->AddVertex(b.x(), b.y(), b.z());
            }
        }
        const bool rapid = feedClass == toolpath::FeedClass::Rapid;
        occ::handle<Graphic3d_Group> group = prs->NewGroup();
        group->SetPrimitivesAspect(new Graphic3d_AspectLine3d(
            toQuantity(colors_.forMove(feedClass)), rapid ? Aspect_TOL_DASH : Aspect_TOL_SOLID,
            emphasized_ && !rapid ? 2.0 : 1.0));
        group->AddPrimitiveArray(segments);
    }
}

} // namespace ocam::gui
