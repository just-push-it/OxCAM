// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "StrategyTestUtil.hpp"

#include <cmath>

namespace ocam::strategies::testing {

std::vector<std::vector<Vec2>> cutPathsAt(const toolpath::Toolpath& tp, double z, double tol) {
    std::vector<std::vector<Vec2>> paths;
    std::vector<Vec2> current;
    Vec3 pos = tp.start;
    for (const auto& m : tp.moves) {
        const bool atZ = std::abs(pos.z() - z) < 1e-7 && std::abs(m.end.z() - z) < 1e-7;
        if (m.kind == toolpath::MoveKind::Rapid || !atZ || m.kind == toolpath::MoveKind::Cycle) {
            if (!current.empty()) {
                paths.push_back(std::move(current));
                current.clear();
            }
        } else if (m.kind != toolpath::MoveKind::Dwell) {
            if (current.empty()) {
                current.push_back(pos.head<2>());
            }
            if (m.isArc()) {
                const auto pts = toolpath::sampleArc(pos, m, tol);
                for (std::size_t i = 1; i < pts.size(); ++i) {
                    current.push_back(pts[i].head<2>());
                }
            } else {
                current.push_back(m.end.head<2>());
            }
        }
        if (m.kind != toolpath::MoveKind::Dwell) {
            pos = m.end;
        }
    }
    if (!current.empty()) {
        paths.push_back(std::move(current));
    }
    return paths;
}

std::vector<Vec3> feedPoints(const toolpath::Toolpath& tp, double tol) {
    std::vector<Vec3> pts;
    for (const auto& run : toolpath::sampleRuns(tp, tol)) {
        if (run.feedClass != toolpath::FeedClass::Rapid) {
            pts.insert(pts.end(), run.points.begin(), run.points.end());
        }
    }
    return pts;
}

void checkGolden(const std::string& name, const toolpath::Toolpath& tp) {
    const auto path = std::filesystem::path(OCAM_GOLDEN_DIR) / (name + ".json");
    const char* update = std::getenv("OCAM_UPDATE_GOLDEN");
    if (update && std::string(update) == "1") {
        writeFileAtomic(path, dumpJson(toolpath::toJson(tp)));
        SUCCEED("golden file updated: " << path.string());
        return;
    }
    INFO("golden file " << path.string() << " (regenerate with OCAM_UPDATE_GOLDEN=1 after review)");
    REQUIRE(std::filesystem::exists(path));
    const toolpath::Toolpath expected =
        toolpath::toolpathFromJson(parseJson(readFileBytes(path), name));
    const auto diff = toolpath::compareToolpaths(expected, tp, 1e-5);
    CHECK(diff.value_or("") == "");
}

} // namespace ocam::strategies::testing
