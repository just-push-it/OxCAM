// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Measurements for DL-0040 (smooth stock display): the cell display against the smooth surface
// on the generated fixture projects and a 100k-move toolpath. Hidden (timing); run with
//   test_document "[smoothbench]"

#include <ocam/document/Document.hpp>
#include <ocam/document/Simulate.hpp>
#include <ocam/geom3d/Bvh.hpp>
#include <ocam/sim/Compare.hpp>
#include <ocam/sim/Simulate.hpp>
#include <ocam/sim/Surface.hpp>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>

using namespace ocam;
using namespace ocam::document;

namespace {

using Clock = std::chrono::steady_clock;

double msSince(Clock::time_point t0) {
    return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

/// Closest point on triangle abc to p (Ericson, Real-Time Collision Detection, 5.1.5).
Vec3 closestOnTriangle(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c) {
    const Vec3 ab = b - a, ac = c - a, ap = p - a;
    const double d1 = ab.dot(ap), d2 = ac.dot(ap);
    if (d1 <= 0 && d2 <= 0) {
        return a;
    }
    const Vec3 bp = p - b;
    const double d3 = ab.dot(bp), d4 = ac.dot(bp);
    if (d3 >= 0 && d4 <= d3) {
        return b;
    }
    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) {
        return a + ab * (d1 / (d1 - d3));
    }
    const Vec3 cp = p - c;
    const double d5 = ab.dot(cp), d6 = ac.dot(cp);
    if (d6 >= 0 && d5 <= d6) {
        return c;
    }
    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) {
        return a + ac * (d2 / (d2 - d6));
    }
    const double va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
        return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
    }
    const double denom = 1.0 / (va + vb + vc);
    return a + ab * (vb * denom) + ac * (vc * denom);
}

/// Distance from p to the mesh, capped at `cap`.
double distanceTo(const geom3d::TriangleBvh& bvh, const Vec3& p, double cap) {
    geom3d::Aabb3 box;
    box.extend(p - Vec3::Constant(cap));
    box.extend(p + Vec3::Constant(cap));
    double best = cap;
    bvh.query(box, [&](std::size_t t) {
        const auto tri = bvh.triangle(t);
        best = std::min(best, (closestOnTriangle(p, tri[0], tri[1], tri[2]) - p).norm());
        return true;
    });
    return best;
}

struct MeshStats {
    std::size_t triangles = 0;
    double megabytes = 0.0;
    double nearShare = 0.0; ///< Share of the area within `near` of the part.
    double mean = 0.0;      ///< Area-weighted mean distance of that area [mm].
    double rms = 0.0;
    double p95 = 0.0;
    double max = 0.0;
};

/// Distance of the mesh to the part, sampled at the triangles' centroids and edge midpoints
/// (area-weighted), over the area within `near` of the part: the surfaces meant to be the part.
MeshStats measure(const std::vector<std::shared_ptr<const sim::DisplayMesh>>& meshes,
                  const geom3d::TriangleBvh* part, double near) {
    MeshStats s;
    struct Sample {
        double d, w;
    };
    std::vector<Sample> samples;
    double total = 0.0;
    for (const auto& m : meshes) {
        if (!m) {
            continue;
        }
        s.triangles += m->triangleCount();
        s.megabytes +=
            static_cast<double>(m->positions.size() * (12 + 12 + 3) + m->indices.size() * 4) / 1e6;
        if (!part) {
            continue;
        }
        for (std::size_t t = 0; t + 2 < m->indices.size(); t += 3) {
            Vec3 v[3];
            for (int k = 0; k < 3; ++k) {
                const auto& q = m->positions[m->indices[t + static_cast<std::size_t>(k)]];
                v[k] = Vec3(q[0], q[1], q[2]);
            }
            const double area = 0.5 * (v[1] - v[0]).cross(v[2] - v[0]).norm();
            total += area;
            const Vec3 pts[4] = {(v[0] + v[1] + v[2]) / 3.0, 0.5 * (v[0] + v[1]),
                                 0.5 * (v[1] + v[2]), 0.5 * (v[2] + v[0])};
            for (const Vec3& p : pts) {
                const double d = distanceTo(*part, p, near);
                if (d < near) {
                    samples.push_back({d, area / 4.0});
                }
            }
        }
    }
    if (samples.empty()) {
        return s;
    }
    double w = 0.0, sum = 0.0, sq = 0.0;
    for (const Sample& x : samples) {
        w += x.w;
        sum += x.w * x.d;
        sq += x.w * x.d * x.d;
        s.max = std::max(s.max, x.d);
    }
    s.nearShare = total > 0 ? w / total : 0.0;
    s.mean = sum / w;
    s.rms = std::sqrt(sq / w);
    std::sort(samples.begin(), samples.end(),
              [](const Sample& a, const Sample& b) { return a.d < b.d; });
    double acc = 0.0;
    for (const Sample& x : samples) {
        acc += x.w;
        if (acc >= 0.95 * w) {
            s.p95 = x.d;
            break;
        }
    }
    return s;
}

void print(const std::string& name, const char* method, double buildMs, const MeshStats& s) {
    std::printf(
        "| %-14s | %-22s | %9zu | %6.1f | %8.1f | %5.1f%% | %7.4f | %7.4f | %7.4f | %7.4f |\n",
        name.c_str(), method, s.triangles, s.megabytes, buildMs, 100.0 * s.nearShare, s.mean, s.rms,
        s.p95, s.max);
}

std::vector<std::shared_ptr<const sim::DisplayMesh>> one(sim::DisplayMesh m) {
    return {std::make_shared<const sim::DisplayMesh>(std::move(m))};
}

/// Playback: the operation in `frames` steps, the display updated after each; prints the mean and
/// largest update times.
void playback(const std::string& name, const sim::Stock& start, const sim::ToolShape& tool,
              const toolpath::Toolpath& path, const sim::PartReference* part, int frames,
              int maxCells) {
    sim::DisplayOptions smoothOptions;
    smoothOptions.maxCells = maxCells;
    sim::StockDisplay smooth(smoothOptions);
    sim::Stock stock = start;
    smooth.update(stock, part, 0, true);
    double simMs = 0, smoothMs = 0, smoothMax = 0;
    const std::size_t n = path.moves.size();
    sim::SimulateOptions options;
    options.checkCollisions = false;
    std::size_t done = 0;
    for (int f = 1; f <= frames; ++f) {
        const std::size_t to = n * static_cast<std::size_t>(f) / static_cast<std::size_t>(frames);
        auto t0 = Clock::now();
        sim::simulateMoves(stock, tool, path, done, to, options);
        simMs += msSince(t0);
        done = to;
        t0 = Clock::now();
        smooth.update(stock, part, 0, false);
        const double s = msSince(t0);
        smoothMs += s;
        smoothMax = std::max(smoothMax, s);
    }
    std::printf("playback %-14s %zu moves in %d frames: simulate %.1f ms/frame; display update "
                "%.1f ms/frame (max %.1f)\n",
                name.c_str(), n, frames, simMs / frames, smoothMs / frames, smoothMax);
}

} // namespace

TEST_CASE("smooth stock display: fixtures", "[.][smoothbench]") {
    const int maxCells = 800; // as the GUI
    std::printf("| fixture        | display                | triangles |     MB | build ms | near  "
                " | mean mm | rms mm  | p95 mm  | max mm  |\n");
    std::printf("|---|---|---:|---:|---:|---:|---:|---:|---:|---:|\n");
    // OCAM_SMOOTH_FIXTURES=dome_project,... picks fixtures.
    std::vector<std::string> files{"sample_project", "dome_project", "shaft_project"};
    if (const char* only = std::getenv("OCAM_SMOOTH_FIXTURES")) {
        files.clear();
        std::string list = only;
        for (std::size_t a = 0; a <= list.size();) {
            const std::size_t b = std::min(list.find(',', a), list.size());
            files.push_back(list.substr(a, b - a));
            a = b + 1;
        }
    }
    for (const std::string& name : files) {
        const char* file = name.c_str();
        CancelToken cancel;
        NullProgress progress;
        Document doc =
            Document::open(std::filesystem::path(OCAM_FIXTURE_DIR) / (std::string(file) + ".ocam"),
                           cancel, progress);
        REQUIRE_FALSE(doc.project().setups.empty());
        const Uuid setupId = doc.project().setups.front().id;
        auto t0 = Clock::now();
        const SetupSimulation sim = simulateSetup(doc, setupId, {}, cancel, progress);
        const double simMs = msSince(t0);
        std::size_t moves = 0;
        for (const auto& op : sim.operations) {
            moves += op.toolpath.moves.size();
        }
        std::printf("| %s: %zu operations, %zu moves, resolution %.3f mm, simulated in %.0f ms; "
                    "rays %d x %d |\n",
                    file, sim.operations.size(), moves, sim.resolution, simMs,
                    sim.finalStock().grid(2).nu(), sim.finalStock().grid(2).nv());
        const geom3d::TriangleBvh part(sim.part);
        const sim::Stock& stock = sim.finalStock();
        const sim::Comparison* cmp = sim.comparison ? &*sim.comparison : nullptr;
        const double near = 0.5;

        sim::DisplayOptions smoothOptions;
        smoothOptions.maxCells = maxCells;
        t0 = Clock::now();
        sim::DisplayMesh smooth = sim::buildDisplayMesh(stock, cmp, smoothOptions);
        const double smoothMs = msSince(t0);
        print(file, "dual contouring", smoothMs, measure(one(std::move(smooth)), &part, near));

        // The same at every ray (no merging), for the accuracy the grid itself allows.
        smoothOptions.maxCells = 100000;
        t0 = Clock::now();
        sim::DisplayMesh fine = sim::buildDisplayMesh(stock, cmp, smoothOptions);
        const double fineMs = msSince(t0);
        print(file, "dual contouring, all rays", fineMs,
              measure(one(std::move(fine)), &part, near));

        if (sim.latheInitial && sim.operations.back().latheAfter) {
            t0 = Clock::now();
            sim::DisplayMesh rev =
                sim::revolvedMesh(sim.operations.back().latheAfter->profile(), 0.005);
            const double revMs = msSince(t0);
            print(file, "revolved profile", revMs, measure(one(std::move(rev)), &part, near));
        }

        // Playback of the longest milling operation.
        std::size_t longest = sim.operations.size();
        for (std::size_t k = 0; k < sim.operations.size(); ++k) {
            const auto& op = sim.operations[k];
            if (op.tool && !op.latheAfter &&
                (longest == sim.operations.size() ||
                 op.toolpath.moves.size() > sim.operations[longest].toolpath.moves.size())) {
                longest = k;
            }
        }
        if (longest < sim.operations.size()) {
            const auto& op = sim.operations[longest];
            playback(std::string(file) + "/" + op.name, sim.stockBefore(longest), *op.tool,
                     op.toolpath, sim.partReference.get(), 60, maxCells);
        }
    }
}

TEST_CASE("smooth stock display: 100k moves", "[.][smoothbench]") {
    // The simulation benchmark's zigzag (SimTest "simulation throughput"), 0.1 mm rays.
    geom3d::Aabb3 b;
    b.extend(Vec3(0, 0, -20));
    b.extend(Vec3(100, 100, 0));
    const double h = 0.1;
    const sim::Stock start = sim::Stock::box(b, h);
    toolpath::Toolpath path;
    path.start = Vec3(0, 0, 5);
    auto line = [&](const Vec3& p) {
        toolpath::Move m;
        m.kind = toolpath::MoveKind::Linear;
        m.end = p;
        m.feedRate = 10.0;
        path.moves.push_back(m);
    };
    line({0, 0, -2});
    const int perRow = 100, rows = 1000;
    for (int r = 0; r < rows; ++r) {
        const double y = 100.0 * r / rows;
        for (int k = 1; k <= perRow; ++k) {
            const double x = r % 2 == 0 ? k : 100 - k;
            line({x, y + 100.0 / rows * k / perRow, -2.0 - 0.01 * (r % 5)});
        }
    }
    const sim::ToolShape tool = sim::ToolShape::fromTool(tooling::makeBallEndMill(1, 10, 25, 60));
    sim::Stock stock = start;
    auto t0 = Clock::now();
    sim::simulateToolpath(stock, tool, path);
    std::printf("100k moves (ball 10) at %.2f mm: simulated in %.0f ms\n", h, msSince(t0));
    sim::DisplayOptions smoothOptions;
    smoothOptions.maxCells = 800;
    t0 = Clock::now();
    const sim::DisplayMesh smooth = sim::buildDisplayMesh(stock, nullptr, smoothOptions);
    const double smoothMs = msSince(t0);
    std::printf("final display: %zu triangles in %.0f ms\n", smooth.triangleCount(), smoothMs);
    playback("zigzag-100k", start, tool, path, nullptr, 200, 800);
}
