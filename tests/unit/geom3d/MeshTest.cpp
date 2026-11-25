// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ocam/geom3d/Mesh.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

using namespace ocam;
using namespace ocam::geom3d;
using Catch::Matchers::WithinAbs;

namespace {

// Axis-aligned box as 12 triangles with separate vertices per face (like per-face
// tessellation output), outward oriented.
Mesh unweldedBox(const Vec3& lo, const Vec3& hi) {
    const Vec3 c[8] = {{lo.x(), lo.y(), lo.z()}, {hi.x(), lo.y(), lo.z()}, {hi.x(), hi.y(), lo.z()},
                       {lo.x(), hi.y(), lo.z()}, {lo.x(), lo.y(), hi.z()}, {hi.x(), lo.y(), hi.z()},
                       {hi.x(), hi.y(), hi.z()}, {lo.x(), hi.y(), hi.z()}};
    const int faces[6][4] = {{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 1, 5, 4},
                             {2, 3, 7, 6}, {1, 2, 6, 5}, {3, 0, 4, 7}};
    Mesh m;
    for (const auto& f : faces) {
        const auto base = static_cast<std::uint32_t>(m.vertices.size());
        for (int k : f) {
            m.vertices.push_back(c[k]);
        }
        m.triangles.push_back({base, base + 1, base + 2});
        m.triangles.push_back({base, base + 2, base + 3});
    }
    return m;
}

} // namespace

TEST_CASE("box mesh measures", "[geom3d][mesh]") {
    const Mesh m = unweldedBox({0, 0, 0}, {10, 20, 30});
    CHECK_THAT(m.signedVolume(), WithinAbs(6000.0, 1e-9));
    CHECK_THAT(m.surfaceArea(), WithinAbs(2 * (200.0 + 300.0 + 600.0), 1e-9));
    CHECK((m.bounds().size() - Vec3(10, 20, 30)).norm() < 1e-12);
    CHECK((m.triangleNormal(0) - Vec3(0, 0, -1)).norm() < 1e-12);
}

TEST_CASE("welding closes a per-face mesh", "[geom3d][mesh]") {
    const Mesh raw = unweldedBox({0, 0, 0}, {1, 1, 1});
    CHECK_FALSE(raw.isClosed());
    const Mesh welded = weldVertices(raw, 1e-9);
    CHECK(welded.vertices.size() == 8);
    CHECK(welded.triangles.size() == 12);
    CHECK(welded.isClosed());
    CHECK_THAT(welded.signedVolume(), WithinAbs(1.0, 1e-12));

    Mesh flipped = welded;
    std::swap(flipped.triangles[0][1], flipped.triangles[0][2]);
    CHECK_FALSE(flipped.isClosed());
}

TEST_CASE("weld drops collapsed triangles", "[geom3d][mesh]") {
    Mesh m;
    m.vertices = {{0, 0, 0}, {1e-7, 0, 0}, {0, 1, 0}};
    m.triangles = {{0, 1, 2}};
    CHECK(weldVertices(m, 1e-6).triangles.empty());
}

TEST_CASE("append and transform", "[geom3d][mesh]") {
    Mesh a = unweldedBox({0, 0, 0}, {1, 1, 1});
    Mesh b = unweldedBox({0, 0, 0}, {1, 1, 1});
    b.transform(Transform3(Eigen::Translation3d(5, 0, 0)));
    a.append(b);
    CHECK(a.triangles.size() == 24);
    CHECK_THAT(a.signedVolume(), WithinAbs(2.0, 1e-12));
    CHECK_THAT(a.bounds().max.x(), WithinAbs(6.0, 1e-12));
}
