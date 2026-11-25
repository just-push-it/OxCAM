// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ocam/foundation/Error.hpp>
#include <ocam/geom3d/Bvh.hpp>
#include <ocam/geom3d/Convex.hpp>
#include <ocam/geom3d/Stl.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <random>
#include <set>

using namespace ocam;
using namespace ocam::geom3d;
using Catch::Matchers::WithinAbs;

namespace {

Mesh box(const Vec3& lo, const Vec3& hi) {
    Mesh m;
    for (int i = 0; i < 8; ++i) {
        m.vertices.emplace_back((i & 1) ? hi.x() : lo.x(), (i & 2) ? hi.y() : lo.y(),
                                (i & 4) ? hi.z() : lo.z());
    }
    const std::uint32_t faces[6][4] = {{0, 2, 3, 1}, {4, 5, 7, 6}, {0, 1, 5, 4},
                                       {2, 6, 7, 3}, {0, 4, 6, 2}, {1, 3, 7, 5}};
    for (const auto& f : faces) {
        m.triangles.push_back({f[0], f[1], f[2]});
        m.triangles.push_back({f[0], f[2], f[3]});
    }
    return m;
}

} // namespace

TEST_CASE("binary STL round trip", "[geom3d][stl]") {
    const Mesh m = box(Vec3(-1.5, 0, 2), Vec3(3, 4.25, 7));
    const std::string bytes = writeStl(m);
    CHECK(bytes.size() == 84 + 12 * 50);
    const Mesh back = readStl(bytes, "box.stl");
    CHECK(back.vertices.size() == 8);
    CHECK(back.triangles.size() == 12);
    CHECK(back.isClosed());
    CHECK_THAT(back.signedVolume(), WithinAbs(m.signedVolume(), 1e-9));
    // Stored meshes give the same bytes again.
    CHECK(writeStl(back) == writeStl(readStl(writeStl(back), "again")));
}

TEST_CASE("ASCII STL is read", "[geom3d][stl]") {
    const std::string text = "solid t\n"
                             " facet normal 0 0 1\n  outer loop\n"
                             "   vertex 0 0 0\n   vertex 1 0 0\n   vertex 0 1 0\n"
                             "  endloop\n endfacet\n"
                             " facet normal 0 0 1\n  outer loop\n"
                             "   vertex 1 0 0\n   vertex 1 1 0\n   vertex 0 1 0\n"
                             "  endloop\n endfacet\nendsolid t\n";
    const Mesh m = readStl(text, "t.stl");
    CHECK(m.triangles.size() == 2);
    CHECK(m.vertices.size() == 4);
    CHECK_THAT(m.surfaceArea(), WithinAbs(1.0, 1e-12));
}

TEST_CASE("invalid STL is rejected", "[geom3d][stl]") {
    CHECK_THROWS_AS(readStl("", "empty.stl"), Error);
    CHECK_THROWS_AS(readStl("hello world", "x.stl"), Error);
    CHECK_THROWS_AS(readStl("solid x\nfacet outer loop vertex 0 0 nan\n", "nan.stl"), Error);
    CHECK_THROWS_AS(readStl("solid x\nvertex 0 0 0\nvertex 1 0 0\n", "short.stl"), Error);
    std::string truncated = writeStl(box(Vec3::Zero(), Vec3::Ones()));
    truncated.resize(truncated.size() - 7);
    CHECK_THROWS_AS(readStl(truncated, "cut.stl"), Error);
    CHECK_THROWS_AS(readStl("solid empty\nendsolid empty\n", "none.stl"), Error);
}

TEST_CASE("BVH pair queries find every close triangle pair", "[geom3d][bvh]") {
    std::mt19937 rng(3);
    std::uniform_real_distribution<double> u(-10.0, 10.0);
    auto soup = [&](int n) {
        Mesh m;
        for (int i = 0; i < n; ++i) {
            const Vec3 c(u(rng), u(rng), u(rng));
            const auto base = static_cast<std::uint32_t>(m.vertices.size());
            for (int k = 0; k < 3; ++k) {
                m.vertices.push_back(c + 0.1 * Vec3(u(rng), u(rng), u(rng)));
            }
            m.triangles.push_back({base, base + 1, base + 2});
        }
        return m;
    };
    const TriangleBvh a(soup(300));
    const TriangleBvh b(soup(300));
    Transform3 xf = Transform3::Identity();
    xf.linear() = Eigen::AngleAxisd(0.6, Vec3(1, 2, 3).normalized()).toRotationMatrix();
    xf.translation() = Vec3(1, -2, 0.5);
    const double margin = 0.3;
    std::set<std::pair<std::size_t, std::size_t>> found;
    a.queryPairs(b, xf, margin, [&](std::size_t i, std::size_t j) {
        found.emplace(i, j);
        return true;
    });
    // Every pair of triangles closer than the margin is among the candidates.
    int close = 0;
    for (std::size_t i = 0; i < a.mesh().triangles.size(); ++i) {
        const auto [p, q, r] = a.triangle(i);
        const Convex ta = Convex::triangle(p, q, r);
        for (std::size_t j = 0; j < b.mesh().triangles.size(); ++j) {
            const auto [s, t, w] = b.triangle(j);
            const Convex tb = Convex::triangle(xf * s, xf * t, xf * w);
            if (distance(ta, tb).distance < margin) {
                ++close;
                CHECK(found.count({i, j}) == 1);
            }
        }
    }
    CHECK(close > 0);
    // Stopping early.
    int calls = 0;
    CHECK_FALSE(a.queryPairs(b, xf, margin, [&](std::size_t, std::size_t) {
        ++calls;
        return false;
    }));
    CHECK(calls == 1);
}
