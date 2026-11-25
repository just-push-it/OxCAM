// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/geom3d/Mesh.hpp"

#include <cmath>
#include <map>
#include <tuple>
#include <unordered_map>

namespace ocam::geom3d {

Aabb3 Mesh::bounds() const noexcept {
    Aabb3 box;
    for (const auto& v : vertices) {
        box.extend(v);
    }
    return box;
}

double Mesh::surfaceArea() const noexcept {
    double a = 0.0;
    for (const auto& t : triangles) {
        a += 0.5 * (vertices[t[1]] - vertices[t[0]]).cross(vertices[t[2]] - vertices[t[0]]).norm();
    }
    return a;
}

double Mesh::signedVolume() const noexcept {
    double v6 = 0.0;
    for (const auto& t : triangles) {
        v6 += vertices[t[0]].dot(vertices[t[1]].cross(vertices[t[2]]));
    }
    return v6 / 6.0;
}

bool Mesh::isClosed() const {
    // Directed edge counts; a closed oriented manifold uses each directed edge exactly once and
    // its reverse exactly once.
    std::map<std::pair<std::uint32_t, std::uint32_t>, int> directed;
    for (const auto& t : triangles) {
        for (int i = 0; i < 3; ++i) {
            const auto a = t[static_cast<std::size_t>(i)];
            const auto b = t[static_cast<std::size_t>((i + 1) % 3)];
            if (++directed[{a, b}] > 1) {
                return false;
            }
        }
    }
    for (const auto& [edge, count] : directed) {
        auto it = directed.find({edge.second, edge.first});
        if (it == directed.end() || it->second != count) {
            return false;
        }
    }
    return !triangles.empty();
}

Vec3 Mesh::triangleNormal(std::size_t t) const noexcept {
    const auto& tri = triangles[t];
    const Vec3 n = (vertices[tri[1]] - vertices[tri[0]]).cross(vertices[tri[2]] - vertices[tri[0]]);
    const double len = n.norm();
    return len > 0.0 ? Vec3(n / len) : Vec3::Zero();
}

void Mesh::append(const Mesh& other) {
    const auto base = static_cast<std::uint32_t>(vertices.size());
    vertices.insert(vertices.end(), other.vertices.begin(), other.vertices.end());
    triangles.reserve(triangles.size() + other.triangles.size());
    for (const auto& t : other.triangles) {
        triangles.push_back({t[0] + base, t[1] + base, t[2] + base});
    }
}

void Mesh::transform(const Transform3& xf) {
    for (auto& v : vertices) {
        v = xf * v;
    }
}

Mesh weldVertices(const Mesh& mesh, double tolerance) {
    // Hash grid with cell size = tolerance; each vertex checks its own and neighbouring cells.
    const double cell = tolerance > 0.0 ? tolerance : 1e-12;
    using Key = std::tuple<long long, long long, long long>;
    struct KeyHash {
        std::size_t operator()(const Key& k) const noexcept {
            const auto [x, y, z] = k;
            return static_cast<std::size_t>(x * 73856093LL ^ y * 19349663LL ^ z * 83492791LL);
        }
    };
    std::unordered_map<Key, std::vector<std::uint32_t>, KeyHash> grid;

    Mesh out;
    std::vector<std::uint32_t> remap(mesh.vertices.size());
    for (std::size_t i = 0; i < mesh.vertices.size(); ++i) {
        const Vec3& p = mesh.vertices[i];
        const auto kx = static_cast<long long>(std::floor(p.x() / cell));
        const auto ky = static_cast<long long>(std::floor(p.y() / cell));
        const auto kz = static_cast<long long>(std::floor(p.z() / cell));

        std::uint32_t found = UINT32_MAX;
        for (long long dx = -1; dx <= 1 && found == UINT32_MAX; ++dx) {
            for (long long dy = -1; dy <= 1 && found == UINT32_MAX; ++dy) {
                for (long long dz = -1; dz <= 1 && found == UINT32_MAX; ++dz) {
                    auto it = grid.find({kx + dx, ky + dy, kz + dz});
                    if (it == grid.end()) {
                        continue;
                    }
                    for (auto idx : it->second) {
                        if ((out.vertices[idx] - p).squaredNorm() <= tolerance * tolerance) {
                            found = idx;
                            break;
                        }
                    }
                }
            }
        }
        if (found == UINT32_MAX) {
            found = static_cast<std::uint32_t>(out.vertices.size());
            out.vertices.push_back(p);
            grid[{kx, ky, kz}].push_back(found);
        }
        remap[i] = found;
    }

    out.triangles.reserve(mesh.triangles.size());
    for (const auto& t : mesh.triangles) {
        const Mesh::Triangle r{remap[t[0]], remap[t[1]], remap[t[2]]};
        if (r[0] != r[1] && r[1] != r[2] && r[0] != r[2]) {
            out.triangles.push_back(r);
        }
    }
    return out;
}

} // namespace ocam::geom3d
