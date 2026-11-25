// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/geom3d/AxisFit.hpp"

#include "ocam/geom3d/Convex.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <cmath>
#include <unordered_map>
#include <vector>

namespace ocam::geom3d {
namespace {

std::uint64_t edgeKey(std::uint32_t a, std::uint32_t b) {
    return a < b ? (std::uint64_t(a) << 32) | b : (std::uint64_t(b) << 32) | a;
}

} // namespace

std::optional<AxisLine> revolutionAxis(const Mesh& mesh, std::size_t seed, double maxBend) {
    const std::size_t n = mesh.triangles.size();
    if (seed >= n) {
        return std::nullopt;
    }
    // Triangles by edge.
    std::unordered_map<std::uint64_t, std::vector<std::uint32_t>> byEdge;
    byEdge.reserve(n * 3);
    for (std::size_t t = 0; t < n; ++t) {
        const auto& tri = mesh.triangles[t];
        for (int k = 0; k < 3; ++k) {
            byEdge[edgeKey(tri[k], tri[(k + 1) % 3])].push_back(static_cast<std::uint32_t>(t));
        }
    }
    // The smooth region around the seed.
    const double minCos = std::cos(maxBend);
    std::vector<bool> in(n, false);
    std::vector<std::uint32_t> region{static_cast<std::uint32_t>(seed)};
    in[seed] = true;
    for (std::size_t i = 0; i < region.size(); ++i) {
        const std::uint32_t t = region[i];
        const Vec3 nt = mesh.triangleNormal(t);
        const auto& tri = mesh.triangles[t];
        for (int k = 0; k < 3; ++k) {
            for (const std::uint32_t u : byEdge[edgeKey(tri[k], tri[(k + 1) % 3])]) {
                if (!in[u] && mesh.triangleNormal(u).dot(nt) >= minCos) {
                    in[u] = true;
                    region.push_back(u);
                }
            }
        }
    }

    // Area-weighted normals and facet centres.
    double total = 0.0;
    Vec3 mean = Vec3::Zero();
    struct Facet {
        Vec3 n, c;
        double w;
    };
    std::vector<Facet> facets;
    facets.reserve(region.size());
    for (const std::uint32_t t : region) {
        const auto& tri = mesh.triangles[t];
        const Vec3 a = mesh.vertices[tri[0]], b = mesh.vertices[tri[1]], c = mesh.vertices[tri[2]];
        const double w = 0.5 * (b - a).cross(c - a).norm();
        const Vec3 nt = mesh.triangleNormal(t);
        if (w <= 0.0 || nt.squaredNorm() < 0.5) {
            continue;
        }
        facets.push_back({nt, (a + b + c) / 3.0, w});
        total += w;
        mean += w * nt;
    }
    if (facets.size() < 3 || total <= 0.0) {
        return std::nullopt;
    }
    mean /= total;
    Mat3 spread = Mat3::Zero();
    for (const Facet& f : facets) {
        const Vec3 d = f.n - mean;
        spread += f.w * d * d.transpose();
    }
    spread /= total;
    const Eigen::SelfAdjointEigenSolver<Mat3> eig(spread);
    const Vec3 values = eig.eigenvalues(); // ascending
    if (values.z() < 1e-3) {
        return std::nullopt; // flat: the normals barely vary
    }
    Vec3 axis;
    if (values.y() > 0.05 * values.z()) {
        axis = eig.eigenvectors().col(0); // normals spread in a plane (or a cone) around it
    } else {
        // A narrow arc: the normals turn about the axis, across the mean normal.
        axis = mean.cross(eig.eigenvectors().col(2));
        if (axis.norm() < 1e-9) {
            return std::nullopt;
        }
    }
    axis.normalize();

    // The point where the normal lines meet, in the least-squares sense.
    Mat3 a = Mat3::Zero();
    Vec3 rhs = Vec3::Zero();
    for (const Facet& f : facets) {
        const Mat3 p = Mat3::Identity() - f.n * f.n.transpose();
        a += f.w * p;
        rhs += f.w * p * f.c;
    }
    const Eigen::SelfAdjointEigenSolver<Mat3> conditioning(a);
    if (conditioning.eigenvalues().x() < 1e-6 * conditioning.eigenvalues().z()) {
        return std::nullopt;
    }
    const Vec3 point = a.fullPivLu().solve(rhs);
    double radius = 0.0;
    for (const Facet& f : facets) {
        const Vec3 d = f.c - point;
        radius += f.w * (d - d.dot(axis) * axis).norm();
    }
    return AxisLine{point, axis, radius / total, facets.size()};
}

std::optional<FlatFace> flatFace(const Mesh& mesh, std::size_t seed, double maxBend) {
    const std::size_t n = mesh.triangles.size();
    if (seed >= n || mesh.triangleNormal(seed).squaredNorm() < 0.5) {
        return std::nullopt;
    }
    std::unordered_map<std::uint64_t, std::vector<std::uint32_t>> byEdge;
    byEdge.reserve(n * 3);
    for (std::size_t t = 0; t < n; ++t) {
        const auto& tri = mesh.triangles[t];
        for (int k = 0; k < 3; ++k) {
            byEdge[edgeKey(tri[k], tri[(k + 1) % 3])].push_back(static_cast<std::uint32_t>(t));
        }
    }
    const Vec3 normal = mesh.triangleNormal(seed);
    const double minCos = std::cos(maxBend);
    std::vector<bool> in(n, false);
    std::vector<std::uint32_t> region{static_cast<std::uint32_t>(seed)};
    in[seed] = true;
    FlatFace face;
    face.normal = normal;
    Vec3 weighted = Vec3::Zero();
    for (std::size_t i = 0; i < region.size(); ++i) {
        const std::uint32_t t = region[i];
        const auto& tri = mesh.triangles[t];
        const Vec3 a = mesh.vertices[tri[0]], b = mesh.vertices[tri[1]], c = mesh.vertices[tri[2]];
        const double area = 0.5 * (b - a).cross(c - a).norm();
        face.area += area;
        weighted += area * (a + b + c) / 3.0;
        for (int k = 0; k < 3; ++k) {
            for (const std::uint32_t u : byEdge[edgeKey(tri[k], tri[(k + 1) % 3])]) {
                // Coplanar with the seed, not just with the neighbour: no creeping round curves.
                if (!in[u] && mesh.triangleNormal(u).dot(normal) >= minCos) {
                    in[u] = true;
                    region.push_back(u);
                }
            }
        }
    }
    if (face.area <= 0.0) {
        return std::nullopt;
    }
    face.centre = weighted / face.area;
    return face;
}

std::optional<std::size_t> nearestTriangle(const Mesh& mesh, const Vec3& p, double tolerance) {
    std::optional<std::size_t> best;
    double bestDistance = tolerance;
    const Convex point = Convex::sphere(p, 0.0);
    for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
        const auto& tri = mesh.triangles[t];
        const Vec3& a = mesh.vertices[tri[0]];
        const Vec3& b = mesh.vertices[tri[1]];
        const Vec3& c = mesh.vertices[tri[2]];
        Aabb3 box;
        box.extend(a);
        box.extend(b);
        box.extend(c);
        if (!box.contains(p, bestDistance)) {
            continue;
        }
        const double d = distance(point, Convex::triangle(a, b, c)).distance;
        if (d <= bestDistance) {
            bestDistance = d;
            best = t;
        }
    }
    return best;
}

std::optional<AxisLine> circleThrough(const Vec3& a, const Vec3& b, const Vec3& c) {
    const Vec3 ab = b - a, ac = c - a;
    const Vec3 normal = ab.cross(ac);
    const double n2 = normal.squaredNorm();
    if (n2 < 1e-12 * ab.squaredNorm() * ac.squaredNorm() || n2 == 0.0) {
        return std::nullopt;
    }
    // Circumcentre of the triangle.
    const Vec3 centre =
        a +
        (ac.squaredNorm() * normal.cross(ab) + ab.squaredNorm() * ac.cross(normal)) / (2.0 * n2);
    return AxisLine{centre, normal.normalized(), (a - centre).norm(), 0};
}

} // namespace ocam::geom3d
