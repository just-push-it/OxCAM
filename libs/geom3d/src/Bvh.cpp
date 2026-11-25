// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/geom3d/Bvh.hpp"

#include <algorithm>
#include <cmath>

namespace ocam::geom3d {
namespace {

constexpr std::uint32_t kLeafSize = 4;

/// Slab test: entry distance of the ray into the box, or nullopt when it misses within tMax.
bool rayHitsBox(const Aabb3& b, const Vec3& o, const Vec3& inv, double tMax) {
    double t0 = 0.0, t1 = tMax;
    for (int a = 0; a < 3; ++a) {
        double tn = (b.min[a] - o[a]) * inv[a];
        double tf = (b.max[a] - o[a]) * inv[a];
        if (std::isnan(tn) || std::isnan(tf)) {
            // Ray parallel to the slab and starting on its plane: treat as inside.
            if (o[a] < b.min[a] || o[a] > b.max[a]) {
                return false;
            }
            continue;
        }
        if (tn > tf) {
            std::swap(tn, tf);
        }
        t0 = std::max(t0, tn);
        t1 = std::min(t1, tf);
        if (t0 > t1) {
            return false;
        }
    }
    return true;
}

} // namespace

bool overlaps(const Aabb3& a, const Aabb3& b) noexcept {
    return !(a.isEmpty() || b.isEmpty()) && (a.min.array() <= b.max.array()).all() &&
           (b.min.array() <= a.max.array()).all();
}

Aabb3 transformed(const Aabb3& box, const Transform3& xf) noexcept {
    if (box.isEmpty()) {
        return box;
    }
    const Vec3 centre = xf * (0.5 * (box.min + box.max));
    const Vec3 half = xf.linear().cwiseAbs() * (0.5 * (box.max - box.min));
    return {centre - half, centre + half};
}

TriangleBvh::TriangleBvh(Mesh mesh) : mesh_(std::move(mesh)) {
    const std::size_t n = mesh_.triangles.size();
    if (n == 0) {
        return;
    }
    boxes_.resize(n);
    centroids_.resize(n);
    order_.resize(n);
    for (std::size_t t = 0; t < n; ++t) {
        Aabb3 b;
        Vec3 c = Vec3::Zero();
        for (const std::uint32_t v : mesh_.triangles[t]) {
            b.extend(mesh_.vertices[v]);
            c += mesh_.vertices[v];
        }
        boxes_[t] = b;
        centroids_[t] = c / 3.0;
        order_[t] = static_cast<std::uint32_t>(t);
    }
    nodes_.reserve(2 * n / kLeafSize + 1);
    build(0, static_cast<std::uint32_t>(n));
}

std::uint32_t TriangleBvh::build(std::uint32_t first, std::uint32_t count) {
    const auto index = static_cast<std::uint32_t>(nodes_.size());
    nodes_.emplace_back();
    Aabb3 box, cbox;
    for (std::uint32_t k = first; k < first + count; ++k) {
        box.extend(boxes_[order_[k]]);
        cbox.extend(centroids_[order_[k]]);
    }
    nodes_[index].box = box;
    const Vec3 extent = cbox.size();
    if (count <= kLeafSize || extent.maxCoeff() <= 0.0) {
        nodes_[index].first = first;
        nodes_[index].count = count;
        return index;
    }
    int axis = 0;
    extent.maxCoeff(&axis);
    const std::uint32_t mid = first + count / 2;
    std::nth_element(order_.begin() + first, order_.begin() + mid, order_.begin() + first + count,
                     [&](std::uint32_t a, std::uint32_t b) {
                         return centroids_[a][axis] < centroids_[b][axis];
                     });
    const std::uint32_t left = build(first, mid - first);
    const std::uint32_t right = build(mid, first + count - mid);
    nodes_[index].first = left;
    nodes_[index].right = right;
    nodes_[index].count = 0;
    return index;
}

bool TriangleBvh::query(const Aabb3& box, const std::function<bool(std::size_t)>& visit) const {
    if (nodes_.empty()) {
        return true;
    }
    std::vector<std::uint32_t> stack{0};
    while (!stack.empty()) {
        const Node& node = nodes_[stack.back()];
        stack.pop_back();
        if (!overlaps(node.box, box)) {
            continue;
        }
        if (node.count > 0) {
            for (std::uint32_t k = node.first; k < node.first + node.count; ++k) {
                if (overlaps(boxes_[order_[k]], box) && !visit(order_[k])) {
                    return false;
                }
            }
        } else {
            stack.push_back(node.first);
            stack.push_back(node.right);
        }
    }
    return true;
}

bool TriangleBvh::queryPairs(const TriangleBvh& other, const Transform3& otherToThis, double margin,
                             const std::function<bool(std::size_t, std::size_t)>& visit) const {
    if (nodes_.empty() || other.nodes_.empty()) {
        return true;
    }
    const Vec3 grow = Vec3::Constant(margin);
    auto near = [&](const Aabb3& mine, const Aabb3& theirs) {
        Aabb3 moved = transformed(theirs, otherToThis);
        moved.min -= grow;
        moved.max += grow;
        return overlaps(mine, moved);
    };
    std::vector<std::pair<std::uint32_t, std::uint32_t>> stack{{0, 0}};
    while (!stack.empty()) {
        const auto [i, j] = stack.back();
        stack.pop_back();
        const Node& a = nodes_[i];
        const Node& b = other.nodes_[j];
        if (!near(a.box, b.box)) {
            continue;
        }
        if (a.count > 0 && b.count > 0) {
            for (std::uint32_t k = a.first; k < a.first + a.count; ++k) {
                for (std::uint32_t l = b.first; l < b.first + b.count; ++l) {
                    if (near(boxes_[order_[k]], other.boxes_[other.order_[l]]) &&
                        !visit(order_[k], other.order_[l])) {
                        return false;
                    }
                }
            }
            continue;
        }
        // Descend into the larger box (or the one that is not a leaf).
        const bool splitMine = b.count > 0 || (a.count == 0 && a.box.size().squaredNorm() >=
                                                                   b.box.size().squaredNorm());
        if (splitMine) {
            stack.emplace_back(a.first, j);
            stack.emplace_back(a.right, j);
        } else {
            stack.emplace_back(i, b.first);
            stack.emplace_back(i, b.right);
        }
    }
    return true;
}

std::array<Vec3, 3> TriangleBvh::triangle(std::size_t t) const {
    const auto& tri = mesh_.triangles[t];
    return {mesh_.vertices[tri[0]], mesh_.vertices[tri[1]], mesh_.vertices[tri[2]]};
}

std::optional<RayHit> TriangleBvh::raycast(const Vec3& origin, const Vec3& direction,
                                           double tMax) const {
    if (nodes_.empty() || direction.squaredNorm() < 1e-300) {
        return std::nullopt;
    }
    const Vec3 d = direction.normalized();
    const Vec3 inv(1.0 / d.x(), 1.0 / d.y(), 1.0 / d.z());
    std::optional<RayHit> best;
    double limit = tMax;
    std::vector<std::uint32_t> stack{0};
    while (!stack.empty()) {
        const Node& node = nodes_[stack.back()];
        stack.pop_back();
        if (!rayHitsBox(node.box, origin, inv, limit)) {
            continue;
        }
        if (node.count == 0) {
            stack.push_back(node.first);
            stack.push_back(node.right);
            continue;
        }
        for (std::uint32_t k = node.first; k < node.first + node.count; ++k) {
            // Moeller-Trumbore.
            const auto [a, b, c] = triangle(order_[k]);
            const Vec3 e1 = b - a, e2 = c - a;
            const Vec3 p = d.cross(e2);
            const double det = e1.dot(p);
            if (std::abs(det) < 1e-18) {
                continue;
            }
            const double invDet = 1.0 / det;
            const Vec3 s = origin - a;
            const double u = s.dot(p) * invDet;
            if (u < -1e-12 || u > 1.0 + 1e-12) {
                continue;
            }
            const Vec3 q = s.cross(e1);
            const double v = d.dot(q) * invDet;
            if (v < -1e-12 || u + v > 1.0 + 1e-12) {
                continue;
            }
            const double t = e2.dot(q) * invDet;
            if (t >= 0.0 && t <= limit) {
                limit = t;
                best = RayHit{t, order_[k], origin + t * d};
            }
        }
    }
    return best;
}

} // namespace ocam::geom3d
