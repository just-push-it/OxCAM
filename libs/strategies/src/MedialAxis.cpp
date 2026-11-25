// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Approximate medial axis transform on a grid: see MedialAxis.hpp and DL-0031.

#include "MedialAxis.hpp"

#include "Clearing.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <map>
#include <set>

namespace ocam::strategies::detail {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

/// One-dimensional squared-distance transform (Felzenszwalb & Huttenlocher) of f: the lower
/// envelope of the parabolas (q, f[q]); f = 0 at sources, infinity elsewhere.
void transform1d(const std::vector<double>& f, std::vector<double>& d) {
    const int n = static_cast<int>(f.size());
    d.assign(f.size(), kInf);
    std::vector<int> v(f.size(), 0);
    std::vector<double> z(f.size() + 1, 0.0);
    int k = 0;
    v[0] = 0;
    z[0] = -kInf;
    z[1] = kInf;
    for (int q = 1; q < n; ++q) {
        if (f[q] == kInf) {
            continue;
        }
        if (f[v[k]] == kInf) { // no parabola yet
            v[k] = q;
            z[k] = -kInf;
            z[k + 1] = kInf;
            continue;
        }
        double s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0 * q - 2.0 * v[k]);
        while (s <= z[k]) {
            --k;
            s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0 * q - 2.0 * v[k]);
        }
        ++k;
        v[k] = q;
        z[k] = s;
        z[k + 1] = kInf;
    }
    if (f[v[0]] == kInf) {
        return; // nothing in this line
    }
    k = 0;
    for (int q = 0; q < n; ++q) {
        while (z[k + 1] < q) {
            ++k;
        }
        d[q] = (q - v[k]) * (q - v[k]) + f[v[k]];
    }
}

/// Zhang-Suen thinning of a mask to one-pixel-wide chains.
void thin(std::vector<std::uint8_t>& mask, int nx, int ny) {
    const auto at = [&](int i, int j) -> int {
        return (i >= 0 && j >= 0 && i < nx && j < ny) ? mask[static_cast<std::size_t>(j) * nx + i]
                                                      : 0;
    };
    bool changed = true;
    while (changed) {
        changed = false;
        for (int pass = 0; pass < 2; ++pass) {
            std::vector<std::size_t> remove;
            for (int j = 0; j < ny; ++j) {
                for (int i = 0; i < nx; ++i) {
                    if (!at(i, j)) {
                        continue;
                    }
                    const int p2 = at(i, j - 1), p3 = at(i + 1, j - 1), p4 = at(i + 1, j),
                              p5 = at(i + 1, j + 1), p6 = at(i, j + 1), p7 = at(i - 1, j + 1),
                              p8 = at(i - 1, j), p9 = at(i - 1, j - 1);
                    const int b = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
                    if (b < 2 || b > 6) {
                        continue;
                    }
                    const int seq[9] = {p2, p3, p4, p5, p6, p7, p8, p9, p2};
                    int a = 0;
                    for (int k = 0; k < 8; ++k) {
                        a += seq[k] == 0 && seq[k + 1] == 1;
                    }
                    if (a != 1) {
                        continue;
                    }
                    const bool c1 = pass == 0 ? (p2 * p4 * p6 == 0) : (p2 * p4 * p8 == 0);
                    const bool c2 = pass == 0 ? (p4 * p6 * p8 == 0) : (p2 * p6 * p8 == 0);
                    if (c1 && c2) {
                        remove.push_back(static_cast<std::size_t>(j) * nx + i);
                    }
                }
            }
            for (const std::size_t k : remove) {
                mask[k] = 0;
                changed = true;
            }
        }
    }
    // Staircases: Zhang-Suen keeps the corner pixel of every step of a diagonal (it preserves
    // 4-connectivity); the chains are 8-connected here, so a pixel that is not an end and
    // whose set neighbours stay 8-connected among themselves without it goes, one at a time,
    // until the chains are one pixel wide in that sense.
    changed = true;
    while (changed) {
        changed = false;
        for (int j = 0; j < ny; ++j) {
            for (int i = 0; i < nx; ++i) {
                if (!at(i, j)) {
                    continue;
                }
                int di[8], dj[8], count = 0;
                for (int b = -1; b <= 1; ++b) {
                    for (int a = -1; a <= 1; ++a) {
                        if ((a || b) && at(i + a, j + b)) {
                            di[count] = a;
                            dj[count] = b;
                            ++count;
                        }
                    }
                }
                if (count < 2) {
                    continue; // an end, or alone
                }
                // Components of the set neighbours under 8-adjacency among themselves.
                int label[8];
                for (int k = 0; k < count; ++k) {
                    label[k] = k;
                }
                for (int k = 0; k < count; ++k) {
                    for (int l = k + 1; l < count; ++l) {
                        if (std::abs(di[k] - di[l]) <= 1 && std::abs(dj[k] - dj[l]) <= 1) {
                            const int from = label[l], to = label[k];
                            for (int m = 0; m < count; ++m) {
                                if (label[m] == from) {
                                    label[m] = to;
                                }
                            }
                        }
                    }
                }
                bool one = true;
                for (int k = 1; k < count && one; ++k) {
                    one = label[k] == label[0];
                }
                if (one) {
                    mask[static_cast<std::size_t>(j) * nx + i] = 0;
                    changed = true;
                }
            }
        }
    }
}

} // namespace

MedialGraph medialAxis(const geom2d::Regions& area, double cell, double prune, double minRadius,
                       double chordTolerance) {
    MedialGraph graph;
    if (area.empty()) {
        return graph;
    }
    geom2d::Aabb2 box;
    for (const auto& region : area) {
        box.extend(region.bounds());
    }
    box.min -= Vec2::Constant(2.0 * cell);
    box.max += Vec2::Constant(2.0 * cell);
    const int nx = std::max(3, static_cast<int>(std::ceil((box.max.x() - box.min.x()) / cell)));
    const int ny = std::max(3, static_cast<int>(std::ceil((box.max.y() - box.min.y()) / cell)));
    const auto centre = [&](int i, int j) {
        return Vec2(box.min.x() + (i + 0.5) * cell, box.min.y() + (j + 0.5) * cell);
    };
    CellGrid inside(box, cell);
    inside.fill(area, true, chordTolerance);

    // Squared distance (in cells) from each inside cell to the nearest outside cell.
    std::vector<double> f(static_cast<std::size_t>(nx) * ny, 0.0);
    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            f[static_cast<std::size_t>(j) * nx + i] = inside.at(centre(i, j)) ? kInf : 0.0;
        }
    }
    std::vector<double> column(ny), d1(ny), row(nx), d2(nx);
    std::vector<double> dist(f.size(), 0.0);
    for (int i = 0; i < nx; ++i) {
        for (int j = 0; j < ny; ++j) {
            column[j] = f[static_cast<std::size_t>(j) * nx + i];
        }
        transform1d(column, d1);
        for (int j = 0; j < ny; ++j) {
            dist[static_cast<std::size_t>(j) * nx + i] = d1[j];
        }
    }
    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            row[i] = dist[static_cast<std::size_t>(j) * nx + i];
        }
        transform1d(row, d2);
        for (int i = 0; i < nx; ++i) {
            dist[static_cast<std::size_t>(j) * nx + i] = std::sqrt(d2[i]) * cell;
        }
    }
    const auto dAt = [&](int i, int j) {
        return (i >= 0 && j >= 0 && i < nx && j < ny) ? dist[static_cast<std::size_t>(j) * nx + i]
                                                      : 0.0;
    };

    // Centres of maximal discs: a cell whose disc no neighbour's disc holds (a neighbour's
    // distance would then be the cell's plus the step between them; a third of a cell of
    // slack for the raster), and not too near the boundary. A ridge test along the grid's
    // directions missed crests running at other angles, leaving gaps in the axis.
    std::vector<std::uint8_t> mask(f.size(), 0);
    // A tenth of a cell: along the bisector of a corner of half-angle a the distance grows by
    // sin(a) of the step, so more slack than 1 - sin(a) loses that branch (a third of a cell
    // lost every right-angled corner's); an off-axis cell's grows by at least cos(22.5 deg) of
    // it toward one of its eight neighbours; less slack than this leaves rings of noise in
    // a disc.
    const double slack = 0.2 * cell;
    for (int j = 1; j + 1 < ny; ++j) {
        for (int i = 1; i + 1 < nx; ++i) {
            const double d = dAt(i, j);
            if (d < std::max(minRadius, 1.5 * cell)) {
                continue;
            }
            bool maximal = true;
            for (int dj = -1; maximal && dj <= 1; ++dj) {
                for (int di = -1; maximal && di <= 1; ++di) {
                    if (di == 0 && dj == 0) {
                        continue;
                    }
                    const double step = cell * std::sqrt(static_cast<double>(di * di + dj * dj));
                    maximal = dAt(i + di, j + dj) < d + step - slack;
                }
            }
            mask[static_cast<std::size_t>(j) * nx + i] = maximal ? 1 : 0;
        }
    }
    if (const char* dumpDir = std::getenv("OCAM_CLEARING_DUMP")) {
        char name[512];
        std::snprintf(name, sizeof name, "%s/axis_%zu.txt", dumpDir,
                      static_cast<std::size_t>(std::llround(box.min.x() * 1000.0)));
        if (std::ofstream out{name}) {
            for (int j = 0; j < ny; ++j) {
                for (int i = 0; i < nx; ++i) {
                    if (mask[static_cast<std::size_t>(j) * nx + i]) {
                        const Vec2 c = centre(i, j);
                        out << "M " << c.x() << ' ' << c.y() << ' ' << dAt(i, j) << '\n';
                    }
                }
            }
        }
    }
    const std::vector<std::uint8_t> before = mask;
    thin(mask, nx, ny);
    {
        // The deepest cell is the largest circle and always on the axis: it stands whatever
        // the thinning did to the block around it (a disc's centre).
        std::size_t best = 0;
        double bestD = -1.0;
        for (std::size_t k = 0; k < before.size(); ++k) {
            if (before[k] && dist[k] > bestD) {
                bestD = dist[k];
                best = k;
            }
        }
        if (bestD >= 0.0) {
            mask[best] = 1;
        }
    }
    if (const char* dumpDir = std::getenv("OCAM_CLEARING_DUMP")) {
        char name[512];
        std::snprintf(name, sizeof name, "%s/axis_%zu.txt", dumpDir,
                      static_cast<std::size_t>(std::llround(box.min.x() * 1000.0)));
        if (std::ofstream out{name, std::ios::app}) {
            for (int j = 0; j < ny; ++j) {
                for (int i = 0; i < nx; ++i) {
                    if (mask[static_cast<std::size_t>(j) * nx + i]) {
                        const Vec2 c = centre(i, j);
                        out << "S " << c.x() << ' ' << c.y() << ' ' << dAt(i, j) << '\n';
                    }
                }
            }
        }
    }

    // Chains between nodes (pixels with one neighbour or more than two).
    const auto m = [&](int i, int j) -> bool {
        return i >= 0 && j >= 0 && i < nx && j < ny && mask[static_cast<std::size_t>(j) * nx + i];
    };
    const auto neighbours = [&](int i, int j) {
        std::vector<std::pair<int, int>> out;
        for (int dj = -1; dj <= 1; ++dj) {
            for (int di = -1; di <= 1; ++di) {
                if ((di || dj) && m(i + di, j + dj)) {
                    out.emplace_back(i + di, j + dj);
                }
            }
        }
        return out;
    };
    std::map<std::pair<int, int>, int> nodeOf;
    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            if (m(i, j) && neighbours(i, j).size() != 2) {
                nodeOf[{i, j}] = static_cast<int>(graph.nodes.size());
                MedialGraph::Node node;
                node.p = centre(i, j);
                node.radius = dAt(i, j);
                graph.nodes.push_back(node);
            }
        }
    }
    std::set<std::pair<std::pair<int, int>, std::pair<int, int>>> used; // directed pixel steps
    const auto walk = [&](std::pair<int, int> from, std::pair<int, int> next) {
        MedialGraph::Edge edge;
        edge.a = nodeOf[from];
        edge.points.push_back(centre(from.first, from.second));
        edge.radii.push_back(dAt(from.first, from.second));
        std::pair<int, int> prev = from;
        std::pair<int, int> cur = next;
        for (int guard = 0; guard < nx * ny; ++guard) {
            used.insert({prev, cur});
            used.insert({cur, prev});
            edge.points.push_back(centre(cur.first, cur.second));
            edge.radii.push_back(dAt(cur.first, cur.second));
            const auto it = nodeOf.find(cur);
            if (it != nodeOf.end()) {
                edge.b = it->second;
                break;
            }
            const auto ns = neighbours(cur.first, cur.second);
            std::pair<int, int> step = prev;
            for (const auto& n : ns) {
                if (n != prev) {
                    step = n;
                    break;
                }
            }
            if (step == prev) {
                // A loose end that is not a node (should not happen): make it one.
                nodeOf[cur] = static_cast<int>(graph.nodes.size());
                MedialGraph::Node node;
                node.p = centre(cur.first, cur.second);
                node.radius = dAt(cur.first, cur.second);
                graph.nodes.push_back(node);
                edge.b = nodeOf[cur];
                break;
            }
            prev = cur;
            cur = step;
        }
        if (edge.b < 0) {
            return;
        }
        for (std::size_t k = 1; k < edge.points.size(); ++k) {
            edge.length += (edge.points[k] - edge.points[k - 1]).norm();
        }
        const int id = static_cast<int>(graph.edges.size());
        graph.nodes[edge.a].edges.push_back(id);
        if (edge.b != edge.a) {
            graph.nodes[edge.b].edges.push_back(id);
        }
        graph.edges.push_back(std::move(edge));
    };
    for (const auto& [pixel, id] : nodeOf) {
        for (const auto& n : neighbours(pixel.first, pixel.second)) {
            if (!used.count({pixel, n})) {
                walk(pixel, n);
            }
        }
    }
    // Cycles without a node: start one at any unused pixel.
    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            if (!m(i, j) || nodeOf.count({i, j})) {
                continue;
            }
            const auto ns = neighbours(i, j);
            if (ns.size() == 2 && !used.count({{i, j}, ns.front()})) {
                nodeOf[{i, j}] = static_cast<int>(graph.nodes.size());
                MedialGraph::Node node;
                node.p = centre(i, j);
                node.radius = dAt(i, j);
                graph.nodes.push_back(node);
                walk({i, j}, ns.front());
            }
        }
    }

    // Clean-up, until nothing changes: spurs (a branch to a dead end shorter than `prune`, or
    // whose circles all lie within its junction's circle: the pixel noise of a plateau, and
    // the boundary's own corners) are removed; bridges shorter than two cells between
    // junctions are contracted (a junction the raster split into a cluster of pixels); a node
    // left with two branches joins them into one.
    if (const char* dumpDir = std::getenv("OCAM_CLEARING_DUMP")) {
        char name[512];
        std::snprintf(name, sizeof name, "%s/axis_%zu.txt", dumpDir,
                      static_cast<std::size_t>(std::llround(box.min.x() * 1000.0)));
        if (std::ofstream out{name, std::ios::app}) {
            for (std::size_t e = 0; e < graph.edges.size(); ++e) {
                const MedialGraph::Edge& edge = graph.edges[e];
                out << "E " << e << ' ' << edge.a << ' ' << edge.b << ' ' << edge.length << ' '
                    << edge.points.size() << ' ' << edge.points.front().x() << ' '
                    << edge.points.front().y() << ' ' << edge.points.back().x() << ' '
                    << edge.points.back().y() << '\n';
            }
        }
    }
    std::vector<bool> edgeAlive(graph.edges.size(), true);
    std::vector<std::array<bool, 2>> trimmed(graph.edges.size(), {false, false});
    std::vector<bool> nodeAlive(graph.nodes.size(), true);
    const auto detach = [&](int node, int e) {
        auto& list = graph.nodes[static_cast<std::size_t>(node)].edges;
        list.erase(std::remove(list.begin(), list.end(), e), list.end());
    };
    const auto removeEdge = [&](int e) {
        MedialGraph::Edge& edge = graph.edges[static_cast<std::size_t>(e)];
        detach(edge.a, e);
        if (edge.b != edge.a) {
            detach(edge.b, e);
        }
        edgeAlive[static_cast<std::size_t>(e)] = false;
    };
    bool changed = true;
    while (changed) {
        changed = false;
        // A branch's tail into a rounded end is a ridge of the distance but not of maximal
        // circles (each lies within the one at the end arc's centre): trimmed back to the last
        // circle that holds all the ones beyond it; the stub of a spur so trimmed goes next.
        for (std::size_t e = 0; e < graph.edges.size(); ++e) {
            if (!edgeAlive[e]) {
                continue;
            }
            MedialGraph::Edge& edge = graph.edges[e];
            if (edge.a == edge.b || edge.points.size() < 3) {
                continue;
            }
            for (const bool atA : {true, false}) {
                const int node = atA ? edge.a : edge.b;
                if (graph.nodes[static_cast<std::size_t>(node)].edges.size() != 1 ||
                    edge.points.size() < 3 || trimmed[e][atA ? 0 : 1]) {
                    continue;
                }
                trimmed[e][atA ? 0 : 1] = true;
                if (!atA) {
                    std::reverse(edge.points.begin(), edge.points.end());
                    std::reverse(edge.radii.begin(), edge.radii.end());
                }
                std::size_t keep = 0;
                for (std::size_t k = 1; k + 1 < edge.points.size(); ++k) {
                    bool holdsAll = true;
                    for (std::size_t i = 0; holdsAll && i < k; ++i) {
                        holdsAll = (edge.points[i] - edge.points[k]).norm() + edge.radii[i] <=
                                   edge.radii[k] + cell;
                    }
                    if (holdsAll) {
                        keep = k;
                    }
                }
                if (keep > 0) {
                    edge.points.erase(edge.points.begin(),
                                      edge.points.begin() + static_cast<std::ptrdiff_t>(keep));
                    edge.radii.erase(edge.radii.begin(),
                                     edge.radii.begin() + static_cast<std::ptrdiff_t>(keep));
                    graph.nodes[static_cast<std::size_t>(node)].p = edge.points.front();
                    graph.nodes[static_cast<std::size_t>(node)].radius = edge.radii.front();
                    edge.length = 0.0;
                    for (std::size_t k = 1; k < edge.points.size(); ++k) {
                        edge.length += (edge.points[k] - edge.points[k - 1]).norm();
                    }
                    changed = true;
                }
                if (!atA) {
                    std::reverse(edge.points.begin(), edge.points.end());
                    std::reverse(edge.radii.begin(), edge.radii.end());
                }
            }
        }
        for (std::size_t e = 0; e < graph.edges.size(); ++e) {
            if (!edgeAlive[e]) {
                continue;
            }
            const MedialGraph::Edge& edge = graph.edges[e];
            if (edge.a == edge.b) {
                continue;
            }
            const bool endA = graph.nodes[static_cast<std::size_t>(edge.a)].edges.size() == 1;
            const bool endB = graph.nodes[static_cast<std::size_t>(edge.b)].edges.size() == 1;
            if (endA == endB) {
                continue; // both ends loose (an islandlet) or neither (a bridge)
            }
            const MedialGraph::Node& hub =
                graph.nodes[static_cast<std::size_t>(endA ? edge.b : edge.a)];
            bool contained = true;
            for (std::size_t i = 0; contained && i < edge.points.size(); ++i) {
                contained = (edge.points[i] - hub.p).norm() + edge.radii[i] <= hub.radius + cell;
            }
            if (edge.length < prune || contained) {
                const int loose = endA ? edge.a : edge.b;
                removeEdge(static_cast<int>(e));
                nodeAlive[static_cast<std::size_t>(loose)] = false;
                changed = true;
            }
        }
        for (std::size_t e = 0; e < graph.edges.size(); ++e) {
            if (!edgeAlive[e]) {
                continue;
            }
            MedialGraph::Edge& edge = graph.edges[e];
            if (edge.a == edge.b || edge.length >= 2.0 * cell) {
                continue;
            }
            const int keep = graph.nodes[static_cast<std::size_t>(edge.a)].radius >=
                                     graph.nodes[static_cast<std::size_t>(edge.b)].radius
                                 ? edge.a
                                 : edge.b;
            const int drop = keep == edge.a ? edge.b : edge.a;
            removeEdge(static_cast<int>(e));
            for (const int other : graph.nodes[static_cast<std::size_t>(drop)].edges) {
                MedialGraph::Edge& o = graph.edges[static_cast<std::size_t>(other)];
                if (o.a == drop) {
                    o.a = keep;
                }
                if (o.b == drop) {
                    o.b = keep;
                }
                auto& list = graph.nodes[static_cast<std::size_t>(keep)].edges;
                if (std::find(list.begin(), list.end(), other) == list.end()) {
                    list.push_back(other);
                }
            }
            graph.nodes[static_cast<std::size_t>(drop)].edges.clear();
            nodeAlive[static_cast<std::size_t>(drop)] = false;
            changed = true;
        }
        for (std::size_t e = 0; e < graph.edges.size(); ++e) {
            if (!edgeAlive[e]) {
                continue;
            }
            const MedialGraph::Edge& edge = graph.edges[e];
            if (edge.a != edge.b) {
                continue; // a loop from a node to itself: noise when short or within its circle
            }
            const MedialGraph::Node& hub = graph.nodes[static_cast<std::size_t>(edge.a)];
            bool contained = true;
            for (std::size_t i = 0; contained && i < edge.points.size(); ++i) {
                contained = (edge.points[i] - hub.p).norm() + edge.radii[i] <= hub.radius + cell;
            }
            if (edge.length < prune || contained) {
                removeEdge(static_cast<int>(e));
                changed = true;
            }
        }
        for (std::size_t n = 0; n < graph.nodes.size(); ++n) {
            if (!nodeAlive[n] || graph.nodes[n].edges.size() != 2) {
                continue;
            }
            const int e1 = graph.nodes[n].edges[0];
            const int e2 = graph.nodes[n].edges[1];
            MedialGraph::Edge& first = graph.edges[static_cast<std::size_t>(e1)];
            MedialGraph::Edge& second = graph.edges[static_cast<std::size_t>(e2)];
            if (e1 == e2 || first.a == first.b || second.a == second.b) {
                continue;
            }
            // Oriented: the first ends at n, the second starts at n.
            if (first.a == static_cast<int>(n)) {
                std::reverse(first.points.begin(), first.points.end());
                std::reverse(first.radii.begin(), first.radii.end());
                std::swap(first.a, first.b);
                std::swap(trimmed[static_cast<std::size_t>(e1)][0],
                          trimmed[static_cast<std::size_t>(e1)][1]);
            }
            if (second.b == static_cast<int>(n)) {
                std::reverse(second.points.begin(), second.points.end());
                std::reverse(second.radii.begin(), second.radii.end());
                std::swap(second.a, second.b);
                std::swap(trimmed[static_cast<std::size_t>(e2)][0],
                          trimmed[static_cast<std::size_t>(e2)][1]);
            }
            MedialGraph::Edge merged;
            merged.a = first.a;
            merged.b = second.b;
            merged.points = first.points;
            merged.points.insert(merged.points.end(), second.points.begin() + 1,
                                 second.points.end());
            merged.radii = first.radii;
            merged.radii.insert(merged.radii.end(), second.radii.begin() + 1, second.radii.end());
            merged.length = first.length + second.length;
            removeEdge(e1);
            removeEdge(e2);
            nodeAlive[n] = false;
            const int id = static_cast<int>(graph.edges.size());
            graph.nodes[static_cast<std::size_t>(merged.a)].edges.push_back(id);
            if (merged.b != merged.a) {
                graph.nodes[static_cast<std::size_t>(merged.b)].edges.push_back(id);
            }
            graph.edges.push_back(std::move(merged));
            edgeAlive.push_back(true);
            trimmed.push_back({trimmed[static_cast<std::size_t>(e1)][0],
                               trimmed[static_cast<std::size_t>(e2)][1]});
            changed = true;
        }
    }
    // A connected area has one connected axis: of the pieces left, the longest is it and the
    // rest is the thinning's debris.
    {
        std::vector<int> comp(graph.nodes.size(), -1);
        std::vector<double> lengths;
        for (std::size_t start = 0; start < graph.nodes.size(); ++start) {
            if (!nodeAlive[start] || comp[start] >= 0 || graph.nodes[start].edges.empty()) {
                continue;
            }
            const int id = static_cast<int>(lengths.size());
            lengths.push_back(0.0);
            std::vector<std::size_t> stack{start};
            comp[start] = id;
            while (!stack.empty()) {
                const std::size_t n = stack.back();
                stack.pop_back();
                for (const int e : graph.nodes[n].edges) {
                    const MedialGraph::Edge& edge = graph.edges[static_cast<std::size_t>(e)];
                    if (!edgeAlive[static_cast<std::size_t>(e)]) {
                        continue;
                    }
                    for (const int other : {edge.a, edge.b}) {
                        if (comp[static_cast<std::size_t>(other)] < 0) {
                            comp[static_cast<std::size_t>(other)] = id;
                            stack.push_back(static_cast<std::size_t>(other));
                        }
                    }
                }
            }
        }
        if (lengths.size() > 1) {
            for (std::size_t e = 0; e < graph.edges.size(); ++e) {
                if (edgeAlive[e]) {
                    lengths[static_cast<std::size_t>(
                        comp[static_cast<std::size_t>(graph.edges[e].a)])] += graph.edges[e].length;
                }
            }
            const int best = static_cast<int>(std::max_element(lengths.begin(), lengths.end()) -
                                              lengths.begin());
            for (std::size_t e = 0; e < graph.edges.size(); ++e) {
                if (edgeAlive[e] && comp[static_cast<std::size_t>(graph.edges[e].a)] != best) {
                    edgeAlive[e] = false;
                }
            }
        }
    }
    // Compact: the live edges and their nodes, renumbered, the branches smoothed off the pixel
    // staircase (ends fixed).
    MedialGraph out;
    std::vector<int> nodeMap(graph.nodes.size(), -1);
    for (std::size_t e = 0; e < graph.edges.size(); ++e) {
        if (!edgeAlive[e]) {
            continue;
        }
        const MedialGraph::Edge& edge = graph.edges[e];
        for (const int n : {edge.a, edge.b}) {
            if (nodeMap[static_cast<std::size_t>(n)] < 0) {
                nodeMap[static_cast<std::size_t>(n)] = static_cast<int>(out.nodes.size());
                MedialGraph::Node node = graph.nodes[static_cast<std::size_t>(n)];
                node.edges.clear();
                out.nodes.push_back(node);
            }
        }
        MedialGraph::Edge copy = edge;
        copy.a = nodeMap[static_cast<std::size_t>(edge.a)];
        copy.b = nodeMap[static_cast<std::size_t>(edge.b)];
        if (copy.points.size() >= 5) {
            const std::vector<Vec2> pts = copy.points;
            const std::vector<double> rs = copy.radii;
            for (std::size_t k = 2; k + 2 < pts.size(); ++k) {
                copy.points[k] = (pts[k - 2] + pts[k - 1] + pts[k] + pts[k + 1] + pts[k + 2]) / 5.0;
                copy.radii[k] = (rs[k - 2] + rs[k - 1] + rs[k] + rs[k + 1] + rs[k + 2]) / 5.0;
            }
            copy.length = 0.0;
            for (std::size_t k = 1; k < copy.points.size(); ++k) {
                copy.length += (copy.points[k] - copy.points[k - 1]).norm();
            }
        }
        const int id = static_cast<int>(out.edges.size());
        out.nodes[static_cast<std::size_t>(copy.a)].edges.push_back(id);
        if (copy.b != copy.a) {
            out.nodes[static_cast<std::size_t>(copy.b)].edges.push_back(id);
        }
        out.edges.push_back(std::move(copy));
    }
    // Of the nodes standing on their own (a disc's centre, an axis contracted to a point, the
    // raster's stray cells), the largest circle, when nothing kept is as large: it is the
    // root of a piece whose branches are noise, or of one with none.
    {
        double kept = -1.0;
        for (const MedialGraph::Node& node : out.nodes) {
            kept = std::max(kept, node.radius);
        }
        for (const MedialGraph::Edge& edge : out.edges) {
            for (const double rr : edge.radii) {
                kept = std::max(kept, rr);
            }
        }
        int best = -1;
        for (std::size_t n = 0; n < graph.nodes.size(); ++n) {
            if (nodeAlive[n] && graph.nodes[n].edges.empty() &&
                (best < 0 ||
                 graph.nodes[n].radius > graph.nodes[static_cast<std::size_t>(best)].radius)) {
                best = static_cast<int>(n);
            }
        }
        if (best >= 0 && graph.nodes[static_cast<std::size_t>(best)].radius > kept) {
            MedialGraph::Node node = graph.nodes[static_cast<std::size_t>(best)];
            node.edges.clear();
            out.nodes.push_back(node);
        }
    }
    return out;
}

} // namespace ocam::strategies::detail
