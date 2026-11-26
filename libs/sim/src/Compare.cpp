// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/sim/Compare.hpp"

#include "ocam/sim/Surface.hpp"

#include <ocam/foundation/Error.hpp>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <mutex>
#include <tuple>
#include <utility>

namespace ocam::sim {

namespace {

/// Extent along the ray of where `keep(in a, in b)` holds (from the lowest to the highest
/// point); false where it holds nowhere.
template <typename Keep>
bool spanWhere(const IntervalList& a, const IntervalList& b, Keep keep, double& lo, double& hi) {
    lo = std::numeric_limits<double>::infinity();
    hi = -lo;
    // Sweep the ends of both lists in order; between consecutive ends the membership in a and
    // in b is constant.
    std::size_t i = 0, j = 0;
    bool inA = false, inB = false;
    double at = -std::numeric_limits<double>::infinity();
    auto endOf = [](const IntervalList& l, std::size_t k, bool in) {
        return in ? static_cast<double>(l[k].hi) : static_cast<double>(l[k].lo);
    };
    while (i < a.size() || j < b.size()) {
        const double ea = i < a.size() ? endOf(a, i, inA) : std::numeric_limits<double>::infinity();
        const double eb = j < b.size() ? endOf(b, j, inB) : std::numeric_limits<double>::infinity();
        const double next = std::min(ea, eb);
        if (keep(inA, inB) && next > at) {
            lo = std::min(lo, at);
            hi = std::max(hi, next);
        }
        at = next;
        if (ea == next) {
            if (inA) {
                ++i;
            }
            inA = !inA;
        }
        if (eb == next) {
            if (inB) {
                ++j;
            }
            inB = !inB;
        }
    }
    return lo <= hi;
}

/// Where along the ray the part material that must remain (`eroded`) is missing from the
/// stock `s`.
std::pair<float, float> gougeSpan(const IntervalList& eroded, const IntervalList& s) {
    double lo = 0.0, hi = 0.0;
    if (!spanWhere(
            eroded, s, [](bool inPart, bool inStock) { return inPart && !inStock; }, lo, hi)) {
        return {0.0f, 0.0f};
    }
    return {static_cast<float>(lo), static_cast<float>(hi)};
}

/// Where along the ray the stock `s` lies beyond the part as far as the tolerance allows
/// (`dilated`).
std::pair<float, float> excessSpan(const IntervalList& s, const IntervalList& dilated) {
    double lo = 0.0, hi = 0.0;
    if (!spanWhere(
            s, dilated, [](bool inStock, bool inPart) { return inStock && !inPart; }, lo, hi)) {
        return {0.0f, 0.0f};
    }
    return {static_cast<float>(lo), static_cast<float>(hi)};
}

} // namespace

bool PartReference::matches(const DexelGrid& grid) const noexcept {
    return grid.nu() == nu && grid.nv() == nv && grid.u0() == u0 && grid.v0() == v0 &&
           grid.spacing() == spacing;
}

PartReference preparePart(const Stock& stock, const geom3d::Mesh& part,
                          const CompareOptions& options) {
    const DexelGrid& g = stock.grid(2);
    PartReference ref;
    ref.u0 = g.u0();
    ref.v0 = g.v0();
    ref.spacing = g.spacing();
    ref.nu = g.nu();
    ref.nv = g.nv();
    ref.tolerance = options.tolerance;
    const std::size_t count = static_cast<std::size_t>(ref.nu) * static_cast<std::size_t>(ref.nv);
    if (count == 0) {
        return ref;
    }
    DexelGrid partGrid(2, g.u0(), g.v0(), g.spacing(), g.nu(), g.nv());
    partGrid.fillFromMesh(part);
    ref.part.resize(count);
    ref.eroded.resize(count);
    ref.dilated.resize(count);

    const double h = g.spacing();
    const double tol = std::max(0.0, options.tolerance);
    const double reach = std::max(tol, h) / h + 1e-9;
    const int k = static_cast<int>(std::floor(reach));
    std::vector<std::pair<int, int>> offsets;
    for (int dj = -k; dj <= k; ++dj) {
        for (int di = -k; di <= k; ++di) {
            if (di * di + dj * dj <= reach * reach) {
                offsets.emplace_back(di, dj);
            }
        }
    }

    tbb::parallel_for(tbb::blocked_range<int>(0, ref.nv), [&](const tbb::blocked_range<int>& rows) {
        for (int j = rows.begin(); j != rows.end(); ++j) {
            for (int i = 0; i < ref.nu; ++i) {
                const IntervalList& p = partGrid.ray(i, j);
                IntervalList eroded = p;
                IntervalList dilated = p;
                for (const auto& [di, dj] : offsets) {
                    const int ii = i + di, jj = j + dj;
                    if (ii < 0 || jj < 0 || ii >= ref.nu || jj >= ref.nv) {
                        eroded.clear();
                        continue;
                    }
                    const IntervalList& q = partGrid.ray(ii, jj);
                    if (!eroded.empty()) {
                        eroded = intersect(eroded, q);
                    }
                    dilated = unite(dilated, q);
                }
                const std::size_t idx =
                    static_cast<std::size_t>(j) * static_cast<std::size_t>(ref.nu) +
                    static_cast<std::size_t>(i);
                ref.part[idx] = p;
                ref.eroded[idx] = grow(eroded, -tol);
                ref.dilated[idx] = grow(dilated, tol);
            }
        }
    });
    return ref;
}

Comparison compareToPart(const Stock& stock, const geom3d::Mesh& part,
                         const CompareOptions& options) {
    return compareToPart(stock, preparePart(stock, part, options));
}

Comparison compareToPart(const Stock& stock, const PartReference& ref) {
    const DexelGrid& g = stock.grid(2);
    if (!ref.matches(g)) {
        throw Error(ErrorCode::InvalidArgument, "the part was prepared on a different stock grid");
    }
    Comparison c;
    c.nu = g.nu();
    c.nv = g.nv();
    c.tolerance = ref.tolerance;
    const std::size_t count = static_cast<std::size_t>(c.nu) * static_cast<std::size_t>(c.nv);
    c.excess.assign(count, 0.0f);
    c.gouge.assign(count, 0.0f);
    c.gougeLo.assign(count, 0.0f);
    c.gougeHi.assign(count, 0.0f);
    if (count == 0) {
        return c;
    }
    const double h = g.spacing();
    std::mutex mutex;
    tbb::parallel_for(tbb::blocked_range<int>(0, c.nv), [&](const tbb::blocked_range<int>& rows) {
        double excessVolume = 0.0, gougeVolume = 0.0, maxExcess = 0.0, maxGouge = 0.0;
        std::size_t gougeRays = 0, excessRays = 0;
        for (int j = rows.begin(); j != rows.end(); ++j) {
            for (int i = 0; i < c.nu; ++i) {
                const std::size_t idx =
                    static_cast<std::size_t>(j) * static_cast<std::size_t>(c.nu) +
                    static_cast<std::size_t>(i);
                const IntervalList& s = g.ray(i, j);
                const IntervalList& p = ref.part[idx];
                if (s.empty() && p.empty()) {
                    continue;
                }
                const double gouge = differenceLength(ref.eroded[idx], s);
                const double beyond = differenceLength(s, ref.dilated[idx]);
                if (gouge > 1e-4) {
                    c.gouge[idx] = static_cast<float>(gouge);
                    std::tie(c.gougeLo[idx], c.gougeHi[idx]) = gougeSpan(ref.eroded[idx], s);
                    gougeVolume += gouge * h * h;
                    maxGouge = std::max(maxGouge, gouge);
                    ++gougeRays;
                }
                if (beyond > 1e-4) {
                    const double raw = differenceLength(s, p);
                    c.excess[idx] = static_cast<float>(raw);
                    excessVolume += beyond * h * h;
                    maxExcess = std::max(maxExcess, raw);
                    ++excessRays;
                }
            }
        }
        const std::scoped_lock lock(mutex);
        c.excessVolume += excessVolume;
        c.gougeVolume += gougeVolume;
        c.maxExcess = std::max(c.maxExcess, maxExcess);
        c.maxGouge = std::max(c.maxGouge, maxGouge);
        c.gougeRays += gougeRays;
        c.excessRays += excessRays;
    });
    return c;
}

Rgb operationColour(int index) {
    static constexpr Rgb kPalette[] = {{66, 133, 244}, {245, 166, 35}, {155, 89, 182},
                                       {0, 188, 212},  {233, 30, 140}, {255, 214, 0},
                                       {121, 85, 72},  {96, 125, 139}};
    constexpr int n = static_cast<int>(std::size(kPalette));
    return kPalette[static_cast<std::size_t>(((index % n) + n) % n)];
}

std::vector<std::uint8_t> changedRays(const Stock& before, const Stock& after, bool topOnly) {
    const DexelGrid& a = before.grid(2);
    const DexelGrid& b = after.grid(2);
    if (a.nu() != b.nu() || a.nv() != b.nv() || a.u0() != b.u0() || a.v0() != b.v0() ||
        a.spacing() != b.spacing()) {
        return {};
    }
    std::vector<std::uint8_t> out(
        static_cast<std::size_t>(a.nu()) * static_cast<std::size_t>(a.nv()), 0);
    tbb::parallel_for(tbb::blocked_range<int>(0, a.nv()), [&](const tbb::blocked_range<int>& rows) {
        for (int j = rows.begin(); j != rows.end(); ++j) {
            for (int i = 0; i < a.nu(); ++i) {
                const IntervalList& ra = a.ray(i, j);
                const IntervalList& rb = b.ray(i, j);
                if (topOnly ? topOf(ra) != topOf(rb) : ra != rb) {
                    out[static_cast<std::size_t>(j) * static_cast<std::size_t>(a.nu()) +
                        static_cast<std::size_t>(i)] = 1;
                }
            }
        }
    });
    return out;
}

Rgb comparisonColour(double excess, double gouge, double tolerance, double excessScale,
                     const Rgb& stock) {
    auto mix = [](const Rgb& a, const Rgb& b, double t) {
        t = std::clamp(t, 0.0, 1.0);
        Rgb out{};
        for (std::size_t k = 0; k < 3; ++k) {
            out[k] = static_cast<std::uint8_t>(std::lround(a[k] + t * (b[k] - a[k])));
        }
        return out;
    };
    const double scale = std::max(excessScale, tolerance * 2.0 + 1e-9);
    if (gouge > 0.0) {
        return mix({235, 70, 55}, {130, 0, 0}, gouge / scale);
    }
    if (excess > tolerance) {
        const double t = (excess - tolerance) / (scale - tolerance);
        return t < 0.5 ? mix({240, 225, 70}, {240, 145, 45}, t * 2.0)
                       : mix({240, 145, 45}, stock, t * 2.0 - 1.0);
    }
    return {80, 185, 95};
}

namespace {

struct Cell {
    bool filled = false;
    float top = 0.0f;
    float bottom = 0.0f;
    Rgb colour{}; ///< Of the top.
    /// Most of the cell's rays had their top cut (also for cells now empty): walls facing this
    /// cell were exposed by that cut and take its colour.
    bool cut = false;
    Rgb cutColour{};
    bool keepsColour = false; ///< At size: walls show the cell's own colour.
    /// Part material is missing between gougeLo and gougeHi (none when gougeLo > gougeHi):
    /// whatever surface of the cell lies there shows gougeColour, the rest its usual colour.
    float gougeLo = std::numeric_limits<float>::infinity();
    float gougeHi = -std::numeric_limits<float>::infinity();
    Rgb gougeColour{};
    /// Compared with a part: whatever surface of the cell lies clear of the stock left to cut,
    /// between excessLo and excessHi, is at size and shows sizeColour. A ray can have stock left
    /// well away from a surface it ends on, as when a live tool goes in from the side.
    bool compared = false;
    float excessLo = std::numeric_limits<float>::infinity();
    float excessHi = -std::numeric_limits<float>::infinity();
    Rgb sizeColour{};

    /// True when [z0, z1] comes within `margin` of the missing material.
    bool gougedAt(double z0, double z1, double margin) const noexcept {
        return gougeLo <= gougeHi && z1 >= static_cast<double>(gougeLo) - margin &&
               z0 <= static_cast<double>(gougeHi) + margin;
    }
    /// True when the surface at `z` is at size: the cell's rays mostly are, or the stock left is
    /// more than `margin` away.
    bool atSizeAt(double z, double margin) const noexcept {
        return compared && (keepsColour || z < static_cast<double>(excessLo) - margin ||
                            z > static_cast<double>(excessHi) + margin);
    }
};

/// Display cells of k x k Z rays over a grid.
struct CellGrid {
    int k = 1;
    int cu = 0;
    int cv = 0;
    int nu = 0;
    int nv = 0;
    double h = 1.0;
    double u0 = 0.0;
    double v0 = 0.0;
    std::vector<Cell> cells;

    void reset(const DexelGrid& g, int maxCells) {
        k = std::max(1, static_cast<int>(std::ceil(static_cast<double>(std::max(g.nu(), g.nv())) /
                                                   std::max(1, maxCells))));
        nu = g.nu();
        nv = g.nv();
        cu = (nu + k - 1) / k;
        cv = (nv + k - 1) / k;
        h = g.spacing();
        u0 = g.u0();
        v0 = g.v0();
        cells.assign(static_cast<std::size_t>(cu) * static_cast<std::size_t>(cv), Cell{});
    }
    Cell& cell(int ci, int cj) {
        return cells[static_cast<std::size_t>(cj) * static_cast<std::size_t>(cu) +
                     static_cast<std::size_t>(ci)];
    }
    bool colouredByCuts = false; ///< Cut marks colour the cells (walls by their neighbours).
    Rgb stock = kStockColour;    ///< Uncut material.
    /// The cell at (ci, cj) whether filled or not; nullptr outside the grid.
    const Cell* any(int ci, int cj) const {
        if (ci < 0 || cj < 0 || ci >= cu || cj >= cv) {
            return nullptr;
        }
        return &cells[static_cast<std::size_t>(cj) * static_cast<std::size_t>(cu) +
                      static_cast<std::size_t>(ci)];
    }
    const Cell* at(int ci, int cj) const {
        if (ci < 0 || cj < 0 || ci >= cu || cj >= cv) {
            return nullptr;
        }
        const Cell& c = cells[static_cast<std::size_t>(cj) * static_cast<std::size_t>(cu) +
                              static_cast<std::size_t>(ci)];
        return c.filled ? &c : nullptr;
    }
};

/// What colours the rays: comparison values and cut marks per Z ray (any may be absent).
struct RayColours {
    const float* excess = nullptr;
    const float* gouge = nullptr;
    const float* gougeLo = nullptr; ///< Without them a gouge colours the whole ray.
    const float* gougeHi = nullptr;
    const float* excessLo = nullptr; ///< Without them excess counts for the whole ray.
    const float* excessHi = nullptr;
    double tolerance = 0.0;
    const std::uint8_t* cutBy = nullptr;
    double excessScale = 2.0;
    Rgb stock = kStockColour; ///< Uncut material.
};

/// Height (median ray top), extent and colour of one cell. `tops` is scratch space.
void computeCell(const DexelGrid& g, const RayColours& rc, CellGrid& cg, int ci, int cj,
                 std::vector<float>& tops) {
    tops.clear();
    const int k = cg.k;
    int total = 0, filled = 0, excessCount = 0, cutCount = 0, lastCut = 0;
    double excessSum = 0.0, maxGouge = 0.0;
    float bottom = std::numeric_limits<float>::infinity();
    float gougeLo = std::numeric_limits<float>::infinity();
    float gougeHi = -std::numeric_limits<float>::infinity();
    float excessLo = std::numeric_limits<float>::infinity();
    float excessHi = -std::numeric_limits<float>::infinity();
    for (int j = cj * k; j < std::min((cj + 1) * k, g.nv()); ++j) {
        for (int i = ci * k; i < std::min((ci + 1) * k, g.nu()); ++i) {
            ++total;
            const std::size_t idx = static_cast<std::size_t>(j) * static_cast<std::size_t>(g.nu()) +
                                    static_cast<std::size_t>(i);
            if (rc.cutBy && rc.cutBy[idx] > 0) {
                ++cutCount;
                lastCut = std::max(lastCut, static_cast<int>(rc.cutBy[idx]));
            }
            // Also on rays cut through: the walls beside them show where the part is missing.
            if (rc.gouge && rc.gouge[idx] > 0.0f) {
                maxGouge = std::max(maxGouge, static_cast<double>(rc.gouge[idx]));
                gougeLo = std::min(gougeLo, rc.gougeLo ? rc.gougeLo[idx]
                                                       : -std::numeric_limits<float>::max());
                gougeHi = std::max(gougeHi, rc.gougeHi ? rc.gougeHi[idx]
                                                       : std::numeric_limits<float>::max());
            }
            const IntervalList& r = g.ray(i, j);
            if (r.empty()) {
                continue;
            }
            ++filled;
            tops.push_back(r.back().hi);
            bottom = std::min(bottom, r.front().lo);
            if (rc.excess) {
                if (rc.excess[idx] > 0.0f) {
                    ++excessCount;
                    excessSum += static_cast<double>(rc.excess[idx]);
                    excessLo = std::min(excessLo, rc.excessLo ? rc.excessLo[idx]
                                                              : -std::numeric_limits<float>::max());
                    excessHi = std::max(excessHi, rc.excessHi ? rc.excessHi[idx]
                                                              : std::numeric_limits<float>::max());
                }
            }
        }
    }
    Cell& cell = cg.cell(ci, cj);
    cell = Cell{};
    cell.cut = cutCount > 0 && cutCount * 2 >= total;
    cell.cutColour = cell.cut ? operationColour(lastCut - 1) : rc.stock;
    if (maxGouge > 0.0) {
        cell.gougeLo = gougeLo;
        cell.gougeHi = gougeHi;
        cell.gougeColour = comparisonColour(0.0, maxGouge, rc.tolerance, rc.excessScale, rc.stock);
    }
    if (filled == 0 || filled * 2 < total) {
        return;
    }
    cell.filled = true;
    // The median ray height: an average would invent heights between a cut and an uncut ray
    // where a wall crosses the cell.
    const auto mid = tops.begin() + static_cast<std::ptrdiff_t>(tops.size() / 2);
    std::nth_element(tops.begin(), mid, tops.end());
    cell.top = *mid;
    cell.bottom = std::min(bottom, cell.top);
    const bool compared = rc.excess != nullptr;
    const bool atSize = compared && excessCount * 2 < filled;
    cell.keepsColour = atSize;
    cell.compared = compared;
    cell.excessLo = excessLo;
    cell.excessHi = excessHi;
    if (compared) {
        cell.sizeColour = comparisonColour(0.0, 0.0, rc.tolerance, rc.excessScale, rc.stock);
    }
    if (rc.cutBy && !atSize) {
        // The top: who cut it last, or plain stock.
        cell.colour = cutCount * 2 >= filled ? operationColour(lastCut - 1) : rc.stock;
    } else if (!compared) {
        cell.colour = rc.stock;
    } else if (excessCount * 2 >= filled) {
        cell.colour =
            comparisonColour(excessSum / excessCount, 0.0, rc.tolerance, rc.excessScale, rc.stock);
    } else {
        cell.colour = comparisonColour(0.0, 0.0, rc.tolerance, rc.excessScale, rc.stock);
    }
}

/// One ray against the prepared part, as compareToPart counts it.
struct RayDeviation {
    float gouge = 0.0f;   ///< Part material missing beyond the tolerance (0 below 1e-4).
    float gougeLo = 0.0f; ///< Where along the ray it is missing.
    float gougeHi = 0.0f;
    float excess = 0.0f;   ///< Stock outside the part (0 when within the tolerance).
    float beyond = 0.0f;   ///< Stock beyond the tolerance (for the excess volume).
    float excessLo = 0.0f; ///< Where along the ray it is.
    float excessHi = 0.0f;
};
RayDeviation compareRay(const IntervalList& s, const PartReference& ref, std::size_t idx) {
    RayDeviation d;
    const IntervalList& p = ref.part[idx];
    if (s.empty() && p.empty()) {
        return d;
    }
    const double gouge = differenceLength(ref.eroded[idx], s);
    const double beyond = differenceLength(s, ref.dilated[idx]);
    if (gouge > 1e-4) {
        d.gouge = static_cast<float>(gouge);
        std::tie(d.gougeLo, d.gougeHi) = gougeSpan(ref.eroded[idx], s);
    }
    if (beyond > 1e-4) {
        d.excess = static_cast<float>(differenceLength(s, p));
        d.beyond = static_cast<float>(beyond);
        std::tie(d.excessLo, d.excessHi) = excessSpan(s, ref.dilated[idx]);
    }
    return d;
}

bool sameGrid(const DexelGrid& a, const DexelGrid& b) {
    return a.nu() == b.nu() && a.nv() == b.nv() && a.u0() == b.u0() && a.v0() == b.v0() &&
           a.spacing() == b.spacing();
}

/// Colours of the surface from the display cells: tops the cell's colour, undersides the stock
/// (unless at size or gouged), walls the cut that exposed them (the cell the wall faces).
SurfaceColour surfaceColours(const CellGrid& cg, const CutColours* cuts = nullptr) {
    return [&cg, cuts](const Vec3& p, const Vec3& n) -> Rgb {
        const double cell = cg.k * cg.h;
        auto index = [&](double x, double y) {
            return std::pair{static_cast<int>(std::floor((x - cg.u0) / cell)),
                             static_cast<int>(std::floor((y - cg.v0) / cell))};
        };
        // The filled cell under (x, y), else the nearest filled neighbour.
        auto filledNear = [&](double x, double y) -> const Cell* {
            const auto [ci, cj] = index(x, y);
            if (const Cell* c = cg.at(ci, cj)) {
                return c;
            }
            const Cell* best = nullptr;
            double bestD = std::numeric_limits<double>::infinity();
            for (int dj = -1; dj <= 1; ++dj) {
                for (int di = -1; di <= 1; ++di) {
                    if (const Cell* c = cg.at(ci + di, cj + dj)) {
                        const double cx = cg.u0 + (ci + di + 0.5) * cell - x;
                        const double cy = cg.v0 + (cj + dj + 0.5) * cell - y;
                        const double d = cx * cx + cy * cy;
                        if (d < bestD) {
                            bestD = d;
                            best = c;
                        }
                    }
                }
            }
            return best;
        };
        // A gouge colours the surface where the part material is missing: in the cell behind
        // the surface or the one in front of it, within a lattice cube (the surface's vertices
        // lie within one of the material's boundary).
        const double margin = 1.5 * cell;
        auto gouged = [&](const Cell* c) {
            return c && c->gougedAt(p.z(), p.z(), margin);
        };
        const Cell* inner = filledNear(p.x() - n.x() * 0.5 * cell, p.y() - n.y() * 0.5 * cell);
        if (gouged(inner)) {
            return inner->gougeColour;
        }
        {
            const auto [oi, oj] = index(p.x() + n.x() * 0.5 * cell, p.y() + n.y() * 0.5 * cell);
            const Cell* outer = cg.any(oi, oj);
            if (gouged(outer)) {
                return outer->gougeColour;
            }
        }
        // At size keeps the comparison's colour; the rest shows its cut.
        if (cg.colouredByCuts && inner && inner->atSizeAt(p.z(), margin)) {
            return inner->sizeColour;
        }
        if (cuts) {
            if (const auto c = (*cuts)(p, n)) {
                return *c;
            }
        }
        if (n.z() > 0.5) {
            const Cell* own = filledNear(p.x(), p.y());
            return own ? own->colour : cg.stock;
        }
        if (n.z() < -0.5) {
            const Cell* own = filledNear(p.x(), p.y());
            return own && (!cg.colouredByCuts || own->keepsColour) ? own->colour : cg.stock;
        }
        if (!cg.colouredByCuts) {
            return inner ? inner->colour : cg.stock;
        }
        const auto [bi, bj] = index(p.x() + n.x() * 0.5 * cell, p.y() + n.y() * 0.5 * cell);
        const Cell* beside = cg.any(bi, bj);
        return beside && beside->cut ? beside->cutColour : cg.stock;
    };
}

/// Range along the ray where two interval lists hold different material (the extent of their
/// symmetric difference); false when they hold the same.
bool changedSpan(const IntervalList& a, const IntervalList& b, double& lo, double& hi) {
    if (a == b) {
        return false;
    }
    return spanWhere(a, b, [](bool inA, bool inB) { return inA != inB; }, lo, hi);
}

} // namespace

DisplayMesh buildDisplayMesh(const Stock& stock, const Comparison* comparison,
                             const DisplayOptions& options) {
    const DexelGrid& g = stock.grid(2);
    if (g.isEmpty()) {
        return {};
    }
    if (comparison && (comparison->nu != g.nu() || comparison->nv != g.nv())) {
        comparison = nullptr;
    }
    const std::size_t count = static_cast<std::size_t>(g.nu()) * static_cast<std::size_t>(g.nv());
    RayColours rc;
    rc.excessScale = options.excessScale;
    rc.stock = options.stockColour;
    if (comparison) {
        rc.excess = comparison->excess.data();
        rc.gouge = comparison->gouge.data();
        if (comparison->gougeLo.size() == count && comparison->gougeHi.size() == count) {
            rc.gougeLo = comparison->gougeLo.data();
            rc.gougeHi = comparison->gougeHi.data();
        }
        rc.tolerance = comparison->tolerance;
    }
    if (options.cutBy && options.cutBy->size() == count) {
        rc.cutBy = options.cutBy->data();
    }
    CellGrid cg;
    cg.reset(g, options.maxCells);
    cg.colouredByCuts = rc.cutBy != nullptr;
    cg.stock = options.stockColour;
    tbb::parallel_for(tbb::blocked_range<int>(0, cg.cv), [&](const tbb::blocked_range<int>& rows) {
        std::vector<float> tops;
        for (int cj = rows.begin(); cj != rows.end(); ++cj) {
            for (int ci = 0; ci < cg.cu; ++ci) {
                computeCell(g, rc, cg, ci, cj, tops);
            }
        }
    });
    return buildSurfaceMesh(stock, options.maxCells, surfaceColours(cg));
}

// ---------------------------------------------------------------------------- StockDisplay

struct StockDisplay::State {
    std::optional<Stock> previous;
    const PartReference* part = nullptr;
    CellGrid cells;
    std::vector<std::uint8_t> cutBy;
    std::vector<float> excess, gouge, gougeLo, gougeHi, beyond, excessLo, excessHi;
    SurfaceLattice lattice;
    int chunksU = 0;
    int chunksV = 0;
};

StockDisplay::StockDisplay(const DisplayOptions& options)
    : options_(options), state_(std::make_unique<State>()) {}

StockDisplay::~StockDisplay() = default;

void StockDisplay::update(const Stock& stock, const PartReference* part, int cutting, bool restart,
                          const std::vector<std::uint8_t>* cutBy,
                          const std::vector<const Stock*>* starts) {
    State& s = *state_;
    const DexelGrid& g = stock.grid(2);
    if (g.isEmpty()) {
        chunks_.clear();
        s.previous.reset();
        return;
    }
    if (part && !part->matches(g)) {
        part = nullptr;
    }
    const std::size_t count = static_cast<std::size_t>(g.nu()) * static_cast<std::size_t>(g.nv());
    const bool full = restart || !s.previous || part != s.part || !sameGrid(s.previous->grid(2), g);
    const auto mark = static_cast<std::uint8_t>(std::clamp(cutting + 1, 0, 255));
    CellGrid& cg = s.cells;
    auto colours = [&] {
        RayColours rc;
        rc.excessScale = options_.excessScale;
        rc.stock = options_.stockColour;
        if (part) {
            rc.excess = s.excess.data();
            rc.gouge = s.gouge.data();
            rc.gougeLo = s.gougeLo.data();
            rc.gougeHi = s.gougeHi.data();
            rc.excessLo = s.excessLo.data();
            rc.excessHi = s.excessHi.data();
            rc.tolerance = part->tolerance;
        }
        rc.cutBy = s.cutBy.data();
        return rc;
    };
    auto deviate = [&](int i, int j, std::size_t idx) {
        const RayDeviation d = compareRay(g.ray(i, j), *part, idx);
        s.excess[idx] = d.excess;
        s.gouge[idx] = d.gouge;
        s.gougeLo[idx] = d.gougeLo;
        s.gougeHi[idx] = d.gougeHi;
        s.beyond[idx] = d.beyond;
        s.excessLo[idx] = d.excessLo;
        s.excessHi[idx] = d.excessHi;
    };

    std::vector<std::uint8_t> dirty; // per chunk
    if (full) {
        cg.reset(g, options_.maxCells);
        cg.colouredByCuts = true;
        cg.stock = options_.stockColour;
        s.cutBy = cutBy && cutBy->size() == count ? *cutBy : std::vector<std::uint8_t>(count, 0);
        s.excess.assign(part ? count : 0, 0.0f);
        s.gouge.assign(part ? count : 0, 0.0f);
        s.gougeLo.assign(part ? count : 0, 0.0f);
        s.gougeHi.assign(part ? count : 0, 0.0f);
        s.beyond.assign(part ? count : 0, 0.0f);
        s.excessLo.assign(part ? count : 0, 0.0f);
        s.excessHi.assign(part ? count : 0, 0.0f);
        if (part) {
            tbb::parallel_for(
                tbb::blocked_range<int>(0, g.nv()), [&](const tbb::blocked_range<int>& rows) {
                    for (int j = rows.begin(); j != rows.end(); ++j) {
                        for (int i = 0; i < g.nu(); ++i) {
                            deviate(i, j,
                                    static_cast<std::size_t>(j) * static_cast<std::size_t>(g.nu()) +
                                        static_cast<std::size_t>(i));
                        }
                    }
                });
        }
        const RayColours rc = colours();
        tbb::parallel_for(tbb::blocked_range<int>(0, cg.cv),
                          [&](const tbb::blocked_range<int>& rows) {
                              std::vector<float> tops;
                              for (int cj = rows.begin(); cj != rows.end(); ++cj) {
                                  for (int ci = 0; ci < cg.cu; ++ci) {
                                      computeCell(g, rc, cg, ci, cj, tops);
                                  }
                              }
                          });
        s.lattice = surfaceLattice(stock, options_.maxCells);
        s.chunksU = (s.lattice.ni + kChunk - 1) / kChunk;
        s.chunksV = (s.lattice.nj + kChunk - 1) / kChunk;
        chunks_.assign(static_cast<std::size_t>(s.chunksU) * static_cast<std::size_t>(s.chunksV),
                       nullptr);
        dirty.assign(chunks_.size(), 1);
    } else {
        // Only the tiles that changed since the last update: copies share untouched tiles.
        const DexelGrid& pg = s.previous->grid(2);
        std::vector<std::pair<int, int>> changed;
        for (int tj = 0; tj < g.tileRows(); ++tj) {
            for (int ti = 0; ti < g.tileColumns(); ++ti) {
                if (g.tileId(ti, tj) != pg.tileId(ti, tj)) {
                    changed.emplace_back(ti, tj);
                }
            }
        }
        constexpr int T = DexelGrid::kTile;
        tbb::parallel_for(std::size_t{0}, changed.size(), [&](std::size_t n) {
            const auto [ti, tj] = changed[n];
            for (int j = tj * T; j < std::min((tj + 1) * T, g.nv()); ++j) {
                for (int i = ti * T; i < std::min((ti + 1) * T, g.nu()); ++i) {
                    const IntervalList& now = g.ray(i, j);
                    const IntervalList& before = pg.ray(i, j);
                    if (now == before) {
                        continue;
                    }
                    const std::size_t idx =
                        static_cast<std::size_t>(j) * static_cast<std::size_t>(g.nu()) +
                        static_cast<std::size_t>(i);
                    if (mark > 0 && topOf(now) != topOf(before)) {
                        s.cutBy[idx] = mark; // the surface seen from above was cut
                    }
                    if (part) {
                        deviate(i, j, idx);
                    }
                }
            }
        });
        dirty.assign(chunks_.size(), 0);
        // Rays [i0, i1] x [j0, j1] changed: the surface's vertices within two lattice
        // cubes and their shading one further depend on them.
        auto markSmooth = [&](int i0, int i1, int j0, int j1) {
            const int st = s.lattice.stride;
            const int x0 = std::max((i0 - 1) / st - 3, 0) / kChunk;
            const int x1 = std::min((i1 + 1) / st + 3, s.lattice.ni - 1) / kChunk;
            const int y0 = std::max((j0 - 1) / st - 3, 0) / kChunk;
            const int y1 = std::min((j1 + 1) / st + 3, s.lattice.nj - 1) / kChunk;
            for (int y = y0; y <= y1; ++y) {
                for (int x = x0; x <= x1; ++x) {
                    dirty[static_cast<std::size_t>(y) * static_cast<std::size_t>(s.chunksU) +
                          static_cast<std::size_t>(x)] = 1;
                }
            }
        };
        const RayColours rc = colours();
        std::vector<float> tops;
        for (const auto& [ti, tj] : changed) {
            const int ci0 = ti * T / cg.k;
            const int ci1 = std::min((std::min((ti + 1) * T, g.nu()) - 1) / cg.k, cg.cu - 1);
            const int cj0 = tj * T / cg.k;
            const int cj1 = std::min((std::min((tj + 1) * T, g.nv()) - 1) / cg.k, cg.cv - 1);
            for (int cj = cj0; cj <= cj1; ++cj) {
                for (int ci = ci0; ci <= ci1; ++ci) {
                    computeCell(g, rc, cg, ci, cj, tops);
                }
            }
            markSmooth(ti * T, std::min((ti + 1) * T, g.nu()) - 1, tj * T,
                       std::min((tj + 1) * T, g.nv()) - 1);
        }
        if (stock.triDexel()) {
            // Walls move on the X and Y grids alone when no Z ray lies in the material cut.
            for (int axis = 0; axis < 2; ++axis) {
                const DexelGrid& ga = stock.grid(axis);
                const DexelGrid& pa = s.previous->grid(axis);
                if (ga.isEmpty() || !sameGrid(ga, pa)) {
                    continue;
                }
                std::vector<std::pair<int, int>> sideTiles;
                for (int tj = 0; tj < ga.tileRows(); ++tj) {
                    for (int ti = 0; ti < ga.tileColumns(); ++ti) {
                        if (ga.tileId(ti, tj) != pa.tileId(ti, tj)) {
                            sideTiles.emplace_back(ti, tj);
                        }
                    }
                }
                // Per changed tile, the Z-grid ray boxes [i0, i1] x [j0, j1] its rays changed.
                std::vector<std::vector<std::array<int, 4>>> boxes(sideTiles.size());
                tbb::parallel_for(std::size_t{0}, sideTiles.size(), [&](std::size_t n) {
                    const auto [ti, tj] = sideTiles[n];
                    for (int j = tj * T; j < std::min((tj + 1) * T, ga.nv()); ++j) {
                        for (int i = ti * T; i < std::min((ti + 1) * T, ga.nu()); ++i) {
                            double lo = 0.0, hi = 0.0;
                            if (!changedSpan(ga.ray(i, j), pa.ray(i, j), lo, hi)) {
                                continue;
                            }
                            // X rays: u = Y (i), along X; Y rays: v = X (j), along Y.
                            const auto [a0, a1] = axis == 0 ? g.uRange(lo, hi) : std::pair{j, j};
                            const auto [b0, b1] = axis == 0 ? std::pair{i, i} : g.vRange(lo, hi);
                            boxes[n].push_back({a0, std::max(a0, a1), b0, std::max(b0, b1)});
                        }
                    }
                });
                for (const auto& list : boxes) {
                    for (const auto& b : list) {
                        markSmooth(b[0], b[1], b[2], b[3]);
                    }
                }
            }
        }
    }
    std::optional<CutColours> cuts;
    if (starts && !starts->empty()) {
        cuts.emplace(stock, *starts, options_.stockColour);
    }
    tbb::parallel_for(std::size_t{0}, chunks_.size(), [&](std::size_t c) {
        if (!dirty[c]) {
            return;
        }
        const int x = static_cast<int>(c % static_cast<std::size_t>(s.chunksU));
        const int y = static_cast<int>(c / static_cast<std::size_t>(s.chunksU));
        auto mesh = std::make_shared<DisplayMesh>();
        contourStock(stock, s.lattice, x * kChunk, (x + 1) * kChunk - 1, y * kChunk,
                     (y + 1) * kChunk - 1, surfaceColours(cg, cuts ? &*cuts : nullptr), {}, *mesh);
        chunks_[c] = std::move(mesh);
    });
    s.part = part;
    s.previous = stock;
}

std::optional<Comparison> StockDisplay::comparison() const {
    const State& s = *state_;
    if (!s.part || !s.previous) {
        return std::nullopt;
    }
    Comparison c;
    c.nu = s.cells.nu;
    c.nv = s.cells.nv;
    c.tolerance = s.part->tolerance;
    const double area = s.cells.h * s.cells.h;
    for (std::size_t idx = 0; idx < s.excess.size(); ++idx) {
        if (s.gouge[idx] > 0.0f) {
            ++c.gougeRays;
            c.gougeVolume += static_cast<double>(s.gouge[idx]) * area;
            c.maxGouge = std::max(c.maxGouge, static_cast<double>(s.gouge[idx]));
        }
        if (s.excess[idx] > 0.0f) {
            ++c.excessRays;
            c.excessVolume += static_cast<double>(s.beyond[idx]) * area;
            c.maxExcess = std::max(c.maxExcess, static_cast<double>(s.excess[idx]));
        }
    }
    return c;
}

} // namespace ocam::sim
