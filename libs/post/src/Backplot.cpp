// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// G-code -> Toolpath (Fanuc-style subset), and comparison against CL toolpaths.

#include "ocam/post/Post.hpp"

#include <ocam/foundation/Error.hpp>
#include <ocam/machine/Kinematics.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <limits>
#include <map>
#include <sstream>

namespace ocam::post {
namespace {

using toolpath::FeedClass;
using toolpath::Move;
using toolpath::MoveKind;

struct Word {
    char letter;
    double value;
    bool point = false; ///< Written with a decimal point ("P1.5").
};

std::vector<Word> tokenize(std::string_view line, int lineNo) {
    std::vector<Word> words;
    std::size_t i = 0;
    while (i < line.size()) {
        const char c = static_cast<char>(std::toupper(static_cast<unsigned char>(line[i])));
        if (std::isspace(static_cast<unsigned char>(c))) {
            ++i;
            continue;
        }
        if (c < 'A' || c > 'Z') {
            throw Error(ErrorCode::Parse,
                        fmt::format("line {}: unexpected character '{}'", lineNo, line[i]));
        }
        std::size_t j = i + 1;
        while (j < line.size() && std::isspace(static_cast<unsigned char>(line[j]))) {
            ++j;
        }
        std::size_t k = j;
        while (k < line.size() && (std::isdigit(static_cast<unsigned char>(line[k])) ||
                                   line[k] == '.' || line[k] == '-' || line[k] == '+')) {
            ++k;
        }
        if (k == j) {
            throw Error(ErrorCode::Parse,
                        fmt::format("line {}: word '{}' has no value", lineNo, c));
        }
        std::string number(line.substr(j, k - j));
        const bool point = number.find('.') != std::string::npos;
        if (!number.empty() && number.front() == '+') {
            number.erase(0, 1);
        }
        if (number.back() == '.') {
            number.push_back('0');
        }
        double value = 0.0;
        const auto res = std::from_chars(number.data(), number.data() + number.size(), value);
        if (res.ec != std::errc() || res.ptr != number.data() + number.size()) {
            throw Error(ErrorCode::Parse,
                        fmt::format("line {}: invalid number '{}'", lineNo, number));
        }
        words.push_back({c, value, point});
        i = k;
    }
    return words;
}

std::string stripComments(std::string_view line) {
    std::string out;
    int depth = 0;
    for (char c : line) {
        if (c == ';' && depth == 0) {
            break;
        }
        if (c == '(') {
            ++depth;
        } else if (c == ')') {
            depth = std::max(0, depth - 1);
        } else if (depth == 0) {
            out.push_back(c);
        }
    }
    return out;
}

double moveLength(const Vec3& from, const Move& m) {
    if (m.kind == MoveKind::Dwell) {
        return 0.0;
    }
    if (m.isArc()) {
        const double r =
            (toolpath::planeCoords(from, m.plane) - toolpath::planeCoords(m.center, m.plane))
                .norm();
        const double arc = r * std::abs(toolpath::arcSweep(from, m));
        return std::hypot(arc, toolpath::planeNormal(m.end, m.plane) -
                                   toolpath::planeNormal(from, m.plane));
    }
    return (m.end - from).norm();
}

} // namespace

BackplotResult backplot(std::string_view gcode, const BackplotOptions& options) {
    BackplotResult result;
    toolpath::Toolpath& path = result.path;
    path.start = Vec3::Zero();

    LengthUnit unit = LengthUnit::Millimeter;
    bool absolute = true;
    int motion = -1; // 0-3, or 73/81..85 for canned cycles
    double feed = 0.0;
    bool returnToR = true;
    Vec3 pos = Vec3::Zero();
    double cycleZ = 0.0, cycleR = 0.0, cycleQ = 0.0, cycleP = 0.0;
    bool polar = false;       // polar interpolation on: X and Y in the part's frame
    bool polarRadial = false; // Haas's G112: X is a radius there
    bool polarC = false;      // Fanuc's G12.1: C is the Y
    double wrapYRadius = 0.0; // Haas's G107: Y is a length on a cylinder of this radius
    bool warnedFeed = false;
    bool perRev = false;
    toolpath::ArcPlane plane = toolpath::ArcPlane::XY; // G17 / G18 / G19
    double rpm = 0.0;
    double pitch = 0.0;
    bool inverseTime = false;
    double inverseF = 0.0;       // G93 F [1/min]
    machine::IndexAngles rotary; // A B C [rad]
    Vec3 axis = Vec3::UnitZ();   // tool axis from the rotary positions

    std::istringstream stream{std::string(gcode)};
    std::string rawLine;
    int lineNo = 0;
    while (std::getline(stream, rawLine)) {
        ++lineNo;
        std::string line = stripComments(rawLine);
        line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
        const auto first = line.find_first_not_of(" \t");
        if (first == std::string::npos || line[first] == '%') {
            continue;
        }
        if (std::toupper(static_cast<unsigned char>(line[first])) == 'O') {
            continue; // program number
        }
        const auto words = tokenize(line, lineNo);

        std::map<char, double> w;
        std::vector<double> gs;
        std::vector<int> ms;
        // A dwell P with a decimal point is in seconds ("G04 P1.5"), without one in milliseconds
        // (Haas-style controls; Fanuc-style posts write whole milliseconds).
        double dwellP = 0.0;
        for (const Word& word : words) {
            if (word.letter == 'G') {
                gs.push_back(word.value);
            } else if (word.letter == 'M') {
                ms.push_back(static_cast<int>(std::lround(word.value)));
            } else {
                w[word.letter] = word.value;
                if (word.letter == 'P') {
                    dwellP = word.point ? word.value : word.value / 1000.0;
                }
            }
        }
        auto has = [&](char c) {
            return w.contains(c);
        };

        bool home = false;
        bool dwell = false;
        bool newCycle = false;
        bool radialTap = false;
        bool wrapBlock = false;
        for (double g : gs) {
            const int code = static_cast<int>(std::lround(g * 10.0)); // G54.1 -> 541
            switch (code) {
            case 0:
                motion = 0;
                break;
            case 10:
                motion = 1;
                break;
            case 20:
                motion = 2;
                break;
            case 30:
                motion = 3;
                break;
            case 40:
                dwell = true;
                break;
            case 170:
                plane = toolpath::ArcPlane::XY;
                break;
            case 180:
                plane = toolpath::ArcPlane::ZX;
                break;
            case 190:
                plane = toolpath::ArcPlane::YZ;
                break;
            case 320:
                motion = 32; // thread cut
                break;
            case 500:
            case 960:
            case 970:
                break; // spindle limit, constant surface speed / rpm
            case 950:
                // Haas's live tool tap along Z, where a lathe's block has a Z; otherwise feed per
                // revolution (G-code system B).
                if (options.lathe && has('Z')) {
                    motion = 84;
                    newCycle = true;
                } else {
                    perRev = true;
                }
                break;
            case 1860: // Haas's live tool tap along Z, the other way
            case 1840: // Haas's left-hand tap on the spindle's axis
                motion = 84;
                newCycle = true;
                break;
            case 740: // A mill's left-hand tap; on a lathe, G74 is another cycle altogether.
                if (!options.lathe) {
                    motion = 74;
                    newCycle = true;
                }
                break;
            case 71: // Fanuc's cylindrical interpolation: C the radius, 0 for off; C in degrees
                wrapBlock = true;
                break;
            case 1070: // Haas's: R or Q the cylinder, neither for off; Y a length on it
                wrapYRadius = has('R') ? toMm(w['R'], unit) : has('Q') ? toMm(w['Q'], unit) / 2.0 : 0.0;
                wrapBlock = true;
                break;
            case 121:  // Fanuc's polar interpolation
            case 1120: // Haas's
                polar = true;
                polarRadial = code == 1120;
                polarC = code == 121;
                plane = toolpath::ArcPlane::XY;
                pos = Vec3(pos.x() * std::cos(rotary.c), pos.x() * std::sin(rotary.c), pos.z());
                break;
            case 131:
            case 1130:
                polar = false;
                if (code == 131) {
                    plane = toolpath::ArcPlane::ZX;
                }
                if (pos.head<2>().norm() > 1e-9) {
                    rotary.c = std::atan2(pos.y(), pos.x());
                    axis = Vec3(std::cos(rotary.c), std::sin(rotary.c), 0.0);
                }
                pos = Vec3(pos.head<2>().norm(), 0.0, pos.z());
                break;
            case 880:  // Fanuc's tap across the part
            case 1950: // Haas's live tool taps across the part
            case 1960:
                radialTap = true;
                break;
            case 200:
                unit = LengthUnit::Inch;
                break;
            case 210:
                unit = LengthUnit::Millimeter;
                break;
            case 280:
            case 530: // machine coordinates for the block (G53 G49 Z0.): not modelled
                home = true;
                break;
            case 400:
                path.compensation = toolpath::Compensation::None;
                break;
            case 410:
                path.compensation = toolpath::Compensation::Left;
                break;
            case 420:
                path.compensation = toolpath::Compensation::Right;
                break;
            case 430:
            case 434: // tool centre point control
            case 490:
            case 541:
                break;
            case 930:
                inverseTime = true;
                break;
            case 940:
                inverseTime = false;
                break;
            case 540:
            case 550:
            case 560:
            case 570:
            case 580:
            case 590:
            case 1540: // G154 P: extended work offsets (Haas-style)
            case 1870: // G187: smoothness / corner rounding (Haas-style)
                break;
            case 730:
            case 810:
            case 820:
            case 830:
            case 840:
            case 850:
                motion = code / 10;
                newCycle = true;
                break;
            case 800:
                motion = -1;
                break;
            case 900:
                absolute = true;
                break;
            case 910:
                absolute = false;
                break;
            case 980:
                if (options.lathe) {
                    perRev = false; // feed per minute
                } else {
                    returnToR = false;
                }
                break;
            case 990:
                if (options.lathe) {
                    perRev = true; // feed per revolution
                } else {
                    returnToR = true;
                }
                break;
            default:
                result.warnings.push_back(fmt::format("line {}: G{} ignored", lineNo, g));
            }
        }
        if (wrapBlock) {
            continue;
        }
        if (wrapYRadius > 0.0 && has('Y')) {
            w['C'] = toMm(w['Y'], unit) / wrapYRadius * 180.0 / kPi;
            w.erase('Y');
        }
        if (polar) {
            if (polarC && has('C')) {
                w['Y'] = w['C'];
                w.erase('C');
            }
            if (polarRadial && has('X')) {
                w['X'] *= 2.0; // read as a lathe's X below
            }
        }
        for (int m : ms) {
            if (m == 6 && has('T')) {
                path.toolNumber = static_cast<int>(std::lround(w['T']));
                result.toolChanges.push_back(path.toolNumber);
            }
        }
        if (options.lathe && has('T') && std::find(ms.begin(), ms.end(), 6) == ms.end()) {
            // T0101: tool 01, offset 01.
            const int t = static_cast<int>(std::lround(w['T']));
            path.toolNumber = t >= 100 ? t / 100 : t;
            result.toolChanges.push_back(path.toolNumber);
        }
        const bool g96 = std::find_if(gs.begin(), gs.end(), [](double g) {
                             return std::lround(g * 10.0) == 960;
                         }) != gs.end();
        if (has('S') && !g96 && std::find_if(gs.begin(), gs.end(), [](double g) {
                                    return std::lround(g * 10.0) == 500;
                                }) == gs.end()) {
            path.spindleSpeed = w['S'] / 60.0;
            rpm = w['S'];
        }
        if (has('F') && motion != 4) {
            if (inverseTime) {
                inverseF = w['F'];
            } else if (motion == 32) {
                pitch = toMm(w['F'], unit);
            } else if (perRev) {
                feed = toMm(w['F'], unit) * (rpm > 0.0 ? rpm / 60.0 : 1.0);
            } else {
                feed = feedPerMinToMmPerSec(w['F'], unit);
            }
        }
        if (dwell) {
            Move d;
            d.kind = MoveKind::Dwell;
            d.feedClass = FeedClass::Plunge;
            d.end = pos;
            d.dwell = has('P') ? dwellP : (has('X') ? w['X'] : 0.0);
            path.moves.push_back(d);
            continue;
        }
        if (home) {
            continue; // reference return: position becomes machine home (not modelled)
        }

        auto coordinate = [&](char axis, double current) {
            if (!has(axis)) {
                return current;
            }
            const double v = toMm(w[axis], unit) * (options.lathe && axis == 'X' ? 0.5 : 1.0);
            return absolute ? v : current + v;
        };
        bool hasRotary = false;
        for (const char letter : {'A', 'B', 'C'}) {
            if (has(letter)) {
                const double v = w[letter] * kPi / 180.0;
                double& r = letter == 'A' ? rotary.a : letter == 'B' ? rotary.b : rotary.c;
                r = absolute ? v : r + v;
                hasRotary = true;
            }
        }
        if (hasRotary && options.lathe) {
            axis = Vec3(std::cos(rotary.c), std::sin(rotary.c), 0.0); // as solving a live tool
        } else if (hasRotary && options.machine) {
            axis = machine::toolAxis(*options.machine, rotary);
        }
        const bool hasAxis = has('X') || has('Y') || has('Z') || hasRotary;

        if (radialTap) {
            // In along X to the block's X and back to where the tool stands.
            Move m;
            m.kind = MoveKind::Linear;
            m.feedClass = FeedClass::Plunge;
            m.axis = axis;
            m.end = Vec3(coordinate('X', pos.x()), pos.y(), pos.z());
            path.moves.push_back(m);
            m.end = pos;
            path.moves.push_back(m);
            continue;
        }

        if (motion >= 73) {
            // Canned cycle: parameters are modal; a block with X/Y (or the cycle code) drills a
            // hole.
            if (has('Z')) {
                if (!absolute) {
                    result.warnings.push_back(
                        fmt::format("line {}: incremental cycle depths are not supported", lineNo));
                }
                cycleZ = toMm(w['Z'], unit);
            }
            if (has('R')) {
                cycleR = toMm(w['R'], unit);
            } else if (options.lathe && newCycle) {
                cycleR = pos.z(); // without R, a lathe's cycle starts where the tool is
            }
            if (has('Q')) {
                cycleQ = toMm(w['Q'], unit);
            }
            if (has('P')) {
                cycleP = dwellP;
            }
            if (!(has('X') || has('Y') || newCycle)) {
                continue;
            }
            toolpath::DrillCycle c;
            switch (motion) {
            case 73:
                c.type = toolpath::CycleType::ChipBreak;
                break;
            case 82:
                c.type = toolpath::CycleType::Dwell;
                break;
            case 83:
                c.type = toolpath::CycleType::Peck;
                break;
            case 84:
                c.type = toolpath::CycleType::Tap;
                break;
            case 74:
                c.type = toolpath::CycleType::TapLeft;
                break;
            case 85:
                c.type = toolpath::CycleType::Bore;
                break;
            default:
                c.type = toolpath::CycleType::Drill;
                break;
            }
            c.topZ = cycleR;
            c.bottomZ = cycleZ;
            c.retractZ = cycleR;
            c.peck =
                (c.type == toolpath::CycleType::Peck || c.type == toolpath::CycleType::ChipBreak)
                    ? cycleQ
                    : 0.0;
            c.dwell = c.type == toolpath::CycleType::Dwell ? cycleP : 0.0;
            c.feedRate = feed;
            auto it = std::find(path.cycles.begin(), path.cycles.end(), c);
            const int index = static_cast<int>(it - path.cycles.begin());
            if (it == path.cycles.end()) {
                path.cycles.push_back(c);
            }
            const double initialZ = pos.z();
            Move m;
            m.kind = MoveKind::Cycle;
            m.feedClass = FeedClass::Plunge;
            m.axis = axis;
            m.end = Vec3(coordinate('X', pos.x()), coordinate('Y', pos.y()),
                         returnToR ? cycleR : std::max(initialZ, cycleR));
            m.feedRate = feed;
            m.cycle = index;
            path.moves.push_back(m);
            pos = m.end;
            continue;
        }

        if (!hasAxis || motion < 0) {
            continue;
        }
        const Vec3 target(coordinate('X', pos.x()), coordinate('Y', pos.y()),
                          coordinate('Z', pos.z()));
        Move m;
        m.end = target;
        m.axis = axis;
        if (motion == 0) {
            m.kind = MoveKind::Rapid;
            m.feedClass = FeedClass::Rapid;
        } else if (inverseTime && motion == 1) {
            // Inverse time: the move takes 1/F minutes.
            m.kind = MoveKind::Linear;
            m.feedClass = FeedClass::Cut;
            if (inverseF <= 0.0) {
                throw Error(ErrorCode::Parse,
                            fmt::format("line {}: inverse-time move without F", lineNo));
            }
            m.feedRate = (target - pos).norm() * inverseF / 60.0;
            inverseF = 0.0; // F is not modal in G93
        } else {
            if (feed <= 0.0 && motion != 32 && !warnedFeed) {
                result.warnings.push_back(
                    fmt::format("line {}: feed move without a feed rate", lineNo));
                warnedFeed = true;
            }
            m.feedRate = feed;
            m.feedClass = FeedClass::Cut;
            if (motion == 1 || motion == 32) {
                m.kind = MoveKind::Linear;
                if (motion == 32) {
                    m.pitch = pitch;
                    m.feedRate = pitch * (rpm > 0.0 ? rpm / 60.0 : 1.0);
                }
            } else {
                m.kind = motion == 2 ? MoveKind::ArcCW : MoveKind::ArcCCW;
                m.plane = plane;
                // The centre's offset words of the plane: I/J (G17), I/K (G18), J/K (G19).
                const bool useI = plane != toolpath::ArcPlane::YZ;
                const bool useJ = plane != toolpath::ArcPlane::ZX;
                const bool useK = plane != toolpath::ArcPlane::XY;
                if ((useI && has('I')) || (useJ && has('J')) || (useK && has('K'))) {
                    m.center = Vec3(pos.x() + (useI && has('I') ? toMm(w['I'], unit) : 0.0),
                                    pos.y() + (useJ && has('J') ? toMm(w['J'], unit) : 0.0),
                                    pos.z() + (useK && has('K') ? toMm(w['K'], unit) : 0.0));
                } else if (has('R')) {
                    // Centre from radius: R > 0 short arc, R < 0 long arc.
                    const double r = toMm(w['R'], unit);
                    const Vec2 a = toolpath::planeCoords(pos, plane);
                    const Vec2 b = toolpath::planeCoords(target, plane);
                    const Vec2 mid = 0.5 * (a + b);
                    const double half = 0.5 * (b - a).norm();
                    if (half < 1e-12 || std::abs(r) < half - 1e-9) {
                        throw Error(ErrorCode::Parse,
                                    fmt::format("line {}: arc radius too small", lineNo));
                    }
                    const double h = std::sqrt(std::max(0.0, r * r - half * half));
                    const Vec2 dir = (b - a) / (2.0 * half);
                    const Vec2 left(-dir.y(), dir.x());
                    // CCW short arc: centre to the left of the chord; CW short arc: to the right.
                    double side = motion == 3 ? 1.0 : -1.0;
                    if (r < 0.0) {
                        side = -side;
                    }
                    const Vec2 c = mid + side * h * left;
                    m.center =
                        toolpath::fromPlaneCoords(c, toolpath::planeNormal(pos, plane), plane);
                } else {
                    throw Error(ErrorCode::Parse,
                                fmt::format("line {}: arc without I/J or R", lineNo));
                }
            }
        }
        path.moves.push_back(m);
        pos = target;
    }
    return result;
}

std::optional<std::string> compareBackplot(const std::vector<toolpath::Toolpath>& expected,
                                           const toolpath::Toolpath& plottedIn, double tol,
                                           double* maxDeviation, bool compareFeeds) {
    const toolpath::Toolpath plotted = toolpath::expandCycles(plottedIn);
    double maxDev = 0.0;
    std::size_t pi = 0;
    Vec3 ppos = plotted.start;
    auto advance = [&]() {
        const Move& m = plotted.moves[pi++];
        if (m.kind != MoveKind::Dwell) {
            ppos = m.end;
        }
    };
    auto finish = [&](std::optional<std::string> r) {
        if (maxDeviation) {
            *maxDeviation = maxDev;
        }
        return r;
    };

    for (std::size_t k = 0; k < expected.size(); ++k) {
        const toolpath::Toolpath e = toolpath::expandCycles(expected[k]);
        const bool multiAxis = toolpath::isMultiAxis(e);
        while (pi < plotted.moves.size() && (ppos - e.start).norm() > tol) {
            advance();
        }
        if ((ppos - e.start).norm() > tol) {
            return finish(
                fmt::format("toolpath {}: the program never reaches its start position", k + 1));
        }
        Vec3 epos = e.start;
        for (std::size_t mi = 0; mi < e.moves.size(); ++mi) {
            const Move& em = e.moves[mi];
            if (em.kind != MoveKind::Dwell && moveLength(epos, em) < tol) {
                epos = em.end;
                continue;
            }
            // A machine function with no dwell is a code the program does not move by (DL-0091).
            if (em.kind == MoveKind::Dwell && em.function != toolpath::MachineFunction::None &&
                em.dwell <= 0.0) {
                continue;
            }
            while (pi < plotted.moves.size() && plotted.moves[pi].kind != MoveKind::Dwell &&
                   moveLength(ppos, plotted.moves[pi]) < tol) {
                advance();
            }
            if (pi >= plotted.moves.size()) {
                return finish(
                    fmt::format("toolpath {}: the program ends before move {}", k + 1, mi + 1));
            }
            const auto describe = [&](const char* what, double dev) {
                return fmt::format("toolpath {}, move {} ({}): {} differs by {:.6f} mm", k + 1,
                                   mi + 1, toolpath::toString(em.kind), what, dev);
            };
            // A post may write a rapid as a feed move (Haas-style 5-axis: no rapids in G93
            // motion); the end point is still compared, not the feed. Not the other way round.
            const bool rapidAsFeed =
                em.kind == MoveKind::Rapid && plotted.moves[pi].kind == MoveKind::Linear;
            if (plotted.moves[pi].kind != em.kind && !rapidAsFeed) {
                return finish(fmt::format("toolpath {}, move {}: expected {}, program has {}",
                                          k + 1, mi + 1, toolpath::toString(em.kind),
                                          toolpath::toString(plotted.moves[pi].kind)));
            }
            if (em.isArc()) {
                // The post may split long arcs: consume plotted pieces on the same circle.
                while (true) {
                    const Move& pm = plotted.moves[pi];
                    const double centreDev = pm.plane == em.plane
                                                 ? (toolpath::planeCoords(pm.center, em.plane) -
                                                    toolpath::planeCoords(em.center, em.plane))
                                                       .norm()
                                                 : std::numeric_limits<double>::infinity();
                    maxDev = std::max(maxDev, centreDev);
                    if (centreDev > tol) {
                        return finish(describe("arc centre", centreDev));
                    }
                    if ((pm.end - em.end).norm() <= tol || pi + 1 >= plotted.moves.size() ||
                        plotted.moves[pi + 1].kind != em.kind) {
                        break;
                    }
                    advance();
                }
            }
            const Move& pm = plotted.moves[pi];
            const double dev = (pm.end - em.end).norm();
            maxDev = std::max(maxDev, dev);
            if (dev > tol) {
                return finish(describe("end point", dev));
            }
            if (em.kind == MoveKind::Dwell && std::abs(pm.dwell - em.dwell) > 1e-3) {
                return finish(describe("dwell", std::abs(pm.dwell - em.dwell)));
            }
            if (multiAxis) {
                const double axisDev = (pm.axis - em.axis).norm();
                if (axisDev > 1e-4) {
                    return finish(fmt::format("toolpath {}, move {}: tool axis differs by {:.6f}",
                                              k + 1, mi + 1, axisDev));
                }
            }
            double feedTol = std::max(0.51 / 60.0, 1e-6 * em.feedRate);
            if (multiAxis) {
                // Inverse-time feeds of short moves: rounding of the positions shows in the feed
                // along them.
                feedTol = std::max(feedTol,
                                   em.feedRate * 2.0 * tol / std::max(moveLength(epos, em), tol));
            }
            if (compareFeeds && em.kind != MoveKind::Rapid && em.kind != MoveKind::Dwell &&
                std::abs(pm.feedRate - em.feedRate) > feedTol) {
                return finish(
                    fmt::format("toolpath {}, move {}: feed {:.1f} mm/min, program has {:.1f}",
                                k + 1, mi + 1, em.feedRate * 60.0, pm.feedRate * 60.0));
            }
            advance();
            epos = em.end;
        }
    }
    return finish(std::nullopt);
}

} // namespace ocam::post
