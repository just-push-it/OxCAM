// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// See docs/design-log/0068.

#include "ocam/document/LatheFitting.hpp"

#include <ocam/foundation/Error.hpp>
#include <ocam/foundation/Units.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <cmath>
#include <set>

namespace ocam::embedded {
extern const std::string_view fittingSchemaV2;
extern const std::string_view fittingRadialBlock;
extern const std::string_view fittingAxialBlock;
extern const std::string_view fittingLiveCentre;
} // namespace ocam::embedded

namespace ocam::document {
namespace {

Json vecJson(const Vec3& v) {
    return Json::array({v.x(), v.y(), v.z()});
}

Vec3 vecFrom(const Json& j) {
    return {j.at(0).get<double>(), j.at(1).get<double>(), j.at(2).get<double>()};
}

Json pointJson(const FittingPoint& p) {
    Json j{{"point", vecJson(p.point)}, {"direction", vecJson(p.direction)}};
    if (!p.on.empty()) {
        j["on"] = p.on;
    }
    return j;
}

FittingPoint pointFrom(const Json& j) {
    FittingPoint p;
    p.point = vecFrom(j.at("point"));
    p.direction = vecFrom(j.at("direction"));
    p.on = j.value("on", std::string());
    return p;
}

Vec3 perpendicularPart(const Vec3& v, const Vec3& normal) {
    return v - normal * normal.dot(v);
}

} // namespace

const FormatSpec& fittingFormat() {
    static const FormatSpec spec = [] {
        FormatSpec s{"oxcam.fitting", kFittingVersion, {}, {}};
        // Version 2: what a block takes is said, not read from a size of 0. A size stays a stated
        // size; one of 0 was "any", which is now a size not stated.
        s.migrations[1] = [](Json& doc) {
            const double shank = doc.value("shank", 0.0);
            const double bore = doc.value("bore", 0.0);
            doc.erase("shank");
            doc.erase("bore");
            if (doc.value("kind", std::string()) != "block") {
                return;
            }
            doc["takes"] = shank > 0.0 && bore <= 0.0   ? "square"
                           : bore > 0.0 && shank <= 0.0 ? "round"
                                                        : "both";
            if (shank > 0.0) {
                doc["square_shank"] = shank;
            }
            if (bore > 0.0) {
                doc["round_bar"] = bore;
            }
        };
        return s;
    }();
    return spec;
}

const SchemaValidator& fittingSchema() {
    static const SchemaValidator validator(
        parseJson(embedded::fittingSchemaV2, "fitting.v2.schema.json"));
    return validator;
}

Json toJson(const LatheFitting& f) {
    Json j{{"format", fittingFormat().format},
           {"version", kFittingVersion},
           {"id", f.id.toString()},
           {"name", f.name},
           {"kind", f.kind == LatheFitting::Kind::Block ? "block" : "centre"},
           {"step", f.sha256},
           {"colour", f.colour},
           {"seat", pointJson(f.seat)},
           {"seat_turn", f.seatTurn},
           {"mount", pointJson(f.mount)},
           {"overhang", f.overhang}};
    if (!f.description.empty()) {
        j["description"] = f.description;
    }
    if (f.kind == LatheFitting::Kind::Block) {
        j["takes"] = f.takes == LatheFitting::Takes::Square  ? "square"
                     : f.takes == LatheFitting::Takes::Round ? "round"
                                                             : "both";
        if (f.squareShank && f.takesSquare()) {
            j["square_shank"] = *f.squareShank;
        }
        if (f.roundBar && f.takesRound()) {
            j["round_bar"] = *f.roundBar;
        }
    }
    if (!f.mountType.empty()) {
        j["mount_type"] = f.mountType;
    }
    return j;
}

LatheFitting fittingFromJson(Json doc, std::string_view sourceName) {
    fittingFormat().upgrade(doc, sourceName);
    fittingSchema().validate(doc, sourceName);
    try {
        LatheFitting f;
        f.id = *Uuid::parse(doc.at("id").get<std::string>());
        f.name = doc.at("name").get<std::string>();
        f.description = doc.value("description", std::string());
        f.kind = doc.at("kind").get<std::string>() == "block" ? LatheFitting::Kind::Block
                                                              : LatheFitting::Kind::Centre;
        f.sha256 = doc.at("step").get<std::string>();
        f.colour = doc.value("colour", f.colour);
        f.seat = pointFrom(doc.at("seat"));
        f.seatTurn = doc.value("seat_turn", 0.0);
        f.mount = pointFrom(doc.at("mount"));
        f.overhang = doc.value("overhang", f.overhang);
        const std::string takes = doc.value("takes", std::string("square"));
        f.takes = takes == "round"  ? LatheFitting::Takes::Round
                  : takes == "both" ? LatheFitting::Takes::Both
                                    : LatheFitting::Takes::Square;
        if (const auto it = doc.find("square_shank"); it != doc.end()) {
            f.squareShank = it->get<double>();
        }
        if (const auto it = doc.find("round_bar"); it != doc.end()) {
            f.roundBar = it->get<double>();
        }
        f.mountType = doc.value("mount_type", std::string());
        return f;
    } catch (const Json::exception& e) {
        throw Error(ErrorCode::Parse, fmt::format("{}: {}", sourceName, e.what()));
    }
}

std::vector<std::string> LatheFitting::validate() const {
    std::vector<std::string> issues;
    if (name.empty()) {
        issues.emplace_back("a fitting needs a name");
    }
    if (sha256.empty()) {
        issues.emplace_back("a fitting needs a model");
    }
    if (seat.direction.norm() < 1e-9 || mount.direction.norm() < 1e-9) {
        issues.emplace_back("a fitting's points need directions");
        return issues;
    }
    if (kind == Kind::Block) {
        const double along = seat.direction.normalized().dot(mount.direction.normalized());
        if (std::abs(std::abs(along) - 1.0) > 1e-3 && std::abs(along) > 1e-2) {
            issues.emplace_back(
                "a block's tool must point the way its seat does (a tool across the "
                "spindle) or square to it (a tool along the spindle)");
        } else if (along < -0.5) {
            issues.emplace_back("the block's tool points into the turret: reverse its direction");
        }
        if (overhang <= 0.0) {
            issues.emplace_back("a block's tool needs to stick out of it: set its overhang");
        }
    } else if ((mount.point - seat.point).norm() < 1e-9) {
        issues.emplace_back("a centre's tip and seat cannot be the same point");
    }
    if ((squareShank && *squareShank <= 0.0) || (roundBar && *roundBar <= 0.0)) {
        issues.emplace_back("a size a block takes must be more than 0: set it, or say it does not "
                            "state one");
    }
    return issues;
}

FittingPackage makeFitting(std::string name, LatheFitting::Kind kind, const cad::Model& model) {
    FittingPackage out;
    out.file = stepFileOf(model);
    out.fitting.name = std::move(name);
    out.fitting.kind = kind;
    out.fitting.sha256 = out.file.sha256;
    return out;
}

const std::vector<FittingPackage>& builtinFittings() {
    static const std::vector<FittingPackage> all = [] {
        std::vector<FittingPackage> out;
        const auto add = [&](const char* id, const char* name, const char* description,
                             LatheFitting::Kind kind, std::string_view step, FittingPoint seat,
                             FittingPoint mount, double overhang, LatheFitting::Takes takes,
                             std::optional<double> square, std::optional<double> round) {
            FittingPackage p;
            p.builtin = true;
            p.file = stepFile(std::make_shared<const std::string>(step));
            p.fitting.id = *Uuid::parse(id);
            p.fitting.name = name;
            p.fitting.description = description;
            p.fitting.kind = kind;
            p.fitting.sha256 = p.file.sha256;
            p.fitting.seat = std::move(seat);
            p.fitting.mount = std::move(mount);
            p.fitting.overhang = overhang;
            p.fitting.takes = takes;
            p.fitting.squareShank = square;
            p.fitting.roundBar = round;
            p.fitting.mountType =
                kind == LatheFitting::Kind::Block ? kGenericBlockMount : kGenericQuillInterface;
            out.push_back(std::move(p));
        };
        // Sizes are placeholders: nothing here is any maker's block. See tests/fixtures/gen.
        add("0a5e1e0d-7a11-4570-8c6b-000000000101", "Block for a 25 mm shank across (generic)",
            "An example, with placeholder sizes: duplicate it and put in your own model.",
            LatheFitting::Kind::Block, embedded::fittingRadialBlock,
            {Vec3::Zero(), Vec3::UnitZ(), "the bottom of the block"},
            {Vec3(27.5, 0.0, 60.0), Vec3::UnitZ(), "the mouth of the slot"}, 30.0,
            LatheFitting::Takes::Square, 25.0, std::nullopt);
        add("0a5e1e0d-7a11-4570-8c6b-000000000102", "Block for a 32 mm bar along (generic)",
            "An example, with placeholder sizes: duplicate it and put in your own model.",
            LatheFitting::Kind::Block, embedded::fittingAxialBlock,
            {Vec3::Zero(), Vec3::UnitZ(), "the bottom of the block"},
            {Vec3(40.0, 0.0, 35.0), Vec3::UnitX(), "the mouth of the bore"}, 60.0,
            LatheFitting::Takes::Round, std::nullopt, 32.0);
        add("0a5e1e0d-7a11-4570-8c6b-000000000103", "Live centre (generic)",
            "An example, with placeholder sizes: duplicate it and put in your own model.",
            LatheFitting::Kind::Centre, embedded::fittingLiveCentre,
            {Vec3::Zero(), Vec3::UnitZ(), "the back of the body"},
            {Vec3(0.0, 0.0, 99.0), Vec3::UnitZ(), "the point"}, 0.0, LatheFitting::Takes::Square,
            std::nullopt, std::nullopt);
        return out;
    }();
    return all;
}

namespace {

std::string quoted(const std::vector<std::string>& names) {
    std::string out;
    for (std::size_t k = 0; k < names.size(); ++k) {
        out += (k == 0 ? "'" : k + 1 == names.size() ? " or '" : ", '") + names[k] + "'";
    }
    return out;
}

bool named(const std::vector<std::string>& names, const std::string& name) {
    return std::any_of(names.begin(), names.end(),
                       [&](const std::string& n) { return machine::sameMountName(n, name); });
}

} // namespace

std::optional<std::string> mountMismatch(const LatheFitting& fitting,
                                         const machine::Machine& machine, int station) {
    if (!machine.lathe) {
        return std::nullopt;
    }
    if (fitting.kind == LatheFitting::Kind::Block) {
        const machine::LatheTurret& t = machine.lathe->turret;
        if (station <= 0 && !t.stationMounts.empty()) {
            // On no station in particular: it fits when some station takes it.
            std::vector<std::string> all;
            for (int k = 1; k <= std::max(machine.toolCapacity, 1); ++k) {
                if (!mountMismatch(fitting, machine, k)) {
                    return std::nullopt;
                }
                for (const std::string& kind : t.blockMountsAt(k)) {
                    if (!named(all, kind)) {
                        all.push_back(kind);
                    }
                }
            }
            return fitting.mountType.empty()
                       ? fmt::format("block '{}' does not say how it mounts, and machine '{}' "
                                     "takes {}",
                                     fitting.name, machine.name, quoted(all))
                       : fmt::format("block '{}' mounts by '{}', and machine '{}' takes {}",
                                     fitting.name, fitting.mountType, machine.name, quoted(all));
        }
        const std::vector<std::string>& takes = t.blockMountsAt(station);
        if (takes.empty()) {
            return std::nullopt;
        }
        const std::string where =
            station > 0 && !t.stationMounts.empty()
                ? fmt::format("station {} of machine '{}'", station, machine.name)
                : fmt::format("machine '{}'", machine.name);
        if (fitting.mountType.empty()) {
            return fmt::format("block '{}' does not say how it mounts, and {} takes {}",
                               fitting.name, where, quoted(takes));
        }
        if (named(takes, fitting.mountType)) {
            return std::nullopt;
        }
        return fmt::format("block '{}' mounts by '{}', and {} takes {}", fitting.name,
                           fitting.mountType, where, quoted(takes));
    }
    if (!machine.lathe->tailstock || machine.lathe->tailstock->quillInterface.empty()) {
        return std::nullopt;
    }
    const std::string& quill = machine.lathe->tailstock->quillInterface;
    if (fitting.mountType.empty()) {
        return fmt::format("centre '{}' does not say what it fits, and machine '{}''s quill takes "
                           "'{}'",
                           fitting.name, machine.name, quill);
    }
    if (machine::sameMountName(quill, fitting.mountType)) {
        return std::nullopt;
    }
    return fmt::format("centre '{}' fits '{}', and machine '{}''s quill takes '{}'", fitting.name,
                       fitting.mountType, machine.name, quill);
}

void saveFitting(const std::filesystem::path& folder, const FittingPackage& package) {
    if (const auto issues = package.fitting.validate(); !issues.empty()) {
        throw Error(ErrorCode::InvalidArgument, issues.front());
    }
    if (!package.file.bytes) {
        throw Error(ErrorCode::InvalidArgument,
                    fmt::format("the model of '{}' is not loaded", package.fitting.name));
    }
    std::error_code ec;
    std::filesystem::create_directories(folder, ec);
    writeFileAtomic(folder / (package.file.sha256 + ".step"), *package.file.bytes);
    writeFileAtomic(folder / (package.fitting.id.toString() + ".json"),
                    dumpJson(toJson(package.fitting)));
}

std::vector<FittingPackage> loadFittings(const std::filesystem::path& folder,
                                         std::vector<std::string>* problems) {
    std::vector<FittingPackage> out;
    std::error_code ec;
    for (const auto& item : std::filesystem::directory_iterator(folder, ec)) {
        if (!item.is_regular_file() || item.path().extension() != ".json") {
            continue;
        }
        try {
            const std::u8string u8 = item.path().filename().u8string();
            const std::string name(u8.begin(), u8.end());
            FittingPackage p;
            p.fitting = fittingFromJson(parseJson(readFileBytes(item.path()), name), name);
            p.file = stepFile(std::make_shared<const std::string>(
                readFileBytes(folder / (p.fitting.sha256 + ".step"))));
            if (p.file.sha256 != p.fitting.sha256) {
                throw Error(ErrorCode::Parse, fmt::format("{}: its model has changed", name));
            }
            out.push_back(std::move(p));
        } catch (const Error& e) {
            if (problems) {
                problems->push_back(e.what());
            }
        }
    }
    std::sort(out.begin(), out.end(), [](const FittingPackage& a, const FittingPackage& b) {
        return a.fitting.name < b.fitting.name;
    });
    return out;
}

bool removeFittingFile(const std::filesystem::path& folder, const Uuid& id) {
    std::error_code ec;
    const std::filesystem::path file = folder / (id.toString() + ".json");
    std::string sha;
    try {
        sha = parseJson(readFileBytes(file), "fitting").value("step", std::string());
    } catch (const Error&) {
        return false;
    }
    const bool removed = std::filesystem::remove(file, ec);
    bool used = false;
    for (const FittingPackage& p : loadFittings(folder)) {
        used = used || p.fitting.sha256 == sha;
    }
    if (!used && !sha.empty()) {
        std::filesystem::remove(folder / (sha + ".step"), ec);
    }
    return removed;
}

bool blockAcross(const LatheFitting& block) {
    return std::abs(block.seat.direction.normalized().dot(block.mount.direction.normalized())) >
           0.5;
}

Vec3 towardsWork(const LatheFitting& block) {
    const Vec3 z = block.seat.direction.normalized();
    Vec3 x = Vec3::Zero();
    if (!blockAcross(block)) {
        x = perpendicularPart(block.mount.direction.normalized(), z);
    } else {
        x = perpendicularPart(block.mount.point - block.seat.point, z);
    }
    if (x.norm() < 1e-6) {
        x = perpendicularPart(Vec3::UnitX(), z);
    }
    if (x.norm() < 1e-6) {
        x = perpendicularPart(Vec3::UnitY(), z);
    }
    return Eigen::AngleAxisd(degToRad(block.seatTurn), z) * x.normalized();
}

Vec3 blockUp(const LatheFitting& block) {
    return block.seat.direction.normalized().cross(towardsWork(block)).normalized();
}

Transform3 blockOnStation(const LatheFitting& block, const machine::LatheLayout& layout,
                          bool towardsPlusZ) {
    const machine::LatheTurret& t = layout.turret;
    // The station's frame in the working position: Z towards the spindle, X towards the work.
    const Vec3 sz = -Vec3::UnitX();
    const Vec3 sx = towardsPlusZ ? Vec3(Vec3::UnitZ()) : Vec3(-Vec3::UnitZ());
    const Vec3 sy = sz.cross(sx);
    Mat3 station;
    station << sx, sy, sz;
    const Vec3 bz = block.seat.direction.normalized();
    const Vec3 bx = towardsWork(block);
    const Vec3 by = bz.cross(bx);
    Mat3 own;
    own << bx, by, bz;
    Transform3 xf = Transform3::Identity();
    xf.linear() = station * own.transpose();
    const Vec3 seatAt = t.centre + Vec3(-t.blockRadius, t.blockAcross, t.blockAlong);
    xf.translation() = seatAt - xf.linear() * block.seat.point;
    return xf;
}

Transform3 centreInWcs(const LatheFitting& centre, double tipZ) {
    // From tip to seat: +Z of the WCS, the quill's side.
    const Vec3 z = (centre.seat.point - centre.mount.point).normalized();
    const Vec3 r = std::abs(z.x()) < 0.9 ? Vec3(Vec3::UnitX()) : Vec3(Vec3::UnitY());
    const Vec3 x = r.cross(z).normalized().cross(z).normalized();
    const Vec3 y = z.cross(x);
    Mat3 own;
    own << x, y, z;
    Transform3 xf = Transform3::Identity();
    xf.linear() = own.transpose();
    xf.translation() = Vec3(0.0, 0.0, tipZ) - xf.linear() * centre.mount.point;
    return xf;
}

double centreLength(const LatheFitting& centre) {
    return (centre.seat.point - centre.mount.point).norm();
}

void removeUnusedFittings(Project& project) {
    std::set<Uuid> used;
    for (const Setup& s : project.setups) {
        for (const StationBlock& b : s.turretBlocks) {
            used.insert(b.block);
        }
        if (s.tailstock && s.tailstock->centre) {
            used.insert(*s.tailstock->centre);
        }
    }
    std::erase_if(project.fittings, [&](const LatheFitting& f) { return !used.contains(f.id); });
}

} // namespace ocam::document
