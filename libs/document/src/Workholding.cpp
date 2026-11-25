// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/document/Workholding.hpp"

#include "ocam/document/Document.hpp"
#include "ocam/document/Generate.hpp"
#include "ocam/document/PostSetup.hpp"
#include "ocam/document/Simulate.hpp"

#include <ocam/cad/Datum.hpp>
#include <ocam/cad/Revolve.hpp>
#include <ocam/foundation/Error.hpp>
#include <ocam/foundation/Math.hpp>
#include <ocam/foundation/Sha256.hpp>
#include <ocam/foundation/Units.hpp>
#include <ocam/geom2d/Clip.hpp>
#include <ocam/machine/Machine.hpp>

#include <fmt/format.h>
#include <miniz.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <regex>
#include <set>

namespace ocam::embedded {
extern const std::string_view workholdingSchemaV4;
}

namespace ocam::document {
namespace {

// Refuse archive entries above this size (guards against decompression bombs).
constexpr mz_uint64 kMaxEntryBytes = 1ULL * 1024 * 1024 * 1024;
constexpr double kTol = 1e-6;

Error userError(const std::string& message) {
    return Error(ErrorCode::InvalidArgument, message);
}

Json vec(const Vec3& v) {
    return Json::array({v.x(), v.y(), v.z()});
}

Vec3 vecFrom(const Json& j) {
    return {j.at(0).get<double>(), j.at(1).get<double>(), j.at(2).get<double>()};
}

Uuid uuidFrom(const Json& j) {
    const auto id = Uuid::parse(j.get<std::string>());
    if (!id) {
        throw Error(ErrorCode::Parse, "invalid UUID: " + j.get<std::string>());
    }
    return *id;
}

struct ZipReader {
    mz_zip_archive zip{};
    ~ZipReader() { mz_zip_reader_end(&zip); }
};

struct ZipWriter {
    mz_zip_archive zip{};
    ~ZipWriter() { mz_zip_writer_end(&zip); }
};

/// The smallest and largest projection of a box's corners on a direction.
std::pair<double, double> extent(const geom3d::Aabb3& box, const Vec3& dir) {
    double lo = std::numeric_limits<double>::infinity();
    double hi = -lo;
    for (int i = 0; i < 8; ++i) {
        const Vec3 corner((i & 1) ? box.max.x() : box.min.x(), (i & 2) ? box.max.y() : box.min.y(),
                          (i & 4) ? box.max.z() : box.min.z());
        lo = std::min(lo, corner.dot(dir));
        hi = std::max(hi, corner.dot(dir));
    }
    return {lo, hi};
}

// Version 1: bodies with jaw roles, one datum frame (origin on the fixed jaw face, or midway
// between the jaw faces, at the jaw tops; X the opening direction, Z up) and an opening range.
// It becomes a moving jaw member (and its linked opposite jaw), typed datums on the jaw faces
// and tops, and one jaw holding: the stock sits exactly as it did.
void migrateFromV1(Json& doc) {
    const Json& frame = doc.at("datum");
    const Vec3 origin = vecFrom(frame.at("origin"));
    const Vec3 z = vecFrom(frame.at("z_axis")).normalized();
    Vec3 x = vecFrom(frame.at("x_axis"));
    x = (x - x.dot(z) * z).normalized();
    const bool centring = doc.value("kind", "vise") == "self_centring";
    const double modelled = doc.at("modelled_opening").get<double>();
    const double lo = doc.at("min_opening").get<double>() - modelled;
    const double hi = doc.at("max_opening").get<double>() - modelled;
    const double share = centring ? 0.5 : 1.0;

    Json members = Json::array();
    members.push_back({{"name", "Moving jaw"},
                       {"direction", vec(x)},
                       {"min_travel", share * lo},
                       {"max_travel", share * hi}});
    if (centring) {
        members.push_back({{"name", "Opposite jaw"},
                           {"direction", vec(-x)},
                           {"min_travel", share * lo},
                           {"max_travel", share * hi},
                           {"linked_to", "Moving jaw"},
                           {"ratio", 1.0}});
    }
    Json parts = Json::array();
    for (const Json& b : doc.at("bodies")) {
        const std::string role = b.at("role").get<std::string>();
        parts.push_back({{"id", b.at("id")},
                         {"name", b.value("name", "")},
                         {"sha256", b.at("sha256")},
                         {"source_path", b.value("source_path", "")},
                         {"member", role == "jaw"            ? "Moving jaw"
                                    : role == "opposite_jaw" ? "Opposite jaw"
                                                             : ""}});
    }
    auto typed = [](const char* name, const Vec3& point, const Vec3& direction,
                    const char* member) {
        return Json{{"name", name},     {"kind", "typed"},     {"shape", "plane"},
                    {"member", member}, {"point", vec(point)}, {"direction", vec(direction)}};
    };
    const char* sideFace = centring ? "Opposite jaw face" : "Fixed jaw face";
    Json datums = Json::array();
    datums.push_back(typed(sideFace, centring ? Vec3(origin - 0.5 * modelled * x) : origin, x,
                           centring ? "Opposite jaw" : ""));
    datums.push_back(typed("Moving jaw face", origin + share * modelled * x, -x, "Moving jaw"));
    datums.push_back(typed("Jaw tops", origin, z, ""));
    Json holdings = Json::array();
    holdings.push_back({{"name", "Jaws"},
                        {"kind", "jaws"},
                        {"face_a", sideFace},
                        {"face_b", "Moving jaw face"},
                        {"seat", "Jaw tops"},
                        {"grip_depth", 5.0}});
    for (const char* key :
         {"kind", "datum", "modelled_opening", "min_opening", "max_opening", "bodies"}) {
        doc.erase(key);
    }
    doc["parts"] = std::move(parts);
    doc["members"] = std::move(members);
    doc["datums"] = std::move(datums);
    doc["holdings"] = std::move(holdings);
    doc["up"] = vec(z);
}

Json signatureJson(const cad::ShapeSignature& s) {
    return {{"centroid", vec(s.centroid)}, {"direction", vec(s.direction)}, {"measure", s.measure}};
}

cad::ShapeSignature signatureFrom(const Json& j) {
    cad::ShapeSignature s;
    s.centroid = vecFrom(j.at("centroid"));
    s.direction = vecFrom(j.at("direction"));
    s.measure = j.at("measure").get<double>();
    return s;
}

const char* typeName(cad::SubShapeType t) {
    return t == cad::SubShapeType::Edge     ? "edge"
           : t == cad::SubShapeType::Vertex ? "vertex"
                                            : "face";
}

Json datumJson(const WorkholdingDatum& d) {
    Json j{{"name", d.name}, {"kind", std::string(toString(d.kind))}};
    switch (d.kind) {
    case DatumKind::Picked:
        j["part"] = d.part ? d.part->toString() : std::string();
        j["type"] = typeName(d.type);
        j["index"] = d.index;
        j["signature"] = signatureJson(d.signature);
        [[fallthrough]];
    case DatumKind::Typed:
        j["shape"] = std::string(toString(d.shape));
        j["point"] = vec(d.point);
        j["direction"] = vec(d.direction);
        if (d.kind == DatumKind::Typed) {
            j["member"] = d.member;
        }
        break;
    case DatumKind::Offset:
        j["from"] = d.from;
        j["along"] = std::string(toString(d.along));
        j["distance"] = d.distance;
        break;
    case DatumKind::Midpoint:
        j["from"] = d.from;
        j["to"] = d.to;
        break;
    }
    return j;
}

WorkholdingDatum datumFrom(const Json& j) {
    WorkholdingDatum d;
    d.name = j.at("name").get<std::string>();
    d.kind = parseDatumKind(j.at("kind").get<std::string>()).value();
    if (j.contains("part") && !j.at("part").get<std::string>().empty()) {
        d.part = uuidFrom(j.at("part"));
    }
    const std::string type = j.value("type", "face");
    d.type = type == "edge"     ? cad::SubShapeType::Edge
             : type == "vertex" ? cad::SubShapeType::Vertex
                                : cad::SubShapeType::Face;
    d.index = j.value("index", 0);
    if (j.contains("signature")) {
        d.signature = signatureFrom(j.at("signature"));
    }
    d.shape = parseDatumShape(j.value("shape", "point")).value();
    d.member = j.value("member", "");
    if (j.contains("point")) {
        d.point = vecFrom(j.at("point"));
    }
    if (j.contains("direction")) {
        d.direction = vecFrom(j.at("direction"));
    }
    d.from = j.value("from", "");
    d.to = j.value("to", "");
    d.along = parseDatumAxis(j.value("along", "z")).value();
    d.distance = j.value("distance", 0.0);
    return d;
}

const WorkholdingDatum& requireDatum(const WorkholdingTemplate& w, std::string_view name) {
    const WorkholdingDatum* d = w.findDatum(name);
    if (!d) {
        throw userError(fmt::format("workholding '{}' has no datum '{}'", w.name, name));
    }
    return *d;
}

/// The member a datum moves with, following offsets and midpoints to their first datum.
std::string datumMember(const WorkholdingTemplate& w, std::string_view name, int depth = 0) {
    const WorkholdingDatum* d = w.findDatum(name);
    if (!d || depth > 64) {
        return {};
    }
    switch (d->kind) {
    case DatumKind::Picked: {
        const WorkholdingPart* p = d->part ? w.findPart(*d->part) : nullptr;
        return p ? p->member : std::string();
    }
    case DatumKind::Typed:
        return d->member;
    default:
        return datumMember(w, d->from, depth + 1);
    }
}

DatumValue datumValue(const WorkholdingTemplate& w, std::string_view name,
                      const std::map<std::string, double>& travel, int depth) {
    if (depth > 64) {
        throw userError(fmt::format("datum '{}' of '{}' is defined from itself", name, w.name));
    }
    const WorkholdingDatum& d = requireDatum(w, name);
    auto moved = [&](const std::string& member) {
        const WorkholdingMember* m = member.empty() ? nullptr : w.findMember(member);
        const auto t = travel.find(member);
        return m && t != travel.end() ? Vec3(m->direction.normalized() * t->second)
                                      : Vec3(Vec3::Zero());
    };
    auto unit = [](const Vec3& v) {
        return v.norm() > 1e-12 ? Vec3(v.normalized()) : Vec3(Vec3::Zero());
    };
    switch (d.kind) {
    case DatumKind::Picked:
    case DatumKind::Typed: {
        const std::string member = d.kind == DatumKind::Typed ? d.member : datumMember(w, d.name);
        return {d.point + moved(member), unit(d.direction)};
    }
    case DatumKind::Offset: {
        const DatumValue from = datumValue(w, d.from, travel, depth + 1);
        const Vec3 axis = d.along == DatumAxis::X   ? Vec3::UnitX()
                          : d.along == DatumAxis::Y ? Vec3::UnitY()
                          : d.along == DatumAxis::Z ? Vec3::UnitZ()
                                                    : from.direction;
        if (axis.norm() < 1e-12) {
            throw userError(fmt::format("datum '{}' of '{}' moves along the direction of '{}', "
                                        "which has none",
                                        d.name, w.name, d.from));
        }
        return {from.point + d.distance * axis, from.direction};
    }
    case DatumKind::Midpoint: {
        const DatumValue a = datumValue(w, d.from, travel, depth + 1);
        const DatumValue b = datumValue(w, d.to, travel, depth + 1);
        return {0.5 * (a.point + b.point), a.direction};
    }
    }
    return {};
}

/// The parts of a placed workholding.
std::vector<PosedPart> posedParts(const WorkholdingTemplate& w, const Transform3& toWcs,
                                  const std::map<std::string, double>& travel) {
    std::vector<PosedPart> out;
    for (const WorkholdingPart& p : w.parts) {
        Transform3 move = Transform3::Identity();
        if (const WorkholdingMember* m = p.member.empty() ? nullptr : w.findMember(p.member)) {
            const auto t = travel.find(p.member);
            move.translation() = m->direction.normalized() * (t == travel.end() ? 0.0 : t->second);
        }
        out.push_back({p.id, p.name, p.sha256, p.member, p.colour, p.collides, p.shown,
                       toWcs * move, std::nullopt});
    }
    return out;
}

using OpeningFn = std::function<double(const std::map<std::string, double>&)>;

/// The member that drives an opening and how much the opening changes per millimetre of its
/// travel: the member that moves on its own and changes the opening, preferably the one `prefer`
/// moves with (its leader when it follows another). Nullopt when no member changes it.
struct Driver {
    std::string member;
    double slope = 0.0; ///< Opening change per mm of the member's travel.
};
std::optional<Driver> driverOf(const WorkholdingTemplate& w,
                               const std::map<std::string, double>& own, const std::string& prefer,
                               const OpeningFn& opening) {
    std::vector<std::string> candidates;
    if (!prefer.empty()) {
        candidates.push_back(prefer);
    }
    for (const WorkholdingMember& m : w.members) {
        if (m.linkedTo.empty() && m.name != prefer) {
            candidates.push_back(m.name);
        }
    }
    const double now = opening(memberTravels(w, own));
    for (std::string driver : candidates) {
        if (const WorkholdingMember* m = w.findMember(driver); m && !m->linkedTo.empty()) {
            driver = m->linkedTo;
        }
        const double start = own.contains(driver) ? own.at(driver) : 0.0;
        std::map<std::string, double> probe = own;
        probe[driver] = start + 1.0;
        const double k = opening(memberTravels(w, probe)) - now;
        if (std::abs(k) >= 1e-9) {
            return Driver{driver, k};
        }
    }
    return std::nullopt;
}

/// Closes the members on an opening: `opening(travel)` is the opening with the members at
/// travel (memberTravels). The driving member (driverOf) is set so the opening is `target`;
/// nothing moves when none drives it and the opening already is the target.
std::map<std::string, double> closeOn(const WorkholdingTemplate& w,
                                      const std::map<std::string, double>& own, double target,
                                      const std::string& prefer, const OpeningFn& opening,
                                      const std::string& what) {
    const double now = opening(memberTravels(w, own));
    if (const auto d = driverOf(w, own, prefer, opening)) {
        std::map<std::string, double> closed = own;
        closed[d->member] =
            (own.contains(d->member) ? own.at(d->member) : 0.0) + (target - now) / d->slope;
        return memberTravels(w, closed);
    }
    if (std::abs(target - now) > kTol) {
        throw userError(fmt::format("no member of '{}' moves {}: they stay {:.3f} mm apart, not "
                                    "{:.3f} mm",
                                    w.name, what, now, target));
    }
    return memberTravels(w, own);
}

Vec3 upOf(const WorkholdingTemplate& w) {
    if (w.up.norm() < 1e-9) {
        throw userError(fmt::format("workholding '{}' has no up direction", w.name));
    }
    return w.up.normalized();
}

/// A chuck's axis: its point where the axis crosses the plane of the seat (the front of the
/// jaws), its direction out of the chuck (the seat's side).
struct ChuckAxis {
    Vec3 point = Vec3::Zero();
    Vec3 direction = Vec3::UnitZ();
};
ChuckAxis chuckAxis(const WorkholdingTemplate& w, const StockHolding& h,
                    const std::map<std::string, double>& travel) {
    const DatumValue a = datumAt(w, h.axis, travel);
    const DatumValue seat = datumAt(w, h.seat, travel);
    if (a.direction.norm() < kTol) {
        throw userError(fmt::format("the axis '{}' of '{}' has no direction: pick a round face of "
                                    "the chuck",
                                    h.axis, w.name));
    }
    Vec3 d = a.direction.normalized();
    if (seat.direction.dot(d) < 0.0) {
        d = -d;
    }
    return {a.point + (seat.point - a.point).dot(d) * d, d};
}

/// The radial direction from a chuck's axis to its gripping face A, and the radius gripped.
std::pair<Vec3, double> chuckRadial(const WorkholdingTemplate& w, const StockHolding& h,
                                    const std::map<std::string, double>& travel) {
    const ChuckAxis axis = chuckAxis(w, h, travel);
    Vec3 r = datumAt(w, h.faceA, travel).point - axis.point;
    r -= r.dot(axis.direction) * axis.direction;
    if (r.norm() < kTol) {
        throw userError(fmt::format("the gripping face '{}' of '{}' is on its axis: pick a jaw's "
                                    "gripping face",
                                    h.faceA, w.name));
    }
    return {r.normalized(), r.norm()};
}

/// The face whose member drives a holding's opening: a jaw holding's moving face B, a chuck's
/// gripping face.
const std::string& drivingFace(const StockHolding& h) {
    return h.kind == HoldingKind::Chuck ? h.faceA : h.faceB;
}

/// A jaw or dovetail holding's X axis: across the gap from face A to face B, square to up. A
/// chuck's: from its axis towards face A.
Vec3 holdingAxis(const WorkholdingTemplate& w, const StockHolding& h,
                 const std::map<std::string, double>& travel) {
    if (h.kind == HoldingKind::Chuck) {
        return chuckRadial(w, h, travel).first;
    }
    const Vec3 up = upOf(w);
    const DatumValue pa = datumAt(w, h.faceA, travel);
    const DatumValue pb = datumAt(w, h.faceB, travel);
    Vec3 d = pb.point - pa.point;
    d -= d.dot(up) * up;
    if (d.norm() < kTol) {
        d = pa.direction - pa.direction.dot(up) * up;
    }
    if (d.norm() < kTol) {
        throw userError(fmt::format("the faces '{}' and '{}' of '{}' have no gap across them",
                                    h.faceA, h.faceB, w.name));
    }
    return d.normalized();
}

/// Where a dovetail flank's plane crosses the seat plane, along the holding's X axis `aT`.
double flankAt(const WorkholdingTemplate& w, const StockHolding& h, const Vec3& aT,
               const std::string& flank, const std::map<std::string, double>& travel) {
    const Vec3 up = upOf(w);
    const Vec3 bT = up.cross(aT);
    const DatumValue f = datumAt(w, flank, travel);
    const double seat = datumAt(w, h.seat, travel).point.dot(up);
    const double an = aT.dot(f.direction);
    if (std::abs(an) < 1e-6) {
        throw userError(fmt::format("the flank '{}' of '{}' runs across the dovetail: pick a "
                                    "flank face",
                                    flank, w.name));
    }
    return (f.point.dot(f.direction) - f.point.dot(bT) * bT.dot(f.direction) -
            seat * up.dot(f.direction)) /
           an;
}

/// The opening of a jaw holding (between the faces along `aT`), a dovetail (between the flanks
/// at the seat) or a chuck (the diameter gripped).
double openingWith(const WorkholdingTemplate& w, const StockHolding& h, const Vec3& aT,
                   const std::map<std::string, double>& travel) {
    if (h.kind == HoldingKind::Chuck) {
        return 2.0 * chuckRadial(w, h, travel).second;
    }
    if (h.kind == HoldingKind::Dovetail) {
        return flankAt(w, h, aT, h.faceB, travel) - flankAt(w, h, aT, h.faceA, travel);
    }
    if (h.kind != HoldingKind::Jaws) {
        throw userError(fmt::format("the holding '{}' of '{}' has no opening", h.name, w.name));
    }
    return (datumAt(w, h.faceB, travel).point - datumAt(w, h.faceA, travel).point).dot(aT);
}

/// A jaw set on a chuck's master jaw 1 (DL-0069): its coordinates -> the chuck's template
/// coordinates, for master jaw 1's travel. Radii are measured from the chuck's axis along master
/// jaw 1's radial direction, heights along the axis out of the chuck.
struct JawGeometry {
    Vec3 axisPoint = Vec3::Zero();
    Vec3 axis = Vec3::UnitZ();     ///< Out of the chuck.
    Vec3 radial = Vec3::UnitX();   ///< Master jaw 1's way out from the axis.
    Vec3 seat0 = Vec3::Zero();     ///< Its seat datum at travel 0.
    Vec3 seatPerMm = Vec3::Zero(); ///< ... and how it moves per mm of travel.
    Mat3 turn = Mat3::Identity();  ///< The jaw's coordinates turned onto the master jaw.
    Vec3 jawSeat = Vec3::Zero();   ///< The jaw set's seat point, its own coordinates.

    Transform3 jawToTemplate(double travel) const {
        Transform3 xf = Transform3::Identity();
        xf.linear() = turn;
        xf.translation() = seat0 + seatPerMm * travel - turn * jawSeat;
        return xf;
    }
    double radius(const Vec3& jawPoint, double travel) const {
        return (jawToTemplate(travel) * jawPoint - axisPoint).dot(radial);
    }
    double height(const Vec3& jawPoint, double travel) const {
        return (jawToTemplate(travel) * jawPoint - axisPoint).dot(axis);
    }
    double radiusPerMm() const { return seatPerMm.dot(radial); }
};

JawGeometry jawGeometry(const WorkholdingTemplate& w, const StockHolding& h,
                        const WorkholdingTemplate& js, const JawOption& option) {
    const JawMount& mount = *w.jawMount;
    JawGeometry g;
    const ChuckAxis ax = chuckAxis(w, h, memberTravels(w, {}));
    g.axisPoint = ax.point;
    g.axis = ax.direction;
    const WorkholdingMember* m1 = w.findMember("Jaw 1");
    if (!m1) {
        throw userError(fmt::format("'{}' has no master jaw 'Jaw 1' for jaws to go on", w.name));
    }
    const Vec3 s0 = datumAt(w, mount.seat, memberTravels(w, {{"Jaw 1", 0.0}})).point;
    const Vec3 s1 = datumAt(w, mount.seat, memberTravels(w, {{"Jaw 1", 1.0}})).point;
    g.seat0 = s0;
    g.seatPerMm = s1 - s0;
    Vec3 r = s0 - ax.point;
    r -= r.dot(g.axis) * g.axis;
    if (r.norm() < kTol) {
        throw userError(fmt::format("the jaw seat '{}' of '{}' is on its axis: put it on master "
                                    "jaw 1",
                                    mount.seat, w.name));
    }
    g.radial = r.normalized();
    if (std::abs(g.radiusPerMm()) < 1e-9) {
        throw userError(
            fmt::format("master jaw 1 of '{}' does not move its jaw seat in or out", w.name));
    }
    const DatumValue seat = datumAt(js, option.seat, {});
    if (seat.direction.norm() < kTol) {
        throw userError(fmt::format("the seat '{}' of jaw set '{}' needs a direction: out of the "
                                    "chuck",
                                    option.seat, js.name));
    }
    const Vec3 u = seat.direction.normalized();
    Vec3 in = option.inward - option.inward.dot(u) * u;
    if (in.norm() < kTol) {
        throw userError(fmt::format("jaw set '{}': the way in towards the chuck's axis runs along "
                                    "its seat's direction",
                                    js.name));
    }
    in.normalize();
    Mat3 own;
    own << u, in, u.cross(in);
    const Vec3 inT = -g.radial;
    Mat3 to;
    to << g.axis, inT, g.axis.cross(inT);
    g.turn = to * own.transpose();
    g.jawSeat = seat.point;
    return g;
}

/// Whether a jaw step's gripping face faces the axis (grips an outside diameter).
bool gripsOutside(const JawGeometry& g, const WorkholdingTemplate& js, const JawStep& step) {
    const DatumValue grip = datumAt(js, step.grip, {});
    if (grip.direction.norm() < kTol) {
        throw userError(fmt::format("the gripping face '{}' of jaw set '{}' needs its normal",
                                    step.grip, js.name));
    }
    return (g.turn * grip.direction).dot(g.radial) < 0.0;
}

/// A turn of the chuck's template about its axis taking master jaw 1 to master jaw `k`.
Transform3 masterTurn(const WorkholdingTemplate& w, const JawGeometry& g, int k) {
    const WorkholdingMember* m = w.findMember(fmt::format("Jaw {}", k));
    Vec3 r = m->direction - m->direction.dot(g.axis) * g.axis;
    if (r.norm() < kTol) {
        return Transform3::Identity();
    }
    r.normalize();
    const double angle = std::atan2(g.radial.cross(r).dot(g.axis), g.radial.dot(r));
    return Eigen::Translation3d(g.axisPoint) * Eigen::AngleAxisd(angle, g.axis) *
           Eigen::Translation3d(-g.axisPoint);
}

} // namespace

const FormatSpec& workholdingFormat() {
    static const FormatSpec spec = [] {
        FormatSpec s{"oxcam.workholding", kWorkholdingVersion, {}, {"opencam.workholding"}};
        s.migrations[1] = migrateFromV1;
        s.migrations[2] = [](Json&) {
        }; // version 3 adds chucks
        // Version 4 adds jaw mounts and jaw sets: a chuck of version 3 keeps its own jaws, which
        // its setups go on using (DL-0069).
        s.migrations[3] = [](Json&) {
        };
        return s;
    }();
    return spec;
}

const SchemaValidator& workholdingSchema() {
    static const SchemaValidator validator(
        parseJson(embedded::workholdingSchemaV4, "workholding.v4.schema.json"));
    return validator;
}

Json toJson(const WorkholdingTemplate& w) {
    Json parts = Json::array();
    for (const WorkholdingPart& p : w.parts) {
        parts.push_back({{"id", p.id.toString()},
                         {"name", p.name},
                         {"sha256", p.sha256},
                         {"source_path", p.sourcePath},
                         {"member", p.member},
                         {"colour", fmt::format("#{:06X}", p.colour & 0xFFFFFFu)},
                         {"collides", p.collides},
                         {"shown", p.shown}});
        if (p.topJaw) {
            parts.back()["top_jaw"] = true;
        }
    }
    Json members = Json::array();
    for (const WorkholdingMember& m : w.members) {
        Json j{{"name", m.name},
               {"direction", vec(m.direction)},
               {"min_travel", m.minTravel},
               {"max_travel", m.maxTravel}};
        if (!m.linkedTo.empty()) {
            j["linked_to"] = m.linkedTo;
            j["ratio"] = m.ratio;
        }
        members.push_back(std::move(j));
    }
    Json datums = Json::array();
    for (const WorkholdingDatum& d : w.datums) {
        datums.push_back(datumJson(d));
    }
    Json holdings = Json::array();
    for (const StockHolding& h : w.holdings) {
        Json j{{"name", h.name}, {"kind", std::string(toString(h.kind))}, {"seat", h.seat}};
        switch (h.kind) {
        case HoldingKind::Jaws:
            j["face_a"] = h.faceA;
            j["face_b"] = h.faceB;
            j["grip_depth"] = h.gripDepth;
            break;
        case HoldingKind::Dovetail:
            j["face_a"] = h.faceA;
            j["face_b"] = h.faceB;
            j["depth"] = h.depth;
            j["tenon_width"] = h.tenonWidth;
            break;
        case HoldingKind::Seat:
            j["x_datum"] = h.xDatum;
            j["anchor"] = Json::array({h.anchor[0], h.anchor[1], h.anchor[2]});
            break;
        case HoldingKind::Chuck:
            j["axis"] = h.axis;
            j["face_a"] = h.faceA;
            j["grip_depth"] = h.gripDepth;
            j["grip_range"] = h.gripRange;
            break;
        }
        holdings.push_back(std::move(j));
    }
    Json doc{{"format", workholdingFormat().format},
             {"version", kWorkholdingVersion},
             {"units", "mm"},
             {"id", w.id.toString()},
             {"name", w.name},
             {"description", w.description},
             {"parts", std::move(parts)},
             {"members", std::move(members)},
             {"datums", std::move(datums)},
             {"holdings", std::move(holdings)},
             {"up", vec(w.up)}};
    if (!w.mount.empty()) {
        doc["mount"] = w.mount;
    }
    if (const auto& m = w.jawMount) {
        doc["jaw_mount"] = {{"interface", m->interfaceName}, {"seat", m->seat}};
    }
    if (const auto& s = w.jawSet) {
        Json options = Json::array();
        for (const JawOption& o : s->options) {
            Json steps = Json::array();
            for (const JawStep& st : o.steps) {
                steps.push_back({{"name", st.name}, {"grip", st.grip}, {"stop", st.stop}});
            }
            options.push_back({{"name", o.name},
                               {"seat", o.seat},
                               {"inward", vec(o.inward)},
                               {"steps", std::move(steps)}});
        }
        doc["jaw_set"] = {{"interface", s->interfaceName},
                          {"kind", s->kind == JawSet::Kind::Hard ? "hard" : "soft"},
                          {"options", std::move(options)}};
        if (!s->front.empty()) {
            doc["jaw_set"]["front"] = s->front;
        }
    }
    return doc;
}

WorkholdingTemplate workholdingFromJson(Json doc, std::string_view sourceName) {
    workholdingFormat().upgrade(doc, sourceName);
    workholdingSchema().validate(doc, sourceName);
    try {
        WorkholdingTemplate w;
        w.id = uuidFrom(doc.at("id"));
        w.name = doc.at("name").get<std::string>();
        w.description = doc.value("description", "");
        for (const Json& j : doc.at("parts")) {
            WorkholdingPart p;
            p.id = uuidFrom(j.at("id"));
            p.name = j.value("name", "");
            p.sha256 = j.at("sha256").get<std::string>();
            p.sourcePath = j.value("source_path", "");
            p.member = j.value("member", "");
            if (j.contains("colour")) {
                p.colour = static_cast<std::uint32_t>(
                    std::stoul(j.at("colour").get<std::string>().substr(1), nullptr, 16));
            }
            p.collides = j.value("collides", true);
            p.shown = j.value("shown", true);
            p.topJaw = j.value("top_jaw", false);
            w.parts.push_back(std::move(p));
        }
        for (const Json& j : doc.value("members", Json::array())) {
            WorkholdingMember m;
            m.name = j.at("name").get<std::string>();
            m.direction = vecFrom(j.at("direction"));
            m.minTravel = j.at("min_travel").get<double>();
            m.maxTravel = j.at("max_travel").get<double>();
            m.linkedTo = j.value("linked_to", "");
            m.ratio = j.value("ratio", 1.0);
            w.members.push_back(std::move(m));
        }
        for (const Json& j : doc.value("datums", Json::array())) {
            w.datums.push_back(datumFrom(j));
        }
        for (const Json& j : doc.at("holdings")) {
            StockHolding h;
            h.name = j.at("name").get<std::string>();
            h.kind = parseHoldingKind(j.at("kind").get<std::string>()).value();
            h.faceA = j.value("face_a", "");
            h.faceB = j.value("face_b", "");
            h.seat = j.value("seat", "");
            h.xDatum = j.value("x_datum", "");
            h.axis = j.value("axis", "");
            h.gripRange = j.value("grip_range", h.gripRange);
            h.gripDepth = j.value("grip_depth", h.gripDepth);
            h.depth = j.value("depth", h.depth);
            h.tenonWidth = j.value("tenon_width", h.tenonWidth);
            if (j.contains("anchor")) {
                for (int k = 0; k < 3; ++k) {
                    h.anchor[static_cast<std::size_t>(k)] = j.at("anchor").at(k).get<int>();
                }
            }
            w.holdings.push_back(std::move(h));
        }
        w.mount = doc.value("mount", "");
        if (doc.contains("up")) {
            w.up = vecFrom(doc.at("up"));
        }
        if (doc.contains("jaw_mount")) {
            const Json& m = doc.at("jaw_mount");
            // An earlier draft's "pitch" and "positions" are not read.
            w.jawMount =
                JawMount{m.at("interface").get<std::string>(), m.at("seat").get<std::string>()};
        }
        if (doc.contains("jaw_set")) {
            const Json& s = doc.at("jaw_set");
            JawSet set;
            set.interfaceName = s.at("interface").get<std::string>();
            set.kind =
                s.at("kind").get<std::string>() == "soft" ? JawSet::Kind::Soft : JawSet::Kind::Hard;
            set.front = s.value("front", "");
            const auto option = [](const Json& o, std::string name) {
                JawOption out;
                out.name = std::move(name);
                out.seat = o.at("seat").get<std::string>();
                out.inward = vecFrom(o.at("inward"));
                for (const Json& st : o.at("steps")) {
                    out.steps.push_back({st.at("name").get<std::string>(),
                                         st.at("grip").get<std::string>(),
                                         st.at("stop").get<std::string>()});
                }
                return out;
            };
            if (s.contains("options")) {
                for (const Json& o : s.at("options")) {
                    set.options.push_back(option(o, o.at("name").get<std::string>()));
                }
            } else {
                // An earlier draft: one way on, its seat and steps on the set.
                set.options.push_back(option(s, "Normal"));
            }
            w.jawSet = std::move(set);
        }
        return w;
    } catch (const Json::exception& e) {
        throw Error(ErrorCode::Parse, fmt::format("{}: {}", sourceName, e.what()));
    } catch (const std::bad_optional_access&) {
        throw Error(ErrorCode::Parse, fmt::format("{}: unknown enumeration value", sourceName));
    } catch (const std::logic_error& e) {
        throw Error(ErrorCode::Parse, fmt::format("{}: {}", sourceName, e.what()));
    }
}

WorkholdingPart addWorkholdingPart(WorkholdingPackage& package, std::string name,
                                   const cad::Model& model, std::string sourcePath) {
    std::string bytes = model.exportStepData();
    WorkholdingPart part;
    part.name = std::move(name);
    part.sha256 = sha256Hex(bytes);
    part.sourcePath = std::move(sourcePath);
    package.files.emplace(part.sha256, std::move(bytes));
    package.workholding.parts.push_back(part);
    return part;
}

void removeWorkholdingPart(WorkholdingPackage& package, const Uuid& part) {
    std::vector<WorkholdingPart>& parts = package.workholding.parts;
    const auto it =
        std::find_if(parts.begin(), parts.end(), [&](const auto& p) { return p.id == part; });
    if (it == parts.end()) {
        throw userError("the part to remove is not in the workholding");
    }
    const std::string sha = it->sha256;
    parts.erase(it);
    if (std::none_of(parts.begin(), parts.end(), [&](const auto& p) { return p.sha256 == sha; })) {
        package.files.erase(sha);
    }
    std::erase_if(package.workholding.datums,
                  [&](const WorkholdingDatum& d) { return d.part && *d.part == part; });
}

WorkholdingPart mergeWorkholdingParts(WorkholdingPackage& package, const std::vector<Uuid>& ids,
                                      std::string name, const PartModelFn& modelOf) {
    if (ids.size() < 2) {
        throw userError("choose two or more parts to merge");
    }
    std::vector<cad::Model> models;
    for (const Uuid& id : ids) {
        const WorkholdingPart* p = package.workholding.findPart(id);
        if (!p) {
            throw userError("a part to merge is not in the workholding");
        }
        const cad::Model* m = modelOf(*p);
        if (!m) {
            throw userError(fmt::format("the part '{}' is not loaded", p->name));
        }
        models.push_back(*m);
    }
    const WorkholdingPart first = *package.workholding.findPart(ids.front());
    WorkholdingPart merged =
        addWorkholdingPart(package, std::move(name), cad::compound(models), first.sourcePath);
    merged.member = first.member;
    merged.colour = first.colour;
    merged.collides = first.collides;
    merged.shown = first.shown;
    // In the first part's place.
    std::vector<WorkholdingPart>& parts = package.workholding.parts;
    parts.pop_back();
    const auto at =
        std::find_if(parts.begin(), parts.end(), [&](const auto& p) { return p.id == first.id; });
    parts.insert(at, merged);
    for (const Uuid& id : ids) {
        removeWorkholdingPart(package, id);
    }
    return merged;
}

std::vector<SolidPart> solidPartsFrom(const std::vector<ImportedPart>& parts,
                                      const std::vector<std::string>& groups) {
    if (groups.size() != parts.size()) {
        throw Error(ErrorCode::InvalidArgument, "one group per part");
    }
    std::vector<std::string> order;
    std::map<std::string, std::vector<cad::Model>> models;
    std::map<std::string, std::optional<std::uint32_t>> colours;
    for (std::size_t i = 0; i < parts.size(); ++i) {
        const std::string& group = groups[i];
        if (group.empty()) {
            continue;
        }
        const ImportedPart& p = parts[i];
        if (p.mesh || p.model.isNull()) {
            throw Error(
                ErrorCode::InvalidArgument,
                fmt::format("'{}' is a mesh: workholding parts come from STEP files", p.name));
        }
        auto [it, added] = models.try_emplace(group);
        if (added) {
            order.push_back(group);
        }
        it->second.push_back(p.model);
        if (!colours[group] && p.colour) {
            colours[group] = p.colour;
        }
    }
    std::vector<SolidPart> out;
    for (const std::string& group : order) {
        const auto& m = models[group];
        out.push_back({group, m.size() == 1 ? m.front() : cad::compound(m), colours[group]});
    }
    return out;
}

WorkholdingDatum pickedDatum(std::string name, const WorkholdingPart& part, const cad::Model& model,
                             cad::SubShapeType type, int index, const std::optional<Vec3>& onFace) {
    const cad::DatumGeometry g = onFace && type == cad::SubShapeType::Face
                                     ? cad::pointOnFace(model, index, *onFace)
                                     : cad::datumGeometry(model, type, index);
    WorkholdingDatum d;
    d.name = std::move(name);
    d.kind = DatumKind::Picked;
    d.part = part.id;
    d.type = type;
    d.index = index;
    d.signature = cad::signatureOf(model, type, index);
    d.shape = g.shape == cad::DatumGeometry::Shape::Plane  ? DatumShape::Plane
              : g.shape == cad::DatumGeometry::Shape::Axis ? DatumShape::Axis
              : g.shape == cad::DatumGeometry::Shape::Line ? DatumShape::Line
                                                           : DatumShape::Point;
    d.point = g.point;
    d.direction = g.direction;
    return d;
}

std::string writeWorkholdingArchive(const WorkholdingPackage& package) {
    ZipWriter w;
    if (!mz_zip_writer_init_heap(&w.zip, 0, 64 * 1024)) {
        throw Error(ErrorCode::Internal, "cannot initialize zip writer");
    }
    auto add = [&](const std::string& name, const std::string& data) {
        if (!mz_zip_writer_add_mem(&w.zip, name.c_str(), data.data(), data.size(),
                                   MZ_DEFAULT_LEVEL)) {
            throw Error(ErrorCode::Internal, "cannot add zip entry " + name);
        }
    };
    add("workholding.json", dumpJson(toJson(package.workholding)));
    std::set<std::string> written;
    for (const WorkholdingPart& p : package.workholding.parts) {
        const auto it = package.files.find(p.sha256);
        if (it == package.files.end()) {
            throw Error(ErrorCode::InvalidArgument,
                        fmt::format("the file of part '{}' is missing", p.name));
        }
        if (written.insert(p.sha256).second) {
            add("bodies/" + p.sha256 + ".step", it->second);
        }
    }
    void* buffer = nullptr;
    size_t size = 0;
    if (!mz_zip_writer_finalize_heap_archive(&w.zip, &buffer, &size)) {
        throw Error(ErrorCode::Internal, "cannot finalize zip archive");
    }
    std::unique_ptr<void, decltype(&mz_free)> owned(buffer, &mz_free);
    return {static_cast<const char*>(buffer), size};
}

WorkholdingPackage readWorkholdingArchive(std::string_view bytes, std::string_view sourceName) {
    ZipReader r;
    if (!mz_zip_reader_init_mem(&r.zip, bytes.data(), bytes.size(), 0)) {
        throw Error(ErrorCode::Parse, fmt::format("{}: not a workholding file", sourceName));
    }
    static const std::regex bodyName("^bodies/([0-9a-f]{64})\\.step$");
    std::string json;
    std::map<std::string, std::string> files;
    const mz_uint count = mz_zip_reader_get_num_files(&r.zip);
    for (mz_uint i = 0; i < count; ++i) {
        mz_zip_archive_file_stat stat{};
        if (!mz_zip_reader_file_stat(&r.zip, i, &stat) || stat.m_is_directory) {
            continue;
        }
        const std::string name = stat.m_filename;
        std::smatch m;
        const bool isJson = name == "workholding.json";
        const bool isBody = std::regex_match(name, m, bodyName);
        if (!isJson && !isBody) {
            continue;
        }
        if (stat.m_uncomp_size > kMaxEntryBytes) {
            throw Error(ErrorCode::Parse,
                        fmt::format("{}: entry {} is too large", sourceName, name));
        }
        size_t size = 0;
        void* data = mz_zip_reader_extract_to_heap(&r.zip, i, &size, 0);
        if (!data) {
            throw Error(ErrorCode::Parse, fmt::format("{}: cannot extract {}", sourceName, name));
        }
        std::unique_ptr<void, decltype(&mz_free)> owned(data, &mz_free);
        std::string content(static_cast<const char*>(data), size);
        if (isJson) {
            json = std::move(content);
        } else {
            const std::string sha = m[1].str();
            if (sha256Hex(content) != sha) {
                throw Error(
                    ErrorCode::Parse,
                    fmt::format("{}: part file {} is corrupt (hash mismatch)", sourceName, name));
            }
            files.emplace(sha, std::move(content));
        }
    }
    if (json.empty()) {
        throw Error(ErrorCode::Parse, fmt::format("{}: no workholding.json", sourceName));
    }
    WorkholdingPackage out;
    out.workholding = workholdingFromJson(
        parseJson(json, fmt::format("{}/workholding.json", sourceName)), sourceName);
    for (const WorkholdingPart& p : out.workholding.parts) {
        const auto it = files.find(p.sha256);
        if (it == files.end()) {
            throw Error(ErrorCode::Parse,
                        fmt::format("{}: the file of part '{}' is missing", sourceName, p.name));
        }
        out.files.emplace(p.sha256, it->second);
    }
    return out;
}

void saveWorkholdingFile(const std::filesystem::path& path, const WorkholdingPackage& package) {
    writeFileAtomic(path, writeWorkholdingArchive(package));
}

WorkholdingPackage loadWorkholdingFile(const std::filesystem::path& path) {
    const std::u8string u8 = path.filename().u8string();
    return readWorkholdingArchive(readFileBytes(path), std::string(u8.begin(), u8.end()));
}

std::map<std::string, double> memberTravels(const WorkholdingTemplate& w,
                                            const std::map<std::string, double>& own) {
    std::map<std::string, double> out;
    for (const WorkholdingMember& m : w.members) {
        if (m.linkedTo.empty()) {
            const auto it = own.find(m.name);
            out[m.name] = it == own.end() ? 0.0 : it->second;
        }
    }
    for (const WorkholdingMember& m : w.members) {
        if (!m.linkedTo.empty()) {
            const auto it = out.find(m.linkedTo);
            out[m.name] = m.ratio * (it == out.end() ? 0.0 : it->second);
        }
    }
    return out;
}

DatumValue datumAt(const WorkholdingTemplate& w, std::string_view name,
                   const std::map<std::string, double>& travel) {
    return datumValue(w, name, travel, 0);
}

double holdingOpening(const WorkholdingTemplate& w, const StockHolding& holding,
                      const std::map<std::string, double>& travel) {
    return openingWith(w, holding, holdingAxis(w, holding, travel), travel);
}

std::map<std::string, double> travelsForOpening(const WorkholdingTemplate& w,
                                                const StockHolding& holding, double opening,
                                                const std::map<std::string, double>& own) {
    const Vec3 aT = holdingAxis(w, holding, memberTravels(w, own));
    return closeOn(
        w, own, opening, datumMember(w, drivingFace(holding)),
        [&](const std::map<std::string, double>& t) { return openingWith(w, holding, aT, t); },
        holding.kind == HoldingKind::Dovetail ? "the flanks" : "the jaws");
}

namespace {

/// The driving member of a holding's opening, the travel range it can use (its own and its
/// followers', each within its range) and the opening with every member at `own`.
struct OpeningDrive {
    Driver driver;
    double start = 0.0; ///< The driver's travel in `own`.
    double now = 0.0;   ///< The opening at `own`.
    double lo = 0.0;    ///< The driver's usable travel.
    double hi = 0.0;
};
std::optional<OpeningDrive> openingDrive(const WorkholdingTemplate& w, const StockHolding& h,
                                         const std::map<std::string, double>& own) {
    const Vec3 aT = holdingAxis(w, h, memberTravels(w, own));
    const OpeningFn opening = [&](const std::map<std::string, double>& t) {
        return openingWith(w, h, aT, t);
    };
    const auto d = driverOf(w, own, datumMember(w, drivingFace(h)), opening);
    if (!d) {
        return std::nullopt;
    }
    const WorkholdingMember* m = w.findMember(d->member);
    OpeningDrive out{*d, own.contains(d->member) ? own.at(d->member) : 0.0,
                     opening(memberTravels(w, own)), m->minTravel, m->maxTravel};
    for (const WorkholdingMember& f : w.members) {
        if (f.linkedTo == d->member && std::abs(f.ratio) > 1e-12) {
            const double a = f.minTravel / f.ratio;
            const double b = f.maxTravel / f.ratio;
            out.lo = std::max(out.lo, std::min(a, b));
            out.hi = std::min(out.hi, std::max(a, b));
        }
    }
    return out;
}

} // namespace

std::optional<std::pair<double, double>> openingRange(const WorkholdingTemplate& w,
                                                      const StockHolding& holding,
                                                      const std::map<std::string, double>& own) {
    const auto d = openingDrive(w, holding, own);
    if (!d || d->hi < d->lo) {
        return std::nullopt;
    }
    const double a = d->now + (d->lo - d->start) * d->driver.slope;
    const double b = d->now + (d->hi - d->start) * d->driver.slope;
    return std::pair{std::min(a, b), std::max(a, b)};
}

void setOpeningRange(WorkholdingTemplate& w, const StockHolding& holding, double smallest,
                     double largest) {
    const auto d = openingDrive(w, holding, {});
    if (!d) {
        throw userError(fmt::format("no member of '{}' moves the faces of '{}': put the moving "
                                    "jaw's face on a moving part",
                                    w.name, holding.name));
    }
    const double t0 = (smallest - d->now) / d->driver.slope;
    const double t1 = (largest - d->now) / d->driver.slope;
    for (WorkholdingMember& m : w.members) {
        const double r = m.name == d->driver.member       ? 1.0
                         : m.linkedTo == d->driver.member ? m.ratio
                                                          : 0.0;
        if (r != 0.0) {
            m.minTravel = std::min(r * t0, r * t1);
            m.maxTravel = std::max(r * t0, r * t1);
        }
    }
}

std::vector<std::string> missingAnswers(const WorkholdingAnswers& a) {
    std::vector<std::string> out;
    const bool centring = a.type == FixtureType::SelfCentringVise;
    switch (a.type) {
    case FixtureType::Vise:
    case FixtureType::SelfCentringVise:
        if (centring && a.opposite.empty()) {
            out.emplace_back("click the parts of the first jaw");
        }
        if (a.moving.empty()) {
            out.emplace_back(centring ? "click the parts of the second jaw"
                                      : "click the parts of the moving jaw");
        }
        if (!a.faceA) {
            out.emplace_back(centring ? "click the first jaw's clamping face"
                                      : "click the fixed jaw's clamping face");
        }
        if (!a.faceB) {
            out.emplace_back(centring ? "click the second jaw's clamping face"
                                      : "click the moving jaw's clamping face");
        }
        if (!a.top) {
            out.emplace_back("click the top of the jaws");
        }
        break;
    case FixtureType::Dovetail:
        if (!a.faceA) {
            out.emplace_back("click the fixed flank");
        }
        if (!a.faceB) {
            out.emplace_back("click the other flank");
        }
        if (!a.top) {
            out.emplace_back("click the seat the stock's shoulder sits on");
        }
        break;
    case FixtureType::Seat:
        if (!a.top) {
            out.emplace_back("click the face the stock sits on");
        }
        break;
    case FixtureType::Chuck:
    case FixtureType::Collet:
        if (a.type == FixtureType::Chuck && a.moving.empty()) {
            out.emplace_back("click the parts of every jaw");
        }
        if (!a.axis) {
            out.emplace_back("click a round face of the chuck on the spindle axis");
        }
        if (!a.faceA) {
            out.emplace_back(a.type == FixtureType::Chuck ? "click a jaw's gripping face"
                                                          : "click the collet's bore");
        }
        if (!a.top) {
            out.emplace_back(a.type == FixtureType::Chuck
                                 ? "click the front of the jaws (or the step the bar sits against)"
                                 : "click the front of the collet");
        }
        if (a.gripDepth < 0.0 || a.gripRange < 0.0) {
            out.emplace_back("the grip length and range must not be negative");
        }
        break;
    }
    if (a.type != FixtureType::Seat && a.type != FixtureType::Collet &&
        !(a.type == FixtureType::Dovetail && a.moving.empty()) && !(a.maxOpening > a.minOpening)) {
        out.emplace_back("the largest opening must be larger than the smallest");
    }
    return out;
}

namespace {

/// A chuck or collet from the wizard's answers (buildWorkholding): its jaws grouped by their
/// angle about the axis, each a member moving radially, all linked to the one with the gripping
/// face; datums on the axis, the gripping face (a point on it at the radius gripped) and the
/// front; the chuck holding.
template <typename PickFn>
void buildChuck(WorkholdingTemplate& w, const WorkholdingAnswers& a,
                std::vector<WorkholdingPart>& parts, const PickFn& pick,
                const PartModelFn& modelOf) {
    const bool collet = a.type == FixtureType::Collet;
    auto partOf = [&](const Uuid& id) -> WorkholdingPart& {
        for (WorkholdingPart& p : parts) {
            if (p.id == id) {
                return p;
            }
        }
        throw userError("a part picked for the workholding is not among its parts");
    };
    const WorkholdingDatum axis = pick(*a.axis, "Spindle axis");
    const WorkholdingDatum front = pick(*a.top, collet ? "Collet front" : "Jaw front");
    if (axis.direction.norm() < kTol) {
        throw userError("the spindle axis needs a round face (or the chuck's flat face)");
    }
    Vec3 d = axis.direction.normalized();
    if (front.direction.dot(d) < 0.0) {
        d = -d;
    }
    if (std::abs(front.direction.dot(d)) < 0.9) {
        throw userError(collet ? "the front of the collet must face along the spindle axis"
                               : "the front of the jaws must face along the spindle axis");
    }
    w.up = d;
    auto radial = [&](const Vec3& p) {
        const Vec3 r = p - axis.point;
        return Vec3(r - r.dot(d) * d);
    };
    // The gripping face: a point on it at the radius gripped (a round face's datum is its axis).
    WorkholdingPick grip = *a.faceA;
    if (!grip.onFace && grip.type == cad::SubShapeType::Face) {
        if (const cad::Model* m = modelOf(partOf(grip.part))) {
            const cad::SubShapeGeometry g = cad::subShapeGeometry(*m, grip.type, grip.index);
            if (g.kind == cad::SubShapeGeometry::Kind::Cylinder) {
                const Vec3 c = cad::signatureOf(*m, grip.type, grip.index).centroid;
                const Vec3 gd = g.direction.normalized();
                Vec3 out = c - g.point;
                out -= out.dot(gd) * gd;
                if (out.norm() < kTol) {
                    out = gd.unitOrthogonal(); // a full bore: any side
                }
                grip.onFace = g.point + (c - g.point).dot(gd) * gd + g.radius * out.normalized();
            }
        }
    }
    const WorkholdingDatum face = pick(grip, "Gripping face");
    if (radial(face.point).norm() < kTol) {
        throw userError(collet ? "the collet's bore is on the spindle axis: pick the bore's face"
                               : "the gripping face is on the spindle axis: pick a jaw's face");
    }
    if (!collet) {
        // Jaws: parts at the same angle about the axis move together.
        const Vec3 e1 = d.unitOrthogonal();
        const Vec3 e2 = d.cross(e1);
        struct Jaw {
            double angle = 0.0;
            Vec3 direction = Vec3::Zero();
            std::vector<Uuid> parts;
        };
        std::vector<Jaw> jaws;
        for (const Uuid& id : a.moving) {
            const WorkholdingPart& part = partOf(id);
            const cad::Model* m = modelOf(part);
            if (!m) {
                throw userError(fmt::format("the part '{}' is not loaded", part.name));
            }
            const Vec3 r = radial(m->info().bounds.center());
            if (r.norm() < kTol) {
                throw userError(fmt::format("the part '{}' sits on the spindle axis: it cannot be "
                                            "a jaw",
                                            part.name));
            }
            const double angle = std::atan2(r.dot(e2), r.dot(e1));
            auto same = std::find_if(jaws.begin(), jaws.end(), [&](const Jaw& j) {
                return std::abs(std::remainder(j.angle - angle, 2.0 * kPi)) < degToRad(10.0);
            });
            if (same == jaws.end()) {
                jaws.push_back({angle, r.normalized(), {id}});
            } else {
                same->parts.push_back(id);
            }
        }
        if (jaws.size() < 2) {
            throw userError("click the parts of every jaw: they all sit at one angle about the "
                            "axis");
        }
        // The jaw with the gripping face drives; the others follow it.
        const auto driver = std::find_if(jaws.begin(), jaws.end(), [&](const Jaw& j) {
            return std::find(j.parts.begin(), j.parts.end(), grip.part) != j.parts.end();
        });
        if (driver == jaws.end()) {
            throw userError("the gripping face must be on one of the jaws");
        }
        std::rotate(jaws.begin(), driver, driver + 1);
        for (std::size_t i = 0; i < jaws.size(); ++i) {
            const std::string name = fmt::format("Jaw {}", i + 1);
            w.members.push_back({name, jaws[i].direction, 0.0, 1.0, i == 0 ? "" : "Jaw 1", 1.0});
            for (const Uuid& id : jaws[i].parts) {
                partOf(id).member = name;
            }
        }
        w.parts = parts;
    }
    StockHolding h;
    h.name = collet ? "Collet" : "Chuck";
    h.kind = HoldingKind::Chuck;
    h.axis = axis.name;
    h.faceA = face.name;
    h.seat = front.name;
    h.gripDepth = a.gripDepth;
    h.gripRange = collet ? a.gripRange : 0.0;
    w.holdings.push_back(h);
    if (!w.members.empty()) {
        setOpeningRange(w, w.holdings.front(), a.minOpening, a.maxOpening);
    }
}

} // namespace

WorkholdingTemplate buildWorkholding(const WorkholdingAnswers& a,
                                     std::vector<WorkholdingPart> parts,
                                     const PartModelFn& modelOf) {
    if (const auto missing = missingAnswers(a); !missing.empty()) {
        throw userError("the workholding is not set up yet: " + missing.front());
    }
    WorkholdingTemplate w;
    w.name = a.name.empty() ? std::string("Workholding") : a.name;
    w.description = a.description;
    auto partIndex = [&](const Uuid& id) {
        for (std::size_t i = 0; i < parts.size(); ++i) {
            if (parts[i].id == id) {
                return i;
            }
        }
        throw userError("a part picked for the workholding is not among its parts");
    };
    const char* kMoving = "Moving jaw";
    const char* kOpposite = "Opposite jaw";
    for (WorkholdingPart& p : parts) {
        p.member.clear();
    }
    for (const Uuid& id : a.moving) {
        // A chuck's jaws get their members as they are grouped (buildChuck).
        if (a.type != FixtureType::Chuck) {
            parts[partIndex(id)].member = kMoving;
        }
    }
    for (const Uuid& id : a.opposite) {
        parts[partIndex(id)].member = kOpposite;
    }
    w.parts = parts;
    auto pick = [&](const WorkholdingPick& p, const char* name) {
        const WorkholdingPart& part = parts[partIndex(p.part)];
        const cad::Model* model = modelOf(part);
        if (!model) {
            throw userError(fmt::format("the part '{}' is not loaded", part.name));
        }
        WorkholdingDatum d = pickedDatum(name, part, *model, p.type, p.index, p.onFace);
        w.datums.push_back(d);
        return d;
    };
    const bool centring = a.type == FixtureType::SelfCentringVise;
    const bool dovetail = a.type == FixtureType::Dovetail;
    if (a.type == FixtureType::Chuck || a.type == FixtureType::Collet) {
        buildChuck(w, a, parts, pick, modelOf);
    } else if (a.type == FixtureType::Seat) {
        const WorkholdingDatum seat = pick(*a.top, "Seat");
        w.up =
            seat.direction.norm() > kTol ? Vec3(seat.direction.normalized()) : Vec3(Vec3::UnitZ());
        StockHolding h;
        h.name = "Seat";
        h.kind = HoldingKind::Seat;
        h.seat = "Seat";
        if (a.xEdge) {
            pick(*a.xEdge, "X edge");
            h.xDatum = "X edge";
        }
        h.anchor = a.anchor;
        w.holdings.push_back(h);
    } else {
        const char* nameA = dovetail   ? "Fixed flank"
                            : centring ? "Opposite jaw face"
                                       : "Fixed jaw face";
        const char* nameB = dovetail ? "Moving flank" : "Moving jaw face";
        const WorkholdingDatum fa = pick(*a.faceA, nameA);
        const WorkholdingDatum fb = pick(*a.faceB, nameB);
        const WorkholdingDatum top = pick(*a.top, dovetail ? "Seat" : "Jaw tops");
        w.up = top.direction.norm() > kTol ? Vec3(top.direction.normalized()) : Vec3(Vec3::UnitZ());
        Vec3 across = fb.point - fa.point;
        across -= across.dot(w.up) * w.up;
        if (across.norm() < kTol) {
            throw userError("the two clamping faces have no gap between them across the jaws");
        }
        across.normalize();
        if (!a.moving.empty()) {
            w.members.push_back({kMoving, across, 0.0, 1.0, {}, 1.0});
        }
        if (centring) {
            w.members.push_back({kOpposite, Vec3(-across), 0.0, 1.0, kMoving, 1.0});
        }
        StockHolding h;
        h.name = dovetail ? "Dovetail" : "Jaws";
        h.kind = dovetail ? HoldingKind::Dovetail : HoldingKind::Jaws;
        h.faceA = nameA;
        h.faceB = nameB;
        h.seat = dovetail ? "Seat" : "Jaw tops";
        h.gripDepth = a.gripDepth;
        h.depth = a.tenonDepth;
        w.holdings.push_back(h);
        if (!w.members.empty()) {
            setOpeningRange(w, w.holdings.front(), a.minOpening, a.maxOpening);
        }
    }
    if (a.mount) {
        pick(*a.mount, "Mount");
    } else {
        // The bottom centre of all the parts.
        geom3d::Aabb3 all;
        for (const WorkholdingPart& p : parts) {
            if (const cad::Model* m = modelOf(p)) {
                all.extend(m->info().bounds);
            }
        }
        WorkholdingDatum bottom;
        bottom.name = "Mount";
        bottom.kind = DatumKind::Typed;
        bottom.shape = DatumShape::Point;
        const Vec3 c = all.isEmpty() ? Vec3(Vec3::Zero()) : all.center();
        const double low = all.isEmpty() ? 0.0 : extent(all, w.up).first;
        bottom.point = c - (c.dot(w.up) - low) * w.up;
        bottom.direction = -w.up;
        w.datums.push_back(bottom);
    }
    w.mount = "Mount";
    if (const auto issues = w.validate(); !issues.empty()) {
        throw userError(issues.front());
    }
    return w;
}

std::string reachMessage(const WorkholdingReach& r, LengthUnit unit) {
    const char* suffix = unit == LengthUnit::Inch ? "in" : "mm";
    const auto length = [&](double mm) {
        return fmt::format("{:.3f}", fromMm(mm, unit));
    };
    const std::string on = r.step.empty() ? std::string() : fmt::format("on step '{}' ", r.step);
    const std::string range = fmt::format("{}\u2013{} {}", length(std::min(r.from, r.to)),
                                          length(std::max(r.from, r.to)), suffix);
    switch (r.kind) {
    case WorkholdingReach::Kind::Grip:
        return fmt::format("the jaws can't reach \u00d8{} {}: {}they grip \u00d8{}",
                           length(r.wanted), suffix, on, range);
    case WorkholdingReach::Kind::Clamp:
        return fmt::format("the jaws can't clamp \u00d8{} {} to be bored: {}they grip \u00d8{}",
                           length(r.wanted), suffix, on, range);
    case WorkholdingReach::Kind::Collet:
        return fmt::format("the collet can't hold \u00d8{} {}: it holds \u00d8{}", length(r.wanted),
                           suffix, range);
    case WorkholdingReach::Kind::Open:
        return fmt::format("the jaws can't reach {} {}: they open {}", length(r.wanted), suffix,
                           range);
    case WorkholdingReach::Kind::Tenon:
        return fmt::format("the flanks can't hold a {} {} tenon: they open {}", length(r.wanted),
                           suffix, range);
    }
    return {};
}

WorkholdingPose poseWorkholding(const WorkholdingTemplate& w, const SetupWorkholding& placement,
                                const geom3d::Aabb3& stockWcs, const WorkholdingTemplate* jawSet,
                                std::optional<double> insideDiameter, bool asFarAsItGoes) {
    if (stockWcs.isEmpty()) {
        throw userError("the workholding needs the setup's stock");
    }
    if (w.holdings.empty()) {
        throw userError(fmt::format("workholding '{}' has no stock holding", w.name));
    }
    const StockHolding* holding =
        placement.holding.empty() ? &w.holdings.front() : w.findHolding(placement.holding);
    if (!holding) {
        throw userError(
            fmt::format("workholding '{}' has no holding '{}'", w.name, placement.holding));
    }
    // The stock's axes in the WCS: X turned about Z (exact for the four allowed turns).
    Vec3 aW;
    switch (placement.rotation) {
    case 0:
        aW = Vec3::UnitX();
        break;
    case 90:
        aW = Vec3::UnitY();
        break;
    case 180:
        aW = -Vec3::UnitX();
        break;
    case 270:
        aW = -Vec3::UnitY();
        break;
    default:
        throw userError("the workholding turns by 0, 90, 180 or 270 degrees");
    }
    const Vec3 bW = Vec3::UnitZ().cross(aW);
    const auto [a0, a1] = extent(stockWcs, aW);
    const auto [b0, b1] = extent(stockWcs, bW);
    const double z0 = stockWcs.min.z();
    const double z1 = stockWcs.max.z();

    WorkholdingPose out;
    out.holding = holding->name;
    out.stockWidth = a1 - a0;
    if (w.up.norm() < 1e-9) {
        throw userError(fmt::format("workholding '{}' has no up direction", w.name));
    }
    const Vec3 up = w.up.normalized();
    const std::map<std::string, double>& own = placement.travel;
    std::map<std::string, double> travel = memberTravels(w, own);
    Vec3 aT = Vec3::UnitX();
    Vec3 zT = up;
    Vec3 anchorT = Vec3::Zero();
    Vec3 anchorW = Vec3::Zero();

    // Typed travel (a seat's members, a collet's): kept in range by the setup page.
    auto checkRanges = [&](const std::string& what) {
        for (const WorkholdingMember& m : w.members) {
            const double t = travel.contains(m.name) ? travel.at(m.name) : 0.0;
            if (t < m.minTravel - kTol || t > m.maxTravel + kTol) {
                throw userError(fmt::format("{} needs '{}' of '{}' to travel {:.3f} mm, outside "
                                            "its range of {:.3f} to {:.3f} mm",
                                            what, m.name, w.name, t, m.minTravel, m.maxTravel));
            }
        }
    };
    // Members closing on the stock: those that cannot reach it stop at the end of their travel
    // nearest it, and what it asks is said (refused unless posed as far as it goes).
    std::set<std::string> stopped;
    auto reach = [&](const std::function<WorkholdingReach()>& asked) {
        bool short_ = false;
        for (const WorkholdingMember& m : w.members) {
            const auto it = travel.find(m.name);
            const double t = it == travel.end() ? 0.0 : it->second;
            if (t < m.minTravel - kTol || t > m.maxTravel + kTol) {
                travel[m.name] = std::clamp(t, m.minTravel, m.maxTravel);
                stopped.insert(m.name);
                short_ = true;
            }
        }
        if (short_ && !out.outOfReach) {
            out.outOfReach = asked();
            if (!asFarAsItGoes) {
                throw userError(reachMessage(*out.outOfReach, LengthUnit::Millimeter));
            }
        }
        return short_;
    };
    const auto openingReach = [&](WorkholdingReach::Kind kind, double wanted) {
        return [&w, holding, &own, kind, wanted] {
            const auto range = openingRange(w, *holding, own);
            return WorkholdingReach{
                kind, wanted, range ? range->first : 0.0, range ? range->second : 0.0, {}};
        };
    };

    switch (holding->kind) {
    case HoldingKind::Jaws: {
        aT = holdingAxis(w, *holding, travel);
        out.opening = placement.jaws == JawSetting::OnStock ? out.stockWidth : placement.opening;
        travel = travelsForOpening(w, *holding, out.opening, own);
        if (reach(openingReach(WorkholdingReach::Kind::Open, out.opening))) {
            out.opening =
                std::clamp(out.opening, std::min(out.outOfReach->from, out.outOfReach->to),
                           std::max(out.outOfReach->from, out.outOfReach->to));
        } else if (out.opening < out.stockWidth - kTol) {
            out.warnings.push_back(fmt::format("the jaws of '{}' are {:.3f} mm apart, closer "
                                               "than the stock is wide ({:.3f} mm)",
                                               w.name, out.opening, out.stockWidth));
        }
        const Vec3 bT = up.cross(aT);
        const Vec3 pa = datumAt(w, holding->faceA, travel).point;
        const double grip = placement.gripDepth.value_or(holding->gripDepth);
        const double top = datumAt(w, holding->seat, travel).point.dot(up);
        anchorT = pa.dot(aT) * aT + (pa.dot(bT) + placement.shift) * bT + (top - grip) * up;
        anchorW = a0 * aW + 0.5 * (b0 + b1) * bW + z0 * Vec3::UnitZ();
        break;
    }
    case HoldingKind::Dovetail: {
        aT = holdingAxis(w, *holding, travel);
        const Vec3 bT = up.cross(aT);
        out.opening = placement.jaws == JawSetting::Opening ? placement.opening
                      : holding->tenonWidth > 0.0           ? holding->tenonWidth
                                                            : openingWith(w, *holding, aT, travel);
        travel = travelsForOpening(w, *holding, out.opening, own);
        reach(openingReach(WorkholdingReach::Kind::Tenon, out.opening));
        const double centre = 0.5 * (flankAt(w, *holding, aT, holding->faceA, travel) +
                                     flankAt(w, *holding, aT, holding->faceB, travel));
        const Vec3 pa = datumAt(w, holding->faceA, travel).point;
        const double seat = datumAt(w, holding->seat, travel).point.dot(up);
        anchorT = centre * aT + (pa.dot(bT) + placement.shift) * bT + (seat - holding->depth) * up;
        anchorW = 0.5 * (a0 + a1) * aW + 0.5 * (b0 + b1) * bW + z0 * Vec3::UnitZ();
        break;
    }
    case HoldingKind::Seat: {
        checkRanges("the setup's member travel");
        const DatumValue seat = datumAt(w, holding->seat, travel);
        zT = seat.direction.norm() > kTol ? seat.direction : up;
        Vec3 x = holding->xDatum.empty() ? Vec3(Vec3::UnitX())
                                         : datumAt(w, holding->xDatum, travel).direction;
        x -= x.dot(zT) * zT;
        if (x.norm() < kTol) {
            x = Vec3::UnitY() - Vec3::UnitY().dot(zT) * zT;
        }
        aT = x.normalized();
        anchorT = seat.point;
        auto pick = [](int i, double lo, double hi) {
            return i == 0 ? lo : i == 1 ? 0.5 * (lo + hi) : hi;
        };
        anchorW = pick(holding->anchor[0], a0, a1) * aW + pick(holding->anchor[1], b0, b1) * bW +
                  pick(holding->anchor[2], z0, z1) * Vec3::UnitZ();
        break;
    }
    case HoldingKind::Chuck: {
        if (placement.jawSet) {
            // A jaw set on the master jaws (DL-0069).
            if (!jawSet || !jawSet->jawSet) {
                throw userError("the setup's jaw set is not in the project");
            }
            if (!w.jawMount) {
                throw userError(fmt::format("'{}' has no jaw mount for jaw sets to go on", w.name));
            }
            if (!jawSetFits(w, *jawSet)) {
                out.warnings.push_back(fmt::format("jaw set '{}' is for '{}', and '{}' takes '{}'",
                                                   jawSet->name, jawSet->jawSet->interfaceName,
                                                   w.name, w.jawMount->interfaceName));
            }
            const JawSet& set = *jawSet->jawSet;
            const JawOption* option = set.findOption(placement.jawOption);
            if (!option) {
                throw userError(fmt::format("jaw set '{}' has no option '{}'", jawSet->name,
                                            placement.jawOption));
            }
            const bool outside = placement.grip == GripSide::Outside;
            const JawGeometry g = jawGeometry(w, *holding, *jawSet, *option);
            const bool soft = set.kind == JawSet::Kind::Soft;
            double held = 0.0;
            if (placement.jaws == JawSetting::Opening) {
                held = placement.opening;
            } else if (outside) {
                held = out.stockWidth;
            } else if (insideDiameter) {
                held = *insideDiameter;
            } else {
                throw userError("an inside grip needs the bore it grips: the stock has none where "
                                "it is held, so set the jaws' opening");
            }
            const WorkholdingMember& m1 = *w.findMember("Jaw 1");
            // Whether a step's face reaches the held diameter within master jaw 1's travel.
            const auto reaches = [&](const JawStep& st) {
                const double t = (0.5 * held - g.radius(datumAt(*jawSet, st.grip, {}).point, 0.0)) /
                                 g.radiusPerMm();
                return t >= m1.minTravel - kTol && t <= m1.maxTravel + kTol;
            };
            // The step: the setup's, else (Auto) the first that grips this way and reaches the
            // stock, else the first that grips this way; soft jaws: the first.
            const JawStep* step = nullptr;
            for (const JawStep& st : option->steps) {
                if (!placement.step.empty()) {
                    if (st.name == placement.step) {
                        step = &st;
                        break;
                    }
                } else if (soft) {
                    step = &st;
                    break;
                } else if (gripsOutside(g, *jawSet, st) == outside) {
                    if (!step) {
                        step = &st;
                    }
                    if (reaches(st)) {
                        step = &st;
                        break;
                    }
                }
            }
            const std::string entry = set.options.size() > 1
                                          ? fmt::format("{} ({})", jawSet->name, option->name)
                                          : jawSet->name;
            const char* way = outside ? "an outside diameter (OD)" : "an inside diameter (ID)";
            std::string refused;
            if (!step) {
                refused = placement.step.empty()
                              ? fmt::format("'{}' has no step that grips {}: choose another entry "
                                            "in Jaw set, or the other Part gripping",
                                            entry, way)
                              : fmt::format("'{}' has no step '{}'", entry, placement.step);
            } else if (!soft && gripsOutside(g, *jawSet, *step) != outside) {
                refused = fmt::format("step '{}' of '{}' does not grip {}: choose another step",
                                      step->name, entry, way);
            }
            if (!refused.empty()) {
                if (!asFarAsItGoes) {
                    throw userError(refused);
                }
                // Drawn where they stand, by their first step, so what is wrong can be seen.
                out.refusal = refused;
                step = &option->steps.front();
            }
            out.opening = held;
            JawPlacement jp;
            jp.jawSet = jawSet->name;
            jp.option = option->name;
            jp.step = step->name;
            jp.outside = outside;
            jp.gripDiameter = held;
            jp.minTravel = m1.minTravel;
            jp.maxTravel = m1.maxTravel;
            const Vec3 gripPoint = datumAt(*jawSet, step->grip, {}).point;
            const double perMm = g.radiusPerMm();
            jp.radiusPerMm = perMm;
            std::optional<JawCut> cut;
            double stopHeight = 0.0;
            // The diameters a step reaches over master jaw 1's travel.
            const auto stepReach = [&](WorkholdingReach::Kind kind, double wanted) {
                return [&g, &gripPoint, &m1, step, kind, wanted] {
                    return WorkholdingReach{kind, wanted, 2.0 * g.radius(gripPoint, m1.minTravel),
                                            2.0 * g.radius(gripPoint, m1.maxTravel), step->name};
                };
            };
            if (!out.refusal.empty()) {
                const auto now = travel.find("Jaw 1");
                jp.travel =
                    std::clamp(now == travel.end() ? 0.0 : now->second, m1.minTravel, m1.maxTravel);
                travel = memberTravels(w, {{"Jaw 1", jp.travel}});
                for (const WorkholdingMember& m : w.members) {
                    stopped.insert(m.name);
                }
                stopHeight = g.height(datumAt(*jawSet, step->stop, {}).point, jp.travel);
            } else if (!soft) {
                jp.travel = (0.5 * held - g.radius(gripPoint, 0.0)) / perMm;
                travel = memberTravels(w, {{"Jaw 1", jp.travel}});
                reach(stepReach(WorkholdingReach::Kind::Grip, held));
                jp.travel = travel.at("Jaw 1");
                stopHeight = g.height(datumAt(*jawSet, step->stop, {}).point, jp.travel);
            } else {
                const SoftBore bore = placement.softBore.value_or(SoftBore{});
                if (!(bore.depth > 0.0) || !(bore.clampDiameter > 0.0)) {
                    throw userError(fmt::format("soft jaws '{}' are not bored yet: say how deep, "
                                                "and the ring or plug they clamp while bored",
                                                jawSet->name));
                }
                const double d =
                    bore.diameter.value_or(outside ? held + bore.clearance : held - bore.clearance);
                if (!(d > 0.0)) {
                    throw userError(fmt::format("the soft jaws' bore of {:.3f} mm is no bore", d));
                }
                // Bored while clamping the ring on the step; then closed (or opened) on the work.
                std::vector<double> t{(0.5 * bore.clampDiameter - g.radius(gripPoint, 0.0)) / perMm,
                                      0.0};
                // The master jaws in their range while bored, and while holding the work.
                travel = memberTravels(w, {{"Jaw 1", t[0]}});
                reach(stepReach(WorkholdingReach::Kind::Clamp, bore.clampDiameter));
                t[0] = travel.at("Jaw 1");
                t[1] = t[0] + (0.5 * held - 0.5 * d) / perMm;
                travel = memberTravels(w, {{"Jaw 1", t[1]}});
                reach([&m1, &t, perMm, d, held] {
                    return WorkholdingReach{WorkholdingReach::Kind::Grip,
                                            held,
                                            d + 2.0 * perMm * (m1.minTravel - t[0]),
                                            d + 2.0 * perMm * (m1.maxTravel - t[0]),
                                            {}};
                });
                t[1] = travel.at("Jaw 1");
                jp.boringTravel = t[0];
                jp.travel = t[1];
                jp.boreDiameter = d;
                jp.boreDepth = bore.depth;
                jp.clampDiameter = bore.clampDiameter;
                const Vec3 front = datumAt(*jawSet, set.front, {}).point;
                const double bottom = g.height(front, t[0]) - bore.depth;
                stopHeight = bottom;
                const Transform3 back = g.jawToTemplate(t[0]).inverse();
                JawCut c;
                c.axisPoint = back * g.axisPoint;
                c.axisDirection = (back.linear() * g.axis).normalized();
                c.radius = 0.5 * d;
                c.from = bottom;
                c.bore = outside;
                cut = c;
            }
            travel = memberTravels(w, {{"Jaw 1", jp.travel}});
            if (!(placement.stopOffset >= 0.0)) {
                throw userError("the stock cannot go in past the jaws' stop");
            }
            jp.stopZ = z0 - placement.stopOffset;
            out.jaws = jp;
            // The stock's held end against the stop (or as far out from it as set), on the axis.
            aT = g.radial;
            zT = g.axis;
            anchorT = g.axisPoint + g.axis * (stopHeight + placement.stopOffset);
            anchorW = 0.5 * (a0 + a1) * aW + 0.5 * (b0 + b1) * bW + z0 * Vec3::UnitZ();
            const Transform3 inTemplate = Frame{anchorT, aT, zT}.toParent();
            const Transform3 inWcs = Frame{anchorW, aW, Vec3::UnitZ()}.toParent();
            out.travel = travel;
            out.templateToWcs = inWcs * inTemplate.inverse();
            out.parts = posedParts(w, out.templateToWcs, travel);
            for (PosedPart& part : out.parts) {
                part.outOfReach = stopped.contains(part.member);
            }
            std::erase_if(out.parts, [&](const PosedPart& p) {
                const WorkholdingPart* part = w.findPart(p.id);
                return part && part->topJaw;
            });
            const Transform3 onJaw1 = g.jawToTemplate(jp.travel);
            for (int k = 1; w.findMember(fmt::format("Jaw {}", k)); ++k) {
                const Transform3 toWcs = out.templateToWcs * masterTurn(w, g, k) * onJaw1;
                for (const WorkholdingPart& p : jawSet->parts) {
                    // One id per copy: views keep their models by id, and showed one jaw for all.
                    Uuid::Bytes copy = p.id.bytes();
                    copy[15] = static_cast<std::uint8_t>(copy[15] ^ (k - 1));
                    PosedPart posed{Uuid(copy),  fmt::format("Jaw {}: {}", k, p.name),
                                    p.sha256,    fmt::format("Jaw {}", k),
                                    p.colour,    p.collides,
                                    p.shown,     toWcs,
                                    std::nullopt};
                    posed.cut = cut;
                    posed.outOfReach = out.outOfReach.has_value() || !out.refusal.empty();
                    out.parts.push_back(std::move(posed));
                }
            }
            out.mount = out.templateToWcs *
                        (w.mount.empty() ? Vec3(Vec3::Zero()) : datumAt(w, w.mount, travel).point);
            return out;
        }
        // A bar on WCS Z through the stock's middle, its bottom end in the jaws.
        out.opening = placement.jaws == JawSetting::OnStock ? out.stockWidth : placement.opening;
        const OpeningFn opening = [&](const std::map<std::string, double>& t) {
            return openingWith(w, *holding, Vec3::Zero(), t);
        };
        if (driverOf(w, own, datumMember(w, holding->faceA), opening)) {
            travel = travelsForOpening(w, *holding, out.opening, own);
            if (reach(openingReach(WorkholdingReach::Kind::Grip, out.opening))) {
                out.opening =
                    std::clamp(out.opening, std::min(out.outOfReach->from, out.outOfReach->to),
                               std::max(out.outOfReach->from, out.outOfReach->to));
            }
        } else {
            // A collet: its bore as modelled, closing by up to its grip range.
            checkRanges("the setup's member travel");
            const double bore = opening(travel);
            if (out.opening > bore + kTol || out.opening < bore - holding->gripRange - kTol) {
                out.outOfReach = WorkholdingReach{WorkholdingReach::Kind::Collet,
                                                  out.opening,
                                                  bore - holding->gripRange,
                                                  bore,
                                                  {}};
                if (!asFarAsItGoes) {
                    throw userError(reachMessage(*out.outOfReach, LengthUnit::Millimeter));
                }
                for (const WorkholdingPart& part : w.parts) {
                    stopped.insert(part.member); // the whole collet
                }
            }
            out.opening = bore;
        }
        if (!out.outOfReach && out.opening < out.stockWidth - kTol) {
            out.warnings.push_back(fmt::format("the chuck '{}' is open to {:.3f} mm, less than "
                                               "the stock's {:.3f} mm",
                                               w.name, out.opening, out.stockWidth));
        }
        const ChuckAxis axis = chuckAxis(w, *holding, travel);
        aT = holdingAxis(w, *holding, travel);
        zT = axis.direction;
        anchorT = axis.point;
        const double grip = placement.gripDepth.value_or(holding->gripDepth);
        anchorW = 0.5 * (a0 + a1) * aW + 0.5 * (b0 + b1) * bW + (z0 + grip) * Vec3::UnitZ();
        break;
    }
    }
    out.travel = travel;
    const Transform3 inTemplate = Frame{anchorT, aT, zT}.toParent();
    const Transform3 inWcs = Frame{anchorW, aW, Vec3::UnitZ()}.toParent();
    out.templateToWcs = inWcs * inTemplate.inverse();
    out.parts = posedParts(w, out.templateToWcs, travel);
    for (PosedPart& part : out.parts) {
        part.outOfReach = stopped.contains(part.member);
    }
    out.mount = out.templateToWcs *
                (w.mount.empty() ? Vec3(Vec3::Zero()) : datumAt(w, w.mount, travel).point);
    return out;
}

Json toJson(const WorkholdingDatum& datum) {
    return datumJson(datum);
}

WorkholdingDatum datumFromJson(const Json& j) {
    try {
        return datumFrom(j);
    } catch (const std::bad_optional_access&) {
        throw Error(ErrorCode::Parse, "unknown enumeration value in a workholding datum");
    }
}

DatumValue workholdingPoint(const Document& doc, const Setup& setup,
                            const WorkholdingDatum& datum) {
    const auto pose = setupWorkholdingPose(doc, setup);
    const WorkholdingTemplate* t =
        setup.workholding ? doc.project().findWorkholding(setup.workholding->templateId) : nullptr;
    if (!pose || !t) {
        throw userError("the setup has no workholding");
    }
    // As a datum of the template, so offsets, members and picks work as there.
    WorkholdingTemplate w = *t;
    WorkholdingDatum d = datum;
    d.name = "\x01point"; // never a template datum's name
    if (d.kind == DatumKind::Picked && (!d.part || !w.findPart(*d.part))) {
        throw userError("the point was picked on a part the workholding no longer has");
    }
    w.datums.push_back(d);
    const DatumValue v = datumAt(w, d.name, pose->travel);
    const Transform3 toProject = setup.wcs.toParent() * pose->templateToWcs;
    return {toProject * v.point, (toProject.linear() * v.direction).normalized()};
}

namespace {

std::optional<WorkholdingPose> posedForSetup(const Document& doc, const Setup& setup) {
    if (!setup.workholding) {
        return std::nullopt;
    }
    const WorkholdingTemplate* w = doc.project().findWorkholding(setup.workholding->templateId);
    if (!w) {
        throw userError("the setup's workholding is no longer in the project");
    }
    const std::optional<geom3d::Mesh> stock = stockMeshInWcs(doc, setup);
    if (!stock) {
        throw userError("the workholding needs the setup's stock, which cannot be resolved");
    }
    geom3d::Aabb3 bounds = stock->bounds();
    // On a lathe, stock from an earlier setup is what that setup left (DL-0044): a chuck closes on
    // what is there in its jaws, not on the bar it was.
    if (setup.stock.kind == StockKind::FromSetup && machineFor(doc.project(), setup).isLathe()) {
        CancelToken never;
        const geom2d::Regions carried = latheGeometry(doc, setup, never).stock;
        geom2d::Aabb2 all;
        for (const auto& r : carried) {
            all.extend(r.bounds());
        }
        const StockHolding* holding = setup.workholding->holding.empty()
                                          ? (w->holdings.empty() ? nullptr : &w->holdings.front())
                                          : w->findHolding(setup.workholding->holding);
        double radius = all.max.y();
        if (holding && holding->kind == HoldingKind::Chuck) {
            const double grip = setup.workholding->gripDepth.value_or(holding->gripDepth);
            geom2d::Aabb2 gripped;
            for (const auto& r : geom2d::intersect(
                     carried,
                     {{geom2d::Polyline::rectangle(Vec2(all.min.x(), -1.0),
                                                   Vec2(all.min.x() + grip, all.max.y() + 1.0)),
                       {}}},
                     {1e-3})) {
                gripped.extend(r.bounds());
            }
            if (!gripped.isEmpty()) {
                radius = gripped.max.y();
            }
        }
        bounds = {};
        bounds.extend(Vec3(-radius, -radius, all.min.x()));
        bounds.extend(Vec3(radius, radius, all.max.x()));
    }
    // A jaw set (DL-0069), and for an inside grip the bore the stock has where it is held.
    const WorkholdingTemplate* jawSet = nullptr;
    std::optional<double> inside;
    if (const auto& id = setup.workholding->jawSet) {
        jawSet = doc.project().findWorkholding(*id);
        if (setup.workholding->grip == GripSide::Inside &&
            setup.workholding->jaws == JawSetting::OnStock &&
            machineFor(doc.project(), setup).isLathe()) {
            CancelToken never;
            const geom2d::Regions stockNow = latheGeometry(doc, setup, never).stock;
            geom2d::Aabb2 all;
            for (const auto& r : stockNow) {
                all.extend(r.bounds());
            }
            const StockHolding* holding =
                setup.workholding->holding.empty()
                    ? (w->holdings.empty() ? nullptr : &w->holdings.front())
                    : w->findHolding(setup.workholding->holding);
            const double grip =
                setup.workholding->gripDepth.value_or(holding ? holding->gripDepth : 0.0);
            geom2d::Aabb2 gripped;
            for (const auto& r : geom2d::intersect(
                     stockNow,
                     {{geom2d::Polyline::rectangle(
                           Vec2(all.min.x(), -1.0),
                           Vec2(all.min.x() + std::max(grip, 0.1), all.max.y() + 1.0)),
                       {}}},
                     {1e-3})) {
                gripped.extend(r.bounds());
            }
            if (!gripped.isEmpty() && gripped.min.y() > 1e-3) {
                inside = 2.0 * gripped.min.y();
            }
        }
    }
    return poseWorkholding(*w, *setup.workholding, bounds, jawSet, inside, true);
}

} // namespace

std::optional<WorkholdingPose> setupWorkholdingPose(const Document& doc, const Setup& setup) {
    auto pose = posedForSetup(doc, setup);
    if (pose && pose->outOfReach) {
        throw userError(reachMessage(*pose->outOfReach, doc.project().displayUnit));
    }
    if (pose && !pose->refusal.empty()) {
        throw userError(pose->refusal);
    }
    return pose;
}

std::optional<WorkholdingPose> shownWorkholdingPose(const Document& doc, const Setup& setup) {
    return posedForSetup(doc, setup);
}

bool jawSetFits(const WorkholdingTemplate& chuck, const WorkholdingTemplate& jawSet) {
    return chuck.jawMount && jawSet.jawSet &&
           machine::sameMountName(chuck.jawMount->interfaceName, jawSet.jawSet->interfaceName);
}

const cad::Model* posedPartModel(const Document& doc, const PosedPart& part) {
    const cad::Model* model = doc.modelData(part.sha256);
    if (!model || !part.cut) {
        return model;
    }
    // Made once per jaw and cut: the same cut on the same file is the same solid.
    static std::mutex lock;
    static std::map<std::string, std::unique_ptr<const cad::Model>> made;
    const JawCut& c = *part.cut;
    const std::string key =
        fmt::format("{}|{:.9g},{:.9g},{:.9g}|{:.9g},{:.9g},{:.9g}|{:.9g}|{:.9g}|{}", part.sha256,
                    c.axisPoint.x(), c.axisPoint.y(), c.axisPoint.z(), c.axisDirection.x(),
                    c.axisDirection.y(), c.axisDirection.z(), c.radius, c.from, c.bore);
    const std::lock_guard<std::mutex> guard(lock);
    if (const auto it = made.find(key); it != made.end()) {
        return it->second.get();
    }
    // The removed volume about Z, then onto the cut's axis.
    constexpr double kFar = 1.0e4;
    const std::vector<Vec2> profile =
        c.bore
            ? std::vector<Vec2>{{c.from, 0.0}, {kFar, 0.0}, {kFar, c.radius}, {c.from, c.radius}}
            : std::vector<Vec2>{{c.from, c.radius}, {kFar, c.radius}, {kFar, kFar}, {c.from, kFar}};
    const Vec3 z = c.axisDirection.normalized();
    const Vec3 x = (std::abs(z.x()) < 0.9 ? Vec3(Vec3::UnitX()) : Vec3(Vec3::UnitY()))
                       .cross(z)
                       .normalized()
                       .cross(z)
                       .normalized();
    Transform3 onAxis = Transform3::Identity();
    onAxis.linear() << x, z.cross(x), z;
    onAxis.translation() = c.axisPoint;
    const cad::Model removed = cad::transformed(cad::revolvedAboutZ(profile), onAxis);
    auto cut = std::make_unique<const cad::Model>(cad::subtracted(*model, removed));
    return made.emplace(key, std::move(cut)).first->second.get();
}

void removeUnusedWorkholdings(Project& project) {
    std::erase_if(project.workholdings, [&](const WorkholdingTemplate& w) {
        return std::none_of(project.setups.begin(), project.setups.end(), [&](const Setup& s) {
            return s.workholding &&
                   (s.workholding->templateId == w.id || s.workholding->jawSet == w.id);
        });
    });
}

std::optional<WorkholdingTemplate> viseTemplate(const std::string& name, const ViseParts& vise,
                                                const Vec3& axis, const Vec3& up, double maxOpening,
                                                std::string* why) {
    auto fail = [&](const char* reason) -> std::optional<WorkholdingTemplate> {
        if (why) {
            *why = reason;
        }
        return std::nullopt;
    };
    if (axis.norm() < 1e-12 || up.norm() < 1e-12 ||
        std::abs(axis.normalized().dot(up.normalized())) > 1e-6) {
        return fail("the jaw axis and the up direction must be perpendicular");
    }
    const Vec3 a = axis.normalized();
    const Vec3 z = up.normalized();
    const Vec3 b = z.cross(a);
    const geom3d::Aabb3* low = nullptr;
    const geom3d::Aabb3* high = nullptr;
    geom3d::Aabb3 all;
    for (const auto& [part, box] : vise.parts) {
        all.extend(box);
        low = part.id == vise.fixedJaw ? &box : low;
        high = part.id == vise.movingJaw ? &box : high;
    }
    if (!low) {
        return fail(vise.selfCentring ? "mark the opposite jaw" : "mark the fixed jaw");
    }
    if (!high) {
        return fail("mark the moving jaw");
    }
    const auto [l0, l1] = extent(*low, a);
    const auto [h0, h1] = extent(*high, a);
    if (h0 < l1 - 1e-9) {
        return fail("the moving jaw is not on the side the jaw axis points to");
    }
    const double modelled = h0 - l1;
    const double lowTop = extent(*low, z).second;
    const double top = vise.selfCentring ? std::min(lowTop, extent(*high, z).second) : lowTop;
    geom3d::Aabb3 jaws = *low;
    if (vise.selfCentring) {
        jaws.extend(*high);
    }
    const auto [c0, c1] = extent(jaws, b);
    const double middle = 0.5 * (c0 + c1);
    const double share = vise.selfCentring ? 0.5 : 1.0;

    WorkholdingTemplate w;
    w.name = name;
    w.up = z;
    w.members.push_back(
        {"Moving jaw", a, -share * modelled, share * (maxOpening - modelled), {}, 1.0});
    if (vise.selfCentring) {
        w.members.push_back({"Opposite jaw", Vec3(-a), -share * modelled,
                             share * (maxOpening - modelled), "Moving jaw", 1.0});
    }
    for (auto [part, box] : vise.parts) {
        part.member = part.id == vise.movingJaw                       ? "Moving jaw"
                      : vise.selfCentring && part.id == vise.fixedJaw ? "Opposite jaw"
                                                                      : "";
        w.parts.push_back(part);
    }
    auto typed = [](const char* datumName, const Vec3& point, const Vec3& direction,
                    const std::string& member) {
        WorkholdingDatum d;
        d.name = datumName;
        d.kind = DatumKind::Typed;
        d.shape = DatumShape::Plane;
        d.point = point;
        d.direction = direction;
        d.member = member;
        return d;
    };
    const char* lowFace = vise.selfCentring ? "Opposite jaw face" : "Fixed jaw face";
    w.datums.push_back(
        typed(lowFace, l1 * a + middle * b + top * z, a, vise.selfCentring ? "Opposite jaw" : ""));
    w.datums.push_back(typed("Moving jaw face", h0 * a + middle * b + top * z, -a, "Moving jaw"));
    w.datums.push_back(typed("Jaw tops", l1 * a + middle * b + top * z, z, ""));
    const Vec3 c = all.center();
    w.datums.push_back(typed("Bottom centre", c - (c.dot(z) - extent(all, z).first) * z, -z, ""));
    w.mount = "Bottom centre";
    StockHolding h;
    h.name = "Jaws";
    h.kind = HoldingKind::Jaws;
    h.faceA = lowFace;
    h.faceB = "Moving jaw face";
    h.seat = "Jaw tops";
    w.holdings.push_back(h);
    return w;
}

} // namespace ocam::document
