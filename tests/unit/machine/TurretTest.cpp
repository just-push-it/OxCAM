// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// A lathe on the machine model's tree: its layout, stations and positions (DL-0067).

#include <ocam/foundation/Error.hpp>
#include <ocam/foundation/Json.hpp>
#include <ocam/foundation/Units.hpp>
#include <ocam/machine/Turret.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <cmath>

using namespace ocam;
using namespace ocam::machine;
using Catch::Approx;
using Catch::Matchers::ContainsSubstring;

TEST_CASE("a lathe with a layout has a tree and solids", "[machine][turret]") {
    const Machine& m = genericLatheTurret();
    REQUIRE(m.lathe);
    CHECK(m.validate().empty());
    const MachineModel model = effectiveModel(m);
    CHECK(model.spindleCarrier == "T");
    for (const char* axis : {"X", "Y", "Z", "T", "C", "W"}) {
        CHECK(model.axis(axis));
    }
    CHECK_FALSE(model.axis("Z2"));
    CHECK(model.axis("T")->rotary());
    CHECK(model.axis("C")->rotary());
    CHECK_FALSE(model.axis("W")->rotary());
    const std::vector<LatheSolid> solids = latheSolids(m);
    const auto carrierOf = [&](const char* name) {
        const auto it = std::find_if(solids.begin(), solids.end(),
                                     [&](const LatheSolid& s) { return s.shape.name == name; });
        REQUIRE(it != solids.end());
        return it->carrier;
    };
    CHECK(carrierOf("Turret") == "T");
    CHECK(carrierOf("Chuck") == "C");
    CHECK(carrierOf("Tailstock body") == "W");
    CHECK(carrierOf("Enclosure, back").empty());
    // The layout's enclosure walls are the enclosure, and nothing else of it is.
    for (const LatheSolid& s : solids) {
        CHECK(s.enclosure == (s.shape.name.rfind("Enclosure, ", 0) == 0));
    }
    for (const ModelPart& p : componentParts(m, model)) {
        CHECK(p.enclosure == (p.name.rfind("Enclosure, ", 0) == 0));
    }

    const Machine& sub = genericLatheSub();
    CHECK(sub.validate().empty());
    const MachineModel subModel = effectiveModel(sub);
    CHECK(subModel.axis("Z2"));
    CHECK(subModel.axis("C2")->parent == "Z2");
    CHECK_FALSE(subModel.axis("W"));

    // A lathe without a layout is as it was: nothing on the machine.
    CHECK(effectiveModel(genericLathe2()).empty());
    CHECK(latheSolids(genericLathe2()).empty());
}

TEST_CASE("a turret's stations and the way it indexes", "[machine][turret]") {
    const Machine& m = genericLatheTurret();
    REQUIRE(m.toolCapacity == 12);
    CHECK(stationAngle(m, 1) == Approx(0.0));
    CHECK(stationAngle(m, 4) == Approx(kPi / 2.0));
    SECTION("the short way round") {
        CHECK(indexTo(m, stationAngle(m, 1), 3) == Approx(kPi / 3.0));
        CHECK(indexTo(m, stationAngle(m, 1), 12) == Approx(-kPi / 6.0));
        CHECK(indexTo(m, stationAngle(m, 12), 1) == Approx(stationAngle(m, 12) + kPi / 6.0));
    }
    SECTION("half way round goes forward") {
        CHECK(indexTo(m, stationAngle(m, 1), 7) == Approx(kPi));
    }
    SECTION("the angle goes on from where it is") {
        const double t = indexTo(m, 4.0 * kPi + stationAngle(m, 2), 3);
        CHECK(t == Approx(4.0 * kPi + stationAngle(m, 3)));
    }
    // T at a station's angle brings it to the working position: what stood at the station's angle
    // back from there comes round to the point in front of the spindle.
    const LatheTurret& t = m.lathe->turret;
    const Vec3 working = tipPoint(*m.lathe, Vec2(t.radialReach, 0.0));
    const Vec3 parked = turretTurn(t, -stationAngle(m, 5)) * working;
    CHECK((parked - working).norm() > 100.0);
    CHECK((turretTurn(t, stationAngle(m, 5)) * parked - working).norm() < 1e-9);
}

TEST_CASE("a tool's default place in the turret", "[machine][turret]") {
    const LatheLayout& l = *genericLatheTurret().lathe;
    const LatheTurret& t = l.turret;
    SECTION("across the spindle: reach from the index line, its shank on the disc's face") {
        const Vec2 tip = defaultTipFromCentre(l, {true, 8.0, false});
        CHECK(tip.x() == Approx(t.radialReach));
        CHECK(tip.y() == Approx(-0.5 * t.width - 8.0));
    }
    SECTION("along the spindle: at the pocket's pitch, out from the face") {
        const Vec2 tip = defaultTipFromCentre(l, {false, -6.0, false});
        CHECK(tip.x() == Approx(t.axialPitch - 6.0));
        CHECK(tip.y() == Approx(-0.5 * t.width - t.axialReach));
    }
    SECTION("the sub-spindle's side") {
        CHECK(defaultTipFromCentre(l, {false, 0.0, true}).y() ==
              Approx(0.5 * t.width + t.axialReach));
        CHECK(defaultTipFromCentre(l, {true, 8.0, true}).y() == Approx(0.5 * t.width + 8.0));
    }
    CHECK(tipPoint(l, Vec2(100.0, -20.0)).isApprox(t.centre + Vec3(-100.0, 0.0, -20.0)));
}

TEST_CASE("a lathe's approach goes out first only when it must", "[machine][turret]") {
    // Radius and Z of where the tool stands and where it goes.
    const auto outFirst = [](double r0, double z0, double r1, double z1) {
        return approachOutFirst(Vec3(r0, 0.0, z0), Vec3(r1, 0.0, z1));
    };
    CHECK_FALSE(outFirst(21.0, 6.0, 21.0, 6.0));   // nowhere to go
    CHECK_FALSE(outFirst(30.0, 6.0, 10.0, 2.0));   // in, and down to the front: Z first
    CHECK_FALSE(outFirst(10.0, -30.0, 30.0, 6.0)); // out of a bore: Z first
    CHECK_FALSE(outFirst(10.0, 6.0, 30.0, 6.0));   // out, level: either
    CHECK(outFirst(12.0, 11.0, 31.0, -30.0));      // from the axis to a cross hole down the part
    // By the radius, as a hole around the axis stands at any C.
    CHECK(approachOutFirst(Vec3(0.0, 12.0, 11.0), Vec3(31.0, 0.0, -30.0)));
    CHECK_FALSE(approachOutFirst(Vec3(0.0, 40.0, 11.0), Vec3(31.0, 0.0, -30.0)));
}

TEST_CASE("the axes put the working tool on the program's point", "[machine][turret]") {
    const Machine& m = genericLatheTurret();
    const MachineModel model = effectiveModel(m);
    WorkMount mount;
    mount.carrier = "C";
    mount.wcs = Eigen::Translation3d(0.0, 0.0, 150.0) * Transform3::Identity();
    LatheMotion motion;
    motion.station = 3;
    motion.tip = tipPoint(*m.lathe, Vec2(190.0, -60.0));
    motion.held.t = stationAngle(m, 3);
    toolpath::Toolpath path;
    SECTION("turning: X and Z, whatever the turret's angle") {
        const auto axes = latheAxes(model, motion, mount, path, Vec3(20.0, 0.0, -30.0));
        REQUIRE(axes);
        CHECK(axes->t == motion.held.t);
        const Vec3 at = motion.tip + axes->linear;
        CHECK(at.isApprox(mount.wcs * Vec3(20.0, 0.0, -30.0), 1e-9));
    }
    SECTION("a tool that does not reach the same Z travels further") {
        LatheMotion other = motion;
        other.tip = tipPoint(*m.lathe, Vec2(110.0, -120.0));
        const auto a = latheAxes(model, motion, mount, path, Vec3(0.0, 0.0, 0.0));
        const auto b = latheAxes(model, other, mount, path, Vec3(0.0, 0.0, 0.0));
        REQUIRE(a);
        REQUIRE(b);
        CHECK(b->linear.z() - a->linear.z() == Approx(60.0));
        CHECK(b->linear.x() - a->linear.x() == Approx(-80.0));
    }
    SECTION("polar interpolation: the part turns, the tool stays in the X-Z plane") {
        path.live = true;
        path.liveAxes = toolpath::LiveAxes::Polar;
        toolpath::Move move;
        move.axis = Vec3::UnitZ();
        path.moves.push_back(move);
        const Vec3 tip(10.0, 10.0, 0.0);
        const auto axes = latheAxes(model, motion, mount, path, tip);
        REQUIRE(axes);
        CHECK(axes->rotary.c == Approx(kPi / 4.0));
        const Vec3 at = motion.tip + axes->linear;
        CHECK(at.y() == Approx(0.0).margin(1e-9));
        CHECK(at.x() == Approx(std::sqrt(200.0)));
    }
    SECTION("held C on the face: Y moves on a machine with one") {
        path.live = true;
        path.liveAxes = toolpath::LiveAxes::HeldC;
        toolpath::Move move;
        move.axis = Vec3::UnitZ();
        path.moves.push_back(move);
        const auto axes = latheAxes(model, motion, mount, path, Vec3(10.0, 7.0, 0.0));
        REQUIRE(axes);
        CHECK(axes->linear.y() == Approx(7.0));
    }
}

TEST_CASE("a lathe's layout in the machine file", "[machine][turret]") {
    const Machine& m = genericLatheSub();
    const Machine back = machineFromJson(toJson(m));
    CHECK(back == m);
    CHECK(back.lathe->subSpindle->workZ2 == m.lathe->subSpindle->workZ2);
    SECTION("where a block seats") {
        CHECK(back.lathe->turret.blockRadius == 150.0);
        CHECK(back.lathe->turret.blockAlong == -30.0);
        Json older = toJson(m);
        older["version"] = 8;
        older["lathe"]["turret"].erase("block_radius");
        older["lathe"]["turret"].erase("block_along");
        const Machine read = machineFromJson(older);
        CHECK(read.lathe->turret.blockRadius == read.lathe->turret.diameter / 2.0);
        CHECK(read.lathe->turret.blockAlong == 0.0);
    }
    SECTION("the blocks a turret takes and what its quill takes") {
        const Machine& turret = genericLatheTurret();
        CHECK(turret.lathe->turret.blockMounts ==
              std::vector<std::string>{"Generic bolt-on (placeholder)"});
        CHECK(turret.lathe->tailstock->quillInterface == "Generic taper (placeholder)");
        Machine mine = turret;
        mine.lathe->turret.blockMounts = {"VDI 40", "BMT 55"};
        mine.lathe->tailstock->quillInterface = "MT 4";
        CHECK(machineFromJson(toJson(mine)) == mine);
        // Version 9 did not say.
        Json older = toJson(turret);
        older["version"] = 9;
        older["lathe"]["turret"].erase("block_mounts");
        older["lathe"]["tailstock"].erase("quill_interface");
        const Machine read = machineFromJson(older);
        CHECK(read.lathe->turret.blockMounts.empty());
        CHECK(read.lathe->tailstock->quillInterface.empty());
        // Not written when not said.
        CHECK_FALSE(toJson(read).at("lathe").at("turret").contains("block_mounts"));
        CHECK(sameMountName("VDI 40", "vdi40"));
        CHECK(sameMountName(" MT  4", "mt 4"));
        CHECK_FALSE(sameMountName("VDI 40", "VDI 30"));
    }
    SECTION("how far the quill goes out") {
        Machine mine = genericLatheTurret();
        CHECK(mine.lathe->tailstock->quillTravel == 0.0);
        // Not said: not written.
        CHECK_FALSE(toJson(mine).at("lathe").at("tailstock").contains("quill_travel"));
        mine.lathe->tailstock->quillTravel = 100.0;
        const Json doc = toJson(mine);
        CHECK(doc.at("lathe").at("tailstock").at("quill_travel") == 100.0);
        CHECK_NOTHROW(machineSchema().validate(doc, "machine"));
        CHECK(machineFromJson(doc) == mine);
        CHECK(mine.validate().empty());
        Json negative = doc;
        negative["lathe"]["tailstock"]["quill_travel"] = -1.0;
        CHECK_THROWS(machineSchema().validate(negative, "machine"));
        Machine bad = mine;
        bad.lathe->tailstock->quillTravel = -1.0;
        CHECK_FALSE(bad.validate().empty());
        // Version 14 did not say.
        Json older = toJson(genericLatheTurret());
        older["version"] = 14;
        const Machine read = machineFromJson(older);
        CHECK(read.lathe->tailstock->quillTravel == 0.0);
        CHECK(toJson(read).at("version") == kMachineVersion);
    }
    SECTION("stations that take other blocks, a seat sideways, and the workholding mount") {
        Machine mine = genericLatheTurret();
        LatheTurret& t = mine.lathe->turret;
        t.blockMounts = {"Bolt-on"};
        t.stationMounts = {{{2, 4, 6}, {"Milling unit"}}};
        t.blockAcross = -75.0;
        mine.lathe->workholdingMount = 45.0;
        CHECK(t.blockMountsAt(1) == std::vector<std::string>{"Bolt-on"});
        CHECK(t.blockMountsAt(4) == std::vector<std::string>{"Milling unit"});
        CHECK(mine.validate().empty());
        const Json doc = toJson(mine);
        CHECK_NOTHROW(machineSchema().validate(doc, "machine"));
        CHECK(machineFromJson(doc) == mine);
        CHECK(doc.at("lathe").at("turret").at("block_across") == -75.0);
        CHECK(doc.at("lathe").at("workholding_mount").at("z") == 45.0);

        // What is wrong is said.
        Machine bad = mine;
        bad.lathe->turret.stationMounts.push_back({{4, 13}, {"Other"}});
        const std::vector<std::string> issues = bad.validate();
        const auto said = [&](const char* text) {
            return std::any_of(issues.begin(), issues.end(), [&](const std::string& s) {
                return s.find(text) != std::string::npos;
            });
        };
        CHECK(said("station 13"));
        CHECK(said("station 4 takes are said more than once"));

        // Version 10 had none of them.
        Json older = toJson(genericLatheTurret());
        older["version"] = 10;
        const Machine read = machineFromJson(older);
        CHECK(read.lathe->turret.stationMounts.empty());
        CHECK(read.lathe->turret.blockAcross == 0.0);
        CHECK(read.lathe->workholdingMount == 0.0);
        CHECK_FALSE(toJson(read).at("lathe").contains("workholding_mount"));
        CHECK_FALSE(toJson(read).at("lathe").at("turret").contains("station_mounts"));
    }
    SECTION("an older file has none") {
        Json doc = toJson(genericLathe2());
        doc["version"] = 7;
        doc.erase("lathe");
        CHECK_FALSE(machineFromJson(doc).lathe);
    }
    SECTION("problems are named") {
        Machine broken = m;
        broken.lathe->turret.radialReach = -1.0;
        CHECK_FALSE(broken.validate().empty());
        Machine noStations = m;
        noStations.toolCapacity = 0;
        REQUIRE_FALSE(noStations.validate().empty());
        CHECK_THAT(noStations.validate().front(), ContainsSubstring("stations"));
        Machine mill = genericMill3();
        mill.lathe = m.lathe;
        CHECK_FALSE(mill.validate().empty());
    }
    SECTION("a lathe's own model needs the layout, and its axes") {
        Machine own = m;
        own.model = effectiveModel(m);
        CHECK(own.validate().empty());
        own.lathe.reset();
        REQUIRE_FALSE(own.validate().empty());
        CHECK_THAT(own.validate().front(), ContainsSubstring("layout"));
        Machine wrong = m;
        wrong.model = effectiveModel(m);
        wrong.model.spindleCarrier = "X";
        REQUIRE_FALSE(wrong.validate().empty());
        CHECK_THAT(wrong.validate().front(), ContainsSubstring("turret"));
        Machine reversed = m;
        reversed.model = effectiveModel(m);
        for (ModelAxis& a : reversed.model.axes) {
            if (a.name == "X") {
                a.direction = -a.direction;
            }
        }
        REQUIRE_FALSE(reversed.validate().empty());
        CHECK_THAT(reversed.validate().front(), ContainsSubstring("X axis"));
    }
}

TEST_CASE("how a machine stands", "[machine][orientation]") {
    // Without its own, a machine stands as its kinematics: a mill with Z up and its front at -Y
    // (the world's axes are the machine's), a lathe with Y up, the spindle's Z across and the
    // operator at -X.
    Machine mill = genericMill3();
    CHECK_FALSE(mill.orientation);
    CHECK(mill.standing() == MachineOrientation{AxisSense::PlusZ, AxisSense::MinusY});
    CHECK(mill.standing().worldAxes().isApprox(Mat3::Identity()));
    const MachineOrientation lathe = defaultOrientation(Kinematics::Lathe2);
    CHECK(lathe == MachineOrientation{AxisSense::PlusY, AxisSense::MinusX});
    const Mat3 world = lathe.worldAxes();
    CHECK(world.col(0).isApprox(Vec3::UnitZ())); // right: away from the headstock
    CHECK(world.col(1).isApprox(Vec3::UnitX())); // away from the operator: the turret
    CHECK(world.col(2).isApprox(Vec3::UnitY())); // up
    CHECK(std::abs(world.determinant() - 1.0) < 1e-12);

    // The generic lathes say it, the generic mills do not.
    for (const Machine* m : builtinMachines()) {
        CAPTURE(m->id);
        CHECK(m->standing() == defaultOrientation(m->kinematics));
        CHECK(m->orientation.has_value() == m->isLathe());
    }

    // A vertical lathe: Z up, the operator at -X. Read and written as "+Z" and "-X".
    Machine vtl = genericLatheTurret();
    vtl.orientation = MachineOrientation{AxisSense::PlusZ, AxisSense::MinusX};
    CHECK(vtl.validate().empty());
    const Json doc = toJson(vtl);
    CHECK(doc.at("version") == kMachineVersion);
    CHECK(doc.at("orientation") == Json{{"up", "+Z"}, {"front", "-X"}});
    CHECK(machineFromJson(doc) == vtl);
    const Mat3 vtlWorld = vtl.standing().worldAxes();
    CHECK(vtlWorld.col(2).isApprox(Vec3::UnitZ()));
    CHECK(vtlWorld.col(1).isApprox(Vec3::UnitX()));
    CHECK(vtlWorld.col(0).isApprox(-Vec3::UnitY()));
    CHECK(std::abs(vtlWorld.determinant() - 1.0) < 1e-12);
    // Its enclosure's door is still at -X, and the roof is at +Z now: the +Y wall is a side.
    const auto shown = [](const Machine& m, const char* name) {
        const std::vector<LatheSolid> solids = latheSolids(m);
        const auto it = std::find_if(solids.begin(), solids.end(),
                                     [&](const LatheSolid& s) { return s.shape.name == name; });
        REQUIRE(it != solids.end());
        return it->shown;
    };
    CHECK_FALSE(shown(vtl, "Enclosure, door"));
    CHECK_FALSE(shown(vtl, "Enclosure, far end"));
    CHECK(shown(vtl, "Enclosure, roof"));
    CHECK_FALSE(shown(genericLatheTurret(), "Enclosure, door"));
    CHECK_FALSE(shown(genericLatheTurret(), "Enclosure, roof"));
    CHECK(shown(genericLatheTurret(), "Enclosure, far end"));

    // A machine without it writes none, and a version 11 file reads as before.
    Json old = toJson(mill);
    CHECK_FALSE(old.contains("orientation"));
    old["version"] = 11;
    CHECK(machineFromJson(old) == mill);
    Json oldLathe = toJson(genericLatheTurret());
    oldLathe.erase("orientation");
    oldLathe["version"] = 11;
    const Machine upgraded = machineFromJson(oldLathe);
    CHECK_FALSE(upgraded.orientation);
    CHECK(upgraded.standing() == lathe);

    // Up and front on one axis, or a name that is not one, are refused.
    Machine both = mill;
    both.orientation = MachineOrientation{AxisSense::PlusZ, AxisSense::MinusZ};
    REQUIRE(both.validate().size() == 1);
    CHECK_THAT(both.validate().front(),
               Catch::Matchers::ContainsSubstring("+Z up and its front at -Z"));
    CHECK_THROWS_AS(machineFromJson(toJson(both)), Error);
    Json bad = toJson(vtl);
    bad["orientation"]["up"] = "up";
    CHECK_THROWS_AS(machineFromJson(bad), Error);
    CHECK(axisSenseFromString("-y") == AxisSense::MinusY);
    CHECK(axisSenseFromString("Z") == AxisSense::PlusZ);
    CHECK_FALSE(axisSenseFromString("+W"));
}
