// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ocam/foundation/Error.hpp>
#include <ocam/tooling/Profile.hpp>
#include <ocam/tooling/Tool.hpp>
#include <ocam/tooling/ToolLibrary.hpp>
#include <ocam/tooling/TurningHolder.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>

using namespace ocam;
using namespace ocam::tooling;
using Catch::Approx;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

TEST_CASE("tool type and coolant names round-trip", "[tooling]") {
    for (ToolType t : allToolTypes()) {
        CHECK(parseToolType(toString(t)) == t);
        CHECK_FALSE(displayName(t).empty());
    }
    CHECK_FALSE(parseToolType("spork").has_value());
    for (Coolant c :
         {Coolant::Off, Coolant::Flood, Coolant::Mist, Coolant::Air, Coolant::ThroughTool}) {
        CHECK(parseCoolant(toString(c)) == c);
    }
}

TEST_CASE("cutting preset speeds and feeds", "[tooling]") {
    CuttingPreset p;
    p.surfaceSpeed = 200.0 * 1000.0 / 60.0; // 200 m/min
    p.feedPerTooth = 0.05;
    const double n = p.spindleSpeed(10.0);                           // rev/s
    CHECK_THAT(n * 60.0, WithinRel(200000.0 / (kPi * 10.0), 1e-12)); // ~6366 rpm
    CHECK_THAT(p.feedRate(10.0, 3), WithinRel(0.05 * 3 * n, 1e-12));
    CHECK(p.spindleSpeed(0.0) == 0.0);
}

TEST_CASE("every default tool is valid and rotating tools have a profile", "[tooling]") {
    for (ToolType type : allToolTypes()) {
        INFO(toString(type));
        const Tool tool = makeDefaultTool(type, 7, 8.0);
        CHECK(tool.validate().empty());
        CHECK(tool.number == 7);
        if (isTurningTool(type)) {
            CHECK_THROWS_AS(toolProfile(tool), Error);
            continue;
        }
        const ToolProfile profile = toolProfile(tool);
        const auto& v = profile.outline.vertices();
        REQUIRE(v.size() >= 3);
        CHECK_FALSE(profile.outline.isClosed());
        CHECK(v.front().p.x() == 0.0);
        CHECK(v.back().p.x() == 0.0);
        CHECK_THAT(v.back().p.y(), WithinAbs(tool.mill.overallLength, 1e-12));
        CHECK(profile.cuttingLength > 0.0);
        CHECK(profile.maxRadius > 0.0);
    }
}

TEST_CASE("profile shapes match the analytic tool geometry", "[tooling][profile]") {
    SECTION("ball end mill") {
        const ToolProfile p = toolProfile(makeBallEndMill(1, 10.0, 20.0, 60.0));
        CHECK_THAT(profileRadiusAt(p, 0.0), WithinAbs(0.0, 1e-9));
        CHECK_THAT(profileRadiusAt(p, 2.5), WithinAbs(std::sqrt(25.0 - 2.5 * 2.5), 1e-3));
        CHECK_THAT(profileRadiusAt(p, 5.0), WithinAbs(5.0, 1e-9));
        CHECK_THAT(profileRadiusAt(p, 30.0), WithinAbs(5.0, 1e-9));
        CHECK(profileRadiusAt(p, 61.0) == 0.0);
    }
    SECTION("bull-nose end mill") {
        const ToolProfile p = toolProfile(makeBullNoseEndMill(1, 12.0, 2.0, 24.0, 70.0));
        CHECK_THAT(profileRadiusAt(p, 0.0), WithinAbs(4.0, 1e-9));
        CHECK_THAT(profileRadiusAt(p, 2.0), WithinAbs(6.0, 1e-9));
        CHECK_THAT(profileRadiusAt(p, 1.0), WithinAbs(4.0 + std::sqrt(3.0), 1e-3));
    }
    SECTION("drill point") {
        const Tool drill = makeDrill(1, 8.0, degToRad(118.0), 40.0, 80.0);
        const ToolProfile p = toolProfile(drill);
        const double tip = 4.0 / std::tan(degToRad(59.0));
        CHECK_THAT(profileRadiusAt(p, tip / 2.0), WithinAbs(2.0, 1e-6));
        CHECK_THAT(profileRadiusAt(p, tip), WithinAbs(4.0, 1e-6));
    }
    SECTION("stepped shank") {
        Tool t = makeFlatEndMill(1, 6.0, 12.0, 50.0);
        t.mill.shankDiameter = 8.0;
        const ToolProfile p = toolProfile(t);
        CHECK_THAT(profileRadiusAt(p, 6.0), WithinAbs(3.0, 1e-9));
        CHECK_THAT(profileRadiusAt(p, 30.0), WithinAbs(4.0, 1e-9));
        CHECK(p.maxRadius == 3.0);
    }
    SECTION("lollipop sphere") {
        const Tool t = makeDefaultTool(ToolType::Lollipop, 1, 10.0);
        const ToolProfile p = toolProfile(t);
        CHECK_THAT(profileRadiusAt(p, 5.0), WithinAbs(5.0, 1e-3));
        CHECK_THAT(profileRadiusAt(p, 20.0), WithinAbs(2.5, 1e-9)); // neck
    }
}

TEST_CASE("invalid tools are reported", "[tooling]") {
    Tool t = makeFlatEndMill(0, -1.0, 10.0, 5.0);
    const auto issues = t.validate();
    CHECK(issues.size() >= 2);
    CHECK_THROWS_AS(toolProfile(t), Error);

    Tool bull = makeBullNoseEndMill(1, 10.0, 6.0, 20.0, 60.0);
    CHECK_FALSE(bull.validate().empty()); // corner radius larger than the radius

    Tool drill = makeDrill(1, 8.0, 0.0, 40.0, 80.0);
    CHECK_FALSE(drill.validate().empty());

    Tool tap = makeDefaultTool(ToolType::Tap, 1);
    tap.mill.threadPitch = 0.0;
    CHECK_FALSE(tap.validate().empty());

    Tool withPreset = makeFlatEndMill(1, 10.0, 20.0, 60.0);
    CuttingPreset bad;
    bad.plungeFeed = -1.0;
    withPreset.presets.push_back(bad);
    CHECK(withPreset.validate().size() == 1);

    Tool insert = makeDefaultTool(ToolType::TurningInsert, 1);
    insert.insert.orientation = 12;
    CHECK(insert.validate().size() == 1);
}

TEST_CASE("holder validation", "[tooling]") {
    Holder h;
    h.profile = geom2d::Polyline::fromPoints({{0, 0}, {10, 0}, {10, 30}, {0, 30}}, false);
    CHECK(h.validate().empty());
    h.profile = geom2d::Polyline::fromPoints({{5, 0}, {10, 0}}, false);
    CHECK_FALSE(h.validate().empty());
    h.profile = geom2d::Polyline::fromPoints({{0, 0}, {10, 0}, {0, 0}}, false);
    CHECK_FALSE(h.validate().empty()); // no length
}

TEST_CASE("stepped holders", "[tooling]") {
    HolderDimensions d;
    d.noseDiameter = 20.0;
    d.noseTopDiameter = 26.0;
    d.noseLength = 50.0;
    d.bodyDiameter = 0.0; // no body
    d.flangeDiameter = 63.0;
    d.flangeLength = 15.0;
    const Holder h = makeHolder("shrink fit", d);
    CHECK(h.name == "shrink fit");
    CHECK(h.validate().empty());
    CHECK_THAT(h.length(), WithinAbs(65.0, 1e-12));
    CHECK_THAT(h.maxRadius(), WithinAbs(31.5, 1e-12));
    const auto& v = h.profile.vertices();
    REQUIRE(v.size() == 6);
    CHECK(v[1].p == Vec2(10.0, 0.0));
    CHECK(v[2].p == Vec2(13.0, 50.0));
    CHECK(v[3].p == Vec2(31.5, 50.0));
    CHECK(v.back().p == Vec2(0.0, 65.0));

    CHECK_THROWS_AS(makeHolder("nothing", HolderDimensions{0, 0, 0, 0, 0, 0, 0}), Error);
}

TEST_CASE("tool families and holder kinds", "[tooling]") {
    std::size_t counted = 0;
    for (ToolFamily family :
         {ToolFamily::Milling, ToolFamily::HoleMaking, ToolFamily::Turning, ToolFamily::Probing}) {
        for (ToolType type : toolTypesOf(family)) {
            CHECK(toolFamily(type) == family);
            // A bar puller is a lathe's tool with no insert (DL-0091).
            CHECK((family == ToolFamily::Turning) ==
                  (isTurningTool(type) || type == ToolType::BarPuller));
            ++counted;
        }
    }
    CHECK(counted == allToolTypes().size());
    CHECK(toolFamily(ToolType::Tap) == ToolFamily::HoleMaking);
    CHECK(toolFamily(ToolType::ThreadMill) == ToolFamily::Milling);
    CHECK(toolFamily(ToolType::Probe) == ToolFamily::Probing);

    for (HolderKind kind : {HolderKind::Milling, HolderKind::Turning}) {
        CHECK(parseHolderKind(toString(kind)) == kind);
    }
    CHECK_FALSE(parseHolderKind("lathe").has_value());

    Holder collet = makeHolder("ER32", {});
    CHECK(collet.kind == HolderKind::Milling);
    CHECK(holds(collet, ToolType::FlatEndMill));
    CHECK(holds(collet, ToolType::Tap));
    CHECK_FALSE(holds(collet, ToolType::TurningInsert));
    Holder block = collet;
    block.kind = HolderKind::Turning;
    CHECK(holds(block, ToolType::GroovingInsert));
    CHECK_FALSE(holds(block, ToolType::Drill));
}

TEST_CASE("stickout and the assembly's gauge length", "[tooling]") {
    Tool t = makeFlatEndMill(1, 10.0, 22.0, 75.0);
    t.stickout = 0.0;
    CHECK(visibleLength(t) == 75.0); // 0 = the overall length
    CHECK(gaugeLength(t, nullptr) == 75.0);
    t.stickout = 40.0;
    CHECK(visibleLength(t) == 40.0);
    CHECK(gaugeLength(t, nullptr) == 40.0); // straight in the spindle

    HolderDimensions d; // nose 40 + body 20 + flange 16
    const Holder holder = makeHolder("chuck", d);
    CHECK_THAT(gaugeLength(t, &holder), WithinAbs(40.0 + 76.0, 1e-12));
    // Setting the gauge length sets the stickout.
    CHECK_THAT(stickoutForGauge(130.0, &holder), WithinAbs(54.0, 1e-12));
    CHECK_THAT(stickoutForGauge(130.0, nullptr), WithinAbs(130.0, 1e-12));
    CHECK(stickoutForGauge(70.0, &holder) <= 0.0); // shorter than the holder
    t.stickout = stickoutForGauge(130.0, &holder);
    CHECK_THAT(gaugeLength(t, &holder), WithinAbs(130.0, 1e-12));
}

TEST_CASE("an ISO holder with custom values", "[tooling][turning]") {
    TurningHolderCode c = *parseDesignation("PCLNR 2525M 12");
    c.style = kCustomStyle;
    c.customStyle = 70.0;
    c.clearance = kCustomClearance;
    c.customClearance = 11.0;
    c.length = kCustomLength;
    c.customLength = 175.0;
    CHECK(validate(c).empty());
    CHECK(designation(c) == "PCXOR 2525X 12");
    CHECK(holderLength(c) == 175.0);
    CHECK(sideCuttingEdgeAngle(c) == Catch::Approx(degToRad(70.0)));
    const Holder h = makeTurningHolder(c);
    CHECK(h.length() == 175.0);
    SECTION("a tool takes them, and only an insert with that relief fits") {
        Tool tool = makeDefaultTool(ToolType::TurningInsert, 1);
        CHECK(holderMismatches(tool, h) == std::vector{HolderMismatch::Clearance});
        fitToHolder(tool, h);
        CHECK(tool.insert.reliefAngle == Catch::Approx(degToRad(11.0)));
        CHECK(tool.insert.cuttingEdgeAngle == Catch::Approx(degToRad(70.0)));
        CHECK(holderMismatches(tool, h).empty());
    }
    SECTION("the letters without their values are problems") {
        TurningHolderCode bad = c;
        bad.customStyle.reset();
        CHECK_FALSE(validate(bad).empty());
        bad = c;
        bad.customClearance.reset();
        CHECK_FALSE(validate(bad).empty());
        bad = c;
        bad.customLength = 0.0;
        CHECK_FALSE(validate(bad).empty());
    }
    SECTION("an angle that leaves the corner no clearance is a problem") {
        TurningHolderCode bad = c;
        bad.customStyle = 170.0; // with an 80 degree corner
        CHECK_FALSE(validate(bad).empty());
    }
    SECTION("a designation cannot say them") {
        CHECK_FALSE(parseDesignation("PCXOR 2525X 12"));
        const auto back = parseDesignation("PCXOR 2020X 12", &c);
        REQUIRE(back);
        CHECK(back->height == 20.0);
        CHECK(back->customStyle == 70.0);
        CHECK(back->customClearance == 11.0);
        CHECK(back->customLength == 175.0);
    }
}

TEST_CASE("which holders fit an insert", "[tooling][turning]") {
    using enum HolderMismatch;
    Tool tool = makeDefaultTool(ToolType::TurningInsert, 1); // C, 12.7, 0 degrees, not set
    const Holder fits = makeTurningHolder(*parseDesignation("PCLNR 2525M 12"));
    using Misses = std::vector<HolderMismatch>;
    SECTION("a new tool fits a new holder") {
        CHECK(holderMismatches(tool, makeTurningHolder(TurningHolderCode{})).empty());
    }
    SECTION("shape, size and clearance have to match") {
        CHECK(holderMismatches(tool, fits).empty());
        CHECK(holderMismatches(tool, makeTurningHolder(*parseDesignation("PDJNR 2525M 15"))) ==
              Misses{Shape});
        CHECK(holderMismatches(tool, makeTurningHolder(*parseDesignation("PCLNR 2525M 09"))) ==
              Misses{Size});
        CHECK(holderMismatches(tool, makeTurningHolder(*parseDesignation("PCLCR 2525M 12"))) ==
              Misses{Clearance});
        CHECK(holderMismatches(tool, makeTurningHolder(*parseDesignation("SDJCR 1212F 07"))) ==
              Misses{Shape, Size, Clearance});
    }
    SECTION("the hand does not: the tool takes it from the holder") {
        tool.insert.hand = "R";
        CHECK(
            holderMismatches(tool, makeTurningHolder(*parseDesignation("PCLNL 2525M 12"))).empty());
    }
    SECTION("locking, when it is set") {
        const auto with = [&](InsertLocking locking, const char* holder) {
            tool.insert.locking = locking;
            return holderMismatches(tool, makeTurningHolder(*parseDesignation(holder)));
        };
        for (const char* holder :
             {"PCLNR 2525M 12", "CCLNR 2525M 12", "MCLNR 2525M 12", "SCLNR 2525M 12"}) {
            CHECK(with(InsertLocking::NotSet, holder).empty());
        }
        CHECK(with(InsertLocking::CylindricalHole, "PCLNR 2525M 12").empty());
        CHECK(with(InsertLocking::CylindricalHole, "MCLNR 2525M 12").empty());
        CHECK(with(InsertLocking::CylindricalHole, "CCLNR 2525M 12") == Misses{Locking});
        CHECK(with(InsertLocking::CylindricalHole, "SCLNR 2525M 12") == Misses{Locking});
        CHECK(with(InsertLocking::NoHole, "CCLNR 2525M 12").empty());
        CHECK(with(InsertLocking::NoHole, "PCLNR 2525M 12") == Misses{Locking});
        CHECK(with(InsertLocking::CountersunkHole, "SCLNR 2525M 12").empty());
        CHECK(with(InsertLocking::CountersunkHole, "MCLNR 2525M 12") == Misses{Locking});
    }
    SECTION("a holder without a designation only has a kind") {
        const Holder plain = makeTurningStick({});
        tool.insert.shape = "D";
        CHECK(holderMismatches(tool, plain).empty());
        const Tool groove = makeDefaultTool(ToolType::GroovingInsert, 2);
        CHECK(holderMismatches(groove, plain).empty());
        CHECK(holderMismatches(groove, fits) == Misses{Kind});
        Holder milling = plain;
        milling.kind = HolderKind::Milling;
        CHECK(holderMismatches(tool, milling) == Misses{Kind});
    }
}

TEST_CASE("what shows of a turning tool's holder", "[tooling][turning]") {
    Tool tool = makeDefaultTool(ToolType::TurningInsert, 1);
    SECTION("no holder shows nothing") {
        CHECK(turningToolParts(tool, nullptr, TurningSide::Outside).empty());
    }
    SECTION("a shank: its top at the tip's height, with the head around the insert") {
        const Holder h = makeTurningHolder(*parseDesignation("PCLNR 2525M 12"));
        const auto parts = turningToolParts(tool, &h, TurningSide::Outside);
        REQUIRE(parts.size() == 2);
        CHECK(parts[1].low == Catch::Approx(-25.0));
        CHECK(parts[1].high == Catch::Approx(0.0));
        CHECK_FALSE(parts[1].round);
        // The back of the shank is f from the tip.
        double back = -1e9;
        for (const Vec2& p : parts[1].outline.discretize(0.01)) {
            back = std::max(back, p.x());
        }
        CHECK(back == Catch::Approx(h.iso->f));
        // Mirrored away from the chuck.
        const auto away = turningToolParts(tool, &h, TurningSide::Outside, true);
        double front = 1e9;
        for (const Vec2& p : away[1].outline.discretize(0.01)) {
            front = std::min(front, p.x());
        }
        CHECK(front == Catch::Approx(-h.iso->f));
    }
    SECTION("a boring bar: round, on the tip's height") {
        const Holder h = makeTurningHolder(*parseDesignation("S25T-PCLNR 12"));
        const auto parts = turningToolParts(tool, &h, TurningSide::Inside);
        REQUIRE(parts.size() == 2);
        CHECK(parts[1].round);
        CHECK(parts[1].low == Catch::Approx(-12.5));
        CHECK(parts[1].high == Catch::Approx(12.5));
    }
    SECTION("a plain holder") {
        const Holder shank = makeTurningStick({false, 20.0, 25.0, 125.0});
        const auto s = turningToolParts(tool, &shank, TurningSide::Outside);
        REQUIRE(s.size() == 1);
        CHECK(s[0].low == Catch::Approx(-20.0));
        CHECK_FALSE(s[0].round);
        const Holder bar = makeTurningStick({true, 16.0, 11.0, 150.0});
        const auto b = turningToolParts(tool, &bar, TurningSide::Inside);
        REQUIRE(b.size() == 1);
        CHECK(b[0].round);
    }
    SECTION("a solid bar has its head and its round shank") {
        const Tool solid = makeSolidBoringBar(1);
        const auto parts = turningToolParts(solid, nullptr, TurningSide::Inside);
        REQUIRE(parts.size() == 2);
        CHECK_FALSE(parts[0].round);
        CHECK(parts[1].round);
    }
    SECTION("an outline of its own stands over the insert, in bands") {
        Holder h;
        h.kind = HolderKind::Turning;
        h.profile = geom2d::Polyline::fromPoints(
            {Vec2(0, 0), Vec2(10, 0), Vec2(10, 30), Vec2(15, 30), Vec2(15, 60), Vec2(0, 60)},
            false);
        const auto parts = turningToolParts(tool, &h, TurningSide::Outside);
        REQUIRE(parts.size() == 2);
        CHECK(parts[0].high == Catch::Approx(10.0));
        CHECK(parts[1].high == Catch::Approx(15.0));
        // A milling holder's outline says nothing about a lathe.
        h.kind = HolderKind::Milling;
        CHECK(turningToolParts(tool, &h, TurningSide::Outside).empty());
    }
}

TEST_CASE("ISO turning holders", "[tooling][turning]") {
    // Designations read and write, in the catalogues' forms.
    const auto shank = parseDesignation("PCLNR 2525M 12");
    REQUIRE(shank);
    CHECK_FALSE(shank->boringBar);
    CHECK(shank->clamping == 'P');
    CHECK(shank->insertShape == 'C');
    CHECK(shank->style == 'L');
    CHECK(shank->hand == 'R');
    CHECK(shank->height == 25.0);
    CHECK(shank->width == 25.0);
    CHECK(shank->length == 'M');
    CHECK(shank->insertSize == 12);
    CHECK(shank->f == 32.0);
    CHECK(designation(*shank) == "PCLNR 2525M 12");
    CHECK(parseDesignation("pclnr2525m12-x") == shank);
    CHECK(validate(*shank).empty());
    CHECK(holderLength(*shank) == 150.0);

    const auto bar = parseDesignation("S25T-PCLNR 12");
    REQUIRE(bar);
    CHECK(bar->boringBar);
    CHECK(bar->barType == 'S');
    CHECK(bar->height == 25.0);
    CHECK(bar->length == 'T');
    CHECK(bar->f == 17.0);
    CHECK(designation(*bar) == "S25T-PCLNR 12");
    CHECK(parseDesignation("S25TPCLNR12") == bar);
    CHECK(holderLength(*bar) == 300.0);
    CHECK(smallestBore(*bar) == 29.5);
    CHECK(holderSide(*bar) == TurningSide::Inside);

    CHECK_FALSE(parseDesignation(""));
    CHECK_FALSE(parseDesignation("PCLNR"));
    CHECK_FALSE(parseDesignation("XCLNR 2525M 12")); // no such clamping
    CHECK_FALSE(parseDesignation("PCQNR 2525M 12")); // no such style

    // Sizes: a side's whole millimetres to the inscribed circle, as the catalogues give them.
    auto icOf = [](const char* code) {
        return inscribedCircle(*parseDesignation(code));
    };
    CHECK(icOf("PCLNR 2525M 12") == 12.7);
    CHECK(icOf("PCLNR 2525M 09") == 9.525);
    CHECK(icOf("PDJNR 2525M 15") == 12.7);
    CHECK(icOf("PDJNR 2525M 11") == 9.525);
    CHECK(icOf("PTGNR 2525M 16") == 9.525);
    CHECK(icOf("PSBNR 2525M 12") == 12.7);
    CHECK(icOf("PVJNR 2525M 16") == 9.525);
    CHECK(icOf("PWLNR 2525M 08") == 12.7);
    CHECK(icOf("PWLNR 2525M 06") == 9.525);
    CHECK(icOf("SRDCN 2525M 12") == 12.0);

    // A neutral holder has the tip on the shank's middle.
    CHECK(parseDesignation("SVVBN 2525M 16")->f == 12.5);

    // The style places the insert: 95 degrees, an 80 degree corner, 5 degrees of end relief.
    Tool t = makeDefaultTool(ToolType::TurningInsert, 1);
    Holder h = makeTurningHolder(*shank);
    CHECK(h.kind == HolderKind::Turning);
    CHECK(h.name == "PCLNR 2525M 12");
    CHECK(h.validate().empty());
    CHECK(h.length() == 150.0);
    CHECK(h.maxRadius() == 12.5);
    CHECK(holds(h, ToolType::TurningInsert));
    CHECK_FALSE(holds(h, ToolType::GroovingInsert));
    fitToHolder(t, h);
    CHECK(t.insert.shape == "C");
    CHECK(t.insert.inscribedCircle == 12.7);
    CHECK(t.insert.hand == "R");
    CHECK(t.insert.orientation == 3);
    CHECK(radToDeg(t.insert.cuttingEdgeAngle) == Approx(95.0));
    TurningShape s = turningShape(t);
    CHECK(radToDeg(s.leadAngle) == Approx(85.0));
    CHECK(radToDeg(s.trailingAngle) == Approx(5.0));

    // 93 degrees with a 55 degree corner; 45 degrees with a square; end cutting at 93 degrees
    // measures from the facing feed: the end edge 3 degrees off it.
    auto placed = [&](const char* code) {
        Tool any = makeDefaultTool(ToolType::TurningInsert, 1);
        fitToHolder(any, makeTurningHolder(*parseDesignation(code)));
        const TurningShape shape = turningShape(any);
        return std::pair(radToDeg(shape.leadAngle), radToDeg(shape.trailingAngle));
    };
    CHECK(placed("PDJNR 2525M 15").first == Approx(87.0));
    CHECK(placed("PDJNR 2525M 15").second == Approx(32.0));
    CHECK(placed("PSSNR 2525M 12").first == Approx(135.0));
    CHECK(placed("PSSNR 2525M 12").second == Approx(45.0));
    CHECK(placed("PDUNR 2525M 15").first == Approx(58.0));
    CHECK(placed("PDUNR 2525M 15").second == Approx(3.0));
    CHECK(placed("PTGNR 2525M 16").first == Approx(90.0));
    CHECK(placed("PTGNR 2525M 16").second == Approx(30.0));

    // Combinations whose other edge would dig in are refused.
    CHECK_FALSE(validate(*parseDesignation("PSLNR 2525M 12")).empty()); // 90 corner at 95
    CHECK_FALSE(validate(*parseDesignation("PSUNR 2525M 12")).empty()); // end-cutting square
    CHECK(validate(*parseDesignation("SRDCN 2525M 12")).empty());       // round fits anything
    Tool wrong = makeDefaultTool(ToolType::TurningInsert, 1);
    wrong.insert.shape = "H"; // a 120 degree corner: at 75 degrees both edges dig in
    wrong.insert.cuttingEdgeAngle = degToRad(75.0);
    CHECK_THROWS(turningShape(wrong));
    CHECK_FALSE(wrong.validate().empty());

    // The holder's outline: the shank's back f behind the tip, the insert inside the head.
    const TurningHolderOutline out = turningHolderOutline(*shank, s.outline);
    double shankLo = 1e9, shankHi = -1e9, shankTop = -1e9;
    for (const auto& v : out.shank.vertices()) {
        shankLo = std::min(shankLo, v.p.x());
        shankHi = std::max(shankHi, v.p.x());
        shankTop = std::max(shankTop, v.p.y());
    }
    CHECK(shankHi == 32.0);
    CHECK(shankLo == 7.0);
    CHECK(shankTop == 150.0);
    CHECK(out.head.isClosed());
    CHECK(out.head.isCounterClockwise());
    CHECK(out.head.area() >= s.outline.area());
    // The bar's axis f below the tip, the insert pointing up out of it.
    Tool boring = makeDefaultTool(ToolType::TurningInsert, 1);
    fitToHolder(boring, makeTurningHolder(*bar));
    CHECK(boring.insert.orientation == 2);
    const TurningHolderOutline barOut =
        turningHolderOutline(*bar, turningShape(boring, TurningSide::Inside).outline);
    double barLo = 1e9, barHi = -1e9;
    for (const auto& v : barOut.shank.vertices()) {
        barLo = std::min(barLo, v.p.y());
        barHi = std::max(barHi, v.p.y());
    }
    CHECK(barHi == -17.0 + 12.5);
    CHECK(barLo == -17.0 - 12.5);

    // In a library: a tool whose insert is not the holder's is reported.
    ToolLibrary lib;
    lib.holders.push_back(h);
    Tool held = makeDefaultTool(ToolType::TurningInsert, 1);
    held.holderId = h.id;
    fitToHolder(held, h);
    lib.tools.push_back(held);
    CHECK(lib.validate().empty());
    lib.tools[0].insert.shape = "D";
    CHECK_FALSE(lib.validate().empty());
}

TEST_CASE("a hole tool's silhouette on the spindle axis", "[tooling][turning]") {
    auto outline = [](const Tool& t) {
        const geom2d::Polyline p = holeToolOutline(t);
        CHECK(p.isCounterClockwise());
        return p;
    };
    Tool drill = makeDefaultTool(ToolType::Drill, 1);
    drill.mill.diameter = 10.0;
    drill.mill.fluteLength = 40.0;
    drill.mill.tipAngle = degToRad(118.0);
    const double tip = 5.0 / std::tan(degToRad(59.0));
    // The tip at the origin; both sides of the axis.
    const auto points = outline(drill).discretize(1e-3);
    REQUIRE(points.size() == 5);
    CHECK(points[0].norm() == Approx(0.0).margin(1e-12));
    CHECK(points[1].x() == Approx(tip));
    CHECK(points[1].y() == Approx(-5.0));
    CHECK(points[2].x() == Approx(40.0));
    CHECK(points[3].y() == Approx(5.0));
    CHECK(outline(drill).area() == Approx(10.0 * 40.0 - 10.0 * tip / 2.0));

    Tool reamer = makeDefaultTool(ToolType::Reamer, 2);
    reamer.mill.diameter = 8.0;
    reamer.mill.fluteLength = 30.0;
    CHECK(outline(reamer).area() == Approx(8.0 * 30.0)); // flat
    CHECK(outline(reamer).discretize(1e-3).size() == 4);

    Tool centre = makeDefaultTool(ToolType::CenterDrill, 3);
    centre.mill.diameter = 6.3;
    centre.mill.tipDiameter = 2.5;
    centre.mill.tipLength = 3.0;
    centre.mill.fluteLength = 12.0;
    centre.mill.tipAngle = degToRad(118.0);
    // The pilot's point, then straight to the body: no notch at the pilot's shoulder.
    const auto bridged = outline(centre).discretize(1e-3);
    CHECK(bridged.size() == 7);
    CHECK(bridged[1].y() == Approx(-1.25));
    CHECK(bridged[2].y() == Approx(-3.15));
}

TEST_CASE("turning insert outlines", "[tooling][turning]") {
    Tool c = makeDefaultTool(ToolType::TurningInsert, 1);
    c.insert.shape = "C";
    c.insert.noseRadius = 0.8;
    const TurningShape od = turningShape(c);
    const auto pts = od.outline.discretize(1e-4);
    double minZ = 1e9, minR = 1e9;
    for (const Vec2& p : pts) {
        minZ = std::min(minZ, p.x());
        minR = std::min(minR, p.y());
    }
    // The imaginary tip is the programmed point: the nose touches z = 0 and r = 0.
    CHECK(std::abs(minZ) < 1e-4); // up to the discretization chord
    CHECK(std::abs(minR) < 1e-4);
    CHECK(od.outline.isCounterClockwise());
    CHECK(std::abs(od.trailingAngle - 5.0 * kPi / 180.0) < 1e-12);
    CHECK(std::abs(od.leadAngle - 85.0 * kPi / 180.0) < 1e-12);
    // Inside tools are mirrored: the nose touches r = 0 from below.
    const TurningShape id = turningShape(c, TurningSide::Inside);
    double maxR = -1e9;
    for (const Vec2& p : id.outline.discretize(1e-4)) {
        maxR = std::max(maxR, p.y());
    }
    CHECK(std::abs(maxR) < 1e-4);
    CHECK(id.outline.isCounterClockwise());
    // The outline is the shape itself, its circle inscribed, closing on the leading edge.
    for (const InsertShape& s : insertShapes()) {
        INFO(s.letter);
        Tool any = makeDefaultTool(ToolType::TurningInsert, 1);
        any.insert.shape = std::string(1, s.letter);
        any.insert.noseRadius = 0.0;
        any.insert.inscribedCircle = 12.7;
        const TurningShape shape = turningShape(any);
        CHECK(shape.outline.isCounterClockwise());
        if (s.includedAngle > 0.0) {
            CHECK(std::abs(radToDeg(shape.leadAngle - shape.trailingAngle) - s.includedAngle) <
                  1e-9);
        }
    }
    {
        // A triangle: three corners and side ic * sqrt(3), so an area of 3 sqrt(3) ic^2 / 4.
        Tool t = makeDefaultTool(ToolType::TurningInsert, 1);
        t.insert.shape = "T";
        t.insert.noseRadius = 0.0;
        t.insert.inscribedCircle = 10.0;
        const TurningShape shape = turningShape(t);
        CHECK(shape.outline.vertices().size() == 3);
        CHECK(std::abs(std::abs(shape.outline.signedArea()) - 3.0 * std::sqrt(3.0) * 25.0) < 1e-6);
        // A trigon: six equal sides of 8.69 mm on a 12.7 mm circle, as a WNMG 08.
        t.insert.shape = "W";
        t.insert.inscribedCircle = 12.7;
        const geom2d::Polyline trigon = turningShape(t).outline;
        const auto& w = trigon.vertices();
        REQUIRE(w.size() == 6);
        CHECK(std::abs((w[1].p - w[0].p).norm() - 8.69) < 0.01);
    }
    CHECK(findInsertShape("cnmg")->letter == 'C');
    CHECK(findInsertShape("Q") == nullptr);

    // Grooving: a rectangle of the insert width.
    Tool g = makeDefaultTool(ToolType::GroovingInsert, 2);
    g.insert.width = 3.0;
    g.insert.noseRadius = 0.2;
    const TurningShape groove = turningShape(g);
    CHECK(groove.width == 3.0);
    CHECK(std::abs(groove.outline.bounds().max.x() - 3.0) < 1e-9);
    CHECK_THROWS(turningShape(makeFlatEndMill(1, 6, 20, 50)));
}

TEST_CASE("a spindle probe: stylus, body and trigger offset", "[tooling][probe]") {
    Tool probe = makeDefaultTool(ToolType::Probe, 30, 4.0);
    CHECK(probe.validate().empty());
    CHECK(probe.presets.empty()); // it does not cut: its feeds are its operations'
    CHECK(holderKindFor(ToolType::Probe) == HolderKind::Milling);
    probe.mill.triggerOffset = 0.015;
    CHECK_THAT(effectiveStylusRadius(probe.mill), WithinAbs(1.985, 1e-12));

    // The profile: a ball on the stem up to the stylus length, then the body.
    const ToolProfile profile = toolProfile(probe);
    CHECK_THAT(profile.cuttingLength, WithinAbs(probe.mill.fluteLength, 1e-12));
    CHECK_THAT(profileRadiusAt(profile, 2.0), WithinAbs(2.0, 1e-3));
    CHECK_THAT(profileRadiusAt(profile, probe.mill.fluteLength / 2.0),
               WithinAbs(probe.mill.neckDiameter / 2.0, 1e-9));
    CHECK_THAT(profileRadiusAt(profile, probe.mill.overallLength - 1.0),
               WithinAbs(probe.mill.shankDiameter / 2.0, 1e-9));

    Tool bad = probe;
    bad.mill.neckDiameter = 0.0;
    CHECK_FALSE(bad.validate().empty());
    bad = probe;
    bad.mill.triggerOffset = 1.5; // more than a quarter of the ball
    CHECK_FALSE(bad.validate().empty());
    bad = probe;
    bad.mill.triggerOffset = -0.01;
    CHECK_FALSE(bad.validate().empty());
}

TEST_CASE("a bar puller: the bar it takes, and a body over it", "[tooling][barpull]") {
    Tool t = makeDefaultTool(ToolType::BarPuller, 7, 40.0);
    CHECK(t.validate().empty());
    CHECK_FALSE(isTurningTool(t.type));
    CHECK(toolFamily(t.type) == ToolFamily::Turning); // listed with the lathe's tools
    CHECK_THAT(t.mill.tipDiameter, WithinAbs(40.0, 1e-12));
    CHECK(t.mill.diameter > t.mill.tipDiameter);
    // Its default preset feeds onto the bar and pulls; it has no cutting speed.
    REQUIRE(t.presets.size() == 1);
    CHECK(t.presets[0].plungeFeed > 0.0);
    CHECK(t.presets[0].pullFeed > 0.0);
    CHECK(t.presets[0].surfaceSpeed == 0.0);
    // Its outline has the bar's bore in it, as deep as its jaws.
    const ToolProfile profile = toolProfile(t);
    CHECK_THAT(profile.outline.vertices().front().p.y(), WithinAbs(t.mill.fluteLength, 1e-12));
    t.mill.tipDiameter = t.mill.diameter;
    CHECK_FALSE(t.validate().empty());
}
