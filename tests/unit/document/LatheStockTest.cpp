// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// A lathe setup taking its stock from an earlier setup (DL-0044), on flip_project.ocam: a spool
// turned from its front, then flipped and turned from its back.

#include "FixtureSpecs.hpp"

#include <ocam/document/Generate.hpp>
#include <ocam/document/PostSetup.hpp>
#include <ocam/document/Simulate.hpp>
#include <ocam/document/Workholding.hpp>
#include <ocam/foundation/Error.hpp>
#include <ocam/geom2d/Clip.hpp>
#include <ocam/sim/Compare.hpp>
#include <ocam/tooling/ToolLibrary.hpp>
#include <ocam/tooling/TurningHolder.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <filesystem>
#include <numbers>

using namespace ocam;
using namespace ocam::document;
using namespace ocam::fixtures::spool;
using Catch::Approx;
using Catch::Matchers::ContainsSubstring;

namespace {

struct Flip {
    Flip() : doc(Document::open(std::filesystem::path(OCAM_FIXTURE_DIR) / "flip_project.ocam")) {}
    const Setup& first() const { return doc.project().setups.front(); }
    const Setup& second() const { return doc.project().setups.back(); }
    SetupSimulation simulate(const Setup& setup) {
        SimulationOptions options;
        return simulateSetup(doc, setup.id, options, cancel, progress);
    }
    Document doc;
    CancelToken cancel;
    NullProgress progress;
};

} // namespace

TEST_CASE("a flipped lathe setup starts from what the first side left", "[document][lathe][flip]") {
    Flip f;
    const SetupSimulation one = f.simulate(f.first());
    REQUIRE_FALSE(one.hasErrors());
    const SetupSimulation two = f.simulate(f.second());
    for (const auto& op : two.operations) {
        INFO(op.name << ": " << op.error);
        CHECK(op.error.empty());
    }
    REQUIRE(two.latheInitial);
    SECTION("the same stock, mirrored") {
        REQUIRE(one.operations.back().latheAfter);
        CHECK(two.latheInitial->volume() ==
              Approx(one.operations.back().latheAfter->volume()).epsilon(1e-4));
        geom2d::Aabb2 before, after;
        for (const auto& r : one.operations.back().latheAfter->profile()) {
            before.extend(r.bounds());
        }
        for (const auto& r : two.latheInitial->profile()) {
            after.extend(r.bounds());
        }
        // First side: the origin at the part's front, the bar's extra 3 mm behind its back. Now
        // that extra is in front.
        CHECK(before.min.x() == Approx(-kLength - 3.0).margin(1e-3));
        CHECK(after.max.x() == Approx(3.0).margin(1e-3));
        CHECK(after.max.y() == Approx(before.max.y()).margin(1e-3));
    }
    SECTION("the second side finishes the part") {
        REQUIRE(two.comparison);
        CHECK_FALSE(two.comparison->hasGouges());
        CHECK(two.comparison->maxExcess < 0.25); // the nose radius in the corners at most
        CHECK(two.operations.back().latheAfter->volume() == Approx(kVolume).epsilon(0.01));
    }
    SECTION("its passes start at the stock that is left and stop where it ends") {
        const toolpath::Toolpath face =
            generateToolpath(f.doc, f.second(), f.second().operations[0], f.cancel, f.progress);
        // The first pass takes off part of the extra 3 mm.
        double deepest = 0.0, highest = -1e9;
        for (const auto& m : face.moves) {
            if (m.kind == toolpath::MoveKind::Linear) {
                deepest = std::min(deepest, m.end.z());
                highest = std::max(highest, m.end.z());
            }
        }
        CHECK(highest <= 3.0);
        CHECK(deepest == Approx(0.0).margin(0.5));
        const toolpath::Toolpath rough =
            generateToolpath(f.doc, f.second(), f.second().operations[1], f.cancel, f.progress);
        for (const auto& m : rough.moves) {
            if (m.kind == toolpath::MoveKind::Linear) {
                // Up to the flange, not along the first side's finished end.
                INFO("Z" << m.end.z() << " X" << m.end.x());
                CHECK(m.end.z() > -(kEndLength + kFlangeLength) - 2.5);
            }
        }
    }
}

TEST_CASE("what cannot be carried to a lathe setup is refused", "[document][lathe][flip]") {
    Flip f;
    const auto refused = [&](const char* text) {
        CHECK_THROWS_WITH(latheGeometry(f.doc, f.second(), f.cancel), ContainsSubstring(text));
        CHECK_THROWS_WITH(f.simulate(f.second()), ContainsSubstring(text));
        CHECK_THROWS_WITH(
            generateToolpath(f.doc, f.second(), f.second().operations[0], f.cancel, f.progress),
            ContainsSubstring(text));
    };
    SECTION("stock a mill cut") {
        f.doc.modify("mill", [&](Project& p) { p.setups.front().machine = "generic-3axis-vmc"; });
        refused("which is not on a lathe");
    }
    SECTION("a mill further up the chain") {
        const Uuid firstId = f.first().id;
        f.doc.modify("chain", [&](Project& p) {
            Setup middle = p.setups.back();
            middle.id = Uuid::generate();
            middle.name = "Middle";
            middle.wcs = p.setups.front().wcs;
            middle.operations.clear();
            p.setups.front().machine = "generic-3axis-vmc";
            p.setups.insert(p.setups.begin() + 1, middle);
            p.setups.back().stock.sourceSetup = middle.id;
            CHECK(p.setups[1].stock.sourceSetup == firstId);
        });
        refused("'First side', which is not on a lathe");
    }
    SECTION("another spindle axis") {
        f.doc.modify("off axis", [&](Project& p) { p.setups.back().wcs.origin.x() = 5.0; });
        refused("spindle axis");
    }
    SECTION("no earlier setup") {
        f.doc.modify("none", [&](Project& p) { p.setups.back().stock.sourceSetup.reset(); });
        CHECK_THROWS_WITH(latheGeometry(f.doc, f.second(), f.cancel),
                          ContainsSubstring("choose one"));
    }
}

TEST_CASE("the stock carried over a lathe setup", "[document][lathe][flip]") {
    Flip f;
    SECTION("an earlier operation that fails is warned of") {
        f.doc.modify("no tool", [&](Project& p) { p.setups.front().operations[1].toolId.reset(); });
        const LatheGeometry g = latheGeometry(f.doc, f.second(), f.cancel);
        REQUIRE(g.warnings.size() == 1);
        CHECK_THAT(g.warnings.front(), ContainsSubstring("leaves out 'turn_rough'"));
        const toolpath::Toolpath tp =
            generateToolpath(f.doc, f.second(), f.second().operations[0], f.cancel, f.progress);
        CHECK_THAT(tp.warnings.front(), ContainsSubstring("leaves out 'turn_rough'"));
        CHECK(f.simulate(f.second()).warnings.front().find("leaves out") != std::string::npos);
    }
    SECTION("a change in the first side reaches the second side's toolpaths") {
        const std::string key = toolpathInputKey(f.doc, f.second(), f.second().operations[1]);
        f.doc.modify("stock", [&](Project& p) {
            p.setups.front().operations[1].presetOverrides.axialDepth = 1.0;
        });
        CHECK(toolpathInputKey(f.doc, f.second(), f.second().operations[1]) != key);
    }
    SECTION("the same way round, it is the same stock") {
        const auto left = f.simulate(f.first()).operations.back().latheAfter->volume();
        f.doc.modify("same way", [&](Project& p) {
            Setup& two = p.setups.back();
            two.wcs = p.setups.front().wcs;
            two.operations.clear();
        });
        const LatheGeometry g = latheGeometry(f.doc, f.second(), f.cancel);
        CHECK(sim::revolvedVolume(g.stock) == Approx(left).epsilon(1e-4));
    }
    SECTION("what parting off leaves behind stays behind") {
        // Part off in the first side, behind the spool: only the spool's piece goes on.
        tooling::Tool parting = tooling::makeDefaultTool(tooling::ToolType::PartingInsert, 5);
        parting.insert.width = 3.0;
        parting.insert.noseRadius = 0.1;
        parting.presets = {tooling::defaultPreset(parting)};
        f.doc.modify("part off", [&](Project& p) {
            p.tools.tools.push_back(parting);
            p.setups.front().stock.offsetBottom = 10.0;
            Operation off;
            off.name = "Part off";
            off.type = "turn_partoff";
            off.toolId = parting.id;
            off.presetId = parting.presets.front().id;
            off.parameters = Json::object();
            p.setups.front().operations.push_back(off);
        });
        const SetupSimulation one = f.simulate(f.first());
        REQUIRE_FALSE(one.hasErrors());
        const auto& left = one.operations.back().latheAfter->profile();
        REQUIRE(left.size() == 2);
        const LatheGeometry g = latheGeometry(f.doc, f.second(), f.cancel);
        REQUIRE(g.stock.size() == 1);
        CHECK(sim::revolvedVolume(g.stock) < one.operations.back().latheAfter->volume());
        CHECK(g.stock.front().bounds().min.x() == Approx(-kLength).margin(0.5));
    }
    SECTION("a chuck closes on the diameter the first side left in its jaws") {
        Document chucked =
            Document::open(std::filesystem::path(OCAM_FIXTURE_DIR) / "chuck_project.ocam");
        const Setup firstSetup = chucked.project().setups.front();
        chucked.modify("second side", [&](Project& p) {
            Setup two = firstSetup;
            two.id = Uuid::generate();
            two.name = "Second side";
            two.operations.clear();
            two.stock.kind = StockKind::FromSetup;
            two.stock.sourceSetup = firstSetup.id;
            p.setups.push_back(two);
        });
        const auto bar = setupWorkholdingPose(chucked, chucked.project().setups.front());
        REQUIRE(bar);
        // The same way round: the bar's back end, still whole, is in the jaws. Flipped, its turned
        // front end is.
        chucked.modify("flip", [&](Project& p) {
            Setup& two = p.setups.back();
            const Vec3 back = two.wcs.origin - 80.0 * two.wcs.zAxis;
            two.wcs.origin = back;
            two.wcs.zAxis = -two.wcs.zAxis;
        });
        const auto flipped = setupWorkholdingPose(chucked, chucked.project().setups.back());
        REQUIRE(flipped);
        CHECK(flipped->opening == Approx(fixtures::shaft::kMinorDiameter).margin(0.01));
        CHECK(bar->opening > flipped->opening + 5.0);
    }
}

TEST_CASE("grooves outside, in a bore and in a face", "[document][lathe][groove]") {
    using namespace fixtures::grooved;
    CancelToken cancel;
    NullProgress progress;
    Document doc = Document::open(std::filesystem::path(OCAM_FIXTURE_DIR) / "grooved_project.ocam");
    const Setup& setup = doc.project().setups.front();
    SimulationOptions options;
    const SetupSimulation sim = simulateSetup(doc, setup.id, options, cancel, progress);
    REQUIRE(sim.operations.size() == 6);
    for (const auto& op : sim.operations) {
        INFO(op.name << ": " << op.error);
        CHECK(op.error.empty());
    }
    const auto feeds = [](const toolpath::Toolpath& tp) {
        std::vector<Vec3> ends;
        for (const auto& m : tp.moves) {
            if (m.kind == toolpath::MoveKind::Linear) {
                ends.push_back(m.end);
            }
        }
        return ends;
    };
    SECTION("each plunges to its floor, between its walls") {
        const auto outside = feeds(sim.operations[3].toolpath);
        REQUIRE_FALSE(outside.empty());
        for (const Vec3& e : outside) {
            CHECK(e.x() == Approx(kOutsideFloor / 2.0));
            CHECK(e.z() >= kOutsideLow - fixtures::grooved::kLength - 1e-9);
            CHECK(e.z() + 2.0 <= kOutsideHigh - fixtures::grooved::kLength + 1e-9);
        }
        const auto inside = feeds(sim.operations[4].toolpath);
        REQUIRE_FALSE(inside.empty());
        for (const Vec3& e : inside) {
            CHECK(e.x() == Approx(kInsideRoof / 2.0));
            CHECK(e.z() >= kInsideLow - fixtures::grooved::kLength - 1e-9);
            CHECK(e.z() + 2.0 <= kInsideHigh - fixtures::grooved::kLength + 1e-9);
        }
        CHECK(sim.operations[4].toolpath.tipQuadrant == 2);
        const auto face = feeds(sim.operations[5].toolpath);
        REQUIRE_FALSE(face.empty());
        for (const Vec3& e : face) {
            CHECK(e.z() == Approx(-kFaceDepth));
            CHECK(e.x() >= kFaceInner / 2.0 - 1e-9);
            CHECK(e.x() + 2.0 <= kFaceOuter / 2.0 + 1e-9);
        }
        CHECK(sim.operations[5].toolpath.tipQuadrant == 8);
    }
    SECTION("the part comes out to size, with nothing gouged") {
        REQUIRE(sim.comparison);
        CHECK_FALSE(sim.comparison->hasGouges());
        CHECK(sim.comparison->maxExcess < 0.5);
    }
    SECTION("each removes material") {
        for (std::size_t k = 3; k < 6; ++k) {
            CHECK(sim.operations[k].removedVolume > 10.0);
        }
    }
    SECTION("the tip of a face tool stands along Z, across its width") {
        const auto box = sim.operations[5].insert.bounds();
        CHECK(box.min.y() == Approx(0.0).margin(1e-6));
        CHECK(box.max.y() == Approx(2.0));
        CHECK(box.max.x() > 5.0);
    }
}

TEST_CASE("an inside groove needs a bore, and only a grooving insert cuts in a face",
          "[document][lathe][groove]") {
    using namespace fixtures::grooved;
    CancelToken cancel;
    NullProgress progress;
    Document doc = Document::open(std::filesystem::path(OCAM_FIXTURE_DIR) / "grooved_project.ocam");
    doc.modify("solid bar", [&](Project& p) { p.setups.front().stock.kind = StockKind::Cylinder; });
    const Setup& setup = doc.project().setups.front();
    CHECK_THROWS_WITH(generateToolpath(doc, setup, setup.operations[4], cancel, progress),
                      ContainsSubstring("needs a bore"));
    tooling::Tool insert = tooling::makeDefaultTool(tooling::ToolType::TurningInsert, 1);
    CHECK_THROWS(tooling::turningShape(insert, tooling::TurningSide::Face));
}

TEST_CASE("roughing across Z and turning a Z range", "[document][lathe][range]") {
    CancelToken cancel;
    NullProgress progress;
    Document doc = Document::open(std::filesystem::path(OCAM_FIXTURE_DIR) / "ranges_project.ocam");
    const Uuid id = doc.project().setups.front().id;
    const auto setup = [&]() -> const Setup& {
        return *doc.project().findSetup(id);
    };
    const auto generate = [&](std::size_t k) {
        return generateToolpath(doc, setup(), setup().operations[k], cancel, progress);
    };
    SECTION("across: radial passes stepping back by the depth of cut, to the part and no further") {
        const toolpath::Toolpath tp = generate(1);
        std::vector<double> levels;
        Vec3 pos = tp.start;
        for (const auto& m : tp.moves) {
            if (m.kind == toolpath::MoveKind::Linear && m.feedClass == toolpath::FeedClass::Cut &&
                std::abs(m.end.z() - pos.z()) < 1e-9 && m.end.x() < pos.x()) {
                levels.push_back(m.end.z());
                CHECK(m.end.x() >= kEndDiameter / 2.0);
            }
            pos = m.end;
        }
        REQUIRE(levels.size() >= 12);
        for (std::size_t k = 1; k < levels.size(); ++k) {
            CHECK(levels[k] < levels[k - 1]);
            CHECK(levels[k - 1] - levels[k] <= 2.0 + 1e-9);
        }
    }
    SECTION("the moves stay within the range, and what lies behind it is left alone") {
        SimulationOptions options;
        const SetupSimulation sim = simulateSetup(doc, id, options, cancel, progress);
        REQUIRE(sim.operations.size() == 3);
        for (const auto& op : sim.operations) {
            INFO(op.name << ": " << op.error);
            CHECK(op.error.empty());
        }
        for (std::size_t k : {std::size_t{1}, std::size_t{2}}) {
            for (const auto& m : sim.operations[k].toolpath.moves) {
                if (m.kind == toolpath::MoveKind::Linear) {
                    CHECK(m.end.z() > -kEndLength - 1.0);
                }
            }
        }
        // The flange and the far end still stand at the bar's radius, the near end is finished.
        const auto& left = sim.operations.back().latheAfter->profile();
        geom2d::Aabb2 behind;
        for (const auto& r : geom2d::intersect(
                 left,
                 {{geom2d::Polyline::rectangle(Vec2(-kLength, 0.0), Vec2(-kEndLength - 2.0, 100.0)),
                   {}}},
                 {1e-3})) {
            behind.extend(r.bounds());
        }
        CHECK(behind.max.y() == Approx(kFlangeDiameter / 2.0 + 1.0).margin(1e-3));
        REQUIRE(sim.comparison);
        CHECK_FALSE(sim.comparison->hasGouges());
    }
    SECTION("a range behind stock that is still there is refused") {
        doc.modify("behind the flange", [&](Project& p) {
            Operation& o = p.setups.front().operations[1];
            o.parameters["pattern"] = "along";
            o.parameters["z_front"] = -kEndLength - kFlangeLength;
            o.parameters["z_back"] = -kLength;
        });
        CHECK_THROWS_WITH(generate(1), ContainsSubstring("where the tool comes from"));
    }
    SECTION("nonsense is refused") {
        doc.modify("backwards",
                   [&](Project& p) { p.setups.front().operations[1].parameters["z_back"] = 5.0; });
        CHECK_THROWS_WITH(generate(1), ContainsSubstring("must be in front of its back"));
    }
    SECTION("a change earlier reaches a ranged operation") {
        const std::string key = toolpathInputKey(doc, setup(), setup().operations[2]);
        doc.modify("earlier", [&](Project& p) {
            p.setups.front().operations[1].presetOverrides.axialDepth = 1.0;
        });
        CHECK(toolpathInputKey(doc, setup(), setup().operations[2]) != key);
    }
}

TEST_CASE("back turning with a left-hand tool", "[document][lathe][backturn]") {
    CancelToken cancel;
    NullProgress progress;
    Document doc =
        Document::open(std::filesystem::path(OCAM_FIXTURE_DIR) / "backturn_project.ocam");
    const Uuid id = doc.project().setups.front().id;
    const auto setup = [&]() -> const Setup& {
        return *doc.project().findSetup(id);
    };
    const auto generate = [&](std::size_t k) {
        return generateToolpath(doc, setup(), setup().operations[k], cancel, progress);
    };
    SECTION("its passes run towards the front, from behind what they cut, up to the flange") {
        const toolpath::Toolpath tp = generate(3);
        CHECK(tp.tipQuadrant == 4);
        bool any = false;
        Vec3 pos = tp.start;
        for (const auto& m : tp.moves) {
            if (m.kind == toolpath::MoveKind::Linear && m.feedClass == toolpath::FeedClass::Cut &&
                std::abs(m.end.x() - pos.x()) < 1e-9) {
                any = true;
                CHECK(m.end.z() > pos.z());
                CHECK(m.end.z() <= -kEndLength - kFlangeLength + 1e-6);
            }
            pos = m.end;
        }
        CHECK(any);
    }
    SECTION("the part comes out to size on both sides of the flange, nothing gouged") {
        SimulationOptions options;
        const SetupSimulation sim = simulateSetup(doc, id, options, cancel, progress);
        for (const auto& op : sim.operations) {
            INFO(op.name << ": " << op.error);
            CHECK(op.error.empty());
        }
        REQUIRE(sim.comparison);
        CHECK_FALSE(sim.comparison->hasGouges());
        // The left-hand tool cuts with its mirrored outline, its body towards the front.
        CHECK(sim.operations[3].insert.bounds().max.x() <= 1e-9);
        CHECK(sim.operations[1].insert.bounds().min.x() >= -1e-9);
        // Everything but the flange's skin (it has no operation here) is off.
        CHECK(sim.operations.back().latheAfter->volume() ==
              Approx(kVolume + std::numbers::pi * 0.25 *
                                   ((kFlangeDiameter + 2.0) * (kFlangeDiameter + 2.0) -
                                    kFlangeDiameter * kFlangeDiameter) *
                                   kFlangeLength)
                  .epsilon(0.02));
    }
    SECTION("the hand and the direction have to agree") {
        doc.modify("right away", [&](Project& p) {
            Operation& o = p.setups.front().operations[3];
            o.toolId = p.tools.tools[0].id;
            o.presetId = p.tools.tools[0].presets.front().id;
        });
        CHECK_THROWS_WITH(generate(3), ContainsSubstring("is right-hand"));
        doc.modify("left towards", [&](Project& p) {
            Operation& o = p.setups.front().operations[1];
            o.toolId = p.tools.tools[1].id;
            o.presetId = p.tools.tools[1].presets.front().id;
        });
        CHECK_THROWS_WITH(generate(1), ContainsSubstring("is left-hand"));
    }
    SECTION("an ISO left-hand holder and its insert run the same way") {
        tooling::Holder holder =
            tooling::makeTurningHolder(*tooling::parseDesignation("PCLNL 2525M 12"));
        doc.modify("holder", [&](Project& p) {
            tooling::fitToHolder(p.tools.tools[1], holder);
            p.tools.tools[1].holderId = holder.id;
            p.tools.holders = {holder};
        });
        CHECK_NOTHROW(generate(4));
    }
    SECTION("from behind with stock still there, it is refused") {
        doc.modify("deeper", [&](Project& p) {
            p.setups.front().operations[3].parameters["z_back"] = -kLength + 10.0;
        });
        CHECK_THROWS_WITH(generate(3), ContainsSubstring("where the tool comes from"));
    }
}

TEST_CASE("nose compensation in the control", "[document][lathe][noseComp]") {
    CancelToken cancel;
    NullProgress progress;
    Document doc = Document::open(std::filesystem::path(OCAM_FIXTURE_DIR) / "ranges_project.ocam");
    const Uuid id = doc.project().setups.front().id;
    const auto finish = [&]() -> const Operation& {
        return doc.project().findSetup(id)->operations[2];
    };
    doc.modify("in control", [&](Project& p) {
        p.setups.front().operations[2].parameters["compensation"] = "control";
    });
    const Setup& setup = *doc.project().findSetup(id);
    const toolpath::Toolpath tp = generateToolpath(doc, setup, finish(), cancel, progress);
    SECTION("the program is the profile, on its way in across the radius") {
        CHECK(tp.compensation == toolpath::Compensation::Right);
        // Along the near end it runs on the part's own radius.
        bool along = false;
        Vec3 pos = tp.start;
        for (const auto& m : tp.moves) {
            if (m.kind == toolpath::MoveKind::Linear && m.feedClass == toolpath::FeedClass::Cut &&
                std::abs(m.end.x() - pos.x()) < 1e-9 && pos.z() - m.end.z() > 5.0) {
                along = true;
                CHECK(m.end.x() == Approx(kEndDiameter / 2.0).margin(1e-6));
            }
            pos = m.end;
        }
        CHECK(along);
        const auto lead = std::find_if(tp.moves.begin(), tp.moves.end(), [](const auto& m) {
            return m.feedClass == toolpath::FeedClass::LeadIn;
        });
        REQUIRE(lead != tp.moves.end());
        REQUIRE(lead != tp.moves.begin());
        CHECK(std::abs(lead->end.x() - std::prev(lead)->end.x()) > 0.1);
    }
    SECTION("it posts G42 and G40, says what the offset page holds, and verifies") {
        const machine::Machine& machine = machineFor(doc.project(), setup);
        const PostOutput out =
            postSetup(doc, id, machine, defaultPostFor(machine), cancel, progress);
        INFO((out.errors.empty() ? std::string() : out.errors.front()));
        REQUIRE(out.ok());
        INFO(out.gcode);
        CHECK_THAT(out.gcode, ContainsSubstring("G42"));
        CHECK_THAT(out.gcode, ContainsSubstring("G40"));
        CHECK_THAT(out.gcode, ContainsSubstring("TIP 3"));
        post::BackplotOptions options;
        options.lathe = true;
        options.machine = &machine;
        const post::BackplotResult plotted = post::backplot(out.gcode, options);
        CHECK(post::compareBackplot(out.toolpaths, plotted.path, 1.5e-3, nullptr, false)
                  .value_or("") == "");
    }
    SECTION("the simulation cuts as the control will, the nose offset") {
        SimulationOptions options;
        const SetupSimulation sim = simulateSetup(doc, id, options, cancel, progress);
        for (const auto& op : sim.operations) {
            INFO(op.name << ": " << op.error);
            CHECK(op.error.empty());
        }
        REQUIRE(sim.comparison);
        CHECK_FALSE(sim.comparison->hasGouges());
    }
    SECTION("the tool's tip orientation has to fit the cut") {
        doc.modify("tip", [&](Project& p) { p.tools.tools[0].insert.orientation = 2; });
        CHECK_THROWS_WITH(
            generateToolpath(doc, *doc.project().findSetup(id), finish(), cancel, progress),
            ContainsSubstring("tip orientation 3"));
    }
}

TEST_CASE("a tailstock and a steady rest are kept clear of", "[document][lathe][supports]") {
    using namespace ocam::fixtures::shaft;
    CancelToken cancel;
    NullProgress progress;
    Document doc = Document::open(std::filesystem::path(OCAM_FIXTURE_DIR) / "centres_project.ocam");
    const auto second = [&]() -> const Setup& {
        return doc.project().setups.back();
    };
    const auto generate = [&](std::size_t k) {
        return generateToolpath(doc, second(), second().operations[k], cancel, progress);
    };
    SECTION(
        "turning on the tailstock runs, clear of the centre, from the stock the first setup left") {
        SimulationOptions options;
        const SetupSimulation sim = simulateSetup(doc, second().id, options, cancel, progress);
        for (const auto& op : sim.operations) {
            INFO(op.name << ": " << op.error);
            CHECK(op.error.empty());
            CHECK(op.fixtureCollisions.empty());
        }
        REQUIRE(sim.comparison);
        // The centre hole is in the part: the first setup cut it, on purpose.
        CHECK(sim.comparison->maxGouge < 6.0);
    }
    SECTION("facing into the centre is refused") {
        doc.modify("face", [&](Project& p) { p.setups.back().operations[0].type = "turn_face"; });
        CHECK_THROWS_WITH(generate(0), ContainsSubstring("Tailstock"));
    }
    SECTION("a quill the tool cannot pass is refused") {
        doc.modify("quill", [&](Project& p) {
            p.setups.back().tailstock->tipZ = -12.0;
            p.setups.back().tailstock->quillDiameter = 200.0;
        });
        CHECK_THROWS_WITH(generate(0), ContainsSubstring("Tailstock"));
    }
    SECTION("passes stop in front of a steady rest, which closes on the stock") {
        doc.modify("steady", [&](Project& p) {
            p.setups.back().steadyRest = LatheSteadyRest{};
            p.setups.back().steadyRest->z = -40.0;
            p.setups.back().steadyRest->width = 20.0;
        });
        const auto supports = latheSupports(doc, second());
        REQUIRE(supports.size() == 2);
        CHECK(supports[1].name == "Steady rest");
        const toolpath::Toolpath tp = generate(0);
        for (const auto& m : tp.moves) {
            CHECK(m.end.z() > -30.0);
        }
        CHECK(std::any_of(tp.warnings.begin(), tp.warnings.end(), [](const std::string& w) {
            return w.find("not turned") != std::string::npos;
        }));
    }
    SECTION("the project keeps them") {
        doc.modify("steady", [&](Project& p) {
            p.setups.back().steadyRest = LatheSteadyRest{};
            p.setups.back().steadyRest->z = -40.0;
        });
        const auto path = std::filesystem::temp_directory_path() / "oxcam_lathe_supports_test.ocam";
        doc.save(path);
        const Document reopened = Document::open(path);
        std::filesystem::remove(path);
        CHECK(reopened.project().setups.back().fixtureClearance == second().fixtureClearance);
        CHECK(reopened.project().setups.back().tailstock == second().tailstock);
        CHECK(reopened.project().setups.back().steadyRest == second().steadyRest);
        CHECK(reopened.project().validate().empty());
    }
    SECTION("on a mill they are nothing") {
        doc.modify("mill", [&](Project& p) { p.setups.back().machine = "generic-3axis-vmc"; });
        CHECK(latheSupports(doc, second()).empty());
    }
    SECTION("nonsense is reported") {
        doc.modify("bad", [&](Project& p) { p.setups.back().tailstock->quillLength = 0.0; });
        CHECK_FALSE(doc.project().validate().empty());
    }
}

TEST_CASE("a plain holder", "[document][lathe][holder]") {
    CancelToken cancel;
    NullProgress progress;
    Document doc = Document::open(std::filesystem::path(OCAM_FIXTURE_DIR) / "grooved_project.ocam");
    const auto hold = [&](const tooling::TurningStick& stick) {
        const tooling::Holder holder = tooling::makeTurningStick(stick);
        doc.modify("holder", [&](Project& p) {
            p.tools.holders = {holder};
            p.tools.tools[1].holderId = holder.id;
        });
    };
    const auto generate = [&](std::size_t k) {
        const Setup& setup = doc.project().setups.front();
        return generateToolpath(doc, setup, setup.operations[k], cancel, progress);
    };
    SECTION("a shank runs outside") {
        hold({});
        CHECK_NOTHROW(generate(3));
        CHECK_THROWS_WITH(generate(4), ContainsSubstring("a shank, which cuts outside"));
    }
    SECTION("it runs along Z in a face") {
        hold({});
        CHECK_NOTHROW(generate(5));
    }
    SECTION("a thin bar runs in the bore, a thick one is refused across the axis") {
        hold({true, 12.0, 9.0, 150.0});
        CHECK_NOTHROW(generate(4));
        CHECK_THROWS_WITH(generate(3), ContainsSubstring("a bar, which cuts inside"));
        hold({true, 16.0, 11.0, 150.0});
        CHECK_THROWS_WITH(generate(4), ContainsSubstring("the holder of T2"));
    }
    SECTION("the library file keeps it") {
        tooling::ToolLibrary lib;
        lib.holders = {tooling::makeTurningStick({true, 12.0, 9.0, 150.0})};
        const tooling::ToolLibrary back = tooling::toolLibraryFromJson(tooling::toJson(lib));
        REQUIRE(back.holders.size() == 1);
        CHECK(back.holders[0].stick == lib.holders[0].stick);
        CHECK(back.holders[0].profile == lib.holders[0].profile);
        CHECK(tooling::holds(back.holders[0], tooling::ToolType::GroovingInsert));
    }
}

TEST_CASE("a part-off's end feed", "[document][lathe]") {
    CancelToken cancel;
    NullProgress progress;
    Document doc = Document::open(std::filesystem::path(OCAM_FIXTURE_DIR) / "shaft_project.ocam");
    const auto generate = [&] {
        const Setup& setup = doc.project().setups.front();
        return generateToolpath(doc, setup, setup.operations[4], cancel, progress);
    };
    // The cut's last feed move over its first, which is the share of the feed per revolution.
    const auto endShare = [&] {
        std::vector<double> rates;
        for (const toolpath::Move& m : generate().moves) {
            if (m.feedClass == toolpath::FeedClass::Cut) {
                rates.push_back(m.feedRate);
            }
        }
        REQUIRE_FALSE(rates.empty());
        return rates.back() / rates.front();
    };
    const auto setPreset = [&](double f) {
        doc.modify("preset", [&](Project& p) { p.tools.tools[2].presets[0].partOffEndFeed = f; });
    };
    const auto setOwn = [&](std::optional<double> f) {
        doc.modify("own", [&](Project& p) {
            p.setups.front().operations[4].presetOverrides.partOffEndFeed = f;
        });
    };
    SECTION("from the preset") {
        CHECK(endShare() == Approx(0.5)); // the preset starts at 0.025 of 0.05
        setPreset(0.01);
        CHECK(endShare() == Approx(0.2));
        setPreset(0.075); // not limited by the feed
        CHECK(endShare() == Approx(1.5));
    }
    SECTION("from the operation") {
        setOwn(0.04);
        CHECK(endShare() == Approx(0.8));
    }
    SECTION("0 is refused, not replaced") {
        setPreset(0.0);
        CHECK_THROWS_WITH(generate(), ContainsSubstring("the part-off end feed of T5 is 0"));
        setOwn(0.02);
        CHECK(endShare() == Approx(0.4));
        setPreset(0.05);
        setOwn(0.0);
        CHECK_THROWS_WITH(generate(), ContainsSubstring("the part-off end feed of T5 is 0"));
    }
    SECTION("no distance, no end feed") {
        setPreset(0.0);
        doc.modify("distance", [&](Project& p) {
            p.setups.front().operations[4].parameters["slow_distance"] = 0.0;
        });
        CHECK(endShare() == Approx(1.0));
    }
}

TEST_CASE("turning's lead feed", "[document][lathe]") {
    CancelToken cancel;
    NullProgress progress;
    Document doc = Document::open(std::filesystem::path(OCAM_FIXTURE_DIR) / "shaft_project.ocam");
    const auto generate = [&](std::size_t k) {
        const Setup& setup = doc.project().setups.front();
        return generateToolpath(doc, setup, setup.operations[k], cancel, progress);
    };
    // Every lead move of an operation over its cutting feed, which is the lead's share of the feed
    // per revolution; every cutting move runs at the feed. Face, rough and finish all have leads.
    const auto leadShare = [&](std::size_t k) {
        const toolpath::Toolpath tp = generate(k);
        double cut = 0.0;
        double lead = 0.0;
        for (const toolpath::Move& m : tp.moves) {
            if (m.feedClass == toolpath::FeedClass::Cut) {
                CHECK((cut == 0.0 || m.feedRate == cut));
                cut = m.feedRate;
            } else if (m.feedClass == toolpath::FeedClass::LeadIn ||
                       m.feedClass == toolpath::FeedClass::LeadOut) {
                CHECK((lead == 0.0 || m.feedRate == lead));
                lead = m.feedRate;
            }
        }
        REQUIRE(cut > 0.0);
        REQUIRE(lead > 0.0);
        return lead / cut;
    };
    const auto setPreset = [&](double f) {
        doc.modify("preset", [&](Project& p) { p.tools.tools[0].presets[0].leadFeedPerRev = f; });
    };
    SECTION("from the preset") {
        for (const std::size_t k : {0u, 1u, 2u}) {
            CHECK(leadShare(k) == Approx(1.0)); // the preset starts at the feed
        }
        setPreset(0.1);
        for (const std::size_t k : {0u, 1u, 2u}) {
            CHECK(leadShare(k) == Approx(0.5));
        }
    }
    SECTION("from the operation") {
        doc.modify("own", [&](Project& p) {
            p.setups.front().operations[2].presetOverrides.leadFeedPerRev = 0.05;
        });
        CHECK(leadShare(2) == Approx(0.25));
        CHECK(leadShare(0) == Approx(1.0));
    }
    SECTION("it does not follow the feed") {
        doc.modify("feed", [&](Project& p) { p.tools.tools[0].presets[0].feedPerTooth = 0.4; });
        CHECK(leadShare(2) == Approx(0.5));
    }
    SECTION("0 is refused, not replaced") {
        setPreset(0.0);
        for (const std::size_t k : {0u, 1u, 2u}) {
            CHECK_THROWS_WITH(generate(k),
                              ContainsSubstring("the lead feed of the cutting preset"));
        }
    }
}
