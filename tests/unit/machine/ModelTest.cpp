// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ocam/foundation/Error.hpp>
#include <ocam/foundation/Sha256.hpp>
#include <ocam/machine/Kinematics.hpp>
#include <ocam/machine/Model.hpp>
#include <ocam/toolpath/PathBuilder.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <random>

using namespace ocam;
using namespace ocam::machine;
using Catch::Matchers::ContainsSubstring;

namespace {

/// A machine of every mill kinematics type (the built-ins where there is one).
Machine machineOf(Kinematics k) {
    for (const Machine* m : builtinMachines()) {
        if (m->kinematics == k) {
            return *m;
        }
    }
    Machine m = k == Kinematics::Mill5HeadBC ? genericMill5HeadTableBC() : genericMill5TableAC();
    m.id = "test-" + std::string(toString(k));
    m.kinematics = k;
    for (Axis& a : m.axes) {
        if (a.name == "A") {
            a.name = "B";
        }
    }
    if (k == Kinematics::Mill5HeadBC) {
        m.components.clear();
    }
    return m;
}

const Kinematics kMills[] = {
    Kinematics::Mill3,        Kinematics::Mill4A,           Kinematics::Mill5TableAC,
    Kinematics::Mill5TableBC, Kinematics::Mill5HeadTableBC, Kinematics::Mill5HeadBC};

bool near(const Vec3& a, const Vec3& b, double tol = 1e-9) {
    return (a - b).norm() < tol;
}
bool near(const Mat3& a, const Mat3& b, double tol = 1e-9) {
    return (a - b).cwiseAbs().maxCoeff() < tol;
}

/// Random rotary positions within limits (radians).
IndexAngles randomAngles(const Machine& m, std::mt19937& rng) {
    IndexAngles q;
    for (const char c : m.rotaryAxes()) {
        const Axis* a = m.axis(std::string(1, c));
        std::uniform_real_distribution<double> d(std::max(a->min, -180.0), std::min(a->max, 180.0));
        (c == 'A' ? q.a : c == 'B' ? q.b : q.c) = degToRad(d(rng));
    }
    return q;
}

geom3d::Mesh boxMesh(const Vec3& lo, const Vec3& hi) {
    Component c;
    c.min = lo;
    c.max = hi;
    return componentMesh(c);
}

std::filesystem::path tempFolder(const char* name) {
    const auto dir = std::filesystem::temp_directory_path() /
                     ("ocam-model-test-" + std::to_string(std::random_device{}()) + "-" + name);
    std::filesystem::create_directories(dir);
    return dir;
}

} // namespace

TEST_CASE("default models follow each kinematics type", "[machine][model]") {
    std::mt19937 rng(7);
    for (const Kinematics k : kMills) {
        CAPTURE(toString(k));
        Machine m = machineOf(k);
        m.model = defaultModel(m);
        REQUIRE(validateModel(m).empty());
        CHECK(m.validate().empty());
        const MachineModel& model = m.model;
        const double gauge = 120.0;
        const std::string table = tableEnd(model);
        const Transform3 tableZero = poseAt(model, {}).table;
        for (int trial = 0; trial < 50; ++trial) {
            const IndexAngles q = randomAngles(m, rng);
            std::uniform_real_distribution<double> d(-80.0, 80.0);
            const Vec3 tip(d(rng), d(rng), d(rng) + 40.0);
            const auto pos = positionFor(model, q, table, tableZero * tip, gauge);
            REQUIRE(pos);
            CHECK(pos->rotary == q);
            const ModelPose pose = poseAt(model, *pos);
            // The tool tip is on the programmed point of the part.
            const Vec3 toolTip = pose.spindle.translation() - gauge * pose.spindle.linear().col(2);
            CHECK(near(toolTip, pose.table * tip, 1e-7));
            // The table and the head turn as solve has them.
            CHECK(near(pose.table.linear(), tableRotation(m, q)));
            CHECK(near(pose.spindle.linear(), headRotation(m, q)));
            CHECK(near(pose.table.linear().transpose() * pose.spindle.linear().col(2),
                       toolAxis(m, q)));
            // Parts move with their carriers: a part on the table stays put on the table.
            const Transform3 onTable = pose.carried(model, table);
            const Vec3 corner = tableCentre(model, Vec3::Zero()) + Vec3(30, -20, 10);
            CHECK(near(onTable * corner, pose.table * Vec3(30, -20, 10), 1e-9));
            // Cradle parts turn with the outer table rotary only.
            const Transform3 cradle = pose.carried(model, cradleCarrier(model));
            CHECK(near(cradle.linear(), cradleRotation(m, q)));
        }
        // With the rotary axes at 0, the linear axes are the tool tip relative to the table.
        const auto origin = positionFor(model, {}, table, tableZero * Vec3::Zero(), gauge);
        const auto moved = positionFor(model, {}, table, tableZero * Vec3(10, 20, 30), gauge);
        REQUIRE(origin);
        REQUIRE(moved);
        CHECK(near(moved->linear - origin->linear, Vec3(10, 20, 30)));
        CHECK(near(origin->linear, Vec3(0, 0, gauge - kDefaultSpindleHeight)));
    }
}

TEST_CASE("machine components become parts on the model's carriers", "[machine][model]") {
    const Machine& m = genericMill5TableAC();
    const MachineModel model = effectiveModel(m);
    const auto parts = componentParts(m, model);
    REQUIRE(parts.size() == m.components.size());
    for (std::size_t i = 0; i < parts.size(); ++i) {
        CAPTURE(parts[i].name);
        REQUIRE(parts[i].geometry);
        CHECK(parts[i].geometry->isClosed());
        CHECK(parts[i].geometry->signedVolume() > 0.0);
        switch (m.components[i].attach) {
        case Component::Attach::Head:
            CHECK(parts[i].carrier == "Z");
            break;
        case Component::Attach::Table:
            CHECK(parts[i].carrier == "C");
            break;
        case Component::Attach::Cradle:
            CHECK(parts[i].carrier == "A");
            break;
        }
    }
    CHECK(cradleCarrier(effectiveModel(genericMill4A())) == "X");
    CHECK(effectiveModel(genericLathe2()).empty());
}

TEST_CASE("model validation names what does not fit the kinematics", "[machine][model]") {
    Machine m = genericMill5TableAC();
    m.model = defaultModel(m);
    REQUIRE(m.validate().empty());

    SECTION("a table axis pointing the machine axis's way") {
        m.model.axes[1].direction = Vec3::UnitX(); // X moves the table
        const auto issues = validateModel(m);
        REQUIRE(issues.size() == 1);
        CHECK_THAT(issues[0],
                   ContainsSubstring("the X axis moves the table") && ContainsSubstring("-X"));
    }
    SECTION("a rotary axis on the wrong side") {
        for (ModelAxis& a : m.model.axes) {
            if (a.name == "C") {
                a.parent = "Z"; // C on the head
            }
        }
        const auto issues = validateModel(m);
        REQUIRE_FALSE(issues.empty());
        CHECK_THAT(issues[0], ContainsSubstring("the C axis carries neither the spindle nor the "
                                                "A axis"));
    }
    SECTION("rotary axes in the wrong order") {
        for (ModelAxis& a : m.model.axes) {
            if (a.name == "A") {
                a.parent = "C";
            } else if (a.name == "C") {
                a.parent = "X";
            }
        }
        const auto issues = validateModel(m);
        REQUIRE(issues.size() == 1);
        CHECK_THAT(issues[0], ContainsSubstring("the A axis must carry the C axis"));
    }
    SECTION("unknown carriers, missing axes and loops") {
        m.model.parts.push_back(makePart("base", boxMesh(Vec3::Zero(), Vec3::Ones())));
        m.model.parts.back().carrier = "B";
        m.model.axes.erase(m.model.axes.begin() + 2); // Z
        const auto issues = validateModel(m);
        CHECK(std::any_of(issues.begin(), issues.end(), [](const std::string& s) {
            return s.find("needs the Z axis") != std::string::npos;
        }));
        CHECK(std::any_of(issues.begin(), issues.end(), [](const std::string& s) {
            return s.find("part 'base' is carried by 'B'") != std::string::npos;
        }));
        Machine loop = genericMill5TableAC();
        loop.model = defaultModel(loop);
        loop.model.axes[0].parent = "X"; // Y on X on Y
        const auto loopIssues = validateModel(loop);
        REQUIRE_FALSE(loopIssues.empty());
        CHECK_THAT(loopIssues[0], ContainsSubstring("loop"));
    }
    SECTION("a lathe's needs its layout") {
        Machine lathe = genericLathe2();
        lathe.model = m.model;
        CHECK_THAT(validateModel(lathe).at(0), ContainsSubstring("layout"));
    }
}

TEST_CASE("machine files keep the model and its meshes next to them", "[machine][model]") {
    Machine m = genericMill5TableAC();
    m.id = "test-trunnion";
    m.model = defaultModel(m);
    ModelPart base = makePart("base", boxMesh(Vec3(-500, -400, -600), Vec3(500, 400, -300)));
    base.colour = 0x445566;
    ModelPart cradle = makePart("cradle", boxMesh(Vec3(-280, -170, -260), Vec3(280, 170, -200)));
    cradle.carrier = "A";
    cradle.rotation = Vec3(0, 0, 90);
    cradle.offset = Vec3(1, 2, 3);
    ModelPart head = makePart("head", boxMesh(Vec3(-100, -100, 0), Vec3(100, 150, 300)));
    head.carrier = "Z";
    head.offset = m.model.spindlePoint;
    head.collides = false;
    cradle.shown = false; // hidden in the views, saved with the machine
    base.enclosure = true; // hidden with the enclosure, saved with the machine
    m.model.parts = {base, cradle, head};
    REQUIRE(m.validate().empty());

    const auto dir = tempFolder("roundtrip");
    const auto file = dir / "test-trunnion.json";
    saveMachine(m, file);
    CHECK(std::filesystem::is_regular_file(meshFolder(file) / (base.mesh + ".stl")));
    const Machine loaded = loadMachine(file);
    CHECK(loaded == m);
    REQUIRE(loaded.model.parts.size() == 3);
    CHECK(loaded.model.parts[0].enclosure);
    CHECK_FALSE(loaded.model.parts[1].enclosure);
    for (std::size_t i = 0; i < 3; ++i) {
        REQUIRE(loaded.model.parts[i].geometry);
        CHECK(loaded.model.parts[i].geometry->triangles.size() ==
              m.model.parts[i].geometry->triangles.size());
        CHECK(partFileBytes(loaded.model.parts[i]) == partFileBytes(m.model.parts[i]));
    }
    CHECK(machineFromJson(toJson(m)) == m);

    // Meshes no part uses are removed; a missing mesh is an error.
    m.model.parts.pop_back();
    saveMachine(m, file);
    CHECK_FALSE(std::filesystem::exists(meshFolder(file) / (head.mesh + ".stl")));
    std::filesystem::remove(meshFolder(file) / (base.mesh + ".stl"));
    CHECK_THROWS_AS(loadMachine(file), Error);
    std::filesystem::remove_all(dir);
}

TEST_CASE("STEP parts keep their STEP file next to the machine file", "[machine][model]") {
    Machine m = genericMill5TableAC();
    m.id = "step-trunnion";
    m.model = defaultModel(m);
    m.model.meshTolerance = 0.35;
    // The machine library stores the bytes as they are; reading the B-rep is libs/document's.
    const auto bytes = std::make_shared<const std::string>("ISO-10303-21;\nthe table's body\n");
    ModelPart table = stepPart("table", bytes);
    table.carrier = "C";
    CHECK(table.exact());
    CHECK(table.step == sha256Hex(*bytes));
    CHECK(table.mesh.empty());
    CHECK(partFileName(table) == table.step + ".step");
    ModelPart base = makePart("base", boxMesh(Vec3(-500, -400, -600), Vec3(500, 400, -300)));
    CHECK_FALSE(base.exact());
    m.model.parts = {table, base};
    REQUIRE(validateModel(m).empty());

    const Json doc = toJson(m);
    CHECK(doc.at("version") == kMachineVersion);
    CHECK(doc.at("model").at("mesh_tolerance") == 0.35);
    CHECK(doc.at("model").at("parts").at(0).at("step") == table.step);
    CHECK_FALSE(doc.at("model").at("parts").at(0).contains("mesh"));
    CHECK(doc.at("model").at("parts").at(1).at("mesh") == base.mesh);
    // A part with neither or both is refused.
    Json both = doc;
    both["model"]["parts"][0]["mesh"] = base.mesh;
    CHECK_THROWS(machineFromJson(both));
    Json neither = doc;
    neither["model"]["parts"][0].erase("step");
    CHECK_THROWS(machineFromJson(neither));

    const auto dir = tempFolder("step-parts");
    const auto file = dir / "step-trunnion.json";
    saveMachine(m, file);
    CHECK(std::filesystem::is_regular_file(meshFolder(file) / (table.step + ".step")));
    CHECK(std::filesystem::is_regular_file(meshFolder(file) / (base.mesh + ".stl")));
    const Machine loaded = loadMachine(file);
    CHECK(loaded == m);
    CHECK(loaded.model.meshTolerance == 0.35);
    REQUIRE(loaded.model.parts.at(0).stepFile);
    CHECK(*loaded.model.parts[0].stepFile == *bytes);
    CHECK_FALSE(loaded.model.parts[0].geometry); // tessellated by libs/document
    REQUIRE(loaded.model.parts.at(1).geometry);

    // A STEP file no part uses is removed; a corrupt one is an error.
    m.model.parts = {base};
    saveMachine(m, file);
    CHECK_FALSE(std::filesystem::exists(meshFolder(file) / (table.step + ".step")));
    m.model.parts = {table, base};
    saveMachine(m, file);
    writeFileAtomic(meshFolder(file) / (table.step + ".step"), "something else");
    CHECK_THROWS_AS(loadMachine(file), Error);
    std::filesystem::remove_all(dir);

    // Version 4 files: mesh parts, loaded as they are.
    Json v4 = toJson(m);
    v4["version"] = 4;
    v4["model"].erase("mesh_tolerance");
    v4["model"]["parts"].erase(0);
    const Machine old = machineFromJson(v4);
    REQUIRE(old.model.parts.size() == 1);
    CHECK(old.model.parts[0].mesh == base.mesh);
    CHECK(old.model.meshTolerance == 0.2);
}

TEST_CASE("version 1 machine files still load", "[machine][model]") {
    const Machine v1 =
        loadMachine(OCAM_SOURCE_DIR "/tests/unit/machine/data/machine-v1-trunnion-ac.json");
    CHECK(v1 == genericMill5TableAC());
    CHECK(v1.model.empty());
    CHECK(toJson(v1).at("version") == kMachineVersion);
}

TEST_CASE("a machine's revision is kept, and files without one are revision 0", "[machine]") {
    Machine m = genericMill5TableAC();
    CHECK(m.revision == 0);
    CHECK_FALSE(toJson(m).contains("revision"));
    m.revision = 7;
    const Json doc = toJson(m);
    CHECK(doc.at("revision") == 7);
    CHECK(machineFromJson(doc).revision == 7);
    // A version 12 file has none.
    Json v12 = toJson(genericMill5TableAC());
    v12["version"] = 12;
    CHECK(machineFromJson(v12).revision == 0);
    // Not a count below 0.
    Json negative = doc;
    negative["revision"] = -1;
    CHECK_THROWS_AS(machineFromJson(negative), Error);
}

TEST_CASE("version 13 machine files have no enclosure parts", "[machine][model]") {
    Json v13 = toJson(genericMill5TableAC());
    v13["version"] = 13;
    CHECK(machineFromJson(v13) == genericMill5TableAC());
}

TEST_CASE("version 3 machine files drop the table's carrier and origin", "[machine][model]") {
    // Version 3 models had a table carrier and origin (the work origin); where the work sits is
    // each setup's placement now.
    Machine m = genericMill5TableAC();
    m.id = "v3-sample";
    m.model = defaultModel(m);
    Json doc = toJson(m);
    doc["version"] = 3;
    doc["model"]["table"] = Json{{"carrier", "C"}, {"origin", Json::array({0.0, 0.0, -30.0})}};
    const Machine v3 = machineFromJson(doc);
    CHECK(v3 == m);
    CHECK(tableEnd(v3.model) == "C");
    CHECK_FALSE(toJson(v3).at("model").contains("table"));
    CHECK(toJson(v3).at("version") == kMachineVersion);
    // A version 4 file with a table is refused by the schema.
    Json v4 = toJson(m);
    v4["model"]["table"] = Json{{"origin", Json::array({0.0, 0.0, 0.0})}};
    CHECK_THROWS(machineFromJson(v4));
}

TEST_CASE("the table chain and its centre come from the kinematic tree", "[machine][model]") {
    // A/C trunnion: A on X, C on A; the table centre where their lines meet.
    MachineModel ac = defaultModel(genericMill5TableAC());
    CHECK(tableEnd(ac) == "C");
    REQUIRE(tableRotaries(ac).size() == 2);
    CHECK(tableRotaries(ac).front()->name == "A");
    for (ModelAxis& a : ac.axes) {
        if (a.name == "A") {
            a.pivot = Vec3(0, 0, -40); // A 40 mm below the table top
        } else if (a.name == "C") {
            a.pivot = Vec3(15, 0, 0); // C 15 mm right of machine zero
        }
    }
    CHECK(near(tableCentre(ac, Vec3(500, 500, 500)), Vec3(15, 0, -40), 1e-9));
    // One rotary axis: the point of its line nearest the WCS origin.
    const MachineModel a4 = defaultModel(genericMill4A());
    CHECK(tableEnd(a4) == "A");
    CHECK(near(tableCentre(a4, Vec3(-180, 20, 30)), Vec3(-180, 0, 0), 1e-9));
    const WorkMount mount{"A", Transform3(Eigen::Translation3d(Vec3(-180, 20, 30)))};
    CHECK(near(Vec3(wcsInTable(a4, mount).translation()), Vec3(0, 20, 30), 1e-9));
    CHECK(rotariesNotCarrying(a4, "X") == std::vector<std::string>{"A"});
    CHECK(rotariesNotCarrying(a4, "A").empty());
    // Head-table: C turns the table. Head-head: nothing but the spindle moves; the work sits on
    // the base.
    CHECK(tableEnd(defaultModel(genericMill5HeadTableBC())) == "C");
    Machine fork = genericMill5HeadTableBC();
    fork.kinematics = Kinematics::Mill5HeadBC;
    CHECK(tableEnd(defaultModel(fork)).empty());
    CHECK(mountProblem(defaultModel(fork), "").empty());
    CHECK_THAT(mountProblem(a4, "Z"), ContainsSubstring("carries the spindle"));
}

TEST_CASE("version 2 machine files still load", "[machine][model]") {
    const Machine v2 =
        loadMachine(OCAM_SOURCE_DIR "/tests/unit/machine/data/machine-v2-trunnion-ac.json");
    CHECK(v2.model.axes.size() == 5);
    CHECK(tableEnd(v2.model) == "C");
    CHECK(v2.validate().empty());
    CHECK(toJson(v2).at("version") == kMachineVersion);
    Machine plain = v2;
    plain.id = genericMill5TableAC().id;
    plain.model = {};
    CHECK(plain == genericMill5TableAC());
}

TEST_CASE("rotary tracks follow the setup's index", "[machine][model]") {
    const Machine& m = genericMill5TableAC();
    toolpath::Toolpath tp;
    tp.start = Vec3(0, 0, 50);
    toolpath::PathBuilder b(tp);
    b.rapid(Vec3(0, 0, 5));
    b.feed(Vec3(20, 0, 5), toolpath::FeedClass::Cut, 10.0);
    Transform3 wcs = Transform3::Identity();
    wcs.linear() = Eigen::AngleAxisd(kPi / 2, Vec3::UnitX()).toRotationMatrix();
    const RotaryTrack track = rotaryTrack(m, tp, wcs);
    REQUIRE(track.angles.size() == tp.moves.size());
    CHECK(track.start == *indexAngles(m, wcs.linear().col(2)));
    CHECK(track.angles.back() == track.start);
    CHECK(mountedWcs(m, tp, wcs).isApprox(wcs));
    // A machine that cannot index has the part clamped square to it.
    const Transform3 square = mountedWcs(genericMill3(), tp, wcs);
    CHECK(square.linear().isIdentity());
    CHECK(rotaryTrack(genericMill3(), tp, wcs).start == IndexAngles{});
}
