// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Deterministic mutation testing of every loader: whatever the input, a loader must either
// succeed or throw ocam::Error, never crash or leak another exception type. (Coverage-guided
// fuzzing with libFuzzer lives in tests/fuzz and runs nightly.)

#include "TestProject.hpp"

#include <ocam/document/ProjectIo.hpp>
#include <ocam/foundation/Error.hpp>
#include <ocam/tooling/ToolLibrary.hpp>

#include <catch2/catch_test_macros.hpp>

#include <functional>
#include <random>
#include <string>

using namespace ocam;

namespace {

// Replaces a random node of the document with a random value, or deletes an object member.
void mutateStructure(Json& node, std::mt19937& rng, int depth = 0) {
    std::uniform_int_distribution<int> pick(0, 9);
    const bool container = node.is_object() || node.is_array();
    if (container && !node.empty() && (depth < 2 || pick(rng) < 7)) {
        std::uniform_int_distribution<std::size_t> idx(0, node.size() - 1);
        auto it = node.begin();
        std::advance(it, static_cast<std::ptrdiff_t>(idx(rng)));
        if (node.is_object() && pick(rng) == 0) {
            node.erase(it);
            return;
        }
        mutateStructure(*it, rng, depth + 1);
        return;
    }
    switch (pick(rng)) {
    case 0:
        node = nullptr;
        break;
    case 1:
        node = -1;
        break;
    case 2:
        node = 1e308;
        break;
    case 3:
        node = "x";
        break;
    case 4:
        node = Json::array();
        break;
    case 5:
        node = Json::object();
        break;
    case 6:
        node = true;
        break;
    case 7:
        node = "00000000-0000-4000-8000-000000000000";
        break;
    case 8:
        node = std::numeric_limits<std::int64_t>::min();
        break;
    default:
        node = Json::array({1, 2, 3});
        break;
    }
}

void mutateBytes(std::string& text, std::mt19937& rng) {
    std::uniform_int_distribution<int> op(0, 2);
    std::uniform_int_distribution<int> byte(0, 255);
    const int edits = 1 + static_cast<int>(rng() % 4);
    for (int e = 0; e < edits && !text.empty(); ++e) {
        std::uniform_int_distribution<std::size_t> pos(0, text.size() - 1);
        const std::size_t p = pos(rng);
        switch (op(rng)) {
        case 0:
            text[p] = static_cast<char>(byte(rng));
            break;
        case 1:
            text.erase(p, 1 + rng() % 8);
            break;
        default:
            text.insert(p, 1, static_cast<char>(byte(rng)));
            break;
        }
    }
}

// Runs `load` and fails the test if it throws anything other than ocam::Error.
void expectOnlyOcamErrors(const std::function<void()>& load) {
    try {
        load();
    } catch (const Error&) {
        // expected for bad input
    } catch (const std::exception& e) {
        FAIL("unexpected exception type: " << e.what());
    }
}

} // namespace

TEST_CASE("tool library loader survives mutated input", "[robustness]") {
    const Json base = tooling::toJson(document::testing::sampleProject().tools);
    std::mt19937 rng(20261018);
    for (int i = 0; i < 1500; ++i) {
        Json doc = base;
        mutateStructure(doc, rng);
        expectOnlyOcamErrors([&] { (void)tooling::toolLibraryFromJson(doc); });

        std::string text = dumpJson(base);
        mutateBytes(text, rng);
        expectOnlyOcamErrors([&] { (void)tooling::toolLibraryFromJson(parseJson(text, "t")); });
    }
}

TEST_CASE("project loader survives mutated input", "[robustness]") {
    const Json base = document::toJson(document::testing::sampleProject());
    std::mt19937 rng(54);
    for (int i = 0; i < 1500; ++i) {
        Json doc = base;
        mutateStructure(doc, rng);
        expectOnlyOcamErrors([&] { (void)document::projectFromJson(doc); });

        std::string text = dumpJson(base);
        mutateBytes(text, rng);
        expectOnlyOcamErrors([&] { (void)document::projectFromJson(parseJson(text, "p")); });
    }
}

TEST_CASE("archive reader survives mutated input", "[robustness]") {
    document::ArchiveContents contents;
    contents.projectJson = dumpJson(document::toJson(document::testing::sampleProject()));
    const std::string step(2000, 'S');
    contents.models.emplace(sha256Hex(step), step);
    const std::string base = document::writeArchive(contents);

    std::mt19937 rng(7);
    for (int i = 0; i < 1000; ++i) {
        std::string bytes = base;
        mutateBytes(bytes, rng);
        expectOnlyOcamErrors([&] { (void)document::readArchive(bytes, "fuzz.ocam"); });
    }
}
