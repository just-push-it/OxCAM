// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ocam/foundation/Cancel.hpp>
#include <ocam/foundation/Error.hpp>
#include <ocam/foundation/Progress.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <string>
#include <thread>
#include <vector>

using namespace ocam;
using Catch::Matchers::WithinAbs;

namespace {

struct RecordingSink final : ProgressSink {
    std::vector<double> values;
    std::string lastStage;
    void report(double fraction, std::string_view stage) override {
        values.push_back(fraction);
        lastStage = stage;
    }
};

} // namespace

TEST_CASE("CancelToken", "[cancel]") {
    CancelToken token;
    CHECK_FALSE(token.isCancelled());
    CHECK_NOTHROW(token.throwIfCancelled());

    std::thread([&] { token.requestCancel(); }).join();

    CHECK(token.isCancelled());
    CHECK_THROWS_AS(token.throwIfCancelled(), OperationCancelled);
    try {
        token.throwIfCancelled();
    } catch (const Error& e) {
        CHECK(e.code() == ErrorCode::Cancelled);
    }
}

TEST_CASE("ProgressRange maps, nests and clamps", "[progress]") {
    RecordingSink root;
    ProgressRange half(root, 0.5, 1.0);
    ProgressRange quarter(half, 0.0, 0.5); // maps to [0.5, 0.75] of root

    quarter.report(0.0, "start");
    quarter.report(1.0, "done");
    quarter.report(2.0, ""); // clamped to 1.0

    REQUIRE(root.values.size() == 3);
    CHECK_THAT(root.values[0], WithinAbs(0.5, 1e-12));
    CHECK_THAT(root.values[1], WithinAbs(0.75, 1e-12));
    CHECK_THAT(root.values[2], WithinAbs(0.75, 1e-12));
    CHECK(root.lastStage.empty());
}

TEST_CASE("ProgressRange never moves backwards", "[progress]") {
    RecordingSink root;
    ProgressRange range(root, 0.0, 1.0);
    range.report(0.6, "");
    range.report(0.4, "");
    range.report(0.8, "");
    REQUIRE(root.values.size() == 2);
    CHECK_THAT(root.values.back(), WithinAbs(0.8, 1e-12));
}

TEST_CASE("NullProgress accepts anything", "[progress]") {
    NullProgress sink;
    CHECK_NOTHROW(sink.report(-1.0, "ignored"));
}
