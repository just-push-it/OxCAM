// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ocam/foundation/Log.hpp>

#include <catch2/catch_test_macros.hpp>

#include <string>
#include <utility>
#include <vector>

using namespace ocam;

TEST_CASE("log level filtering and custom sink", "[log]") {
    std::vector<std::pair<LogLevel, std::string>> captured;
    setLogSink([&](LogLevel level, std::string_view msg) { captured.emplace_back(level, msg); });
    setLogLevel(LogLevel::Info);

    logDebug("hidden {}", 1);
    logInfo("tool {} at {:.2f} mm", 12, 3.14159);
    logWarn("plain");
    logMessage(LogLevel::Off, "never emitted");

    setLogLevel(LogLevel::Error);
    logWarn("filtered");
    logError("kept");

    setLogSink({});
    setLogLevel(LogLevel::Info);

    REQUIRE(captured.size() == 3);
    CHECK(captured[0] == std::pair{LogLevel::Info, std::string("tool 12 at 3.14 mm")});
    CHECK(captured[1] == std::pair{LogLevel::Warn, std::string("plain")});
    CHECK(captured[2] == std::pair{LogLevel::Error, std::string("kept")});
}

TEST_CASE("log level names", "[log]") {
    CHECK(toString(LogLevel::Warn) == "warn");
    CHECK(toString(LogLevel::Off) == "off");
}
