// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ocam/foundation/Units.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

using namespace ocam;
using Catch::Matchers::WithinAbs;

TEST_CASE("length conversion round-trips", "[units]") {
    STATIC_REQUIRE(toMm(1.0, LengthUnit::Inch) == 25.4);
    STATIC_REQUIRE(toMm(12.5, LengthUnit::Millimeter) == 12.5);
    REQUIRE_THAT(fromMm(toMm(0.375, LengthUnit::Inch), LengthUnit::Inch), WithinAbs(0.375, 1e-15));
}

TEST_CASE("feed conversion uses per-minute at the boundary", "[units]") {
    // 60 in/min = 1524 mm/min = 25.4 mm/s
    REQUIRE_THAT(feedPerMinToMmPerSec(60.0, LengthUnit::Inch), WithinAbs(25.4, 1e-12));
    REQUIRE_THAT(mmPerSecToFeedPerMin(25.4, LengthUnit::Millimeter), WithinAbs(1524.0, 1e-9));
}

TEST_CASE("angle conversion", "[units]") {
    REQUIRE_THAT(degToRad(180.0), WithinAbs(kPi, 1e-15));
    REQUIRE_THAT(radToDeg(kPi / 2.0), WithinAbs(90.0, 1e-12));
}

TEST_CASE("length unit parsing", "[units]") {
    CHECK(parseLengthUnit("mm") == LengthUnit::Millimeter);
    CHECK(parseLengthUnit("Millimetres") == LengthUnit::Millimeter);
    CHECK(parseLengthUnit("IN") == LengthUnit::Inch);
    CHECK(parseLengthUnit("inches") == LengthUnit::Inch);
    CHECK_FALSE(parseLengthUnit("").has_value());
    CHECK_FALSE(parseLengthUnit("cm").has_value());
    CHECK(toString(LengthUnit::Inch) == "in");
    CHECK(toString(LengthUnit::Millimeter) == "mm");
}
