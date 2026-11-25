// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ocam/foundation/Uuid.hpp>

#include <catch2/catch_test_macros.hpp>

#include <unordered_set>

using namespace ocam;

TEST_CASE("default Uuid is nil", "[uuid]") {
    constexpr Uuid nil;
    STATIC_REQUIRE(nil.isNil());
    CHECK(nil.toString() == "00000000-0000-0000-0000-000000000000");
}

TEST_CASE("generated Uuids are version 4, RFC variant, and unique", "[uuid]") {
    std::unordered_set<Uuid> seen;
    for (int i = 0; i < 1000; ++i) {
        const Uuid id = Uuid::generate();
        REQUIRE(id.version() == 4);
        REQUIRE((id.bytes()[8] & 0xC0) == 0x80);
        REQUIRE(seen.insert(id).second);
    }
}

TEST_CASE("Uuid string round-trip", "[uuid]") {
    const Uuid id = Uuid::generate();
    const auto parsed = Uuid::parse(id.toString());
    REQUIRE(parsed.has_value());
    CHECK(*parsed == id);

    const auto upper = Uuid::parse("F81D4FAE-7DEC-11D0-A765-00A0C91E6BF6");
    REQUIRE(upper.has_value());
    CHECK(upper->toString() == "f81d4fae-7dec-11d0-a765-00a0c91e6bf6");
}

TEST_CASE("Uuid rejects malformed text", "[uuid]") {
    CHECK_FALSE(Uuid::parse("").has_value());
    CHECK_FALSE(Uuid::parse("{f81d4fae-7dec-11d0-a765-00a0c91e6bf6}").has_value());
    CHECK_FALSE(Uuid::parse("f81d4fae7dec11d0a76500a0c91e6bf6").has_value());
    CHECK_FALSE(Uuid::parse("f81d4fae-7dec-11d0-a765-00a0c91e6bfg").has_value());
    CHECK_FALSE(Uuid::parse("f81d4fae-7dec-11d0-a765+00a0c91e6bf6").has_value());
    CHECK_FALSE(Uuid::parse("f81d4fa-e7dec-11d0-a765-00a0c91e6bf6").has_value());
}

TEST_CASE("Uuid ordering is total", "[uuid]") {
    const auto a = *Uuid::parse("00000000-0000-0000-0000-000000000001");
    const auto b = *Uuid::parse("00000000-0000-0000-0000-000000000002");
    CHECK(a < b);
    CHECK(a != b);
}
