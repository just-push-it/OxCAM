// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ocam/foundation/Error.hpp>
#include <ocam/foundation/Json.hpp>
#include <ocam/foundation/Sha256.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <filesystem>
#include <string>

using namespace ocam;
using Catch::Matchers::ContainsSubstring;

TEST_CASE("SHA-256 test vectors (FIPS 180-4 examples)", "[sha256]") {
    CHECK(sha256Hex("") == "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
    CHECK(sha256Hex("abc") == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    CHECK(sha256Hex("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq") ==
          "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
    CHECK(sha256Hex(std::string(1'000'000, 'a')) ==
          "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
}

TEST_CASE("SHA-256 incremental updates match one-shot", "[sha256]") {
    std::string data;
    for (int i = 0; i < 1000; ++i) {
        data += static_cast<char>(i * 7 + 3);
    }
    for (std::size_t chunk : {1u, 3u, 63u, 64u, 65u, 200u}) {
        Sha256 h;
        for (std::size_t i = 0; i < data.size(); i += chunk) {
            h.update(std::string_view(data).substr(i, chunk));
        }
        CHECK(Sha256::toHex(h.finish()) == sha256Hex(data));
    }
}

TEST_CASE("atomic write and read back", "[json]") {
    const auto path = std::filesystem::temp_directory_path() / "oxcam_atomic_test.json";
    writeFileAtomic(path, "first");
    writeFileAtomic(path, "second"); // replaces existing
    CHECK(readFileBytes(path) == "second");
    std::filesystem::remove(path);
    CHECK_THROWS_AS(readFileBytes(path), Error);
}

TEST_CASE("JSON parse errors and stable dumps", "[json]") {
    CHECK_THROWS_WITH(parseJson("{ \"a\": ", "test.json"), ContainsSubstring("test.json"));
    // A number too large to hold is the lexer's out_of_range, a parse error all the same.
    CHECK_THROWS_WITH(parseJson("[4e720]", "big.json"), ContainsSubstring("big.json"));
    const Json doc = parseJson(R"({"b": 1, "a": [true, null]})", "x");
    CHECK(dumpJson(doc) == "{\n  \"a\": [\n    true,\n    null\n  ],\n  \"b\": 1\n}\n");
}

TEST_CASE("schema validation reports every violation", "[json]") {
    const SchemaValidator validator(Json::parse(R"({
        "type": "object",
        "required": ["name", "count"],
        "properties": {
            "name": {"type": "string"},
            "count": {"type": "integer", "minimum": 0}
        }
    })"));
    CHECK_NOTHROW(validator.validate(Json{{"name", "x"}, {"count", 2}}, "ok"));
    try {
        validator.validate(Json{{"name", 5}, {"count", -1}}, "bad.json");
        FAIL("expected a schema error");
    } catch (const Error& e) {
        CHECK(e.code() == ErrorCode::Parse);
        CHECK_THAT(std::string(e.what()), ContainsSubstring("bad.json"));
        CHECK_THAT(std::string(e.what()), ContainsSubstring("/name"));
        CHECK_THAT(std::string(e.what()), ContainsSubstring("/count"));
    }
    // Unresolvable references are rejected when the schema is loaded.
    CHECK_THROWS_AS(SchemaValidator(Json::parse(R"({"$ref": "#/definitions/missing"})")), Error);
}

TEST_CASE("format migrations", "[json]") {
    FormatSpec spec{"oxcam.test", 3, {}};
    spec.migrations[1] = [](Json& d) {
        d["added_in_2"] = true;
    };
    spec.migrations[2] = [](Json& d) {
        d["renamed"] = d.at("old");
        d.erase("old");
    };

    Json v1{{"format", "oxcam.test"}, {"version", 1}, {"old", 42}};
    CHECK(spec.upgrade(v1, "v1") == 1);
    CHECK(v1.at("version") == 3);
    CHECK(v1.at("added_in_2") == true);
    CHECK(v1.at("renamed") == 42);
    CHECK_FALSE(v1.contains("old"));

    Json current{{"format", "oxcam.test"}, {"version", 3}};
    CHECK(spec.upgrade(current, "v3") == 3);

    auto codeOf = [&](Json doc) {
        try {
            spec.upgrade(doc, "doc");
        } catch (const Error& e) {
            return e.code();
        }
        return ErrorCode::Internal;
    };
    CHECK(codeOf(Json{{"format", "oxcam.test"}, {"version", 4}}) == ErrorCode::Unsupported);
    CHECK(codeOf(Json{{"format", "other"}, {"version", 1}}) == ErrorCode::Parse);
    CHECK(codeOf(Json{{"format", "oxcam.test"}, {"version", "1"}}) == ErrorCode::Parse);
    CHECK(codeOf(Json::array()) == ErrorCode::Parse);

    FormatSpec gap{"oxcam.test", 3, {{2, [](Json&) {
                                      }}}};
    Json old{{"format", "oxcam.test"}, {"version", 1}};
    CHECK_THROWS_AS(gap.upgrade(old, "gap"), Error);
}

TEST_CASE("a format's former name reads as its name", "[json]") {
    FormatSpec spec{"oxcam.test", 2, {}, {"old.test"}};
    spec.migrations[1] = [](Json& d) {
        d["added_in_2"] = true;
    };
    Json former{{"format", "old.test"}, {"version", 1}};
    CHECK(spec.upgrade(former, "former") == 1);
    CHECK(former.at("format") == "oxcam.test");
    CHECK(former.at("added_in_2") == true);
    Json other{{"format", "other.test"}, {"version", 2}};
    CHECK_THROWS_AS(spec.upgrade(other, "other"), Error);
}
