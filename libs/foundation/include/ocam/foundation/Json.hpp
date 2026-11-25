// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// JSON file I/O shared by every persisted OxCAM format (tool libraries, projects, machines):
// parsing with ocam::Error reporting, deterministic output, atomic writes, JSON Schema
// validation, and version migration.
//
// Every format is a JSON object carrying "format": "<name>" and "version": <int>. Loaders call
// FormatSpec::upgrade() (which applies registered migrations up to the current version), then
// SchemaValidator::validate(), then convert to C++ types.

#pragma once

#include <nlohmann/json.hpp>

#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace ocam {

using Json = nlohmann::json;

/// Reads a whole file. Throws Error(Io).
std::string readFileBytes(const std::filesystem::path& path);

/// Writes via a temporary file in the same directory, then renames over the target, so readers
/// never observe a partially written file. Throws Error(Io).
void writeFileAtomic(const std::filesystem::path& path, std::string_view bytes);

/// Parses JSON text. Throws Error(Parse) mentioning `sourceName`.
Json parseJson(std::string_view text, std::string_view sourceName);

/// Two-space indented, keys sorted, trailing newline: stable output for diffs and golden tests.
std::string dumpJson(const Json& doc);

/// Validates documents against a JSON Schema (draft-07).
class SchemaValidator {
public:
    /// Throws Error(Internal) if the schema itself is invalid.
    explicit SchemaValidator(const Json& schema);
    ~SchemaValidator();
    SchemaValidator(SchemaValidator&&) noexcept;
    SchemaValidator& operator=(SchemaValidator&&) noexcept;

    /// Throws Error(Parse) listing every violation (JSON pointer + message).
    void validate(const Json& doc, std::string_view sourceName) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/// Format identity and its migrations.
struct FormatSpec {
    using Migration = std::function<void(Json&)>;

    std::string format;                  ///< Value of the "format" key.
    int currentVersion;                  ///< Version written by this build.
    std::map<int, Migration> migrations; ///< Key N upgrades a version-N document to N+1.
    /// Names the format was written under before (the LEGACY "opencam.*", ADR-0005): read like
    /// `format`, and renamed to it by upgrade().
    std::vector<std::string> formerNames;

    /// Checks "format" (or a former name, which becomes `format`), then migrates in place to
    /// currentVersion and updates "version".
    /// Returns the version found in the document. Throws Error(Parse) for a missing or wrong
    /// format, Error(Unsupported) for documents newer than this build or with a gap in the
    /// migration chain.
    int upgrade(Json& doc, std::string_view sourceName) const;
};

} // namespace ocam
