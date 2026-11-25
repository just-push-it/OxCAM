// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/foundation/Json.hpp"

#include "ocam/foundation/Error.hpp"
#include "ocam/foundation/Uuid.hpp"

#include <fmt/format.h>
#include <nlohmann/json-schema.hpp>

#include <algorithm>
#include <fstream>
#include <sstream>
#include <vector>

namespace ocam {
namespace {

std::string displayPath(const std::filesystem::path& path) {
    const std::u8string u8 = path.u8string();
    return {u8.begin(), u8.end()};
}

// Collects every violation instead of stopping at the first.
class CollectingErrorHandler : public nlohmann::json_schema::basic_error_handler {
public:
    void error(const Json::json_pointer& pointer, const Json& /*instance*/,
               const std::string& message) override {
        basic_error_handler::error(pointer, {}, message);
        const std::string where = pointer.to_string();
        messages.push_back(fmt::format("{}: {}", where.empty() ? "/" : where, message));
    }
    std::vector<std::string> messages;
};

} // namespace

std::string readFileBytes(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        throw Error(ErrorCode::Io, "cannot open file: " + displayPath(path));
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    if (in.bad()) {
        throw Error(ErrorCode::Io, "error reading file: " + displayPath(path));
    }
    return std::move(ss).str();
}

void writeFileAtomic(const std::filesystem::path& path, std::string_view bytes) {
    std::filesystem::path tmp = path;
    tmp += ".tmp-" + Uuid::generate().toString().substr(0, 8);
    {
        std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
        if (!out) {
            throw Error(ErrorCode::Io, "cannot create file: " + displayPath(tmp));
        }
        out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
        out.flush();
        if (!out) {
            std::error_code ignored;
            std::filesystem::remove(tmp, ignored);
            throw Error(ErrorCode::Io, "error writing file: " + displayPath(tmp));
        }
    }
    std::error_code ec;
    std::filesystem::rename(tmp, path, ec);
    if (ec) {
        std::error_code ignored;
        std::filesystem::remove(tmp, ignored);
        throw Error(ErrorCode::Io,
                    fmt::format("cannot replace {}: {}", displayPath(path), ec.message()));
    }
}

Json parseJson(std::string_view text, std::string_view sourceName) {
    try {
        return Json::parse(text);
    } catch (const Json::exception& e) {
        // parse_error, and out_of_range for a number literal too large to hold ("4e720").
        throw Error(ErrorCode::Parse, fmt::format("{}: invalid JSON: {}", sourceName, e.what()));
    }
}

std::string dumpJson(const Json& doc) {
    return doc.dump(2) + "\n";
}

struct SchemaValidator::Impl {
    nlohmann::json_schema::json_validator validator;
};

SchemaValidator::SchemaValidator(const Json& schema) : impl_(std::make_unique<Impl>()) {
    try {
        impl_->validator.set_root_schema(schema);
    } catch (const std::exception& e) {
        throw Error(ErrorCode::Internal, fmt::format("invalid JSON schema: {}", e.what()));
    }
}

SchemaValidator::~SchemaValidator() = default;
SchemaValidator::SchemaValidator(SchemaValidator&&) noexcept = default;
SchemaValidator& SchemaValidator::operator=(SchemaValidator&&) noexcept = default;

void SchemaValidator::validate(const Json& doc, std::string_view sourceName) const {
    CollectingErrorHandler handler;
    impl_->validator.validate(doc, handler);
    if (handler) {
        std::string text = fmt::format("{}: does not match the schema", sourceName);
        for (const auto& m : handler.messages) {
            text += "\n  " + m;
        }
        throw Error(ErrorCode::Parse, text);
    }
}

int FormatSpec::upgrade(Json& doc, std::string_view sourceName) const {
    if (!doc.is_object()) {
        throw Error(ErrorCode::Parse, fmt::format("{}: expected a JSON object", sourceName));
    }
    const auto formatIt = doc.find("format");
    if (formatIt == doc.end() || !formatIt->is_string()) {
        throw Error(ErrorCode::Parse, fmt::format("{}: not a '{}' file", sourceName, format));
    }
    if (const auto found = formatIt->get<std::string>(); found != format) {
        if (std::find(formerNames.begin(), formerNames.end(), found) == formerNames.end()) {
            throw Error(ErrorCode::Parse, fmt::format("{}: not a '{}' file", sourceName, format));
        }
        *formatIt = format;
    }
    const auto versionIt = doc.find("version");
    if (versionIt == doc.end() || !versionIt->is_number_integer()) {
        throw Error(ErrorCode::Parse, fmt::format("{}: missing integer 'version'", sourceName));
    }
    const int found = versionIt->get<int>();
    if (found > currentVersion) {
        throw Error(ErrorCode::Unsupported,
                    fmt::format("{}: '{}' version {} is newer than this build supports ({})",
                                sourceName, format, found, currentVersion));
    }
    if (found < 1) {
        throw Error(ErrorCode::Parse, fmt::format("{}: invalid version {}", sourceName, found));
    }
    for (int v = found; v < currentVersion; ++v) {
        const auto it = migrations.find(v);
        if (it == migrations.end()) {
            throw Error(ErrorCode::Unsupported, fmt::format("{}: no migration from '{}' version {}",
                                                            sourceName, format, v));
        }
        it->second(doc);
        doc["version"] = v + 1;
    }
    return found;
}

} // namespace ocam
