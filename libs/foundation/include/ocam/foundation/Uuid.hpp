// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <array>
#include <compare>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <string_view>

namespace ocam {

/// RFC 9562 UUID. Identifies tools, holders, setups and operations in persisted files.
class Uuid {
public:
    using Bytes = std::array<std::uint8_t, 16>;

    /// The nil UUID (all zeros).
    constexpr Uuid() noexcept = default;
    constexpr explicit Uuid(const Bytes& bytes) noexcept : bytes_(bytes) {}

    /// Random (version 4) UUID.
    static Uuid generate();

    /// Parses the canonical 8-4-4-4-12 hex form (either case). Braces are not accepted.
    static std::optional<Uuid> parse(std::string_view text) noexcept;

    /// Canonical lowercase 8-4-4-4-12 form.
    std::string toString() const;

    constexpr bool isNil() const noexcept {
        for (auto b : bytes_) {
            if (b != 0) {
                return false;
            }
        }
        return true;
    }

    constexpr int version() const noexcept { return bytes_[6] >> 4; }
    constexpr const Bytes& bytes() const noexcept { return bytes_; }

    friend constexpr auto operator<=>(const Uuid&, const Uuid&) noexcept = default;

private:
    Bytes bytes_{};
};

} // namespace ocam

template <> struct std::hash<ocam::Uuid> {
    std::size_t operator()(const ocam::Uuid& id) const noexcept {
        // FNV-1a over the 16 bytes; UUIDs are already well distributed.
        std::uint64_t h = 1469598103934665603ULL;
        for (auto b : id.bytes()) {
            h = (h ^ b) * 1099511628211ULL;
        }
        return static_cast<std::size_t>(h);
    }
};
