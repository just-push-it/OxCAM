// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// SHA-256 (FIPS 180-4). Used to identify embedded model files by content.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>

namespace ocam {

class Sha256 {
public:
    using Digest = std::array<std::uint8_t, 32>;

    Sha256() noexcept;

    void update(std::span<const std::byte> data) noexcept;
    void update(std::string_view data) noexcept { update(std::as_bytes(std::span(data))); }

    /// Finishes the hash. The object must not be updated afterwards.
    Digest finish() noexcept;

    static Digest hash(std::string_view data) noexcept;
    static std::string toHex(const Digest& digest);

private:
    void compress(const std::uint8_t* block) noexcept;

    std::array<std::uint32_t, 8> state_;
    std::array<std::uint8_t, 64> buffer_{};
    std::size_t bufferSize_ = 0;
    std::uint64_t totalBytes_ = 0;
};

/// Lowercase hex SHA-256 of `data`.
std::string sha256Hex(std::string_view data);

} // namespace ocam
