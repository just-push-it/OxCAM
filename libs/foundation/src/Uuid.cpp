// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/foundation/Uuid.hpp"

#include <random>

namespace ocam {
namespace {

std::mt19937_64& threadRng() {
    thread_local std::mt19937_64 rng = [] {
        std::random_device rd;
        std::seed_seq seq{rd(), rd(), rd(), rd(), rd(), rd(), rd(), rd()};
        return std::mt19937_64(seq);
    }();
    return rng;
}

int hexValue(char c) noexcept {
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    return -1;
}

constexpr bool isDashPosition(std::size_t i) noexcept {
    return i == 8 || i == 13 || i == 18 || i == 23;
}

} // namespace

Uuid Uuid::generate() {
    auto& rng = threadRng();
    Bytes bytes{};
    for (std::size_t i = 0; i < bytes.size(); i += 8) {
        auto word = rng();
        for (std::size_t j = 0; j < 8; ++j) {
            bytes[i + j] = static_cast<std::uint8_t>(word >> (8 * j));
        }
    }
    bytes[6] = static_cast<std::uint8_t>((bytes[6] & 0x0F) | 0x40); // version 4
    bytes[8] = static_cast<std::uint8_t>((bytes[8] & 0x3F) | 0x80); // RFC variant
    return Uuid(bytes);
}

std::optional<Uuid> Uuid::parse(std::string_view text) noexcept {
    if (text.size() != 36) {
        return std::nullopt;
    }
    Bytes bytes{};
    std::size_t out = 0;
    for (std::size_t i = 0; i < text.size();) {
        if (isDashPosition(i)) {
            if (text[i] != '-') {
                return std::nullopt;
            }
            ++i;
            continue;
        }
        const int hi = hexValue(text[i]);
        const int lo = hexValue(text[i + 1]);
        if (hi < 0 || lo < 0 || isDashPosition(i + 1)) {
            return std::nullopt;
        }
        bytes[out++] = static_cast<std::uint8_t>((hi << 4) | lo);
        i += 2;
    }
    return Uuid(bytes);
}

std::string Uuid::toString() const {
    constexpr char kHex[] = "0123456789abcdef";
    std::string s;
    s.reserve(36);
    for (std::size_t i = 0; i < bytes_.size(); ++i) {
        if (i == 4 || i == 6 || i == 8 || i == 10) {
            s.push_back('-');
        }
        s.push_back(kHex[bytes_[i] >> 4]);
        s.push_back(kHex[bytes_[i] & 0x0F]);
    }
    return s;
}

} // namespace ocam
