// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Straightforward implementation of FIPS 180-4, section 6.2.

#include "ocam/foundation/Sha256.hpp"

#include <algorithm>
#include <bit>
#include <cstring>

namespace ocam {
namespace {

// First 32 bits of the fractional parts of the cube roots of the first 64 primes.
constexpr std::array<std::uint32_t, 64> kRoundConstants = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};

} // namespace

Sha256::Sha256() noexcept
    // First 32 bits of the fractional parts of the square roots of the first 8 primes.
    : state_{0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
             0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19} {}

void Sha256::compress(const std::uint8_t* block) noexcept {
    std::array<std::uint32_t, 64> w{};
    for (std::size_t i = 0; i < 16; ++i) {
        w[i] = (std::uint32_t{block[4 * i]} << 24) | (std::uint32_t{block[4 * i + 1]} << 16) |
               (std::uint32_t{block[4 * i + 2]} << 8) | std::uint32_t{block[4 * i + 3]};
    }
    for (std::size_t i = 16; i < 64; ++i) {
        const std::uint32_t s0 =
            std::rotr(w[i - 15], 7) ^ std::rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
        const std::uint32_t s1 =
            std::rotr(w[i - 2], 17) ^ std::rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
        w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }

    auto [a, b, c, d, e, f, g, h] = state_;
    for (std::size_t i = 0; i < 64; ++i) {
        const std::uint32_t s1 = std::rotr(e, 6) ^ std::rotr(e, 11) ^ std::rotr(e, 25);
        const std::uint32_t ch = (e & f) ^ (~e & g);
        const std::uint32_t t1 = h + s1 + ch + kRoundConstants[i] + w[i];
        const std::uint32_t s0 = std::rotr(a, 2) ^ std::rotr(a, 13) ^ std::rotr(a, 22);
        const std::uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
        const std::uint32_t t2 = s0 + maj;
        h = g;
        g = f;
        f = e;
        e = d + t1;
        d = c;
        c = b;
        b = a;
        a = t1 + t2;
    }
    state_[0] += a;
    state_[1] += b;
    state_[2] += c;
    state_[3] += d;
    state_[4] += e;
    state_[5] += f;
    state_[6] += g;
    state_[7] += h;
}

void Sha256::update(std::span<const std::byte> data) noexcept {
    totalBytes_ += data.size();
    const auto* p = reinterpret_cast<const std::uint8_t*>(data.data());
    std::size_t n = data.size();
    if (bufferSize_ > 0) {
        const std::size_t take = std::min(n, buffer_.size() - bufferSize_);
        std::memcpy(buffer_.data() + bufferSize_, p, take);
        bufferSize_ += take;
        p += take;
        n -= take;
        if (bufferSize_ == buffer_.size()) {
            compress(buffer_.data());
            bufferSize_ = 0;
        }
    }
    while (n >= 64) {
        compress(p);
        p += 64;
        n -= 64;
    }
    if (n > 0) {
        std::memcpy(buffer_.data(), p, n);
        bufferSize_ = n;
    }
}

Sha256::Digest Sha256::finish() noexcept {
    const std::uint64_t bitLength = totalBytes_ * 8;
    buffer_[bufferSize_++] = 0x80;
    if (bufferSize_ > 56) {
        std::memset(buffer_.data() + bufferSize_, 0, 64 - bufferSize_);
        compress(buffer_.data());
        bufferSize_ = 0;
    }
    std::memset(buffer_.data() + bufferSize_, 0, 56 - bufferSize_);
    for (int i = 0; i < 8; ++i) {
        buffer_[56 + static_cast<std::size_t>(i)] =
            static_cast<std::uint8_t>(bitLength >> (56 - 8 * i));
    }
    compress(buffer_.data());

    Digest out{};
    for (std::size_t i = 0; i < 8; ++i) {
        out[4 * i] = static_cast<std::uint8_t>(state_[i] >> 24);
        out[4 * i + 1] = static_cast<std::uint8_t>(state_[i] >> 16);
        out[4 * i + 2] = static_cast<std::uint8_t>(state_[i] >> 8);
        out[4 * i + 3] = static_cast<std::uint8_t>(state_[i]);
    }
    return out;
}

Sha256::Digest Sha256::hash(std::string_view data) noexcept {
    Sha256 h;
    h.update(data);
    return h.finish();
}

std::string Sha256::toHex(const Digest& digest) {
    constexpr char kHex[] = "0123456789abcdef";
    std::string s;
    s.reserve(64);
    for (auto b : digest) {
        s.push_back(kHex[b >> 4]);
        s.push_back(kHex[b & 0x0F]);
    }
    return s;
}

std::string sha256Hex(std::string_view data) {
    return Sha256::toHex(Sha256::hash(data));
}

} // namespace ocam
