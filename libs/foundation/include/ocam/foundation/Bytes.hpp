// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Little-endian binary encoding for caches that can always be thrown away (the local simulation
// cache). Documents and libraries are JSON; nothing the user keeps is written this way.

#pragma once

#include "ocam/foundation/Error.hpp"
#include "ocam/foundation/Math.hpp"

#include <cstdint>
#include <cstring>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

namespace ocam {

class ByteWriter {
public:
    template <typename T> void pod(const T& value) {
        static_assert(std::is_trivially_copyable_v<T>);
        out_.append(reinterpret_cast<const char*>(&value), sizeof(T));
    }
    void u8(std::uint8_t v) { pod(v); }
    void u32(std::uint32_t v) { pod(v); }
    void u64(std::uint64_t v) { pod(v); }
    void f32(float v) { pod(v); }
    void f64(double v) { pod(v); }
    void boolean(bool v) { u8(v ? 1 : 0); }
    void str(std::string_view s) {
        u64(s.size());
        out_.append(s);
    }
    void vec2(const Vec2& v) {
        f64(v.x());
        f64(v.y());
    }
    void vec3(const Vec3& v) {
        f64(v.x());
        f64(v.y());
        f64(v.z());
    }
    void transform(const Transform3& t) {
        for (int k = 0; k < 16; ++k) {
            f64(t.matrix().data()[k]);
        }
    }
    /// A count, then the items' bytes.
    template <typename T> void array(const std::vector<T>& items) {
        static_assert(std::is_trivially_copyable_v<T>);
        u64(items.size());
        out_.append(reinterpret_cast<const char*>(items.data()), items.size() * sizeof(T));
    }

    const std::string& bytes() const noexcept { return out_; }
    std::string take() { return std::move(out_); }

private:
    std::string out_;
};

/// Reads what ByteWriter wrote. Reading past the end throws Error(Parse), so damaged or
/// truncated data is never trusted.
class ByteReader {
public:
    explicit ByteReader(std::string_view in) : in_(in) {}

    template <typename T> T pod() {
        static_assert(std::is_trivially_copyable_v<T>);
        need(sizeof(T));
        T value;
        std::memcpy(&value, in_.data() + at_, sizeof(T));
        at_ += sizeof(T);
        return value;
    }
    std::uint8_t u8() { return pod<std::uint8_t>(); }
    std::uint32_t u32() { return pod<std::uint32_t>(); }
    std::uint64_t u64() { return pod<std::uint64_t>(); }
    float f32() { return pod<float>(); }
    double f64() { return pod<double>(); }
    bool boolean() { return u8() != 0; }
    std::string str() {
        const std::size_t n = count(1);
        std::string s(in_.substr(at_, n));
        at_ += n;
        return s;
    }
    Vec2 vec2() {
        const double x = f64();
        return {x, f64()};
    }
    Vec3 vec3() {
        const double x = f64();
        const double y = f64();
        return {x, y, f64()};
    }
    Transform3 transform() {
        Transform3 t;
        for (int k = 0; k < 16; ++k) {
            t.matrix().data()[k] = f64();
        }
        return t;
    }
    template <typename T> std::vector<T> array() {
        static_assert(std::is_trivially_copyable_v<T>);
        const std::size_t n = count(sizeof(T));
        std::vector<T> items(n);
        std::memcpy(items.data(), in_.data() + at_, n * sizeof(T));
        at_ += n * sizeof(T);
        return items;
    }
    /// A count of items of at least `itemSize` bytes each, which must still fit in the data.
    std::size_t count(std::size_t itemSize) {
        const std::uint64_t n = u64();
        if (itemSize > 0 && n > (in_.size() - at_) / itemSize) {
            fail();
        }
        return static_cast<std::size_t>(n);
    }
    bool atEnd() const noexcept { return at_ == in_.size(); }

private:
    void need(std::size_t n) const {
        if (n > in_.size() - at_) {
            fail();
        }
    }
    [[noreturn]] static void fail() {
        throw Error(ErrorCode::Parse, "truncated or damaged binary data");
    }

    std::string_view in_;
    std::size_t at_ = 0;
};

} // namespace ocam
