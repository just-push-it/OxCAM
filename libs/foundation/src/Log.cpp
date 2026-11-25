// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/foundation/Log.hpp"

#include <atomic>
#include <cstdio>
#include <mutex>

namespace ocam {
namespace {

struct LogState {
    std::mutex mutex;
    LogSink sink;
    std::atomic<LogLevel> level{LogLevel::Info};
};

LogState& state() {
    static LogState s;
    return s;
}

void defaultSink(LogLevel level, std::string_view message) {
    fmt::print(stderr, "[{}] {}\n", toString(level), message);
}

} // namespace

void setLogSink(LogSink sink) {
    auto& s = state();
    std::lock_guard lock(s.mutex);
    s.sink = std::move(sink);
}

void setLogLevel(LogLevel level) noexcept {
    state().level.store(level);
}

LogLevel logLevel() noexcept {
    return state().level.load();
}

std::string_view toString(LogLevel level) noexcept {
    switch (level) {
    case LogLevel::Trace:
        return "trace";
    case LogLevel::Debug:
        return "debug";
    case LogLevel::Info:
        return "info";
    case LogLevel::Warn:
        return "warn";
    case LogLevel::Error:
        return "error";
    case LogLevel::Off:
        return "off";
    }
    return "unknown";
}

void logMessage(LogLevel level, std::string_view message) {
    auto& s = state();
    if (level < s.level.load() || level == LogLevel::Off) {
        return;
    }
    std::lock_guard lock(s.mutex);
    if (s.sink) {
        s.sink(level, message);
    } else {
        defaultSink(level, message);
    }
}

} // namespace ocam
