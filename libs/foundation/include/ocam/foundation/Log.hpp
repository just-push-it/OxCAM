// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <fmt/format.h>

#include <functional>
#include <string_view>
#include <utility>

namespace ocam {

enum class LogLevel { Trace, Debug, Info, Warn, Error, Off };

using LogSink = std::function<void(LogLevel, std::string_view)>;

/// Replaces the process-wide sink. Passing an empty function restores the default (stderr).
/// The sink may be called from any thread; calls are serialized.
void setLogSink(LogSink sink);

/// Messages below this level are dropped before formatting. Default: Info.
void setLogLevel(LogLevel level) noexcept;
LogLevel logLevel() noexcept;

std::string_view toString(LogLevel level) noexcept;

void logMessage(LogLevel level, std::string_view message);

template <typename... Args>
void log(LogLevel level, fmt::format_string<Args...> format, Args&&... args) {
    if (level < logLevel()) {
        return;
    }
    logMessage(level, fmt::format(format, std::forward<Args>(args)...));
}

template <typename... Args> void logDebug(fmt::format_string<Args...> f, Args&&... args) {
    log(LogLevel::Debug, f, std::forward<Args>(args)...);
}
template <typename... Args> void logInfo(fmt::format_string<Args...> f, Args&&... args) {
    log(LogLevel::Info, f, std::forward<Args>(args)...);
}
template <typename... Args> void logWarn(fmt::format_string<Args...> f, Args&&... args) {
    log(LogLevel::Warn, f, std::forward<Args>(args)...);
}
template <typename... Args> void logError(fmt::format_string<Args...> f, Args&&... args) {
    log(LogLevel::Error, f, std::forward<Args>(args)...);
}

} // namespace ocam
