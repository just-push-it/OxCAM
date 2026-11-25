// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <string_view>

namespace ocam {

/// Receives progress from long-running kernels. Implementations must be cheap and thread-safe
/// if a kernel reports from worker threads.
class ProgressSink {
public:
    virtual ~ProgressSink() = default;

    /// `fraction` is in [0, 1]. `stage` is a short human-readable label and may be empty.
    virtual void report(double fraction, std::string_view stage) = 0;
};

/// Discards all progress.
class NullProgress final : public ProgressSink {
public:
    void report(double /*fraction*/, std::string_view /*stage*/) override {}
};

/// Maps a sub-task's [0, 1] onto [begin, end] of a parent sink, so nested kernels can report
/// without knowing where they sit in the overall job. Reported values are clamped and never
/// move backwards.
class ProgressRange final : public ProgressSink {
public:
    ProgressRange(ProgressSink& parent, double begin, double end) noexcept;

    void report(double fraction, std::string_view stage) override;

private:
    ProgressSink& parent_;
    double begin_;
    double end_;
    double last_ = -1.0;
};

} // namespace ocam
