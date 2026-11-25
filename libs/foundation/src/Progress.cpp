// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/foundation/Progress.hpp"

#include <algorithm>

namespace ocam {

ProgressRange::ProgressRange(ProgressSink& parent, double begin, double end) noexcept
    : parent_(parent), begin_(std::clamp(begin, 0.0, 1.0)), end_(std::clamp(end, begin_, 1.0)) {}

void ProgressRange::report(double fraction, std::string_view stage) {
    const double mapped = begin_ + (end_ - begin_) * std::clamp(fraction, 0.0, 1.0);
    if (mapped < last_) {
        return;
    }
    last_ = mapped;
    parent_.report(mapped, stage);
}

} // namespace ocam
