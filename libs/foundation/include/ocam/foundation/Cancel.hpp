// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <atomic>

namespace ocam {

/// Cooperative cancellation flag shared between a caller (usually the UI) and a long-running
/// kernel. Kernels poll it at convenient points, for example once per Z level or per tile.
class CancelToken {
public:
    CancelToken() = default;
    CancelToken(const CancelToken&) = delete;
    CancelToken& operator=(const CancelToken&) = delete;

    void requestCancel() noexcept { cancelled_.store(true, std::memory_order_relaxed); }
    bool isCancelled() const noexcept { return cancelled_.load(std::memory_order_relaxed); }

    /// Throws OperationCancelled if cancellation was requested.
    void throwIfCancelled() const;

private:
    std::atomic<bool> cancelled_{false};
};

} // namespace ocam
