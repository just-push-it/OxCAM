// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdexcept>
#include <string>
#include <string_view>

namespace ocam {

enum class ErrorCode {
    InvalidArgument, ///< Caller passed a value outside the documented domain.
    Io,              ///< File or stream failure.
    Parse,           ///< Malformed input file.
    Unsupported,     ///< Valid input that this version cannot handle.
    Cancelled,       ///< Work stopped through a CancelToken.
    Internal,        ///< Broken invariant; always a bug.
};

std::string_view toString(ErrorCode code) noexcept;

/// Base exception for all OxCAM library errors.
class Error : public std::runtime_error {
public:
    Error(ErrorCode code, const std::string& message) : std::runtime_error(message), code_(code) {}

    ErrorCode code() const noexcept { return code_; }

private:
    ErrorCode code_;
};

/// Thrown by CancelToken::throwIfCancelled().
class OperationCancelled : public Error {
public:
    OperationCancelled() : Error(ErrorCode::Cancelled, "operation cancelled") {}
};

} // namespace ocam
