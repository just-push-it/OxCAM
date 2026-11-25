// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/foundation/Error.hpp"

namespace ocam {

std::string_view toString(ErrorCode code) noexcept {
    switch (code) {
    case ErrorCode::InvalidArgument:
        return "invalid argument";
    case ErrorCode::Io:
        return "I/O error";
    case ErrorCode::Parse:
        return "parse error";
    case ErrorCode::Unsupported:
        return "unsupported";
    case ErrorCode::Cancelled:
        return "cancelled";
    case ErrorCode::Internal:
        return "internal error";
    }
    return "unknown error";
}

} // namespace ocam
