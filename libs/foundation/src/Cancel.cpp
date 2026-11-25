// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/foundation/Cancel.hpp"

#include "ocam/foundation/Error.hpp"

namespace ocam {

void CancelToken::throwIfCancelled() const {
    if (isCancelled()) {
        throw OperationCancelled();
    }
}

} // namespace ocam
