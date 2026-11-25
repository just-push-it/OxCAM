// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// A test hook: drives the tool library dialog's holder widgets the way a user's edits do (each
// widget's own signals, no mouse), checks the library after each step, and saves a screenshot of
// each step.

#pragma once

#include <QString>

namespace ocam::gui {

/// Returns the number of failed checks; each failure is printed. Screenshots go in `directory`.
int checkHolderEditor(const QString& directory);

} // namespace ocam::gui
