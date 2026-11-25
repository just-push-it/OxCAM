// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Tooltips laid out for reading: wrapped at a comfortable width with some padding, and a bold
// title where there is a name to give (the operation, the command, the setting). Plain
// tooltips anywhere in the application get the same layout without a title.

#pragma once

#include <QString>

class QApplication;

namespace ocam::gui {

/// `text` (plain) as a tooltip: wrapped, padded, below a bold `title` when there is one.
QString formatToolTip(const QString& text, const QString& title = {});

/// Lays out the application's plain widget tooltips with formatToolTip(). Tooltips that are
/// already rich text are shown as they are.
void installToolTipFormatting(QApplication& app);

/// The wait before a tooltip appears [ms] (Qt's own is 700).
inline constexpr int kToolTipDelay = 1250;

} // namespace ocam::gui
