// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The one "Show built-ins" check box of every dialog that lists the examples OxCAM comes with
// next to the user's own entries (machines, turret blocks and centres, posts). Off by default:
// the user's own entries are what is listed. A dialog opened on a built-in ticks it, so what it
// is opened on is shown.

#pragma once

#include <QCheckBox>
#include <QCoreApplication>

namespace ocam::gui {

inline QCheckBox* newShowBuiltinsBox(QWidget* parent = nullptr) {
    auto* box =
        new QCheckBox(QCoreApplication::translate("ShowBuiltins", "Show built-ins"), parent);
    box->setObjectName(QStringLiteral("showBuiltins"));
    box->setToolTip(QCoreApplication::translate(
        "ShowBuiltins", "Also list the examples that come with OxCAM, next to your own"));
    return box;
}

} // namespace ocam::gui
