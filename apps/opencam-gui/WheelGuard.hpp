// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The mouse wheel changes a value field (spin box, combo box, slider) only when the field has
// the keyboard focus, so scrolling a page over a field scrolls the page instead of changing the
// value unseen. Installed on the application.

#pragma once

#include <QObject>

namespace ocam::gui {

class WheelGuard : public QObject {
    Q_OBJECT

public:
    explicit WheelGuard(QObject* parent = nullptr);

    /// Installs a guard on the application (once).
    static void install();

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;
};

} // namespace ocam::gui
