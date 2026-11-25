// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <QDialog>
#include <QStringList>

namespace ocam::gui {

/// Shows a posted program with its warnings and lets the user save it.
class PostDialog : public QDialog {
    Q_OBJECT

public:
    PostDialog(const QString& gcode, const QStringList& warnings, const QString& suggestedName,
               const QString& extension, QWidget* parent = nullptr);

private:
    void save();

    QString gcode_;
    QString suggestedName_;
    QString extension_;
};

} // namespace ocam::gui
