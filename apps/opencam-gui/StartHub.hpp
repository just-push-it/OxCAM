// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The start hub shown when OxCAM opens without a file: start a project, open a recent one,
// tips and what is new in this version. Recent projects are kept in the settings
// ("recentProjects"), most recent first.

#pragma once

#include <QDialog>
#include <QStringList>

class QCheckBox;
class QLabel;
class QListWidget;

namespace ocam::gui {

/// The recent project list in the settings.
class RecentProjects {
public:
    static QStringList list();
    /// Moves (or adds) a path to the front; keeps at most 12 entries.
    static void add(const QString& path);
    static void remove(const QString& path);
};

class StartHub : public QDialog {
    Q_OBJECT

public:
    enum class Action {
        None,
        NewProject,
        OpenProject,
        ImportStep,
        OpenRecent,
        ToolLibraries,
        MachineLibrary
    };

    explicit StartHub(QWidget* parent = nullptr);

    Action action() const noexcept { return action_; }
    /// For OpenRecent: the project to open.
    QString path() const { return path_; }

    /// Whether the hub opens at startup (settings "showStartHub", default true).
    static bool showAtStartup();

private:
    void finish(Action action, const QString& path = {});
    void fillRecent();
    void showTip(int index);

    Action action_ = Action::None;
    QString path_;
    QListWidget* recent_ = nullptr;
    QLabel* tip_ = nullptr;
    QCheckBox* showAtStart_ = nullptr;
    int tipIndex_ = 0;
};

} // namespace ocam::gui
