// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The main window's commands, each with a stable id. The user binds them to keys and pins them
// to the toolbar (Edit > Preferences); both are kept in the settings by id, so they survive
// changes to menus and labels.
//
// Commands that pick one of several options (the toolpath display, the standard views) form an
// option group. Commands of one group may share a key: each press then moves to the next of
// them, so one key can toggle toolpaths on and off, or step through the views.

#pragma once

#include <QKeySequence>
#include <QList>
#include <QString>
#include <QStringList>
#include <map>
#include <optional>
#include <vector>

class QAction;
class QWidget;

namespace ocam::gui {

struct Command {
    QString id;       ///< e.g. "setup.simulate"
    QString category; ///< e.g. "Setup"
    QString label;    ///< As listed in the preferences.
    QAction* action = nullptr;
    QList<QKeySequence> defaultShortcuts;
    QString optionGroup; ///< Commands of one group can share a key (empty: none).
};

/// Whether two commands may share a key (one option group).
bool sharesKeysWith(const Command& a, const Command& b);

/// The user's key binding of a command (settings "shortcuts/<id>"): nullopt keeps the default
/// shortcuts, an empty sequence is none.
std::optional<QKeySequence> customShortcut(const QString& id);
void setCustomShortcut(const QString& id, const std::optional<QKeySequence>& shortcut);

/// The shortcuts a command has with a binding (nullopt: its defaults).
QList<QKeySequence> shortcutsWith(const Command& command,
                                  const std::optional<QKeySequence>& custom);

/// The command ids pinned to the toolbar, in order (settings "pinnedCommands").
QStringList pinnedCommands();
void setPinnedCommands(const QStringList& ids);

class CommandRegistry {
public:
    /// Shortcuts of registered commands work wherever the focus is in `window`.
    explicit CommandRegistry(QWidget* window) : window_(window) {}

    /// Registers `action` under `id`: its shortcuts now are the defaults, the user's binding
    /// applies. Returns `action`. Commands of one `optionGroup` may share a key; call
    /// applyShortcuts() once all are registered so shared keys step through them.
    QAction* add(const QString& id, const QString& category, const QString& label, QAction* action,
                 const QString& optionGroup = {});
    const std::vector<Command>& commands() const noexcept { return commands_; }
    const Command* find(const QString& id) const;
    /// Applies the user's bindings again (after the preferences changed).
    void applyShortcuts();
    /// The keys bound to a command, shared ones included (the action itself holds only the
    /// keys it has alone).
    QList<QKeySequence> shortcutsOf(const Command& command) const;

private:
    /// Runs the next of `members` (indices into commands_, one option group): the one after
    /// the option chosen now, or after the one run last.
    void cycle(const std::vector<std::size_t>& members);

    QWidget* window_;
    std::vector<Command> commands_;
    std::vector<QList<QKeySequence>> bound_; ///< Per command, as applied.
    std::vector<QAction*> cyclers_;          ///< One per shared key.
    std::map<QString, QString> lastRun_;     ///< Option group -> the command run last.
};

} // namespace ocam::gui
