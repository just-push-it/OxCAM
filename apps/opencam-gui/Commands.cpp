// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "Commands.hpp"

#include <QAction>
#include <QSettings>
#include <QWidget>
#include <algorithm>

namespace ocam::gui {
namespace {

QString shortcutKey(const QString& id) {
    return QStringLiteral("shortcuts/") + id;
}

} // namespace

bool sharesKeysWith(const Command& a, const Command& b) {
    return !a.optionGroup.isEmpty() && a.optionGroup == b.optionGroup;
}

std::optional<QKeySequence> customShortcut(const QString& id) {
    const QSettings settings;
    if (!settings.contains(shortcutKey(id))) {
        return std::nullopt;
    }
    return QKeySequence::fromString(settings.value(shortcutKey(id)).toString(),
                                    QKeySequence::PortableText);
}

void setCustomShortcut(const QString& id, const std::optional<QKeySequence>& shortcut) {
    QSettings settings;
    if (shortcut) {
        settings.setValue(shortcutKey(id), shortcut->toString(QKeySequence::PortableText));
    } else {
        settings.remove(shortcutKey(id));
    }
}

QList<QKeySequence> shortcutsWith(const Command& command,
                                  const std::optional<QKeySequence>& custom) {
    if (!custom) {
        return command.defaultShortcuts;
    }
    return custom->isEmpty() ? QList<QKeySequence>() : QList<QKeySequence>{*custom};
}

QStringList pinnedCommands() {
    return QSettings().value(QStringLiteral("pinnedCommands")).toStringList();
}

void setPinnedCommands(const QStringList& ids) {
    QSettings().setValue(QStringLiteral("pinnedCommands"), ids);
}

QAction* CommandRegistry::add(const QString& id, const QString& category, const QString& label,
                              QAction* action, const QString& optionGroup) {
    commands_.push_back({id, category, label, action, action->shortcuts(), optionGroup});
    bound_.push_back(shortcutsWith(commands_.back(), customShortcut(id)));
    action->setShortcuts(bound_.back()); // keys shared with others: applyShortcuts()
    // In the window as well as in its menus: the shortcut works while a dock has the focus,
    // and for commands no menu shows.
    window_->addAction(action);
    if (!optionGroup.isEmpty()) {
        // A shared key moves on from the option run last (for options that are not checked).
        QObject::connect(action, &QAction::triggered, window_,
                         [this, id, optionGroup] { lastRun_[optionGroup] = id; });
    }
    return action;
}

const Command* CommandRegistry::find(const QString& id) const {
    const auto it = std::find_if(commands_.begin(), commands_.end(),
                                 [&](const Command& c) { return c.id == id; });
    return it == commands_.end() ? nullptr : &*it;
}

QList<QKeySequence> CommandRegistry::shortcutsOf(const Command& command) const {
    const auto index = static_cast<std::size_t>(&command - commands_.data());
    return index < bound_.size() ? bound_[index] : QList<QKeySequence>();
}

void CommandRegistry::applyShortcuts() {
    bound_.clear();
    for (const Command& command : commands_) {
        bound_.push_back(shortcutsWith(command, customShortcut(command.id)));
    }
    std::vector<QList<QKeySequence>> own = bound_;

    // Keys that several commands of one option group share: a cycler steps through them.
    for (QAction* cycler : cyclers_) {
        window_->removeAction(cycler);
        delete cycler;
    }
    cyclers_.clear();
    std::map<QString, std::vector<std::size_t>> byKey; // portable key text -> commands
    for (std::size_t i = 0; i < bound_.size(); ++i) {
        for (const QKeySequence& key : bound_[i]) {
            byKey[key.toString(QKeySequence::PortableText)].push_back(i);
        }
    }
    for (const auto& [text, members] : byKey) {
        if (members.size() < 2 || !std::all_of(members.begin(), members.end(), [&](std::size_t i) {
                return sharesKeysWith(commands_[i], commands_[members.front()]);
            })) {
            continue; // a key of one command (or a clash Qt reports as ambiguous)
        }
        const QKeySequence key = QKeySequence::fromString(text, QKeySequence::PortableText);
        for (const std::size_t i : members) {
            own[i].removeAll(key);
        }
        auto* cycler = new QAction(window_);
        cycler->setShortcut(key);
        QObject::connect(cycler, &QAction::triggered, window_,
                         [this, group = members] { cycle(group); });
        window_->addAction(cycler);
        cyclers_.push_back(cycler);
    }
    for (std::size_t i = 0; i < commands_.size(); ++i) {
        commands_[i].action->setShortcuts(own[i]);
    }
}

void CommandRegistry::cycle(const std::vector<std::size_t>& members) {
    // The option chosen now: a checked one, else the one run last.
    std::optional<std::size_t> current;
    for (std::size_t k = 0; k < members.size(); ++k) {
        const QAction* a = commands_[members[k]].action;
        if (a->isCheckable() && a->isChecked()) {
            current = k;
        }
    }
    if (!current) {
        const auto last = lastRun_.find(commands_[members.front()].optionGroup);
        for (std::size_t k = 0; last != lastRun_.end() && k < members.size(); ++k) {
            if (commands_[members[k]].id == last->second) {
                current = k;
            }
        }
    }
    const std::size_t next = current ? (*current + 1) % members.size() : 0;
    QAction* action = commands_[members[next]].action;
    if (action->isEnabled()) {
        action->trigger();
    }
}

} // namespace ocam::gui
