// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Edit > Preferences: general settings, keyboard shortcuts of the commands, the commands pinned
// to the toolbar, and the starting values of new operations by type. Nothing is saved until OK.

#pragma once

#include "Commands.hpp"
#include "ViewColors.hpp"

#include <ocam/foundation/Json.hpp>
#include <ocam/foundation/Units.hpp>

#include <QDialog>
#include <map>
#include <optional>
#include <string>

class QCheckBox;
class QComboBox;
class QKeySequenceEdit;
class QLineEdit;
class QListWidget;
class QPushButton;
class QScrollArea;
class QTabWidget;
class QTreeWidget;
class QTreeWidgetItem;

namespace ocam::gui {

class PreferencesDialog : public QDialog {
    Q_OBJECT

public:
    enum Page { General, Shortcuts, Toolbar, OperationDefaults, Colors };

    explicit PreferencesDialog(const CommandRegistry& commands, QWidget* parent = nullptr);

    void showPage(Page page);
    /// Shows the defaults of one operation type on the Operation Defaults page.
    void showOperationType(const std::string& type);
    /// Saves everything, then closes.
    void accept() override;

private:
    QWidget* buildGeneral();
    QWidget* buildShortcuts();
    QWidget* buildToolbar();
    QWidget* buildDefaults();
    QWidget* buildColors();
    LengthUnit displayUnit() const;

    const CommandRegistry& commands_;
    QTabWidget* tabs_ = nullptr;

    // General
    QComboBox* unit_ = nullptr;
    QComboBox* library_ = nullptr;
    QComboBox* language_ = nullptr;
    QCheckBox* hub_ = nullptr;

    // Keyboard shortcuts: the bindings as edited (nullopt: the command's defaults).
    std::map<QString, std::optional<QKeySequence>> bindings_;
    QTreeWidget* shortcutTree_ = nullptr;
    QKeySequenceEdit* keyEdit_ = nullptr;
    QPushButton* clearKey_ = nullptr;
    QPushButton* resetKey_ = nullptr;
    QList<QKeySequence> shortcutsOf(const Command& command) const;
    const Command* currentCommand() const;
    void assignShortcut(const QKeySequence& shortcut);
    void refreshShortcutRows();
    void filterShortcuts(const QString& text);

    // Toolbar
    QTreeWidget* available_ = nullptr;
    QListWidget* pinned_ = nullptr;
    void pin(const QString& id);
    void movePinned(int delta);

    // Operation defaults, by type, as edited (loaded on first view).
    std::map<std::string, Json> defaults_;
    QListWidget* types_ = nullptr;
    QScrollArea* defaultsPage_ = nullptr;
    std::string shownType_;

    // Colors, as edited.
    ViewColors colors_;
    Json& defaultsFor(const std::string& type);
    void rebuildDefaultsPage();
    void markTypes();
};

} // namespace ocam::gui
