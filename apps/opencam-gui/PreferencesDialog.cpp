// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "PreferencesDialog.hpp"

#include "OperationDefaults.hpp"
#include "ParamEditors.hpp"
#include "SimulationPanel.hpp"
#include "StartHub.hpp"
#include "ToolLibraries.hpp"
#include "ToolTips.hpp"
#include "Translations.hpp"
#include "Units.hpp"

#include <ocam/strategies/Common.hpp>

#include <QAction>
#include <QCheckBox>
#include <QColorDialog>
#include <QComboBox>
#include <QCoreApplication>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QIcon>
#include <QKeySequenceEdit>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPixmap>
#include <QPushButton>
#include <QScrollArea>
#include <QScrollBar>
#include <QSettings>
#include <QTabWidget>
#include <QTimer>
#include <QToolButton>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <algorithm>
#include <filesystem>
#include <functional>
#include <memory>

namespace ocam::gui {
namespace {

constexpr int kIdRole = Qt::UserRole;

QString qs(const std::string& s) {
    return QString::fromStdString(s);
}

std::filesystem::path toPath(const QString& s) {
    return std::filesystem::path(s.toStdU16String());
}

QString fromPath(const std::filesystem::path& p) {
    return QString::fromStdU16String(p.u16string());
}

void setBold(QWidget* w, bool bold) {
    QFont font = w->font();
    font.setBold(bold);
    w->setFont(font);
}

/// The commands grouped by category, categories in the order they first appear.
std::vector<std::pair<QString, std::vector<const Command*>>>
byCategory(const CommandRegistry& registry) {
    std::vector<std::pair<QString, std::vector<const Command*>>> groups;
    for (const Command& c : registry.commands()) {
        auto it = std::find_if(groups.begin(), groups.end(),
                               [&](const auto& g) { return g.first == c.category; });
        if (it == groups.end()) {
            groups.push_back({c.category, {}});
            it = std::prev(groups.end());
        }
        it->second.push_back(&c);
    }
    return groups;
}

QLabel* note(const QString& text) {
    auto* label = new QLabel(text);
    label->setWordWrap(true);
    label->setStyleSheet(QStringLiteral("color: gray;"));
    return label;
}

} // namespace

PreferencesDialog::PreferencesDialog(const CommandRegistry& commands, QWidget* parent)
    : QDialog(parent), commands_(commands) {
    setWindowTitle(tr("Preferences"));
    resize(760, 560);
    auto* layout = new QVBoxLayout(this);
    tabs_ = new QTabWidget;
    tabs_->addTab(buildGeneral(), tr("General"));
    tabs_->addTab(buildShortcuts(), tr("Keyboard Shortcuts"));
    tabs_->addTab(buildToolbar(), tr("Toolbar"));
    tabs_->addTab(buildDefaults(), tr("Operation Defaults"));
    tabs_->addTab(buildColors(), tr("Colors"));
    layout->addWidget(tabs_);
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    layout->addWidget(buttons);
}

void PreferencesDialog::showPage(Page page) {
    tabs_->setCurrentIndex(static_cast<int>(page));
}

LengthUnit PreferencesDialog::displayUnit() const {
    return unit_->currentIndex() == 1 ? LengthUnit::Inch : LengthUnit::Millimeter;
}

// -- General ---------------------------------------------------------------------------------

QWidget* PreferencesDialog::buildGeneral() {
    auto* page = new QWidget;
    auto* form = new QFormLayout(page);
    unit_ = new QComboBox;
    unit_->addItem(tr("Millimetres"));
    unit_->addItem(tr("Inches"));
    unit_->setCurrentIndex(preferredUnit() == LengthUnit::Inch ? 1 : 0);
    unit_->setToolTip(tr("The display unit of new projects and the unit of new tools. Each "
                         "project keeps its own display unit (Project > Display unit)."));
    form->addRow(tr("Default units"), unit_);
    library_ = new QComboBox;
    library_->addItem(tr("(none)"));
    for (const auto& entry : ToolLibraries::load()) {
        library_->addItem(qs(entry.library.name), fromPath(entry.file));
        if (entry.isDefault) {
            library_->setCurrentIndex(library_->count() - 1);
        }
    }
    library_->setToolTip(tr("The library listed first when choosing an operation's tool. Tools > "
                            "Tool Libraries manages the libraries."));
    form->addRow(tr("Default tool library"), library_);
    language_ = new QComboBox;
    language_->addItem(tr("System default"), QString());
    for (const QString& name : availableLanguages()) {
        // Its own name, and the English one when that differs: "Deutsch (German)".
        const QLocale locale(name);
        const QString native = locale.nativeLanguageName();
        const QString english = QLocale::languageToString(locale.language());
        language_->addItem(native.compare(english, Qt::CaseInsensitive) == 0
                               ? native
                               : QStringLiteral("%1 (%2)").arg(native, english),
                           name);
    }
    language_->setCurrentIndex(std::max(0, language_->findData(chosenLanguage())));
    language_->setToolTip(tr("The language of OxCAM's menus, dialogs and tooltips. Takes "
                             "effect the next time OxCAM starts."));
    form->addRow(tr("Language"), language_);
    hub_ = new QCheckBox(tr("Show the start hub when OxCAM starts"));
    hub_->setChecked(StartHub::showAtStartup());
    hub_->setToolTip(tr("The start hub offers new and recent projects, STEP import, tips and "
                        "what is new (File > Start Hub)."));
    form->addRow(tr("Startup"), hub_);
    // The defaults page shows lengths in this unit.
    connect(unit_, &QComboBox::currentIndexChanged, this, [this] { rebuildDefaultsPage(); });
    return page;
}

// -- Keyboard shortcuts ----------------------------------------------------------------------

QWidget* PreferencesDialog::buildShortcuts() {
    for (const Command& c : commands_.commands()) {
        bindings_[c.id] = customShortcut(c.id);
    }
    auto* page = new QWidget;
    auto* layout = new QVBoxLayout(page);
    auto* filter = new QLineEdit;
    filter->setPlaceholderText(tr("Search commands or shortcuts"));
    filter->setClearButtonEnabled(true);
    layout->addWidget(filter);

    shortcutTree_ = new QTreeWidget;
    shortcutTree_->setColumnCount(2);
    shortcutTree_->setHeaderLabels({tr("Command"), tr("Shortcut")});
    shortcutTree_->setRootIsDecorated(true);
    shortcutTree_->setUniformRowHeights(true);
    shortcutTree_->header()->setSectionResizeMode(0, QHeaderView::Stretch);
    shortcutTree_->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    shortcutTree_->header()->setStretchLastSection(false);
    for (const auto& [category, list] : byCategory(commands_)) {
        auto* group = new QTreeWidgetItem(shortcutTree_, {category});
        group->setFlags(Qt::ItemIsEnabled);
        for (const Command* c : list) {
            auto* item = new QTreeWidgetItem(group, {c->label});
            item->setToolTip(0, c->action->toolTip());
            item->setData(0, kIdRole, c->id);
        }
    }
    shortcutTree_->expandAll();
    layout->addWidget(shortcutTree_, 1);

    auto* row = new QHBoxLayout;
    row->addWidget(new QLabel(tr("Shortcut:")));
    keyEdit_ = new QKeySequenceEdit;
    keyEdit_->setMaximumSequenceLength(1);
    keyEdit_->setToolTip(tr("Click, then press the keys"));
    row->addWidget(keyEdit_, 1);
    clearKey_ = new QPushButton(tr("None"));
    clearKey_->setToolTip(tr("No shortcut for this command"));
    row->addWidget(clearKey_);
    resetKey_ = new QPushButton(tr("Default"));
    resetKey_->setToolTip(tr("Back to the built-in shortcut"));
    row->addWidget(resetKey_);
    auto* resetAll = new QPushButton(tr("Reset All"));
    resetAll->setToolTip(tr("Every command back to its built-in shortcut"));
    row->addWidget(resetAll);
    layout->addLayout(row);
    layout->addWidget(note(
        tr("Bold shortcuts differ from the built-in ones. Giving a command a shortcut another one "
           "has moves it (you are asked first). Commands that choose between options (the "
           "toolpath display, the standard views) can share a key, marked ⟳: each press moves to "
           "the next of them. Give \"All Operations\" and \"Hidden\" one key to toggle the "
           "toolpaths.")));

    auto selectionChanged = [this] {
        const Command* c = currentCommand();
        keyEdit_->setEnabled(c != nullptr);
        clearKey_->setEnabled(c != nullptr);
        resetKey_->setEnabled(c != nullptr && bindings_[c->id].has_value());
        const QSignalBlocker block(keyEdit_);
        const QList<QKeySequence> keys = c ? shortcutsOf(*c) : QList<QKeySequence>();
        keyEdit_->setKeySequence(keys.isEmpty() ? QKeySequence() : keys.front());
    };
    connect(shortcutTree_, &QTreeWidget::currentItemChanged, this, selectionChanged);
    connect(keyEdit_, &QKeySequenceEdit::editingFinished, this, [this] {
        if (!keyEdit_->keySequence().isEmpty()) {
            assignShortcut(keyEdit_->keySequence());
        }
    });
    connect(clearKey_, &QPushButton::clicked, this, [this] { assignShortcut(QKeySequence()); });
    connect(resetKey_, &QPushButton::clicked, this, [this, selectionChanged] {
        if (const Command* c = currentCommand()) {
            // The built-in shortcut may have been moved to another command meanwhile.
            for (const QKeySequence& key : c->defaultShortcuts) {
                for (const Command& other : commands_.commands()) {
                    if (other.id != c->id && !sharesKeysWith(other, *c) &&
                        shortcutsOf(other).contains(key)) {
                        QList<QKeySequence> theirs = shortcutsOf(other);
                        theirs.removeAll(key);
                        bindings_[other.id] = theirs.isEmpty() ? QKeySequence() : theirs.front();
                    }
                }
            }
            bindings_[c->id].reset();
            refreshShortcutRows();
            selectionChanged();
        }
    });
    connect(resetAll, &QPushButton::clicked, this, [this, selectionChanged] {
        for (auto& [id, binding] : bindings_) {
            binding.reset();
        }
        refreshShortcutRows();
        selectionChanged();
    });
    connect(filter, &QLineEdit::textChanged, this, &PreferencesDialog::filterShortcuts);
    refreshShortcutRows();
    selectionChanged();
    return page;
}

QList<QKeySequence> PreferencesDialog::shortcutsOf(const Command& command) const {
    const auto it = bindings_.find(command.id);
    return shortcutsWith(command, it == bindings_.end() ? std::nullopt : it->second);
}

const Command* PreferencesDialog::currentCommand() const {
    const QTreeWidgetItem* item = shortcutTree_->currentItem();
    return item ? commands_.find(item->data(0, kIdRole).toString()) : nullptr;
}

void PreferencesDialog::assignShortcut(const QKeySequence& shortcut) {
    const Command* c = currentCommand();
    if (!c) {
        return;
    }
    const QList<QKeySequence> mine = shortcutsOf(*c);
    if (shortcut == QKeySequence(Qt::Key_Escape)) {
        QMessageBox::information(this, tr("Shortcut in use"),
                                 tr("Esc stops picking in the view; it cannot run a command."));
        const QSignalBlocker block(keyEdit_);
        keyEdit_->setKeySequence(mine.isEmpty() ? QKeySequence() : mine.front());
        return;
    }
    if (!shortcut.isEmpty()) {
        // Commands of the same option group share the key (it steps through them); any other
        // command with it gives it up, after asking.
        std::vector<const Command*> holders;
        QStringList names;
        for (const Command& other : commands_.commands()) {
            if (other.id != c->id && !sharesKeysWith(other, *c) &&
                shortcutsOf(other).contains(shortcut)) {
                holders.push_back(&other);
                names << tr("“%1” (%2)").arg(other.label, other.category);
            }
        }
        if (!holders.empty()) {
            const auto answer =
                QMessageBox::question(this, tr("Shortcut in use"),
                                      tr("%1 runs %2. Use it for “%3” instead?")
                                          .arg(shortcut.toString(QKeySequence::NativeText),
                                               names.join(QStringLiteral(", ")), c->label));
            if (answer != QMessageBox::Yes) {
                const QSignalBlocker block(keyEdit_);
                keyEdit_->setKeySequence(mine.isEmpty() ? QKeySequence() : mine.front());
                return;
            }
            for (const Command* other : holders) {
                QList<QKeySequence> theirs = shortcutsOf(*other);
                theirs.removeAll(shortcut);
                bindings_[other->id] = theirs.isEmpty() ? QKeySequence() : theirs.front();
            }
        }
    }
    // The built-in shortcut is no custom binding: the command keeps following the defaults.
    const QList<QKeySequence> wanted =
        shortcut.isEmpty() ? QList<QKeySequence>() : QList<QKeySequence>{shortcut};
    if (wanted == c->defaultShortcuts) {
        bindings_[c->id].reset();
    } else {
        bindings_[c->id] = shortcut;
    }
    refreshShortcutRows();
    resetKey_->setEnabled(bindings_[c->id].has_value());
    const QSignalBlocker block(keyEdit_);
    keyEdit_->setKeySequence(shortcut);
}

void PreferencesDialog::refreshShortcutRows() {
    for (int g = 0; g < shortcutTree_->topLevelItemCount(); ++g) {
        QTreeWidgetItem* group = shortcutTree_->topLevelItem(g);
        for (int i = 0; i < group->childCount(); ++i) {
            QTreeWidgetItem* item = group->child(i);
            const Command* c = commands_.find(item->data(0, kIdRole).toString());
            if (!c) {
                continue;
            }
            // Keys shared within an option group step through its commands: marked ⟳.
            const QList<QKeySequence> keys = shortcutsOf(*c);
            QStringList sharers;
            for (const Command& other : commands_.commands()) {
                if (other.id != c->id && sharesKeysWith(other, *c) &&
                    std::any_of(keys.begin(), keys.end(), [&](const QKeySequence& k) {
                        return shortcutsOf(other).contains(k);
                    })) {
                    sharers << other.label;
                }
            }
            QString text = QKeySequence::listToString(keys, QKeySequence::NativeText);
            if (!sharers.isEmpty()) {
                text += QStringLiteral("  ⟳");
                item->setToolTip(1, formatToolTip(tr("Each press moves to the next of: %1")
                                                      .arg(sharers.join(QStringLiteral(", "))),
                                                  tr("Shared key")));
            } else {
                item->setToolTip(1, QString());
            }
            item->setText(1, text);
            QFont font = item->font(1);
            font.setBold(bindings_[c->id].has_value());
            item->setFont(1, font);
        }
    }
}

void PreferencesDialog::filterShortcuts(const QString& text) {
    for (int g = 0; g < shortcutTree_->topLevelItemCount(); ++g) {
        QTreeWidgetItem* group = shortcutTree_->topLevelItem(g);
        const bool groupMatches = group->text(0).contains(text, Qt::CaseInsensitive);
        int shown = 0;
        for (int i = 0; i < group->childCount(); ++i) {
            QTreeWidgetItem* item = group->child(i);
            const bool match = text.isEmpty() || groupMatches ||
                               item->text(0).contains(text, Qt::CaseInsensitive) ||
                               item->text(1).contains(text, Qt::CaseInsensitive);
            item->setHidden(!match);
            shown += match ? 1 : 0;
        }
        group->setHidden(shown == 0);
    }
}

// -- Toolbar ---------------------------------------------------------------------------------

QWidget* PreferencesDialog::buildToolbar() {
    auto* page = new QWidget;
    auto* layout = new QVBoxLayout(page);
    layout->addWidget(note(tr("Pinned commands get a button on the toolbar, to the right of the "
                              "Stock section, in this order.")));
    auto* columns = new QHBoxLayout;

    auto* left = new QVBoxLayout;
    left->addWidget(new QLabel(tr("Commands")));
    available_ = new QTreeWidget;
    available_->setHeaderHidden(true);
    for (const auto& [category, list] : byCategory(commands_)) {
        auto* group = new QTreeWidgetItem(available_, {category});
        group->setFlags(Qt::ItemIsEnabled);
        for (const Command* c : list) {
            auto* item = new QTreeWidgetItem(group, {c->label});
            item->setToolTip(0, c->action->toolTip());
            item->setData(0, kIdRole, c->id);
        }
    }
    available_->expandAll();
    left->addWidget(available_);
    columns->addLayout(left, 1);

    auto* middle = new QVBoxLayout;
    middle->addStretch();
    auto* pinButton = new QPushButton(tr("Pin →"));
    auto* unpinButton = new QPushButton(tr("← Unpin"));
    middle->addWidget(pinButton);
    middle->addWidget(unpinButton);
    middle->addStretch();
    columns->addLayout(middle);

    auto* right = new QVBoxLayout;
    right->addWidget(new QLabel(tr("Pinned to the toolbar")));
    pinned_ = new QListWidget;
    for (const QString& id : pinnedCommands()) {
        pin(id);
    }
    right->addWidget(pinned_);
    auto* order = new QHBoxLayout;
    auto* up = new QPushButton(tr("Move Up"));
    auto* down = new QPushButton(tr("Move Down"));
    order->addWidget(up);
    order->addWidget(down);
    order->addStretch();
    right->addLayout(order);
    columns->addLayout(right, 1);
    layout->addLayout(columns, 1);

    auto pinCurrent = [this] {
        if (const QTreeWidgetItem* item = available_->currentItem()) {
            pin(item->data(0, kIdRole).toString());
        }
    };
    auto unpinCurrent = [this] {
        delete pinned_->takeItem(pinned_->currentRow());
    };
    connect(pinButton, &QPushButton::clicked, this, pinCurrent);
    connect(available_, &QTreeWidget::itemDoubleClicked, this, pinCurrent);
    connect(unpinButton, &QPushButton::clicked, this, unpinCurrent);
    connect(pinned_, &QListWidget::itemDoubleClicked, this, unpinCurrent);
    connect(up, &QPushButton::clicked, this, [this] { movePinned(-1); });
    connect(down, &QPushButton::clicked, this, [this] { movePinned(1); });
    return page;
}

void PreferencesDialog::pin(const QString& id) {
    const Command* c = commands_.find(id);
    if (!c) {
        return; // a category, or a command this version no longer has
    }
    for (int i = 0; i < pinned_->count(); ++i) {
        if (pinned_->item(i)->data(kIdRole).toString() == id) {
            pinned_->setCurrentRow(i);
            return;
        }
    }
    auto* item = new QListWidgetItem(QStringLiteral("%1  ·  %2").arg(c->label, c->category));
    item->setData(kIdRole, id);
    pinned_->addItem(item);
    pinned_->setCurrentItem(item);
}

void PreferencesDialog::movePinned(int delta) {
    const int row = pinned_->currentRow();
    const int to = row + delta;
    if (row < 0 || to < 0 || to >= pinned_->count()) {
        return;
    }
    QListWidgetItem* item = pinned_->takeItem(row);
    pinned_->insertItem(to, item);
    pinned_->setCurrentRow(to);
}

// -- Operation defaults ----------------------------------------------------------------------

QWidget* PreferencesDialog::buildDefaults() {
    auto* page = new QWidget;
    auto* layout = new QHBoxLayout(page);
    types_ = new QListWidget;
    types_->setMaximumWidth(240);
    for (const strategies::StrategyInfo& info : strategies::strategies()) {
        if (info.params.empty()) {
            continue;
        }
        auto* item = new QListWidgetItem(trStrategy(info.label));
        item->setData(kIdRole, qs(info.id));
        item->setToolTip(formatToolTip(trStrategy(info.description), trStrategy(info.label)));
        types_->addItem(item);
    }
    layout->addWidget(types_);
    auto* right = new QVBoxLayout;
    defaultsPage_ = new QScrollArea;
    defaultsPage_->setWidgetResizable(true);
    defaultsPage_->setFrameShape(QFrame::NoFrame);
    right->addWidget(defaultsPage_, 1);
    auto* restore = new QPushButton(tr("Restore Built-in Values"));
    restore->setToolTip(tr("Every value of this operation type back to the built-in default"));
    auto* buttons = new QHBoxLayout;
    buttons->addStretch();
    buttons->addWidget(restore);
    right->addLayout(buttons);
    layout->addLayout(right, 1);

    connect(types_, &QListWidget::currentRowChanged, this, [this](int row) {
        if (row >= 0) {
            shownType_ = types_->item(row)->data(kIdRole).toString().toStdString();
            rebuildDefaultsPage();
        }
    });
    connect(restore, &QPushButton::clicked, this, [this] {
        if (!shownType_.empty()) {
            defaultsFor(shownType_) = Json::object();
            rebuildDefaultsPage();
            markTypes();
        }
    });
    markTypes();
    types_->setCurrentRow(0);
    return page;
}

void PreferencesDialog::showOperationType(const std::string& type) {
    for (int i = 0; i < types_->count(); ++i) {
        if (types_->item(i)->data(kIdRole).toString().toStdString() == type) {
            types_->setCurrentRow(i);
        }
    }
}

Json& PreferencesDialog::defaultsFor(const std::string& type) {
    auto it = defaults_.find(type);
    if (it == defaults_.end()) {
        it = defaults_.emplace(type, operationDefaults(type)).first;
    }
    return it->second;
}

void PreferencesDialog::markTypes() {
    for (int i = 0; i < types_->count(); ++i) {
        QListWidgetItem* item = types_->item(i);
        const std::string type = item->data(kIdRole).toString().toStdString();
        QFont font = item->font();
        font.setBold(!defaultsFor(type).empty());
        item->setFont(font);
    }
}

void PreferencesDialog::rebuildDefaultsPage() {
    const strategies::StrategyInfo* info =
        shownType_.empty() ? nullptr : strategies::findStrategy(shownType_);
    if (!info || !defaultsPage_) {
        return;
    }
    const std::string type = shownType_;
    auto* page = new QWidget;
    auto* form = new QFormLayout(page);
    form->addRow(note(tr("New “%1” operations start with these values. Bold values are your own; "
                         "↺ goes back to the built-in value, or to the tool's cutting preset "
                         "where it sets one (step down, step over, ramp angle).")
                          .arg(trStrategy(info->label))));

    struct Row {
        const strategies::ParamDef* def;
        QWidget* label;
        QWidget* field;
    };
    auto rows = std::make_shared<std::vector<Row>>();
    auto updateRows = [this, type, info, rows] {
        const Json shown = defaultsFor(type);
        for (const Row& r : *rows) {
            const bool applies = info->applies(*r.def, shown);
            r.label->setEnabled(applies);
            r.field->setEnabled(applies);
        }
    };
    const Json shown = defaultsFor(type);
    for (const strategies::ParamDef& def : info->params) {
        const std::string key = def.key;
        const bool set = defaultsFor(type).contains(key);
        auto* label = new QLabel(trStrategy(def.label));
        label->setToolTip(formatToolTip(trStrategy(def.help), trStrategy(def.label)));
        setBold(label, set);
        auto* field = new QWidget;
        auto* row = new QHBoxLayout(field);
        row->setContentsMargins(0, 0, 0, 0);
        auto* reset = new QToolButton;
        reset->setText(QStringLiteral("↺"));
        reset->setToolTip(tr("Back to the built-in value"));
        reset->setEnabled(set);
        reset->setAutoRaise(true);
        const Json* current = shown.contains(key) ? &shown.at(key) : nullptr;
        QWidget* editor =
            createParamEditor(def, current, displayUnit(),
                              [this, type, key, label, reset, updateRows](const Json& value) {
                                  defaultsFor(type)[key] = value;
                                  setBold(label, true);
                                  reset->setEnabled(true);
                                  updateRows();
                                  markTypes();
                              });
        editor->setToolTip(QString()); // the label explains it
        row->addWidget(editor, 1);
        row->addWidget(reset);
        connect(reset, &QToolButton::clicked, this, [this, type, key] {
            defaultsFor(type).erase(key);
            markTypes();
            // Not from inside the button's own click: the page it is on is replaced.
            QTimer::singleShot(0, this, [this] { rebuildDefaultsPage(); });
        });
        form->addRow(label, field);
        rows->push_back({&def, label, field});
    }
    updateRows();
    const int scroll = defaultsPage_->widget() ? defaultsPage_->verticalScrollBar()->value() : 0;
    defaultsPage_->setWidget(page); // deletes the previous page
    defaultsPage_->verticalScrollBar()->setValue(scroll);
}

// -- Colors ---------------------------------------------------------------------------------

QWidget* PreferencesDialog::buildColors() {
    colors_ = ViewColors::load();
    auto* page = new QWidget;
    auto* layout = new QVBoxLayout(page);
    layout->addWidget(note(tr("The colours of the 3D view. Click a colour to change it; ↺ goes "
                              "back to the built-in one.")));
    auto* scroll = new QScrollArea;
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    auto* inner = new QWidget;
    auto* form = new QFormLayout(inner);
    const ViewColors defaults;
    // A button showing a colour: a swatch and its hex code.
    auto show = [](QPushButton* button, const QColor& colour) {
        QPixmap swatch(28, 14);
        swatch.fill(colour);
        button->setIcon(QIcon(swatch));
        button->setIconSize(swatch.size());
        button->setText(colour.name().toUpper());
    };
    std::vector<std::function<void()>> refreshers;
    QString group;
    for (const ViewColors::Entry& e : ViewColors::entries()) {
        if (group != QLatin1String(e.group)) {
            group = QLatin1String(e.group);
            auto* heading =
                new QLabel(QStringLiteral("<b>%1</b>")
                               .arg(QCoreApplication::translate("ViewColors", e.group)));
            form->addRow(heading);
        }
        const QString label = QCoreApplication::translate("ViewColors", e.label);
        auto* row = new QWidget;
        auto* line = new QHBoxLayout(row);
        line->setContentsMargins(0, 0, 0, 0);
        auto* button = new QPushButton;
        auto* reset = new QToolButton;
        reset->setText(QStringLiteral("↺"));
        reset->setAutoRaise(true);
        reset->setToolTip(tr("Back to the built-in colour"));
        line->addWidget(button);
        line->addWidget(reset);
        line->addStretch();
        const auto field = e.field;
        auto refresh = [this, button, reset, field, defaults, show] {
            show(button, colors_.*field);
            reset->setEnabled(colors_.*field != defaults.*field);
        };
        refresh();
        refreshers.push_back(refresh);
        connect(button, &QPushButton::clicked, this, [this, field, label, refresh] {
            const QColor picked = QColorDialog::getColor(colors_.*field, this, label);
            if (picked.isValid()) {
                colors_.*field = picked;
                refresh();
            }
        });
        connect(reset, &QToolButton::clicked, this, [this, field, defaults, refresh] {
            colors_.*field = defaults.*field;
            refresh();
        });
        form->addRow(label, row);
    }
    scroll->setWidget(inner);
    layout->addWidget(scroll, 1);
    auto* restore = new QPushButton(tr("Restore Built-in Colors"));
    connect(restore, &QPushButton::clicked, this, [this, refreshers] {
        colors_ = ViewColors();
        for (const auto& refresh : refreshers) {
            refresh();
        }
    });
    auto* buttons = new QHBoxLayout;
    buttons->addStretch();
    buttons->addWidget(restore);
    layout->addLayout(buttons);
    return page;
}

// -- Saving ----------------------------------------------------------------------------------

void PreferencesDialog::accept() {
    setPreferredUnit(displayUnit());
    QSettings().setValue(QStringLiteral("showStartHub"), hub_->isChecked());
    setChosenLanguage(language_->currentData().toString());
    if (library_->currentIndex() > 0) {
        ToolLibraries::setDefaultFile(toPath(library_->currentData().toString()));
    }
    for (const auto& [id, binding] : bindings_) {
        setCustomShortcut(id, binding);
    }
    QStringList pins;
    for (int i = 0; i < pinned_->count(); ++i) {
        pins << pinned_->item(i)->data(kIdRole).toString();
    }
    setPinnedCommands(pins);
    for (const auto& [type, values] : defaults_) {
        setOperationDefaults(type, values);
    }
    colors_.save();
    QDialog::accept();
}

} // namespace ocam::gui
