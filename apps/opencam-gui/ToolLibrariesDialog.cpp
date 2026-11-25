// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ToolLibrariesDialog.hpp"

#include "ToolLibraries.hpp"

#include <ocam/foundation/Error.hpp>

#include <QColor>
#include <QDialogButtonBox>
#include <QFileDialog>
#include <QFont>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>

namespace ocam::gui {
namespace {

QString fromPath(const std::filesystem::path& p) {
    return QString::fromStdU16String(p.u16string());
}

} // namespace

ToolLibrariesDialog::ToolLibrariesDialog(EditFn edit, QWidget* parent)
    : QDialog(parent), edit_(std::move(edit)) {
    setWindowTitle(tr("Tool Libraries"));
    resize(560, 380);
    auto* layout = new QVBoxLayout(this);
    auto* intro = new QLabel(
        tr("Keep separate libraries for machines or processes. Tools picked for an operation "
           "are copied into the project; the default library is listed first."));
    intro->setWordWrap(true);
    layout->addWidget(intro);

    auto* row = new QHBoxLayout;
    list_ = new QListWidget;
    row->addWidget(list_, 1);
    auto* buttons = new QVBoxLayout;
    auto* create = new QPushButton(tr("New..."));
    auto* addFile = new QPushButton(tr("Add File..."));
    editButton_ = new QPushButton(tr("Edit..."));
    renameButton_ = new QPushButton(tr("Rename..."));
    renameButton_->setToolTip(tr("Changes the library's name (the file keeps its name)"));
    defaultButton_ = new QPushButton(tr("Make Default"));
    removeButton_ = new QPushButton(tr("Remove From List"));
    removeButton_->setToolTip(tr("Takes the library off the list; the file is kept"));
    for (QPushButton* b :
         {create, addFile, editButton_, renameButton_, defaultButton_, removeButton_}) {
        buttons->addWidget(b);
    }
    buttons->addStretch();
    row->addLayout(buttons);
    layout->addLayout(row);
    auto* close = new QDialogButtonBox(QDialogButtonBox::Close);
    layout->addWidget(close);
    connect(close, &QDialogButtonBox::rejected, this, &QDialog::reject);

    connect(create, &QPushButton::clicked, this, [this] {
        bool ok = false;
        const QString name = QInputDialog::getText(this, tr("New Tool Library"), tr("Name:"),
                                                   QLineEdit::Normal, QString(), &ok)
                                 .trimmed();
        if (!ok || name.isEmpty()) {
            return;
        }
        try {
            ToolLibraries::create(name.toStdString());
        } catch (const Error& e) {
            QMessageBox::warning(this, tr("New Tool Library"), QString::fromUtf8(e.what()));
        }
        reload();
    });
    connect(addFile, &QPushButton::clicked, this, [this] {
        const QString file = QFileDialog::getOpenFileName(this, tr("Add Tool Library"), QString(),
                                                          tr("OxCAM tool libraries (*.ocamtools)"));
        if (!file.isEmpty()) {
            ToolLibraries::add(std::filesystem::path(file.toStdU16String()));
            reload();
        }
    });
    connect(editButton_, &QPushButton::clicked, this, [this] {
        const auto file = selected();
        if (!file.empty() && edit_) {
            edit_(file);
            reload();
        }
    });
    connect(list_, &QListWidget::itemDoubleClicked, editButton_, &QPushButton::click);
    connect(renameButton_, &QPushButton::clicked, this, [this] {
        const auto file = selected();
        if (file.empty()) {
            return;
        }
        try {
            tooling::ToolLibrary lib = tooling::loadToolLibrary(file);
            bool ok = false;
            const QString name =
                QInputDialog::getText(this, tr("Rename Tool Library"), tr("Name:"),
                                      QLineEdit::Normal, QString::fromStdString(lib.name), &ok)
                    .trimmed();
            if (!ok || name.isEmpty() || name.toStdString() == lib.name) {
                return;
            }
            lib.name = name.toStdString();
            tooling::saveToolLibrary(lib, file);
        } catch (const Error& e) {
            QMessageBox::warning(this, tr("Rename Tool Library"), QString::fromUtf8(e.what()));
        }
        reload();
    });
    connect(defaultButton_, &QPushButton::clicked, this, [this] {
        const auto file = selected();
        if (!file.empty()) {
            ToolLibraries::setDefaultFile(file);
            reload();
        }
    });
    connect(removeButton_, &QPushButton::clicked, this, [this] {
        const auto file = selected();
        if (!file.empty()) {
            ToolLibraries::remove(file);
            reload();
        }
    });
    connect(list_, &QListWidget::currentRowChanged, this, [this](int listRow) {
        for (QPushButton* b : {editButton_, renameButton_, defaultButton_, removeButton_}) {
            b->setEnabled(listRow >= 0);
        }
    });
    reload();
}

void ToolLibrariesDialog::reload() {
    const QString current =
        list_->currentItem() ? list_->currentItem()->data(Qt::UserRole).toString() : QString();
    list_->clear();
    std::vector<std::string> problems;
    for (const auto& entry : ToolLibraries::load(&problems)) {
        auto* item = new QListWidgetItem(
            tr("%1 — %n tool(s)%2", nullptr, static_cast<int>(entry.library.tools.size()))
                .arg(QString::fromStdString(entry.library.name.empty() ? std::string("(unnamed)")
                                                                       : entry.library.name),
                     entry.isDefault ? tr("  (default)") : QString()),
            list_);
        item->setData(Qt::UserRole, fromPath(entry.file));
        item->setToolTip(fromPath(entry.file));
        if (entry.isDefault) {
            QFont bold = item->font();
            bold.setBold(true);
            item->setFont(bold);
        }
        if (item->data(Qt::UserRole).toString() == current) {
            list_->setCurrentItem(item);
        }
    }
    for (const auto& problem : problems) {
        auto* item = new QListWidgetItem(QString::fromStdString(problem), list_);
        item->setForeground(QColor(200, 60, 40));
        item->setFlags(Qt::NoItemFlags);
    }
    if (!list_->currentItem() && list_->count() > 0 &&
        problems.size() < static_cast<std::size_t>(list_->count())) {
        list_->setCurrentRow(0);
    }
    for (QPushButton* b : {editButton_, renameButton_, defaultButton_, removeButton_}) {
        b->setEnabled(list_->currentItem() != nullptr);
    }
}

std::filesystem::path ToolLibrariesDialog::selected() const {
    const QListWidgetItem* item = list_->currentItem();
    if (!item || item->data(Qt::UserRole).toString().isEmpty()) {
        return {};
    }
    return std::filesystem::path(item->data(Qt::UserRole).toString().toStdU16String());
}

} // namespace ocam::gui
