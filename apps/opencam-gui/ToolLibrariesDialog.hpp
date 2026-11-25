// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Manages the user's tool libraries (see ToolLibraries): create, add existing files, edit,
// rename, take off the list, and choose the default.

#pragma once

#include <QDialog>
#include <filesystem>
#include <functional>

class QListWidget;
class QPushButton;

namespace ocam::gui {

class ToolLibrariesDialog : public QDialog {
    Q_OBJECT

public:
    /// `edit` opens a library file in the tool library editor (modal).
    using EditFn = std::function<void(const std::filesystem::path&)>;
    explicit ToolLibrariesDialog(EditFn edit, QWidget* parent = nullptr);

private:
    void reload();
    std::filesystem::path selected() const;

    EditFn edit_;
    QListWidget* list_ = nullptr;
    QPushButton* editButton_ = nullptr;
    QPushButton* renameButton_ = nullptr;
    QPushButton* removeButton_ = nullptr;
    QPushButton* defaultButton_ = nullptr;
};

} // namespace ocam::gui
