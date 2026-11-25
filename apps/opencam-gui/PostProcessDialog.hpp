// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Setup > Post Process: choose the post and set its program options before posting (or, from
// the setup page's Configure button, without posting). Each option starts from the setup's
// value, else the machine's (its default post's, machine library), else the user's own default
// (PostLibrary), else the post's. Saving stores the choice and every value in the setup, so the
// project posts the same anywhere.

#pragma once

#include "PostChoices.hpp"

#include <ocam/document/Project.hpp>
#include <ocam/foundation/Json.hpp>
#include <ocam/machine/Machine.hpp>
#include <ocam/post/Engine.hpp>

#include <QDialog>
#include <optional>
#include <string>
#include <vector>

class QCheckBox;
class QComboBox;
class QGroupBox;
class QLabel;
class QPushButton;

namespace ocam::gui {

class PostOptionsForm;

class PostProcessDialog : public QDialog {
    Q_OBJECT

public:
    enum class Mode {
        Post,      ///< "Post": the caller posts after apply()
        Configure, ///< "Save": settings only
    };

    /// `project` must outlive the dialog's exec().
    PostProcessDialog(const document::Project& project, const Uuid& setupId, Mode mode,
                      QWidget* parent = nullptr);

    /// Stores the chosen post (copying a library post or file into the project) and the option
    /// values in the setup. Values of another post the setup used before are dropped. Returns
    /// whether the project changed.
    bool apply(document::Project& project) const;

private:
    void choose(int index);
    /// Lists the entries: the built-in posts only with Show built-ins ticked, or when chosen.
    void fillBox();
    void addPostFile();
    void showOptions(bool withSetup);
    void saveDefaults();

    const document::Project& project_;
    Uuid setupId_;
    machine::Machine machine_;
    std::string setupPostId_;   ///< The post the setup uses now (empty when it cannot be resolved).
    std::string machinePostId_; ///< The machine's default post (empty likewise).
    std::vector<PostEntry> entries_;
    int entry_ = 0;
    int lastIndex_ = 0; ///< The combo row shown before "Add a post file..." was picked.
    std::optional<post::PostScript> post_;

    QComboBox* box_ = nullptr;
    QCheckBox* showBuiltins_ = nullptr;
    QLabel* problems_ = nullptr;
    QGroupBox* optionsBox_ = nullptr;
    PostOptionsForm* options_ = nullptr;
    QLabel* operationNote_ = nullptr;
    QPushButton* saveDefaults_ = nullptr;
    QPushButton* useDefaults_ = nullptr;
    QPushButton* okButton_ = nullptr;
};

} // namespace ocam::gui
