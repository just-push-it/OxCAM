// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "PostProcessDialog.hpp"

#include "ComboLists.hpp"

#include "PostOptionsForm.hpp"
#include "ShowBuiltins.hpp"

#include <ocam/document/PostSetup.hpp>
#include <ocam/foundation/Error.hpp>

#include <QComboBox>
#include <QDialogButtonBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QSignalBlocker>
#include <QVBoxLayout>
#include <algorithm>

namespace ocam::gui {

namespace {

QString qs(const std::string& s) {
    return QString::fromStdString(s);
}

constexpr int kAddFile = -1; // the combo row that adds a post file

} // namespace

PostProcessDialog::PostProcessDialog(const document::Project& project, const Uuid& setupId,
                                     Mode mode, QWidget* parent)
    : QDialog(parent), project_(project), setupId_(setupId) {
    const document::Setup* setup = project.findSetup(setupId);
    if (!setup) {
        throw Error(ErrorCode::InvalidArgument, "unknown setup");
    }
    machine_ = document::machineFor(project, *setup);
    try {
        setupPostId_ = document::postFor(project, *setup, machine_).manifest().id;
    } catch (const Error&) {
        // the setup's post is missing: the dialog shows why when it is chosen
    }
    setWindowTitle(mode == Mode::Post ? tr("Post process %1").arg(qs(setup->name))
                                      : tr("Post settings of %1").arg(qs(setup->name)));
    setMinimumWidth(600);

    auto* layout = new QVBoxLayout(this);
    auto* heading =
        new QLabel(tr("<b>Setup:</b> %1 &nbsp;&nbsp; <b>Machine:</b> %2")
                       .arg(qs(setup->name).toHtmlEscaped(), qs(machine_.name).toHtmlEscaped()));
    layout->addWidget(heading);

    auto* form = new QFormLayout;
    box_ = new QComboBox;
    box_->setObjectName(QStringLiteral("postChoice"));
    box_->setToolTip(tr("The post processor that writes the program for the machine's control"));
    form->addRow(tr("Post"), box_);
    showBuiltins_ = newShowBuiltinsBox();
    form->addRow(QString(), showBuiltins_);
    auto* hint = new QLabel(tr("A machine's default post and its option values are set in the "
                               "machine library. Choosing another post here keeps it for this "
                               "setup."));
    hint->setWordWrap(true);
    hint->setStyleSheet(QStringLiteral("color: gray;"));
    form->addRow(QString(), hint);
    layout->addLayout(form);

    problems_ = new QLabel;
    problems_->setWordWrap(true);
    problems_->setStyleSheet(QStringLiteral("color: #c0392b;"));
    problems_->hide();
    layout->addWidget(problems_);

    optionsBox_ = new QGroupBox(tr("Program options"));
    auto* optionsLayout = new QVBoxLayout(optionsBox_);
    options_ = new PostOptionsForm;
    optionsLayout->addWidget(options_);
    layout->addWidget(optionsBox_);
    operationNote_ = new QLabel(tr("This post also has options per operation: they are on each "
                                   "operation's page."));
    operationNote_->setWordWrap(true);
    operationNote_->setStyleSheet(QStringLiteral("color: gray;"));
    layout->addWidget(operationNote_);
    layout->addStretch();

    auto* buttons = new QHBoxLayout;
    saveDefaults_ = new QPushButton(tr("Save as my defaults"));
    saveDefaults_->setToolTip(tr("Start this post's options from these values in setups and "
                                 "machines with no values of their own (for you, on this "
                                 "computer)"));
    useDefaults_ = new QPushButton(tr("Use defaults"));
    useDefaults_->setToolTip(tr("Drop this setup's values: each option takes the machine's value, "
                                "else your default, else the post's"));
    buttons->addWidget(saveDefaults_);
    buttons->addWidget(useDefaults_);
    buttons->addStretch();
    auto* box = new QDialogButtonBox(QDialogButtonBox::Cancel);
    okButton_ =
        box->addButton(mode == Mode::Post ? tr("Post") : tr("Save"), QDialogButtonBox::AcceptRole);
    okButton_->setDefault(true);
    buttons->addWidget(box);
    layout->addLayout(buttons);
    connect(box, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(box, &QDialogButtonBox::rejected, this, &QDialog::reject);
    connect(saveDefaults_, &QPushButton::clicked, this, &PostProcessDialog::saveDefaults);
    connect(useDefaults_, &QPushButton::clicked, this, [this] { showOptions(false); });

    // The choices: the machine's default, the posts for its kinematics, a post file.
    QStringList problems;
    entries_ = postEntries(project, *setup, machine_, &problems);
    machinePostId_ = entries_.front().id;
    entry_ = currentPostEntry(entries_, *setup);
    fillBox();
    connect(showBuiltins_, &QCheckBox::toggled, this, [this] { fillBox(); });
    connect(box_, &QComboBox::currentIndexChanged, this, [this](int row) {
        if (box_->itemData(row).toInt() == kAddFile) {
            addPostFile();
            return;
        }
        lastIndex_ = row;
        choose(box_->itemData(row).toInt());
    });
    choose(currentPostEntry(entries_, *setup));
    if (!problems.isEmpty()) {
        // Posts that do not load (the library's, the project's): shown, not fatal.
        problems_->setText(problems_->text().isEmpty() ? problems.join(QLatin1Char('\n'))
                                                       : problems_->text() + QLatin1Char('\n') +
                                                             problems.join(QLatin1Char('\n')));
        problems_->show();
    }
}

void PostProcessDialog::fillBox() {
    const QSignalBlocker block(box_);
    box_->clear();
    const bool show = showBuiltins_->isChecked();
    std::optional<PostEntry::Kind> previous;
    for (std::size_t i = 0; i < entries_.size(); ++i) {
        const int index = static_cast<int>(i);
        if (entries_[i].kind == PostEntry::Kind::BuiltIn && !show && index != entry_) {
            continue;
        }
        if (!previous || *previous != entries_[i].kind) {
            if (const QString heading = postKindHeading(entries_[i].kind); !heading.isEmpty()) {
                addListHeading(box_, heading);
            } else if (previous) {
                box_->insertSeparator(box_->count());
            }
        }
        previous = entries_[i].kind;
        box_->addItem(entries_[i].label, index);
    }
    box_->insertSeparator(box_->count());
    box_->addItem(tr("Add a post file..."), kAddFile);
    lastIndex_ = box_->findData(entry_);
    box_->setCurrentIndex(lastIndex_);
    showEntriesInFull(box_);
}

void PostProcessDialog::choose(int index) {
    entry_ = index;
    post_.reset();
    QStringList problems;
    try {
        post_ = entryPost(entries_.at(static_cast<std::size_t>(index)), project_, machine_);
        const auto& machines = post_->manifest().machines;
        if (std::ranges::find(machines, std::string(machine::toString(machine_.kinematics))) ==
            machines.end()) {
            problems << tr("%1 does not write for this machine's kinematics (%2).")
                            .arg(qs(post_->manifest().name),
                                 qs(std::string(machine::toString(machine_.kinematics))));
        }
    } catch (const Error& e) {
        problems << qs(e.what());
    }
    problems_->setText(problems.join(QLatin1Char('\n')));
    problems_->setVisible(!problems.isEmpty());
    okButton_->setEnabled(post_ && problems.isEmpty());
    showOptions(true);
}

void PostProcessDialog::addPostFile() {
    const QString path = QFileDialog::getOpenFileName(this, tr("Add a post file"), QString(),
                                                      tr("Post processors (*.js)"));
    if (path.isEmpty()) {
        const QSignalBlocker block(box_);
        box_->setCurrentIndex(lastIndex_);
        return;
    }
    try {
        const std::filesystem::path file(path.toStdU16String());
        const std::u8string name = file.filename().u8string();
        std::string source = readFileBytes(file);
        CancelToken cancel;
        post::PostScript loaded =
            post::PostScript::load(source, std::string(name.begin(), name.end()), cancel);
        PostEntry entry;
        entry.kind = PostEntry::Kind::InLibrary;
        entry.id = loaded.manifest().id;
        entry.label = tr("File: %1").arg(qs(loaded.manifest().name));
        entry.library = PostLibrary::Entry{std::move(loaded), std::move(source), file};
        entries_.push_back(std::move(entry));
        const int index = static_cast<int>(entries_.size()) - 1;
        const QSignalBlocker block(box_);
        const int row = box_->count() - 2; // before the separator and "Add a post file..."
        box_->insertItem(row, entries_.back().label, index);
        box_->setCurrentIndex(row);
        lastIndex_ = row;
        choose(index);
    } catch (const Error& e) {
        const QSignalBlocker block(box_);
        box_->setCurrentIndex(lastIndex_);
        QMessageBox::warning(this, tr("Add a post file"),
                             tr("The post was not added:\n\n%1").arg(qs(e.what())));
    }
}

void PostProcessDialog::showOptions(bool withSetup) {
    std::vector<PostOptionsForm::Layer> layers;
    if (post_) {
        const std::string& id = post_->manifest().id;
        const document::Setup* setup = project_.findSetup(setupId_);
        if (withSetup && setup && id == setupPostId_ && setup->postOptions.is_object() &&
            setup->postOptions.contains("program")) {
            layers.push_back({tr("this setup"), tr("The value the setup was last posted with"),
                              setup->postOptions.at("program")});
        }
        if (id == machinePostId_) {
            layers.push_back({tr("machine"),
                              tr("The machine's value for its default post (machine library)"),
                              machine_.postOptions});
        }
        layers.push_back({tr("your default"), tr("Your saved default for this post"),
                          PostLibrary::userDefaults(id)});
    }
    options_->show(post_ ? &*post_ : nullptr, layers);
    if (post_) {
        optionsBox_->setTitle(tr("Program options of %1").arg(qs(post_->manifest().name)));
    }
    optionsBox_->setVisible(static_cast<bool>(post_));
    operationNote_->setVisible(options_->hasOperationOptions());
    saveDefaults_->setEnabled(static_cast<bool>(post_));
    useDefaults_->setEnabled(static_cast<bool>(post_));
}

void PostProcessDialog::saveDefaults() {
    if (!post_) {
        return;
    }
    PostLibrary::setUserDefaults(post_->manifest().id, options_->values());
    options_->markAll({tr("your default"), tr("Your saved default for this post"), {}});
}

bool PostProcessDialog::apply(document::Project& project) const {
    document::Setup* setup = project.findSetup(setupId_);
    if (!setup || !post_) {
        return false;
    }
    bool changed = false;
    const PostEntry& entry = entries_.at(static_cast<std::size_t>(entry_));
    if (entry.library) {
        const document::ProjectPost copy = projectCopy(*entry.library);
        if (std::ranges::find(project.posts, copy) == project.posts.end()) {
            addProjectPost(project, copy);
            changed = true;
        }
        setup = project.findSetup(setupId_);
    }
    const std::optional<document::PostChoice> choice = postChoiceOf(entry);
    if (setup->post != choice) {
        setup->post = choice;
        changed = true;
    }
    // Option values belong to one post: another post's are dropped.
    Json options = post_->manifest().id == setupPostId_ && setup->postOptions.is_object()
                       ? setup->postOptions
                       : Json::object();
    options["program"] = options_->values();
    if (options != setup->postOptions) {
        setup->postOptions = std::move(options);
        changed = true;
    }
    return changed;
}

} // namespace ocam::gui
