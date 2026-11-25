// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "PostChoices.hpp"

#include <ocam/document/PostSetup.hpp>
#include <ocam/foundation/Error.hpp>

#include <fmt/format.h>

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QObject>
#include <QSpinBox>
#include <algorithm>

namespace ocam::gui {

namespace {

QString qs(const std::string& s) {
    return QString::fromStdString(s);
}

std::string ss(const QString& s) {
    return s.toStdString();
}

bool writesFor(const post::PostScript& post, const machine::Machine& machine) {
    const auto& machines = post.manifest().machines;
    return std::ranges::find(machines, std::string(machine::toString(machine.kinematics))) !=
           machines.end();
}

} // namespace

std::vector<PostEntry> postEntries(const document::Project& project, const document::Setup& setup,
                                   const machine::Machine& machine, QStringList* problems) {
    auto chosen = [&](const std::string& id, bool inProject) {
        return setup.post && setup.post->id == id && setup.post->inProject == inProject;
    };
    std::vector<post::PostScript> projectPosts;
    try {
        projectPosts = document::projectPosts(project);
    } catch (const Error& e) {
        if (problems) {
            *problems << qs(e.what());
        }
    }
    std::vector<std::string> libraryProblems;
    std::vector<PostLibrary::Entry> library = PostLibrary::load(&libraryProblems);
    if (problems) {
        for (const auto& p : libraryProblems) {
            *problems << qs(p);
        }
    }
    auto inProject = [&](const std::string& id) {
        return std::ranges::any_of(projectPosts,
                                   [&](const auto& p) { return p.manifest().id == id; });
    };

    std::vector<PostEntry> out;
    // The machine's default: a project post, a built-in, or a library post the project does not
    // carry yet (copied when used).
    PostEntry machineEntry;
    const std::vector<std::string> builtins = post::builtinPostIds();
    if (!machine.post.empty() && !inProject(machine.post) &&
        std::ranges::find(builtins, machine.post) == builtins.end()) {
        const auto it = std::ranges::find_if(
            library, [&](const auto& e) { return e.post.manifest().id == machine.post; });
        if (it != library.end()) {
            machineEntry.id = machine.post;
            machineEntry.library = *it;
            machineEntry.label = QObject::tr("The machine's default: %1 (from your post library)")
                                     .arg(qs(it->post.manifest().name));
        } else {
            machineEntry.label =
                QObject::tr("The machine's default: '%1' (not found)").arg(qs(machine.post));
        }
    } else {
        document::Setup plain = setup;
        plain.post.reset();
        try {
            const post::PostScript p = document::postFor(project, plain, machine);
            machineEntry.id = p.manifest().id;
            machineEntry.label =
                QObject::tr("The machine's default: %1").arg(qs(p.manifest().name));
        } catch (const Error& e) {
            machineEntry.label = QObject::tr("The machine's default (none)");
            if (problems) {
                *problems << qs(e.what());
            }
        }
    }
    out.push_back(std::move(machineEntry));

    for (const std::string& id : builtins) {
        const post::PostScript& p = post::builtinPost(id);
        if (writesFor(p, machine) || chosen(id, false)) {
            out.push_back({PostEntry::Kind::BuiltIn, id, qs(p.manifest().name), std::nullopt});
        }
    }
    for (const post::PostScript& p : projectPosts) {
        if (writesFor(p, machine) || chosen(p.manifest().id, true)) {
            out.push_back(
                {PostEntry::Kind::InProject, p.manifest().id, qs(p.manifest().name), std::nullopt});
        }
    }
    for (PostLibrary::Entry& e : library) {
        if (writesFor(e.post, machine) && !inProject(e.post.manifest().id)) {
            const std::string id = e.post.manifest().id;
            const QString label = qs(e.post.manifest().name);
            out.push_back({PostEntry::Kind::InLibrary, id, label, std::move(e)});
        }
    }
    return out;
}

QString postKindHeading(PostEntry::Kind kind) {
    switch (kind) {
    case PostEntry::Kind::Machine:
        return QString();
    case PostEntry::Kind::BuiltIn:
        return QObject::tr("Built in");
    case PostEntry::Kind::InProject:
        return QObject::tr("In this project");
    case PostEntry::Kind::InLibrary:
        return QObject::tr("In your library");
    }
    return QString();
}

int currentPostEntry(const std::vector<PostEntry>& entries, const document::Setup& setup) {
    if (!setup.post) {
        return 0;
    }
    const auto kind = setup.post->inProject ? PostEntry::Kind::InProject : PostEntry::Kind::BuiltIn;
    for (std::size_t i = 0; i < entries.size(); ++i) {
        if (entries[i].kind == kind && entries[i].id == setup.post->id) {
            return static_cast<int>(i);
        }
    }
    return 0;
}

std::optional<document::PostChoice> postChoiceOf(const PostEntry& entry) {
    switch (entry.kind) {
    case PostEntry::Kind::Machine:
        return std::nullopt;
    case PostEntry::Kind::BuiltIn:
        return document::PostChoice{entry.id, false};
    case PostEntry::Kind::InProject:
    case PostEntry::Kind::InLibrary:
        break;
    }
    return document::PostChoice{entry.id, true};
}

post::PostScript entryPost(const PostEntry& entry, const document::Project& project,
                           const machine::Machine& machine) {
    if (entry.library) {
        return entry.library->post;
    }
    document::Setup setup;
    setup.post = postChoiceOf(entry);
    return document::postFor(project, setup, machine);
}

void addProjectPost(document::Project& project, const document::ProjectPost& copy) {
    CancelToken cancel;
    const std::string id = post::PostScript::load(copy.source, copy.file, cancel).manifest().id;
    for (document::ProjectPost& p : project.posts) {
        try {
            if (post::PostScript::load(p.source, p.file, cancel).manifest().id == id) {
                p = copy;
                return;
            }
        } catch (const Error&) {
            // a copy that does not load is not this one
        }
    }
    project.posts.push_back(copy);
}

document::ProjectPost projectCopy(const PostLibrary::Entry& entry) {
    const std::u8string file = entry.file.filename().u8string();
    return {std::string(file.begin(), file.end()), entry.source};
}

void addWorkOffsetRows(QFormLayout* form, const document::Project& project,
                       const document::Setup& setup, int current, bool setupsChoice,
                       std::function<void(int)> set) {
    // The setup's post and machine (the generic Fanuc-style ranges when the post is missing).
    post::PostManifest manifest = post::builtinPost("fanuc-generic-mill").manifest();
    int last = 99;
    try {
        const machine::Machine& machine = document::machineFor(project, setup);
        manifest = document::postFor(project, setup, machine).manifest();
        last = document::lastWorkOffset(manifest, machine);
    } catch (const Error&) {
        // the setup page says why
    }
    // The ranges the machine reaches, cut at its count; the one holding `current` even beyond
    // it, so the value shows (posting refuses it).
    std::vector<post::WorkOffsetRange> ranges;
    for (post::WorkOffsetRange r : manifest.workOffsets) {
        const bool holdsCurrent = current >= r.from && current <= r.to;
        r.to = std::min(r.to, std::max(last, holdsCurrent ? current : 0));
        if (r.from <= r.to) {
            ranges.push_back(r);
        }
    }
    const bool useSetups = setupsChoice && current == 0;
    const int shown = useSetups ? setup.workOffset : current;
    int rangeIndex = -1;
    for (std::size_t i = 0; i < ranges.size(); ++i) {
        if (shown >= ranges[i].from && shown <= ranges[i].to) {
            rangeIndex = static_cast<int>(i);
        }
    }

    // Range: the post's ranges by name (and the setup's own work offset for operations).
    constexpr int kSetups = -1;
    auto* range = new QComboBox;
    if (setupsChoice) {
        const auto own = post::workOffsetCode(manifest, setup.workOffset);
        range->addItem(
            QObject::tr("The setup's (%1)").arg(qs(own.value_or(std::to_string(setup.workOffset)))),
            kSetups);
    }
    for (std::size_t i = 0; i < ranges.size(); ++i) {
        const post::WorkOffsetRange& r = ranges[i];
        range->addItem(qs(r.name), static_cast<int>(i));
        range->setItemData(range->count() - 1,
                           qs(fmt::format("{} to {}", *post::workOffsetCode(manifest, r.from),
                                          *post::workOffsetCode(manifest, r.to))),
                           Qt::ToolTipRole);
    }
    range->setCurrentIndex(useSetups ? 0 : std::max(0, range->findData(rangeIndex)));
    range->setToolTip(QObject::tr("The work offset ranges the setup's post writes, as far as the "
                                  "machine has them (machine library: Work offsets)"));
    QObject::connect(range, &QComboBox::currentIndexChanged, range, [range, ranges, set] {
        const int i = range->currentData().toInt();
        set(i == kSetups ? 0 : ranges.at(static_cast<std::size_t>(i)).from);
    });
    form->addRow(QObject::tr("Work offset"), range);

    // The number in the range's code: "G154 P" [2].
    if (useSetups || rangeIndex < 0) {
        if (!useSetups) {
            auto* missing =
                new QLabel(QObject::tr("Work offset %1 is not one the post writes: choose a range")
                               .arg(current));
            missing->setWordWrap(true);
            missing->setStyleSheet(QStringLiteral("color: #c0392b;"));
            form->addRow(QString(), missing);
        }
        return;
    }
    const post::WorkOffsetRange r = ranges[static_cast<std::size_t>(rangeIndex)];
    const auto at = r.code.find("{}");
    auto* number = new QSpinBox;
    number->setRange(r.start, r.start + (r.to - r.from));
    number->setValue(r.start + (shown - r.from));
    number->setSuffix(qs(r.code.substr(at + 2)));
    number->setKeyboardTracking(false); // typed numbers count when entered
    number->setToolTip(QObject::tr("%1 to %2")
                           .arg(qs(*post::workOffsetCode(manifest, r.from)),
                                qs(*post::workOffsetCode(manifest, r.to))));
    QObject::connect(number, &QSpinBox::valueChanged, number,
                     [r, set](int v) { set(r.from + (v - r.start)); });
    auto* label = new QLabel(qs(r.code.substr(0, at)).trimmed());
    label->setProperty("singleLine", true);
    form->addRow(label, number);
    if (shown > last) {
        auto* beyond = new QLabel(QObject::tr("Beyond the machine's %1 work offsets (machine "
                                              "library: Work offsets)")
                                      .arg(last));
        beyond->setWordWrap(true);
        beyond->setStyleSheet(QStringLiteral("color: #c0392b;"));
        form->addRow(QString(), beyond);
    }
}

Json postOptionValue(const Json& values, const post::PostOption& option) {
    if (values.is_object() && values.contains(option.id)) {
        return values.at(option.id);
    }
    return option.defaultValue;
}

QWidget* postOptionEditor(const post::PostOption& option, const Json& value,
                          std::function<void(const Json&)> set) {
    switch (option.type) {
    case post::PostOption::Type::Bool: {
        auto* box = new QCheckBox;
        box->setChecked(value.is_boolean() && value.get<bool>());
        QObject::connect(box, &QCheckBox::toggled, box, [set](bool on) { set(on); });
        return box;
    }
    case post::PostOption::Type::Number: {
        auto* spin = new QDoubleSpinBox;
        spin->setDecimals(4);
        spin->setRange(option.min.value_or(-1e9), option.max.value_or(1e9));
        spin->setValue(value.is_number() ? value.get<double>() : 0.0);
        QObject::connect(spin, &QDoubleSpinBox::editingFinished, spin,
                         [spin, set] { set(spin->value()); });
        return spin;
    }
    case post::PostOption::Type::Choice: {
        auto* combo = new QComboBox;
        for (const std::string& choice : option.choices) {
            combo->addItem(qs(choice));
        }
        if (value.is_string()) {
            combo->setCurrentText(qs(value.get<std::string>()));
        }
        QObject::connect(combo, &QComboBox::currentTextChanged, combo,
                         [set](const QString& text) { set(ss(text)); });
        return combo;
    }
    case post::PostOption::Type::Text:
        break;
    }
    auto* line = new QLineEdit(value.is_string() ? qs(value.get<std::string>()) : QString());
    QObject::connect(line, &QLineEdit::editingFinished, line,
                     [line, set] { set(ss(line->text())); });
    return line;
}

} // namespace ocam::gui
