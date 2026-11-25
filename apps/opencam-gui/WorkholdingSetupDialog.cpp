// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "WorkholdingSetupDialog.hpp"

#include "OcctView.hpp"
#include "Units.hpp"
#include "WorkholdingLibraryDialog.hpp"

#include <ocam/document/PostSetup.hpp>
#include <ocam/document/Workholding.hpp>
#include <ocam/foundation/Error.hpp>
#include <ocam/foundation/Progress.hpp>

#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QDir>
#include <QFile>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QMessageBox>
#include <QPainter>
#include <QPushButton>
#include <QSplitter>
#include <QTimer>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <algorithm>
#include <set>

namespace ocam::gui {
namespace {

using document::HoldingKind;
using document::SetupWorkholding;
using document::WorkholdingPackage;
using document::WorkholdingTemplate;

QString qs(const std::string& s) {
    return QString::fromStdString(s);
}

std::string ss(const QString& s) {
    return s.trimmed().toStdString();
}

// What a row of the lists stands for.
enum class Source { None, Project, Library };
constexpr int kSourceRole = Qt::UserRole;
constexpr int kIdRole = Qt::UserRole + 1;
constexpr int kOptionRole = Qt::UserRole + 2;

QString kindText(const WorkholdingTemplate& t) {
    if (t.jawSet) {
        return t.jawSet->kind == document::JawSet::Kind::Soft ? QObject::tr("Soft jaws")
                                                              : QObject::tr("Hard jaws");
    }
    if (t.holdings.empty()) {
        return QObject::tr("Workholding");
    }
    switch (t.holdings.front().kind) {
    case HoldingKind::Jaws:
        return QObject::tr("Vise");
    case HoldingKind::Dovetail:
        return QObject::tr("Dovetail");
    case HoldingKind::Seat:
        return QObject::tr("Fixture");
    case HoldingKind::Chuck:
        return t.jawMount ? QObject::tr("Chuck (%1)").arg(qs(t.jawMount->interfaceName))
                          : QObject::tr("Chuck");
    }
    return {};
}

QString rowText(const QTreeWidgetItem* item, int columns) {
    QStringList parts;
    for (int c = 0; c < columns; ++c) {
        parts << item->text(c);
    }
    return parts.join(QStringLiteral(" | "));
}

QTreeWidget* newList(const QStringList& headers, const char* name) {
    auto* list = new QTreeWidget;
    list->setObjectName(QString::fromLatin1(name));
    list->setHeaderLabels(headers);
    list->setRootIsDecorated(false);
    list->setUniformRowHeights(true);
    list->setSelectionMode(QAbstractItemView::SingleSelection);
    list->header()->setStretchLastSection(true);
    list->header()->setSectionResizeMode(0, QHeaderView::Stretch);
    for (int c = 1; c < headers.size(); ++c) {
        list->header()->setSectionResizeMode(c, QHeaderView::ResizeToContents);
    }
    list->header()->setStretchLastSection(false);
    return list;
}

} // namespace

WorkholdingSetupDialog::WorkholdingSetupDialog(const document::Document& doc, const Uuid& setupId,
                                               LengthUnit unit, QWidget* parent)
    : QDialog(parent), doc_(doc.snapshot()), setupId_(setupId), unit_(unit) {
    setWindowTitle(tr("Workholding: %1").arg(qs(setup().name)));
    setObjectName(QStringLiteral("workholdingSetupDialog"));
    resize(1200, 820);

    auto* layout = new QVBoxLayout(this);
    auto* splitter = new QSplitter;
    layout->addWidget(splitter, 1);

    // Left: what holds the stock, its jaws, and how it holds it.
    auto* side = new QWidget;
    side->setMinimumWidth(480);
    auto* sideLayout = new QVBoxLayout(side);
    sideLayout->setContentsMargins(0, 0, 0, 0);

    auto* holdingBox = new QGroupBox(tr("Workholding"));
    auto* holdingLayout = new QVBoxLayout(holdingBox);
    holdings_ = newList({tr("Name"), tr("Kind"), tr("From")}, "workholdingSetupHoldings");
    holdings_->setToolTip(tr("A vise, chuck or fixture from this project or the library. Choosing "
                             "a library one copies it into the project."));
    holdingLayout->addWidget(holdings_);
    sideLayout->addWidget(holdingBox, 2);

    jawsBox_ = new QGroupBox(tr("Jaw Selection"));
    auto* jawsLayout = new QVBoxLayout(jawsBox_);
    jaws_ = newList({tr("Name"), tr("Kind"), tr("From")}, "workholdingSetupJaws");
    jaws_->setToolTip(tr("The top jaws modelled with the chuck, or a jaw set for its jaw "
                         "interface, each way it goes on the master jaws"));
    jawsLayout->addWidget(jaws_);
    otherJaws_ = new QLabel;
    otherJaws_->setWordWrap(true);
    otherJaws_->setStyleSheet(QStringLiteral("color: palette(mid)"));
    jawsLayout->addWidget(otherJaws_);
    sideLayout->addWidget(jawsBox_, 2);

    placementBox_ = new QGroupBox(tr("Holding the stock"));
    placementLayout_ = new QVBoxLayout(placementBox_);
    sideLayout->addWidget(placementBox_, 3);
    splitter->addWidget(side);

    // Right: the stock held, and what the project has copied in.
    auto* viewPane = new QWidget;
    auto* viewLayout = new QVBoxLayout(viewPane);
    viewLayout->setContentsMargins(0, 0, 0, 0);
    preview_ = new OcctView;
    preview_->setMinimumSize(420, 340);
    viewLayout->addWidget(preview_, 1);
    auto* fit = new QPushButton(tr("Fit"));
    auto* fitRow = new QHBoxLayout;
    fitRow->addWidget(fit);
    fitRow->addStretch(1);
    viewLayout->addLayout(fitRow);
    summary_ = new QLabel;
    summary_->setObjectName(QStringLiteral("workholdingSetupSummary"));
    summary_->setWordWrap(true);
    viewLayout->addWidget(summary_);
    problem_ = new QLabel;
    problem_->setObjectName(QStringLiteral("workholdingSetupProblem"));
    problem_->setWordWrap(true);
    problem_->setStyleSheet(QStringLiteral("color: #c0392b;"));
    viewLayout->addWidget(problem_);

    auto* copiesBox = new QGroupBox(tr("Copied into this project"));
    auto* copiesLayout = new QVBoxLayout(copiesBox);
    copies_ = newList({tr("Name"), tr("Used by"), tr("Library")}, "workholdingSetupCopies");
    copies_->setToolTip(tr("The project keeps its own copy of every workholding its setups use, "
                           "so it opens the same anywhere. Unused copies are dropped."));
    copies_->setMaximumHeight(130);
    copiesLayout->addWidget(copies_);
    auto* copyButtons = new QHBoxLayout;
    update_ = new QPushButton(tr("Update from library"));
    update_->setObjectName(QStringLiteral("workholdingSetupUpdate"));
    update_->setToolTip(tr("Replace the project's copy with the library's, keeping what every "
                           "setup chose"));
    save_ = new QPushButton(tr("Save to library"));
    save_->setObjectName(QStringLiteral("workholdingSetupSave"));
    save_->setToolTip(tr("Put the project's copy in the library"));
    auto* manage = new QPushButton(tr("Manage library..."));
    copyButtons->addWidget(update_);
    copyButtons->addWidget(save_);
    copyButtons->addStretch(1);
    copyButtons->addWidget(manage);
    copiesLayout->addLayout(copyButtons);
    viewLayout->addWidget(copiesBox);
    splitter->addWidget(viewPane);
    splitter->setStretchFactor(0, 2);
    splitter->setStretchFactor(1, 3);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    layout->addWidget(buttons);

    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    connect(holdings_, &QTreeWidget::itemSelectionChanged, this,
            &WorkholdingSetupDialog::holdingChosen);
    connect(jaws_, &QTreeWidget::itemSelectionChanged, this, &WorkholdingSetupDialog::jawsChosen);
    connect(copies_, &QTreeWidget::itemSelectionChanged, this,
            &WorkholdingSetupDialog::copySelected);
    connect(fit, &QPushButton::clicked, this, [this] { preview_->fitAll(); });
    connect(update_, &QPushButton::clicked, this, [this] {
        if (const QTreeWidgetItem* item = copies_->currentItem()) {
            updateCopy(item->text(0));
        }
    });
    connect(save_, &QPushButton::clicked, this, [this] {
        const QTreeWidgetItem* item = copies_->currentItem();
        const auto id = item ? Uuid::parse(ss(item->data(0, kIdRole).toString())) : std::nullopt;
        if (!id) {
            return;
        }
        if (inLibrary(*id) &&
            QMessageBox::question(this, tr("Save to library"),
                                  tr("Replace the library's %1 with this project's copy?")
                                      .arg(item->text(0))) != QMessageBox::Yes) {
            return;
        }
        try {
            if (const auto package = doc_.workholdingPackage(*id)) {
                WorkholdingLibrary::save(*package);
            }
        } catch (const Error& e) {
            QMessageBox::warning(this, tr("Save to library"), QString::fromUtf8(e.what()));
        }
        reloadLibrary();
        rebuild();
    });
    connect(manage, &QPushButton::clicked, this, [this] {
        WorkholdingLibraryDialog dialog(this);
        dialog.exec();
        reloadLibrary();
        rebuild();
    });

    reloadLibrary();
    rebuild();
}

const document::Setup& WorkholdingSetupDialog::setup() const {
    return *doc_.project().findSetup(setupId_);
}

void WorkholdingSetupDialog::reloadLibrary() {
    library_ = WorkholdingLibrary::load();
}

const WorkholdingPackage* WorkholdingSetupDialog::inLibrary(const Uuid& id) const {
    for (const auto& entry : library_) {
        if (entry.package.workholding.id == id) {
            return &entry.package;
        }
    }
    return nullptr;
}

void WorkholdingSetupDialog::rebuild() {
    filling_ = true;
    fillHoldings();
    fillJaws();
    fillCopies();
    filling_ = false;
    fillPlacement();
    updatePreview();
}

void WorkholdingSetupDialog::fillHoldings() {
    const document::Project& p = doc_.project();
    const auto& w = setup().workholding;
    holdings_->clear();
    const auto add = [&](const QString& name, const QString& kind, const QString& from,
                         Source source, const Uuid* id) {
        auto* item = new QTreeWidgetItem(holdings_, {name, kind, from});
        item->setData(0, kSourceRole, static_cast<int>(source));
        if (id) {
            item->setData(0, kIdRole, qs(id->toString()));
        }
        if ((source == Source::None && !w) ||
            (source == Source::Project && w && id && w->templateId == *id)) {
            holdings_->setCurrentItem(item);
        }
        return item;
    };
    add(tr("(none)"), QString(), QString(), Source::None, nullptr);
    for (const WorkholdingTemplate& t : p.workholdings) {
        if (t.jawSet) {
            continue;
        }
        const WorkholdingPackage* lib = inLibrary(t.id);
        add(qs(t.name), kindText(t),
            !lib                    ? tr("This project only")
            : lib->workholding == t ? tr("This project")
                                    : tr("This project (the library's differs)"),
            Source::Project, &t.id);
    }
    for (const auto& entry : library_) {
        const WorkholdingTemplate& t = entry.package.workholding;
        if (t.jawSet || p.findWorkholding(t.id)) {
            continue;
        }
        add(qs(t.name), kindText(t), tr("Library"), Source::Library, &t.id);
    }
}

void WorkholdingSetupDialog::fillJaws() {
    const document::Project& p = doc_.project();
    const auto& w = setup().workholding;
    jaws_->clear();
    otherJaws_->clear();
    const WorkholdingTemplate* chuck = w ? p.findWorkholding(w->templateId) : nullptr;
    const bool mounts = chuck && chuck->jawMount;
    jawsBox_->setVisible(mounts);
    if (!mounts) {
        return;
    }
    const auto addSet = [&](const WorkholdingTemplate& t, Source source, const QString& from) {
        for (const document::JawOption& o : t.jawSet->options) {
            const QString name = t.jawSet->options.size() > 1
                                     ? tr("%1 (%2)").arg(qs(t.name), qs(o.name))
                                     : qs(t.name);
            auto* item = new QTreeWidgetItem(jaws_, {name, kindText(t), from});
            item->setData(0, kSourceRole, static_cast<int>(source));
            item->setData(0, kIdRole, qs(t.id.toString()));
            item->setData(0, kOptionRole, qs(o.name));
            if (source == Source::Project && w->jawSet == t.id &&
                t.jawSet->findOption(w->jawOption) == &o) {
                jaws_->setCurrentItem(item);
            }
        }
    };
    int other = 0;
    for (const WorkholdingTemplate& t : p.workholdings) {
        if (!t.jawSet) {
            continue;
        }
        if (!document::jawSetFits(*chuck, t) && w->jawSet != t.id) {
            ++other;
            continue;
        }
        const WorkholdingPackage* lib = inLibrary(t.id);
        addSet(t, Source::Project,
               !lib                    ? tr("This project only")
               : lib->workholding == t ? tr("This project")
                                       : tr("This project (the library's differs)"));
    }
    for (const auto& entry : library_) {
        const WorkholdingTemplate& t = entry.package.workholding;
        if (!t.jawSet || p.findWorkholding(t.id)) {
            continue;
        }
        if (!document::jawSetFits(*chuck, t)) {
            ++other;
            continue;
        }
        addSet(t, Source::Library, tr("Library"));
    }
    QStringList notes;
    if (jaws_->topLevelItemCount() == 0) {
        notes << tr("No jaw set in the project or the library fits its interface (%1): make one "
                    "in the library.")
                     .arg(qs(chuck->jawMount->interfaceName));
    }
    if (other > 0) {
        notes << tr("%n more jaw set(s) for other interfaces.", nullptr, other);
    }
    otherJaws_->setText(notes.join(QLatin1Char(' ')));
}

void WorkholdingSetupDialog::pickJaws(const std::vector<WorkholdingTemplate>& earlier) {
    const auto& w = setup().workholding;
    const WorkholdingTemplate* chuck = w ? doc_.project().findWorkholding(w->templateId) : nullptr;
    if (!chuck || !chuck->jawMount || w->jawSet) {
        return;
    }
    // The project's sets (as they were before the chuck changed), then the library's.
    for (const WorkholdingTemplate& t : earlier) {
        if (t.jawSet && !t.jawSet->options.empty() && document::jawSetFits(*chuck, t)) {
            doc_.modify("Put jaws on the chuck", [&](document::Project& p) {
                if (!p.findWorkholding(t.id)) {
                    p.workholdings.push_back(t); // its files stay in the document
                }
                SetupWorkholding& h = *p.findSetup(setupId_)->workholding;
                h.jawSet = t.id;
                h.jawOption = t.jawSet->options.front().name;
                h.step.clear();
            });
            return;
        }
    }
    for (const auto& entry : library_) {
        const WorkholdingTemplate& t = entry.package.workholding;
        if (t.jawSet && !t.jawSet->options.empty() && document::jawSetFits(*chuck, t)) {
            const WorkholdingPackage chosen = entry.package;
            if (useFromLibrary(chosen)) {
                doc_.modify("Put jaws on the chuck", [&](document::Project& p) {
                    p.findSetup(setupId_)->workholding->jawOption =
                        chosen.workholding.jawSet->options.front().name;
                });
            }
            return;
        }
    }
}

void WorkholdingSetupDialog::fillCopies() {
    const document::Project& p = doc_.project();
    const QString current = copies_->currentItem() ? copies_->currentItem()->text(0) : QString();
    copies_->clear();
    for (const WorkholdingTemplate& t : p.workholdings) {
        QStringList users;
        for (const document::Setup& s : p.setups) {
            if (s.workholding && (s.workholding->templateId == t.id || s.workholding->jawSet == t.id)) {
                users << qs(s.name);
            }
        }
        const WorkholdingPackage* lib = inLibrary(t.id);
        auto* item = new QTreeWidgetItem(
            copies_, {qs(t.name), users.isEmpty() ? tr("(none)") : users.join(QStringLiteral(", ")),
                      !lib                    ? tr("Not in it")
                      : lib->workholding == t ? tr("Same")
                                              : tr("Differs")});
        item->setData(0, kIdRole, qs(t.id.toString()));
        if (item->text(0) == current) {
            copies_->setCurrentItem(item);
        }
    }
    copySelected();
}

void WorkholdingSetupDialog::copySelected() {
    const QTreeWidgetItem* item = copies_->currentItem();
    const auto id = item ? Uuid::parse(ss(item->data(0, kIdRole).toString())) : std::nullopt;
    const WorkholdingTemplate* own = id ? doc_.project().findWorkholding(*id) : nullptr;
    const WorkholdingPackage* lib = id ? inLibrary(*id) : nullptr;
    update_->setEnabled(own && lib && lib->workholding != *own);
    save_->setEnabled(own && (!lib || lib->workholding != *own));
}

bool WorkholdingSetupDialog::useFromLibrary(const WorkholdingPackage& package) {
    QApplication::setOverrideCursor(Qt::WaitCursor);
    try {
        CancelToken cancel;
        NullProgress progress;
        doc_.useWorkholding(package, document::importWorkholdingBodies(package, cancel, progress),
                            setupId_);
        QApplication::restoreOverrideCursor();
        return true;
    } catch (const Error& e) {
        QApplication::restoreOverrideCursor();
        QMessageBox::warning(this, tr("Workholding"), QString::fromUtf8(e.what()));
        return false;
    }
}

void WorkholdingSetupDialog::holdingChosen() {
    if (filling_) {
        return;
    }
    const QTreeWidgetItem* item = holdings_->currentItem();
    if (!item) {
        return;
    }
    const auto source = static_cast<Source>(item->data(0, kSourceRole).toInt());
    const auto id = Uuid::parse(ss(item->data(0, kIdRole).toString()));
    const std::vector<WorkholdingTemplate> earlier = doc_.project().workholdings;
    // Jaws that do not go on the chosen chuck come off it.
    const auto fitJaws = [this](document::Project& p) {
        document::Setup& s = *p.findSetup(setupId_);
        if (!s.workholding || !s.workholding->jawSet) {
            return;
        }
        const WorkholdingTemplate* chuck = p.findWorkholding(s.workholding->templateId);
        const WorkholdingTemplate* set = p.findWorkholding(*s.workholding->jawSet);
        if (!chuck || !chuck->jawMount || !set || !document::jawSetFits(*chuck, *set)) {
            s.workholding->jawSet.reset();
            s.workholding->jawOption.clear();
            s.workholding->step.clear();
            s.workholding->softBore.reset();
        }
    };
    try {
        if (source == Source::None) {
            doc_.modify("Remove workholding", [&](document::Project& p) {
                p.findSetup(setupId_)->workholding.reset();
                document::removeUnusedWorkholdings(p);
            });
        } else if (source == Source::Project && id) {
            doc_.modify("Change workholding", [&](document::Project& p) {
                document::Setup& s = *p.findSetup(setupId_);
                SetupWorkholding w = s.workholding.value_or(SetupWorkholding{});
                w.templateId = *id;
                s.workholding = w;
                fitJaws(p);
            });
        } else if (source == Source::Library && id) {
            if (const WorkholdingPackage* package = inLibrary(*id)) {
                const WorkholdingPackage chosen = *package;
                if (useFromLibrary(chosen)) {
                    doc_.modify("Fit jaws", [&](document::Project& p) {
                        fitJaws(p);
                        document::removeUnusedWorkholdings(p);
                    });
                }
            }
        }
        // A chuck that takes jaw sets holds nothing without one: the first that fits goes on.
        pickJaws(earlier);
        doc_.modify("Tidy", [](document::Project& p) { document::removeUnusedWorkholdings(p); });
    } catch (const Error& e) {
        QMessageBox::warning(this, tr("Workholding"), QString::fromUtf8(e.what()));
    }
    rebuild();
}

void WorkholdingSetupDialog::jawsChosen() {
    if (filling_) {
        return;
    }
    const QTreeWidgetItem* item = jaws_->currentItem();
    if (!item || !setup().workholding) {
        return;
    }
    const auto source = static_cast<Source>(item->data(0, kSourceRole).toInt());
    const auto id = Uuid::parse(ss(item->data(0, kIdRole).toString()));
    const std::string option = ss(item->data(0, kOptionRole).toString());
    const auto place = [&](const std::string& label, const std::function<void(SetupWorkholding&)>& f) {
        doc_.modify(label, [&](document::Project& p) {
            f(*p.findSetup(setupId_)->workholding);
            document::removeUnusedWorkholdings(p);
        });
    };
    try {
        if (source == Source::Project && id) {
            place("Change jaw set", [&](SetupWorkholding& h) {
                if (h.jawSet != *id || h.jawOption != option) {
                    h.step.clear(); // another set's or way's steps are its own
                }
                h.jawSet = *id;
                h.jawOption = option;
            });
        } else if (source == Source::Library && id) {
            if (const WorkholdingPackage* package = inLibrary(*id)) {
                const WorkholdingPackage chosen = *package;
                if (useFromLibrary(chosen)) {
                    place("Change jaw set", [&](SetupWorkholding& h) { h.jawOption = option; });
                }
            }
        }
    } catch (const Error& e) {
        QMessageBox::warning(this, tr("Jaws"), QString::fromUtf8(e.what()));
    }
    rebuild();
}

bool WorkholdingSetupDialog::updateCopy(const QString& name) {
    for (const WorkholdingTemplate& t : doc_.project().workholdings) {
        if (qs(t.name) != name) {
            continue;
        }
        const WorkholdingPackage* lib = inLibrary(t.id);
        if (!lib || lib->workholding == t) {
            return false;
        }
        const WorkholdingPackage package = *lib;
        QApplication::setOverrideCursor(Qt::WaitCursor);
        try {
            CancelToken cancel;
            NullProgress progress;
            doc_.updateWorkholding(package,
                                   document::importWorkholdingBodies(package, cancel, progress));
            QApplication::restoreOverrideCursor();
        } catch (const Error& e) {
            QApplication::restoreOverrideCursor();
            QMessageBox::warning(this, tr("Update from library"), QString::fromUtf8(e.what()));
            return false;
        }
        rebuild();
        return true;
    }
    return false;
}

void WorkholdingSetupDialog::updatePreview() {
    const document::Project& p = doc_.project();
    const document::Setup& s = setup();
    std::vector<OcctView::ViewModel> models;
    if (s.modelId) {
        const document::ModelEntry* entry = p.findModel(*s.modelId);
        const cad::Model* model = doc_.model(*s.modelId);
        if (entry && model) {
            models.push_back({entry->id, *model, entry->placement.toParent(), false, false});
        }
    }
    problem_->clear();
    QString what = tr("No workholding.");
    if (s.workholding) {
        const WorkholdingTemplate* t = p.findWorkholding(s.workholding->templateId);
        const WorkholdingTemplate* set =
            s.workholding->jawSet ? p.findWorkholding(*s.workholding->jawSet) : nullptr;
        what = t ? qs(t->name) : tr("(missing)");
        if (set && set->jawSet) {
            const document::JawOption* o = set->jawSet->findOption(s.workholding->jawOption);
            what += tr(", with %1").arg(o && set->jawSet->options.size() > 1
                                            ? tr("%1 (%2)").arg(qs(set->name), qs(o->name))
                                            : qs(set->name));
        }
        try {
            if (const auto pose = document::shownWorkholdingPose(doc_, s)) {
                const Transform3 wcsToProject = s.wcs.toParent();
                for (const document::PosedPart& body : pose->parts) {
                    if (!body.shown) {
                        continue;
                    }
                    if (const cad::Model* m = document::posedPartModel(doc_, body)) {
                        models.push_back({body.id, *m, wcsToProject * body.toWcs, true, false,
                                          body.outOfReach ? std::optional<std::uint32_t>(0xD03A2Fu)
                                                          : std::nullopt});
                    }
                }
                if (pose->outOfReach) {
                    problem_->setText(qs(document::reachMessage(*pose->outOfReach, unit_)) +
                                      QLatin1Char('.'));
                } else if (!pose->refusal.empty()) {
                    problem_->setText(qs(pose->refusal) + QLatin1Char('.'));
                }
            }
        } catch (const Error& e) {
            problem_->setText(QString::fromUtf8(e.what()));
        }
        if (problem_->text().isEmpty() && t && t->jawMount && !set) {
            problem_->setText(tr("choose the chuck's jaws in Jaw Selection."));
        }
        const QString text = problem_->text();
        if (!text.isEmpty()) {
            problem_->setText(text.left(1).toUpper() + text.mid(1));
        }
    }
    summary_->setText(what);
    // As the main view: in the setup's axes, turned as its machine stands (DL-0077).
    Mat3 frame = Mat3::Identity();
    Mat3 standing = Mat3::Identity();
    try {
        frame = s.wcs.normalized().toParent().linear();
    } catch (const Error&) {
    }
    if (const machine::Machine* m = document::findMachineFor(p, s)) {
        standing = m->standing().worldAxes();
    }
    const bool turned = preview_->setReferenceFrame(frame, standing);
    preview_->setModels(models);
    preview_->setStock(doc_.stockShape(setupId_));
    if (turned || !fitted_) {
        preview_->snapView(V3d_XposYnegZpos, false);
        preview_->fitAll();
        fitted_ = true;
    }
}

void WorkholdingSetupDialog::rebuildLater() {
    if (rebuildPending_) {
        return;
    }
    rebuildPending_ = true;
    // After the edit's signal returns: the form that sent it is replaced.
    QTimer::singleShot(0, this, [this] {
        rebuildPending_ = false;
        rebuild();
    });
}

void WorkholdingSetupDialog::place(const std::string& label,
                                   const std::function<void(SetupWorkholding&)>& change) {
    try {
        doc_.modify(label, [&](document::Project& p) {
            if (document::Setup* s = p.findSetup(setupId_); s && s->workholding) {
                change(*s->workholding);
            }
            document::removeUnusedWorkholdings(p);
        });
    } catch (const Error& e) {
        QMessageBox::warning(this, tr("Workholding"), QString::fromUtf8(e.what()));
    }
    rebuildLater();
}

void WorkholdingSetupDialog::fillPlacement() {
    using document::GripSide;
    using document::JawSetting;
    delete placement_; // its widgets' signals have returned (rebuildLater)
    placement_ = new QWidget;
    placementLayout_->addWidget(placement_);
    auto* form = new QFormLayout(placement_);
    form->setContentsMargins(0, 0, 0, 0);
    form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);

    const document::Project& p = doc_.project();
    const document::Setup& s = setup();
    const WorkholdingTemplate* t =
        s.workholding ? p.findWorkholding(s.workholding->templateId) : nullptr;
    if (!s.workholding || !t) {
        form->addRow(new QLabel(tr("Choose a workholding.")));
        return;
    }
    const SetupWorkholding& w = *s.workholding;
    const document::StockHolding* holding =
        t->holdings.empty() ? nullptr
                            : (w.holding.empty() ? &t->holdings.front() : t->findHolding(w.holding));
    if (t->holdings.size() > 1) {
        auto* choice = new QComboBox;
        choice->setObjectName(QStringLiteral("setupHolding"));
        for (const auto& h : t->holdings) {
            choice->addItem(qs(h.name), qs(h.name));
        }
        choice->setCurrentIndex(std::max(0, choice->findData(qs(holding ? holding->name : ""))));
        choice->setToolTip(tr("Where in the workholding the stock sits"));
        connect(choice, &QComboBox::currentIndexChanged, this, [this, choice] {
            const std::string name = ss(choice->currentData().toString());
            place("Change stock holding", [&](SetupWorkholding& h) { h.holding = name; });
        });
        form->addRow(tr("Holding"), choice);
    }
    const HoldingKind kind = holding ? holding->kind : HoldingKind::Jaws;
    const bool chuck = kind == HoldingKind::Chuck;
    const WorkholdingTemplate* set = w.jawSet ? p.findWorkholding(*w.jawSet) : nullptr;
    const bool jawSetOn = chuck && t->jawMount && set && set->jawSet;
    const bool soft = jawSetOn && set->jawSet->kind == document::JawSet::Kind::Soft;

    if (kind != HoldingKind::Seat) {
        const bool dovetail = kind == HoldingKind::Dovetail;
        auto* jaws = new QComboBox;
        jaws->setObjectName(QStringLiteral("setupJawPosition"));
        jaws->addItem(dovetail ? tr("The holding's tenon width") : tr("Auto"),
                      static_cast<int>(JawSetting::OnStock));
        jaws->addItem(dovetail ? tr("A tenon width of") : tr("Manual"),
                      static_cast<int>(JawSetting::Opening));
        jaws->setCurrentIndex(jaws->findData(static_cast<int>(w.jaws)));
        jaws->setToolTip(dovetail ? tr("The width of the stock's dovetail tenon at its shoulder; "
                                       "the moving flank closes on it")
                                  : tr("Auto: the jaws close on the stock. Manual: they stand "
                                       "at the opening you type."));
        connect(jaws, &QComboBox::currentIndexChanged, this, [this, jaws] {
            const auto setting = static_cast<JawSetting>(jaws->currentData().toInt());
            // A typed opening starts from the opening the members have now.
            double now = 0.0;
            try {
                if (const auto pose = document::shownWorkholdingPose(doc_, setup())) {
                    now = pose->opening;
                }
            } catch (const Error&) {
            }
            place("Change jaws", [&](SetupWorkholding& h) {
                h.jaws = setting;
                if (setting == JawSetting::Opening && h.opening <= 0.0) {
                    h.opening = now;
                }
            });
        });
        form->addRow(dovetail ? tr("Tenon") : tr("Jaw position"), jaws);
        if (w.jaws == JawSetting::Opening) {
            auto* opening = new LengthSpinBox(w.opening, unit_, 0.0);
            opening->setObjectName(QStringLiteral("setupJawOpening"));
            opening->setToolTip(dovetail ? tr("The tenon's width at its shoulder")
                                : chuck  ? tr("The diameter the jaws are open to")
                                         : tr("The distance between the jaw faces"));
            connect(opening, &LengthSpinBox::mmChanged, this, [this](double v) {
                place("Change jaw opening", [&](SetupWorkholding& h) { h.opening = v; });
            });
            form->addRow(dovetail ? tr("Tenon width") : tr("Opening"), opening);
        }
    }
    // With a jaw set on the chuck, its step's stop face sets how far the work goes in.
    if (kind == HoldingKind::Jaws || (chuck && !jawSetOn)) {
        auto* grip =
            new LengthSpinBox(w.gripDepth.value_or(holding ? holding->gripDepth : 0.0), unit_, 0.0);
        grip->setObjectName(QStringLiteral("setupGripDepth"));
        grip->setToolTip(chuck ? tr("How far the bar reaches into the jaws, behind their front "
                                    "(the holding's own until changed)")
                               : tr("How far the stock reaches down into the jaws, below their "
                                    "tops (the holding's own until changed)"));
        connect(grip, &LengthSpinBox::mmChanged, this, [this](double v) {
            place("Change grip depth", [&](SetupWorkholding& h) { h.gripDepth = v; });
        });
        form->addRow(chuck ? tr("Grip length") : tr("Grip depth"), grip);
    }
    auto* along = new QComboBox;
    along->setObjectName(QStringLiteral("setupJawAlong"));
    along->addItem(tr("WCS X"), 0);
    along->addItem(tr("WCS Y"), 90);
    along->addItem(tr("WCS -X"), 180);
    along->addItem(tr("WCS -Y"), 270);
    along->setCurrentIndex(std::max(0, along->findData(w.rotation)));
    along->setToolTip(kind == HoldingKind::Seat
                          ? tr("The stock's direction that lies along the seat's X axis")
                      : chuck ? tr("The direction of the jaw with the gripping face (the bar is "
                                   "centred on the spindle either way)")
                              : tr("The stock's direction across the jaws (from face A towards "
                                   "face B)"));
    connect(along, &QComboBox::currentIndexChanged, this, [this, along] {
        const int rotation = along->currentData().toInt();
        place("Turn workholding", [&](SetupWorkholding& h) { h.rotation = rotation; });
    });
    form->addRow(kind == HoldingKind::Seat ? tr("Stock X along")
                 : chuck                   ? tr("First jaw along")
                                           : tr("Jaws close along"),
                 along);
    if (kind != HoldingKind::Seat && !chuck) {
        auto* shift = new LengthSpinBox(w.shift, unit_);
        shift->setObjectName(QStringLiteral("setupJawShift"));
        shift->setToolTip(tr("Moves the stock along the jaws, away from their centre"));
        connect(shift, &LengthSpinBox::mmChanged, this, [this](double v) {
            place("Shift stock in the jaws", [&](SetupWorkholding& h) { h.shift = v; });
        });
        form->addRow(tr("Shift along jaws"), shift);
    }

    std::optional<document::JawPlacement> placed;
    if (jawSetOn) {
        auto* grip = new QComboBox;
        grip->setObjectName(QStringLiteral("setupJawGrip"));
        grip->addItem(tr("OD"), static_cast<int>(GripSide::Outside));
        grip->addItem(tr("ID"), static_cast<int>(GripSide::Inside));
        grip->setCurrentIndex(w.grip == GripSide::Inside ? 1 : 0);
        grip->setToolTip(tr("What the jaws hold: the stock's outside, or a bore in it (with the "
                            "jaws opening into it)"));
        connect(grip, &QComboBox::currentIndexChanged, this, [this, grip] {
            const auto side = static_cast<GripSide>(grip->currentData().toInt());
            place("Change grip", [&](SetupWorkholding& h) {
                h.grip = side;
                h.step.clear();
            });
        });
        form->addRow(tr("Part gripping"), grip);
        const document::JawOption* option = set->jawSet->findOption(w.jawOption);
        auto* step = new QComboBox;
        step->setObjectName(QStringLiteral("setupJawStep"));
        step->addItem(tr("Auto"), QString());
        if (option) {
            for (const document::JawStep& st : option->steps) {
                step->addItem(qs(st.name), qs(st.name));
            }
        }
        step->setCurrentIndex(std::max(0, step->findData(qs(w.step))));
        step->setToolTip(soft ? tr("The step the jaws clamp the boring ring or plug by (Auto: "
                                   "the first)")
                              : tr("The step that grips, its face against the work (Auto: the "
                                   "first that grips OD or ID as chosen)"));
        connect(step, &QComboBox::currentIndexChanged, this, [this, step] {
            const std::string name = ss(step->currentData().toString());
            place("Change jaw step", [&](SetupWorkholding& h) { h.step = name; });
        });
        form->addRow(soft ? tr("Clamps on") : tr("Step"), step);
        auto* offset = new LengthSpinBox(w.stopOffset, unit_, 0.0);
        offset->setObjectName(QStringLiteral("setupJawStopOffset"));
        connect(offset, &LengthSpinBox::mmChanged, this, [this](double v) {
            place("Change jaw face offset", [&](SetupWorkholding& h) { h.stopOffset = v; });
        });
        form->addRow(tr("Jaw face offset"), offset);

        // Where the jaws went, as the pose has it. Why they cannot is said under the view, once.
        try {
            if (const auto pose = document::shownWorkholdingPose(doc_, s);
                pose && !pose->outOfReach && pose->refusal.empty()) {
                placed = pose->jaws;
            }
        } catch (const Error&) {
        }
        if (soft) {
            const document::SoftBore bore = w.softBore.value_or(document::SoftBore{});
            auto* fromStock = new QCheckBox(tr("From the held diameter"));
            fromStock->setObjectName(QStringLiteral("setupSoftBoreFromStock"));
            fromStock->setChecked(!bore.diameter);
            fromStock->setToolTip(tr("Bored to the diameter of the stock where the jaws hold it, "
                                     "plus the clearance (minus it for an inside grip); off: "
                                     "typed"));
            const double boredTo = placed && placed->boreDiameter ? *placed->boreDiameter : 0.0;
            connect(fromStock, &QCheckBox::toggled, this, [this, boredTo](bool on) {
                place("Change soft jaw bore", [&](SetupWorkholding& h) {
                    document::SoftBore b = h.softBore.value_or(document::SoftBore{});
                    b.diameter = on ? std::nullopt : std::optional<double>(boredTo);
                    h.softBore = b;
                });
            });
            form->addRow(tr("Bore"), fromStock);
            const auto boreField = [&](const QString& label, const QString& tip, double value,
                                       const char* name, double lo,
                                       std::function<void(document::SoftBore&, double)> set) {
                auto* box = new LengthSpinBox(value, unit_, lo);
                box->setObjectName(QString::fromLatin1(name));
                box->setToolTip(tip);
                connect(box, &LengthSpinBox::mmChanged, this, [this, set](double v) {
                    place("Change soft jaw bore", [&](SetupWorkholding& h) {
                        document::SoftBore b = h.softBore.value_or(document::SoftBore{});
                        set(b, v);
                        h.softBore = b;
                    });
                });
                form->addRow(label, box);
            };
            if (bore.diameter) {
                boreField(tr("Bore diameter"), tr("What the jaws are bored (or turned) to"),
                          *bore.diameter, "setupSoftBoreDiameter", 0.0,
                          [](document::SoftBore& b, double v) { b.diameter = v; });
            } else {
                boreField(tr("Clearance"),
                          tr("How much looser than the held diameter: added for an outside grip, "
                             "taken off for an inside one"),
                          bore.clearance, "setupSoftBoreClearance", -100.0,
                          [](document::SoftBore& b, double v) { b.clearance = v; });
            }
            boreField(tr("Bore depth"), tr("How far back from the jaws' front they are bored"),
                      bore.depth, "setupSoftBoreDepth", 0.0,
                      [](document::SoftBore& b, double v) { b.depth = v; });
            boreField(tr("Clamping while bored"),
                      tr("The diameter of the ring or plug the jaws clamp on their step while "
                         "they are bored"),
                      bore.clampDiameter, "setupSoftBoreClamp", 0.0,
                      [](document::SoftBore& b, double v) { b.clampDiameter = v; });
        }
    }

    if (holding) {
        // Members that move on their own and do not close on the stock: typed travel.
        std::set<std::string> closing;
        if (kind != HoldingKind::Seat) {
            for (const std::string& face : {holding->faceA, holding->faceB}) {
                std::string datum = face;
                for (int depth = 0; depth < 16; ++depth) {
                    const document::WorkholdingDatum* d = t->findDatum(datum);
                    if (!d) {
                        break;
                    }
                    if (d->kind == document::DatumKind::Typed) {
                        closing.insert(d->member);
                        break;
                    }
                    if (d->kind == document::DatumKind::Picked) {
                        if (const auto* part = d->part ? t->findPart(*d->part) : nullptr) {
                            closing.insert(part->member);
                        }
                        break;
                    }
                    datum = d->from;
                }
            }
        }
        for (const auto& m : t->members) {
            if (!m.linkedTo.empty() || closing.contains(m.name)) {
                continue;
            }
            const auto it = w.travel.find(m.name);
            auto* travel = new LengthSpinBox(it == w.travel.end() ? 0.0 : it->second, unit_,
                                             m.minTravel, m.maxTravel);
            travel->setToolTip(tr("Where this member stands, from where it is modelled"));
            const std::string name = m.name;
            connect(travel, &LengthSpinBox::mmChanged, this, [this, name](double v) {
                place("Move " + name, [&](SetupWorkholding& h) { h.travel[name] = v; });
            });
            form->addRow(tr("%1 travel").arg(qs(m.name)), travel);
        }
    }

    // What the jaws come to.
    QString text;
    if (placed) {
        const document::JawPlacement& j = *placed;
        text = tr("%1 holds Ø%2, master jaw 1 at %3 (its travel: %4 to %5).")
                   .arg(qs(j.step), formatLength(j.gripDiameter, unit_),
                        formatLength(j.travel, unit_), formatLength(j.minTravel, unit_),
                        formatLength(j.maxTravel, unit_));
        if (j.boreDiameter) {
            text += QLatin1Char(' ') +
                    tr("Bored to Ø%1, %2 deep, clamping Ø%3 with master jaw 1 at %4.")
                        .arg(formatLength(*j.boreDiameter, unit_), formatLength(j.boreDepth, unit_),
                             formatLength(j.clampDiameter, unit_),
                             formatLength(j.boringTravel, unit_));
        }
    }
    auto* note = new QLabel(text);
    note->setObjectName(QStringLiteral("setupJawPlacement"));
    note->setWordWrap(true);
    note->setVisible(!text.isEmpty());
    form->addRow(note);
}

QStringList WorkholdingSetupDialog::holdingRows() const {
    QStringList out;
    for (int i = 0; i < holdings_->topLevelItemCount(); ++i) {
        out << rowText(holdings_->topLevelItem(i), 3);
    }
    return out;
}

QStringList WorkholdingSetupDialog::jawRows() const {
    QStringList out;
    if (!jawsBox_->isHidden()) {
        for (int i = 0; i < jaws_->topLevelItemCount(); ++i) {
            out << rowText(jaws_->topLevelItem(i), 3);
        }
    }
    return out;
}

QStringList WorkholdingSetupDialog::copyRows() const {
    QStringList out;
    for (int i = 0; i < copies_->topLevelItemCount(); ++i) {
        out << rowText(copies_->topLevelItem(i), 3);
    }
    return out;
}

bool WorkholdingSetupDialog::chooseHolding(const QString& name) {
    for (int i = 0; i < holdings_->topLevelItemCount(); ++i) {
        if (holdings_->topLevelItem(i)->text(0) == name) {
            holdings_->setCurrentItem(holdings_->topLevelItem(i));
            return true;
        }
    }
    return false;
}

bool WorkholdingSetupDialog::chooseJaws(const QString& name) {
    for (int i = 0; i < jaws_->topLevelItemCount(); ++i) {
        if (jaws_->topLevelItem(i)->text(0) == name) {
            jaws_->setCurrentItem(jaws_->topLevelItem(i));
            return true;
        }
    }
    return false;
}

QString WorkholdingSetupDialog::problemText() const {
    return problem_->text();
}

QImage WorkholdingSetupDialog::snapshot() {
    QImage image = grab().toImage();
    const QString file = QDir::temp().filePath(QStringLiteral("oxcam-workholding-setup-view.png"));
    if (preview_->dumpImage(file)) {
        const QImage view(file);
        QPainter painter(&image);
        const QPoint at = preview_->mapTo(this, QPoint(0, 0));
        painter.drawImage(QRectF(at.x(), at.y(), preview_->width(), preview_->height()), view);
        QFile::remove(file);
    }
    return image;
}

} // namespace ocam::gui
