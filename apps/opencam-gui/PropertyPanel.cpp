// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "PropertyPanel.hpp"

#include "ComboLists.hpp"
#include "LatheFittingsDialog.hpp"
#include "MachineLibrary.hpp"
#include "ParamEditors.hpp"
#include "PostChoices.hpp"
#include "PostLibrary.hpp"
#include "ToolLibraries.hpp"
#include "ToolTips.hpp"
#include "Translations.hpp"
#include "Units.hpp"
#include "WorkholdingLibrary.hpp"

#include <ocam/cad/Features.hpp>
#include <ocam/document/Generate.hpp>
#include <ocam/document/LatheMachine.hpp>
#include <ocam/document/PostSetup.hpp>
#include <ocam/document/Probing.hpp>
#include <ocam/document/Wcs.hpp>
#include <ocam/document/Workholding.hpp>
#include <ocam/foundation/Error.hpp>
#include <ocam/strategies/Common.hpp>
#include <ocam/strategies/Probing.hpp>

#include <QApplication>
#include <QButtonGroup>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScopeGuard>
#include <QSpinBox>
#include <QStandardItemModel>
#include <QTextDocument>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>
#include <limits>
#include <set>
#include <utility>

namespace ocam::gui {
namespace {

/// Operations that follow closed loops of edges (contours, chamfers).
bool takesEdges(const std::string& type) {
    const strategies::StrategyInfo* info = strategies::findStrategy(type);
    return info && (info->geometry == strategies::GeometryUse::FacesOrEdges ||
                    info->geometry == strategies::GeometryUse::Edges);
}

using document::Project;
using document::Setup;
using document::StockKind;

/// Picking faces on a probing operation fills in the nominal values they give (DL-0090). Faces
/// that are not yet a feature it measures leave the values as they are; the toolpath says why.
void fillProbeNominals(const document::Document& doc, Project& pr, const Uuid& id) {
    const Setup* owner = nullptr;
    const document::Operation* op = std::as_const(pr).findOperation(id, &owner);
    if (!op || !owner || !strategies::isProbingStrategy(op->type) || op->geometry.empty()) {
        return;
    }
    try {
        Json params = document::withProbeNominals(doc, *owner, *op);
        pr.findOperation(id)->parameters = std::move(params);
    } catch (const Error&) {
        // not (yet) a feature the operation measures
    }
}

QString qs(const std::string& s) {
    return QString::fromStdString(s);
}

std::string ss(const QString& s) {
    return s.toStdString();
}

// Operation types offered in the editor: every registered strategy.
QList<QPair<QString, QString>> operationTypes() {
    QList<QPair<QString, QString>> types;
    for (const auto& info : strategies::strategies()) {
        types.append({QString::fromStdString(info.id), trStrategy(info.label)});
    }
    return types;
}

} // namespace

PropertyPanel::PropertyPanel(QWidget* parent) : QScrollArea(parent) {
    setWidgetResizable(true);
    // A page wider than the dock (narrower than its fields can go) scrolls, never cut off.
    setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    setMinimumWidth(240);
}

QPushButton* PropertyPanel::pickButton(const QString& id, const QString& text) {
    auto* button = new QPushButton(text);
    button->setCheckable(true);
    button->setChecked(pending_ && pending_->id == id);
    return button;
}

void PropertyPanel::withPick(const QString& id, std::optional<cad::SubShapeType> type,
                             const QString& prompt,
                             std::function<void(const document::ShapeRef&)> apply) {
    if (pending_ && pending_->id == id) {
        cancelPick(); // the button again: stop waiting
        return;
    }
    std::vector<document::ShapeRef> refs =
        type ? picked(*type)
             : (pickedShapes_ ? pickedShapes_() : std::vector<document::ShapeRef>{});
    if (!refs.empty()) {
        cancelPick();
        apply(refs.front());
        return;
    }
    // Nothing suitable is selected: the next pick in the view finishes the action.
    pending_ = PendingPick{id, type, std::move(apply)};
    emit pickPrompt(prompt + tr(" (Esc cancels)"));
    refreshLater(); // shows the button as waiting
}

void PropertyPanel::startWcsPick(const Uuid& setupId, WcsTarget target) {
    if (wcsPick_ && wcsPick_->setup == setupId && wcsPick_->target == target) {
        cancelPick(); // the button again: stop waiting
        return;
    }
    cancelPick();
    wcsPick_ = WcsPick{setupId, target};
    emit workholdingPickingRequested(true);
    emit vertexPickingRequested(target == WcsTarget::Origin);
    emit pickPrompt(target == WcsTarget::Origin
                        ? tr("Click a circular edge, a face or a corner of the part or a fixture "
                             "for the WCS origin (Esc cancels)")
                        : tr("Click a face or edge of the part or a fixture for the WCS axis "
                             "(Esc cancels)"));
    refreshLater(); // shows the button as waiting
}

void PropertyPanel::stopWcsPick() {
    if (wcsPick_) {
        wcsPick_.reset();
        emit workholdingPickingRequested(false);
        emit vertexPickingRequested(false);
        emit pickPrompt(QString());
        refreshLater();
    }
}

bool PropertyPanel::applyWcsPick(const ViewPick& pick) {
    const WcsPick want = *wcsPick_;
    const Setup* setup = doc_->project().findSetup(want.setup);
    if (!setup) {
        stopWcsPick();
        return true;
    }
    // On the part or a fixture model (a project model), or on a workholding part.
    const document::ModelEntry* entry = doc_->project().findModel(pick.model);
    const cad::Model* model = entry ? doc_->model(pick.model) : nullptr;
    const document::WorkholdingTemplate* holder =
        setup->workholding ? doc_->project().findWorkholding(setup->workholding->templateId)
                           : nullptr;
    const document::WorkholdingPart* part = holder ? holder->findPart(pick.model) : nullptr;
    if (!model && part) {
        model = doc_->modelData(part->sha256);
    }
    if (!model) {
        return false;
    }
    const cad::SubShapeGeometry g = cad::subShapeGeometry(*model, pick.type, pick.index);
    using Kind = cad::SubShapeGeometry::Kind;
    const QString what = pick.type == cad::SubShapeType::Vertex ? tr("Corner")
                         : g.kind == Kind::Circle               ? tr("Circle")
                         : pick.type == cad::SubShapeType::Edge ? tr("Edge")
                         : g.kind == Kind::Plane                ? tr("Face")
                                                                : tr("Round face");
    const QString label = part ? tr("%1 of '%2' (workholding)").arg(what, qs(part->name))
                               : tr("%1 of '%2'").arg(what, qs(entry->name));
    try {
        std::optional<document::ShapeRef> ref;
        std::optional<document::WorkholdingDatum> datum;
        if (part) {
            datum = document::pickedDatum("WCS pick", *part, *model, pick.type, pick.index);
        } else {
            ref = document::ShapeRef{pick.model, pick.type, pick.index,
                                     cad::signatureOf(*model, pick.type, pick.index)};
        }
        if (want.target == WcsTarget::Origin) {
            stopWcsPick();
            commit("Pick WCS origin", [&](Project& p) {
                Setup* s = p.findSetup(want.setup);
                if (!s) {
                    return;
                }
                document::WcsOrigin o = s->originFrom.value_or(document::WcsOrigin{});
                o.from = document::OriginFrom::Point;
                o.point = ref;
                o.workholdingPoint = datum;
                o.label = label.toStdString();
                s->originFrom = o;
            });
            return true;
        }
        // An axis: its direction now, project frame (one on the workholding as it is placed).
        const Vec3 direction = ref ? document::axisOfShape(*doc_, *ref)
                                   : document::workholdingPoint(*doc_, *setup, *datum).direction;
        if (direction.norm() < 1e-9) {
            throw Error(ErrorCode::InvalidArgument,
                        "pick a flat or round face, or a straight or circular edge");
        }
        stopWcsPick();
        commit("Pick WCS axis", [&](Project& p) {
            Setup* s = p.findSetup(want.setup);
            if (!s || !s->orientation) {
                return;
            }
            document::WcsOrientation o = *s->orientation;
            auto& slot = want.target == WcsTarget::ZAxis ? o.z : o.first;
            const bool flip = slot && slot->flip;
            slot = document::AxisPick{label.toStdString(), ref, direction, flip};
            // On the workholding: the WCS turns, the workholding stays where it is.
            document::setOrientation(*s, o, datum.has_value());
        });
    } catch (const Error& e) {
        emit pickPrompt(QString::fromUtf8(e.what()));
    }
    return true;
}

void PropertyPanel::cancelPick() {
    stopWcsPick();
    stopGeometryPick();
    emit vertexPickingRequested(false);
    if (pending_) {
        pending_.reset();
        emit pickPrompt(QString());
        refreshLater();
    }
}

void PropertyPanel::refreshLater() {
    // Not at once: the page being replaced may own the button that is being clicked.
    QTimer::singleShot(0, this, [this] { showSelection(selection_); });
}

void PropertyPanel::viewSelectionChanged(bool individual) {
    if (wcsPick_ && doc_) {
        // A face, edge or vertex for the WCS: the first one on the part, a fixture or the
        // workholding finishes it; anything else keeps it waiting.
        for (const ViewPick& pick : viewPicks_ ? viewPicks_() : std::vector<ViewPick>{}) {
            if (applyWcsPick(pick)) {
                return;
            }
        }
        return;
    }
    if (geometryPick_ && doc_) {
        const Uuid id = *geometryPick_;
        const Setup* owner = nullptr;
        const document::Operation* op = doc_->project().findOperation(id, &owner);
        if (!op || !owner) {
            stopGeometryPick();
            return;
        }
        std::vector<document::ShapeRef> refs =
            pickedShapes_ ? pickedShapes_() : std::vector<document::ShapeRef>{};
        const bool contour = takesEdges(op->type);
        std::erase_if(refs, [&](const document::ShapeRef& r) { return !accepts(*op, *owner, r); });
        if (contour && !individual) {
            // One click on an edge takes (or drops) its whole outline; Shift picks single edges.
            auto same = [](const document::ShapeRef& a, const document::ShapeRef& b) {
                return a.modelId == b.modelId && a.type == b.type && a.index == b.index;
            };
            auto has = [&](const std::vector<document::ShapeRef>& list,
                           const document::ShapeRef& r) {
                return std::any_of(list.begin(), list.end(),
                                   [&](const document::ShapeRef& x) { return same(x, r); });
            };
            auto loopOf = [&](const document::ShapeRef& r) {
                std::vector<document::ShapeRef> loop;
                const cad::Model* model = doc_->model(r.modelId);
                const document::ModelEntry* entry = doc_->project().findModel(r.modelId);
                if (!model || !entry) {
                    return std::vector<document::ShapeRef>{r};
                }
                // The tool axis in the model's own coordinates.
                const Vec3 axis = entry->placement.toParent().linear().transpose() *
                                  owner->wcs.zAxis.normalized();
                for (const int index : cad::edgeLoopOf(*model, r.index, axis)) {
                    loop.push_back({r.modelId, cad::SubShapeType::Edge, index,
                                    cad::signatureOf(*model, cad::SubShapeType::Edge, index)});
                }
                return loop;
            };
            std::vector<document::ShapeRef> result = refs;
            for (const auto& r : refs) {
                if (r.type == cad::SubShapeType::Edge && !has(op->geometry, r)) {
                    for (const auto& e : loopOf(r)) {
                        if (!has(result, e)) {
                            result.push_back(e);
                        }
                    }
                }
            }
            for (const auto& r : op->geometry) {
                if (r.type == cad::SubShapeType::Edge && !has(refs, r)) {
                    for (const auto& e : loopOf(r)) {
                        std::erase_if(result,
                                      [&](const document::ShapeRef& x) { return same(x, e); });
                    }
                }
            }
            if (result.size() != refs.size()) {
                emit highlightShapes(result); // show the whole outlines selected
            }
            refs = std::move(result);
        }
        // Faces and edges already there keep their side (flipped or not).
        for (document::ShapeRef& r : refs) {
            for (const document::ShapeRef& old : op->geometry) {
                if (old.sameShape(r)) {
                    r.side = old.side;
                }
            }
        }
        if (refs != op->geometry) {
            commit("Pick operation geometry", [&](Project& pr) {
                if (document::Operation* o = pr.findOperation(id)) {
                    o->geometry = refs;
                    fillProbeNominals(*doc_, pr, id);
                }
            });
        }
        return;
    }
    if (!pending_) {
        return;
    }
    std::vector<document::ShapeRef> refs =
        pending_->type ? picked(*pending_->type)
                       : (pickedShapes_ ? pickedShapes_() : std::vector<document::ShapeRef>{});
    if (refs.empty()) {
        return; // nothing, or the wrong kind: keep waiting
    }
    const auto apply = std::move(pending_->apply);
    pending_.reset();
    emit pickPrompt(QString());
    apply(refs.front()); // commits, which rebuilds the page
    refreshLater();
}

std::vector<document::ShapeRef> PropertyPanel::picked(cad::SubShapeType type) const {
    std::vector<document::ShapeRef> refs =
        pickedShapes_ ? pickedShapes_() : std::vector<document::ShapeRef>{};
    std::erase_if(refs, [type](const document::ShapeRef& r) { return r.type != type; });
    return refs;
}

void PropertyPanel::compactPage() {
    QWidget* page = widget();
    if (!page) {
        return;
    }
    // Combo boxes size to a few characters (eliding long names), buttons and labels to the
    // width they are given, so the page follows the dock's width.
    for (auto* combo : page->findChildren<QComboBox*>()) {
        combo->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
        combo->setMinimumContentsLength(6);
        // Open, the list shows whole names however narrow the box.
        showEntriesInFull(combo);
    }
    for (auto* button : page->findChildren<QPushButton*>()) {
        if (button->property("keepWidth").toBool()) {
            continue; // a button beside a field keeps its text's width
        }
        button->setMinimumWidth(0);
        button->setSizePolicy(QSizePolicy::Ignored, button->sizePolicy().verticalPolicy());
    }
    for (auto* label : page->findChildren<QLabel*>()) {
        label->setWordWrap(!label->property("singleLine").toBool());
    }
    for (auto* form : page->findChildren<QFormLayout*>()) {
        form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        // A row's label explains it, titled with its name; its input field shows no tooltip
        // of its own (it would cover what is being typed or chosen).
        for (int row = 0; row < form->rowCount(); ++row) {
            QLayoutItem* labelItem = form->itemAt(row, QFormLayout::LabelRole);
            QLayoutItem* fieldItem = form->itemAt(row, QFormLayout::FieldRole);
            auto* label = labelItem ? qobject_cast<QLabel*>(labelItem->widget()) : nullptr;
            if (!label || label->text().isEmpty()) {
                continue;
            }
            QString tip = label->toolTip();
            QWidget* field = fieldItem ? fieldItem->widget() : nullptr;
            if (field &&
                (qobject_cast<QAbstractSpinBox*>(field) || qobject_cast<QComboBox*>(field) ||
                 qobject_cast<QLineEdit*>(field) || qobject_cast<QCheckBox*>(field))) {
                if (tip.isEmpty()) {
                    tip = field->toolTip();
                }
                field->setToolTip(QString());
            }
            if (!tip.isEmpty() && !Qt::mightBeRichText(tip)) {
                label->setToolTip(formatToolTip(tip, label->text()));
            }
            // A label too long to sit beside its field gets a line of its own above it,
            // wrapping across the page. In the label column it widened the column for every
            // row (squeezing all the fields), or wrapped into a row too short to show it.
            const int cap = label->fontMetrics().averageCharWidth() * 26;
            if (label->fontMetrics().horizontalAdvance(label->text()) > cap && fieldItem) {
                const QFormLayout::TakeRowResult taken = form->takeRow(row);
                QWidget* labelWidget = taken.labelItem->widget();
                QLayoutItem* item = taken.fieldItem;
                delete taken.labelItem; // the item, not the widget
                form->insertRow(row, labelWidget);
                if (QWidget* widget = item->widget()) {
                    form->insertRow(row + 1, QString(), widget);
                    delete item; // the wrapper only
                } else if (QLayout* layout = item->layout()) {
                    form->insertRow(row + 1, QString(), layout); // a layout is its own item
                } else {
                    delete item;
                }
                ++row; // past the field's own row
            }
        }
    }
}

void PropertyPanel::commit(const std::string& label, const std::function<void(Project&)>& change) {
    if (!doc_) {
        return;
    }
    try {
        doc_->modify(label, change);
    } catch (const Error& e) {
        QMessageBox::warning(this, tr("Edit"), QString::fromUtf8(e.what()));
    }
}

QFormLayout* PropertyPanel::newForm(const QString& title) {
    statusLabel_ = nullptr; // owned by the page being replaced
    auto* page = new QWidget;
    auto* layout = new QVBoxLayout(page);
    auto* heading = new QLabel(QStringLiteral("<b>%1</b>").arg(title.toHtmlEscaped()));
    layout->addWidget(heading);
    auto* form = new QFormLayout;
    layout->addLayout(form);
    layout->addStretch();
    pageLayout_ = layout;
    setWidget(page); // deletes the previous page
    return form;
}

QString PropertyPanel::jawEntryName(const document::WorkholdingTemplate& set,
                                    const std::string& option) const {
    const document::JawOption* o = set.jawSet ? set.jawSet->findOption(option) : nullptr;
    if (!o || set.jawSet->options.size() < 2) {
        return qs(set.name);
    }
    return tr("%1 (%2)").arg(qs(set.name), qs(o->name));
}

QFormLayout* PropertyPanel::addSection(const QString& key, const QString& title, bool open) {
    const auto it = sectionOpen_.find(key);
    const bool shown = it == sectionOpen_.end() ? open : it->second;
    auto* box = new QWidget;
    box->setObjectName(QStringLiteral("section_") + key);
    auto* layout = new QVBoxLayout(box);
    layout->setContentsMargins(0, 6, 0, 0);
    layout->setSpacing(2);
    auto* head = new QToolButton;
    head->setObjectName(QStringLiteral("sectionHead_") + key);
    head->setText(title);
    head->setToolTip(title);
    head->setCheckable(true);
    head->setChecked(shown);
    head->setArrowType(shown ? Qt::DownArrow : Qt::RightArrow);
    head->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    head->setAutoRaise(true);
    // A long title does not widen the page: the dock can be narrow.
    head->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
    head->setMinimumWidth(0);
    QFont bold = head->font();
    bold.setBold(true);
    head->setFont(bold);
    auto* body = new QWidget;
    auto* form = new QFormLayout(body);
    form->setContentsMargins(12, 0, 0, 0);
    body->setVisible(shown);
    connect(head, &QToolButton::toggled, this, [this, key, head, body](bool on) {
        sectionOpen_[key] = on;
        head->setArrowType(on ? Qt::DownArrow : Qt::RightArrow);
        body->setVisible(on);
    });
    layout->addWidget(head);
    layout->addWidget(body);
    pageLayout_->insertWidget(pageLayout_->count() - 1, box); // above the stretch
    return form;
}

void PropertyPanel::showSelection(const Selection& selection) {
    if (pending_ && !(selection == selection_)) {
        pending_.reset();
        emit pickPrompt(QString());
    }
    if (geometryPick_ &&
        !(selection.kind == Selection::Kind::Operation && selection.id == *geometryPick_)) {
        geometryPick_.reset();
        emit toggleSelectionRequested(false);
        emit pickPrompt(QString());
    }
    selection_ = selection;
    const auto compact = qScopeGuard([this] { compactPage(); });
    if (!doc_) {
        setWidget(new QWidget);
        return;
    }
    const Project& p = doc_->project();
    unit_ = p.displayUnit;
    if (turretOnly_) {
        // The turret window: a lathe setup's turret table, nothing else.
        const Setup* s =
            selection.kind == Selection::Kind::Setup ? p.findSetup(selection.id) : nullptr;
        const machine::Machine* m = s ? document::findMachineFor(p, *s) : nullptr;
        QFormLayout* form = newForm(s ? qs(s->name) : tr("Turret setup"));
        if (m && m->lathe) {
            buildTurret(form, *s, *m);
        } else {
            form->addRow(new QLabel(tr("The setup is not on a lathe with a turret.")));
        }
        return;
    }
    switch (selection.kind) {
    case Selection::Kind::Model:
        if (const auto* m = p.findModel(selection.id)) {
            buildModel(*m);
            return;
        }
        break;
    case Selection::Kind::Tool:
        if (const auto* t = p.tools.findTool(selection.id)) {
            buildTool(*t);
            return;
        }
        break;
    case Selection::Kind::Setup:
        if (const auto* s = p.findSetup(selection.id)) {
            buildSetup(*s);
            return;
        }
        break;
    case Selection::Kind::Operation: {
        const Setup* owner = nullptr;
        if (const auto* op = p.findOperation(selection.id, &owner)) {
            buildOperation(*op, *owner);
            return;
        }
        break;
    }
    default:
        break;
    }
    buildProject();
}

void PropertyPanel::buildProject() {
    const Project& p = doc_->project();
    QFormLayout* form = newForm(tr("Project"));
    auto* name = new QLineEdit(qs(p.name));
    connect(name, &QLineEdit::editingFinished, this, [this, name] {
        commit("Rename project", [&](Project& pr) { pr.name = ss(name->text()); });
    });
    form->addRow(tr("Name"), name);

    auto* unit = new QComboBox;
    unit->addItem(tr("Millimetres"), static_cast<int>(LengthUnit::Millimeter));
    unit->addItem(tr("Inches"), static_cast<int>(LengthUnit::Inch));
    unit->setCurrentIndex(p.displayUnit == LengthUnit::Inch ? 1 : 0);
    connect(unit, &QComboBox::currentIndexChanged, this, [this, unit] {
        commit("Change display unit", [&](Project& pr) {
            pr.displayUnit = static_cast<LengthUnit>(unit->currentData().toInt());
        });
    });
    form->addRow(tr("Display unit"), unit);

    auto* notes = new QPlainTextEdit(qs(p.notes));
    notes->setMaximumHeight(120);
    auto* saveNotes = new QPushButton(tr("Save notes"));
    connect(saveNotes, &QPushButton::clicked, this, [this, notes] {
        commit("Edit notes", [&](Project& pr) { pr.notes = ss(notes->toPlainText()); });
    });
    form->addRow(tr("Notes"), notes);
    form->addRow(QString(), saveNotes);

    const auto issues = p.validate();
    auto* status = new QLabel(
        issues.empty() ? tr("No problems found.")
                       : tr("%n problem(s):\n", nullptr, static_cast<int>(issues.size())) +
                             qs(issues.front()));
    status->setWordWrap(true);
    status->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    form->addRow(tr("Check"), status);
}

void PropertyPanel::buildModel(const document::ModelEntry& model) {
    const Uuid id = model.id;
    QFormLayout* form = newForm(tr("Model"));
    auto* name = new QLineEdit(qs(model.name));
    connect(name, &QLineEdit::editingFinished, this, [this, id, name] {
        commit("Rename model", [&](Project& pr) {
            for (auto& m : pr.models) {
                if (m.id == id) {
                    m.name = ss(name->text());
                }
            }
        });
    });
    form->addRow(tr("Name"), name);
    auto* source = new QLabel(qs(model.sourcePath));
    source->setWordWrap(true);
    source->setTextInteractionFlags(Qt::TextSelectableByMouse);
    form->addRow(tr("Imported from"), source);

    const char* axes[] = {"X", "Y", "Z"};
    for (int a = 0; a < 3; ++a) {
        auto* spin = new LengthSpinBox(model.placement.origin[a], unit_);
        connect(spin, &LengthSpinBox::mmChanged, this, [this, id, a](double v) {
            commit("Move model", [&](Project& pr) {
                for (auto& m : pr.models) {
                    if (m.id == id) {
                        m.placement.origin[a] = v;
                    }
                }
            });
        });
        form->addRow(tr("Placement %1").arg(QLatin1String(axes[a])), spin);
    }
    // Quarter turns about the model's centre (lining up a vendor's vise or clamp model).
    auto* turns = new QHBoxLayout;
    for (int a = 0; a < 3; ++a) {
        auto* turn = new QPushButton(tr("%1 +90°").arg(QLatin1String(axes[a])));
        turn->setToolTip(tr("Turns the model a quarter turn about the project %1 axis, around its "
                            "centre")
                             .arg(QLatin1String(axes[a])));
        connect(turn, &QPushButton::clicked, this, [this, id, a] {
            const auto bounds = doc_->modelBounds(id);
            if (!bounds) {
                return;
            }
            const Vec3 centre = bounds->center();
            const Mat3 q = Eigen::AngleAxisd(kPi / 2.0, Vec3::Unit(a)).toRotationMatrix();
            commit("Turn model", [&](Project& pr) {
                for (auto& m : pr.models) {
                    if (m.id == id) {
                        m.placement.origin = centre + q * (m.placement.origin - centre);
                        m.placement.xAxis = q * m.placement.xAxis;
                        m.placement.zAxis = q * m.placement.zAxis;
                    }
                }
            });
        });
        turns->addWidget(turn);
    }
    form->addRow(tr("Turn"), turns);

    QStringList fixtureOf;
    for (const auto& s : doc_->project().setups) {
        if (std::find(s.fixtures.begin(), s.fixtures.end(), id) != s.fixtures.end()) {
            fixtureOf << qs(s.name);
        }
    }
    if (!fixtureOf.isEmpty()) {
        form->addRow(tr("Fixture in"), new QLabel(fixtureOf.join(QStringLiteral(", "))));
    }

    if (const cad::Model* cadModel = doc_->model(id)) {
        const cad::ModelInfo info = cadModel->info();
        const Vec3 size = info.bounds.size();
        form->addRow(tr("Size"), new QLabel(QStringLiteral("%1 × %2 × %3")
                                                .arg(formatLength(size.x(), unit_, false),
                                                     formatLength(size.y(), unit_, false),
                                                     formatLength(size.z(), unit_))));
        form->addRow(tr("Volume"), new QLabel(formatVolume(info.volume, unit_)));
        form->addRow(tr("Faces"), new QLabel(QString::number(info.faces)));
        const auto holes = cad::findHoles(*cadModel);
        QStringList lines;
        for (const auto& h : holes) {
            lines << QStringLiteral("Ø%1 × %2")
                         .arg(formatLength(h.diameter, unit_, false), formatLength(h.depth, unit_));
        }
        auto* holesLabel = new QLabel(holes.empty() ? tr("none") : lines.join(QLatin1Char('\n')));
        form->addRow(tr("Holes (Z)"), holesLabel);
    } else {
        form->addRow(QString(), new QLabel(tr("Geometry not loaded.")));
    }
}

void PropertyPanel::buildTool(const tooling::Tool& tool) {
    QFormLayout* form = newForm(tr("Tool T%1").arg(tool.number));
    const QString type = trToolType(tool.type);
    form->addRow(tr("Name"), new QLabel(qs(tool.name)));
    form->addRow(tr("Type"), new QLabel(type));
    if (tool.type == tooling::ToolType::BarPuller) {
        form->addRow(tr("Largest bar"), new QLabel(formatLength(tool.mill.tipDiameter, tool.unit)));
        form->addRow(tr("Grip"), new QLabel(formatLength(tool.mill.fluteLength, tool.unit)));
    } else if (!tooling::isTurningTool(tool.type)) {
        form->addRow(tr("Diameter"), new QLabel(formatLength(tool.mill.diameter, tool.unit)));
        form->addRow(tr("Flute length"),
                     new QLabel(formatLength(tool.mill.fluteLength, tool.unit)));
        form->addRow(tr("Flutes"), new QLabel(QString::number(tool.mill.flutes)));
    }
    form->addRow(tr("Presets"), new QLabel(QString::number(tool.presets.size())));
    const auto issues = tool.validate();
    if (!issues.empty()) {
        auto* label = new QLabel(qs(issues.front()));
        label->setStyleSheet(QStringLiteral("color: #c0392b;"));
        label->setWordWrap(true);
        form->addRow(tr("Problem"), label);
    }
    auto* edit = new QPushButton(tr("Edit project tools..."));
    connect(edit, &QPushButton::clicked, this, &PropertyPanel::editProjectToolsRequested);
    form->addRow(QString(), edit);
}

void PropertyPanel::buildSetup(const Setup& setup) {
    const Project& p = doc_->project();
    const Uuid id = setup.id;
    auto edit = [this, id](const std::string& label, std::function<void(Setup&)> change) {
        commit(label, [&](Project& pr) {
            if (Setup* s = pr.findSetup(id)) {
                change(*s);
            }
        });
    };

    QFormLayout* form = newForm(tr("Setup"));
    auto* name = new QLineEdit(qs(setup.name));
    connect(name, &QLineEdit::editingFinished, this,
            [edit, name] { edit("Rename setup", [&](Setup& s) { s.name = ss(name->text()); }); });
    form->addRow(tr("Name"), name);

    auto* model = new QComboBox;
    model->addItem(tr("(none)"));
    for (const auto& m : p.models) {
        model->addItem(qs(m.name), qs(m.id.toString()));
    }
    model->setCurrentIndex(
        setup.modelId ? std::max(0, model->findData(qs(setup.modelId->toString()))) : 0);
    connect(model, &QComboBox::currentIndexChanged, this, [edit, model] {
        const auto mid = Uuid::parse(ss(model->currentData().toString()));
        edit("Change setup model", [&](Setup& s) { s.modelId = mid; });
    });
    form->addRow(tr("Model"), model);

    // Work offsets as the setup's post writes them, as many as it declares and the machine has.
    addWorkOffsetRows(form, p, setup, setup.workOffset, false, [edit](int n) {
        edit("Change work offset", [&](Setup& s) { s.workOffset = n; });
    });

    // Machine: the built-ins, the project's own machines, and the user's machine library
    // (choosing one of those copies it into the project).
    enum Source { BuiltIn, InProject, InLibrary, Manage, Missing };
    constexpr int kSourceRole = Qt::UserRole + 1;
    auto* machineBox = new QComboBox;
    machineBox->setObjectName(QStringLiteral("setupMachine"));
    const document::Project& project = doc_->project();
    const machine::Machine* current = document::findMachineFor(project, setup);
    std::vector<machine::Machine> library;
    // The library's machine of the same id as the project's copy the setup uses, when it is
    // newer: a higher revision, or the same one with other contents (a file changed outside the
    // library, which does not count). A copy of a higher revision than the library's is left.
    std::optional<machine::Machine> newer;
    for (auto& entry : MachineLibrary::load()) {
        if (const machine::Machine* own = project.findMachine(entry.machine.id)) {
            if (own == current && entry.machine.revision >= own->revision &&
                *own != entry.machine) {
                newer = std::move(entry.machine);
            }
        } else {
            library.push_back(std::move(entry.machine));
        }
    }
    // Where a machine comes from is the heading over its group, so its name comes first.
    auto addMachine = [&](const machine::Machine& m, Source source, const QString& from) {
        machineBox->addItem(qs(m.name), qs(m.id));
        machineBox->setItemData(machineBox->count() - 1, static_cast<int>(source), kSourceRole);
        machineBox->setItemData(
            machineBox->count() - 1,
            QStringLiteral("%1\n%2 (%3, %4)")
                .arg(qs(m.name), from, qs(m.id), qs(std::string(machine::toString(m.kinematics)))),
            Qt::ToolTipRole);
    };
    bool builtIns = false;
    for (const machine::Machine* m : machine::builtinMachines()) {
        if (!project.findMachine(m->id)) {
            if (!std::exchange(builtIns, true)) {
                addListHeading(machineBox, tr("Built in"));
            }
            addMachine(*m, BuiltIn, tr("Built in"));
        }
    }
    if (!project.machines.empty()) {
        addListHeading(machineBox, tr("In this project"));
        for (const auto& m : project.machines) {
            addMachine(m, InProject, tr("In this project"));
        }
    }
    if (!library.empty()) {
        addListHeading(machineBox, tr("In your library"));
        for (const auto& m : library) {
            addMachine(m, InLibrary, tr("In your library: copied into the project when chosen"));
        }
    }
    machineBox->insertSeparator(machineBox->count());
    machineBox->addItem(tr("Manage machine library..."));
    machineBox->setItemData(machineBox->count() - 1, static_cast<int>(Manage), kSourceRole);
    if (!current) {
        // What the setup names shows as it is, until another is chosen: nothing stands in for it.
        machineBox->insertItem(0, tr("Not found: %1").arg(qs(setup.machine)), qs(setup.machine));
        machineBox->setItemData(0, static_cast<int>(Missing), kSourceRole);
    }
    int mi = 0;
    for (int i = 0; current && i < machineBox->count(); ++i) {
        if (machineBox->itemData(i).toString() == qs(current->id) &&
            machineBox->itemData(i, kSourceRole).toInt() != InLibrary) {
            mi = i;
        }
    }
    machineBox->setCurrentIndex(mi);
    connect(machineBox, &QComboBox::currentIndexChanged, this,
            [this, edit, machineBox, mi, library, kSourceRole, setupId = setup.id](int i) {
                const auto source =
                    static_cast<Source>(machineBox->itemData(i, kSourceRole).toInt());
                const std::string machineId = machineBox->itemData(i).toString().toStdString();
                if (source == Missing) {
                    return;
                }
                if (source == Manage) {
                    {
                        const QSignalBlocker block(machineBox);
                        machineBox->setCurrentIndex(mi);
                    }
                    // Once the list is done with its signal: the library rebuilds this page,
                    // the list with it.
                    QTimer::singleShot(0, this, [this] { emit manageMachinesRequested(); });
                    return;
                }
                if (source == InLibrary) {
                    const auto it =
                        std::find_if(library.begin(), library.end(),
                                     [&](const machine::Machine& m) { return m.id == machineId; });
                    if (it == library.end()) {
                        return;
                    }
                    const machine::Machine chosen = *it;
                    commit("Use machine " + chosen.name, [&](document::Project& project) {
                        project.machines.push_back(chosen);
                        if (Setup* s = project.findSetup(setupId)) {
                            s->machine = chosen.id;
                        }
                    });
                    return;
                }
                edit("Change machine", [&](Setup& s) { s.machine = machineId; });
            });
    if (!newer) {
        form->addRow(tr("Machine"), machineBox);
    } else {
        // The project keeps its copy (so it opens anywhere) until the user takes the library's:
        // Update, beside the list.
        const machine::Machine* own = project.findMachine(newer->id);
        const int had = own ? own->revision : 0;
        auto* row = new QWidget;
        auto* rowLayout = new QHBoxLayout(row);
        rowLayout->setContentsMargins(0, 0, 0, 0);
        rowLayout->addWidget(machineBox, 1);
        auto* update = new QPushButton(tr("Update"));
        update->setObjectName(QStringLiteral("setupMachineUpdate"));
        update->setToolTip(tr("Replace this project's copy of %1 with the machine library's, "
                              "for every setup of the project that uses it (Undo puts it back)")
                               .arg(qs(newer->name)));
        rowLayout->addWidget(update);
        form->addRow(tr("Machine"), row);
        auto* hint = new QLabel(
            newer->revision > had
                ? tr("The machine library has a newer %1: revision %2, this project's is %3.")
                      .arg(qs(newer->name))
                      .arg(newer->revision)
                      .arg(had)
                : tr("The machine library's %1 differs from this project's copy (both revision "
                     "%2): its file was changed outside the library.")
                      .arg(qs(newer->name))
                      .arg(had));
        hint->setObjectName(QStringLiteral("setupMachineNewer"));
        hint->setWordWrap(true);
        hint->setStyleSheet(QStringLiteral("color: #c08000"));
        form->addRow(QString(), hint);
        connect(update, &QPushButton::clicked, this, [this, chosen = *newer] {
            commit("Update machine " + chosen.name + " from the library",
                   [&](document::Project& project) {
                       for (machine::Machine& m : project.machines) {
                           if (m.id == chosen.id) {
                               m = chosen;
                           }
                       }
                   });
        });
    }
    if (!current) {
        auto* warn = new QLabel(qs(document::machineProblem(project, setup)));
        warn->setWordWrap(true);
        warn->setStyleSheet(QStringLiteral("color: #c0392b;"));
        form->addRow(QString(), warn);
    } else if (const machine::Machine& m = *current; m.isLathe()) {
        // What operations here can ask of the turret: it follows the machine chosen above.
        auto* turret = new QLabel(
            (m.liveTooling
                 ? tr("Live tooling up to %1 rpm, C axis").arg(m.liveMaxSpindle * 60.0, 0, 'f', 0)
                 : tr("No live tooling")) +
            QStringLiteral(", ") + (m.hasYAxis() ? tr("Y axis") : tr("no Y axis")));
        turret->setObjectName(QStringLiteral("setupTurret"));
        turret->setWordWrap(true);
        turret->setToolTip(tr("From the machine (Machine Library). A hole operation chooses its "
                              "Tool drive: a live tool needs live tooling, and a hole across the "
                              "part that does not cross its axis needs a Y axis."));
        form->addRow(tr("Turret"), turret);
    }
    if (current && !current->isLathe()) {
        // Where the work sits on the machine (DL-0037): solve, the post and the simulation use it.
        auto* place = new QPushButton(tr("Place on machine..."));
        place->setToolTip(tr("Snap the work to a face of the machine (a zero-point plate, a "
                             "rotary table, a faceplate) in a view of the machine"));
        connect(place, &QPushButton::clicked, this,
                [this, id] { emit placeOnMachineRequested(id); });
        form->addRow(tr("Placement"), place);
        if (const std::string problem = document::placementProblem(setup, *current);
            !problem.empty()) {
            auto* warn = new QLabel(qs(problem));
            warn->setWordWrap(true);
            warn->setStyleSheet(QStringLiteral("color: #c0392b;"));
            form->addRow(QString(), warn);
        }
    }
    // The usual things first, open: where the work is, what it is cut from, what holds it. The
    // rest folds away.
    buildWcs(addSection(QStringLiteral("wcs"), tr("WCS"), true), setup);
    buildStock(addSection(QStringLiteral("stock"), tr("Stock"), true), setup);
    buildWorkholding(addSection(QStringLiteral("workholding"), tr("Workholding"), true), setup);
    if (current && current->isLathe()) {
        buildLatheSupports(
            addSection(QStringLiteral("supports"), tr("Tailstock and supports"), false), setup);
        if (current->lathe) {
            buildTurret(addSection(QStringLiteral("turret"), tr("Turret: tools and blocks"), false),
                        setup, *current);
        }
    }

    // Fixtures: models that hold the part. Milling toolpaths keep the tool and its holder clear
    // of them.
    form = addSection(QStringLiteral("fixtures"),
                      setup.fixtures.empty()
                          ? tr("Fixtures")
                          : tr("Fixtures (%1)").arg(static_cast<int>(setup.fixtures.size())),
                      false);
    for (const auto& m : p.models) {
        if (setup.modelId == m.id) {
            continue;
        }
        auto* box = new QCheckBox(qs(m.name));
        box->setChecked(std::find(setup.fixtures.begin(), setup.fixtures.end(), m.id) !=
                        setup.fixtures.end());
        box->setToolTip(tr("Toolpaths of this setup keep the tool and its holder clear of it"));
        const Uuid mid = m.id;
        connect(box, &QCheckBox::toggled, this, [edit, mid](bool on) {
            edit(on ? "Add fixture" : "Remove fixture", [&](Setup& s) {
                std::erase(s.fixtures, mid);
                if (on) {
                    s.fixtures.push_back(mid);
                }
            });
        });
        form->addRow(QString(), box);
    }
    auto* fixtureClearance = new LengthSpinBox(setup.fixtureClearance, unit_, 0.0, 1000.0);
    fixtureClearance->setToolTip(
        tr("How close the tool, its shank and its holder may come to the fixtures"));
    connect(fixtureClearance, &LengthSpinBox::mmChanged, this, [edit](double v) {
        edit("Change fixture clearance", [&](Setup& s) { s.fixtureClearance = v; });
    });
    form->addRow(tr("Clearance"), fixtureClearance);

    if (current) {
        buildPost(addSection(QStringLiteral("post"), tr("Post"), false), setup, *current);
    }
}

void PropertyPanel::buildStock(QFormLayout* form, const Setup& setup) {
    using document::StockSize;
    const Project& p = doc_->project();
    const Uuid id = setup.id;
    const document::Stock& stock = setup.stock;
    const char* names[] = {"X", "Y", "Z"};
    // Stock of an earlier file placed in the project frame becomes fixed stock where it is on
    // its first edit, so the rows show (and edit) that.
    const bool legacy = stock.boxCorners || stock.cylinderPlacement;
    const document::Stock shown = legacy ? fixedStock(id, stock.kind).value_or(stock) : stock;
    auto edit = [this, id, legacy](const std::string& label,
                                   std::function<void(document::Stock&)> change) {
        const Setup* now = doc_->project().findSetup(id);
        const auto settled = legacy && now ? fixedStock(id, now->stock.kind) : std::nullopt;
        commit(label, [&](Project& pr) {
            if (Setup* s = pr.findSetup(id)) {
                if (settled) {
                    s->stock = *settled;
                }
                change(s->stock);
            }
        });
    };
    // A new shape or size: fixed stock starts as the stock now, where it is (the bore stays
    // as typed); stock from the part starts centred.
    auto reshape = [this, id](const std::string& label, StockKind kind, StockSize size) {
        const auto seeded = size == StockSize::Fixed && kind != StockKind::FromSetup
                                ? fixedStock(id, kind)
                                : std::nullopt;
        commit(label, [&](Project& pr) {
            if (Setup* s = pr.findSetup(id)) {
                const bool sizeChanged = s->stock.size != size;
                if (seeded) {
                    const double bore = s->stock.innerDiameter;
                    s->stock = *seeded;
                    s->stock.innerDiameter = bore;
                } else if (sizeChanged) {
                    s->stock.shift = Vec3::Zero();
                }
                s->stock.kind = kind;
                s->stock.size = size;
                s->stock.boxCorners.reset();
                s->stock.cylinderPlacement.reset();
            }
        });
    };

    form->addRow(new QLabel(QStringLiteral("<b>%1</b>").arg(tr("Stock settings"))));
    auto* shape = new QComboBox;
    shape->addItem(tr("Box"), static_cast<int>(StockKind::Box));
    shape->addItem(tr("Cylinder"), static_cast<int>(StockKind::Cylinder));
    shape->addItem(tr("Tube"), static_cast<int>(StockKind::Tube));
    shape->addItem(tr("From earlier setup"), static_cast<int>(StockKind::FromSetup));
    shape->setCurrentIndex(shape->findData(static_cast<int>(stock.kind)));
    connect(shape, &QComboBox::currentIndexChanged, this, [reshape, shape, size = stock.size] {
        reshape("Change stock shape", static_cast<StockKind>(shape->currentData().toInt()), size);
    });
    form->addRow(tr("Shape"), shape);

    if (stock.kind == StockKind::FromSetup) {
        auto* source = new QComboBox;
        source->addItem(tr("(choose)"));
        for (const auto& other : p.setups) {
            if (other.id == id) {
                break; // only earlier setups
            }
            source->addItem(qs(other.name), qs(other.id.toString()));
        }
        source->setCurrentIndex(
            stock.sourceSetup ? std::max(0, source->findData(qs(stock.sourceSetup->toString())))
                              : 0);
        connect(source, &QComboBox::currentIndexChanged, this, [edit, source] {
            const auto sid = Uuid::parse(ss(source->currentData().toString()));
            edit("Change stock source", [&](document::Stock& st) { st.sourceSetup = sid; });
        });
        form->addRow(tr("Source setup"), source);
    } else {
        const bool round = stock.kind != StockKind::Box;
        auto* size = new QComboBox;
        size->addItem(tr("Fixed"), static_cast<int>(StockSize::Fixed));
        if (setup.modelId || stock.size != StockSize::Fixed) {
            size->addItem(tr("Offset from part"), static_cast<int>(StockSize::OffsetFromPart));
            size->addItem(tr("No additional stock"), static_cast<int>(StockSize::Part));
        }
        size->setCurrentIndex(size->findData(static_cast<int>(stock.size)));
        size->setToolTip(tr("Fixed: typed dimensions. Offset from part: the part's extent plus "
                            "the offsets. No additional stock: the part's extent"));
        connect(size, &QComboBox::currentIndexChanged, this, [reshape, size, kind = stock.kind] {
            reshape("Change stock size", kind, static_cast<StockSize>(size->currentData().toInt()));
        });
        form->addRow(tr("Size"), size);

        if (round) {
            auto* axis = new QComboBox;
            for (int a = 0; a < 3; ++a) {
                axis->addItem(tr("Along WCS %1").arg(QLatin1String(names[a])), a);
            }
            axis->setCurrentIndex(std::clamp(shown.axis, 0, 2));
            connect(axis, &QComboBox::currentIndexChanged, this, [edit, axis] {
                const int a = axis->currentData().toInt();
                edit("Change stock axis", [&](document::Stock& st) { st.axis = a; });
            });
            form->addRow(tr("Axis"), axis);
        }

        auto stockLength = [&](const QString& label, double value, double min, auto setter) {
            auto* spin = new LengthSpinBox(value, unit_, min);
            connect(spin, &LengthSpinBox::mmChanged, this, [edit, setter](double v) {
                edit("Edit stock", [&](document::Stock& st) { setter(st, v); });
            });
            form->addRow(label, spin);
            return spin;
        };
        if (stock.size == StockSize::Fixed && !round) {
            const char* sizeNames[] = {QT_TR_NOOP("Width (X)"), QT_TR_NOOP("Length (Y)"),
                                       QT_TR_NOOP("Height (Z)")};
            for (int a = 0; a < 3; ++a) {
                stockLength(tr(sizeNames[a]), shown.boxSize[a], 0.0,
                            [a](document::Stock& st, double v) { st.boxSize[a] = v; })
                    ->setToolTip(tr("The stock's size along WCS %1").arg(QLatin1String(names[a])));
            }
        } else if (stock.size == StockSize::Fixed) {
            stockLength(tr("Diameter"), shown.diameter, 0.0,
                        [](document::Stock& st, double v) { st.diameter = v; });
            stockLength(tr("Length"), shown.length, 0.0, [](document::Stock& st, double v) {
                st.length = v;
            })->setToolTip(tr("The stock's length along its axis"));
        }
        if (stock.kind == StockKind::Tube) {
            stockLength(tr("Inner diameter"), shown.innerDiameter, 0.0,
                        [](document::Stock& st, double v) { st.innerDiameter = v; })
                ->setToolTip(tr("The diameter of the tube's bore"));
        }
        if (stock.size == StockSize::OffsetFromPart) {
            // Across the stock, then at its ends: along WCS Z (and for a box) top and bottom.
            const int along = round ? std::clamp(shown.axis, 0, 2) : 2;
            const QString axisName = QLatin1String(names[along]);
            stockLength(tr("Side offset"), stock.offsetSide, 0.0,
                        [](document::Stock& st, double v) { st.offsetSide = v; })
                ->setToolTip(round ? tr("Added to the part's radius about the stock's axis")
                                   : tr("Added to the part in WCS X and Y on every side"));
            stockLength(along == 2 ? tr("Top offset") : tr("End offset +%1").arg(axisName),
                        stock.offsetTop, 0.0,
                        [](document::Stock& st, double v) { st.offsetTop = v; })
                ->setToolTip(tr("Added to the part at its WCS +%1 end").arg(axisName));
            stockLength(along == 2 ? tr("Bottom offset") : tr("End offset −%1").arg(axisName),
                        stock.offsetBottom, 0.0,
                        [](document::Stock& st, double v) { st.offsetBottom = v; })
                ->setToolTip(tr("Added to the part at its WCS −%1 end").arg(axisName));
            stockLength(tr("Round up to"), stock.roundTo, 0.0,
                        [](document::Stock& st, double v) { st.roundTo = v; })
                ->setToolTip(round ? tr("Rounds the stock's diameter and length up to a multiple "
                                        "of this value; 0 = no rounding")
                                   : tr("Rounds the stock's sizes up to a multiple of this value "
                                        "(for example 1.11 to 1.25 with 0.25); the extra goes to "
                                        "both sides in X and Y and to the top. 0 = no rounding"));
        }
        for (int a = 0; a < 3; ++a) {
            stockLength(tr("Base %1").arg(QLatin1String(names[a])), shown.shift[a], -100000.0,
                        [a](document::Stock& st, double v) { st.shift[a] = v; })
                ->setToolTip(tr("Moves the stock along WCS %1 from centred on the part (0 = "
                                "centred)")
                                 .arg(QLatin1String(names[a])));
        }
    }

    if (const auto resolved = doc_->stockShape(id)) {
        const Vec3 extent = resolved->kind == document::StockShape::Kind::Box
                                ? resolved->box.size()
                                : resolved->bounds().size();
        form->addRow(tr("Stock size"), new QLabel(QStringLiteral("%1 × %2 × %3")
                                                      .arg(formatLength(extent.x(), unit_, false),
                                                           formatLength(extent.y(), unit_, false),
                                                           formatLength(extent.z(), unit_))));
    } else {
        auto* warn = new QLabel(stock.kind == StockKind::Tube
                                    ? tr("Stock cannot be resolved with these settings: the "
                                         "tube's inner diameter must be above 0 and smaller "
                                         "than its outer diameter.")
                                    : tr("Stock cannot be resolved with these settings."));
        warn->setWordWrap(true);
        warn->setStyleSheet(QStringLiteral("color: #c0392b;"));
        form->addRow(QString(), warn);
    }
}

std::optional<document::Stock> PropertyPanel::fixedStock(const Uuid& setupId,
                                                         document::StockKind kind) const {
    const Setup* s = doc_ ? doc_->project().findSetup(setupId) : nullptr;
    const auto shape = s ? doc_->stockShape(setupId) : std::nullopt;
    const auto sized = s ? sizedBox(setupId) : std::nullopt;
    if (!shape || !sized) {
        return std::nullopt;
    }
    document::Stock out = s->stock;
    out.kind = kind;
    out.size = document::StockSize::Fixed;
    out.boxCorners.reset();
    out.cylinderPlacement.reset();
    out.shift = sized->second; // the middle of its box, from the part's
    if (kind == document::StockKind::Box) {
        out.boxSize = sized->first;
        return out;
    }
    const document::Frame f = s->wcs.normalized();
    Mat3 axes;
    axes << f.xAxis, f.yAxis(), f.zAxis;
    if (shape->kind == document::StockShape::Kind::Cylinder) {
        // The same bar on the nearest WCS axis.
        (axes.transpose() * shape->axis).cwiseAbs().maxCoeff(&out.axis);
        out.diameter = shape->diameter;
        out.length = shape->length;
    } else {
        // Round stock around the box: its corners on the circle.
        const int k = std::clamp(out.axis, 0, 2);
        out.diameter = std::hypot(sized->first[(k + 1) % 3], sized->first[(k + 2) % 3]);
        out.length = sized->first[k];
    }
    return out;
}

void PropertyPanel::buildLatheSupports(QFormLayout* form, const document::Setup& setup) {
    const Uuid id = setup.id;
    auto edit = [this, id](const char* what, auto change) {
        doc_->modify(what, [&](document::Project& p) {
            if (document::Setup* s = p.findSetup(id)) {
                change(*s);
            }
        });
    };
    const auto spin = [&](double value, double lo, double hi, const QString& label,
                          const QString& tip, auto set) {
        auto* box = new LengthSpinBox(value, unit_, lo, hi);
        box->setToolTip(tip);
        connect(box, &LengthSpinBox::mmChanged, this, [edit, set](double v) {
            edit("Change lathe support", [&](document::Setup& s) { set(s, v); });
        });
        form->addRow(label, box);
    };

    auto* tailstock = new QCheckBox(tr("Tailstock"));
    tailstock->setChecked(setup.tailstock.has_value());
    tailstock->setToolTip(tr("A centre on the spindle axis, kept clear of like a fixture. It stays "
                             "in for the whole setup: to face and centre drill first, do that in "
                             "a setup before it, and continue that setup's stock here."));
    connect(tailstock, &QCheckBox::toggled, this, [edit](bool on) {
        edit("Tailstock", [&](document::Setup& s) {
            s.tailstock = on ? std::make_optional(document::LatheTailstock{}) : std::nullopt;
        });
    });
    form->addRow(QString(), tailstock);
    if (const auto& t = setup.tailstock) {
        // The centre fitted: a model from the library, or a plain cone of these sizes.
        // Only what fits the machine's quill is offered; one already fitted stays, said.
        const machine::Machine* onMachine = document::findMachineFor(doc_->project(), setup);
        const auto fits = [&](const document::LatheFitting& f) {
            return !onMachine || !document::mountMismatch(f, *onMachine);
        };
        std::vector<document::FittingPackage> centres;
        int notFitting = 0;
        for (document::FittingPackage& pkg : LatheFittingLibrary::load()) {
            const document::LatheFitting* own = doc_->project().findFitting(pkg.fitting.id);
            if (pkg.fitting.kind == document::LatheFitting::Kind::Centre &&
                (!own || *own != pkg.fitting)) {
                if (fits(pkg.fitting)) {
                    centres.push_back(std::move(pkg));
                } else {
                    ++notFitting;
                }
            }
        }
        auto* centreBox = new QComboBox;
        centreBox->setObjectName(QStringLiteral("setupTailstockCentre"));
        centreBox->setToolTip(
            tr("The centre in the quill, such as a live centre, shown and checked "
               "as its model. Its tip goes at the point above, and the quill "
               "starts where it seats."));
        centreBox->addItem(tr("A plain cone"));
        for (const document::LatheFitting& f : doc_->project().fittings) {
            const bool fitted = t->centre && *t->centre == f.id;
            if (f.kind == document::LatheFitting::Kind::Centre && (fits(f) || fitted)) {
                centreBox->addItem(fits(f) ? qs(f.name)
                                           : tr("%1 (does not fit this machine)").arg(qs(f.name)),
                                   qs(f.id.toString()));
                if (fitted) {
                    centreBox->setCurrentIndex(centreBox->count() - 1);
                }
            } else if (f.kind == document::LatheFitting::Kind::Centre) {
                ++notFitting;
            }
        }
        if (!centres.empty()) {
            addListHeading(centreBox, tr("In your library"));
        }
        for (const document::FittingPackage& pkg : centres) {
            centreBox->addItem(qs(pkg.fitting.name), qs(pkg.fitting.id.toString()));
            centreBox->setItemData(centreBox->count() - 1,
                                   tr("%1\nIn your library: copied into the project when chosen")
                                       .arg(qs(pkg.fitting.name)),
                                   Qt::ToolTipRole);
        }
        if (notFitting > 0 && onMachine && onMachine->lathe && onMachine->lathe->tailstock) {
            centreBox->insertSeparator(centreBox->count());
            centreBox->addItem(tr("(%n more that do not fit)", nullptr, notFitting));
            centreBox->setItemData(
                centreBox->count() - 1,
                tr("%n more centres that do not fit a quill taking %1", nullptr, notFitting)
                    .arg(qs(onMachine->lathe->tailstock->quillInterface)),
                Qt::ToolTipRole);
            centreBox->setItemData(centreBox->count() - 1, 0, Qt::UserRole - 1); // disabled
        }
        centreBox->insertSeparator(centreBox->count());
        centreBox->addItem(tr("Manage centres..."), QStringLiteral("manage"));
        connect(centreBox, &QComboBox::activated, this, [this, edit, centreBox, centres](int i) {
            const QString itemText = centreBox->itemData(i).toString();
            if (itemText == QLatin1String("manage")) {
                // Once the list is done with its signal: the library rebuilds this page.
                QTimer::singleShot(0, this, [this] { emit manageFittingsRequested(); });
                return;
            }
            const auto chosen = Uuid::parse(ss(itemText));
            if (chosen) {
                const auto it = std::find_if(
                    centres.begin(), centres.end(),
                    [&](const document::FittingPackage& pkg) { return pkg.fitting.id == *chosen; });
                if (it != centres.end()) {
                    try {
                        doc_->useFitting(*it);
                    } catch (const Error& e) {
                        QMessageBox::warning(this, tr("Tailstock"), QString::fromUtf8(e.what()));
                        return;
                    }
                }
            }
            edit("Change the tailstock's centre", [&](document::Setup& st) {
                if (st.tailstock) {
                    st.tailstock->centre = chosen;
                }
            });
        });
        form->addRow(tr("Centre fitted"), centreBox);
        spin(t->tipZ, -100000.0, 100000.0, tr("Centre's point at Z"),
             tr("In the setup's WCS: where the point sits in the part's centre hole"),
             [](document::Setup& s, double v) { s.tailstock->tipZ = v; });
        if (!t->centre) {
            spin(t->centreDiameter, 0.01, 10000.0, tr("Centre diameter"), QString(),
                 [](document::Setup& s, double v) { s.tailstock->centreDiameter = v; });
        }
        spin(t->quillDiameter, 0.01, 10000.0, tr("Quill diameter"), QString(),
             [](document::Setup& s, double v) { s.tailstock->quillDiameter = v; });
        spin(t->quillLength, 0.01, 10000.0, tr("Quill length"),
             onMachine && onMachine->lathe && onMachine->lathe->tailstock &&
                     onMachine->lathe->tailstock->quillTravel > 0.0
                 ? tr("How far the quill is out of the tailstock. Machine '%1''s quill goes "
                      "out %2 mm at most.")
                       .arg(qs(onMachine->name))
                       .arg(onMachine->lathe->tailstock->quillTravel)
                 : tr("How far the quill is out of the tailstock"),
             [](document::Setup& s, double v) { s.tailstock->quillLength = v; });
        // Out further than the machine's quill goes: said, never shortened.
        if (const auto why =
                onMachine ? document::quillBeyondTravel(setup, *onMachine) : std::nullopt) {
            auto* warn = new QLabel(qs(*why));
            warn->setObjectName(QStringLiteral("setupQuillBeyondTravel"));
            warn->setWordWrap(true);
            warn->setStyleSheet(QStringLiteral("color: #c0392b;"));
            form->addRow(QString(), warn);
        }
    }
    auto* steady = new QCheckBox(tr("Steady rest"));
    steady->setChecked(setup.steadyRest.has_value());
    steady->setToolTip(tr("Jaws around the work at one height, kept clear of like a fixture: "
                          "passes stop in front of it. What lies behind it is for another setup "
                          "or Z range."));
    connect(steady, &QCheckBox::toggled, this, [edit](bool on) {
        edit("Steady rest", [&](document::Setup& s) {
            s.steadyRest = on ? std::make_optional(document::LatheSteadyRest{}) : std::nullopt;
        });
    });
    form->addRow(QString(), steady);
    if (const auto& r = setup.steadyRest) {
        spin(r->z, -100000.0, 100000.0, tr("At Z"),
             tr("The middle of its jaws, in the setup's WCS"),
             [](document::Setup& s, double v) { s.steadyRest->z = v; });
        spin(r->width, 0.01, 10000.0, tr("Width"), tr("Of its jaws, along Z"),
             [](document::Setup& s, double v) { s.steadyRest->width = v; });
        auto* grip = new LengthSpinBox(r->gripDiameter, unit_, 0.0, 10000.0);
        grip->setSpecialValueText(tr("The stock's"));
        grip->setToolTip(tr("The diameter its jaws close on"));
        connect(grip, &LengthSpinBox::mmChanged, this, [edit](double v) {
            edit("Change lathe support",
                 [&](document::Setup& s) { s.steadyRest->gripDiameter = v; });
        });
        form->addRow(tr("Grips"), grip);
        spin(r->outerDiameter, 0.01, 10000.0, tr("Outer diameter"), QString(),
             [](document::Setup& s, double v) { s.steadyRest->outerDiameter = v; });
    }
}

QWidget* PropertyPanel::setUpRow(const QString& name, const QString& objectName,
                                 const Uuid& setupId) {
    auto* row = new QWidget;
    auto* layout = new QHBoxLayout(row);
    layout->setContentsMargins(0, 0, 0, 0);
    auto* label = new QLabel(name);
    label->setObjectName(objectName);
    label->setWordWrap(true);
    auto* button = new QPushButton(tr("Set up..."));
    button->setObjectName(objectName + QStringLiteral("SetUp"));
    button->setProperty("keepWidth", true); // beside the name, its text's width
    button->setToolTip(tr("Choose what holds the stock and its jaws, from the project or the "
                          "library, and see what the project has copied in"));
    connect(button, &QPushButton::clicked, this,
            [this, setupId] { emit workholdingSetupRequested(setupId); });
    layout->addWidget(label, 1);
    layout->addWidget(button);
    return row;
}

void PropertyPanel::buildWorkholding(QFormLayout* form, const Setup& setup) {
    const Project& p = doc_->project();
    // What holds the stock, its jaws and how it holds it are set in the workholding dialog
    // (WorkholdingSetupDialog); the page names them, and says what is wrong.
    const document::WorkholdingTemplate* t =
        setup.workholding ? p.findWorkholding(setup.workholding->templateId) : nullptr;
    form->addRow(tr("Workholding"), setUpRow(t ? qs(t->name) : tr("(none)"),
                                             QStringLiteral("setupWorkholdingName"), setup.id));
    if (!setup.workholding) {
        return;
    }
    const document::SetupWorkholding& w = *setup.workholding;
    const document::WorkholdingTemplate* set = w.jawSet ? p.findWorkholding(*w.jawSet) : nullptr;
    if (t && t->jawMount) {
        auto* jaws = new QLabel(set ? jawEntryName(*set, w.jawOption) : tr("(none chosen)"));
        jaws->setObjectName(QStringLiteral("setupJawSetName"));
        form->addRow(tr("Jaws"), jaws);
    }
    // Why the workholding cannot hold the stock this way, or what to look out for: said once.
    // Stock beyond its reach is drawn with the jaws as far as they go.
    const auto problem = [form](const QString& text) {
        auto* label = new QLabel(text.isEmpty() ? text : text.left(1).toUpper() + text.mid(1));
        label->setObjectName(QStringLiteral("setupWorkholdingProblem"));
        label->setWordWrap(true);
        label->setStyleSheet(QStringLiteral("color: #c0392b;"));
        form->addRow(QString(), label);
    };
    try {
        if (const auto pose = document::shownWorkholdingPose(*doc_, setup)) {
            if (pose->outOfReach) {
                problem(qs(document::reachMessage(*pose->outOfReach, unit_)) + QLatin1Char('.'));
            } else if (!pose->refusal.empty()) {
                problem(qs(pose->refusal) + QLatin1Char('.'));
            } else if (t && t->jawMount && !set) {
                problem(tr("No jaws chosen for the chuck: Set up... chooses them."));
            }
            if (!pose->warnings.empty()) {
                QStringList lines;
                for (const auto& warning : pose->warnings) {
                    lines << qs(warning);
                }
                auto* warn = new QLabel(lines.join(QLatin1Char('\n')));
                warn->setWordWrap(true);
                warn->setStyleSheet(QStringLiteral("color: #c08000"));
                form->addRow(QString(), warn);
            }
        }
    } catch (const Error& e) {
        problem(QString::fromUtf8(e.what()));
    }
    // The library's copy of the chuck or its jaw set changed since the project took it: the
    // dialog updates it (the project's may hold the stock where the library's would not).
    QStringList newer;
    for (const auto& entry : WorkholdingLibrary::load()) {
        const document::WorkholdingTemplate& lib = entry.package.workholding;
        const bool used = (t && lib.id == t->id) || (w.jawSet && lib.id == *w.jawSet);
        const document::WorkholdingTemplate* own = p.findWorkholding(lib.id);
        if (used && own && *own != lib) {
            newer << qs(lib.name);
        }
    }
    if (!newer.isEmpty()) {
        auto* hint = new QLabel(tr("The library's %1 differs from this project's copy: Set up... "
                                   "can update it.")
                                    .arg(newer.join(tr(" and "))));
        hint->setObjectName(QStringLiteral("setupWorkholdingNewer"));
        hint->setWordWrap(true);
        hint->setStyleSheet(QStringLiteral("color: #c08000"));
        form->addRow(QString(), hint);
    }
    // Soft jaws are bored by an operation of the setup.
    const bool soft = set && set->jawSet && set->jawSet->kind == document::JawSet::Kind::Soft;
    const bool haveOp =
        std::any_of(setup.operations.begin(), setup.operations.end(),
                    [](const document::Operation& o) { return o.type == "turn_soft_jaws"; });
    if (soft && !haveOp) {
        auto* addOp = new QPushButton(tr("Add the boring operation"));
        addOp->setObjectName(QStringLiteral("setupAddSoftJawOp"));
        addOp->setToolTip(tr("An operation at the start of the setup that bores these jaws to "
                             "their bore: choose its tool and cutting data like any other"));
        const Uuid id = setup.id;
        connect(addOp, &QPushButton::clicked, this, [this, id] {
            commit("Add the soft jaw boring", [&](Project& pr) {
                if (Setup* s = pr.findSetup(id)) {
                    document::Operation op;
                    op.name = ss(tr("Bore soft jaws"));
                    op.type = "turn_soft_jaws";
                    op.parameters = Json::object();
                    s->operations.insert(s->operations.begin(), op);
                }
            });
        });
        form->addRow(QString(), addOp);
    }
}

std::optional<std::pair<Vec3, Vec3>> PropertyPanel::sizedBox(const Uuid& setupId) const {
    const Setup* s = doc_ ? doc_->project().findSetup(setupId) : nullptr;
    if (!s) {
        return std::nullopt;
    }
    try {
        const geom3d::Aabb3 box = document::originBox(*doc_, *s, document::OriginFrom::StockBox);
        Vec3 centre = Vec3::Zero();
        if (s->modelId) {
            const document::Frame f = s->wcs.normalized();
            Mat3 axes;
            axes << f.xAxis, f.yAxis(), f.zAxis;
            const auto part = doc_->modelBounds(*s->modelId, axes);
            if (!part || part->isEmpty()) {
                return std::nullopt;
            }
            centre = part->center();
        }
        return std::pair{Vec3(box.size()), Vec3(box.center() - centre)};
    } catch (const Error&) {
        return std::nullopt;
    }
}

void PropertyPanel::buildWcs(QFormLayout* form, const Setup& setup) {
    const Uuid id = setup.id;
    auto edit = [this, id](const std::string& label, std::function<void(Setup&)> change) {
        commit(label, [&](Project& pr) {
            if (Setup* s = pr.findSetup(id)) {
                change(*s);
            }
        });
    };
    auto problemLabel = [form](const QString& text) {
        auto* warn = new QLabel(text);
        warn->setWordWrap(true);
        warn->setStyleSheet(QStringLiteral("color: #c0392b;"));
        form->addRow(QString(), warn);
    };
    // A picked face, edge or vertex: what it is, the pick button and (axes) the flip box.
    auto pickRow = [this, form, id](const QString& label, const QString& picked, WcsTarget target,
                                    const QString& tip) {
        auto* row = new QHBoxLayout;
        auto* pick = new QPushButton(tr("Pick"));
        pick->setCheckable(true);
        pick->setChecked(wcsPick_ && wcsPick_->setup == id && wcsPick_->target == target);
        pick->setToolTip(picked.isEmpty() ? tip : tr("Picked: %1\n%2").arg(picked, tip));
        connect(pick, &QPushButton::clicked, this,
                [this, id, target] { startWcsPick(id, target); });
        // A share of the row: the page's buttons take the width they are given (compactPage).
        row->addWidget(pick, 2);
        row->addStretch(3);
        form->addRow(label, row);
        return row;
    };

    // -- Part orientation: the part's own axes, or Z and X (or Y) from picked faces and edges.
    form->addRow(new QLabel(QStringLiteral("<b>%1</b>").arg(tr("Part orientation"))));
    const bool ownAxes = (setup.wcs.zAxis.normalized() - Vec3::UnitZ()).norm() < 1e-9 &&
                         (setup.wcs.xAxis.normalized() - Vec3::UnitX()).norm() < 1e-9;
    const document::WcsOrientation orientation =
        setup.orientation.value_or(document::WcsOrientation{});
    auto* mode = new QComboBox;
    mode->addItem(tr("The part's own axes"), static_cast<int>(document::OrientationMode::Default));
    mode->addItem(tr("X and Z from picks"), static_cast<int>(document::OrientationMode::XZ));
    mode->addItem(tr("Y and Z from picks"), static_cast<int>(document::OrientationMode::YZ));
    if (!setup.orientation && !ownAxes) {
        mode->addItem(tr("As set (earlier version)"), -1);
        mode->setCurrentIndex(mode->count() - 1);
    } else {
        mode->setCurrentIndex(mode->findData(static_cast<int>(orientation.mode)));
    }
    mode->setToolTip(tr("The WCS axes: the part's as modelled, or Z from a picked face or edge "
                        "and X (or Y) from another, square to Z. Pick on the part or a fixture."));
    connect(mode, &QComboBox::currentIndexChanged, this, [this, edit, mode] {
        const int m = mode->currentData().toInt();
        if (m < 0) {
            return;
        }
        edit("Change part orientation", [&](Setup& s) {
            document::WcsOrientation o;
            o.mode = static_cast<document::OrientationMode>(m);
            try {
                document::setOrientation(s, o, false);
            } catch (const Error& e) {
                emit pickPrompt(QString::fromUtf8(e.what()));
            }
        });
    });
    form->addRow(tr("Orientation"), mode);
    if (setup.orientation && orientation.mode != document::OrientationMode::Default) {
        const bool alongY = orientation.mode == document::OrientationMode::YZ;
        const struct {
            const std::optional<document::AxisPick>& pick;
            WcsTarget target;
            QString label;
        } axes[] = {{orientation.first, WcsTarget::FirstAxis, alongY ? tr("Y from") : tr("X from")},
                    {orientation.z, WcsTarget::ZAxis, tr("Z from")}};
        for (const auto& a : axes) {
            QHBoxLayout* row =
                pickRow(a.label, a.pick ? qs(a.pick->label) : QString(), a.target,
                        tr("Click a flat or round face, or a straight or circular edge, of the "
                           "part or a fixture: a face gives its normal (a round one its axis), an "
                           "edge its direction"));
            auto* flip = new QCheckBox(tr("Flip"));
            flip->setChecked(a.pick && a.pick->flip);
            flip->setEnabled(a.pick.has_value());
            flip->setToolTip(tr("The axis runs the other way"));
            const WcsTarget target = a.target;
            connect(flip, &QCheckBox::toggled, this, [this, edit, target](bool on) {
                edit("Flip WCS axis", [&](Setup& s) {
                    if (!s.orientation) {
                        return;
                    }
                    document::WcsOrientation o = *s.orientation;
                    auto& pick = target == WcsTarget::ZAxis ? o.z : o.first;
                    if (!pick) {
                        return;
                    }
                    pick->flip = on;
                    try {
                        // A pick on the workholding (no model face): it stays where it is.
                        document::setOrientation(s, o, !pick->shape.has_value());
                    } catch (const Error& e) {
                        emit pickPrompt(QString::fromUtf8(e.what()));
                    }
                });
            });
            row->insertWidget(1, flip);
        }
    }

    // -- Origin: a point of the stock's, the part's or the fixtures' box, or a picked point,
    // moved by an offset along the WCS axes; kept there when the stock or the fixtures change.
    form->addRow(new QLabel(QStringLiteral("<b>%1</b>").arg(tr("WCS origin"))));
    auto* from = new QComboBox;
    from->addItem(tr("Stock box"), static_cast<int>(document::OriginFrom::StockBox));
    from->addItem(tr("Part box"), static_cast<int>(document::OriginFrom::PartBox));
    from->addItem(tr("Fixture box"), static_cast<int>(document::OriginFrom::FixtureBox));
    from->addItem(tr("Picked point"), static_cast<int>(document::OriginFrom::Point));
    if (!setup.originFrom) {
        from->addItem(tr("As set (earlier version)"), -1);
        from->setCurrentIndex(from->count() - 1);
    } else {
        from->setCurrentIndex(from->findData(static_cast<int>(setup.originFrom->from)));
    }
    from->setToolTip(tr("What the WCS origin is measured from: a point of the stock's, the "
                        "part's or the fixtures' (fixture models and workholding) box, or a "
                        "picked circle's centre, face's centre or corner"));
    connect(from, &QComboBox::currentIndexChanged, this, [edit, from] {
        const int f = from->currentData().toInt();
        if (f < 0) {
            return;
        }
        edit("Change WCS origin", [&](Setup& s) {
            document::WcsOrigin o = s.originFrom.value_or(document::WcsOrigin{});
            o.from = static_cast<document::OriginFrom>(f);
            s.originFrom = o;
        });
    });
    form->addRow(tr("Origin from"), from);

    if (!setup.originFrom) {
        // Files of earlier versions: the origin as set, in the part's frame, until a reference
        // is chosen.
        const char* names[] = {"X", "Y", "Z"};
        for (int a = 0; a < 3; ++a) {
            auto* spin = new LengthSpinBox(setup.wcs.origin[a], unit_);
            connect(spin, &LengthSpinBox::mmChanged, this, [edit, a](double v) {
                edit("Move WCS origin", [&](Setup& s) { s.wcs.origin[a] = v; });
            });
            form->addRow(tr("Origin %1 (part's frame)").arg(QLatin1String(names[a])), spin);
        }
        return;
    }
    const document::WcsOrigin& origin = *setup.originFrom;
    if (origin.from == document::OriginFrom::Point) {
        pickRow(tr("Point"), qs(origin.label), WcsTarget::Origin,
                tr("Click a circular edge (its centre), a face (its centre; a round face's axis) "
                   "or a corner of the part or a fixture"));
    } else {
        // The box's level along Z and its 3 x 3 position seen from above.
        auto* level = new QComboBox;
        level->addItem(tr("Top"), 2);
        level->addItem(tr("Middle"), 1);
        level->addItem(tr("Bottom"), 0);
        level->setCurrentIndex(level->findData(origin.box[2]));
        connect(level, &QComboBox::currentIndexChanged, this, [edit, level] {
            const int k = level->currentData().toInt();
            edit("Change WCS origin", [&](Setup& s) {
                if (s.originFrom) {
                    s.originFrom->box[2] = k;
                }
            });
        });
        auto* grid = new QGridLayout;
        grid->setSpacing(2);
        const char* arrows[3][3] = {{"↖", "↑", "↗"}, {"←", "•", "→"}, {"↙", "↓", "↘"}};
        const char* yNames[3] = {QT_TR_NOOP("back"), "", QT_TR_NOOP("front")};
        const char* xNames[3] = {QT_TR_NOOP("left"), "", QT_TR_NOOP("right")};
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                auto* b = new QToolButton;
                b->setText(QString::fromUtf8(arrows[row][col]));
                b->setFixedSize(28, 24);
                b->setCheckable(true);
                const int ix = col;     // 0 = WCS X min (left)
                const int iy = 2 - row; // 2 = WCS Y max (back)
                b->setChecked(origin.box[0] == ix && origin.box[1] == iy);
                const QString where =
                    row == 1 && col == 1
                        ? tr("centre")
                        : QStringLiteral("%1 %2").arg(tr(yNames[row]), tr(xNames[col])).trimmed();
                b->setToolTip(tr("The origin at the %1 of the box, seen from above").arg(where));
                connect(b, &QToolButton::clicked, this, [edit, ix, iy] {
                    edit("Change WCS origin", [&](Setup& s) {
                        if (s.originFrom) {
                            s.originFrom->box[0] = ix;
                            s.originFrom->box[1] = iy;
                        }
                    });
                });
                grid->addWidget(b, row, col);
            }
        }
        auto* boxRow = new QHBoxLayout;
        boxRow->addWidget(level);
        boxRow->addLayout(grid);
        boxRow->addStretch(1);
        form->addRow(tr("Box point"), boxRow);
    }
    const char* names[] = {"X", "Y", "Z"};
    for (int a = 0; a < 3; ++a) {
        auto* spin = new LengthSpinBox(origin.offset[a], unit_);
        spin->setToolTip(
            tr("Moves the origin from the chosen point along WCS %1").arg(QLatin1String(names[a])));
        connect(spin, &LengthSpinBox::mmChanged, this, [edit, a](double v) {
            edit("Offset WCS origin", [&](Setup& s) {
                if (s.originFrom) {
                    s.originFrom->offset[a] = v;
                }
            });
        });
        form->addRow(tr("Offset %1").arg(QLatin1String(names[a])), spin);
    }
    // Why it cannot be found, if it cannot (the view shows where it is).
    try {
        document::resolveWcsOrigin(*doc_, setup);
    } catch (const Error& e) {
        problemLabel(QString::fromUtf8(e.what()));
    }
}

void PropertyPanel::buildPresetOverrides(QFormLayout* form, const document::Operation& op,
                                         const tooling::CuttingPreset& preset,
                                         const tooling::Tool& tool) {
    const Uuid id = op.id;
    auto editOverrides = [this, id](const std::string& label,
                                    std::function<void(document::PresetOverrides&)> change) {
        commit(label, [&](Project& pr) {
            if (document::Operation* o = pr.findOperation(id)) {
                change(o->presetOverrides);
            }
        });
    };
    const document::PresetOverrides& po = op.presetOverrides;
    const tooling::CuttingPreset effective = po.appliedTo(preset);
    const bool turning = tooling::isTurningTool(tool.type);
    // Drilling cycles feed straight down at the cutting feed (fz z n): they are set as a feed per
    // revolution and a plunge feed rate. Taps feed at the pitch.
    const bool tapping = tool.type == tooling::ToolType::Tap;
    const bool drilling = op.type == "drill" || tapping ||
                          (op.type == "bore" && (tool.type == tooling::ToolType::Reamer ||
                                                 tool.type == tooling::ToolType::BoringBar));
    const double d = tool.mill.diameter;
    const int flutes = std::max(1, tool.mill.flutes);
    const bool inch = unit_ == LengthUnit::Inch;
    // Values the operation sets for itself; one equal to the preset's is not stored.
    auto store = [editOverrides](std::optional<double> document::PresetOverrides::*field,
                                 double presetValue, double v) {
        editOverrides("Change speeds and feeds", [&](document::PresetOverrides& o) {
            o.*field = std::abs(v - presetValue) <= 1e-9 * std::max(1.0, std::abs(presetValue))
                           ? std::nullopt
                           : std::optional(v);
        });
    };
    auto spin = [](double value, double min, double max, int decimals, const QString& suffix) {
        auto* s = new QDoubleSpinBox;
        s->setDecimals(decimals);
        s->setRange(min, max);
        s->setSuffix(suffix);
        s->setKeyboardTracking(false);
        s->setValue(value);
        return s;
    };

    const auto addReset = [&] {
        auto* reset = new QPushButton(po.empty() ? tr("From the preset") : tr("Reset to preset"));
        reset->setEnabled(!po.empty());
        reset->setToolTip(tr("Values changed here apply to this operation only; the tool's preset "
                             "stays as it is"));
        connect(reset, &QPushButton::clicked, this, [editOverrides] {
            editOverrides("Reset speeds and feeds", [](document::PresetOverrides& o) { o = {}; });
        });
        form->addRow(QString(), reset);
    };
    if (tool.type == tooling::ToolType::BarPuller) {
        // A bar pull (DL-0091): the spindle stands still, and its two feeds are the puller's.
        const double perMinute = inch ? 60.0 / 25.4 : 60.0;
        const QString suffix = inch ? tr(" in/min") : tr(" mm/min");
        form->addRow(tr("Spindle"), new QLabel(tr("Stopped: the chuck opens")));
        auto* onto = spin(effective.plungeFeed * perMinute, 0.0, 1.0e6, inch ? 2 : 0, suffix);
        onto->setToolTip(tr("Feed of the puller onto the bar and off it again (the preset's "
                            "plunge feed)"));
        connect(onto, &QDoubleSpinBox::valueChanged, this, [store, preset, perMinute](double v) {
            store(&document::PresetOverrides::plungeFeed, preset.plungeFeed, v / perMinute);
        });
        form->addRow(tr("Feed onto the bar"), onto);
        auto* pull = spin(effective.pullFeed * perMinute, 0.0, 1.0e6, inch ? 2 : 0, suffix);
        pull->setToolTip(tr("Feed of the pull, the chuck open"));
        connect(pull, &QDoubleSpinBox::valueChanged, this, [store, preset, perMinute](double v) {
            store(&document::PresetOverrides::pullFeed, preset.pullFeed, v / perMinute);
        });
        form->addRow(tr("Pull feed"), pull);
        addReset();
        return;
    }

    // Cutting speed and spindle speed are one value: n = vc / (pi D).
    auto* vc = spin(surfaceSpeedIn(effective.surfaceSpeed, unit_), 0.1, 100000.0, 1,
                    surfaceSpeedSuffix(unit_));
    connect(vc, &QDoubleSpinBox::valueChanged, this, [store, preset, unit = unit_](double v) {
        store(&document::PresetOverrides::surfaceSpeed, preset.surfaceSpeed,
              surfaceSpeedToMmPerSecond(v, unit));
    });
    vc->setToolTip(tr("Surface speed of the cutting edge (vc); the spindle speed follows from "
                      "it and the tool diameter"));
    form->addRow(tr("Cutting speed"), vc);
    if (!turning && d > 0.0) {
        auto* rpm = spin(effective.spindleSpeed(d) * 60.0, 1.0, 200000.0, 0, tr(" rpm"));
        connect(rpm, &QDoubleSpinBox::valueChanged, this, [store, preset, d](double v) {
            store(&document::PresetOverrides::surfaceSpeed, preset.surfaceSpeed,
                  v / 60.0 * kPi * d); // rev/min -> mm/s at the cutting diameter
        });
        // On a lathe a live tool's speed is its own: the part stands still.
        const Setup* owner = nullptr;
        doc_->project().findOperation(op.id, &owner);
        const machine::Machine* onMachine =
            owner ? document::findMachineFor(doc_->project(), *owner) : nullptr;
        const bool live = onMachine && onMachine->isLathe() && document::liveDrive(op);
        rpm->setToolTip(live
                            ? tr("The live tool's speed: the cutting speed at the tool's diameter. "
                                 "The part stands still at its C angle.")
                            : tr("Spindle speed: the cutting speed at the tool's diameter"));
        form->addRow(live ? tr("Live tool speed") : tr("Spindle speed"), rpm);
    }

    const double n = !turning && d > 0.0 ? effective.spindleSpeed(d) : 0.0; // rev/s
    const double factor = inch ? 60.0 / 25.4 : 60.0; // mm/s -> in/min or mm/min
    const QString rateSuffix = inch ? tr(" in/min") : tr(" mm/min");
    if (tapping) {
        form->addRow(tr("Feed"), new QLabel(tr("Thread pitch × spindle speed")));
    } else if (drilling && d > 0.0) {
        // Feed per revolution (fz z) and the plunge feed rate it gives at this spindle speed.
        auto* fr = new LengthSpinBox(effective.feedPerTooth * flutes, unit_, 0.0);
        connect(fr, &LengthSpinBox::mmChanged, this, [store, preset, flutes](double v) {
            store(&document::PresetOverrides::feedPerTooth, preset.feedPerTooth, v / flutes);
        });
        fr->setToolTip(tr("How far the tool advances per spindle revolution"));
        form->addRow(tr("Feed per revolution"), fr);
        auto* vf =
            spin(effective.feedRate(d, flutes) * factor, 0.0, 1.0e6, inch ? 2 : 0, rateSuffix);
        connect(vf, &QDoubleSpinBox::valueChanged, this,
                [store, preset, n, flutes, factor](double v) {
                    if (n > 0.0) {
                        store(&document::PresetOverrides::feedPerTooth, preset.feedPerTooth,
                              v / factor / (flutes * n));
                    }
                });
        vf->setToolTip(tr("The feed into the hole: feed per revolution × spindle speed"));
        form->addRow(tr("Plunge feed rate"), vf);
    } else {
        // Chip load and feed rate are one value at that spindle speed: vf = fz z n.
        auto* fz = new LengthSpinBox(effective.feedPerTooth, unit_, 0.0);
        connect(fz, &LengthSpinBox::mmChanged, this, [store, preset](double v) {
            store(&document::PresetOverrides::feedPerTooth, preset.feedPerTooth, v);
        });
        fz->setToolTip(turning ? tr("How far the insert advances per spindle revolution")
                               : tr("Chip load: how far the tool advances per tooth (fz); the "
                                    "feed rate follows from it"));
        form->addRow(turning ? tr("Feed per revolution") : tr("Feed per tooth"), fz);
        if (tool.type == tooling::ToolType::TurningInsert) {
            auto* lead = new LengthSpinBox(effective.leadFeedPerRev, unit_, 0.0);
            connect(lead, &LengthSpinBox::mmChanged, this, [store, preset](double v) {
                store(&document::PresetOverrides::leadFeedPerRev, preset.leadFeedPerRev, v);
            });
            lead->setToolTip(tr("Feed per revolution of the lead-in and lead-out moves on and off "
                                "the cut"));
            form->addRow(tr("Lead feed per revolution"), lead);
        }
        if (op.type == "turn_partoff") {
            auto* end = new LengthSpinBox(effective.partOffEndFeed, unit_, 0.0);
            connect(end, &LengthSpinBox::mmChanged, this, [store, preset](double v) {
                store(&document::PresetOverrides::partOffEndFeed, preset.partOffEndFeed, v);
            });
            end->setToolTip(tr("Feed per revolution over the end of the cut, as the part comes "
                               "off; \"End feed over the last\" sets over how much"));
            form->addRow(tr("Part-off end feed"), end);
        }
        if (!turning && d > 0.0) {
            auto* vf =
                spin(effective.feedRate(d, flutes) * factor, 0.0, 1.0e6, inch ? 2 : 0, rateSuffix);
            connect(vf, &QDoubleSpinBox::valueChanged, this,
                    [store, preset, n, flutes, factor](double v) {
                        if (n > 0.0) {
                            store(&document::PresetOverrides::feedPerTooth, preset.feedPerTooth,
                                  v / factor / (flutes * n));
                        }
                    });
            vf->setToolTip(tr("Cutting feed: feed per tooth × flutes × spindle speed"));
            form->addRow(tr("Feed rate"), vf);
        }
    }

    if (!drilling) {
        // The step down and stepover of every strategy are the preset's depths of cut.
        auto addLength = [&](const QString& label, const QString& tip,
                             std::optional<double> document::PresetOverrides::*field,
                             double presetValue) {
            auto* s = new LengthSpinBox((po.*field).value_or(presetValue), unit_, 0.0);
            s->setToolTip(tip);
            connect(s, &LengthSpinBox::mmChanged, this,
                    [store, field, presetValue](double v) { store(field, presetValue, v); });
            form->addRow(label, s);
        };
        addLength(turning ? tr("Depth of cut (ap)") : tr("Axial depth (ap)"),
                  tr("The step down: the depth of each level or pass, from the tool's cutting "
                     "preset"),
                  &document::PresetOverrides::axialDepth, preset.axialDepth);
        if (!turning) {
            addLength(tr("Radial depth (ae)"),
                      tr("The stepover: the width of each pass, from the tool's cutting preset"),
                      &document::PresetOverrides::radialDepth, preset.radialDepth);
        }
    }
    if (!turning && !drilling) {
        // Plunges into material feed at their own rate.
        auto* plunge = spin(effective.plungeFeed * factor, 0.0, 1.0e6, inch ? 2 : 0, rateSuffix);
        plunge->setToolTip(tr("Feed for plunges straight down into the material"));
        connect(plunge, &QDoubleSpinBox::valueChanged, this, [store, preset, factor](double v) {
            store(&document::PresetOverrides::plungeFeed, preset.plungeFeed, v / factor);
        });
        form->addRow(tr("Plunge feed rate"), plunge);
        // Ramps and helixes into the material, too.
        auto* ramp = spin(effective.rampFeed * factor, 0.0, 1.0e6, inch ? 2 : 0, rateSuffix);
        ramp->setToolTip(tr("Feed for ramps and helixes into the material"));
        connect(ramp, &QDoubleSpinBox::valueChanged, this, [store, preset, factor](double v) {
            store(&document::PresetOverrides::rampFeed, preset.rampFeed, v / factor);
        });
        form->addRow(tr("Ramp feed rate"), ramp);
        auto* lead = spin(effective.leadFeed * factor, 0.0, 1.0e6, inch ? 2 : 0, rateSuffix);
        lead->setToolTip(tr("Feed for the lead-in and lead-out moves on and off the wall"));
        connect(lead, &QDoubleSpinBox::valueChanged, this, [store, preset, factor](double v) {
            store(&document::PresetOverrides::leadFeed, preset.leadFeed, v / factor);
        });
        form->addRow(tr("Lead feed rate"), lead);
    }
    addReset();
}

void PropertyPanel::buildGeometry(QFormLayout* form, const document::Operation& op,
                                  strategies::GeometryUse use, const Setup& setup) {
    using strategies::GeometryUse;
    const Uuid id = op.id;
    // Contours follow faces or closed loops of edges, chamfers loops of edges; the others use
    // faces.
    const bool edges = use == GeometryUse::FacesOrEdges || use == GeometryUse::Edges;
    // A wrapped contour's side is the operation's, for open and closed curves alike: no Flip.
    const bool flips = edges && op.type != "rotary_contour";
    auto* list = new QListWidget;
    list->setSelectionMode(QAbstractItemView::ExtendedSelection);
    list->setMaximumHeight(edges ? 160 : 110);
    list->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    list->setResizeMode(QListView::Adjust); // rows as wide as the list
    // Contours and chamfers: each face or closed loop runs on its own side, by default the
    // operation's.
    // Edges that make up a whole outline are one row ("Contour"); each row keeps the
    // indices of its geometry items.
    std::vector<bool> grouped(op.geometry.size(), false);
    int contours = 0;
    int faces = 0;
    int singleEdges = 0;
    for (std::size_t k = 0; k < op.geometry.size(); ++k) {
        const document::ShapeRef& ref = op.geometry[k];
        if (grouped[k]) {
            continue;
        }
        QVariantList rows{static_cast<int>(k)};
        QString text;
        if (ref.type == cad::SubShapeType::Edge && edges) {
            const std::vector<int> outline = outlineOf(ref.modelId, ref.index);
            std::vector<int> members;
            for (const int index : outline) {
                for (std::size_t j = 0; j < op.geometry.size(); ++j) {
                    const auto& other = op.geometry[j];
                    if (other.type == cad::SubShapeType::Edge && other.modelId == ref.modelId &&
                        other.index == index) {
                        members.push_back(static_cast<int>(j));
                    }
                }
            }
            if (outline.size() > 1 && members.size() == outline.size()) {
                rows.clear();
                for (const int j : members) {
                    rows.push_back(j);
                    grouped[static_cast<std::size_t>(j)] = true;
                }
                text = tr("Contour %1").arg(++contours);
            }
        }
        if (text.isEmpty()) {
            // Numbered in the list (clicking a row shows it in the view).
            text = ref.type == cad::SubShapeType::Face ? tr("Face %1").arg(++faces)
                                                       : tr("Edge %1").arg(++singleEdges);
            grouped[k] = true;
        }
        auto* item = new QListWidgetItem(flips ? QString() : text, list);
        item->setData(Qt::UserRole, rows);
        if (!flips) {
            continue;
        }
        auto* row = new QWidget;
        auto* layout = new QHBoxLayout(row);
        layout->setContentsMargins(4, 1, 4, 1);
        // The name gives way to the side (cut short; the whole name in its tooltip).
        auto* name = new QLabel(text);
        name->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
        name->setToolTip(text);
        name->setProperty("singleLine", true); // one line per row (see compactPage)
        layout->addWidget(name, 1);
        // Each outline runs on the side it opens to (away from the part); Flip runs it on the
        // other side.
        const document::GeometrySide natural = [&] {
            const auto automatic =
                document::automaticSide(*doc_, document::operationSetup(setup, op), ref);
            return automatic == document::GeometrySide::Default ? document::GeometrySide::Outside
                                                                : automatic;
        }();
        auto* side = new QCheckBox(tr("Flip"));
        side->setChecked(ref.side != document::GeometrySide::Default && ref.side != natural);
        side->setToolTip(tr("Run on the other side of this outline (normally the side it opens "
                            "to, away from the part)"));
        std::vector<int> members;
        for (const QVariant& r : rows) {
            members.push_back(r.toInt());
        }
        const document::GeometrySide flipped = natural == document::GeometrySide::Outside
                                                   ? document::GeometrySide::Inside
                                                   : document::GeometrySide::Outside;
        connect(side, &QCheckBox::toggled, this, [this, id, members, flipped](bool on) {
            commit(on ? "Flip outline" : "Unflip outline", [&](Project& pr) {
                if (document::Operation* o = pr.findOperation(id)) {
                    for (const int m : members) {
                        if (m < static_cast<int>(o->geometry.size())) {
                            o->geometry[static_cast<std::size_t>(m)].side =
                                on ? flipped : document::GeometrySide::Default;
                        }
                    }
                }
            });
        });
        layout->addWidget(side);
        item->setSizeHint(QSize(0, row->sizeHint().height()));
        list->setItemWidget(item, row);
    }
    // Rows show their faces and edges selected in the view.
    connect(list, &QListWidget::itemSelectionChanged, this, [this, list, op] {
        std::vector<document::ShapeRef> shown;
        for (const QListWidgetItem* item : list->selectedItems()) {
            for (const QVariant& row : item->data(Qt::UserRole).toList()) {
                shown.push_back(op.geometry[static_cast<std::size_t>(row.toInt())]);
            }
        }
        emit highlightShapes(shown);
    });
    QString title;
    QString help;
    switch (use) {
    case GeometryUse::Faces:
        title = tr("Faces");
        help = strategies::isProbingStrategy(op.type)
                   ? tr("The faces to measure: one flat face (a surface), two parallel faces (a "
                        "web or pocket), two at a right angle (a corner) or one cylindrical face "
                        "(a bore or boss). Picking them fills in the nominal values")
                   : tr("The faces to machine");
        break;
    case GeometryUse::Holes:
        title = tr("Holes");
        help = tr("Faces of the holes to machine (an external thread: of the bosses); none = "
                  "every hole or boss that fits the tool");
        break;
    case GeometryUse::Containment:
        title = tr("Contain to");
        help = tr("Faces to finish: the tool stays where it reaches them, between their heights "
                  "unless the heights say otherwise; none = the whole part");
        break;
    case GeometryUse::Walls:
        title = tr("Walls");
        help = tr("Ruled wall faces to cut with the side of the tool. A flat face facing up "
                  "(within 45° of the tool axis) is the floor the tool stays above");
        break;
    case GeometryUse::Edges:
        title = tr("Edges");
        help = tr("Edges to chamfer, across the tool axis; one click takes a whole outline. "
                  "Each runs on the side away from the part unless flipped");
        break;
    case GeometryUse::Chains:
        title = tr("Edges");
        help = tr("Edges to trace: edges that meet run on as one chain, open or closed, seen "
                  "from above");
        break;
    case GeometryUse::AnyEdges:
        title = tr("Edges");
        help = tr("Edges to break, flat or not; each click takes one edge");
        break;
    default:
        if (op.type == "rotary_contour") {
            title = tr("Geometry");
            help = tr("Edges to follow around the part, open or closed, or the floor of a slot "
                      "(a face) to cut inside its walls");
            break;
        }
        title = tr("Geometry");
        help = tr("Faces, or closed loops of edges, to follow. Each runs on the side away from "
                  "the part unless flipped");
        break;
    }
    list->setToolTip(help);
    form->addRow(title, list);

    const bool picking = geometryPick_ == id;
    auto* select = new QPushButton(picking ? tr("Done") : tr("Select"));
    select->setCheckable(true);
    select->setChecked(picking);
    select->setToolTip(use == GeometryUse::Edges
                           ? tr("Click edges in the view to add or remove them (horizontal ones "
                                "only); Done or Esc finishes")
                       : use == GeometryUse::Chains
                           ? tr("Click edges in the view to add or remove them, one at a time; "
                                "Done or Esc finishes")
                       : use == GeometryUse::AnyEdges
                           ? tr("Click edges in the view to add or remove them; Done or Esc "
                                "finishes")
                       : edges ? tr("Click faces or edges in the view to add or remove them; Done "
                                    "or Esc finishes")
                               : tr("Click faces in the view to add or remove them; Done or Esc "
                                    "finishes"));
    auto* remove = new QPushButton(tr("Remove"));
    remove->setToolTip(tr("Remove the items selected in the list"));
    auto* clear = new QPushButton(tr("Clear"));
    auto edit = [this, id](const std::string& label,
                           std::function<void(document::Operation&)> change) {
        commit(label, [&](Project& pr) {
            if (document::Operation* o = pr.findOperation(id)) {
                change(*o);
            }
        });
    };
    connect(select, &QPushButton::clicked, this, [this, id, op, setup] {
        if (geometryPick_ == id) {
            stopGeometryPick();
            return;
        }
        cancelPick();
        geometryPick_ = id;
        // Faces (and edges) already selected in the view become the geometry; otherwise the
        // view shows the current geometry selected, to add to or remove from.
        std::vector<document::ShapeRef> already =
            pickedShapes_ ? pickedShapes_() : std::vector<document::ShapeRef>{};
        std::erase_if(already, [&](const document::ShapeRef& r) { return !accepts(op, setup, r); });
        if (!already.empty()) {
            commit("Pick operation geometry", [&](Project& pr) {
                if (document::Operation* o = pr.findOperation(id)) {
                    o->geometry = already;
                    fillProbeNominals(*doc_, pr, id);
                }
            });
        } else {
            emit highlightShapes(op.geometry);
        }
        emit toggleSelectionRequested(true);
        emit pickPrompt(tr("Click to add or remove geometry; Done or Esc finishes"));
        refreshLater();
    });
    connect(remove, &QPushButton::clicked, this, [list, edit] {
        std::vector<int> rows;
        for (const QListWidgetItem* item : list->selectedItems()) {
            for (const QVariant& row : item->data(Qt::UserRole).toList()) {
                rows.push_back(row.toInt());
            }
        }
        if (rows.empty()) {
            return;
        }
        std::sort(rows.rbegin(), rows.rend());
        edit("Remove operation geometry", [&](document::Operation& o) {
            for (const int row : rows) {
                o.geometry.erase(o.geometry.begin() + row);
            }
        });
    });
    connect(clear, &QPushButton::clicked, this, [edit] {
        edit("Clear operation geometry", [&](document::Operation& o) { o.geometry.clear(); });
    });
    auto* buttons = new QHBoxLayout;
    buttons->addWidget(select);
    buttons->addWidget(remove);
    buttons->addWidget(clear);
    form->addRow(QString(), buttons);
}

void PropertyPanel::buildStartPoint(QFormLayout* form, const document::Operation& op) {
    const Uuid id = op.id;
    auto setStart = [this, id](std::optional<document::ShapeRef> point) {
        commit(point ? "Set start point" : "Clear start point", [&](Project& pr) {
            if (document::Operation* o = pr.findOperation(id)) {
                o->startPoint = point;
            }
        });
    };
    QString text = tr("Automatic");
    if (op.startPoint) {
        text = op.startPoint->type == cad::SubShapeType::Vertex
                   ? tr("Vertex %1").arg(op.startPoint->index)
                   : tr("Edge %1").arg(op.startPoint->index);
    }
    auto* label = new QLabel(text);
    auto* pick = pickButton(QStringLiteral("start"), tr("Pick"));
    pick->setToolTip(op.type == "trace2d"
                         ? tr("Start each chain at its end, or its point, nearest a vertex or a "
                              "vertical edge: the selected one, or the next one you pick")
                         : tr("Start the contour (its lead-in) next to a vertex or a vertical "
                              "edge: the selected one, or the next one you pick"));
    auto* clear = new QPushButton(tr("Auto"));
    clear->setEnabled(op.startPoint.has_value());
    connect(pick, &QPushButton::clicked, this, [this, setStart] {
        emit vertexPickingRequested(true);
        withPick(QStringLiteral("start"), std::nullopt,
                 tr("Pick a vertex or a vertical edge for the start point"),
                 [this, setStart](const document::ShapeRef& ref) {
                     emit vertexPickingRequested(false);
                     if (ref.type == cad::SubShapeType::Face) {
                         QMessageBox::information(this, tr("Start point"),
                                                  tr("Pick a vertex or a vertical edge."));
                         return;
                     }
                     setStart(ref);
                 });
    });
    connect(clear, &QPushButton::clicked, this, [setStart] { setStart(std::nullopt); });
    auto* row = new QHBoxLayout;
    row->addWidget(label);
    row->addWidget(pick, 1);
    row->addWidget(clear, 1);
    form->addRow(tr("Start point"), row);
}

bool PropertyPanel::accepts(const document::Operation& op, const Setup& setup,
                            const document::ShapeRef& ref) const {
    const strategies::StrategyInfo* info = strategies::findStrategy(op.type);
    const strategies::GeometryUse use = info ? info->geometry : strategies::GeometryUse::Faces;
    switch (ref.type) {
    case cad::SubShapeType::Face:
        return use != strategies::GeometryUse::Edges && use != strategies::GeometryUse::Chains &&
               use != strategies::GeometryUse::AnyEdges;
    case cad::SubShapeType::Edge: {
        if (use == strategies::GeometryUse::FacesOrEdges ||
            use == strategies::GeometryUse::Chains || use == strategies::GeometryUse::AnyEdges) {
            return true; // traces and edge breaks: any edge
        }
        if (use != strategies::GeometryUse::Edges) {
            return false;
        }
        // Chamfers: edges across the tool axis (the operation's Z in the model's coordinates).
        const cad::Model* model = doc_->model(ref.modelId);
        const document::ModelEntry* entry = doc_->project().findModel(ref.modelId);
        if (!model || !entry) {
            return false;
        }
        const Vec3 axis =
            document::modelToWcs(*entry, document::operationSetup(setup, op)).linear().transpose() *
            Vec3::UnitZ();
        return cad::edgeAcross(*model, ref.index, axis);
    }
    default:
        return false;
    }
}

bool PropertyPanel::pickingOutlines() const {
    if (!geometryPick_ || !doc_) {
        return false;
    }
    const document::Operation* op = doc_->project().findOperation(*geometryPick_);
    return op && takesEdges(op->type);
}

std::vector<int> PropertyPanel::outlineOf(const Uuid& model, int edgeIndex) const {
    const Setup* owner = nullptr;
    const Uuid opId = geometryPick_ ? *geometryPick_ : selection_.id;
    const cad::Model* m = doc_ ? doc_->model(model) : nullptr;
    const document::ModelEntry* entry = doc_ ? doc_->project().findModel(model) : nullptr;
    const document::Operation* op = doc_ ? doc_->project().findOperation(opId, &owner) : nullptr;
    if (!m || !entry || !op || !owner) {
        return {edgeIndex};
    }
    // The tool axis (the operation's Z) in the model's own coordinates.
    const Vec3 axis = entry->placement.toParent().linear().transpose() *
                      document::operationSetup(*owner, *op).wcs.zAxis.normalized();
    return cad::edgeLoopOf(*m, edgeIndex, axis);
}

void PropertyPanel::stopGeometryPick() {
    if (geometryPick_) {
        geometryPick_.reset();
        emit toggleSelectionRequested(false);
        emit pickPrompt(QString());
        refreshLater();
    }
}

void PropertyPanel::buildHeights(QFormLayout* form, const document::Operation& op,
                                 const Setup& setup) {
    using document::HeightRef;
    using document::HeightSetting;
    const std::string& type = op.type;
    if (type.rfind("turn_", 0) == 0 || type == "bar_pull" || type == "spindle_transfer") {
        return; // turning works on its own geometry, and a bar pull on the bar's front
    }
    const bool holes = document::isHoleOperation(type);
    const bool topBottom = document::usesTopAndBottom(type);
    const Uuid id = op.id;
    // Every height shows what it is measured from: a Default setting (files of earlier
    // versions) as the value it stands for.
    const document::OperationHeights effective = document::effectiveHeights(op);
    auto* heading = new QLabel(tr("<b>Heights</b>"));
    form->addRow(heading);

    auto label = [&](HeightRef ref) -> QString {
        switch (ref) {
        case HeightRef::Default: // never shown: effectiveHeights() replaces it
        case HeightRef::StockTop:
            return tr("Top of stock");
        case HeightRef::StockBottom:
            return tr("Bottom of stock");
        case HeightRef::PartTop:
            return tr("Top of part");
        case HeightRef::PartBottom:
            return tr("Bottom of part");
        case HeightRef::Geometry:
            return tr("Selected geometry");
        case HeightRef::Origin:
            return tr("WCS zero");
        case HeightRef::Clearance:
            return tr("Clearance height");
        case HeightRef::Retract:
            return tr("Retract height");
        case HeightRef::TopHeight:
            return tr("Top height");
        case HeightRef::Picked:
            return tr("Picked face / edge");
        case HeightRef::HoleTop:
            return tr("Each hole's top");
        }
        return {};
    };
    struct Row {
        QString name;
        HeightSetting document::OperationHeights::*field;
        std::vector<HeightRef> refs;
        QString help;
    };
    using R = HeightRef;
    std::vector<Row> rows = {
        {tr("Clearance"),
         &document::OperationHeights::clearance,
         {R::Retract, R::StockTop, R::PartTop, R::Origin},
         tr("Safe height for rapid moves between regions and to and from the operation")},
        {tr("Retract"),
         &document::OperationHeights::retract,
         {R::StockTop, R::PartTop, R::Clearance, R::TopHeight, R::Origin},
         strategies::isProbingStrategy(type)
             ? tr("Height the probe rapids to above each touch; below it, it moves only by "
                  "protected moves")
             : tr("Height the tool rapids to between cuts within the operation")},
    };
    if (strategies::isProbingStrategy(type)) {
        // A probe has no feed height: it goes down from the retract height by protected moves.
    } else if (holes) {
        rows.push_back({tr("Feed (R plane)"),
                        &document::OperationHeights::feed,
                        {R::HoleTop, R::Retract, R::TopHeight, R::StockTop, R::PartTop, R::Origin},
                        tr("Where the hole cycles start feeding and retract to between pecks "
                           "(the R plane): above each hole's own top, or one height for all")});
    } else {
        rows.push_back({tr("Feed"),
                        &document::OperationHeights::feed,
                        {R::Retract, R::TopHeight, R::StockTop, R::PartTop, R::Origin},
                        tr("Where the rapid down into the operation's top ends and feeding "
                           "starts; lower levels start just above the level before")});
    }
    if (topBottom) {
        rows.push_back({tr("Top"),
                        &document::OperationHeights::top,
                        {R::StockTop, R::PartTop, R::Geometry, R::Origin},
                        tr("Top of the material the operation cuts")});
        rows.push_back(
            {tr("Bottom"),
             &document::OperationHeights::bottom,
             {R::PartBottom, R::Geometry, R::StockBottom, R::TopHeight, R::PartTop, R::Origin},
             tr("Final depth of the operation")});
    }
    const strategies::StrategyInfo* info = strategies::findStrategy(type);
    const bool geometry = info && info->geometry != strategies::GeometryUse::None;
    int rowIndex = 0;
    for (Row row : rows) {
        const HeightSetting current = effective.*row.field;
        if (std::find(row.refs.begin(), row.refs.end(), current.from) == row.refs.end()) {
            row.refs.push_back(current.from); // a picked face, say: shown while used
        }
        if (!geometry && current.from != R::Geometry) {
            std::erase(row.refs, R::Geometry); // the operation selects no geometry
        }
        auto* ref = new QComboBox;
        for (const HeightRef r : row.refs) {
            ref->addItem(label(r), static_cast<int>(r));
        }
        ref->setCurrentIndex(std::max(0, ref->findData(static_cast<int>(current.from))));
        auto* offset = new LengthSpinBox(current.offset, unit_, -100000.0);
        // Sized by the panel, not by its range, so the row fits the dock.
        offset->setSizePolicy(QSizePolicy::Ignored, offset->sizePolicy().verticalPolicy());
        offset->setMinimumWidth(50);
        // Pick: the selected face or edge, or the next one picked in the view.
        const QString pickId = QStringLiteral("height%1").arg(rowIndex++);
        auto* pick = new QToolButton;
        pick->setText(tr("Pick"));
        pick->setCheckable(true);
        pick->setChecked(pending_ && pending_->id == pickId);
        pick->setToolTip(tr("Measure this height from a face or edge: the selected one, or the "
                            "next one you pick (the Z of its centre)"));
        auto* line = new QHBoxLayout;
        line->addWidget(ref, 3);
        line->addWidget(offset, 2);
        line->addWidget(pick);
        form->addRow(row.name, line);
        if (QWidget* rowLabel = form->labelForField(line)) {
            rowLabel->setToolTip(
                formatToolTip(row.help + tr("\n\nThe offset is added to the reference along the "
                                            "WCS Z."),
                              row.name));
        }
        const auto field = row.field;
        auto apply = [this, id, field](HeightSetting setting) {
            commit("Change operation heights", [&](Project& pr) {
                if (document::Operation* o = pr.findOperation(id)) {
                    o->heights = document::effectiveHeights(*o); // all of them as shown
                    o->heights.*field = setting;
                }
            });
        };
        connect(ref, &QComboBox::currentIndexChanged, this, [ref, offset, apply, current] {
            const auto from = static_cast<HeightRef>(ref->currentData().toInt());
            apply(
                {from, offset->mm(),
                 from == HeightRef::Picked ? current.shape : std::optional<document::ShapeRef>{}});
        });
        connect(offset, &LengthSpinBox::mmChanged, this, [ref, apply, current](double v) {
            const auto from = static_cast<HeightRef>(ref->currentData().toInt());
            apply(
                {from, v,
                 from == HeightRef::Picked ? current.shape : std::optional<document::ShapeRef>{}});
        });
        const QString heightName = row.name.toLower();
        connect(pick, &QToolButton::clicked, this, [this, pickId, heightName, offset, apply] {
            // The page may be rebuilt before the pick: keep the offset, not the widget.
            const double off = offset->mm();
            withPick(pickId, std::nullopt,
                     tr("Pick a face or edge for the %1 height").arg(heightName),
                     [off, apply](const document::ShapeRef& shape) {
                         apply({HeightRef::Picked, off, shape});
                     });
        });
    }
    // What the settings come to.
    QString summary;
    try {
        const document::OperationHeightValues hv = document::operationHeights(*doc_, setup, op);
        QStringList parts;
        parts << tr("clearance %1").arg(formatLength(hv.clearance, unit_, false))
              << tr("retract %1").arg(formatLength(hv.retract, unit_, false));
        if (hv.feed) {
            parts << tr("feed %1").arg(formatLength(*hv.feed, unit_, false));
        } else if (hv.holeFeed) {
            parts << tr("feed %1 above each hole").arg(formatLength(*hv.holeFeed, unit_, false));
        }
        if (hv.top) {
            parts << tr("top %1").arg(formatLength(*hv.top, unit_, false));
        }
        if (hv.bottom) {
            parts << tr("bottom %1").arg(formatLength(*hv.bottom, unit_, false));
        }
        summary =
            tr("Z: %1 %2").arg(parts.join(QStringLiteral(", ")), lengthSuffix(unit_).trimmed());
        for (const auto& w : hv.warnings) {
            summary += QStringLiteral("\n") + QString::fromStdString(w);
        }
    } catch (const Error& e) {
        summary = QString::fromUtf8(e.what());
    }
    auto* values = new QLabel(summary);
    values->setStyleSheet(QStringLiteral("color: gray"));
    form->addRow(QString(), values);
}

void PropertyPanel::buildOrientation(QFormLayout* form, const document::Operation& op,
                                     const Setup& setup) {
    const Uuid id = op.id;
    auto edit = [this, id](const std::string& label,
                           std::function<void(document::Operation&)> change) {
        commit(label, [&](Project& pr) {
            if (document::Operation* o = pr.findOperation(id)) {
                change(*o);
            }
        });
    };
    form->addRow(new QLabel(tr("<b>Tool orientation</b>")));

    // The tool axis: the setup's WCS, or a direction of its own in the WCS (3+2).
    struct Direction {
        const char* label;
        Vec3 z;
        Vec3 x;
    };
    static const Direction kDirections[] = {
        {QT_TR_NOOP("WCS +Z (top)"), Vec3::UnitZ(), Vec3::UnitX()},
        {QT_TR_NOOP("WCS -Y (front)"), -Vec3::UnitY(), Vec3::UnitX()},
        {QT_TR_NOOP("WCS +X (right)"), Vec3::UnitX(), Vec3::UnitY()},
        {QT_TR_NOOP("WCS +Y (back)"), Vec3::UnitY(), -Vec3::UnitX()},
        {QT_TR_NOOP("WCS -X (left)"), -Vec3::UnitX(), -Vec3::UnitY()},
        {QT_TR_NOOP("WCS -Z (bottom)"), -Vec3::UnitZ(), Vec3::UnitX()},
    };
    const std::optional<document::ToolOrientation>& current = op.orientation;
    auto* axis = new QComboBox;
    axis->addItem(tr("The setup's WCS"));
    int axisIndex = current ? -1 : 0;
    for (const Direction& d : kDirections) {
        axis->addItem(tr(d.label));
        if (current && (current->frame.zAxis.normalized() - d.z).norm() < 1e-9 &&
            (current->frame.xAxis.normalized() - d.x).norm() < 1e-9) {
            axisIndex = axis->count() - 1;
        }
    }
    if (axisIndex < 0) {
        axis->addItem(tr("Custom"));
        axisIndex = axis->count() - 1;
    }
    axis->setCurrentIndex(axisIndex);
    axis->setToolTip(tr("The tool axis this operation cuts along: the setup's WCS Z, or a "
                        "direction of its own in the setup's WCS. The machine's rotary axes "
                        "turn the part (or the head) to it before the operation (3+2)"));
    connect(axis, &QComboBox::currentIndexChanged, this, [edit](int i) {
        if (i == 0) {
            edit("Use the setup's tool axis",
                 [](document::Operation& o) { o.orientation.reset(); });
            return;
        }
        if (i > static_cast<int>(std::size(kDirections))) {
            return;
        }
        const Direction& d = kDirections[i - 1];
        edit("Change operation tool axis", [&](document::Operation& o) {
            document::ToolOrientation t = o.orientation.value_or(document::ToolOrientation{});
            t.frame.zAxis = d.z;
            t.frame.xAxis = d.x;
            o.orientation = t;
        });
    });
    form->addRow(tr("Tool axis"), axis);

    // From picked geometry, like the setup's WCS.
    const Transform3 wcsFromProject = setup.wcs.toParent().inverse();
    const Vec3 xNow = current ? current->frame.xAxis.normalized() : Vec3(Vec3::UnitX());
    const Vec3 zNow = current ? current->frame.zAxis.normalized() : Vec3(Vec3::UnitZ());
    auto* zFromFace = pickButton(QStringLiteral("opZFace"), tr("Z ⟂ face"));
    zFromFace->setToolTip(tr("Tool axis along the outward normal of a planar face, origin at its "
                             "centre: the selected face, or the next one you pick"));
    connect(zFromFace, &QPushButton::clicked, this, [this, edit, wcsFromProject, xNow] {
        withPick(QStringLiteral("opZFace"), cad::SubShapeType::Face,
                 tr("Pick a planar face for the operation's tool axis"),
                 [this, edit, wcsFromProject, xNow](const document::ShapeRef& ref) {
                     const cad::Model* model = doc_->model(ref.modelId);
                     const document::ModelEntry* entry = doc_->project().findModel(ref.modelId);
                     const auto plane =
                         model && entry ? cad::facePlane(*model, ref.index) : std::nullopt;
                     if (!plane) {
                         QMessageBox::information(this, tr("Tool orientation"),
                                                  tr("The picked face is not planar."));
                         return;
                     }
                     const Transform3 toWcs = wcsFromProject * entry->placement.toParent();
                     const Vec3 nz = (toWcs.linear() * plane->normal).normalized();
                     // Keep X where it was as far as possible.
                     Vec3 x = xNow - xNow.dot(nz) * nz;
                     for (const Vec3& fallback : {Vec3(Vec3::UnitX()), Vec3(Vec3::UnitY())}) {
                         if (x.norm() < 1e-6) {
                             x = fallback - fallback.dot(nz) * nz;
                         }
                     }
                     const Vec3 origin = toWcs * ref.signature.centroid;
                     edit("Orient operation to face", [&](document::Operation& o) {
                         document::ToolOrientation t =
                             o.orientation.value_or(document::ToolOrientation{});
                         t.frame = document::Frame{origin, x.normalized(), nz};
                         o.orientation = t;
                     });
                 });
    });
    auto* xFromEdge = pickButton(QStringLiteral("opXEdge"), tr("X ∥ edge"));
    xFromEdge->setToolTip(tr("The operation's +X along a straight edge: the selected edge, or "
                             "the next one you pick; again on the same edge reverses X"));
    connect(xFromEdge, &QPushButton::clicked, this, [this, edit, wcsFromProject, xNow, zNow] {
        withPick(QStringLiteral("opXEdge"), cad::SubShapeType::Edge,
                 tr("Pick a straight edge for the operation's X direction"),
                 [this, edit, wcsFromProject, xNow, zNow](const document::ShapeRef& ref) {
                     const document::ModelEntry* entry = doc_->project().findModel(ref.modelId);
                     if (!entry || ref.signature.direction.isZero()) {
                         QMessageBox::information(this, tr("Tool orientation"),
                                                  tr("Pick a straight edge."));
                         return;
                     }
                     Vec3 x = wcsFromProject.linear() * entry->placement.toParent().linear() *
                              ref.signature.direction;
                     x -= x.dot(zNow) * zNow;
                     if (x.norm() < 1e-6) {
                         QMessageBox::information(
                             this, tr("Tool orientation"),
                             tr("Pick a straight edge that is not parallel to the tool axis."));
                         return;
                     }
                     x.normalize();
                     if (std::abs(x.dot(xNow)) > 1.0 - 1e-9) {
                         x = -xNow;
                     } else if (x.dot(xNow) < 0.0) {
                         x = -x;
                     }
                     edit("Orient operation X to edge", [&](document::Operation& o) {
                         document::ToolOrientation t =
                             o.orientation.value_or(document::ToolOrientation{});
                         t.frame.zAxis = zNow;
                         t.frame.xAxis = x;
                         o.orientation = t;
                     });
                 });
    });
    auto* orientRow = new QHBoxLayout;
    orientRow->addWidget(zFromFace);
    orientRow->addWidget(xFromEdge);
    form->addRow(tr("Orient from pick"), orientRow);
    if (!current) {
        return;
    }

    // Origin (in the setup's WCS): heights measured from "Origin" and a work offset of its own
    // are measured there.
    const char* names[] = {"X", "Y", "Z"};
    for (int a = 0; a < 3; ++a) {
        auto* spin = new LengthSpinBox(current->frame.origin[a], unit_);
        spin->setToolTip(tr("The origin of the operation's frame, in the setup's WCS: heights "
                            "measured from the origin, and a work offset of its own, are "
                            "measured there"));
        connect(spin, &LengthSpinBox::mmChanged, this, [edit, a](double v) {
            edit("Move operation origin", [&](document::Operation& o) {
                if (o.orientation) {
                    o.orientation->frame.origin[a] = v;
                }
            });
        });
        form->addRow(tr("Origin %1 (WCS)").arg(QLatin1String(names[a])), spin);
    }

    // The setup's: positions are worked out from the setup's work offset through the table
    // centre. Another: a work offset of its own, measured at this origin with the machine
    // indexed to this orientation.
    addWorkOffsetRows(form, doc_->project(), setup, current->workOffset, true, [edit](int n) {
        edit("Change operation work offset", [&](document::Operation& o) {
            if (o.orientation) {
                o.orientation->workOffset = n;
            }
        });
    });

    // What the machine does for it.
    const machine::Machine* onMachine = document::findMachineFor(doc_->project(), setup);
    if (!onMachine) {
        return; // the setup page says why
    }
    const machine::Machine& machine = *onMachine;
    auto* index = new QLabel;
    index->setWordWrap(true);
    if (machine.isLathe()) {
        // A lathe's live tool: C turns the part to the tool (DL-0063).
        const Vec3 z = op.orientation->frame.zAxis.normalized();
        if (std::abs(z.z()) < 1e-6) {
            double c = radToDeg(std::atan2(z.y(), z.x()));
            c = c < 0.0 ? c + 360.0 : c;
            index->setText(tr("C%1°: the part turned to the tool").arg(c, 0, 'f', 3));
        } else {
            index->setText(
                tr("Along the spindle: C by the operation's holes or its Live tool axes"));
        }
        index->setToolTip(tr("Where C stands for this operation"));
        form->addRow(tr("C axis"), index);
        return;
    }
    try {
        const document::OperationIndex ix = document::operationIndex(setup, op, machine);
        QStringList words;
        const double values[] = {ix.angles.a, ix.angles.b, ix.angles.c};
        for (const char letter : machine.rotaryAxes()) {
            words << QStringLiteral("%1%2°")
                         .arg(QLatin1Char(letter))
                         .arg(radToDeg(values[letter - 'A']), 0, 'f', 3);
        }
        index->setText(words.isEmpty() ? tr("No rotary axes: the program turns about Z only")
                                       : words.join(QLatin1Char(' ')));
    } catch (const Error& e) {
        index->setText(qs(e.what()));
        index->setStyleSheet(QStringLiteral("color: #c0392b;"));
    }
    index->setToolTip(tr("The rotary positions the setup's machine indexes to for this "
                         "operation"));
    form->addRow(tr("Indexed to"), index);
}

void PropertyPanel::setTurretStation(int station) {
    turretStation_ = station;
    if (QWidget* page = widget()) {
        if (auto* button =
                page->findChild<QToolButton*>(QStringLiteral("setupStationRow%1").arg(station))) {
            button->setChecked(true);
        }
    }
}

void PropertyPanel::selectTurretStation(int station) {
    setTurretStation(station);
    emit turretStationSelected(station);
}

void PropertyPanel::buildTurretSummary(QFormLayout* form, const Setup& setup,
                                       const machine::Machine& machine) {
    const document::TurretLoad load = document::turretLoad(*doc_, setup, machine);
    const document::Project& p = doc_->project();
    // A line a station that has something on it: its block, its tool.
    QStringList lines;
    std::set<int> numbers;
    for (const document::StationBlock& b : setup.turretBlocks) {
        numbers.insert(b.station);
    }
    for (const document::TurretStation& s : load.stations) {
        numbers.insert(s.station);
    }
    for (const int n : numbers) {
        QStringList what;
        for (const document::StationBlock& b : setup.turretBlocks) {
            if (b.station == n) {
                const document::LatheFitting* f = p.findFitting(b.block);
                what << (f ? qs(f->name) : tr("a block not in the project"));
            }
        }
        for (const document::TurretStation& s : load.stations) {
            if (s.station == n) {
                const tooling::Tool* t = p.tools.findTool(s.tool);
                what << (t ? QStringLiteral("T%1  %2").arg(t->number).arg(qs(t->name))
                           : qs(s.name));
            }
        }
        lines << tr("%1: %2").arg(n).arg(what.join(QStringLiteral(", ")));
    }
    auto* summary = new QLabel(lines.isEmpty() ? tr("Nothing on the stations yet.")
                                               : lines.join(QLatin1Char('\n')));
    summary->setObjectName(QStringLiteral("setupTurretSummary"));
    summary->setWordWrap(true);
    summary->setToolTip(tr("Station: its block, its tool. Set them up in the turret window."));
    form->addRow(summary);
    for (const std::string& problem : load.problems) {
        auto* warn = new QLabel(qs(problem));
        warn->setWordWrap(true);
        warn->setStyleSheet(QStringLiteral("color: #c0392b;"));
        form->addRow(warn);
    }
    auto* open = new QPushButton(tr("Turret setup..."));
    open->setObjectName(QStringLiteral("setupTurretWindow"));
    open->setToolTip(tr("Choose each station's block and tool, and their stickout, in a window "
                        "of its own beside the turret preview: the machine with every block and "
                        "tool where the simulation puts them, the chosen station turned to the "
                        "work."));
    const Uuid id = setup.id;
    connect(open, &QPushButton::clicked, this,
            [this, id] { emit turretWindowRequested(id, turretStation_); });
    form->addRow(open);
}

void PropertyPanel::buildTurret(QFormLayout* form, const Setup& setup,
                                const machine::Machine& machine) {
    if (!turretOnly_) {
        buildTurretSummary(form, setup, machine);
        return;
    }
    const Uuid id = setup.id;
    auto edit = [this, id](const std::string& label,
                           const std::function<void(document::Setup&)>& change) {
        commit(label, [&](Project& pr) {
            if (Setup* s = pr.findSetup(id)) {
                change(*s);
            }
        });
    };
    /// The setup's entry for a tool, added when it has none.
    auto entryOf = [](document::Setup& s, const Uuid& tool) -> document::TurretEntry& {
        const auto it =
            std::find_if(s.turret.begin(), s.turret.end(),
                         [&](const document::TurretEntry& e) { return e.tool == tool; });
        if (it != s.turret.end()) {
            return *it;
        }
        document::TurretEntry entry;
        entry.tool = tool;
        s.turret.push_back(entry);
        return s.turret.back();
    };
    const document::TurretLoad load = document::turretLoad(*doc_, setup, machine);

    // Two lines per tool: the panel is narrow.
    auto* table = new QWidget;
    table->setObjectName(QStringLiteral("setupTurretTable"));
    auto* grid = new QGridLayout(table);
    grid->setContentsMargins(0, 0, 0, 0);
    grid->setColumnStretch(1, 1);
    grid->setColumnStretch(2, 1);
    int row = 0;
    auto* stationHead = new QLabel(tr("Station"));
    stationHead->setToolTip(
        tr("Evenly spaced round the turret, as many as the machine has. A tool's "
           "number is its station: the program calls it by it (T0303 for 3)."));
    auto* blockHead = new QLabel(tr("Block on it"));
    blockHead->setToolTip(tr(
        "The tooling block on the station, from the library. It is drawn on "
        "the turret, checked for collisions, and its tool sits in it. With none, a tool sits where "
        "the machine's turret has such a tool."));
    auto* toolHead = new QLabel(tr("Tool in it"));
    toolHead->setToolTip(
        tr("The tool on the station. Choosing one gives it the station's number, everywhere in "
           "the project; a tool there before takes the chosen tool's old number. A tool no "
           "operation of this setup uses rides along: (no tool) takes it out."));
    grid->addWidget(stationHead, row, 0);
    grid->addWidget(blockHead, row, 1);
    grid->addWidget(toolHead, row, 2);
    ++row;
    const QString lies = tr("Across the spindle (a shank along X), or along it (a bar or a drill)");
    const QString toSpindle =
        tr("From the turret's centre towards the spindle, in the working position: what the "
           "machine's tool setter would say. It starts from the machine's turret sizes.");
    const QString alongZ = tr("From the turret's centre along Z, in the working position. Negative "
                              "is towards the chuck.");

    // What can go on a station: the project's blocks, then the library's. Only the blocks that
    // fit are offered; one already there stays, said.
    enum BlockSource { NoBlock, ProjectBlock, LibraryBlock, ManageBlocks };
    static constexpr int kSourceRole = Qt::UserRole + 1;
    std::vector<document::FittingPackage> library;
    for (document::FittingPackage& pkg : LatheFittingLibrary::load()) {
        const document::LatheFitting* own = doc_->project().findFitting(pkg.fitting.id);
        if (pkg.fitting.kind == document::LatheFitting::Kind::Block &&
            (!own || *own != pkg.fitting)) {
            library.push_back(std::move(pkg));
        }
    }
    using Fits = std::function<bool(const document::LatheFitting&)>;
    using Chosen = std::function<void(document::Setup&, const std::optional<Uuid>&)>;
    const auto fittingBox = [&](int station, const Fits& fits, const std::optional<Uuid>& current,
                                const QString& none, const QString& misfit,
                                const std::function<QString(int)>& leftOut, const QString& label,
                                const Chosen& chosen) {
        auto* box = new QComboBox;
        box->setToolTip(blockHead->toolTip());
        box->addItem(none);
        box->setItemData(0, static_cast<int>(NoBlock), kSourceRole);
        int notFitting = 0;
        for (const document::LatheFitting& f : doc_->project().fittings) {
            if (f.kind != document::LatheFitting::Kind::Block) {
                continue;
            }
            const bool ok = fits(f);
            notFitting += ok ? 0 : 1;
            if (ok || (current && *current == f.id)) {
                box->addItem(ok ? qs(f.name) : misfit.arg(qs(f.name)), qs(f.id.toString()));
                box->setItemData(box->count() - 1, static_cast<int>(ProjectBlock), kSourceRole);
            }
        }
        bool fromLibrary = false;
        for (const document::FittingPackage& pkg : library) {
            if (!fits(pkg.fitting)) {
                ++notFitting;
                continue;
            }
            if (!std::exchange(fromLibrary, true)) {
                // Where the rest come from, said once: each name comes first.
                addListHeading(box, tr("In your library"));
            }
            box->addItem(qs(pkg.fitting.name), qs(pkg.fitting.id.toString()));
            box->setItemData(box->count() - 1, static_cast<int>(LibraryBlock), kSourceRole);
            box->setItemData(box->count() - 1,
                             tr("%1\nIn your library: copied into the project when chosen")
                                 .arg(qs(pkg.fitting.name)),
                             Qt::ToolTipRole);
        }
        if (notFitting > 0) {
            box->insertSeparator(box->count());
            box->addItem(tr("(%n more that do not fit)", nullptr, notFitting));
            box->setItemData(box->count() - 1, static_cast<int>(NoBlock), kSourceRole);
            box->setItemData(box->count() - 1, leftOut(notFitting), Qt::ToolTipRole);
            box->setItemData(box->count() - 1, 0, Qt::UserRole - 1); // disabled
        }
        box->insertSeparator(box->count());
        box->addItem(tr("Manage blocks..."));
        box->setItemData(box->count() - 1, static_cast<int>(ManageBlocks), kSourceRole);
        for (int i = 0; current && i < box->count(); ++i) {
            if (box->itemData(i, kSourceRole).toInt() == ProjectBlock &&
                box->itemData(i).toString() == qs(current->toString())) {
                box->setCurrentIndex(i);
            }
        }
        // Its row is the one chosen, for the turret preview (connected first: called first).
        connect(box, &QComboBox::activated, this,
                [this, station] { selectTurretStation(station); });
        connect(box, &QComboBox::activated, this, [this, edit, box, library, label, chosen](int i) {
            const auto source = static_cast<BlockSource>(box->itemData(i, kSourceRole).toInt());
            const auto blockId = Uuid::parse(ss(box->itemData(i).toString()));
            if (source == ManageBlocks) {
                // Once the list is done with its signal: the library rebuilds this page.
                QTimer::singleShot(0, this, [this] { emit manageFittingsRequested(); });
                return;
            }
            if (source == LibraryBlock && blockId) {
                const auto it = std::find_if(library.begin(), library.end(),
                                             [&](const document::FittingPackage& pkg) {
                                                 return pkg.fitting.id == *blockId;
                                             });
                try {
                    doc_->useFitting(*it);
                } catch (const Error& e) {
                    QMessageBox::warning(this, tr("Turret"), QString::fromUtf8(e.what()));
                    return;
                }
            }
            edit(ss(label), [&](document::Setup& st) {
                chosen(st, source != NoBlock ? blockId : std::nullopt);
            });
        });
        return box;
    };
    const auto entryAt = [](document::Setup& st, int number) {
        return std::find_if(st.turretBlocks.begin(), st.turretBlocks.end(),
                            [&](const document::StationBlock& b) { return b.station == number; });
    };
    const auto blockBox = [&](int number) {
        QStringList takes;
        for (const std::string& name : machine.lathe->turret.blockMountsAt(number)) {
            takes << qs(name);
        }
        const auto here =
            std::find_if(setup.turretBlocks.begin(), setup.turretBlocks.end(),
                         [&](const document::StationBlock& b) { return b.station == number; });
        auto* box = fittingBox(
            number,
            [&machine, number](const document::LatheFitting& f) {
                return !document::mountMismatch(f, machine, number);
            },
            here != setup.turretBlocks.end() ? std::optional<Uuid>(here->block) : std::nullopt,
            tr("(no block)"), tr("%1 (does not fit this station)"),
            [takes](int n) {
                return tr("%n more blocks that the station does not take: it takes %1", nullptr, n)
                    .arg(takes.join(QStringLiteral(", ")));
            },
            tr("Change a station's block"),
            [entryAt, number](document::Setup& st, const std::optional<Uuid>& blockId) {
                const auto it = entryAt(st, number);
                if (it != st.turretBlocks.end()) {
                    st.turretBlocks.erase(it);
                }
                if (blockId) {
                    st.turretBlocks.push_back({number, *blockId});
                }
            });
        box->setObjectName(QStringLiteral("setupStationBlock%1").arg(number));
        return box;
    };
    // A station's tool: any of the project's tools. Choosing one gives it the station's number;
    // a tool there before takes its old one.
    std::set<Uuid> usedTools;
    for (const document::TurretStation& s : load.stations) {
        if (s.used) {
            usedTools.insert(s.tool);
        }
    }
    const auto toolBox = [&](int number) {
        auto* box = new QComboBox;
        box->setObjectName(QStringLiteral("setupStationTool%1").arg(number));
        box->setToolTip(toolHead->toolTip());
        const auto here =
            std::find_if(load.stations.begin(), load.stations.end(),
                         [&](const document::TurretStation& s) { return s.station == number; });
        const bool holds = here != load.stations.end();
        box->addItem(tr("(no tool)"));
        if (holds && here->used) {
            // An operation of the setup uses it: it goes to another station, not out.
            box->setItemData(0, 0, Qt::UserRole - 1); // disabled
        }
        for (const tooling::Tool& t : doc_->project().tools.tools) {
            const bool there = holds && here->tool == t.id;
            box->addItem(there && !here->used
                             ? tr("T%1  %2 (not used here)").arg(t.number).arg(qs(t.name))
                             : QStringLiteral("T%1  %2").arg(t.number).arg(qs(t.name)),
                         qs(t.id.toString()));
            if (there) {
                box->setCurrentIndex(box->count() - 1);
            }
        }
        const std::optional<Uuid> current = holds ? std::optional<Uuid>(here->tool) : std::nullopt;
        connect(box, &QComboBox::activated, this,
                [this, id, edit, entryOf, box, number, current, usedTools](int i) {
                    selectTurretStation(number);
                    const auto chosen = Uuid::parse(ss(box->itemData(i).toString()));
                    if (!chosen) {
                        if (current && !usedTools.contains(*current)) {
                            edit("Take a tool out of the turret", [&](document::Setup& st) {
                                std::erase_if(st.turret, [&](const document::TurretEntry& e) {
                                    return e.tool == *current;
                                });
                            });
                        }
                        return;
                    }
                    if (current && *current == *chosen) {
                        return;
                    }
                    commit("Put a tool on a station", [&](Project& pr) {
                        tooling::Tool* tool = pr.tools.findTool(*chosen);
                        if (!tool) {
                            return;
                        }
                        const int from = tool->number;
                        for (tooling::Tool& other : pr.tools.tools) {
                            if (other.id != *chosen && other.number == number) {
                                other.number = from;
                            }
                        }
                        tool->number = number;
                        if (!usedTools.contains(*chosen)) {
                            if (Setup* s = pr.findSetup(id)) {
                                (void)entryOf(*s, *chosen); // rides along
                            }
                        }
                    });
                });
        return box;
    };
    const int lastStation =
        std::max(machine.toolCapacity, load.stations.empty() ? 0 : load.stations.back().station);
    auto* rows = new QButtonGroup(table);
    rows->setExclusive(true);
    for (int number = 1; number <= lastStation; ++number) {
        // The station's number chooses its row: the turret preview shows it.
        auto* label = new QToolButton;
        label->setObjectName(QStringLiteral("setupStationRow%1").arg(number));
        label->setText(QString::number(number));
        label->setCheckable(true);
        label->setAutoRaise(true);
        label->setChecked(number == turretStation_);
        label->setToolTip(stationHead->toolTip() + QStringLiteral("\n\n") +
                          tr("Click to see the station in the turret preview."));
        rows->addButton(label, number);
        connect(label, &QToolButton::clicked, this,
                [this, number] { selectTurretStation(number); });
        grid->addWidget(label, row, 0);
        if (number <= machine.toolCapacity) {
            grid->addWidget(blockBox(number), row, 1);
        }
        grid->addWidget(toolBox(number), row, 2);
        ++row;
        for (const document::TurretStation& s : load.stations) {
            if (s.station != number) {
                continue;
            }
            const Uuid tool = s.tool;
            if (s.block) {
                // Through its block: how far it sticks out, and where that leaves the tip.
                // The tool's stickout: from the block's tool mount (on a holder, its mouth: the
                // front face the tool comes out of) to the tip, along the tool.
                const QString stickoutTip =
                    tr("How far the tool's tip stands out beyond the block's tool mount, along "
                       "the tool: for a turning holder, from the face its shank comes out of to "
                       "the tip. The block holds the tool %1 the spindle.")
                        .arg(s.across ? tr("across") : tr("along"));
                auto* stickoutLabel = new QLabel(tr("Stickout"));
                stickoutLabel->setToolTip(stickoutTip);
                grid->addWidget(stickoutLabel, row, 0);
                auto* overhang = new LengthSpinBox(s.overhang, unit_, 0.0, 5000.0);
                overhang->setObjectName(QStringLiteral("setupTurretOverhang"));
                overhang->setToolTip(stickoutTip);
                connect(overhang, &LengthSpinBox::mmChanged, this, [edit, entryOf, tool](double v) {
                    edit("Change a tool's stickout",
                         [&](document::Setup& st) { entryOf(st, tool).overhang = v; });
                });
                grid->addWidget(overhang, row, 1);
                auto* tip = new QLabel(
                    tr("Tip %1, %2")
                        .arg(formatLength(s.tip.x(), unit_), formatLength(s.tip.y(), unit_)));
                tip->setToolTip(tr("From the turret's centre: towards the spindle, and along Z. "
                                   "From the block."));
                grid->addWidget(tip, row, 2);
            } else {
                auto* liesBox = new QComboBox;
                liesBox->addItem(tr("Across"));
                liesBox->addItem(tr("Along"));
                liesBox->setCurrentIndex(s.across ? 0 : 1);
                liesBox->setToolTip(lies);
                connect(liesBox, &QComboBox::activated, this, [edit, entryOf, tool](int index) {
                    edit("Change how a tool lies in the turret",
                         [&](document::Setup& st) { entryOf(st, tool).across = index == 0; });
                });
                grid->addWidget(liesBox, row, 0);
                auto tipBox = [&](int column, double value, bool towardsSpindle,
                                  const QString& tip) {
                    auto* box = new LengthSpinBox(value, unit_, -5000.0, 5000.0);
                    box->setToolTip(tip);
                    const Vec2 from = s.tip;
                    connect(box, &LengthSpinBox::mmChanged, this,
                            [edit, entryOf, tool, from, towardsSpindle](double v) {
                                edit("Change a tool's tip in the turret", [&](document::Setup& st) {
                                    document::TurretEntry& e = entryOf(st, tool);
                                    Vec2 now = e.tip.value_or(from);
                                    (towardsSpindle ? now.x() : now.y()) = v;
                                    e.tip = now;
                                });
                            });
                    grid->addWidget(box, row, column);
                };
                tipBox(1, s.tip.x(), true, toSpindle);
                tipBox(2, s.tip.y(), false, alongZ);
            }
            ++row;
            if (s.used && (s.tipSet || s.acrossSet)) {
                auto* reset = new QPushButton(tr("Reset"));
                reset->setObjectName(QStringLiteral("setupTurretReset"));
                reset->setToolTip(tr("Back to where the machine's turret or its block has such a "
                                     "tool"));
                connect(reset, &QPushButton::clicked, this, [edit, tool] {
                    edit("Change a tool's place in the turret", [&](document::Setup& st) {
                        std::erase_if(st.turret, [&](const document::TurretEntry& e) {
                            return e.tool == tool;
                        });
                    });
                });
                grid->addWidget(reset, row, 2);
                ++row;
            }
        }
    }
    form->addRow(table);
    for (const std::string& problem : load.problems) {
        auto* warn = new QLabel(qs(problem));
        warn->setWordWrap(true);
        warn->setStyleSheet(QStringLiteral("color: #c0392b;"));
        form->addRow(QString(), warn);
    }
    // A tool that rides along: in the turret, without an operation here.
    auto* add = new QComboBox;
    add->setObjectName(QStringLiteral("setupTurretAdd"));
    add->addItem(tr("Put another tool in the turret..."));
    for (const tooling::Tool& t : doc_->project().tools.tools) {
        const bool inTurret =
            std::any_of(load.stations.begin(), load.stations.end(),
                        [&](const document::TurretStation& s) { return s.tool == t.id; });
        if (!inTurret) {
            add->addItem(QStringLiteral("T%1  %2").arg(t.number).arg(qs(t.name)),
                         qs(t.id.toString()));
        }
    }
    add->setToolTip(tr("A tool that stays in the turret while this setup runs although none of "
                       "its operations uses it, such as a long boring bar or a drill: it is drawn "
                       "on the turret and checked for collisions as the turret moves and "
                       "indexes. Take it out again with its Take out button."));
    connect(add, &QComboBox::activated, this, [edit, entryOf, add](int index) {
        if (index > 0) {
            const auto tool = Uuid::parse(ss(add->itemData(index).toString()));
            if (tool) {
                edit("Add a tool to the turret",
                     [&](document::Setup& st) { (void)entryOf(st, *tool); });
            }
        }
    });
    form->addRow(QString(), add);

    if (machine.lathe->subSpindle) {
        auto* spindle = new QComboBox;
        spindle->setObjectName(QStringLiteral("setupSpindle"));
        spindle->addItem(tr("Main spindle"));
        spindle->addItem(tr("Sub-spindle"));
        spindle->setCurrentIndex(document::onSubSpindle(setup) ? 1 : 0);
        spindle->setToolTip(
            tr("Which spindle holds the work. On the sub-spindle the setup's Z runs "
               "towards the main spindle: set its WCS turned round. Simulated and "
               "checked; no post writes it yet, so posting it is refused."));
        connect(spindle, &QComboBox::activated, this, [edit](int index) {
            edit("Change spindle", [&](document::Setup& st) {
                document::SetupPlacement p = st.placement.value_or(document::SetupPlacement{});
                p.carrier = index == 1 ? "C2" : "C";
                st.placement = p;
            });
        });
        form->addRow(tr("Spindle"), spindle);
    }
    auto* along = new LengthSpinBox(setup.placement ? setup.placement->offset.z() : 0.0, unit_,
                                    -5000.0, 5000.0);
    along->setObjectName(QStringLiteral("setupWorkAlong"));
    along->setToolTip(tr("Moves the work along the spindle from where its chuck holds it: its held "
                         "end at the nose with a chuck of the setup's own, or in the machine's "
                         "chuck by its grip"));
    connect(along, &LengthSpinBox::mmChanged, this, [edit](double v) {
        edit("Move the work along the spindle", [&](document::Setup& st) {
            document::SetupPlacement p = st.placement.value_or(document::SetupPlacement{});
            if (p.carrier.empty()) {
                p.carrier = "C";
            }
            p.offset.z() = v;
            st.placement = p;
        });
    });
    form->addRow(tr("Work along the spindle"), along);
}

void PropertyPanel::buildPost(QFormLayout* form, const Setup& setup,
                              const machine::Machine& machine) {
    const Project& project = doc_->project();
    const Uuid setupId = setup.id;
    QStringList problems;

    // The choices (PostChoices): the machine's default, the posts for its kinematics, a post
    // file (copied into the project).
    constexpr int kAddFile = -1;
    const std::vector<PostEntry> entries = postEntries(project, setup, machine, &problems);
    auto* box = new QComboBox;
    box->setToolTip(tr("The post processor that writes the setup's program for its machine's "
                       "control (Setup > Post Process shows it with its options)"));
    for (std::size_t i = 0; i < entries.size(); ++i) {
        if (i == 0 || entries[i].kind != entries[i - 1].kind) {
            if (const QString heading = postKindHeading(entries[i].kind); !heading.isEmpty()) {
                addListHeading(box, heading);
            } else if (i > 0) {
                box->insertSeparator(box->count());
            }
        }
        box->addItem(entries[i].label, static_cast<int>(i));
    }
    box->insertSeparator(box->count());
    box->addItem(tr("Add a post file..."), kAddFile);
    const int current = box->findData(currentPostEntry(entries, setup));
    box->setCurrentIndex(current);

    // The post the setup uses now: option values belong to it, and are dropped with it.
    std::string currentId;
    try {
        currentId = document::postFor(project, setup, machine).manifest().id;
    } catch (const Error&) {
        // shown below with the options
    }
    // Stores a choice (a project copy of a library post or file first, replacing one with the
    // same id).
    auto choose = [this, setupId,
                   currentId](std::optional<document::PostChoice> choice, const std::string& postId,
                              std::optional<document::ProjectPost> copy, const std::string& label) {
        commit(label, [&](Project& p) {
            if (copy) {
                addProjectPost(p, *copy);
            }
            if (Setup* s = p.findSetup(setupId)) {
                s->post = choice;
                if (postId != currentId) {
                    s->postOptions = Json::object();
                }
            }
        });
    };
    connect(box, &QComboBox::currentIndexChanged, this,
            [this, box, current, entries, choose](int row) {
                const int index = box->itemData(row).toInt();
                if (index != kAddFile) {
                    const PostEntry& entry = entries.at(static_cast<std::size_t>(index));
                    std::optional<document::ProjectPost> copy;
                    if (entry.library) {
                        copy = projectCopy(*entry.library);
                    }
                    choose(postChoiceOf(entry), entry.id, copy, "Change post");
                    return;
                }
                // A post file: checked by loading it, then copied into the project.
                const QSignalBlocker block(box);
                box->setCurrentIndex(current);
                const QString path = QFileDialog::getOpenFileName(
                    this, tr("Add a post file"), QString(), tr("Post processors (*.js)"));
                if (path.isEmpty()) {
                    return;
                }
                try {
                    const std::filesystem::path file(path.toStdU16String());
                    const std::u8string name = file.filename().u8string();
                    document::ProjectPost copy{std::string(name.begin(), name.end()),
                                               readFileBytes(file)};
                    CancelToken cancel;
                    const post::PostScript loaded =
                        post::PostScript::load(copy.source, copy.file, cancel);
                    choose(document::PostChoice{loaded.manifest().id, true}, loaded.manifest().id,
                           copy, "Add post " + loaded.manifest().name);
                } catch (const Error& e) {
                    QMessageBox::warning(this, tr("Add a post file"),
                                         tr("The post was not added:\n\n%1").arg(qs(e.what())));
                }
            });
    auto* configure = new QPushButton(tr("Configure..."));
    configure->setToolTip(tr("Choose the post and set its options for this setup"));
    connect(configure, &QPushButton::clicked, this,
            [this, setupId] { emit postSettingsRequested(setupId); });
    configure->setProperty("keepWidth", true); // compactPage: the box shrinks, not the button
    auto* postRow = new QHBoxLayout;
    postRow->addWidget(box, 1);
    postRow->addWidget(configure);
    form->addRow(tr("Post"), postRow);

    // A project copy the library has a newer version of.
    if (setup.post && setup.post->inProject) {
        for (const auto& entry : PostLibrary::load()) {
            if (entry.post.manifest().id != setup.post->id) {
                continue;
            }
            const document::ProjectPost copy = projectCopy(entry);
            if (std::ranges::find(project.posts, copy) == project.posts.end()) {
                auto* update = new QPushButton(tr("Update from library"));
                update->setToolTip(tr("Replace the project's copy of the post with the one in the "
                                      "post library (%1)")
                                       .arg(qs(entry.file.filename().string())));
                const std::string id = entry.post.manifest().id;
                connect(update, &QPushButton::clicked, this, [this, copy, id] {
                    commit("Update post " + id, [&](Project& p) { addProjectPost(p, copy); });
                });
                form->addRow(QString(), update);
            }
            break;
        }
    }

    // The post's options are set with Configure (the Post Process dialog); a post that cannot be
    // resolved says why here.
    if (currentId.empty()) {
        try {
            (void)document::postFor(project, setup, machine);
        } catch (const Error& e) {
            problems << qs(e.what());
        }
    }
    if (!problems.isEmpty()) {
        auto* warn = new QLabel(problems.join(QLatin1Char('\n')));
        warn->setWordWrap(true);
        warn->setStyleSheet(QStringLiteral("color: #c0392b;"));
        form->addRow(QString(), warn);
    }
}

void PropertyPanel::buildPostOptions(QFormLayout* form, const document::Operation& op,
                                     const Setup& setup) {
    const Project& project = doc_->project();
    std::optional<post::PostScript> chosen;
    try {
        chosen = document::postFor(project, setup, document::machineFor(project, setup));
    } catch (const Error&) {
        return; // the setup page says why
    }
    const std::string opId = op.id.toString();
    const Json none = Json::object();
    const Json& operations =
        setup.postOptions.is_object() && setup.postOptions.contains("operations")
            ? setup.postOptions.at("operations")
            : none;
    const Json& values =
        operations.is_object() && operations.contains(opId) ? operations.at(opId) : none;
    bool heading = false;
    for (const post::PostOption& option : chosen->options()) {
        if (option.scope != post::PostOption::Scope::Operation) {
            continue;
        }
        if (!heading) {
            form->addRow(new QLabel(tr("<b>%1 options</b>").arg(qs(chosen->manifest().name))));
            heading = true;
        }
        const std::string optionId = option.id;
        const Uuid setupId = setup.id;
        form->addRow(qs(option.label),
                     postOptionEditor(option, postOptionValue(values, option),
                                      [this, setupId, opId, optionId](const Json& v) {
                                          commit("Change post option " + optionId, [&](Project& p) {
                                              if (Setup* s = p.findSetup(setupId)) {
                                                  s->postOptions["operations"][opId][optionId] = v;
                                              }
                                          });
                                      }));
    }
}

void PropertyPanel::buildOperation(const document::Operation& op, const Setup& setup) {
    const Project& p = doc_->project();
    const Uuid id = op.id;
    auto edit = [this, id](const std::string& label,
                           std::function<void(document::Operation&)> change) {
        commit(label, [&](Project& pr) {
            if (document::Operation* o = pr.findOperation(id)) {
                change(*o);
            }
        });
    };

    QFormLayout* form = newForm(tr("Operation in %1").arg(qs(setup.name)));
    auto* name = new QLineEdit(qs(op.name));
    connect(name, &QLineEdit::editingFinished, this, [edit, name] {
        edit("Rename operation", [&](document::Operation& o) { o.name = ss(name->text()); });
    });
    name->setToolTip(tr("The operation's name in the project tree; the posted program names "
                        "the operation with it in a comment"));
    form->addRow(tr("Name"), name);

    auto* comment = new QLineEdit(qs(op.comment));
    comment->setToolTip(tr("A note of your own about this operation, kept with the project"));
    connect(comment, &QLineEdit::editingFinished, this, [edit, comment] {
        edit("Edit comment", [&](document::Operation& o) { o.comment = ss(comment->text()); });
    });
    form->addRow(tr("Comment"), comment);

    auto* type = new QComboBox;
    for (const auto& [typeId, label] : operationTypes()) {
        type->addItem(label, typeId);
        if (const auto* s = strategies::findStrategy(ss(typeId))) {
            type->setItemData(type->count() - 1,
                              formatToolTip(trStrategy(s->description), trStrategy(s->label)),
                              Qt::ToolTipRole);
        }
    }
    if (type->findData(qs(op.type)) < 0) {
        type->addItem(qs(op.type), qs(op.type));
    }
    type->setCurrentIndex(type->findData(qs(op.type)));
    connect(type, &QComboBox::currentIndexChanged, this, [edit, type] {
        edit("Change operation type",
             [&](document::Operation& o) { o.type = ss(type->currentData().toString()); });
    });
    if (const auto* s = strategies::findStrategy(op.type)) {
        type->setToolTip(formatToolTip(trStrategy(s->description), trStrategy(s->label)));
    }
    form->addRow(tr("Type"), type);

    auto* enabled = new QCheckBox(tr("Enabled"));
    enabled->setChecked(op.enabled);
    enabled->setToolTip(tr("Disabled operations generate no toolpath and are left out of the "
                           "simulation and the posted program"));
    connect(enabled, &QCheckBox::toggled, this, [edit](bool on) {
        edit(on ? "Enable operation" : "Disable operation",
             [&](document::Operation& o) { o.enabled = on; });
    });
    form->addRow(QString(), enabled);

    // The project's tools, then the tools of the user's libraries (default first); picking a
    // library tool copies it, with its holder, into the project.
    constexpr int kLibraryRole = Qt::UserRole + 1;
    // Turning operations list turning tools, the others milling tools; the operation's own tool
    // stays listed whatever it is (an older project's choice is not hidden).
    const bool turningOperation = op.type.rfind("turn_", 0) == 0;
    // Probing operations list probes, and only they do (DL-0090); a bar pull a puller, and only
    // it (DL-0091).
    const bool probingOperation = strategies::isProbingStrategy(op.type);
    const bool pull = op.type == "bar_pull";
    const auto listed = [&](const tooling::Tool& t) {
        const bool puller = t.type == tooling::ToolType::BarPuller;
        return (pull ? puller
                     : !puller && tooling::isTurningTool(t.type) == turningOperation &&
                           (t.type == tooling::ToolType::Probe) == probingOperation) ||
               (op.toolId && t.id == *op.toolId);
    };
    auto* tool = new QComboBox;
    tool->addItem(tr("(none)"));
    for (const auto& t : p.tools.tools) {
        if (listed(t)) {
            tool->addItem(QStringLiteral("T%1  %2").arg(t.number).arg(qs(t.name)),
                          qs(t.id.toString()));
        }
    }
    const auto libraries = ToolLibraries::load();
    for (const auto& entry : libraries) {
        bool header = false;
        for (const auto& t : entry.library.tools) {
            if (p.tools.findTool(t.id) || !listed(t)) {
                continue; // already copied, or for the other kind of operation
            }
            if (!header) {
                tool->insertSeparator(tool->count());
                tool->addItem(tr("%1 library").arg(qs(entry.library.name)));
                auto* model = qobject_cast<QStandardItemModel*>(tool->model());
                if (model) {
                    model->item(tool->count() - 1)->setEnabled(false);
                }
                header = true;
            }
            tool->addItem(QStringLiteral("   T%1  %2").arg(t.number).arg(qs(t.name)),
                          qs(t.id.toString()));
            tool->setItemData(tool->count() - 1, QString::fromStdU16String(entry.file.u16string()),
                              kLibraryRole);
        }
    }
    tool->setCurrentIndex(op.toolId ? std::max(0, tool->findData(qs(op.toolId->toString()))) : 0);
    tool->setToolTip(tr("The tool that cuts this operation. Tools from a library are copied "
                        "into the project when chosen"));
    connect(tool, &QComboBox::currentIndexChanged, this, [this, id, tool, libraries] {
        const auto tid = Uuid::parse(ss(tool->currentData().toString()));
        const QString file = tool->currentData(kLibraryRole).toString();
        const tooling::Tool* fromLibrary = nullptr;
        const tooling::Holder* holder = nullptr;
        if (!file.isEmpty() && tid) {
            for (const auto& entry : libraries) {
                if (QString::fromStdU16String(entry.file.u16string()) == file) {
                    fromLibrary = entry.library.findTool(*tid);
                    holder = fromLibrary && fromLibrary->holderId
                                 ? entry.library.findHolder(*fromLibrary->holderId)
                                 : nullptr;
                }
            }
        }
        commit(fromLibrary ? "Use library tool " + fromLibrary->name : std::string("Change tool"),
               [&](Project& pr) {
                   if (fromLibrary && !pr.tools.findTool(fromLibrary->id)) {
                       tooling::Tool copy = *fromLibrary;
                       if (const auto* clash = pr.tools.findToolByNumber(copy.number);
                           clash && clash->id != copy.id) {
                           copy.number = pr.tools.nextFreeNumber();
                       }
                       if (holder && !pr.tools.findHolder(holder->id)) {
                           pr.tools.holders.push_back(*holder);
                       }
                       pr.tools.tools.push_back(std::move(copy));
                   }
                   if (document::Operation* o = pr.findOperation(id)) {
                       o->toolId = tid;
                       o->presetId.reset();
                       // The tool's first preset until another is chosen.
                       if (const tooling::Tool* t = tid ? pr.tools.findTool(*tid) : nullptr;
                           t && !t->presets.empty()) {
                           o->presetId = t->presets.front().id;
                       }
                   }
               });
    });
    // A transfer to the sub-spindle has no tool (ADR-0007).
    const bool toolless = op.type == "spindle_transfer";
    if (toolless) {
        tool->deleteLater();
    } else {
        form->addRow(tr("Tool"), tool);
    }

    auto* preset = new QComboBox;
    preset->setPlaceholderText(tr("Choose a preset"));
    const tooling::Tool* selectedTool = op.toolId ? p.tools.findTool(*op.toolId) : nullptr;
    // A probe cuts nothing: its feeds are the operation's parameters, below.
    if (selectedTool && !probingOperation) {
        for (const auto& pr : selectedTool->presets) {
            preset->addItem(qs(pr.name), qs(pr.id.toString()));
        }
    }
    preset->setCurrentIndex(op.presetId ? preset->findData(qs(op.presetId->toString())) : -1);
    preset->setEnabled(selectedTool != nullptr);
    preset->setToolTip(tr("The tool's cutting data for a material or process: speeds and feeds, "
                          "and the depths of cut every strategy steps by. The values below "
                          "change it for this operation only"));
    connect(preset, &QComboBox::currentIndexChanged, this, [edit, preset] {
        const auto pid = Uuid::parse(ss(preset->currentData().toString()));
        edit("Change cutting preset", [&](document::Operation& o) { o.presetId = pid; });
    });
    if (toolless) {
        preset->deleteLater();
        buildTransferPickOff(form, op, setup);
    } else if (probingOperation) {
        delete preset; // never shown
    } else {
        form->addRow(tr("Cutting preset"), preset);
    }
    if (selectedTool && op.presetId && !probingOperation) {
        if (const auto* toolPreset = selectedTool->findPreset(*op.presetId)) {
            buildPresetOverrides(form, op, *toolPreset, *selectedTool);
        }
    }

    const strategies::StrategyInfo* info = strategies::findStrategy(op.type);
    // A lathe's spindle drills on its axis, with nothing to select; a live tool goes to holes.
    const machine::Machine* onMachine = document::findMachineFor(p, setup);
    const bool onLathe = onMachine && onMachine->isLathe();
    const bool liveTool = onLathe && document::liveDrive(op);
    if (info && info->geometry != strategies::GeometryUse::None &&
        !(document::isHoleOperation(op.type) && onLathe && !liveTool)) {
        buildGeometry(form, op, info->geometry, setup);
    }
    if (op.type == "contour2d" || op.type == "trace2d") {
        buildStartPoint(form, op);
    }
    if (!onLathe || liveTool) {
        buildOrientation(form, op, setup);
    }
    buildHeights(form, op, setup);
    buildPostOptions(form, op, setup);

    // Strategy parameters, generated from the strategy's parameter descriptors: those that
    // matter with the others' values (a ramp angle only for ramp entries, say).
    if (info) {
        auto* heading = new QLabel(tr("<b>%1 parameters</b>").arg(trStrategy(info->label)));
        form->addRow(heading);
        for (const strategies::ParamDef& def : info->params) {
            if (!info->applies(def, op.parameters)) {
                continue;
            }
            if ((def.key == "drive" || def.key == "live_axes" || def.key == "live_c") && !onLathe) {
                continue; // a mill's tool always turns, and it has its axes
            }
            const std::string key = def.key;
            const Json* current = op.parameters.contains(key) ? &op.parameters.at(key) : nullptr;
            QWidget* editor = createParamEditor(
                def, current, unit_,
                [edit, key](const Json& v) {
                    edit("Change " + key, [&](document::Operation& o) { o.parameters[key] = v; });
                },
                selectedTool);
            editor->setObjectName(QStringLiteral("param_") + QString::fromStdString(key));
            form->addRow(trStrategy(def.label), editor);
        }
    }

    statusLabel_ = new QLabel;
    statusLabel_->setObjectName(QStringLiteral("toolpathStatus"));
    statusLabel_->setWordWrap(true);
    statusLabel_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    statusLabel_->setToolTip(tr("The toolpath's state: moves, feed length and estimated time, "
                                "or why it could not be generated"));
    form->addRow(tr("Toolpath"), statusLabel_);
    statusOperation_ = id;
    refreshToolpathStatus();
}

void PropertyPanel::buildTransferPickOff(QFormLayout* form, const document::Operation& op,
                                         const Setup& setup) {
    // Where Z2 takes the part, and by how much of it: what the grip and the setups give.
    const Project& p = doc_->project();
    QString text;
    try {
        const machine::Machine& machine = document::machineFor(p, setup);
        const Setup* target = document::transferTarget(p, setup);
        if (!machine.lathe || !machine.lathe->subSpindle) {
            text = tr("The machine has no sub-spindle");
        } else if (!target) {
            text = tr("No setup on the sub-spindle takes its stock from this one");
        } else {
            CancelToken cancel;
            const double grip = document::transferGrip(op, machine);
            const auto z2 = document::pickOffZ2(*doc_, setup, *target, machine, cancel);
            const QString gripText =
                tr("grip %1%2")
                    .arg(formatLength(grip, unit_))
                    .arg(strategies::ParamReader(*strategies::findStrategy(op.type), op.parameters)
                                     .number("grip") > 0.0
                             ? QString()
                             : tr(", the machine's"));
            text = z2 ? tr("Z2 %1 (%2), for setup '%3'")
                            .arg(formatLength(*z2, unit_), gripText, qs(target->name))
                      : tr("The part does not line up with setup '%1'").arg(qs(target->name));
        }
    } catch (const Error& e) {
        text = qs(e.what());
    }
    auto* label = new QLabel(text);
    label->setWordWrap(true);
    label->setObjectName(QStringLiteral("transferPickOff"));
    label->setToolTip(tr("Where the sub-spindle takes the part: Z2 in the machine's own, from the "
                         "grip and where the two setups put the part"));
    form->addRow(tr("Takes the part at"), label);
}

void PropertyPanel::refreshToolpathStatus() {
    if (statusLabel_ && toolpathStatus_) {
        statusLabel_->setText(toolpathStatus_(statusOperation_));
    }
}

} // namespace ocam::gui
