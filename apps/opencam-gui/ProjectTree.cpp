// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ProjectTree.hpp"

#include <ocam/document/PostSetup.hpp>
#include <ocam/tooling/Tool.hpp>

#include <QDropEvent>
#include <QHeaderView>
#include <QPainter>
#include <QPainterPath>
#include <QPixmap>
#include <QSet>

namespace ocam::gui {
namespace {

constexpr int kKindRole = Qt::UserRole;
constexpr int kIdRole = Qt::UserRole + 1;
constexpr int kGroupRole = Qt::UserRole + 2;

QString qs(const std::string& s) {
    return QString::fromStdString(s);
}

/// A short length in a tool's unit: 0.5" or 12.7mm.
QString shortLength(double mm, LengthUnit unit) {
    return unit == LengthUnit::Inch ? QString::number(mm / 25.4, 'g', 4) + QStringLiteral("\"")
                                    : QString::number(mm, 'g', 4) + QStringLiteral("mm");
}

/// Ø0.5", R0.01", 4FL for milling tools; R0.8mm for inserts.
QString toolDetail(const tooling::Tool& t) {
    QStringList parts;
    if (tooling::isTurningTool(t.type)) {
        if (t.insert.noseRadius > 0.0) {
            parts << QStringLiteral("R") + shortLength(t.insert.noseRadius, t.unit);
        }
    } else {
        parts << QStringLiteral("Ø") + shortLength(t.mill.diameter, t.unit);
        if (t.mill.cornerRadius > 0.0) {
            parts << QStringLiteral("R") + shortLength(t.mill.cornerRadius, t.unit);
        }
        if (t.mill.flutes > 0) {
            parts << QStringLiteral("%1FL").arg(t.mill.flutes);
        }
    }
    return parts.join(QStringLiteral(", "));
}

QString itemKey(const QTreeWidgetItem* item) {
    const QString group = item->data(0, kGroupRole).toString();
    return group.isEmpty() ? item->data(0, kIdRole).toString() : group;
}

} // namespace

ProjectTree::ProjectTree(QWidget* parent) : QTreeWidget(parent) {
    setHeaderLabels({tr("Item"), tr("Details")});
    header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    setSelectionMode(QAbstractItemView::SingleSelection);
    // Operations drag between operations or onto a setup (its end); the item flags set in
    // addItem() and rebuild() decide what can be dragged and where it can go.
    setDragDropMode(QAbstractItemView::InternalMove);
    setDefaultDropAction(Qt::MoveAction);
    setDropIndicatorShown(true);
    invisibleRootItem()->setFlags(Qt::ItemIsEnabled);
    setContextMenuPolicy(Qt::CustomContextMenu);
    connect(this, &QWidget::customContextMenuRequested, this, [this](const QPoint& pos) {
        QTreeWidgetItem* item = itemAt(pos);
        if (!item) {
            return;
        }
        const QString group = item->data(0, kGroupRole).toString();
        if (group.isEmpty()) {
            setCurrentItem(item);
        }
        emit contextMenuRequested(group.isEmpty() ? selection() : Selection{}, group,
                                  viewport()->mapToGlobal(pos));
    });
    connect(
        this, &QTreeWidget::currentItemChanged, this,
        [this](QTreeWidgetItem*, QTreeWidgetItem*) { emit currentSelectionChanged(selection()); });
    spin_ = new QTimer(this);
    spin_->setInterval(90);
    connect(spin_, &QTimer::timeout, this, [this] {
        ++spinStep_;
        refreshStates();
    });
}

void ProjectTree::setOperationState(std::function<OperationState(const Uuid&)> state) {
    state_ = std::move(state);
}

QIcon ProjectTree::stateIcon(OperationState state) const {
    constexpr int kSize = 16;
    QPixmap pixmap(kSize, kSize);
    pixmap.fill(Qt::transparent);
    QPainter p(&pixmap);
    p.setRenderHint(QPainter::Antialiasing);
    const QRectF box(1.5, 1.5, kSize - 3.0, kSize - 3.0);
    switch (state) {
    case OperationState::Ready:
        p.setPen(Qt::NoPen);
        p.setBrush(QColor(60, 170, 90));
        p.drawEllipse(box);
        p.setPen(QPen(Qt::white, 1.8, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        p.drawPolyline(QPolygonF({QPointF(4.5, 8.2), QPointF(7.0, 10.8), QPointF(11.5, 5.2)}));
        break;
    case OperationState::Warning: {
        QPainterPath triangle;
        triangle.moveTo(8.0, 1.5);
        triangle.lineTo(14.8, 14.0);
        triangle.lineTo(1.2, 14.0);
        triangle.closeSubpath();
        p.setPen(Qt::NoPen);
        p.setBrush(QColor(240, 170, 40));
        p.drawPath(triangle);
        p.setPen(QPen(QColor(40, 30, 10), 1.8, Qt::SolidLine, Qt::RoundCap));
        p.drawLine(QPointF(8.0, 5.8), QPointF(8.0, 9.6));
        p.drawPoint(QPointF(8.0, 12.0));
        break;
    }
    case OperationState::Error:
        p.setPen(Qt::NoPen);
        p.setBrush(QColor(215, 60, 50));
        p.drawEllipse(box);
        p.setPen(QPen(Qt::white, 1.8, Qt::SolidLine, Qt::RoundCap));
        p.drawLine(QPointF(5.3, 5.3), QPointF(10.7, 10.7));
        p.drawLine(QPointF(10.7, 5.3), QPointF(5.3, 10.7));
        break;
    case OperationState::Generating:
        p.setPen(QPen(QColor(80, 150, 240), 2.2, Qt::SolidLine, Qt::RoundCap));
        p.drawArc(box.adjusted(1, 1, -1, -1), -spinStep_ * 30 * 16, 270 * 16);
        break;
    case OperationState::None:
        p.setPen(QPen(QColor(140, 140, 140), 1.4));
        p.setBrush(Qt::NoBrush);
        p.drawEllipse(box.adjusted(1, 1, -1, -1));
        break;
    }
    return QIcon(pixmap);
}

void ProjectTree::refreshStates() {
    bool generating = false;
    for (QTreeWidgetItemIterator it(this); *it; ++it) {
        if (static_cast<Selection::Kind>((*it)->data(0, kKindRole).toInt()) !=
            Selection::Kind::Operation) {
            continue;
        }
        const auto id = Uuid::parse((*it)->data(0, kIdRole).toString().toStdString());
        const OperationState state = state_ && id ? state_(*id) : OperationState::None;
        generating = generating || state == OperationState::Generating;
        (*it)->setIcon(0, stateIcon(state));
    }
    if (generating && !spin_->isActive()) {
        spin_->start();
    } else if (!generating) {
        spin_->stop();
    }
}

QTreeWidgetItem* ProjectTree::addItem(QTreeWidgetItem* parent, const QString& text,
                                      Selection::Kind kind, const Uuid& id, const QString& detail) {
    auto* item = new QTreeWidgetItem(parent, {text, detail});
    Qt::ItemFlags flags = Qt::ItemIsSelectable | Qt::ItemIsEnabled;
    if (kind == Selection::Kind::Operation) {
        flags |= Qt::ItemIsDragEnabled;
    } else if (kind == Selection::Kind::Setup) {
        flags |= Qt::ItemIsDropEnabled;
    }
    item->setFlags(flags);
    item->setData(0, kKindRole, static_cast<int>(kind));
    item->setData(0, kIdRole, QString::fromStdString(id.toString()));
    return item;
}

void ProjectTree::rebuild(const document::Project& project) {
    const Selection previous = selection();
    QSet<QString> expanded;
    for (QTreeWidgetItemIterator it(this); *it; ++it) {
        if ((*it)->isExpanded()) {
            expanded.insert(itemKey(*it));
        }
    }
    const bool firstBuild = topLevelItemCount() == 0;

    const QSignalBlocker block(this);
    clear();
    auto* root = new QTreeWidgetItem(this, {qs(project.name), tr("project")});
    root->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);
    root->setData(0, kKindRole, static_cast<int>(Selection::Kind::Project));
    root->setData(0, kIdRole, QString::fromStdString(project.id.toString()));

    auto group = [&](const QString& name, const QString& key, std::size_t count) {
        auto* g = new QTreeWidgetItem(root, {name, QString::number(count)});
        g->setData(0, kKindRole, static_cast<int>(Selection::Kind::None));
        g->setData(0, kGroupRole, key);
        g->setFlags(Qt::ItemIsEnabled);
        return g;
    };

    auto* models = group(tr("Models"), QStringLiteral("models"), project.models.size());
    for (const auto& m : project.models) {
        QStringList holding;
        for (const auto& s : project.setups) {
            if (std::find(s.fixtures.begin(), s.fixtures.end(), m.id) != s.fixtures.end()) {
                holding << qs(s.name);
            }
        }
        addItem(models, qs(m.name), Selection::Kind::Model, m.id,
                holding.isEmpty() ? QString()
                                  : tr("fixture: %1").arg(holding.join(QStringLiteral(", "))));
    }

    auto* tools = group(tr("Tools"), QStringLiteral("tools"), project.tools.tools.size());
    for (const auto& t : project.tools.tools) {
        addItem(tools, QStringLiteral("T%1  %2").arg(t.number).arg(qs(t.name)),
                Selection::Kind::Tool, t.id, toolDetail(t));
    }

    auto* setups = group(tr("Setups"), QStringLiteral("setups"), project.setups.size());
    for (const auto& s : project.setups) {
        auto* setupItem = addItem(setups, qs(s.name), Selection::Kind::Setup, s.id,
                                  qs(document::workOffsetName(project, s, s.workOffset)));
        for (const auto& op : s.operations) {
            QString detail = qs(op.type);
            if (op.toolId) {
                if (const auto* tool = project.tools.findTool(*op.toolId)) {
                    detail += QStringLiteral(", T%1").arg(tool->number);
                }
            }
            auto* opItem =
                addItem(setupItem, qs(op.name), Selection::Kind::Operation, op.id, detail);
            if (!op.enabled) {
                QFont f = opItem->font(0);
                f.setStrikeOut(true);
                opItem->setFont(0, f);
            }
        }
    }

    for (QTreeWidgetItemIterator it(this); *it; ++it) {
        (*it)->setExpanded(firstBuild || expanded.contains(itemKey(*it)));
    }
    refreshStates();
    select(previous);
}

void ProjectTree::setReorderEnabled(bool enabled) {
    setDragDropMode(enabled ? QAbstractItemView::InternalMove : QAbstractItemView::NoDragDrop);
}

void ProjectTree::dropEvent(QDropEvent* event) {
    // The document moves the operation (one undo step) and the tree is rebuilt from it, so the
    // view does not move any items itself.
    QTreeWidgetItem* dragged = currentItem();
    QTreeWidgetItem* target = itemAt(event->position().toPoint());
    const DropIndicatorPosition where = dropIndicatorPosition();
    event->setDropAction(Qt::IgnoreAction);
    event->accept();
    stopAutoScroll();
    setState(NoState);
    viewport()->update();
    auto kindOf = [](const QTreeWidgetItem* item) {
        return static_cast<Selection::Kind>(item->data(0, kKindRole).toInt());
    };
    if (!dragged || !target || kindOf(dragged) != Selection::Kind::Operation) {
        return;
    }
    QTreeWidgetItem* setupItem = nullptr;
    int index = 0;
    if (kindOf(target) == Selection::Kind::Operation &&
        (where == AboveItem || where == BelowItem)) {
        setupItem = target->parent();
        index = setupItem->indexOfChild(target) + (where == BelowItem ? 1 : 0);
    } else if (kindOf(target) == Selection::Kind::Setup && where == OnItem) {
        setupItem = target;
        index = target->childCount();
    } else {
        return;
    }
    const auto op = Uuid::parse(dragged->data(0, kIdRole).toString().toStdString());
    const auto setup = Uuid::parse(setupItem->data(0, kIdRole).toString().toStdString());
    if (!op || !setup) {
        return;
    }
    // Emitted after the drop has finished: the document change rebuilds this tree.
    QMetaObject::invokeMethod(
        this,
        [this, op = *op, setup = *setup, index] { emit operationMoveRequested(op, setup, index); },
        Qt::QueuedConnection);
}

Selection ProjectTree::selection() const {
    const QTreeWidgetItem* item = currentItem();
    if (!item) {
        return {};
    }
    const auto kind = static_cast<Selection::Kind>(item->data(0, kKindRole).toInt());
    const auto id = Uuid::parse(item->data(0, kIdRole).toString().toStdString());
    if (kind == Selection::Kind::None || !id) {
        return {};
    }
    return {kind, *id};
}

void ProjectTree::select(const Selection& wanted) {
    if (wanted.kind == Selection::Kind::None) {
        return;
    }
    const QString id = QString::fromStdString(wanted.id.toString());
    for (QTreeWidgetItemIterator it(this); *it; ++it) {
        if ((*it)->data(0, kIdRole).toString() == id &&
            static_cast<Selection::Kind>((*it)->data(0, kKindRole).toInt()) == wanted.kind) {
            setCurrentItem(*it);
            return;
        }
    }
}

} // namespace ocam::gui
