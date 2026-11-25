// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ToolpathDataDialog.hpp"

#include "Units.hpp"

#include <QDialogButtonBox>
#include <QFontMetrics>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QTableView>
#include <QVBoxLayout>
#include <algorithm>

namespace ocam::gui {

using toolpath::CycleType;
using toolpath::FeedClass;
using toolpath::Move;
using toolpath::MoveKind;

namespace {

/// A number, without the minus of a value that rounds to zero.
QString number(double value, int decimals) {
    QString text = QString::number(value, 'f', decimals);
    if (text.startsWith(QLatin1Char('-')) && text.toDouble() == 0.0) {
        text.remove(0, 1);
    }
    return text;
}

} // namespace

void ToolpathDataModel::setPath(std::shared_ptr<const toolpath::Toolpath> path, LengthUnit unit) {
    beginResetModel();
    path_ = std::move(path);
    unit_ = unit;
    endResetModel();
}

int ToolpathDataModel::rowCount(const QModelIndex& parent) const {
    return parent.isValid() || !path_ ? 0 : static_cast<int>(path_->moves.size());
}

int ToolpathDataModel::columnCount(const QModelIndex& parent) const {
    return parent.isValid() ? 0 : ColumnCount;
}

const toolpath::DrillCycle* ToolpathDataModel::cycleOf(const Move& move) const {
    const bool held = move.kind == MoveKind::Cycle && move.cycle >= 0 &&
                      static_cast<std::size_t>(move.cycle) < path_->cycles.size();
    return held ? &path_->cycles[static_cast<std::size_t>(move.cycle)] : nullptr;
}

QString ToolpathDataModel::length(double mm) const {
    return number(fromMm(mm, unit_), lengthDecimals(unit_));
}

QString ToolpathDataModel::lengthWithUnit(double mm) const {
    return length(mm) + lengthSuffix(unit_);
}

QString ToolpathDataModel::details(const Move& move) const {
    QStringList parts;
    if (move.kind == MoveKind::Dwell) {
        parts << tr("%1 s").arg(number(move.dwell, 3));
    }
    if (move.pitch != 0.0) {
        parts << tr("Pitch %1/rev").arg(lengthWithUnit(move.pitch));
    }
    if (const toolpath::DrillCycle* c = cycleOf(move)) {
        parts << tr("Top %1").arg(lengthWithUnit(c->topZ));
        parts << tr("Bottom %1").arg(lengthWithUnit(c->bottomZ));
        parts << tr("Retract %1").arg(lengthWithUnit(c->retractZ));
        if (c->type == CycleType::Peck || c->type == CycleType::ChipBreak) {
            parts << tr("Peck %1").arg(lengthWithUnit(c->peck));
        }
        if (c->type == CycleType::ChipBreak) {
            parts << tr("Chip break %1").arg(lengthWithUnit(c->chipBreakRetract));
        }
        if (c->type == CycleType::Dwell) {
            parts << tr("Dwell %1 s").arg(number(c->dwell, 3));
        }
        if (toolpath::isTap(c->type)) {
            parts << tr("Pitch %1/rev").arg(lengthWithUnit(c->pitch));
        }
    }
    return parts.join(QStringLiteral(", "));
}

QString ToolpathDataModel::text(int row, Column column) const {
    const Move& m = path_->moves[static_cast<std::size_t>(row)];
    switch (column) {
    case Number:
        return QString::number(row + 1); // as the simulation counts them
    case Type:
        switch (m.kind) {
        case MoveKind::Rapid:
            return tr("Rapid");
        case MoveKind::Linear:
            return tr("Linear");
        case MoveKind::ArcCW:
            return tr("Arc CW");
        case MoveKind::ArcCCW:
            return tr("Arc CCW");
        case MoveKind::Dwell:
            return tr("Dwell");
        case MoveKind::Probe:
            return m.measure ? tr("Probe: measure") : tr("Probe: protected move");
        case MoveKind::Cycle:
            break;
        }
        if (const toolpath::DrillCycle* c = cycleOf(m)) {
            switch (c->type) {
            case CycleType::Drill:
                return tr("Drill cycle");
            case CycleType::Dwell:
                return tr("Dwell cycle");
            case CycleType::Peck:
                return tr("Peck cycle");
            case CycleType::ChipBreak:
                return tr("Chip-break cycle");
            case CycleType::Tap:
                return tr("Tap cycle");
            case CycleType::TapLeft:
                return tr("Left-hand tap cycle");
            case CycleType::Bore:
                return tr("Bore cycle");
            case CycleType::BackBore:
                return tr("Back bore cycle");
            }
        }
        return tr("Cycle");
    case Purpose:
        switch (m.feedClass) {
        case FeedClass::Rapid:
            return tr("Rapid");
        case FeedClass::Cut:
            return tr("Cut");
        case FeedClass::Plunge:
            return tr("Plunge");
        case FeedClass::Ramp:
            return tr("Ramp");
        case FeedClass::LeadIn:
            return tr("Lead-in");
        case FeedClass::LeadOut:
            return tr("Lead-out");
        case FeedClass::Link:
            return tr("Link");
        case FeedClass::Probe:
            return tr("Probe");
        }
        return {};
    case X:
        return length(m.end.x());
    case Y:
        return length(m.end.y());
    case Z:
        return length(m.end.z());
    case Feed: {
        // A cycle's feed is its own: that is the one handed to the post. Rapids and dwells have
        // none, so they show none; a feed move at 0 shows its 0.
        const toolpath::DrillCycle* c = cycleOf(m);
        const double feed = c ? c->feedRate : m.feedRate;
        const bool feedMove = m.kind != MoveKind::Rapid && m.kind != MoveKind::Dwell;
        return feedMove || feed != 0.0 ? length(feed * 60.0) : QString();
    }
    case Plane:
        if (!m.isArc()) {
            return {};
        }
        switch (m.plane) {
        case toolpath::ArcPlane::XY:
            return QStringLiteral("XY");
        case toolpath::ArcPlane::ZX:
            return QStringLiteral("ZX");
        case toolpath::ArcPlane::YZ:
            return QStringLiteral("YZ");
        }
        return {};
    case CenterX:
        return m.isArc() ? length(m.center.x()) : QString();
    case CenterY:
        return m.isArc() ? length(m.center.y()) : QString();
    case CenterZ:
        return m.isArc() ? length(m.center.z()) : QString();
    case Radius:
        // In the arc's plane, so a helix's climb does not count.
        return m.isArc() ? length((toolpath::planeCoords(m.end, m.plane) -
                                   toolpath::planeCoords(m.center, m.plane))
                                      .norm())
                         : QString();
    case AxisI:
        return number(m.axis.x(), 6);
    case AxisJ:
        return number(m.axis.y(), 6);
    case AxisK:
        return number(m.axis.z(), 6);
    case Details:
        return details(m);
    case ColumnCount:
        break;
    }
    return {};
}

QVariant ToolpathDataModel::data(const QModelIndex& index, int role) const {
    if (!index.isValid() || !path_) {
        return {};
    }
    const auto column = static_cast<Column>(index.column());
    if (role == Qt::DisplayRole) {
        return text(index.row(), column);
    }
    if (role == Qt::ToolTipRole && column == Details) {
        return text(index.row(), column); // it may not fit its column
    }
    if (role == Qt::TextAlignmentRole) {
        const bool leftAligned =
            column == Type || column == Purpose || column == Plane || column == Details;
        return static_cast<int>((leftAligned ? Qt::AlignLeft : Qt::AlignRight) | Qt::AlignVCenter);
    }
    return {};
}

QVariant ToolpathDataModel::headerData(int section, Qt::Orientation orientation, int role) const {
    if (orientation != Qt::Horizontal || role != Qt::DisplayRole) {
        return QAbstractTableModel::headerData(section, orientation, role);
    }
    const QString unit = lengthSuffix(unit_).trimmed();
    switch (section) {
    case Number:
        return tr("#");
    case Type:
        return tr("Type");
    case Purpose:
        return tr("Purpose");
    case X:
        // A turning toolpath holds the radius: a post may write it as a diameter.
        return path_ && path_->isTurning() ? tr("X radius (%1)").arg(unit) : tr("X (%1)").arg(unit);
    case Y:
        return tr("Y (%1)").arg(unit);
    case Z:
        return tr("Z (%1)").arg(unit);
    case Feed:
        return tr("Feed (%1/min)").arg(unit);
    case Plane:
        return tr("Arc plane");
    case CenterX:
        return tr("Centre X (%1)").arg(unit);
    case CenterY:
        return tr("Centre Y (%1)").arg(unit);
    case CenterZ:
        return tr("Centre Z (%1)").arg(unit);
    case Radius:
        return tr("Radius (%1)").arg(unit);
    case AxisI:
        return tr("Axis I");
    case AxisJ:
        return tr("Axis J");
    case AxisK:
        return tr("Axis K");
    case Details:
        return tr("Details");
    default:
        return {};
    }
}

ToolpathDataDialog::ToolpathDataDialog(QWidget* parent) : QDialog(parent) {
    setObjectName(QStringLiteral("toolpathData"));
    setAttribute(Qt::WA_DeleteOnClose);
    setWindowFlag(Qt::WindowMaximizeButtonHint);
    resize(1120, 640);

    auto* layout = new QVBoxLayout(this);
    summary_ = new QLabel;
    summary_->setWordWrap(true);
    summary_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    layout->addWidget(summary_);

    model_ = new ToolpathDataModel(this);
    table_ = new QTableView;
    table_->setModel(model_);
    table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    table_->setWordWrap(false);
    table_->verticalHeader()->hide();
    // One height for every row, or the header would measure them all on a long toolpath.
    table_->verticalHeader()->setSectionResizeMode(QHeaderView::Fixed);
    table_->verticalHeader()->setDefaultSectionSize(fontMetrics().height() + 6);
    table_->horizontalHeader()->setStretchLastSection(true);
    layout->addWidget(table_, 1);

    message_ = new QLabel;
    message_->setWordWrap(true);
    message_->setAlignment(Qt::AlignCenter);
    message_->setTextFormat(Qt::PlainText);
    message_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    layout->addWidget(message_, 1);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Close);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::close);
    layout->addWidget(buttons);
}

void ToolpathDataDialog::setOperationName(const QString& name) {
    setWindowTitle(tr("Toolpath data: %1").arg(name));
}

QString ToolpathDataDialog::summary(const toolpath::Toolpath& path, LengthUnit unit) const {
    QStringList parts;
    parts << tr("Moves: %1").arg(path.moves.size());
    // A turning insert has no diameter.
    parts << (path.toolDiameter != 0.0 ? tr("Tool T%1, Ø%2")
                                             .arg(path.toolNumber)
                                             .arg(formatLength(path.toolDiameter, unit))
                                       : tr("Tool T%1").arg(path.toolNumber));
    const QString speed = number(path.spindleSpeed * 60.0, 1);
    if (path.surfaceSpeed != 0.0) {
        parts << tr("Surface speed %1%2, up to %3 rpm, feeds worked out at %4 rpm")
                     .arg(number(surfaceSpeedIn(path.surfaceSpeed, unit), 1),
                          surfaceSpeedSuffix(unit), number(path.maxSpindle * 60.0, 1), speed);
    } else {
        parts << (path.live ? tr("Live tool %1 rpm") : tr("Spindle %1 rpm")).arg(speed);
    }
    if (path.compensation != toolpath::Compensation::None) {
        parts << (path.compensation == toolpath::Compensation::Left
                      ? tr("Control offsets it left")
                      : tr("Control offsets it right"));
    }
    parts << tr("Starts at X %1, Y %2, Z %3")
                 .arg(formatLength(path.start.x(), unit, false),
                      formatLength(path.start.y(), unit, false),
                      formatLength(path.start.z(), unit, false));
    return parts.join(QStringLiteral(" · ")) + QLatin1Char('\n') +
           tr("Positions are where the tool tip ends each move, in the setup's work coordinates, "
              "before the post processor.");
}

void ToolpathDataDialog::showPath(const std::shared_ptr<const toolpath::Toolpath>& path,
                                  LengthUnit unit) {
    message_->hide();
    summary_->setText(summary(*path, unit));
    summary_->show();
    table_->show();
    if (model_->path() == path.get() && model_->unit() == unit) {
        return;
    }
    model_->setPath(path, unit);
    const bool multiAxis = toolpath::isMultiAxis(*path);
    table_->setColumnHidden(ToolpathDataModel::AxisI, !multiAxis);
    table_->setColumnHidden(ToolpathDataModel::AxisJ, !multiAxis);
    table_->setColumnHidden(ToolpathDataModel::AxisK, !multiAxis);
    const bool arcs = std::ranges::any_of(path->moves, &toolpath::Move::isArc);
    for (int c = ToolpathDataModel::Plane; c <= ToolpathDataModel::Radius; ++c) {
        table_->setColumnHidden(c, !arcs);
    }
    table_->resizeColumnsToContents();
    // The rows measured are the first ones: leave room for the numbers further down.
    const int room = fontMetrics().horizontalAdvance(QStringLiteral("-00000.0000")) + 12;
    for (int c = ToolpathDataModel::X; c < ToolpathDataModel::Details; ++c) {
        if (c != ToolpathDataModel::Plane) {
            table_->setColumnWidth(c, std::max(table_->columnWidth(c), room));
        }
    }
    table_->setColumnWidth(
        ToolpathDataModel::Number,
        std::max(table_->columnWidth(ToolpathDataModel::Number),
                 fontMetrics().horizontalAdvance(QString::number(model_->rowCount())) + 12));
}

void ToolpathDataDialog::showMessage(const QString& message) {
    model_->setPath(nullptr, LengthUnit::Millimeter);
    summary_->hide();
    table_->hide();
    message_->setText(message);
    message_->show();
}

QString ToolpathDataDialog::message() const {
    return message_->isHidden() ? QString() : message_->text();
}

} // namespace ocam::gui
