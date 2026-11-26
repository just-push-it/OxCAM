// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ToolProfileView.hpp"

#include "Units.hpp"

#include <ocam/foundation/Error.hpp>
#include <ocam/tooling/TurningHolder.hpp>

#include <QCoreApplication>
#include <QPainter>
#include <QPainterPath>
#include <algorithm>

namespace ocam::gui {
namespace {

// Closed outline mirrored about the axis, in (radius, height) tool coordinates.
QPolygonF mirrored(const std::vector<Vec2>& half, double zOffset = 0.0) {
    QPolygonF poly;
    for (const Vec2& p : half) {
        poly << QPointF(p.x(), p.y() + zOffset);
    }
    for (auto it = half.rbegin(); it != half.rend(); ++it) {
        poly << QPointF(-it->x(), it->y() + zOffset);
    }
    return poly;
}

QPolygonF polygonOf(const geom2d::Polyline& outline) {
    QPolygonF poly;
    for (const Vec2& p : outline.discretize(0.005)) {
        poly << QPointF(p.x(), p.y());
    }
    return poly;
}

/// A holder's shank cut short after its head, with a break line, so the insert stays readable:
/// the rectangle's `lo` corner is where the head ends, and it runs along X (boring bars) or Y.
QPolygonF brokenShank(const geom2d::Polyline& shank, const QPolygonF& head, bool alongZ) {
    QPolygonF full = polygonOf(shank);
    const QRectF box = full.boundingRect();
    const QRectF headBox = head.boundingRect();
    const double breadth = alongZ ? box.height() : box.width();
    const double start = alongZ ? box.left() : box.top();
    const double end = alongZ ? box.right() : box.bottom();
    const double cut = start + 1.6 * breadth;
    if (cut >= end || breadth <= 0.0 || headBox.isEmpty()) {
        return full;
    }
    QPolygonF out;
    const double tooth = 0.05 * breadth;
    if (alongZ) {
        out << QPointF(start, box.top()) << QPointF(start, box.bottom());
        out << QPointF(cut, box.bottom());
        for (int k = 1; k < 6; ++k) {
            out << QPointF(cut + (k % 2 != 0 ? tooth : -tooth), box.bottom() - k * breadth / 6.0);
        }
        out << QPointF(cut, box.top());
    } else {
        out << QPointF(box.left(), start) << QPointF(box.right(), start);
        out << QPointF(box.right(), cut);
        for (int k = 1; k < 6; ++k) {
            out << QPointF(box.right() - k * breadth / 6.0, cut + (k % 2 != 0 ? tooth : -tooth));
        }
        out << QPointF(box.left(), cut);
    }
    return out;
}

QPen axisPen() {
    QPen pen(QColor(90, 140, 220));
    pen.setCosmetic(true);
    pen.setStyle(Qt::DashDotLine);
    return pen;
}

} // namespace

ToolProfileView::ToolProfileView(QWidget* parent) : QWidget(parent) {
    setMinimumSize(160, 220);
}

void ToolProfileView::setTool(const tooling::Tool& tool, const tooling::Holder* holder) {
    profile_.reset();
    holder_.reset();
    turning_.reset();
    gauge_ = 0.0;
    message_.clear();
    if (tooling::isTurningTool(tool.type)) {
        try {
            showTurning(tool, holder);
        } catch (const Error& e) {
            message_ = QString::fromUtf8(e.what());
        }
    } else {
        try {
            profile_ = tooling::toolProfile(tool);
            holderBase_ = tooling::visibleLength(tool);
            if (holder && holder->validate().empty()) {
                holder_ = holder->profile;
            }
            gauge_ = tooling::gaugeLength(tool, holder_ ? holder : nullptr);
        } catch (const Error& e) {
            message_ = QString::fromUtf8(e.what());
        }
    }
    update();
}

void ToolProfileView::showTurning(const tooling::Tool& tool, const tooling::Holder* holder) {
    const bool iso = holder && holder->iso && holder->validate().empty() &&
                     tool.type == tooling::ToolType::TurningInsert;
    const bool stick = holder && holder->stick;
    const tooling::TurningSide side = tool.solid || (stick && holder->stick->bar)
                                          ? tooling::TurningSide::Inside
                                      : iso ? tooling::holderSide(*holder->iso)
                                            : tooling::TurningSide::Outside;
    const tooling::TurningShape shape = tooling::turningShape(tool, side);
    TurningView view;
    view.insert = polygonOf(shape.outline);
    if (tool.solid) {
        // The head and shank, and what catalogues sell a solid bar by.
        const auto body = tooling::turningBody(tool, nullptr, side);
        view.head = polygonOf(body.front());
        if (body.size() > 1) {
            view.holder = polygonOf(body.back());
        }
        view.caption = tr("Min. bore Ø%1\nShank Ø%2, S %3")
                           .arg(formatLength(tool.solid->minBore, tool.unit),
                                formatLength(tool.solid->shankDiameter, tool.unit),
                                formatLength(tool.solid->centreToTip, tool.unit));
    } else if (stick) {
        const auto body = tooling::turningBody(tool, holder, side);
        view.holder = polygonOf(body.front());
        view.f = holder->stick->f;
        view.fAlongX = holder->stick->bar;
        view.fText = tr("f = %1").arg(formatLength(holder->stick->f, tool.unit));
        view.caption =
            holder->stick->bar ? tr("f: tip to bar's axis") : tr("f: tip to back of shank");
    } else if (iso) {
        const tooling::TurningHolderCode& c = *holder->iso;
        const tooling::TurningHolderOutline outline =
            tooling::turningHolderOutline(c, shape.outline);
        view.head = polygonOf(outline.head);
        view.holder = brokenShank(outline.shank, view.head, c.boringBar);
        view.f = c.hand == 'L' && !c.boringBar ? -c.f : c.f; // a bar's axis stays below the tip
        view.fAlongX = c.boringBar;
        if (c.hand == 'L') {
            for (QPointF& p : view.insert) {
                p.setX(-p.x());
            }
        }
        view.caption = QString::fromStdString(tooling::designation(c));
        if (shape.width <= 0.0 && tool.insert.cuttingEdgeAngle > 0.0) {
            view.caption += QLatin1Char('\n') +
                            tr("Cutting edge %1°").arg(radToDeg(tool.insert.cuttingEdgeAngle));
        }
        view.fText = tr("f = %1").arg(formatLength(c.f, tool.unit));
        view.caption += QLatin1Char('\n') +
                        (c.boringBar ? tr("f: tip to bar's axis") : tr("f: tip to back of shank"));
    } else if (holder && holder->kind == tooling::HolderKind::Turning &&
               holder->validate().empty()) {
        // An outline of its own (an imported model): above the insert, on its middle.
        const QRectF insert = view.insert.boundingRect();
        view.holder = mirrored(holder->profile.discretize(0.01), insert.bottom());
        view.holder.translate(insert.center().x(), 0.0);
    }
    if (tool.type == tooling::ToolType::TurningInsert) {
        QString about = tr("Nose radius %1").arg(formatLength(tool.insert.noseRadius, tool.unit));
        if (const auto* s = tooling::findInsertShape(tool.insert.shape)) {
            about = tr("%1: %2").arg(QChar::fromLatin1(s->letter),
                                     QCoreApplication::translate("Tooling", s->name)) +
                    QLatin1Char('\n') + about;
        }
        view.caption += (view.caption.isEmpty() ? QString() : QStringLiteral("\n")) + about;
    } else if (tool.type == tooling::ToolType::ThreadingInsert) {
        view.caption = tr("Pitch %1").arg(formatLength(tool.insert.threadPitch, tool.unit));
    } else {
        view.caption = tr("Width %1").arg(formatLength(shape.width, tool.unit));
    }
    turning_ = std::move(view);
}

void ToolProfileView::setHolder(const tooling::Holder& holder) {
    profile_.reset();
    holder_.reset();
    turning_.reset();
    holderBase_ = 0.0;
    gauge_ = 0.0;
    const auto issues = holder.validate();
    if (issues.empty() && (holder.iso || holder.stick)) {
        // With the insert it takes, for a sense of what it holds.
        tooling::Tool sample = tooling::makeDefaultTool(tooling::ToolType::TurningInsert, 1);
        tooling::fitToHolder(sample, holder);
        message_.clear();
        try {
            showTurning(sample, &holder);
        } catch (const Error& e) {
            message_ = QString::fromUtf8(e.what());
        }
    } else if (issues.empty()) {
        holder_ = holder.profile;
        message_.clear();
    } else {
        message_ = QString::fromStdString(issues.front());
    }
    update();
}

void ToolProfileView::paintEvent(QPaintEvent* /*event*/) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.fillRect(rect(), palette().base());

    if (turning_) {
        paintTurning(painter, *turning_);
        return;
    }
    if (!profile_ && !holder_) {
        painter.setPen(palette().color(QPalette::PlaceholderText));
        painter.drawText(rect().adjusted(8, 8, -8, -8), Qt::AlignCenter | Qt::TextWordWrap,
                         message_);
        return;
    }

    QPolygonF tool;
    if (profile_) {
        tool = mirrored(profile_->outline.discretize(0.005));
    }
    QPolygonF holder;
    if (holder_) {
        holder = mirrored(holder_->discretize(0.01), holderBase_);
    }

    QRectF bounds = tool.isEmpty() ? holder.boundingRect() : tool.boundingRect();
    if (!holder.isEmpty()) {
        bounds = bounds.united(holder.boundingRect());
    }
    const double margin = 12.0;
    const double scale = std::min((width() - 2 * margin) / std::max(bounds.width(), 1e-6),
                                  (height() - 2 * margin) / std::max(bounds.height(), 1e-6));

    // Tool coordinates: Y up, tip at the bottom.
    painter.translate(width() / 2.0, height() - margin);
    painter.scale(scale, -scale);
    painter.translate(-bounds.center().x(), -bounds.top());

    QPen outlinePen(palette().color(QPalette::Text));
    outlinePen.setCosmetic(true);

    if (!holder.isEmpty()) {
        painter.setPen(outlinePen);
        painter.setBrush(QColor(120, 120, 130, 110));
        painter.drawPolygon(holder);
    }
    if (!profile_) {
        painter.setPen(axisPen());
        painter.drawLine(QPointF(0, bounds.top() - 2), QPointF(0, bounds.bottom() + 2));
        return;
    }

    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(170, 175, 185));
    painter.drawPolygon(tool);

    // Highlight the cutting part by clipping to heights below the cutting length.
    painter.save();
    painter.setClipRect(
        QRectF(bounds.left() - 1, -1, bounds.width() + 2, profile_->cuttingLength + 1));
    painter.setBrush(QColor(230, 160, 40));
    painter.drawPolygon(tool);
    painter.restore();

    painter.setBrush(Qt::NoBrush);
    painter.setPen(outlinePen);
    painter.drawPolygon(tool);

    painter.setPen(axisPen());
    painter.drawLine(QPointF(0, bounds.top() - 2), QPointF(0, bounds.bottom() + 2));

    if (gauge_ > 0.0) {
        // The spindle gauge line: the gauge length above the tip.
        QPen gaugePen(palette().color(QPalette::Highlight));
        gaugePen.setCosmetic(true);
        gaugePen.setWidthF(1.5);
        gaugePen.setStyle(Qt::DashLine);
        painter.setPen(gaugePen);
        const double reach = std::max(bounds.width() / 2.0, 1.0) + 4.0 / scale;
        painter.drawLine(QPointF(bounds.center().x() - reach, gauge_),
                         QPointF(bounds.center().x() + reach, gauge_));
    }
}

void ToolProfileView::paintTurning(QPainter& painter, const TurningView& view) {
    QRectF bounds = view.insert.boundingRect();
    if (!view.holder.isEmpty()) {
        bounds = bounds.united(view.holder.boundingRect());
    }
    if (!view.head.isEmpty()) {
        bounds = bounds.united(view.head.boundingRect());
    }
    bounds = bounds.united(QRectF(0.0, 0.0, 0.0, 0.0)); // the programmed point
    if (view.f != 0.0) {
        // Room for the dimension line, on the far side of the tip from the holder.
        const QRectF line = view.fAlongX ? QRectF(-0.14 * view.f, -view.f, 0.0, view.f)
                                         : QRectF(0.0, -0.1 * std::abs(view.f), view.f, 0.0);
        bounds = bounds.united(line.normalized());
    }
    const double margin = 24.0;
    // A bar's dimension is written to the left of its line: room for it beside the drawing.
    const double textRoom = view.fAlongX && view.f != 0.0
                                ? painter.fontMetrics().horizontalAdvance(view.fText) + 8.0
                                : 0.0;
    // Below the caption, so it stays readable.
    const double top =
        painter
            .boundingRect(rect().adjusted(8, 8, -8, -8), Qt::AlignLeft | Qt::AlignTop, view.caption)
            .bottom();
    const double room = std::max(height() - top - 2 * margin, 40.0);
    const double scale =
        std::min((width() - 2 * margin - textRoom) / std::max(bounds.width(), 1e-6),
                 room / std::max(bounds.height(), 1e-6));

    const QTransform device = painter.transform();
    painter.translate(width() / 2.0 + textRoom / 2.0, top + margin + room / 2.0);
    painter.scale(scale, -scale);
    painter.translate(-bounds.center());

    QPen outlinePen(palette().color(QPalette::Text));
    outlinePen.setCosmetic(true);
    painter.setPen(outlinePen);
    if (!view.holder.isEmpty()) {
        painter.setBrush(QColor(120, 120, 130, 110));
        painter.drawPolygon(view.holder);
    }
    if (!view.head.isEmpty()) {
        painter.setBrush(QColor(120, 120, 130, 170));
        painter.drawPolygon(view.head);
    }
    painter.setBrush(QColor(230, 160, 40));
    painter.drawPolygon(view.insert);

    // The programmed point: what the toolpath's coordinates are of.
    QPen pointPen(palette().color(QPalette::Highlight));
    pointPen.setCosmetic(true);
    pointPen.setWidthF(1.5);
    painter.setPen(pointPen);
    const double cross = 6.0 / scale;
    painter.drawLine(QPointF(-cross, 0.0), QPointF(cross, 0.0));
    painter.drawLine(QPointF(0.0, -cross), QPointF(0.0, cross));

    // Dimension f: from the tip across to the shank's back, which it marks, or down to the bar's
    // axis.
    QPointF fFrom, fTo;
    if (view.f != 0.0) {
        const double off = 0.08 * std::abs(view.f);
        fFrom = view.fAlongX ? QPointF(-off, 0.0) : QPointF(0.0, -off);
        fTo = view.fAlongX ? QPointF(-off, -view.f) : QPointF(view.f, -off);
        QPen dimension = pointPen;
        dimension.setWidthF(1.0);
        painter.setPen(dimension);
        painter.drawLine(fFrom, fTo);
        painter.drawLine(view.fAlongX ? QPointF(-off, 0.0) : QPointF(0.0, 0.0), fFrom);
        painter.drawLine(view.fAlongX ? QPointF(0.0, -view.f) : QPointF(view.f, 0.0), fTo);
        if (view.fAlongX) {
            // The bar's axis.
            painter.setPen(axisPen());
            painter.drawLine(QPointF(bounds.left(), -view.f), QPointF(bounds.right(), -view.f));
        } else {
            // The shank's back, up to where it is cut short.
            QPen back = pointPen;
            back.setWidthF(2.5);
            painter.setPen(back);
            painter.drawLine(QPointF(view.f, 0.0), QPointF(view.f, bounds.bottom()));
        }
    }
    const QPointF fLabel = painter.transform().map(0.5 * (fFrom + fTo));

    painter.setTransform(device);
    painter.setPen(palette().color(QPalette::Text));
    painter.drawText(rect().adjusted(8, 8, -8, -8), Qt::AlignLeft | Qt::AlignTop, view.caption);
    if (view.f != 0.0) {
        // On the dimension line: under it across a shank, to the left of it down to a bar's axis.
        painter.setPen(palette().color(QPalette::Highlight));
        const double wide = painter.fontMetrics().horizontalAdvance(view.fText);
        painter.drawText(
            fLabel + (view.fAlongX ? QPointF(-wide - 6.0, 4.0) : QPointF(-0.5 * wide, 14.0)),
            view.fText);
    }

    // Which way is which on the lathe.
    const QPointF origin(width() - 44.0, height() - 14.0);
    QPen axes = axisPen();
    axes.setStyle(Qt::SolidLine);
    painter.setPen(axes);
    painter.drawLine(origin, origin + QPointF(26.0, 0.0));
    painter.drawLine(origin, origin + QPointF(0.0, -26.0));
    painter.drawText(origin + QPointF(28.0, 4.0), QStringLiteral("Z"));
    painter.drawText(origin + QPointF(-3.0, -29.0), QStringLiteral("X"));
}

} // namespace ocam::gui
