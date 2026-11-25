// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <ocam/tooling/Profile.hpp>

#include <QPolygonF>
#include <QWidget>
#include <optional>

namespace ocam::gui {

/// Side view of a tool's revolved profile (mirrored about the axis), with the cutting part
/// highlighted, the holder drawn above the stickout and the assembly's gauge line dashed; a
/// holder on its own; or a turning tool in the lathe plane (Z to the right, X up), its insert
/// highlighted and its programmed point marked.
class ToolProfileView : public QWidget {
    Q_OBJECT

public:
    explicit ToolProfileView(QWidget* parent = nullptr);

    /// Shows the tool, or a message when it has no valid profile. The holder sits at the
    /// stickout (the tool's overall length when that is 0).
    void setTool(const tooling::Tool& tool, const tooling::Holder* holder);
    /// Shows a holder on its own (its tool face at the bottom), or a message when its outline
    /// is invalid.
    void setHolder(const tooling::Holder& holder);

    QSize sizeHint() const override { return {220, 320}; }

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    /// A turning tool, in the lathe plane relative to its programmed point.
    struct TurningView {
        QPolygonF insert;
        QPolygonF holder;
        QPolygonF head;       ///< ISO holders: from the insert to the shank.
        double f = 0.0;       ///< ISO holders: dimension f, drawn from the tip (0 = none).
        bool fAlongX = false; ///< Boring bars measure it down to the axis.
        QString fText;        ///< Written on the dimension line.
        QString caption;
    };
    void paintTurning(QPainter& painter, const TurningView& view);
    /// The tool (its insert fitted to the holder), and an ISO holder drawn with it.
    void showTurning(const tooling::Tool& tool, const tooling::Holder* holder);

    std::optional<TurningView> turning_;
    std::optional<tooling::ToolProfile> profile_;
    std::optional<geom2d::Polyline> holder_;
    double holderBase_ = 0.0; ///< Height of the holder's tool face above the tip.
    double gauge_ = 0.0;      ///< Height of the spindle gauge line above the tip (0 = none).
    QString message_;
};

} // namespace ocam::gui
