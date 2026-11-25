// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ToolTips.hpp"

#include <QApplication>
#include <QEvent>
#include <QFontMetrics>
#include <QHelpEvent>
#include <QTextDocument>
#include <QToolTip>
#include <QWidget>

namespace ocam::gui {
namespace {

/// Width the text wraps at [px]: about 50 characters, so a tooltip reads as a short paragraph.
constexpr int kWidth = 340;

class ToolTipFormatter : public QObject {
public:
    using QObject::QObject;

protected:
    bool eventFilter(QObject* watched, QEvent* event) override {
        if (event->type() != QEvent::ToolTip || !watched->isWidgetType()) {
            return false;
        }
        auto* widget = static_cast<QWidget*>(watched);
        const QString tip = widget->toolTip();
        if (tip.isEmpty() || Qt::mightBeRichText(tip)) {
            return false; // nothing to show here (item views, menus), or laid out already
        }
        const auto* help = static_cast<QHelpEvent*>(event);
        QToolTip::showText(help->globalPos(), formatToolTip(tip), widget, widget->rect(),
                           widget->toolTipDuration());
        return true;
    }
};

} // namespace

QString formatToolTip(const QString& text, const QString& title) {
    // Long tips in a fixed-width table: Qt wraps a rich text tooltip at its width, so they
    // grow down, not across the screen. Short ones keep their own width. The cell padding
    // keeps the text off the border.
    const QFontMetrics fm(QToolTip::font());
    const bool wrap = fm.horizontalAdvance(text) > kWidth;
    QString html = wrap ? QStringLiteral("<table width=\"%1\" cellspacing=\"0\" "
                                         "cellpadding=\"4\"><tr><td>")
                              .arg(kWidth)
                        : QStringLiteral("<table cellspacing=\"0\" cellpadding=\"4\"><tr><td>");
    if (!title.isEmpty()) {
        html += QStringLiteral("<p style=\"margin-bottom:4px\"><b>%1</b></p>")
                    .arg(title.toHtmlEscaped());
    }
    QString body = text.toHtmlEscaped();
    body.replace(QLatin1Char('\n'), QStringLiteral("<br>"));
    // Short tips stay on one line (Qt would otherwise wrap them at a width of its own).
    html += QStringLiteral("<p style=\"margin:0;%1\">%2</p></td></tr></table>")
                .arg(wrap ? QString() : QStringLiteral(" white-space:nowrap"), body);
    return html;
}

void installToolTipFormatting(QApplication& app) {
    app.installEventFilter(new ToolTipFormatter(&app));
}

} // namespace ocam::gui
