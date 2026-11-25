// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "WheelGuard.hpp"

#include <QAbstractSlider>
#include <QAbstractSpinBox>
#include <QApplication>
#include <QComboBox>
#include <QScrollBar>
#include <QWheelEvent>

namespace ocam::gui {

namespace {

/// A field whose value the wheel changes (scroll bars scroll, so they are not fields).
bool isValueField(const QWidget* w) {
    return qobject_cast<const QAbstractSpinBox*>(w) || qobject_cast<const QComboBox*>(w) ||
           (qobject_cast<const QAbstractSlider*>(w) && !qobject_cast<const QScrollBar*>(w));
}

bool hasFocus(const QWidget* w) {
    const QWidget* focus = QApplication::focusWidget();
    return focus && (focus == w || w->isAncestorOf(focus));
}

} // namespace

WheelGuard::WheelGuard(QObject* parent) : QObject(parent) {}

void WheelGuard::install() {
    static bool installed = false;
    if (!installed && qApp) {
        qApp->installEventFilter(new WheelGuard(qApp));
        installed = true;
    }
}

bool WheelGuard::eventFilter(QObject* watched, QEvent* event) {
    if (!watched->isWidgetType()) {
        return false;
    }
    auto* widget = static_cast<QWidget*>(watched);
    if (event->type() == QEvent::Polish && isValueField(widget)) {
        // Qt gives the wheel's focus before any filter sees the wheel: fields (and the line edit
        // in a spin box) take the focus by click or tab only.
        auto strong = [](QWidget* w) {
            if (w->focusPolicy() == Qt::WheelFocus) {
                w->setFocusPolicy(Qt::StrongFocus);
            }
        };
        strong(widget);
        for (QWidget* child : widget->findChildren<QWidget*>()) {
            strong(child);
        }
        return false;
    }
    if (event->type() != QEvent::Wheel) {
        return false;
    }
    if (!isValueField(widget) || hasFocus(widget)) {
        return false;
    }
    // Not focused: the wheel goes to what holds the field (the page scrolls), as if the field
    // were not there. A field also keeps the focus it did not have.
    QWidget* parent = widget->parentWidget();
    if (parent) {
        const auto* wheel = static_cast<QWheelEvent*>(event);
        QWheelEvent forwarded(parent->mapFromGlobal(wheel->globalPosition()),
                              wheel->globalPosition(), wheel->pixelDelta(), wheel->angleDelta(),
                              wheel->buttons(), wheel->modifiers(), wheel->phase(),
                              wheel->inverted(), wheel->source());
        QApplication::sendEvent(parent, &forwarded);
    }
    return true;
}

} // namespace ocam::gui
