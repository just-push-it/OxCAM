// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ComboLists.hpp"

#include <QAbstractItemView>
#include <QComboBox>
#include <QFont>
#include <QStyle>
#include <algorithm>

namespace ocam::gui {

namespace {

constexpr int kHeadingRole = Qt::UserRole + 40;

} // namespace

void addListHeading(QComboBox* box, const QString& title) {
    if (box->count() > 0) {
        box->insertSeparator(box->count());
    }
    box->addItem(title);
    const int index = box->count() - 1;
    box->setItemData(index, 0, Qt::UserRole - 1); // disabled: it names a group
    box->setItemData(index, true, kHeadingRole);
    QFont font = box->font();
    font.setBold(true);
    box->setItemData(index, font, Qt::FontRole);
}

bool isListHeading(const QComboBox* box, int index) {
    return box->itemData(index, kHeadingRole).toBool();
}

void showEntriesInFull(QComboBox* box) {
    QAbstractItemView* view = box->view();
    view->setTextElideMode(Qt::ElideRight);
    int widest = 0;
    for (int i = 0; i < box->count(); ++i) {
        const QString text = box->itemText(i);
        if (text.isEmpty()) {
            continue; // a separator
        }
        if (box->itemData(i, Qt::ToolTipRole).toString().isEmpty()) {
            box->setItemData(i, text, Qt::ToolTipRole);
        }
        const QVariant font = box->itemData(i, Qt::FontRole);
        const QFontMetrics metrics(font.isValid() ? font.value<QFont>() : view->font());
        int width = metrics.horizontalAdvance(text);
        if (!box->itemIcon(i).isNull()) {
            width += box->iconSize().width() + 4;
        }
        widest = std::max(widest, width);
    }
    // The item's own margins, the list's frame and a scroll bar's room.
    const QStyle* style = view->style();
    const int margins = 2 * style->pixelMetric(QStyle::PM_FocusFrameHMargin, nullptr, view) +
                        2 * style->pixelMetric(QStyle::PM_DefaultFrameWidth, nullptr, view) +
                        style->pixelMetric(QStyle::PM_ScrollBarExtent, nullptr, view) + 16;
    view->setMinimumWidth(widest + margins);
}

} // namespace ocam::gui
