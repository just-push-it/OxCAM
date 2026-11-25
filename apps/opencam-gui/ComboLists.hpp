// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Lists in combo boxes that show what each entry is: where an entry comes from is a heading over
// its group rather than words in front of its name, and the open list is as wide as its widest
// entry, so a long name shows whole even when the box is narrow.

#pragma once

#include <QString>

class QComboBox;

namespace ocam::gui {

/// A heading in a combo box's list: a separator, then a title nobody can choose. The entries
/// after it, up to the next heading or separator, are its group.
void addListHeading(QComboBox* box, const QString& title);

/// Whether entry `index` of `box` is a heading (addListHeading).
bool isListHeading(const QComboBox* box, int index);

/// The open list shows every entry in full: as wide as its widest entry (the box itself may be
/// narrower), cut at the end if the screen is narrower still, never in the middle. An entry
/// without a tooltip gets its text as one. Call it once the box is filled.
void showEntriesInFull(QComboBox* box);

} // namespace ocam::gui
