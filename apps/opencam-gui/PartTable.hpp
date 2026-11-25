// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The parts table of the machine and workholding editors (DL-0037, DL-0035): a row per part with
// its name, what carries it (the base, a machine axis or a workholding member), optionally its
// placement, its colour, whether it is checked for collisions and whether it is shown, and a
// read-only note. With several rows selected, a change to the carrier, Collides or Shown in one
// of them applies to all of them.

#pragma once

#include <ocam/foundation/Math.hpp>

#include <QTableWidget>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

class QComboBox;

namespace ocam::gui {

class PartTable : public QTableWidget {
    Q_OBJECT

public:
    struct Row {
        std::string name;
        std::string carrier; ///< Empty = the base.
        Vec3 offset = Vec3::Zero();
        Vec3 rotation = Vec3::Zero();
        std::uint32_t colour = 0xB4B8BE;
        bool collides = true;
        bool shown = true;
        QString note; ///< The last column (triangles, faces), read-only.
    };

    /// `placement`: with the X Y Z and Rot X Y Z columns. `noteTitle` heads the last column.
    PartTable(bool placement, const QString& noteTitle, QWidget* parent = nullptr);

    /// The carrier choices besides the base: (shown name, value). Rows keep what they chose.
    void setCarriers(const std::vector<std::pair<QString, QString>>& carriers);
    void addRow(const Row& row);
    Row row(int index) const;
    /// The selected rows, in order.
    std::vector<int> selectedRows() const;
    /// Removes rows (indices into the table as it is).
    void removeRows(std::vector<int> rows);

    int carrierColumn() const noexcept { return 1; }
    int colourColumn() const noexcept { return placement_ ? 8 : 2; }
    int collidesColumn() const noexcept { return colourColumn() + 1; }
    int shownColumn() const noexcept { return colourColumn() + 2; }
    int noteColumn() const noexcept { return colourColumn() + 3; }

signals:
    /// Something in a row was changed by the user.
    void edited();

private:
    void addCarrierChoices(QComboBox* combo, const QString& current) const;

    bool placement_ = false;
    std::vector<std::pair<QString, QString>> carriers_;
};

} // namespace ocam::gui
