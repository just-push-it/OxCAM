// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "PartTable.hpp"

#include <QCheckBox>
#include <QColor>
#include <QComboBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <algorithm>

namespace ocam::gui {
namespace {

QString qs(const std::string& s) {
    return QString::fromStdString(s);
}

double cellNumber(const QTableWidget* t, int row, int col) {
    const QTableWidgetItem* item = t->item(row, col);
    return item ? item->text().toDouble() : 0.0;
}

QTableWidgetItem* numberItem(double v) {
    return new QTableWidgetItem(QString::number(v, 'g', 10));
}

QString colourText(std::uint32_t rgb) {
    return QStringLiteral("#%1").arg(rgb & 0xFFFFFFu, 6, 16, QLatin1Char('0'));
}

void paintColour(QTableWidgetItem* item, const QColor& c) {
    item->setBackground(c);
    item->setForeground(qGray(c.rgb()) > 128 ? Qt::black : Qt::white);
}

/// A check box centred in a table cell. A cell widget, so that clicking it keeps the rows that
/// are selected (several rows change together).
QWidget* checkCell(bool checked) {
    auto* holder = new QWidget;
    auto* layout = new QHBoxLayout(holder);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setAlignment(Qt::AlignCenter);
    auto* box = new QCheckBox;
    box->setChecked(checked);
    layout->addWidget(box);
    return holder;
}

QCheckBox* checkIn(const QTableWidget* table, int row, int column) {
    const QWidget* cell = table->cellWidget(row, column);
    return cell ? cell->findChild<QCheckBox*>() : nullptr;
}

} // namespace

PartTable::PartTable(bool placement, const QString& noteTitle, QWidget* parent)
    : QTableWidget(0, placement ? 12 : 6, parent), placement_(placement) {
    QStringList headers{tr("Part"), tr("Carried by")};
    if (placement) {
        headers << tr("X") << tr("Y") << tr("Z") << tr("Rot X°") << tr("Rot Y°") << tr("Rot Z°");
    }
    headers << tr("Colour") << tr("Collides") << tr("Shown") << noteTitle;
    setHorizontalHeaderLabels(headers);
    horizontalHeader()->setSectionResizeMode(QHeaderView::Interactive);
    horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
    verticalHeader()->hide();
    setSelectionBehavior(QAbstractItemView::SelectRows);
    setSelectionMode(QAbstractItemView::ExtendedSelection);
    connect(this, &QTableWidget::itemChanged, this, [this](QTableWidgetItem* item) {
        if (item && item->column() == colourColumn()) {
            const QColor c(item->text().trimmed());
            if (c.isValid()) {
                const QSignalBlocker block(this);
                paintColour(item, c);
            }
        }
        emit edited();
    });
}

void PartTable::addCarrierChoices(QComboBox* combo, const QString& current) const {
    const QSignalBlocker block(combo);
    combo->clear();
    combo->addItem(tr("Base"), QString());
    for (const auto& [label, value] : carriers_) {
        combo->addItem(label, value);
    }
    // A carrier not (yet) among the choices keeps its value.
    if (!current.isEmpty() && combo->findData(current) < 0) {
        combo->addItem(current, current);
    }
    combo->setCurrentIndex(std::max(0, combo->findData(current)));
}

void PartTable::setCarriers(const std::vector<std::pair<QString, QString>>& carriers) {
    carriers_ = carriers;
    for (int r = 0; r < rowCount(); ++r) {
        if (auto* combo = qobject_cast<QComboBox*>(cellWidget(r, carrierColumn()))) {
            addCarrierChoices(combo, combo->currentData().toString());
        }
    }
}

void PartTable::addRow(const Row& p) {
    const QSignalBlocker block(this);
    const int r = rowCount();
    insertRow(r);
    setItem(r, 0, new QTableWidgetItem(qs(p.name)));
    auto* carrier = new QComboBox;
    addCarrierChoices(carrier, qs(p.carrier));
    connect(carrier, &QComboBox::currentIndexChanged, this, [this, carrier] {
        // Several rows selected: the change applies to all of them.
        const std::vector<int> rows = selectedRows();
        const QString value = carrier->currentData().toString();
        const bool inSelection = std::any_of(rows.begin(), rows.end(), [&](int x) {
            return cellWidget(x, carrierColumn()) == carrier;
        });
        if (inSelection) {
            for (const int x : rows) {
                if (auto* other = qobject_cast<QComboBox*>(cellWidget(x, carrierColumn()));
                    other && other != carrier) {
                    const QSignalBlocker b(other);
                    other->setCurrentIndex(std::max(0, other->findData(value)));
                }
            }
        }
        emit edited();
    });
    setCellWidget(r, carrierColumn(), carrier);
    if (placement_) {
        for (int k = 0; k < 3; ++k) {
            setItem(r, 2 + k, numberItem(p.offset[k]));
            setItem(r, 5 + k, numberItem(p.rotation[k]));
        }
    }
    auto* colour = new QTableWidgetItem(colourText(p.colour));
    paintColour(colour, QColor::fromRgb(p.colour & 0xFFFFFFu));
    setItem(r, colourColumn(), colour);
    for (const auto& [column, checked] :
         {std::pair{collidesColumn(), p.collides}, std::pair{shownColumn(), p.shown}}) {
        QWidget* cell = checkCell(checked);
        setCellWidget(r, column, cell);
        QCheckBox* box = cell->findChild<QCheckBox*>();
        const int col = column;
        connect(box, &QCheckBox::toggled, this, [this, box, col](bool on) {
            const std::vector<int> rows = selectedRows();
            const bool inSelection = std::any_of(
                rows.begin(), rows.end(), [&](int x) { return checkIn(this, x, col) == box; });
            if (inSelection) {
                for (const int x : rows) {
                    if (QCheckBox* other = checkIn(this, x, col); other && other != box) {
                        const QSignalBlocker b(other);
                        other->setChecked(on);
                    }
                }
            }
            emit edited();
        });
    }
    auto* note = new QTableWidgetItem(p.note);
    note->setFlags(Qt::ItemIsEnabled);
    setItem(r, noteColumn(), note);
}

PartTable::Row PartTable::row(int r) const {
    Row p;
    if (const QTableWidgetItem* name = item(r, 0)) {
        p.name = name->text().trimmed().toStdString();
    }
    if (const auto* combo = qobject_cast<QComboBox*>(cellWidget(r, carrierColumn()))) {
        p.carrier = combo->currentData().toString().toStdString();
    }
    if (placement_) {
        p.offset = Vec3(cellNumber(this, r, 2), cellNumber(this, r, 3), cellNumber(this, r, 4));
        p.rotation = Vec3(cellNumber(this, r, 5), cellNumber(this, r, 6), cellNumber(this, r, 7));
    }
    if (const QTableWidgetItem* c = item(r, colourColumn())) {
        const QColor colour(c->text().trimmed());
        if (colour.isValid()) {
            p.colour = static_cast<std::uint32_t>(colour.rgb()) & 0xFFFFFFu;
        }
    }
    if (const QCheckBox* c = checkIn(this, r, collidesColumn())) {
        p.collides = c->isChecked();
    }
    if (const QCheckBox* c = checkIn(this, r, shownColumn())) {
        p.shown = c->isChecked();
    }
    if (const QTableWidgetItem* n = item(r, noteColumn())) {
        p.note = n->text();
    }
    return p;
}

std::vector<int> PartTable::selectedRows() const {
    std::vector<int> rows;
    for (const QModelIndex& index : selectionModel()->selectedRows()) {
        rows.push_back(index.row());
    }
    std::sort(rows.begin(), rows.end());
    return rows;
}

void PartTable::removeRows(std::vector<int> rows) {
    std::sort(rows.begin(), rows.end());
    rows.erase(std::unique(rows.begin(), rows.end()), rows.end());
    for (auto it = rows.rbegin(); it != rows.rend(); ++it) {
        removeRow(*it);
    }
}

} // namespace ocam::gui
