// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// An operation's right-click menu > Show Toolpath Data: every move of its toolpath in program
// order, as the toolpath holds it. A canned cycle is one row, as it is one move in the toolpath
// and one cycle in the NC program; its numbers are in the row's details. Feeds are the toolpath's
// own values (ADR-0004): nothing here substitutes or works out a feed.

#pragma once

#include <ocam/foundation/Units.hpp>
#include <ocam/toolpath/Toolpath.hpp>

#include <QAbstractTableModel>
#include <QDialog>
#include <memory>

class QLabel;
class QTableView;

namespace ocam::gui {

/// The moves of one toolpath. Rows are formatted when the view asks for them, so the length of
/// the toolpath costs nothing here.
class ToolpathDataModel : public QAbstractTableModel {
    Q_OBJECT

public:
    enum Column {
        Number,
        Type,
        Purpose,
        X,
        Y,
        Z,
        Feed,
        Plane,
        CenterX,
        CenterY,
        CenterZ,
        Radius,
        AxisI,
        AxisJ,
        AxisK,
        Details,
        ColumnCount
    };

    using QAbstractTableModel::QAbstractTableModel;

    void setPath(std::shared_ptr<const toolpath::Toolpath> path, LengthUnit unit);
    const toolpath::Toolpath* path() const noexcept { return path_.get(); }
    LengthUnit unit() const noexcept { return unit_; }

    int rowCount(const QModelIndex& parent = {}) const override;
    int columnCount(const QModelIndex& parent = {}) const override;
    QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
    QVariant headerData(int section, Qt::Orientation orientation,
                        int role = Qt::DisplayRole) const override;

    /// The text of one cell, as the table shows it.
    QString text(int row, Column column) const;

private:
    /// The canned cycle of a Cycle move.
    const toolpath::DrillCycle* cycleOf(const toolpath::Move& move) const;
    /// A length in the display unit, for a column that names the unit in its header.
    QString length(double mm) const;
    QString lengthWithUnit(double mm) const;
    QString details(const toolpath::Move& move) const;

    std::shared_ptr<const toolpath::Toolpath> path_;
    LengthUnit unit_ = LengthUnit::Millimeter;
};

/// A window of its own beside the main one: it stays open while the user works, and MainWindow
/// keeps it up to date as the operation's toolpath changes.
class ToolpathDataDialog : public QDialog {
    Q_OBJECT

public:
    explicit ToolpathDataDialog(QWidget* parent = nullptr);

    void setOperationName(const QString& name);
    /// Lists the toolpath's moves (the scroll position stays when it is the same toolpath).
    void showPath(const std::shared_ptr<const toolpath::Toolpath>& path, LengthUnit unit);
    /// Why there is nothing to list.
    void showMessage(const QString& message);

    const ToolpathDataModel* model() const noexcept { return model_; }
    QTableView* table() const noexcept { return table_; }
    /// The text shown instead of the list, or empty when the list shows.
    QString message() const;

private:
    QString summary(const toolpath::Toolpath& path, LengthUnit unit) const;

    ToolpathDataModel* model_ = nullptr;
    QTableView* table_ = nullptr;
    QLabel* summary_ = nullptr;
    QLabel* message_ = nullptr;
};

} // namespace ocam::gui
