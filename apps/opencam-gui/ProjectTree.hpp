// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <ocam/document/Project.hpp>

#include <QIcon>
#include <QTimer>
#include <QTreeWidget>
#include <functional>

namespace ocam::gui {

struct Selection {
    enum class Kind { None, Project, Model, Tool, Setup, Operation };
    Kind kind = Kind::None;
    Uuid id;

    bool operator==(const Selection&) const = default;
};

/// Project outline: models, tools, setups and their operations. Rebuilt from the Project value
/// after every change; selection and expansion survive rebuilds.
class ProjectTree : public QTreeWidget {
    Q_OBJECT

public:
    explicit ProjectTree(QWidget* parent = nullptr);

    /// The toolpath state of an operation, shown as an icon in front of its name.
    enum class OperationState { None, Generating, Ready, Warning, Error };
    void setOperationState(std::function<OperationState(const Uuid&)> state);

    void rebuild(const document::Project& project);
    /// Updates the operations' icons only (after a toolpath finished generating).
    void refreshStates();
    Selection selection() const;
    void select(const Selection& selection);
    /// Operations can be dragged to a new place (off while simulating).
    void setReorderEnabled(bool enabled);

signals:
    void currentSelectionChanged(const ocam::gui::Selection& selection);
    /// Right-click on an item (which becomes current) or on a group ("models", "tools",
    /// "setups", with an empty selection), at a global position.
    void contextMenuRequested(const ocam::gui::Selection& selection, const QString& group,
                              const QPoint& globalPos);
    /// An operation was dropped in front of position `index` of a setup's operations (counted
    /// before the move; the setup's count appends). The tree itself does not change.
    void operationMoveRequested(const ocam::Uuid& operation, const ocam::Uuid& setup, int index);

protected:
    void dropEvent(QDropEvent* event) override;

private:
    QIcon stateIcon(OperationState state) const;

    std::function<OperationState(const Uuid&)> state_;
    QTimer* spin_ = nullptr; ///< Turns the "generating" icons.
    int spinStep_ = 0;

    QTreeWidgetItem* addItem(QTreeWidgetItem* parent, const QString& text, Selection::Kind kind,
                             const Uuid& id, const QString& detail = {});
};

} // namespace ocam::gui
