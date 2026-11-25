// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// A lathe setup's turret in a window of its own: the stations' blocks and tools on the left, the
// turret preview on the right (DL-0068). It follows the project as it changes, and the row chosen
// in the table is the station the preview shows.

#pragma once

#include <ocam/foundation/Uuid.hpp>

#include <QWidget>

class QSplitter;

namespace ocam::document {
class Document;
}

namespace ocam::gui {

class PropertyPanel;
class TurretPreview;

class TurretWindow : public QWidget {
    Q_OBJECT

public:
    explicit TurretWindow(QWidget* parent = nullptr);

    void setDocument(document::Document* doc);
    /// Shows the window for a setup, its station chosen, and brings it to the front.
    void showSetup(const Uuid& setupId, int station);
    /// Rebuilds the table and the preview from the project, if shown.
    void refresh();

    const Uuid& setupId() const noexcept { return setupId_; }
    PropertyPanel* panel() const noexcept { return panel_; }
    TurretPreview* preview() const noexcept { return preview_; }

protected:
    void showEvent(QShowEvent* event) override;
    void closeEvent(QCloseEvent* event) override;

private:
    document::Document* doc_ = nullptr;
    Uuid setupId_;
    QSplitter* split_ = nullptr;
    PropertyPanel* panel_ = nullptr;
    TurretPreview* preview_ = nullptr;
    bool sized_ = false;
};

} // namespace ocam::gui
