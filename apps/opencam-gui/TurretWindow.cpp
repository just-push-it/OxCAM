// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "TurretWindow.hpp"

#include "ProjectTree.hpp"
#include "PropertyPanel.hpp"
#include "TurretPreview.hpp"

#include <ocam/document/Document.hpp>

#include <QCloseEvent>
#include <QHBoxLayout>
#include <QSplitter>

namespace ocam::gui {

TurretWindow::TurretWindow(QWidget* parent) : QWidget(parent, Qt::Window) {
    setObjectName(QStringLiteral("turretWindow"));
    setWindowTitle(tr("Turret setup"));
    auto* layout = new QHBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    split_ = new QSplitter(Qt::Horizontal, this);
    panel_ = new PropertyPanel;
    panel_->setObjectName(QStringLiteral("turretWindowTable"));
    panel_->setTurretOnly(true);
    preview_ = new TurretPreview;
    split_->addWidget(panel_);
    split_->addWidget(preview_);
    split_->setStretchFactor(0, 0);
    split_->setStretchFactor(1, 1);
    split_->setChildrenCollapsible(false);
    layout->addWidget(split_);
    // The row chosen in the table is the station shown, and the other way round.
    connect(panel_, &PropertyPanel::turretStationSelected, preview_, &TurretPreview::setStation);
    connect(preview_, &TurretPreview::stationChanged, panel_, &PropertyPanel::setTurretStation);
}

void TurretWindow::setDocument(document::Document* doc) {
    doc_ = doc;
    panel_->setDocument(doc);
    if (!doc) {
        hide();
    }
}

void TurretWindow::showSetup(const Uuid& setupId, int station) {
    setupId_ = setupId;
    panel_->setTurretStation(station);
    preview_->setStation(station);
    if (!sized_) {
        sized_ = true;
        // Room for the table and for the turret and the work side by side.
        resize(1280, 780);
        split_->setSizes({500, 780});
    }
    show();
    raise();
    if (!testAttribute(Qt::WA_ShowWithoutActivating)) {
        activateWindow();
    }
    refresh();
}

void TurretWindow::refresh() {
    if (!isVisible() || !doc_) {
        return;
    }
    const document::Setup* setup = doc_->project().findSetup(setupId_);
    if (!setup) {
        // The setup was deleted (or undone away): nothing to set up.
        hide();
        return;
    }
    setWindowTitle(tr("Turret setup: %1").arg(QString::fromStdString(setup->name)));
    panel_->showSelection({Selection::Kind::Setup, setupId_});
    preview_->showSetup(*doc_, setupId_);
}

void TurretWindow::closeEvent(QCloseEvent* event) {
    // Hidden, not closed: closing a window destroys its native windows, the preview's 3D view's
    // among them, and the view keeps drawing into the one it was made for (shown again, it was a
    // white box with the page painted over itself).
    event->ignore();
    hide();
}

void TurretWindow::showEvent(QShowEvent* event) {
    QWidget::showEvent(event);
    refresh();
}

} // namespace ocam::gui
