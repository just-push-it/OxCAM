// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "StartHub.hpp"

#include <ocam/foundation/Version.hpp>

#include <QCheckBox>
#include <QDateTime>
#include <QFileInfo>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QMenu>
#include <QPushButton>
#include <QRandomGenerator>
#include <QSettings>
#include <QTextBrowser>
#include <QVBoxLayout>

namespace ocam::gui {
namespace {

const QString kRecentKey = QStringLiteral("recentProjects");
constexpr int kMaxRecent = 12;

const char* const kTips[] = {
    QT_TRANSLATE_NOOP("ocam::gui::StartHub",
                      "Click the faces, edges and corners of the view cube (lower left of the 3D "
                      "view) to look at the part from there. Views follow the selected setup's "
                      "work coordinate system."),
    QT_TRANSLATE_NOOP("ocam::gui::StartHub",
                      "Alt+1 top, Alt+2 front, Alt+3 right, Alt+4 bottom, Alt+5 back, Alt+6 left, "
                      "Alt+7 isometric, Alt+8 rear isometric, Alt+0 fit all. The keypad digits "
                      "work too. F also fits all."),
    QT_TRANSLATE_NOOP("ocam::gui::StartHub",
                      "Right-click a setup in the project tree to add operations, simulate, post "
                      "process, rename or duplicate it."),
    QT_TRANSLATE_NOOP("ocam::gui::StartHub",
                      "Click the face or edge under the cursor (Shift adds more); use faces for an "
                      "operation's geometry or to align a setup's WCS to a planar face."),
    QT_TRANSLATE_NOOP("ocam::gui::StartHub",
                      "Your own machines (kinematics, travel, spindle, feeds, collision parts) "
                      "live in Tools > Machine Library. Setups keep a copy, so projects open "
                      "anywhere."),
    QT_TRANSLATE_NOOP("ocam::gui::StartHub",
                      "F5 simulates the current setup; the simulation panel lists gouges, holder "
                      "and machine collisions. Double-click one to jump to it."),
    QT_TRANSLATE_NOOP("ocam::gui::StartHub",
                      "Ctrl+P posts the current setup with the post processor that fits its "
                      "machine (lathe, 3-, 4- or 5-axis)."),
    QT_TRANSLATE_NOOP("ocam::gui::StartHub",
                      "Every change is undoable (Ctrl+Z). Toolpaths regenerate in the background "
                      "when anything they depend on changes."),
    QT_TRANSLATE_NOOP("ocam::gui::StartHub",
                      "Edit > Preferences binds any command to a key, pins commands to the "
                      "toolbar, and sets the values new operations of each type start with."),
};

const char* const kWhatsNew = QT_TRANSLATE_NOOP(
    "ocam::gui::StartHub",
    "<h3>Multi-axis milling</h3>"
    "<ul>"
    "<li>4th-axis wrapping and wrapped pocketing of turned parts.</li>"
    "<li>Simultaneous 5-axis ball finishing with a tool axis you control (lead/tilt, a direction "
    "or a point).</li>"
    "<li>4- and 5-axis kinematics, posting with tool centre point control or machine positions "
    "with inverse-time feeds, and back-plot verification of tool axes.</li>"
    "<li>Tilted tools in the stock simulation and machine collision checks.</li>"
    "</ul>"
    "<h3>Workspace</h3>"
    "<ul>"
    "<li>Your own keyboard shortcuts, toolbar buttons and starting values for each operation "
    "type (Edit &gt; Preferences).</li>"
    "<li>This start hub with recent projects.</li>"
    "<li>A view cube that follows the selected setup, and Alt+number standard views.</li>"
    "<li>Right-click menus in the project tree.</li>"
    "<li>A machine library for your own machine definitions.</li>"
    "<li>Face picking in the 3D view works again.</li>"
    "</ul>"
    "<h3>Earlier</h3>"
    "<p>Turning on lathes, 3D milling and 3+2 indexing, rest machining, Python bindings, stock "
    "simulation and 2.5D milling.</p>"
    "<p style='color:gray'>OxCAM's G-code has not been proven on a physical controller yet: "
    "dry-run every program.</p>");

QPushButton* bigButton(const QString& text, const QString& detail) {
    auto* b = new QPushButton(QStringLiteral("%1\n%2").arg(text, detail));
    b->setMinimumHeight(54);
    b->setStyleSheet(QStringLiteral("QPushButton { text-align: left; padding: 6px 12px; }"));
    return b;
}

} // namespace

QStringList RecentProjects::list() {
    return QSettings().value(kRecentKey).toStringList();
}

void RecentProjects::add(const QString& path) {
    const QString clean = QFileInfo(path).absoluteFilePath();
    QStringList items = list();
    items.removeAll(clean);
    items.prepend(clean);
    while (items.size() > kMaxRecent) {
        items.removeLast();
    }
    QSettings().setValue(kRecentKey, items);
}

void RecentProjects::remove(const QString& path) {
    QStringList items = list();
    items.removeAll(path);
    QSettings().setValue(kRecentKey, items);
}

bool StartHub::showAtStartup() {
    return QSettings().value(QStringLiteral("showStartHub"), true).toBool();
}

StartHub::StartHub(QWidget* parent) : QDialog(parent) {
    setWindowTitle(tr("Welcome to OxCAM"));
    resize(1000, 600);

    auto* root = new QVBoxLayout(this);
    auto* header = new QLabel(
        tr("<span style='font-size:20pt; font-weight:600'>OxCAM</span>"
           "<span style='font-size:11pt; color:gray'>&nbsp;&nbsp;%1 &mdash; open-source CAM</span>")
            .arg(
                QString::fromUtf8(kVersionString.data(), static_cast<int>(kVersionString.size()))));
    root->addWidget(header);

    auto* columns = new QHBoxLayout;
    root->addLayout(columns, 1);

    // Start actions.
    auto* start = new QVBoxLayout;
    auto* startTitle = new QLabel(tr("<b>Start</b>"));
    start->addWidget(startTitle);
    auto* newProject = bigButton(tr("New project"), tr("An empty project"));
    auto* importStep = bigButton(tr("Import STEP model"), tr("A new project from a model"));
    auto* openProject = bigButton(tr("Open project"), tr("An .ocam project file"));
    auto* tools = bigButton(tr("Tool libraries"), tr("Your cutting tools"));
    auto* machines = bigButton(tr("Machine library"), tr("Your machines and their limits"));
    for (QPushButton* b : {newProject, importStep, openProject, tools, machines}) {
        start->addWidget(b);
    }
    start->addStretch();
    columns->addLayout(start, 3);

    // Recent projects.
    auto* recentColumn = new QVBoxLayout;
    recentColumn->addWidget(new QLabel(tr("<b>Recent projects</b>")));
    recent_ = new QListWidget;
    recent_->setContextMenuPolicy(Qt::CustomContextMenu);
    recent_->setSpacing(2);
    recent_->setTextElideMode(Qt::ElideMiddle);
    recent_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    recentColumn->addWidget(recent_, 1);
    columns->addLayout(recentColumn, 5);

    // Tips and what's new.
    auto* side = new QVBoxLayout;
    side->addWidget(new QLabel(tr("<b>Tip</b>")));
    auto* tipFrame = new QFrame;
    tipFrame->setFrameShape(QFrame::StyledPanel);
    auto* tipLayout = new QVBoxLayout(tipFrame);
    tip_ = new QLabel;
    tip_->setWordWrap(true);
    tip_->setMinimumHeight(70);
    tipLayout->addWidget(tip_);
    auto* nextTip = new QPushButton(tr("Next tip"));
    nextTip->setFlat(true);
    tipLayout->addWidget(nextTip, 0, Qt::AlignRight);
    side->addWidget(tipFrame);
    side->addWidget(new QLabel(tr("<b>What's new</b>")));
    auto* news = new QTextBrowser;
    news->setHtml(tr(kWhatsNew));
    news->setOpenExternalLinks(false);
    side->addWidget(news, 1);
    columns->addLayout(side, 4);

    auto* bottom = new QHBoxLayout;
    showAtStart_ = new QCheckBox(tr("Show this window when OxCAM starts"));
    showAtStart_->setChecked(showAtStartup());
    bottom->addWidget(showAtStart_);
    bottom->addStretch();
    auto* close = new QPushButton(tr("Close"));
    bottom->addWidget(close);
    root->addLayout(bottom);

    connect(newProject, &QPushButton::clicked, this, [this] { finish(Action::NewProject); });
    connect(importStep, &QPushButton::clicked, this, [this] { finish(Action::ImportStep); });
    connect(openProject, &QPushButton::clicked, this, [this] { finish(Action::OpenProject); });
    connect(tools, &QPushButton::clicked, this, [this] { finish(Action::ToolLibraries); });
    connect(machines, &QPushButton::clicked, this, [this] { finish(Action::MachineLibrary); });
    connect(close, &QPushButton::clicked, this, [this] { finish(Action::None); });
    connect(nextTip, &QPushButton::clicked, this, [this] { showTip(tipIndex_ + 1); });
    connect(showAtStart_, &QCheckBox::toggled, this,
            [](bool on) { QSettings().setValue(QStringLiteral("showStartHub"), on); });
    connect(recent_, &QListWidget::itemActivated, this, [this](QListWidgetItem* item) {
        const QString path = item->data(Qt::UserRole).toString();
        if (QFileInfo::exists(path)) {
            finish(Action::OpenRecent, path);
        }
    });
    connect(recent_, &QListWidget::customContextMenuRequested, this, [this](const QPoint& pos) {
        QListWidgetItem* item = recent_->itemAt(pos);
        if (!item) {
            return;
        }
        QMenu menu(this);
        const QString path = item->data(Qt::UserRole).toString();
        QAction* open = menu.addAction(tr("Open"));
        open->setEnabled(QFileInfo::exists(path));
        QAction* forget = menu.addAction(tr("Remove from list"));
        QAction* chosen = menu.exec(recent_->viewport()->mapToGlobal(pos));
        if (chosen == open) {
            finish(Action::OpenRecent, path);
        } else if (chosen == forget) {
            RecentProjects::remove(path);
            fillRecent();
        }
    });

    fillRecent();
    showTip(static_cast<int>(
        QRandomGenerator::global()->bounded(static_cast<quint32>(std::size(kTips)))));
}

void StartHub::fillRecent() {
    recent_->clear();
    const QStringList items = RecentProjects::list();
    for (const QString& path : items) {
        const QFileInfo info(path);
        auto* item = new QListWidgetItem;
        const bool exists = info.exists();
        item->setText(
            exists ? QStringLiteral("%1\n%2   %3")
                         .arg(info.completeBaseName(), info.absolutePath(),
                              info.lastModified().toString(QStringLiteral("yyyy-MM-dd hh:mm")))
                   : tr("%1\n%2   (missing)").arg(info.completeBaseName(), info.absolutePath()));
        item->setData(Qt::UserRole, path);
        item->setToolTip(path);
        if (!exists) {
            item->setForeground(Qt::gray);
        }
        recent_->addItem(item);
    }
    if (items.isEmpty()) {
        auto* item = new QListWidgetItem(tr("No recent projects yet."));
        item->setFlags(Qt::NoItemFlags);
        recent_->addItem(item);
    }
}

void StartHub::showTip(int index) {
    const int n = static_cast<int>(std::size(kTips));
    tipIndex_ = ((index % n) + n) % n;
    tip_->setText(tr(kTips[tipIndex_]));
}

void StartHub::finish(Action action, const QString& path) {
    action_ = action;
    path_ = path;
    if (action == Action::None) {
        reject();
    } else {
        accept();
    }
}

} // namespace ocam::gui
