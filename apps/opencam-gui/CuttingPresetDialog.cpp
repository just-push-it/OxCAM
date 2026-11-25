// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "CuttingPresetDialog.hpp"

#include "Translations.hpp"
#include "Units.hpp"

#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QSplitter>
#include <QVBoxLayout>
#include <cmath>
#include <functional>

namespace ocam::gui {
namespace {

QString qs(const std::string& s) {
    return QString::fromStdString(s);
}

std::string ss(const QString& s) {
    return s.toStdString();
}

/// A spin box with a field and a note after it (a ratio, a hint), as one form row.
QWidget* withNote(QWidget* field, QLabel* note) {
    auto* row = new QWidget;
    auto* layout = new QHBoxLayout(row);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(field, 1);
    note->setMinimumWidth(110);
    layout->addWidget(note);
    return row;
}

QDoubleSpinBox* spin(double max, int decimals, const QString& suffix) {
    auto* s = new QDoubleSpinBox;
    s->setRange(0.0, max);
    s->setDecimals(decimals);
    s->setSuffix(suffix);
    s->setKeyboardTracking(false);
    s->setMinimumWidth(140);
    return s;
}

} // namespace

CuttingPresetDialog::CuttingPresetDialog(const tooling::Tool& tool, int index, QWidget* parent)
    : QDialog(parent), tool_(tool), turning_(tooling::isTurningTool(tool.type)) {
    const QString toolName = tr("T%1 %2").arg(tool.number).arg(qs(tool.name));
    setWindowTitle(tr("Cutting Presets - %1").arg(toolName));
    resize(860, 560);
    const LengthUnit unit = tool_.unit;
    const QString len = lengthSuffix(unit);
    const int lenDecimals = unit == LengthUnit::Inch ? 5 : 4;

    auto* root = new QVBoxLayout(this);
    auto* intro = new QLabel(
        turning_ ? tr("Cutting data for %1, one preset per workpiece material or job. Operations "
                      "take their speeds and feeds from the preset they choose.")
                       .arg(toolName)
                 : tr("Cutting data for %1 (Ø%2, %n flute(s)), one preset per workpiece material "
                      "or job. Operations take their speeds, feeds and depths from the preset "
                      "they choose.",
                      nullptr, tool.mill.flutes)
                       .arg(toolName, formatLength(tool.mill.diameter, unit)));
    intro->setWordWrap(true);
    root->addWidget(intro);

    auto* splitter = new QSplitter;
    root->addWidget(splitter, 1);

    // Left: the presets.
    auto* left = new QWidget;
    auto* leftLayout = new QVBoxLayout(left);
    leftLayout->setContentsMargins(0, 0, 0, 0);
    list_ = new QListWidget;
    leftLayout->addWidget(list_, 1);
    auto* listButtons = new QHBoxLayout;
    auto* add = new QPushButton(tr("Add"));
    duplicate_ = new QPushButton(tr("Duplicate"));
    remove_ = new QPushButton(tr("Delete"));
    up_ = new QPushButton(QStringLiteral("↑"));
    down_ = new QPushButton(QStringLiteral("↓"));
    up_->setToolTip(tr("Move up: the first preset is the one new operations choose"));
    down_->setToolTip(tr("Move down"));
    up_->setMaximumWidth(36);
    down_->setMaximumWidth(36);
    for (QPushButton* b : {add, duplicate_, remove_, up_, down_}) {
        listButtons->addWidget(b);
    }
    leftLayout->addLayout(listButtons);
    splitter->addWidget(left);

    // Right: the form.
    form_ = new QWidget;
    auto* formLayout = new QVBoxLayout(form_);
    formLayout->setContentsMargins(0, 0, 0, 0);

    auto* general = new QGroupBox(tr("Preset"));
    auto* generalForm = new QFormLayout(general);
    name_ = new QLineEdit;
    name_->setPlaceholderText(tr("For example Aluminium roughing"));
    generalForm->addRow(tr("Name"), name_);
    material_ = new QComboBox;
    material_->setEditable(true);
    material_->addItem(QString());
    const std::pair<const char*, const char*> groups[] = {
        {"P", QT_TR_NOOP("Steel")},
        {"M", QT_TR_NOOP("Stainless steel")},
        {"K", QT_TR_NOOP("Cast iron")},
        {"N", QT_TR_NOOP("Non-ferrous: aluminium, copper, plastics")},
        {"S", QT_TR_NOOP("Heat-resistant alloys, titanium")},
        {"H", QT_TR_NOOP("Hardened steel")},
    };
    for (const auto& [letter, text] : groups) {
        material_->addItem(QStringLiteral("%1 - %2").arg(QLatin1String(letter), tr(text)),
                           QLatin1String(letter));
    }
    material_->setToolTip(tr("The workpiece material: an ISO 513 group, or your own text"));
    generalForm->addRow(tr("Material"), material_);
    coolant_ = new QComboBox;
    const std::pair<tooling::Coolant, QString> coolants[] = {
        {tooling::Coolant::Off, tr("Off")},
        {tooling::Coolant::Flood, tr("Flood")},
        {tooling::Coolant::Mist, tr("Mist")},
        {tooling::Coolant::Air, tr("Air blast")},
        {tooling::Coolant::ThroughTool, tr("Through the tool")},
    };
    for (const auto& [c, label] : coolants) {
        coolant_->addItem(label, static_cast<int>(c));
    }
    generalForm->addRow(tr("Coolant"), coolant_);
    formLayout->addWidget(general);

    auto* speeds = new QGroupBox(tr("Speed and feed"));
    speedForm_ = new QFormLayout(speeds);
    surfaceSpeed_ = spin(100000.0, 1, surfaceSpeedSuffix(unit));
    surfaceSpeed_->setToolTip(tr("Cutting speed vc at the tool's diameter"));
    speedForm_->addRow(tr("Cutting speed"), surfaceSpeed_);
    spindle_ = spin(1000000.0, 0, tr(" rpm"));
    spindle_->setToolTip(tr("Spindle speed n for this tool's diameter; typing it sets the "
                            "cutting speed"));
    speedForm_->addRow(tr("Spindle speed"), spindle_);
    feedPerTooth_ = spin(100.0, lenDecimals, len);
    feedPerTooth_->setToolTip(turning_ ? tr("Feed per revolution f")
                                       : tr("Feed per tooth fz (chip load)"));
    speedForm_->addRow(turning_ ? tr("Feed per revolution") : tr("Feed per tooth"), feedPerTooth_);
    leadFeedPerRev_ = spin(100.0, lenDecimals, len);
    leadFeedPerRev_->setToolTip(tr("Feed per revolution of the lead-in and lead-out moves on and "
                                   "off the cut"));
    leadPerRevShare_ = new QLabel;
    speedForm_->addRow(tr("Lead feed per revolution"), withNote(leadFeedPerRev_, leadPerRevShare_));
    partOffEndFeed_ = spin(100.0, lenDecimals, len);
    partOffEndFeed_->setToolTip(tr("Feed per revolution over the end of a part-off, as the part "
                                   "comes off; the operation sets over how much"));
    partOffShare_ = new QLabel;
    speedForm_->addRow(tr("Part-off end feed"), withNote(partOffEndFeed_, partOffShare_));
    feedRate_ = spin(1000000.0, 1, unit == LengthUnit::Inch ? tr(" in/min") : tr(" mm/min"));
    feedRate_->setToolTip(tr("Feed rate vf = fz × flutes × n; typing it sets the feed per tooth"));
    speedForm_->addRow(tr("Feed rate"), feedRate_);
    plungeFeed_ = spin(1000000.0, 1, unit == LengthUnit::Inch ? tr(" in/min") : tr(" mm/min"));
    plungeFeed_->setToolTip(tr("Feed rate of plunges straight down into the material"));
    plungeShare_ = new QLabel;
    speedForm_->addRow(tr("Plunge feed"), withNote(plungeFeed_, plungeShare_));
    rampFeed_ = spin(1000000.0, 1, unit == LengthUnit::Inch ? tr(" in/min") : tr(" mm/min"));
    rampFeed_->setToolTip(tr("Feed rate of ramps and helixes into the material"));
    rampShare_ = new QLabel;
    speedForm_->addRow(tr("Ramp feed"), withNote(rampFeed_, rampShare_));
    leadFeed_ = spin(1000000.0, 1, unit == LengthUnit::Inch ? tr(" in/min") : tr(" mm/min"));
    leadFeed_->setToolTip(tr("Feed rate of the lead-in and lead-out moves on and off the wall"));
    leadShare_ = new QLabel;
    speedForm_->addRow(tr("Lead feed"), withNote(leadFeed_, leadShare_));
    pullFeed_ = spin(1000000.0, 1, unit == LengthUnit::Inch ? tr(" in/min") : tr(" mm/min"));
    pullFeed_->setToolTip(tr("Feed rate of a bar pull, the chuck open"));
    speedForm_->addRow(tr("Pull feed"), pullFeed_);
    rampAngle_ = spin(90.0, 2, QStringLiteral(" °"));
    rampAngle_->setToolTip(tr("Steepest ramp or helix entry into material"));
    speedForm_->addRow(tr("Ramp angle"), rampAngle_);
    formLayout->addWidget(speeds);

    auto* depths = new QGroupBox(tr("Depths of cut"));
    depthForm_ = new QFormLayout(depths);
    axialDepth_ = spin(10000.0, lenDecimals, len);
    axialDepth_->setSpecialValueText(tr("Not set"));
    axialDepth_->setToolTip(tr("Axial depth ap: the step down of roughing operations"));
    axialRatio_ = new QLabel;
    depthForm_->addRow(turning_ ? tr("Depth of cut") : tr("Axial (step down)"),
                       withNote(axialDepth_, axialRatio_));
    radialDepth_ = spin(10000.0, lenDecimals, len);
    radialDepth_->setSpecialValueText(tr("Not set"));
    radialDepth_->setToolTip(tr("Radial depth ae: the stepover of clearing operations"));
    radialRatio_ = new QLabel;
    depthForm_->addRow(tr("Radial (stepover)"), withNote(radialDepth_, radialRatio_));
    removalRate_ = new QLabel;
    depthForm_->addRow(tr("Removal rate"), removalRate_);
    formLayout->addWidget(depths);

    problems_ = new QLabel;
    problems_->setWordWrap(true);
    problems_->setStyleSheet(QStringLiteral("color: #c0392b;"));
    formLayout->addWidget(problems_);
    formLayout->addStretch();
    splitter->addWidget(form_);
    splitter->setSizes({260, 600});

    // Turning: the speed is held constant at the surface, so there is no one spindle speed or
    // feed rate, and no stepover, ramp or plunge.
    for (QWidget* w : {static_cast<QWidget*>(spindle_), static_cast<QWidget*>(feedRate_),
                       plungeFeed_->parentWidget(), rampFeed_->parentWidget(),
                       leadFeed_->parentWidget(), static_cast<QWidget*>(rampAngle_)}) {
        speedForm_->setRowVisible(w, !turning_);
    }
    speedForm_->setRowVisible(leadFeedPerRev_->parentWidget(), leadsPerRev());
    speedForm_->setRowVisible(partOffEndFeed_->parentWidget(), partsOff());
    depthForm_->setRowVisible(radialDepth_->parentWidget(), !turning_);
    depthForm_->setRowVisible(removalRate_, !turning_);
    // A bar puller (DL-0091) cuts nothing: onto the bar at the plunge feed, and the pull.
    const bool puller = tool_.type == tooling::ToolType::BarPuller;
    speedForm_->setRowVisible(pullFeed_, puller);
    if (puller) {
        for (QWidget* w : {static_cast<QWidget*>(surfaceSpeed_), static_cast<QWidget*>(spindle_),
                           static_cast<QWidget*>(feedPerTooth_), static_cast<QWidget*>(feedRate_),
                           rampFeed_->parentWidget(), leadFeed_->parentWidget(),
                           static_cast<QWidget*>(rampAngle_)}) {
            speedForm_->setRowVisible(w, false);
        }
        if (auto* label =
                qobject_cast<QLabel*>(speedForm_->labelForField(plungeFeed_->parentWidget()))) {
            label->setText(tr("Feed onto the bar"));
        }
        depths->setVisible(false);
    }

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    root->addWidget(buttons);

    // List actions.
    connect(list_, &QListWidget::currentRowChanged, this, [this](int row) { select(row); });
    connect(add, &QPushButton::clicked, this, &CuttingPresetDialog::addPreset);
    connect(duplicate_, &QPushButton::clicked, this, [this] {
        const tooling::CuttingPreset* p = current();
        if (!p) {
            return;
        }
        tooling::CuttingPreset copy = *p;
        copy.id = Uuid::generate();
        copy.name += ss(tr(" (copy)"));
        tool_.presets.insert(tool_.presets.begin() + current_ + 1, copy);
        rebuildList();
        select(current_ + 1);
    });
    connect(remove_, &QPushButton::clicked, this, [this] {
        if (!current()) {
            return;
        }
        tool_.presets.erase(tool_.presets.begin() + current_);
        rebuildList();
        select(std::min(current_, static_cast<int>(tool_.presets.size()) - 1));
    });
    auto move = [this](int by) {
        const int to = current_ + by;
        if (!current() || to < 0 || to >= static_cast<int>(tool_.presets.size())) {
            return;
        }
        std::swap(tool_.presets[static_cast<std::size_t>(current_)],
                  tool_.presets[static_cast<std::size_t>(to)]);
        rebuildList();
        select(to);
    };
    connect(up_, &QPushButton::clicked, this, [move] { move(-1); });
    connect(down_, &QPushButton::clicked, this, [move] { move(1); });

    // Field edits go straight into the preset.
    auto edit = [this](const std::function<void(tooling::CuttingPreset&)>& change) {
        tooling::CuttingPreset* p = current();
        if (loading_ || !p) {
            return;
        }
        change(*p);
        refreshItem();
        refreshResults(false);
    };
    connect(name_, &QLineEdit::textEdited, this,
            [this, edit](const QString& t) { edit([&](auto& p) { p.name = ss(t); }); });
    connect(material_, &QComboBox::currentTextChanged, this, [this, edit] {
        // A listed group stores its letter; anything typed is kept as it is.
        const int i = material_->currentIndex();
        const QString value = i > 0 && material_->currentText() == material_->itemText(i)
                                  ? material_->itemData(i).toString()
                                  : material_->currentText().trimmed();
        edit([&](auto& p) { p.materialGroup = ss(value); });
    });
    connect(coolant_, &QComboBox::currentIndexChanged, this, [this, edit] {
        edit([&](auto& p) {
            p.coolant = static_cast<tooling::Coolant>(coolant_->currentData().toInt());
        });
    });
    const double d = tool_.mill.diameter;
    const int z = std::max(tool_.mill.flutes, 0);
    connect(surfaceSpeed_, &QDoubleSpinBox::valueChanged, this, [this, edit, unit](double v) {
        edit([&](auto& p) { p.surfaceSpeed = surfaceSpeedToMmPerSecond(v, unit); });
    });
    connect(spindle_, &QDoubleSpinBox::valueChanged, this, [edit, d](double rpm) {
        if (d > 0.0) {
            edit([&](auto& p) { p.surfaceSpeed = rpm / 60.0 * kPi * d; });
        }
    });
    connect(feedPerTooth_, &QDoubleSpinBox::valueChanged, this, [this, edit](double v) {
        edit([&](auto& p) { p.feedPerTooth = v * lengthFactor(); });
    });
    connect(leadFeedPerRev_, &QDoubleSpinBox::valueChanged, this, [this, edit](double v) {
        edit([&](auto& p) { p.leadFeedPerRev = v * lengthFactor(); });
    });
    connect(partOffEndFeed_, &QDoubleSpinBox::valueChanged, this, [this, edit](double v) {
        edit([&](auto& p) { p.partOffEndFeed = v * lengthFactor(); });
    });
    connect(feedRate_, &QDoubleSpinBox::valueChanged, this, [this, edit, d, z](double v) {
        edit([&](auto& p) {
            const double n = p.spindleSpeed(d);
            if (n > 0.0 && z > 0) {
                p.feedPerTooth = v * lengthFactor() / 60.0 / (z * n);
            }
        });
    });
    connect(plungeFeed_, &QDoubleSpinBox::valueChanged, this, [this, edit](double v) {
        edit([&](auto& p) { p.plungeFeed = v * lengthFactor() / 60.0; });
    });
    connect(rampFeed_, &QDoubleSpinBox::valueChanged, this, [this, edit](double v) {
        edit([&](auto& p) { p.rampFeed = v * lengthFactor() / 60.0; });
    });
    connect(leadFeed_, &QDoubleSpinBox::valueChanged, this, [this, edit](double v) {
        edit([&](auto& p) { p.leadFeed = v * lengthFactor() / 60.0; });
    });
    connect(pullFeed_, &QDoubleSpinBox::valueChanged, this, [this, edit](double v) {
        edit([&](auto& p) { p.pullFeed = v * lengthFactor() / 60.0; });
    });
    connect(rampAngle_, &QDoubleSpinBox::valueChanged, this,
            [edit](double v) { edit([&](auto& p) { p.rampAngle = degToRad(v); }); });
    connect(axialDepth_, &QDoubleSpinBox::valueChanged, this,
            [this, edit](double v) { edit([&](auto& p) { p.axialDepth = v * lengthFactor(); }); });
    connect(radialDepth_, &QDoubleSpinBox::valueChanged, this,
            [this, edit](double v) { edit([&](auto& p) { p.radialDepth = v * lengthFactor(); }); });

    rebuildList();
    select(index >= 0 && index < static_cast<int>(tool_.presets.size()) ? index
           : tool_.presets.empty()                                      ? -1
                                                                        : 0);
}

bool CuttingPresetDialog::partsOff() const {
    return tool_.type == tooling::ToolType::PartingInsert ||
           tool_.type == tooling::ToolType::GroovingInsert;
}

bool CuttingPresetDialog::leadsPerRev() const {
    return tool_.type == tooling::ToolType::TurningInsert;
}

double CuttingPresetDialog::lengthFactor() const {
    return tool_.unit == LengthUnit::Inch ? 25.4 : 1.0;
}

tooling::CuttingPreset* CuttingPresetDialog::current() {
    if (current_ < 0 || current_ >= static_cast<int>(tool_.presets.size())) {
        return nullptr;
    }
    return &tool_.presets[static_cast<std::size_t>(current_)];
}

void CuttingPresetDialog::addPreset() {
    tooling::CuttingPreset p = tooling::defaultPreset(tool_);
    p.id = Uuid::generate();
    p.name = ss(tr("New preset"));
    tool_.presets.push_back(p);
    rebuildList();
    select(static_cast<int>(tool_.presets.size()) - 1);
    name_->setFocus();
    name_->selectAll();
}

void CuttingPresetDialog::rebuildList() {
    const QSignalBlocker block(list_);
    list_->clear();
    for (std::size_t i = 0; i < tool_.presets.size(); ++i) {
        list_->addItem(QString());
    }
    const int keep = current_;
    for (int r = 0; r < list_->count(); ++r) {
        current_ = r;
        refreshItem();
    }
    current_ = keep;
}

void CuttingPresetDialog::refreshItem() {
    const tooling::CuttingPreset* p = current();
    QListWidgetItem* item = p ? list_->item(current_) : nullptr;
    if (!item) {
        return;
    }
    QString text = p->name.empty() ? tr("(unnamed)") : qs(p->name);
    if (!p->materialGroup.empty()) {
        text += QStringLiteral("  [%1]").arg(qs(p->materialGroup));
    }
    item->setText(text);
}

void CuttingPresetDialog::select(int row) {
    current_ = row;
    if (row >= 0 && list_->currentRow() != row) {
        const QSignalBlocker block(list_);
        list_->setCurrentRow(row);
    }
    const bool any = current() != nullptr;
    form_->setEnabled(any);
    for (QPushButton* b : {duplicate_, remove_, up_, down_}) {
        b->setEnabled(any);
    }
    load();
}

void CuttingPresetDialog::load() {
    loading_ = true;
    const tooling::CuttingPreset* p = current();
    const tooling::CuttingPreset shown = p ? *p : tooling::CuttingPreset{};
    name_->setText(p ? qs(shown.name) : QString());
    const int group = material_->findData(qs(shown.materialGroup));
    if (group > 0) {
        material_->setCurrentIndex(group);
    } else {
        material_->setCurrentIndex(0);
        material_->setEditText(qs(shown.materialGroup));
    }
    coolant_->setCurrentIndex(coolant_->findData(static_cast<int>(shown.coolant)));
    rampAngle_->setValue(radToDeg(shown.rampAngle));
    axialDepth_->setValue(shown.axialDepth / lengthFactor());
    radialDepth_->setValue(shown.radialDepth / lengthFactor());
    loading_ = false;
    refreshResults(true);
}

void CuttingPresetDialog::refreshResults(bool all) {
    // The speed and feed fields follow the preset, set without signals; while one is being
    // typed in it is left alone (it holds what was typed).
    const tooling::CuttingPreset* p = current();
    const tooling::CuttingPreset shown = p ? *p : tooling::CuttingPreset{};
    const LengthUnit unit = tool_.unit;
    const double d = tool_.mill.diameter;
    const int z = std::max(tool_.mill.flutes, 0);
    const double n = shown.spindleSpeed(d); // rev/s
    const double vf = shown.feedRate(d, z); // mm/s
    auto set = [all](QDoubleSpinBox* box, double value) {
        if (all || !box->hasFocus()) {
            const QSignalBlocker block(box);
            box->setValue(value);
        }
    };
    set(surfaceSpeed_, surfaceSpeedIn(shown.surfaceSpeed, unit));
    set(spindle_, n * 60.0);
    set(feedPerTooth_, shown.feedPerTooth / lengthFactor());
    set(leadFeedPerRev_, shown.leadFeedPerRev / lengthFactor());
    set(partOffEndFeed_, shown.partOffEndFeed / lengthFactor());
    set(feedRate_, vf * 60.0 / lengthFactor());
    set(plungeFeed_, shown.plungeFeed * 60.0 / lengthFactor());
    set(rampFeed_, shown.rampFeed * 60.0 / lengthFactor());
    set(leadFeed_, shown.leadFeed * 60.0 / lengthFactor());
    set(pullFeed_, shown.pullFeed * 60.0 / lengthFactor());

    plungeShare_->setText(vf > 0.0 && shown.plungeFeed > 0.0
                              ? tr("%1 % of the feed").arg(100.0 * shown.plungeFeed / vf, 0, 'f', 0)
                              : QString());
    rampShare_->setText(vf > 0.0 && shown.rampFeed > 0.0
                            ? tr("%1 % of the feed").arg(100.0 * shown.rampFeed / vf, 0, 'f', 0)
                            : QString());
    leadShare_->setText(vf > 0.0 && shown.leadFeed > 0.0
                            ? tr("%1 % of the feed").arg(100.0 * shown.leadFeed / vf, 0, 'f', 0)
                            : QString());
    leadPerRevShare_->setText(
        shown.feedPerTooth > 0.0 && shown.leadFeedPerRev > 0.0
            ? tr("%1 % of the feed")
                  .arg(100.0 * shown.leadFeedPerRev / shown.feedPerTooth, 0, 'f', 0)
            : QString());
    partOffShare_->setText(
        shown.feedPerTooth > 0.0 && shown.partOffEndFeed > 0.0
            ? tr("%1 % of the feed")
                  .arg(100.0 * shown.partOffEndFeed / shown.feedPerTooth, 0, 'f', 0)
            : QString());
    axialRatio_->setText(d > 0.0 && shown.axialDepth > 0.0 && !turning_
                             ? tr("%1 × D").arg(shown.axialDepth / d, 0, 'f', 2)
                             : QString());
    radialRatio_->setText(d > 0.0 && shown.radialDepth > 0.0
                              ? tr("%1 % of D").arg(100.0 * shown.radialDepth / d, 0, 'f', 0)
                              : QString());
    if (shown.axialDepth > 0.0 && shown.radialDepth > 0.0 && vf > 0.0) {
        const double q = shown.axialDepth * shown.radialDepth * vf * 60.0; // mm³/min
        removalRate_->setText(unit == LengthUnit::Inch
                                  ? tr("%1 in³/min").arg(q / (25.4 * 25.4 * 25.4), 0, 'f', 2)
                                  : tr("%1 cm³/min").arg(q / 1000.0, 0, 'f', 1));
    } else {
        removalRate_->setText(tr("Set both depths to see it"));
    }

    QStringList problems;
    if (p && tool_.type == tooling::ToolType::BarPuller) {
        if (shown.plungeFeed <= 0.0) {
            problems << tr("No feed onto the bar: bar pulls will not generate.");
        }
        if (shown.pullFeed <= 0.0) {
            problems << tr("No pull feed: bar pulls will not generate.");
        }
    } else if (p) {
        if (shown.surfaceSpeed <= 0.0) {
            problems << tr("No cutting speed: operations cannot use this preset.");
        }
        const bool pitchFeed = tool_.type == tooling::ToolType::Tap ||
                               tool_.type == tooling::ToolType::ThreadingInsert;
        if (shown.feedPerTooth <= 0.0 && !pitchFeed) {
            problems << (turning_ ? tr("No feed per revolution: operations cannot use it.")
                                  : tr("No feed per tooth: operations cannot use it."));
        }
        if (leadsPerRev() && shown.leadFeedPerRev <= 0.0) {
            problems << tr("No lead feed per revolution: facing, roughing and finishing will not "
                           "generate.");
        }
        if (partsOff() && shown.partOffEndFeed <= 0.0) {
            problems << tr("No part-off end feed: part-offs with an end distance will not "
                           "generate.");
        }
        if (!turning_ && shown.plungeFeed <= 0.0 && !pitchFeed) {
            problems << tr("No plunge feed: operations that plunge into material will not "
                           "generate.");
        }
        if (!turning_ && shown.rampFeed <= 0.0 && !pitchFeed) {
            problems << tr("No ramp feed: operations that ramp or helix into material will not "
                           "generate.");
        }
        if (!turning_ && shown.leadFeed <= 0.0 && !pitchFeed) {
            problems << tr("No lead feed: operations with lead moves (contours, chamfers) will "
                           "not generate.");
        }
        if (!turning_ && d > 0.0 && shown.radialDepth > d + 1e-9) {
            problems << tr("The stepover is wider than the tool.");
        }
    }
    problems_->setText(problems.join(QLatin1Char('\n')));
}

} // namespace ocam::gui
