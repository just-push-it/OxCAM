// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ViewColors.hpp"

#include <QSettings>

namespace ocam::gui {
namespace {

QString settingsKey(const char* key) {
    return QStringLiteral("colors/") + QString::fromLatin1(key);
}

} // namespace

QColor ViewColors::forMove(toolpath::FeedClass feedClass) const {
    using toolpath::FeedClass;
    switch (feedClass) {
    case FeedClass::Rapid:
        return rapid;
    case FeedClass::Cut:
        return cut;
    case FeedClass::Plunge:
        return plunge;
    case FeedClass::Ramp:
        return ramp;
    case FeedClass::LeadIn:
    case FeedClass::LeadOut:
        return lead;
    case FeedClass::Link:
        return link;
    case FeedClass::Probe:
        return probe;
    }
    return cut;
}

const std::vector<ViewColors::Entry>& ViewColors::entries() {
    static const std::vector<Entry> all = {
        {"part", QT_TRANSLATE_NOOP("ViewColors", "Model"), QT_TRANSLATE_NOOP("ViewColors", "Part"),
         &ViewColors::part},
        {"partThroughStock", QT_TRANSLATE_NOOP("ViewColors", "Model"),
         QT_TRANSLATE_NOOP("ViewColors", "Part under transparent stock"),
         &ViewColors::partThroughStock},
        {"fixture", QT_TRANSLATE_NOOP("ViewColors", "Model"),
         QT_TRANSLATE_NOOP("ViewColors", "Fixtures"), &ViewColors::fixture},
        {"stock", QT_TRANSLATE_NOOP("ViewColors", "Stock"),
         QT_TRANSLATE_NOOP("ViewColors", "Remaining stock (simulation)"), &ViewColors::stock},
        {"stockBox", QT_TRANSLATE_NOOP("ViewColors", "Stock"),
         QT_TRANSLATE_NOOP("ViewColors", "Setup stock"), &ViewColors::stockBox},
        {"rapid", QT_TRANSLATE_NOOP("ViewColors", "Toolpaths"),
         QT_TRANSLATE_NOOP("ViewColors", "Rapid moves"), &ViewColors::rapid},
        {"cut", QT_TRANSLATE_NOOP("ViewColors", "Toolpaths"),
         QT_TRANSLATE_NOOP("ViewColors", "Cutting moves"), &ViewColors::cut},
        {"plunge", QT_TRANSLATE_NOOP("ViewColors", "Toolpaths"),
         QT_TRANSLATE_NOOP("ViewColors", "Plunges"), &ViewColors::plunge},
        {"ramp", QT_TRANSLATE_NOOP("ViewColors", "Toolpaths"),
         QT_TRANSLATE_NOOP("ViewColors", "Ramps and helixes"), &ViewColors::ramp},
        {"lead", QT_TRANSLATE_NOOP("ViewColors", "Toolpaths"),
         QT_TRANSLATE_NOOP("ViewColors", "Lead-ins and lead-outs"), &ViewColors::lead},
        {"link", QT_TRANSLATE_NOOP("ViewColors", "Toolpaths"),
         QT_TRANSLATE_NOOP("ViewColors", "Links"), &ViewColors::link},
        {"probe", QT_TRANSLATE_NOOP("ViewColors", "Toolpaths"),
         QT_TRANSLATE_NOOP("ViewColors", "Probe moves"), &ViewColors::probe},
        {"selected", QT_TRANSLATE_NOOP("ViewColors", "Selection"),
         QT_TRANSLATE_NOOP("ViewColors", "Selected faces and edges"), &ViewColors::selected},
        {"hover", QT_TRANSLATE_NOOP("ViewColors", "Selection"),
         QT_TRANSLATE_NOOP("ViewColors", "Under the cursor"), &ViewColors::hover},
        {"backgroundTop", QT_TRANSLATE_NOOP("ViewColors", "Background"),
         QT_TRANSLATE_NOOP("ViewColors", "Top"), &ViewColors::backgroundTop},
        {"backgroundBottom", QT_TRANSLATE_NOOP("ViewColors", "Background"),
         QT_TRANSLATE_NOOP("ViewColors", "Bottom"), &ViewColors::backgroundBottom},
    };
    return all;
}

ViewColors ViewColors::load() {
    ViewColors colors;
    const QSettings settings;
    for (const Entry& e : entries()) {
        const QColor saved(settings.value(settingsKey(e.key)).toString());
        if (saved.isValid()) {
            colors.*e.field = saved;
        }
    }
    return colors;
}

void ViewColors::save() const {
    QSettings settings;
    const ViewColors defaults;
    for (const Entry& e : entries()) {
        // Only colours changed from the defaults, so new defaults reach everyone else.
        if (this->*e.field == defaults.*e.field) {
            settings.remove(settingsKey(e.key));
        } else {
            settings.setValue(settingsKey(e.key), (this->*e.field).name());
        }
    }
}

} // namespace ocam::gui
