// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "OperationDefaults.hpp"

#include <ocam/strategies/Common.hpp>

#include <QSettings>
#include <algorithm>

namespace ocam::gui {
namespace {

QString settingsKey(std::string_view type) {
    return QStringLiteral("operationDefaults/") +
           QString::fromUtf8(type.data(), static_cast<int>(type.size()));
}

} // namespace

bool validParameterValue(std::string_view type, const std::string& key, const Json& value) {
    const strategies::StrategyInfo* info = strategies::findStrategy(type);
    const strategies::ParamDef* def = info ? info->find(key) : nullptr;
    if (!def) {
        return false;
    }
    switch (def->kind) {
    case strategies::ParamKind::Boolean:
        return value.is_boolean();
    case strategies::ParamKind::Choice:
        return value.is_string() &&
               std::any_of(def->choices.begin(), def->choices.end(),
                           [&](const auto& c) { return c.first == value.get<std::string>(); });
    default:
        return value.is_number() && value.get<double>() >= def->min &&
               value.get<double>() <= def->max;
    }
}

Json operationDefaults(std::string_view type) {
    Json values = Json::object();
    const QByteArray text = QSettings().value(settingsKey(type)).toString().toUtf8();
    if (text.isEmpty()) {
        return values;
    }
    const Json stored = Json::parse(text.constData(), text.constData() + text.size(), nullptr,
                                    /*allow_exceptions=*/false);
    if (!stored.is_object()) {
        return values;
    }
    for (const auto& [key, value] : stored.items()) {
        if (validParameterValue(type, key, value)) {
            values[key] = value;
        }
    }
    return values;
}

void setOperationDefaults(std::string_view type, const Json& values) {
    QSettings settings;
    if (!values.is_object() || values.empty()) {
        settings.remove(settingsKey(type));
    } else {
        settings.setValue(settingsKey(type), QString::fromStdString(values.dump()));
    }
}

void migrateOperationDefaults() {
    QSettings settings;
    const QString legacy = QStringLiteral("smoothingDefault");
    if (!settings.contains(legacy)) {
        return;
    }
    const bool on = settings.value(legacy, true).toBool();
    settings.remove(legacy);
    if (on) {
        return; // the strategies' own default
    }
    for (const strategies::StrategyInfo& info : strategies::strategies()) {
        if (!info.find("smoothing")) {
            continue;
        }
        Json values = operationDefaults(info.id);
        if (!values.contains("smoothing")) {
            values["smoothing"] = false;
            setOperationDefaults(info.id, values);
        }
    }
}

} // namespace ocam::gui
