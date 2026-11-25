// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ParamEditors.hpp"

#include "Translations.hpp"
#include "Units.hpp"

#include <ocam/foundation/Error.hpp>

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <algorithm>

namespace ocam::gui {
namespace {

QString qs(const std::string& s) {
    return QString::fromStdString(s);
}

} // namespace

QWidget* createParamEditor(const strategies::ParamDef& def, const Json* current, LengthUnit unit,
                           std::function<void(const Json&)> changed, const tooling::Tool* tool) {
    QWidget* editor = nullptr;
    switch (def.kind) {
    case strategies::ParamKind::Choice: {
        auto* combo = new QComboBox;
        for (const auto& [value, text] : def.choices) {
            combo->addItem(trStrategy(text), qs(value));
        }
        const std::string value =
            current && current->is_string() ? current->get<std::string>() : def.defaultChoice;
        combo->setCurrentIndex(std::max(0, combo->findData(qs(value))));
        QObject::connect(combo, &QComboBox::currentIndexChanged, combo, [combo, changed] {
            changed(Json(combo->currentData().toString().toStdString()));
        });
        editor = combo;
        break;
    }
    case strategies::ParamKind::Boolean: {
        auto* check = new QCheckBox;
        check->setChecked(current && current->is_boolean() ? current->get<bool>()
                                                           : def.defaultValue != 0.0);
        QObject::connect(check, &QCheckBox::toggled, check,
                         [changed](bool on) { changed(Json(on)); });
        editor = check;
        break;
    }
    case strategies::ParamKind::Integer: {
        auto* spin = new QSpinBox;
        spin->setRange(static_cast<int>(def.min), static_cast<int>(def.max));
        spin->setValue(current && current->is_number() ? current->get<int>()
                                                       : static_cast<int>(def.defaultValue));
        QObject::connect(spin, &QSpinBox::valueChanged, spin,
                         [changed](int v) { changed(Json(v)); });
        editor = spin;
        break;
    }
    default: {
        auto* spin = new QDoubleSpinBox;
        // Stored value = shown value * scale (mm and m/min in the parameter object).
        const bool inch = unit == LengthUnit::Inch;
        double scale = 1.0;
        if ((def.kind == strategies::ParamKind::Length ||
             def.kind == strategies::ParamKind::Feed) &&
            inch) {
            scale = 25.4; // mm per inch, and mm/min per in/min
        } else if (def.kind == strategies::ParamKind::SurfaceSpeed && inch) {
            scale = 0.3048; // m/min per ft/min
        }
        spin->setRange(def.min / scale, def.max / scale);
        spin->setKeyboardTracking(false);
        switch (def.kind) {
        case strategies::ParamKind::Length:
            spin->setDecimals(lengthDecimals(unit));
            spin->setSuffix(lengthSuffix(unit));
            break;
        case strategies::ParamKind::Angle:
            spin->setDecimals(2);
            spin->setSuffix(QStringLiteral(" °"));
            break;
        case strategies::ParamKind::Time:
            spin->setDecimals(2);
            spin->setSuffix(QStringLiteral(" s"));
            break;
        case strategies::ParamKind::Feed:
            spin->setDecimals(inch ? 2 : 0);
            spin->setSingleStep(inch ? 1.0 : 10.0);
            spin->setSuffix(inch ? QObject::tr(" in/min") : QObject::tr(" mm/min"));
            break;
        case strategies::ParamKind::SurfaceSpeed:
            spin->setDecimals(0);
            spin->setSingleStep(10.0);
            spin->setSuffix(surfaceSpeedSuffix(unit));
            break;
        default:
            spin->setDecimals(3);
            spin->setSingleStep(0.05);
            break;
        }
        // Not set: the descriptor's default, or the value it takes from the tool ("From the
        // tool" below the range when there is no tool to take it from).
        double shown = def.defaultValue;
        bool fromTool = false;
        if (current && current->is_number()) {
            shown = current->get<double>();
        } else if (def.toolDefault && tool) {
            try {
                shown = def.toolDefault(*tool);
            } catch (const Error&) {
                // a tool without that size: the plain default
            }
        } else if (def.toolDefault) {
            fromTool = true;
            spin->setMinimum(spin->minimum() - spin->singleStep());
            spin->setSpecialValueText(QObject::tr("From the tool"));
        }
        spin->setValue(fromTool ? spin->minimum() : shown / scale);
        const double lowest = def.min / scale;
        QObject::connect(spin, &QDoubleSpinBox::valueChanged, spin,
                         [changed, scale, lowest](double v) {
                             if (v >= lowest - 1e-12) {
                                 changed(Json(v * scale));
                             }
                         });
        editor = spin;
        break;
    }
    }
    editor->setToolTip(trStrategy(def.help));
    return editor;
}

} // namespace ocam::gui
