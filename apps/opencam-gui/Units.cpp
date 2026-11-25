// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "Units.hpp"

#include <QSettings>
#include <cmath>

namespace ocam::gui {
namespace {

constexpr double kMmPerInch = 25.4;
constexpr double kMmPerFoot = 304.8;

double factor(LengthUnit unit) {
    return unit == LengthUnit::Inch ? kMmPerInch : 1.0;
}

} // namespace

LengthUnit preferredUnit() {
    return QSettings().value(QStringLiteral("defaultUnit"), QStringLiteral("mm")).toString() ==
                   QStringLiteral("in")
               ? LengthUnit::Inch
               : LengthUnit::Millimeter;
}

void setPreferredUnit(LengthUnit unit) {
    QSettings().setValue(QStringLiteral("defaultUnit"),
                         unit == LengthUnit::Inch ? QStringLiteral("in") : QStringLiteral("mm"));
}

QString lengthSuffix(LengthUnit unit) {
    return unit == LengthUnit::Inch ? QStringLiteral(" in") : QStringLiteral(" mm");
}

int lengthDecimals(LengthUnit unit) {
    return unit == LengthUnit::Inch ? 4 : 3;
}

QString formatLength(double mm, LengthUnit unit, bool suffix) {
    const QString number = QString::number(mm / factor(unit), 'f', lengthDecimals(unit));
    return suffix ? number + lengthSuffix(unit) : number;
}

namespace {
LengthUnit otherUnit(LengthUnit unit) {
    return unit == LengthUnit::Inch ? LengthUnit::Millimeter : LengthUnit::Inch;
}
} // namespace

QString formatLengthBoth(double mm, LengthUnit unit) {
    return QStringLiteral("%1 (%2)").arg(formatLength(mm, unit), formatLength(mm, otherUnit(unit)));
}

QString formatAreaBoth(double mm2, LengthUnit unit) {
    return QStringLiteral("%1 (%2)").arg(formatArea(mm2, unit), formatArea(mm2, otherUnit(unit)));
}

QString formatArea(double mm2, LengthUnit unit) {
    const double f = factor(unit);
    return unit == LengthUnit::Inch ? QStringLiteral("%1 in²").arg(mm2 / (f * f), 0, 'f', 4)
                                    : QStringLiteral("%1 mm²").arg(mm2, 0, 'f', 3);
}

QString formatVolume(double mm3, LengthUnit unit) {
    const double f = factor(unit);
    return unit == LengthUnit::Inch ? QStringLiteral("%1 in³").arg(mm3 / (f * f * f), 0, 'f', 3)
                                    : QStringLiteral("%1 mm³").arg(mm3, 0, 'f', 1);
}

QString formatFeed(double mmPerSecond, LengthUnit unit) {
    const double perMinute = mmPerSecond * 60.0 / factor(unit);
    return unit == LengthUnit::Inch ? QStringLiteral("%1 in/min").arg(perMinute, 0, 'f', 1)
                                    : QStringLiteral("%1 mm/min").arg(perMinute, 0, 'f', 0);
}

double surfaceSpeedIn(double mmPerSecond, LengthUnit unit) {
    return unit == LengthUnit::Inch ? mmPerSecond * 60.0 / kMmPerFoot : mmPerSecond * 0.06;
}

double surfaceSpeedToMmPerSecond(double value, LengthUnit unit) {
    return unit == LengthUnit::Inch ? value * kMmPerFoot / 60.0 : value / 0.06;
}

QString surfaceSpeedSuffix(LengthUnit unit) {
    return unit == LengthUnit::Inch ? QStringLiteral(" ft/min") : QStringLiteral(" m/min");
}

LengthSpinBox::LengthSpinBox(double mm, LengthUnit unit, double minMm, double maxMm,
                             QWidget* parent)
    : QDoubleSpinBox(parent), factor_(factor(unit)) {
    setDecimals(lengthDecimals(unit) + 1);
    setRange(minMm / factor_, maxMm / factor_);
    setSingleStep(unit == LengthUnit::Inch ? 0.01 : 0.1);
    setSuffix(lengthSuffix(unit));
    setKeyboardTracking(false);
    setValue(mm / factor_);
    connect(this, &QDoubleSpinBox::valueChanged, this,
            [this](double v) { emit mmChanged(v * factor_); });
}

} // namespace ocam::gui
