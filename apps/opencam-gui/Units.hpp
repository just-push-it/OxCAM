// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Display units in the GUI. Everything is stored in millimetres; widgets show and accept
// values in the project's display unit (or a tool's own unit in the tool editor). The default
// unit for new projects and tools is a user preference.

#pragma once

#include <ocam/foundation/Units.hpp>

#include <QDoubleSpinBox>
#include <QString>

namespace ocam::gui {

/// The preferred unit for new projects and tools (settings "defaultUnit", "mm" or "in").
LengthUnit preferredUnit();
void setPreferredUnit(LengthUnit unit);

/// " mm" or " in".
QString lengthSuffix(LengthUnit unit);
/// Decimals for lengths shown in a unit (3 for mm, 4 for inches).
int lengthDecimals(LengthUnit unit);
/// A length in mm formatted in `unit` with its suffix, e.g. "12.700 mm" / "0.5000 in".
QString formatLength(double mm, LengthUnit unit, bool suffix = true);
/// A length in `unit` followed by the other system in parentheses: "1.2500 in (31.750 mm)".
QString formatLengthBoth(double mm, LengthUnit unit);
/// An area [mm^2] in `unit` and, in parentheses, the other system.
QString formatAreaBoth(double mm2, LengthUnit unit);
/// An area [mm^2] or volume [mm^3] in `unit`.
QString formatArea(double mm2, LengthUnit unit);
QString formatVolume(double mm3, LengthUnit unit);
/// A feed [mm/s] as mm/min or in/min.
QString formatFeed(double mmPerSecond, LengthUnit unit);
/// Surface speed [mm/s] as m/min or ft/min (SFM), with the suffix.
double surfaceSpeedIn(double mmPerSecond, LengthUnit unit);
double surfaceSpeedToMmPerSecond(double value, LengthUnit unit);
QString surfaceSpeedSuffix(LengthUnit unit);

/// A spin box editing a length stored in mm, shown in `unit`; mmChanged() reports mm.
class LengthSpinBox : public QDoubleSpinBox {
    Q_OBJECT

public:
    LengthSpinBox(double mm, LengthUnit unit, double minMm = -100000.0, double maxMm = 100000.0,
                  QWidget* parent = nullptr);
    double mm() const { return value() * factor_; }
    /// Shows `mm` (emits mmChanged unless signals are blocked).
    void setMm(double mm) { setValue(mm / factor_); }

signals:
    void mmChanged(double mm);

private:
    double factor_ = 1.0;
};

} // namespace ocam::gui
