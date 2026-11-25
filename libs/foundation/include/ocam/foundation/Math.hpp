// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <numbers>

namespace ocam {

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

/// Rigid transform (rotation + translation). Used for WCS frames and kinematic chains.
using Transform3 = Eigen::Isometry3d;

inline constexpr double kPi = std::numbers::pi;

} // namespace ocam
