// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Simple geometric feature recognition used to seed 2.5D operations: holes for drilling and
// horizontal planar faces for facing and depth selection. See docs/design-log/0002.

#pragma once

#include "ocam/cad/Model.hpp"

#include <ocam/foundation/Math.hpp>
#include <ocam/foundation/Tolerance.hpp>

#include <optional>
#include <vector>

namespace ocam::cad {

enum class HoleBottom {
    Through, ///< Open at both ends.
    Flat,    ///< Closed by a planar face perpendicular to the axis.
    Cone,    ///< Closed by a coaxial cone (drill point).
};

struct Hole {
    Vec3 entry;          ///< Axis point at the open end [mm].
    Vec3 direction;      ///< Unit vector from `entry` into the material.
    double diameter = 0; ///< [mm]
    double depth = 0;    ///< Length of the cylindrical wall along `direction`, excluding a tip.
    HoleBottom bottom = HoleBottom::Through;
    double tipAngle = 0;    ///< Included angle of a Cone bottom [rad]; 0 otherwise.
    std::vector<int> faces; ///< 1-based face indices (TopExp face map order) of the walls.
};

/// Finds cylindrical holes whose axis is parallel to `axis` (either sense). Concave cylinders
/// only: bosses are ignored. Coaxial cylinders of different diameter (counterbores) are
/// reported as separate holes. Results are sorted by entry point (x, then y, then z).
std::vector<Hole> findHoles(const Model& model, const Vec3& axis = Vec3::UnitZ(),
                            const Tolerance& tol = {1e-6, 1e-6});

/// A cylindrical boss or stud: a convex cylinder, the outside of which a tool can go around.
struct Boss {
    Vec3 entry;             ///< Axis point at the free end [mm].
    Vec3 direction;         ///< Unit vector from `entry` along the boss towards what holds it.
    double diameter = 0;    ///< [mm]
    double length = 0;      ///< Length of the cylindrical face along `direction` [mm].
    std::vector<int> faces; ///< 1-based face indices (TopExp face map order) of the cylinder.
};

/// Finds convex cylinders (bosses, studs, a turned part's diameters) whose axis is parallel to
/// `axis` (either sense). The free end is the one along +`axis`, unless the part goes on outward
/// past the cylinder there (a shoulder facing -`axis`): then it hangs, and its free end is the
/// other. Coaxial cylinders of different diameter are separate bosses. Sorted by entry point.
std::vector<Boss> findBosses(const Model& model, const Vec3& axis = Vec3::UnitZ(),
                             const Tolerance& tol = {1e-6, 1e-6});

struct PlanarFace {
    int face = 0;         ///< 1-based face index.
    double height = 0;    ///< Position along the axis [mm].
    double area = 0;      ///< [mm^2]
    bool facingUp = true; ///< Outward normal along +axis.
};

/// A planar face's centroid and outward normal (model coordinates).
struct FacePlane {
    Vec3 origin = Vec3::Zero();
    Vec3 normal = Vec3::UnitZ();
};

/// The plane of a planar face (1-based index), or nullopt for other surfaces.
std::optional<FacePlane> facePlane(const Model& model, int faceIndex);

/// Planar faces perpendicular to `axis`, sorted by height, highest first.
std::vector<PlanarFace> findPlanarFaces(const Model& model, const Vec3& axis = Vec3::UnitZ(),
                                        const Tolerance& tol = {1e-6, 1e-6});

} // namespace ocam::cad
