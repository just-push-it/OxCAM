// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Machine STEP files split into their bodies (DL-0037): a vendor's machine usually comes as one
// file, an assembly or one multi-body part. Each body becomes a part, placed where the file puts
// it, with its name, the names of the assemblies (and multi-body product) above it and its
// colour.

#pragma once

#include "ocam/cad/Model.hpp"

#include <ocam/foundation/Cancel.hpp>
#include <ocam/foundation/Progress.hpp>

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace ocam::cad {

struct AssemblyPart {
    /// The body's name, else its product's (numbered when the product has several bodies), else
    /// the file's.
    std::string name;
    /// The names of the assemblies (and multi-body product) above it, from the file's top level
    /// down.
    std::vector<std::string> path;
    Model model;                         ///< Placed in the file's frame, in millimetres.
    std::optional<std::uint32_t> colour; ///< 0xRRGGBB, when the file gives one.
    std::size_t solids = 0;
};

/// Reads a STEP file and returns its parts: one per body (solid) of every product instance,
/// placed where the assembly puts it, whether the file is an assembly, one multi-body part or
/// both. A body takes its own name from the file, else its product's (numbered when the product
/// has several bodies; the product then joins the path). Shapes without solids stay whole.
/// Lengths are converted to millimetres from the file's unit.
/// Throws ocam::Error (Io, Parse) like Model::importStep, OperationCancelled when cancelled.
std::vector<AssemblyPart> importStepAssembly(const std::filesystem::path& path, CancelToken& cancel,
                                             ProgressSink& progress);

} // namespace ocam::cad
