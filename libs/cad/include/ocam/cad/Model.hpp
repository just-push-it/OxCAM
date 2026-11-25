// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Imported B-rep model. This header is OpenCASCADE-free: only libs/cad links OCCT. Code that
// genuinely needs the OCCT shape (the GUI viewer, the fixture generator) includes
// ocam/cad/occt/ShapeAccess.hpp.

#pragma once

#include <ocam/foundation/Cancel.hpp>
#include <ocam/foundation/Progress.hpp>
#include <ocam/foundation/Units.hpp>
#include <ocam/geom3d/Bounds.hpp>

#include <cstddef>
#include <filesystem>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace ocam::cad {

struct ModelInfo {
    std::size_t solids = 0;
    std::size_t shells = 0;
    std::size_t faces = 0;
    std::size_t edges = 0;
    geom3d::Aabb3 bounds; ///< Tight bounds [mm].
    double volume = 0.0;  ///< [mm^3]
    double surfaceArea = 0.0;
};

/// Immutable, cheaply copyable handle to an imported shape. All geometry is in millimetres
/// regardless of the units of the source file.
class Model {
public:
    /// Reads a STEP (AP203/AP214/AP242) file. Throws ocam::Error with ErrorCode::Io when the
    /// file cannot be read, ErrorCode::Parse when it is not valid STEP or contains no shape, and
    /// OperationCancelled if `cancel` is triggered.
    static Model importStep(const std::filesystem::path& path, CancelToken& cancel,
                            ProgressSink& progress);
    static Model importStep(const std::filesystem::path& path);

    /// Reads STEP content held in memory (for example, a model embedded in a project file).
    /// `sourceName` is used in error messages. Same errors as importStep().
    static Model importStepData(std::string_view stepText, std::string_view sourceName,
                                CancelToken& cancel, ProgressSink& progress);

    /// Writes the model as STEP AP214, declaring `unit` as the file's length unit. Throws
    /// ocam::Error (Io) on failure.
    void exportStep(const std::filesystem::path& path,
                    LengthUnit unit = LengthUnit::Millimeter) const;
    /// The STEP AP214 file's text (in memory), declaring `unit`: what exportStep writes. The
    /// header holds the time written, so the bytes differ between calls. Throws ocam::Error (Io).
    std::string exportStepData(LengthUnit unit = LengthUnit::Millimeter) const;

    ModelInfo info() const;

    /// A null model (no shape). Most functions require a non-null model.
    Model() = default;

    bool isNull() const noexcept { return !impl_; }

    struct Impl;
    explicit Model(std::shared_ptr<const Impl> impl) noexcept : impl_(std::move(impl)) {}
    const Impl& impl() const noexcept { return *impl_; }

private:
    std::shared_ptr<const Impl> impl_;
};

/// The models as one model (a compound of their shapes, each as it is). Null models are skipped;
/// one model is returned as it is.
Model compound(const std::vector<Model>& models);

/// The model moved rigidly by `xf` (its geometry copied, so sub-shape numbering is kept).
Model transformed(const Model& model, const Transform3& xf);

/// `model` with `removed` taken off (a boolean cut). Throws ocam::Error (InvalidArgument) for a
/// null model, (Internal) when the cut fails.
Model subtracted(const Model& model, const Model& removed);

} // namespace ocam::cad
