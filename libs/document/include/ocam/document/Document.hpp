// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// An open project: the Project value plus runtime state (embedded model files, imported B-rep
// models, undo history, file path). All edits go through modify(), which records an undo step.
// Not thread-safe: use from one thread (the GUI thread); long imports can run elsewhere and be
// handed in with addImportedModel().

#pragma once

#include "ocam/document/LatheFitting.hpp"
#include "ocam/document/Project.hpp"
#include "ocam/document/Stock.hpp"

#include <ocam/cad/Model.hpp>
#include <ocam/foundation/Cancel.hpp>
#include <ocam/foundation/Progress.hpp>

#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace ocam::document {

class ToolpathCache;
struct WorkholdingPackage;

/// A STEP file read and imported but not yet added to a document (lets the expensive import run
/// on a worker thread).
struct ImportedModel {
    std::string name;
    std::string sourcePath;
    std::string bytes;
    std::string sha256;
    cad::Model model;
};

/// Reads and imports a STEP file. Throws like cad::Model::importStep.
ImportedModel importModelFile(const std::filesystem::path& path, CancelToken& cancel,
                              ProgressSink& progress);

/// The bodies of a workholding template imported (by sha256), ready for Document::useWorkholding.
/// Throws like cad::Model::importStepData.
std::map<std::string, cad::Model> importWorkholdingBodies(const WorkholdingPackage& package,
                                                          CancelToken& cancel,
                                                          ProgressSink& progress);

class Document {
public:
    Document();
    /// A new, unmodified document starting from `project` (for example with the user's
    /// preferred display unit); nothing to undo.
    explicit Document(Project project);

    /// Opens a .ocam archive and imports its embedded models. Throws ocam::Error.
    static Document open(const std::filesystem::path& path, CancelToken& cancel,
                         ProgressSink& progress);
    static Document open(const std::filesystem::path& path);

    /// Writes the archive atomically, embedding every referenced model and the cached toolpaths
    /// of the operations' current inputs, and marks the document unmodified. Throws Error(Io).
    void save(const std::filesystem::path& path);

    const Project& project() const noexcept { return project_; }
    const std::optional<std::filesystem::path>& path() const noexcept { return path_; }
    bool isModified() const noexcept { return undoIndex_ != cleanIndex_; }

    // -- Editing --------------------------------------------------------------------------

    /// Applies `change` to a copy of the project and records it as one undo step labelled
    /// `label`. If `change` throws, nothing is modified. No-op changes are not recorded.
    /// After the change, setups whose WCS origin is kept at a reference (Setup::originFrom)
    /// have it moved there (resolveWcsOrigin).
    void modify(const std::string& label, const std::function<void(Project&)>& change);

    bool canUndo() const noexcept { return undoIndex_ > 0; }
    bool canRedo() const noexcept { return undoIndex_ < history_.size(); }
    std::string undoLabel() const;
    std::string redoLabel() const;
    void undo();
    void redo();

    // -- Models ---------------------------------------------------------------------------

    /// Adds an imported model (undoable) and returns its entry id.
    Uuid addImportedModel(ImportedModel imported);
    /// Adds an imported model as a fixture of a setup (one undo step) and returns its entry id.
    /// Throws Error(InvalidArgument) for an unknown setup.
    Uuid addImportedFixture(ImportedModel imported, const Uuid& setupId);
    /// Removes a model and every reference to it (setup parts and fixtures; operation
    /// geometry, heights and start points picked on it), as one undo step.
    void removeModel(const Uuid& modelId);
    Uuid importModel(const std::filesystem::path& path, CancelToken& cancel,
                     ProgressSink& progress);

    /// The loaded B-rep for a model entry, or nullptr.
    const cad::Model* model(const Uuid& modelId) const;
    /// The loaded B-rep of an embedded file (a model or a workholding body) by its sha256, or
    /// nullptr.
    const cad::Model* modelData(const std::string& sha256) const;
    /// The bytes of an embedded file by its sha256, or nullptr.
    std::shared_ptr<const std::string> modelFile(const std::string& sha256) const;

    // -- Workholding ----------------------------------------------------------------------

    /// Puts a workholding template (with its body files and their imported models, see
    /// importWorkholdingBodies) into the project, replacing the project's copy of the same id,
    /// and places it in a setup, keeping the setup's jaw and placement settings (one undo step). A
    /// jaw set goes on the setup's chuck (DL-0069).
    /// Templates no setup uses any more are dropped. Throws Error(InvalidArgument) for an
    /// unknown setup, an invalid template or a body without its file or model.
    void useWorkholding(const WorkholdingPackage& package, std::map<std::string, cad::Model> models,
                        const Uuid& setupId);
    /// Replaces the project's copy of a workholding template (with its body files and models)
    /// without changing what any setup chooses (one undo step). Throws Error(InvalidArgument) when
    /// the project has no copy of it, for an invalid template or a body without its file or model.
    void updateWorkholding(const WorkholdingPackage& package,
                           std::map<std::string, cad::Model> models);
    /// Takes a setup's workholding as `from` (a copy of this document edited apart, see snapshot)
    /// has it: the setup's workholding settings, with the templates `from` has that this project
    /// has not or has otherwise and their files, as one undo step. Templates no setup uses any
    /// more are dropped. Throws Error(InvalidArgument) for a setup either has not.
    void takeWorkholding(const Document& from, const Uuid& setupId);
    /// A project template with its body files (for saving it to a library), or nullopt.
    std::optional<WorkholdingPackage> workholdingPackage(const Uuid& templateId) const;

    /// Puts a fitting and its file into the project, replacing the project's copy of the same id.
    /// Choosing it for a station or a tailstock is an ordinary edit of the setup. Throws
    /// Error(InvalidArgument) for an invalid fitting.
    void useFitting(const FittingPackage& package);
    /// A project fitting with its file (for saving it to a library), or nullopt.
    std::optional<FittingPackage> fittingPackage(const Uuid& id) const;

    /// Bounds of a model in the project frame (placement applied), or nullopt.
    std::optional<geom3d::Aabb3> modelBounds(const Uuid& modelId) const;
    /// Bounds of a model in the coordinates of `axes` (see ModelBoundsFn): exact when the
    /// model's own axes line up with them, otherwise those of its rotated bounding box.
    std::optional<geom3d::Aabb3> modelBounds(const Uuid& modelId, const Mat3& axes) const;
    /// The largest distance of a model (placement applied) from the line through `point` along
    /// the unit `direction` (project frame), measured on its mesh (see ModelRadiusFn), or
    /// nullopt when the model is not loaded.
    std::optional<double> modelRadius(const Uuid& modelId, const Vec3& point,
                                      const Vec3& direction) const;

    // -- Setups ---------------------------------------------------------------------------

    /// Adds a setup for a model with box stock offset from the part and the WCS at the top of the
    /// stock, at its minimum-X / minimum-Y corner. Returns the setup id.
    Uuid addSetup(const std::optional<Uuid>& modelId);

    std::optional<StockShape> stockShape(const Uuid& setupId) const;

    /// A read-only copy for background work (toolpath generation): same project, models and
    /// toolpath cache (shared, not copied), no undo history and no listeners.
    Document snapshot() const;

    /// Generated toolpaths by operation and inputs (see cachedToolpath); shared with snapshots,
    /// thread-safe, loaded and saved with the project.
    ToolpathCache& toolpathCache() const noexcept { return *toolpathCache_; }

    // -- Change notification --------------------------------------------------------------

    using Listener = std::function<void()>;
    /// Called after every change to the project (modify, undo, redo) and after save.
    int addListener(Listener listener);
    void removeListener(int handle);

private:
    struct Step {
        std::string label;
        Project before;
        Project after;
    };

    void setProject(Project project);
    /// Moves the WCS origins kept at a reference (Setup::originFrom) there, in `next`.
    void keepWcsOrigins(Project& next);
    ModelEntry cacheImported(ImportedModel& imported);
    /// Checks a workholding package and keeps its body files and models.
    void cacheWorkholding(const WorkholdingPackage& package,
                          std::map<std::string, cad::Model> models);
    void notify();

    Project project_;
    std::vector<Step> history_;
    std::size_t undoIndex_ = 0;
    std::size_t cleanIndex_ = 0;
    std::optional<std::filesystem::path> path_;

    std::map<std::string, std::shared_ptr<const std::string>> modelBytes_; ///< sha256 -> STEP file
    std::map<std::string, cad::Model> models_; ///< sha256 -> imported model
    std::shared_ptr<ToolpathCache> toolpathCache_;
    struct MeshPoints;
    std::shared_ptr<MeshPoints> meshPoints_; ///< Mesh vertices by sha256 (modelRadius), shared.

    std::map<int, Listener> listeners_;
    int nextListener_ = 1;

    static constexpr std::size_t kMaxHistory = 500;
};

} // namespace ocam::document
