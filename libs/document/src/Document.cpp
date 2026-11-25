// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/document/Document.hpp"

#include "ocam/document/Generate.hpp"
#include "ocam/document/MachineImport.hpp"
#include "ocam/document/ProjectIo.hpp"
#include "ocam/document/ToolpathCache.hpp"
#include "ocam/document/Wcs.hpp"
#include "ocam/document/Workholding.hpp"

#include <ocam/cad/Tessellation.hpp>
#include <ocam/foundation/Error.hpp>
#include <ocam/foundation/Json.hpp>
#include <ocam/foundation/Sha256.hpp>
#include <ocam/machine/Model.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <cmath>
#include <mutex>
#include <utility>
#include <vector>

namespace ocam::document {
namespace {

std::string utf8(const std::filesystem::path& p) {
    const std::u8string u8 = p.u8string();
    return {u8.begin(), u8.end()};
}

} // namespace

ImportedModel importModelFile(const std::filesystem::path& path, CancelToken& cancel,
                              ProgressSink& progress) {
    ImportedModel m;
    m.name = utf8(path.stem());
    m.sourcePath = utf8(path);
    m.bytes = readFileBytes(path);
    m.sha256 = sha256Hex(m.bytes);
    m.model = cad::Model::importStepData(m.bytes, utf8(path.filename()), cancel, progress);
    return m;
}

std::map<std::string, cad::Model> importWorkholdingBodies(const WorkholdingPackage& package,
                                                          CancelToken& cancel,
                                                          ProgressSink& progress) {
    std::map<std::string, cad::Model> out;
    const auto& bodies = package.workholding.parts;
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        const WorkholdingPart& b = bodies[i];
        if (out.contains(b.sha256)) {
            continue;
        }
        const auto file = package.files.find(b.sha256);
        if (file == package.files.end()) {
            throw Error(ErrorCode::InvalidArgument,
                        fmt::format("the file of part '{}' is missing", b.name));
        }
        ProgressRange range(progress, static_cast<double>(i) / bodies.size(),
                            static_cast<double>(i + 1) / bodies.size());
        out.emplace(b.sha256, cad::Model::importStepData(file->second, b.name, cancel, range));
    }
    return out;
}

/// The vertices of models' meshes in their own frames, made once per file and thread-safe.
struct Document::MeshPoints {
    std::mutex mutex;
    std::map<std::string, std::shared_ptr<const std::vector<Vec3>>> bySha;
};

Document::Document()
    : toolpathCache_(std::make_shared<ToolpathCache>()),
      meshPoints_(std::make_shared<MeshPoints>()) {}

Document::Document(Project project)
    : project_(std::move(project)), toolpathCache_(std::make_shared<ToolpathCache>()),
      meshPoints_(std::make_shared<MeshPoints>()) {}

Document Document::open(const std::filesystem::path& path, CancelToken& cancel,
                        ProgressSink& progress) {
    const std::string name = utf8(path.filename());
    ArchiveContents archive = readArchive(readFileBytes(path), name);

    Document doc;
    doc.project_ = projectFromJson(parseJson(archive.projectJson, name + "/project.json"),
                                   name + "/project.json");

    for (machine::Machine& m : doc.project_.machines) {
        machine::attachPartFiles(
            m,
            [&](const std::string& file) -> std::optional<std::string> {
                const auto it = archive.machineFiles.find(file);
                return it == archive.machineFiles.end() ? std::nullopt : std::optional(it->second);
            },
            fmt::format("{} (machine '{}')", name, m.name));
        meshMachineParts(m);
    }
    // The models, then the bodies of the workholding templates: (sha256, name) pairs.
    std::vector<std::pair<std::string, std::string>> files;
    for (const ModelEntry& entry : doc.project_.models) {
        files.emplace_back(entry.sha256, entry.name);
    }
    for (const WorkholdingTemplate& w : doc.project_.workholdings) {
        for (const WorkholdingPart& b : w.parts) {
            files.emplace_back(b.sha256, b.name);
        }
    }
    for (const LatheFitting& f : doc.project_.fittings) {
        files.emplace_back(f.sha256, f.name);
    }
    for (std::size_t i = 0; i < files.size(); ++i) {
        const auto& [sha, fileName] = files[i];
        if (doc.models_.contains(sha)) {
            continue; // same file embedded under several entries
        }
        auto it = archive.models.find(sha);
        if (it == archive.models.end()) {
            throw Error(ErrorCode::Parse,
                        fmt::format("{}: embedded file for model '{}' is missing", name, fileName));
        }
        ProgressRange range(progress, static_cast<double>(i) / files.size(),
                            static_cast<double>(i + 1) / files.size());
        doc.models_.emplace(sha, cad::Model::importStepData(it->second, fileName, cancel, range));
        doc.modelBytes_.emplace(sha, std::make_shared<const std::string>(std::move(it->second)));
    }
    if (!archive.toolpathCache.empty()) {
        // A cache that does not read is generated again.
        try {
            doc.toolpathCache_->load(parseJson(archive.toolpathCache, name + "/cache"));
        } catch (const Error&) {
        }
    }
    doc.path_ = path;
    progress.report(1.0, "Done");
    return doc;
}

Document Document::open(const std::filesystem::path& path) {
    CancelToken cancel;
    NullProgress progress;
    return open(path, cancel, progress);
}

void Document::save(const std::filesystem::path& path) {
    ArchiveContents archive;
    archive.projectJson = dumpJson(toJson(project_));
    for (const auto& entry : project_.models) {
        auto it = modelBytes_.find(entry.sha256);
        if (it == modelBytes_.end()) {
            throw Error(ErrorCode::Internal, "model data missing for '" + entry.name + "'");
        }
        archive.models.emplace(entry.sha256, *it->second);
    }
    for (const WorkholdingTemplate& w : project_.workholdings) {
        for (const WorkholdingPart& b : w.parts) {
            auto it = modelBytes_.find(b.sha256);
            if (it == modelBytes_.end()) {
                throw Error(ErrorCode::Internal, "workholding data missing for '" + b.name + "'");
            }
            archive.models.emplace(b.sha256, *it->second);
        }
    }
    for (const LatheFitting& f : project_.fittings) {
        auto it = modelBytes_.find(f.sha256);
        if (it == modelBytes_.end()) {
            throw Error(ErrorCode::Internal, "the model of fitting '" + f.name + "' is missing");
        }
        archive.models.emplace(f.sha256, *it->second);
    }
    for (const machine::Machine& m : project_.machines) {
        for (const machine::ModelPart& part : m.model.parts) {
            if (part.mesh.empty() && part.step.empty()) {
                continue;
            }
            const std::string file = machine::partFileName(part);
            if (!archive.machineFiles.contains(file)) {
                archive.machineFiles.emplace(file, machine::partFileBytes(part));
            }
        }
    }
    // The toolpaths of the operations as they are now.
    std::map<Uuid, std::string> current;
    for (const Setup& setup : project_.setups) {
        for (const Operation& op : setup.operations) {
            if (op.enabled) {
                current.emplace(op.id, toolpathInputKey(*this, setup, op));
            }
        }
    }
    if (const Json cache = toolpathCache_->toJson(current); !cache.at("entries").empty()) {
        archive.toolpathCache = cache.dump();
    }
    writeFileAtomic(path, writeArchive(archive));
    path_ = path;
    cleanIndex_ = undoIndex_;
    notify();
}

void Document::modify(const std::string& label, const std::function<void(Project&)>& change) {
    Project next = project_;
    change(next);
    keepWcsOrigins(next);
    if (next == project_) {
        return;
    }
    history_.resize(undoIndex_); // drop the redo branch
    if (cleanIndex_ > undoIndex_) {
        cleanIndex_ = static_cast<std::size_t>(-1); // the saved state is no longer reachable
    }
    history_.push_back({label, project_, next});
    if (history_.size() > kMaxHistory) {
        history_.erase(history_.begin());
        cleanIndex_ = cleanIndex_ == 0 ? static_cast<std::size_t>(-1) : cleanIndex_ - 1;
    }
    undoIndex_ = history_.size();
    project_ = std::move(next);
    notify();
}

void Document::keepWcsOrigins(Project& next) {
    if (std::none_of(next.setups.begin(), next.setups.end(),
                     [](const Setup& s) { return s.originFrom.has_value(); })) {
        return;
    }
    // Measured on the project being made: its stock, part and workholding as they now are. The
    // point does not depend on where the origin was (the stock and workholding move with it).
    std::vector<std::pair<Uuid, Vec3>> origins;
    std::swap(project_, next);
    try {
        for (const Setup& s : project_.setups) {
            try {
                if (const auto origin = resolveWcsOrigin(*this, s)) {
                    origins.emplace_back(s.id, *origin);
                }
            } catch (const Error&) {
                // left where it was; the setup's page says why it cannot be found
            }
        }
    } catch (...) {
        std::swap(project_, next);
        throw;
    }
    std::swap(project_, next);
    for (const auto& [id, origin] : origins) {
        next.findSetup(id)->wcs.origin = origin;
    }
}

std::string Document::undoLabel() const {
    return canUndo() ? history_[undoIndex_ - 1].label : std::string();
}

std::string Document::redoLabel() const {
    return canRedo() ? history_[undoIndex_].label : std::string();
}

void Document::undo() {
    if (!canUndo()) {
        return;
    }
    --undoIndex_;
    project_ = history_[undoIndex_].before;
    notify();
}

void Document::redo() {
    if (!canRedo()) {
        return;
    }
    project_ = history_[undoIndex_].after;
    ++undoIndex_;
    notify();
}

ModelEntry Document::cacheImported(ImportedModel& imported) {
    ModelEntry entry;
    entry.name = imported.name;
    entry.sourcePath = imported.sourcePath;
    entry.sha256 = imported.sha256;
    // Runtime caches are not part of undo: keeping them lets redo restore the entry.
    modelBytes_.try_emplace(imported.sha256,
                            std::make_shared<const std::string>(std::move(imported.bytes)));
    models_.try_emplace(imported.sha256, std::move(imported.model));
    return entry;
}

Uuid Document::addImportedModel(ImportedModel imported) {
    const ModelEntry entry = cacheImported(imported);
    modify("Import " + entry.name, [&](Project& p) {
        if (p.name == "Untitled") {
            p.name = entry.name;
        }
        p.models.push_back(entry);
    });
    return entry.id;
}

Uuid Document::addImportedFixture(ImportedModel imported, const Uuid& setupId) {
    if (!project_.findSetup(setupId)) {
        throw Error(ErrorCode::InvalidArgument, "no such setup");
    }
    const ModelEntry entry = cacheImported(imported);
    modify("Import fixture " + entry.name, [&](Project& p) {
        p.models.push_back(entry);
        p.findSetup(setupId)->fixtures.push_back(entry.id);
    });
    return entry.id;
}

void Document::removeModel(const Uuid& modelId) {
    const ModelEntry* entry = project_.findModel(modelId);
    if (!entry) {
        return;
    }
    modify("Remove " + entry->name, [&](Project& p) {
        std::erase_if(p.models, [&](const ModelEntry& m) { return m.id == modelId; });
        auto onModel = [&](const std::optional<ShapeRef>& ref) {
            return ref && ref->modelId == modelId;
        };
        for (Setup& s : p.setups) {
            if (s.modelId == modelId) {
                s.modelId.reset();
            }
            std::erase(s.fixtures, modelId);
            for (Operation& op : s.operations) {
                std::erase_if(op.geometry, [&](const ShapeRef& r) { return r.modelId == modelId; });
                if (onModel(op.startPoint)) {
                    op.startPoint.reset();
                }
                for (HeightSetting* h : {&op.heights.clearance, &op.heights.retract,
                                         &op.heights.feed, &op.heights.top, &op.heights.bottom}) {
                    if (onModel(h->shape)) {
                        *h = HeightSetting{};
                    }
                }
            }
        }
    });
}

Uuid Document::importModel(const std::filesystem::path& path, CancelToken& cancel,
                           ProgressSink& progress) {
    return addImportedModel(importModelFile(path, cancel, progress));
}

const cad::Model* Document::model(const Uuid& modelId) const {
    const ModelEntry* entry = project_.findModel(modelId);
    if (!entry) {
        return nullptr;
    }
    auto it = models_.find(entry->sha256);
    return it == models_.end() ? nullptr : &it->second;
}

const cad::Model* Document::modelData(const std::string& sha256) const {
    auto it = models_.find(sha256);
    return it == models_.end() ? nullptr : &it->second;
}

std::shared_ptr<const std::string> Document::modelFile(const std::string& sha256) const {
    auto it = modelBytes_.find(sha256);
    return it == modelBytes_.end() ? nullptr : it->second;
}

void Document::cacheWorkholding(const WorkholdingPackage& package,
                                std::map<std::string, cad::Model> models) {
    const WorkholdingTemplate& w = package.workholding;
    if (const auto issues = w.validate(); !issues.empty()) {
        throw Error(ErrorCode::InvalidArgument,
                    fmt::format("workholding '{}': {}", w.name, issues.front()));
    }
    for (const WorkholdingPart& b : w.parts) {
        const bool haveFile = package.files.contains(b.sha256) || modelBytes_.contains(b.sha256);
        const bool haveModel = models.contains(b.sha256) || models_.contains(b.sha256);
        if (!haveFile || !haveModel) {
            throw Error(ErrorCode::InvalidArgument,
                        fmt::format("the file of part '{}' is missing", b.name));
        }
    }
    // Runtime caches are not part of undo: keeping them lets redo restore the template.
    for (const WorkholdingPart& b : w.parts) {
        if (const auto file = package.files.find(b.sha256); file != package.files.end()) {
            modelBytes_.try_emplace(b.sha256, std::make_shared<const std::string>(file->second));
        }
        if (auto m = models.find(b.sha256); m != models.end()) {
            models_.try_emplace(b.sha256, std::move(m->second));
        }
    }
}

void Document::updateWorkholding(const WorkholdingPackage& package,
                                 std::map<std::string, cad::Model> models) {
    const WorkholdingTemplate& w = package.workholding;
    if (!project_.findWorkholding(w.id)) {
        throw Error(ErrorCode::InvalidArgument,
                    fmt::format("workholding '{}' is not in the project", w.name));
    }
    cacheWorkholding(package, std::move(models));
    modify("Update workholding " + w.name, [&](Project& p) {
        for (WorkholdingTemplate& own : p.workholdings) {
            if (own.id == w.id) {
                own = w;
            }
        }
    });
}

void Document::takeWorkholding(const Document& from, const Uuid& setupId) {
    const Setup* source = from.project_.findSetup(setupId);
    if (!source || !project_.findSetup(setupId)) {
        throw Error(ErrorCode::InvalidArgument, "no such setup");
    }
    // The other document's templates that this one has not, or has otherwise, with their files.
    std::vector<const WorkholdingTemplate*> taken;
    for (const WorkholdingTemplate& w : from.project_.workholdings) {
        const WorkholdingTemplate* own = project_.findWorkholding(w.id);
        const bool used = source->workholding && (source->workholding->templateId == w.id ||
                                                  source->workholding->jawSet == w.id);
        if ((own && *own != w) || (!own && used)) {
            taken.push_back(&w);
        }
    }
    for (const WorkholdingTemplate* w : taken) {
        for (const WorkholdingPart& b : w->parts) {
            const auto file = from.modelBytes_.find(b.sha256);
            const auto model = from.models_.find(b.sha256);
            if ((file == from.modelBytes_.end() && !modelBytes_.contains(b.sha256)) ||
                (model == from.models_.end() && !models_.contains(b.sha256))) {
                throw Error(ErrorCode::InvalidArgument,
                            fmt::format("the file of part '{}' is missing", b.name));
            }
            if (file != from.modelBytes_.end()) {
                modelBytes_.try_emplace(b.sha256, file->second);
            }
            if (model != from.models_.end()) {
                models_.try_emplace(b.sha256, model->second);
            }
        }
    }
    modify("Set up workholding", [&](Project& p) {
        for (const WorkholdingTemplate* w : taken) {
            const auto own =
                std::find_if(p.workholdings.begin(), p.workholdings.end(),
                             [&](const WorkholdingTemplate& t) { return t.id == w->id; });
            if (own != p.workholdings.end()) {
                *own = *w;
            } else {
                p.workholdings.push_back(*w);
            }
        }
        p.findSetup(setupId)->workholding = source->workholding;
        removeUnusedWorkholdings(p);
    });
}

void Document::useWorkholding(const WorkholdingPackage& package,
                              std::map<std::string, cad::Model> models, const Uuid& setupId) {
    if (!project_.findSetup(setupId)) {
        throw Error(ErrorCode::InvalidArgument, "no such setup");
    }
    const WorkholdingTemplate& w = package.workholding;
    if (w.jawSet && !project_.findSetup(setupId)->workholding) {
        throw Error(
            ErrorCode::InvalidArgument,
            fmt::format("jaw set '{}' goes on a chuck: choose the setup's chuck first", w.name));
    }
    cacheWorkholding(package, std::move(models));
    modify("Use workholding " + w.name, [&](Project& p) {
        auto it = std::find_if(p.workholdings.begin(), p.workholdings.end(),
                               [&](const WorkholdingTemplate& t) { return t.id == w.id; });
        if (it != p.workholdings.end()) {
            *it = w;
        } else {
            p.workholdings.push_back(w);
        }
        Setup& s = *p.findSetup(setupId);
        SetupWorkholding placement = s.workholding.value_or(SetupWorkholding{});
        if (w.jawSet) {
            if (placement.jawSet != w.id) {
                placement.jawOption.clear(); // another set's ways and steps are its own
                placement.step.clear();
            }
            placement.jawSet = w.id; // on the setup's chuck (DL-0069)
        } else {
            placement.templateId = w.id;
        }
        s.workholding = placement;
        removeUnusedWorkholdings(p);
    });
}

std::optional<WorkholdingPackage> Document::workholdingPackage(const Uuid& templateId) const {
    const WorkholdingTemplate* w = project_.findWorkholding(templateId);
    if (!w) {
        return std::nullopt;
    }
    WorkholdingPackage out{*w, {}};
    for (const WorkholdingPart& b : w->parts) {
        if (const auto file = modelFile(b.sha256)) {
            out.files.emplace(b.sha256, *file);
        }
    }
    return out;
}

void Document::useFitting(const FittingPackage& package) {
    if (const auto issues = package.fitting.validate(); !issues.empty()) {
        throw Error(ErrorCode::InvalidArgument,
                    fmt::format("fitting '{}': {}", package.fitting.name, issues.front()));
    }
    if (!package.file.bytes) {
        throw Error(ErrorCode::InvalidArgument,
                    fmt::format("the model of '{}' is not loaded", package.fitting.name));
    }
    // Runtime caches are not part of undo.
    modelBytes_.try_emplace(package.fitting.sha256, package.file.bytes);
    if (!models_.contains(package.fitting.sha256)) {
        models_.emplace(package.fitting.sha256, stepModel(package.file, package.fitting.name));
    }
    modify("Use " + package.fitting.name, [&](Project& p) {
        auto it = std::find_if(p.fittings.begin(), p.fittings.end(),
                               [&](const LatheFitting& f) { return f.id == package.fitting.id; });
        if (it != p.fittings.end()) {
            *it = package.fitting;
        } else {
            p.fittings.push_back(package.fitting);
        }
    });
}

std::optional<FittingPackage> Document::fittingPackage(const Uuid& id) const {
    const LatheFitting* f = project_.findFitting(id);
    const auto file = f ? modelFile(f->sha256) : nullptr;
    if (!file) {
        return std::nullopt;
    }
    return FittingPackage{*f, {f->sha256, file}, false};
}

std::optional<geom3d::Aabb3> Document::modelBounds(const Uuid& modelId) const {
    return modelBounds(modelId, Mat3::Identity());
}

std::optional<geom3d::Aabb3> Document::modelBounds(const Uuid& modelId, const Mat3& axes) const {
    const ModelEntry* entry = project_.findModel(modelId);
    const cad::Model* m = model(modelId);
    if (!entry || !m) {
        return std::nullopt;
    }
    const geom3d::Aabb3 local = m->info().bounds;
    if (local.isEmpty()) {
        return std::nullopt;
    }
    const Transform3 xf = entry->placement.toParent();
    geom3d::Aabb3 out;
    for (int i = 0; i < 8; ++i) {
        const Vec3 corner((i & 1) ? local.max.x() : local.min.x(),
                          (i & 2) ? local.max.y() : local.min.y(),
                          (i & 4) ? local.max.z() : local.min.z());
        out.extend(axes.transpose() * (xf * corner));
    }
    return out;
}

Uuid Document::addSetup(const std::optional<Uuid>& modelId) {
    Setup setup;
    setup.name = fmt::format("Setup {}", project_.setups.size() + 1);
    setup.modelId = modelId;
    setup.workOffset = static_cast<int>(project_.setups.size()) + 1;
    if (!modelId) {
        setup.stock.kind = StockKind::Box;
        setup.stock.size = StockSize::Fixed;
        setup.stock.boxSize = Vec3(100, 100, 20); // X 0..100, Y 0..100, Z -20..0
        setup.stock.shift = Vec3(50, 50, -10);
    }
    Project probe = project_;
    probe.setups.push_back(setup);
    const auto stock =
        resolveStock(probe, probe.setups.back(),
                     [this](const Uuid& id, const Mat3& axes) { return modelBounds(id, axes); });
    if (stock) {
        const geom3d::Aabb3 b = stock->bounds();
        setup.wcs.origin = Vec3(b.min.x(), b.min.y(), b.max.z());
    }
    // The part's own axes, the origin at the stock box's top front left corner (kept there).
    setup.orientation = WcsOrientation{};
    setup.originFrom = WcsOrigin{};
    setup.originFrom->box = {0, 0, 2};
    const Uuid id = setup.id;
    modify("Add " + setup.name, [&](Project& p) { p.setups.push_back(setup); });
    return id;
}

std::optional<double> Document::modelRadius(const Uuid& modelId, const Vec3& point,
                                            const Vec3& direction) const {
    const ModelEntry* entry = project_.findModel(modelId);
    const cad::Model* m = model(modelId);
    if (!entry || !m) {
        return std::nullopt;
    }
    std::shared_ptr<const std::vector<Vec3>> points;
    {
        const std::lock_guard lock(meshPoints_->mutex);
        points = meshPoints_->bySha[entry->sha256];
    }
    if (!points) {
        // The vertices lie on the surfaces, so a round face's radius is exact.
        auto made = std::make_shared<std::vector<Vec3>>(cad::tessellate(*m, {0.01, 0.35}).vertices);
        const std::lock_guard lock(meshPoints_->mutex);
        meshPoints_->bySha[entry->sha256] = made;
        points = std::move(made);
    }
    if (points->empty()) {
        return std::nullopt;
    }
    // The line in the model's own frame.
    const Transform3 toModel = entry->placement.toParent().inverse();
    const Vec3 p = toModel * point;
    const Vec3 d = (toModel.linear() * direction).normalized();
    double r2 = 0.0;
    for (const Vec3& v : *points) {
        const Vec3 q = v - p;
        r2 = std::max(r2, (q - q.dot(d) * d).squaredNorm());
    }
    return std::sqrt(r2);
}

std::optional<StockShape> Document::stockShape(const Uuid& setupId) const {
    const Setup* setup = project_.findSetup(setupId);
    if (!setup) {
        return std::nullopt;
    }
    return resolveStock(
        project_, *setup,
        [this](const Uuid& id, const Mat3& axes) { return modelBounds(id, axes); },
        [this](const Uuid& id, const Vec3& point, const Vec3& direction) {
            return modelRadius(id, point, direction);
        });
}

Document Document::snapshot() const {
    Document copy;
    copy.project_ = project_;
    copy.path_ = path_;
    copy.modelBytes_ = modelBytes_;
    copy.models_ = models_;
    copy.toolpathCache_ = toolpathCache_;
    copy.meshPoints_ = meshPoints_;
    return copy;
}

int Document::addListener(Listener listener) {
    const int handle = nextListener_++;
    listeners_.emplace(handle, std::move(listener));
    return handle;
}

void Document::removeListener(int handle) {
    listeners_.erase(handle);
}

void Document::notify() {
    // Copy so listeners may add or remove listeners while being notified.
    const auto listeners = listeners_;
    for (const auto& [handle, listener] : listeners) {
        listener();
    }
}

} // namespace ocam::document
