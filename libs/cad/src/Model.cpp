// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ModelImpl.hpp"
#include "ocam/cad/occt/ShapeAccess.hpp"

#include <ocam/foundation/Error.hpp>

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepGProp.hxx>
#include <BRep_Builder.hxx>
#include <Bnd_Box.hxx>
#include <DESTEP_Parameters.hxx>
#include <GProp_GProps.hxx>
#include <Message.hxx>
#include <Message_Messenger.hxx>
#include <Message_Printer.hxx>
#include <Message_ProgressIndicator.hxx>
#include <Message_ProgressScope.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <TopExp.hxx>
#include <TopoDS_Compound.hxx>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

namespace ocam::cad {
namespace {

/// Keeps the STEP writer's transfer statistics off the console while it lives.
class QuietMessages {
public:
    QuietMessages() : lock_(mutex()) {
        for (const auto& printer : Message::DefaultMessenger()->Printers()) {
            levels_.push_back(printer->GetTraceLevel());
            printer->SetTraceLevel(Message_Fail);
        }
    }
    ~QuietMessages() {
        std::size_t i = 0;
        for (const auto& printer : Message::DefaultMessenger()->Printers()) {
            if (i < levels_.size()) {
                printer->SetTraceLevel(levels_[i++]);
            }
        }
    }
    QuietMessages(const QuietMessages&) = delete;
    QuietMessages& operator=(const QuietMessages&) = delete;

private:
    static std::mutex& mutex() {
        static std::mutex m;
        return m;
    }
    std::lock_guard<std::mutex> lock_;
    std::vector<Message_Gravity> levels_;
};

// Transfers the roots of an already-read STEP model into a shape.
Model transferRoots(STEPControl_Reader& reader, std::string_view sourceName, CancelToken& cancel,
                    ProgressSink& progress) {
    cancel.throwIfCancelled();
    reader.SetSystemLengthUnit(1.0); // millimetres
    occ::handle<ProgressAdapter> indicator = new ProgressAdapter(cancel, progress, 0.2, 1.0);
    reader.TransferRoots(indicator->Start());
    cancel.throwIfCancelled();

    TopoDS_Shape shape = reader.OneShape();
    if (shape.IsNull()) {
        throw Error(ErrorCode::Parse,
                    "STEP data contains no transferable shape: " + std::string(sourceName));
    }
    progress.report(1.0, "Done");
    return modelFromShape(shape);
}

} // namespace

Model Model::importStepData(std::string_view stepText, std::string_view sourceName,
                            CancelToken& cancel, ProgressSink& progress) {
    progress.report(0.0, "Reading STEP data");
    std::istringstream stream{std::string(stepText)};
    STEPControl_Reader reader;
    if (reader.ReadStream(std::string(sourceName).c_str(), stream) != IFSelect_RetDone) {
        throw Error(ErrorCode::Parse, "not readable STEP data: " + std::string(sourceName));
    }
    return transferRoots(reader, sourceName, cancel, progress);
}

Model Model::importStep(const std::filesystem::path& path, CancelToken& cancel,
                        ProgressSink& progress) {
    std::error_code ec;
    if (!std::filesystem::is_regular_file(path, ec)) {
        throw Error(ErrorCode::Io, "STEP file not found: " + utf8Path(path));
    }

    progress.report(0.0, "Reading STEP file");
    STEPControl_Reader reader;
    if (reader.ReadFile(utf8Path(path).c_str()) != IFSelect_RetDone) {
        throw Error(ErrorCode::Parse, "not a readable STEP file: " + utf8Path(path));
    }
    cancel.throwIfCancelled();

    reader.SetSystemLengthUnit(1.0); // millimetres
    occ::handle<ProgressAdapter> indicator = new ProgressAdapter(cancel, progress, 0.2, 1.0);
    reader.TransferRoots(indicator->Start());
    cancel.throwIfCancelled();

    TopoDS_Shape shape = reader.OneShape();
    if (shape.IsNull()) {
        throw Error(ErrorCode::Parse,
                    "STEP file contains no transferable shape: " + utf8Path(path));
    }
    progress.report(1.0, "Done");
    return modelFromShape(shape);
}

Model Model::importStep(const std::filesystem::path& path) {
    CancelToken cancel;
    NullProgress progress;
    return importStep(path, cancel, progress);
}

void Model::exportStep(const std::filesystem::path& path, LengthUnit unit) const {
    DESTEP_Parameters params;
    params.WriteUnit = unit == LengthUnit::Inch ? UnitsMethods_LengthUnit_Inch
                                                : UnitsMethods_LengthUnit_Millimeter;
    const QuietMessages quiet;
    STEPControl_Writer writer;
    if (writer.Transfer(impl_->shape, STEPControl_AsIs, params) != IFSelect_RetDone ||
        writer.Write(utf8Path(path).c_str()) != IFSelect_RetDone) {
        throw Error(ErrorCode::Io, "failed to write STEP file: " + utf8Path(path));
    }
}

std::string Model::exportStepData(LengthUnit unit) const {
    DESTEP_Parameters params;
    params.WriteUnit = unit == LengthUnit::Inch ? UnitsMethods_LengthUnit_Inch
                                                : UnitsMethods_LengthUnit_Millimeter;
    const QuietMessages quiet;
    STEPControl_Writer writer;
    std::ostringstream out;
    if (writer.Transfer(impl_->shape, STEPControl_AsIs, params) != IFSelect_RetDone ||
        writer.WriteStream(out) != IFSelect_RetDone) {
        throw Error(ErrorCode::Io, "failed to write STEP data");
    }
    return std::move(out).str();
}

Model compound(const std::vector<Model>& models) {
    std::vector<const Model*> real;
    for (const Model& m : models) {
        if (!m.isNull()) {
            real.push_back(&m);
        }
    }
    if (real.size() == 1) {
        return *real.front();
    }
    TopoDS_Compound all;
    BRep_Builder builder;
    builder.MakeCompound(all);
    for (const Model* m : real) {
        builder.Add(all, m->impl().shape);
    }
    return modelFromShape(all);
}

Model transformed(const Model& model, const Transform3& xf) {
    gp_Trsf trsf;
    const Mat3& r = xf.linear();
    const Vec3 t = xf.translation();
    trsf.SetValues(r(0, 0), r(0, 1), r(0, 2), t.x(), r(1, 0), r(1, 1), r(1, 2), t.y(), r(2, 0),
                   r(2, 1), r(2, 2), t.z());
    // Copied, so the result carries no location: names and colours read from STEP stay on it.
    return modelFromShape(BRepBuilderAPI_Transform(model.impl().shape, trsf, true).Shape());
}

Model subtracted(const Model& model, const Model& removed) {
    if (model.isNull() || removed.isNull()) {
        throw Error(ErrorCode::InvalidArgument, "a cut needs two models");
    }
    BRepAlgoAPI_Cut cut(model.impl().shape, removed.impl().shape);
    if (!cut.IsDone()) {
        throw Error(ErrorCode::Internal, "the model could not be cut");
    }
    return modelFromShape(cut.Shape());
}

namespace {

ModelInfo computeInfo(const TopoDS_Shape& shape) {
    ModelInfo info;

    auto count = [&](TopAbs_ShapeEnum type) {
        ShapeIndexMap map;
        TopExp::MapShapes(shape, type, map);
        return static_cast<std::size_t>(map.Extent());
    };
    info.solids = count(TopAbs_SOLID);
    info.shells = count(TopAbs_SHELL);
    info.faces = count(TopAbs_FACE);
    info.edges = count(TopAbs_EDGE);

    Bnd_Box box;
    BRepBndLib::AddOptimal(shape, box, false, false);
    if (!box.IsVoid()) {
        double x0 = 0, y0 = 0, z0 = 0, x1 = 0, y1 = 0, z1 = 0;
        box.Get(x0, y0, z0, x1, y1, z1);
        info.bounds.extend(Vec3(x0, y0, z0));
        info.bounds.extend(Vec3(x1, y1, z1));
    }

    GProp_GProps volume;
    BRepGProp::VolumeProperties(shape, volume);
    info.volume = volume.Mass();

    GProp_GProps surface;
    BRepGProp::SurfaceProperties(shape, surface);
    info.surfaceArea = surface.Mass();
    return info;
}

} // namespace

ModelInfo Model::info() const {
    std::call_once(impl_->info->once, [this] { impl_->info->info = computeInfo(impl_->shape); });
    return impl_->info->info;
}

const TopoDS_Shape& occtShape(const Model& model) {
    return model.impl().shape;
}

Model modelFromShape(const TopoDS_Shape& shape) {
    return Model(std::make_shared<const Model::Impl>(Model::Impl{shape}));
}

} // namespace ocam::cad
