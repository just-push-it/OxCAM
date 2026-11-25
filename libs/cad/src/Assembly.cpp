// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The STEP file is read into an XCAF document (names, colours, product structure); its free
// shapes are walked down through assemblies and component references, composing the
// references' locations, to the leaf products.

#include "ocam/cad/Assembly.hpp"

#include "ModelImpl.hpp"
#include "ocam/cad/occt/ShapeAccess.hpp"

#include <ocam/foundation/Error.hpp>

#include <fmt/format.h>

#include <DESTEP_Parameters.hxx>
#include <Quantity_Color.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <TCollection_AsciiString.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDF_Label.hxx>
#include <TDataStd_Name.hxx>
#include <TDocStd_Document.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <algorithm>
#include <regex>

namespace ocam::cad {
namespace {

std::string labelName(const TDF_Label& label) {
    occ::handle<TDataStd_Name> name;
    if (!label.FindAttribute(TDataStd_Name::GetID(), name)) {
        return {};
    }
    // Without a replacement character the conversion gives UTF-8.
    const TCollection_AsciiString text(name->Get());
    std::string out(text.ToCString());
    // The names OCCT gives products that have none ("Product 3.1.2", "Open CASCADE STEP
    // translator ...") say nothing.
    static const std::regex generated(
        "^(Product [0-9]+(\\.[0-9]+)*|Open CASCADE STEP translator.*)$");
    if (std::regex_match(out, generated)) {
        return {};
    }
    return out;
}

std::uint32_t rgb(const Quantity_Color& c) {
    double r = 0, g = 0, b = 0;
    c.Values(r, g, b, Quantity_TOC_sRGB);
    auto byte = [](double v) {
        return static_cast<std::uint32_t>(std::clamp(v, 0.0, 1.0) * 255.0 + 0.5);
    };
    return (byte(r) << 16) | (byte(g) << 8) | byte(b);
}

struct Walker {
    occ::handle<XCAFDoc_ShapeTool> shapes;
    occ::handle<XCAFDoc_ColorTool> colours;
    CancelToken& cancel;
    std::string fileName;
    std::vector<AssemblyPart> parts;

    /// The label's colour, else its shape's, else that of its first coloured sub-shape (STEP
    /// files often colour the solid or its faces rather than the product).
    std::optional<std::uint32_t> colourOf(const TDF_Label& label) const {
        Quantity_Color c;
        bool found = colours->GetColor(label, XCAFDoc_ColorSurf, c) ||
                     colours->GetColor(label, XCAFDoc_ColorGen, c);
        if (!found && !XCAFDoc_ShapeTool::IsAssembly(label) &&
            !XCAFDoc_ShapeTool::IsReference(label)) {
            const TopoDS_Shape shape = XCAFDoc_ShapeTool::GetShape(label);
            found = !shape.IsNull() && (colours->GetColor(shape, XCAFDoc_ColorSurf, c) ||
                                        colours->GetColor(shape, XCAFDoc_ColorGen, c));
            for (TDF_ChildIterator it(label); !found && it.More(); it.Next()) {
                found = colours->GetColor(it.Value(), XCAFDoc_ColorSurf, c) ||
                        colours->GetColor(it.Value(), XCAFDoc_ColorGen, c);
            }
        }
        if (found) {
            return rgb(c);
        }
        return std::nullopt;
    }

    /// The product `label` placed by `location`. Parts take their product's name: instance
    /// names are mostly generated ("1", "NAUO12").
    void visit(const TDF_Label& label, const TopLoc_Location& location,
               std::vector<std::string> path, std::optional<std::uint32_t> colour) {
        cancel.throwIfCancelled();
        const std::string name = labelName(label);
        if (const auto own = colourOf(label)) {
            colour = own;
        }
        if (XCAFDoc_ShapeTool::IsAssembly(label)) {
            path.push_back(name);
            NCollection_Sequence<TDF_Label> components;
            XCAFDoc_ShapeTool::GetComponents(label, components, false);
            for (const TDF_Label& component : components) {
                TDF_Label referred;
                if (!XCAFDoc_ShapeTool::GetReferredShape(component, referred)) {
                    continue;
                }
                std::optional<std::uint32_t> c = colour;
                if (const auto own = colourOf(component)) {
                    c = own;
                }
                visit(referred, location * XCAFDoc_ShapeTool::GetLocation(component), path, c);
            }
            return;
        }
        const TopoDS_Shape shape = XCAFDoc_ShapeTool::GetShape(label);
        if (shape.IsNull()) {
            return;
        }
        // Every body (solid) is a part of its own, named after the body where the file names
        // it; a product of several bodies becomes a level of the path.
        std::vector<TopoDS_Shape> bodies;
        for (TopExp_Explorer e(shape, TopAbs_SOLID); e.More(); e.Next()) {
            bodies.push_back(e.Current());
        }
        if (bodies.size() <= 1) {
            AssemblyPart part;
            part.name = name;
            if (bodies.size() == 1) {
                TDF_Label body;
                if (shapes->FindSubShape(label, bodies.front(), body)) {
                    if (const std::string own = labelName(body); !own.empty()) {
                        part.name = own;
                    }
                    if (const auto c = colourOf(body)) {
                        colour = c;
                    }
                }
            }
            part.path = std::move(path);
            part.model = modelFromShape(shape.Moved(location));
            part.colour = colour;
            part.solids = bodies.size();
            parts.push_back(std::move(part));
            return;
        }
        if (!name.empty()) {
            path.push_back(name);
        }
        int n = 0;
        for (const TopoDS_Shape& solid : bodies) {
            cancel.throwIfCancelled();
            ++n;
            AssemblyPart part;
            TDF_Label body;
            std::optional<std::uint32_t> c = colour;
            if (shapes->FindSubShape(label, solid, body)) {
                part.name = labelName(body);
                if (const auto own = colourOf(body)) {
                    c = own;
                }
            }
            if (!c) {
                Quantity_Color q;
                if (colours->GetColor(solid, XCAFDoc_ColorSurf, q) ||
                    colours->GetColor(solid, XCAFDoc_ColorGen, q)) {
                    c = rgb(q);
                }
            }
            if (part.name.empty() && !name.empty()) {
                part.name = fmt::format("{} {}", name, n);
            }
            part.path = path;
            part.model = modelFromShape(solid.Moved(location));
            part.colour = c;
            part.solids = 1;
            parts.push_back(std::move(part));
        }
    }
};

} // namespace

std::vector<AssemblyPart> importStepAssembly(const std::filesystem::path& path, CancelToken& cancel,
                                             ProgressSink& progress) {
    std::error_code ec;
    if (!std::filesystem::is_regular_file(path, ec)) {
        throw Error(ErrorCode::Io, "STEP file not found: " + utf8Path(path));
    }
    progress.report(0.0, "Reading STEP file");
    occ::handle<TDocStd_Document> doc;
    XCAFApp_Application::GetApplication()->NewDocument("MDTV-XCAF", doc);
    // Shapes come in millimetres whatever the file's unit.
    XCAFDoc_DocumentTool::SetLengthUnit(doc, 1.0, UnitsMethods_LengthUnit_Millimeter);
    STEPCAFControl_Reader reader;
    reader.SetNameMode(true);
    reader.SetColorMode(true);
    reader.SetLayerMode(false);
    DESTEP_Parameters params;
    params.ReadSubshapeNames = true; // the bodies' names
    if (reader.ReadFile(utf8Path(path).c_str(), params) != IFSelect_RetDone) {
        XCAFApp_Application::GetApplication()->Close(doc);
        throw Error(ErrorCode::Parse, "not a readable STEP file: " + utf8Path(path));
    }
    cancel.throwIfCancelled();
    occ::handle<ProgressAdapter> indicator = new ProgressAdapter(cancel, progress, 0.2, 0.9);
    const bool transferred = reader.Transfer(doc, indicator->Start());
    if (cancel.isCancelled()) {
        XCAFApp_Application::GetApplication()->Close(doc);
        cancel.throwIfCancelled();
    }
    if (!transferred) {
        XCAFApp_Application::GetApplication()->Close(doc);
        throw Error(ErrorCode::Parse,
                    "STEP file contains no transferable shape: " + utf8Path(path));
    }

    const std::u8string stem8 = path.stem().u8string();
    Walker walker{XCAFDoc_DocumentTool::ShapeTool(doc->Main()),
                  XCAFDoc_DocumentTool::ColorTool(doc->Main()),
                  cancel,
                  std::string(stem8.begin(), stem8.end()),
                  {}};
    NCollection_Sequence<TDF_Label> roots;
    walker.shapes->GetFreeShapes(roots);
    try {
        for (const TDF_Label& root : roots) {
            walker.visit(root, TopLoc_Location(), {}, std::nullopt);
        }
    } catch (...) {
        XCAFApp_Application::GetApplication()->Close(doc);
        throw;
    }
    XCAFApp_Application::GetApplication()->Close(doc);
    std::vector<AssemblyPart> parts = std::move(walker.parts);

    // Every part gets a name: the file's, numbered when several have none.
    const auto unnamedCount = std::count_if(parts.begin(), parts.end(),
                                            [](const AssemblyPart& p) { return p.name.empty(); });
    int unnamed = 0;
    for (AssemblyPart& p : parts) {
        if (p.name.empty()) {
            p.name = unnamedCount == 1 ? walker.fileName
                                       : fmt::format("{} {}", walker.fileName, ++unnamed);
        }
    }
    if (parts.empty()) {
        throw Error(ErrorCode::Parse,
                    "STEP file contains no transferable shape: " + utf8Path(path));
    }
    progress.report(1.0, "Done");
    return parts;
}

} // namespace ocam::cad
