// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Generates the STEP fixtures used by the tests: oxcam-fixture-gen <output-dir>
// Geometry is defined in FixtureSpecs.hpp.

#include "FixtureSpecs.hpp"

#include <ocam/cad/Assembly.hpp>
#include <ocam/cad/Features.hpp>
#include <ocam/cad/SubShape.hpp>
#include <ocam/cad/Tessellation.hpp>
#include <ocam/cad/occt/ShapeAccess.hpp>
#include <ocam/document/Document.hpp>
#include <ocam/document/MachineImport.hpp>
#include <ocam/document/Workholding.hpp>
#include <ocam/foundation/Error.hpp>
#include <ocam/foundation/Sha256.hpp>
#include <ocam/machine/Model.hpp>
#include <ocam/tooling/TurningHolder.hpp>

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <DESTEP_Parameters.hxx>
#include <Quantity_Color.hxx>
#include <STEPCAFControl_Writer.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TDataStd_Name.hxx>
#include <TDocStd_Document.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Vertex.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <functional>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Trsf.hxx>
#include <map>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <vector>

using namespace ocam;
using namespace ocam::fixtures;

void writeLatheMachineFixtures(const std::filesystem::path& out, ocam::CancelToken& cancel,
                               ocam::ProgressSink& progress);
void writeFittingModels(const std::filesystem::path& out);

namespace {

// Speeds and feeds come only from presets: give the sample tools their default preset.
tooling::Tool withPreset(tooling::Tool tool) {
    if (tool.presets.empty()) {
        tool.presets.push_back(tooling::defaultPreset(tool));
    }
    return tool;
}

// The step down and stepover are the preset's ap and ae; an operation changing its step down.
document::PresetOverrides stepDown(double ap) {
    document::PresetOverrides o;
    o.axialDepth = ap;
    return o;
}

// Small overshoot so tool bodies cleanly break through faces.
constexpr double kOvershoot = 1.0;

TopoDS_Shape cut(const TopoDS_Shape& a, const TopoDS_Shape& b) {
    BRepAlgoAPI_Cut op(a, b);
    if (!op.IsDone()) {
        throw std::runtime_error("boolean cut failed");
    }
    return op.Shape();
}

TopoDS_Shape fuse(const TopoDS_Shape& a, const TopoDS_Shape& b) {
    BRepAlgoAPI_Fuse op(a, b);
    if (!op.IsDone()) {
        throw std::runtime_error("boolean fuse failed");
    }
    return op.Shape();
}

// Vertical cylinder of diameter d from z0 to z1 at (x, y).
TopoDS_Shape zCylinder(double x, double y, double d, double z0, double z1) {
    return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(x, y, z0), gp_Dir(0, 0, 1)), d / 2, z1 - z0)
        .Shape();
}

TopoDS_Shape box(const Vec3& lo, const Vec3& hi) {
    return BRepPrimAPI_MakeBox(gp_Pnt(lo.x(), lo.y(), lo.z()), gp_Pnt(hi.x(), hi.y(), hi.z()))
        .Shape();
}

/// The trunnion machine as an assembly STEP file (written in inches) with subassemblies, a
/// support used twice and a set of bolts. Every shape is modelled
/// at its own origin and placed by its component's location.
void writeTrunnionAssembly(const std::filesystem::path& file) {
    using namespace trunnion_machine;
    occ::handle<TDocStd_Document> doc;
    XCAFApp_Application::GetApplication()->NewDocument("MDTV-XCAF", doc);
    const occ::handle<XCAFDoc_ShapeTool> shapes = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    const occ::handle<XCAFDoc_ColorTool> colours = XCAFDoc_DocumentTool::ColorTool(doc->Main());
    auto named = [](const TDF_Label& label, const char* name) {
        TDataStd_Name::Set(label, TCollection_ExtendedString(name, true));
        return label;
    };
    auto assembly = [&](const char* name) {
        return named(shapes->NewShape(), name);
    };
    // A product modelled at the corner of its bounding box, and placed there.
    struct Product {
        TDF_Label label;
        gp_Vec at;
    };
    auto product = [&](const char* name, const TopoDS_Shape& shape, std::uint32_t rgb) {
        Bnd_Box b;
        BRepBndLib::Add(shape, b);
        double x0, y0, z0, x1, y1, z1;
        b.Get(x0, y0, z0, x1, y1, z1);
        gp_Trsf toOrigin;
        toOrigin.SetTranslation(gp_Vec(-x0, -y0, -z0));
        // A shape without a location: XCAF would wrap a located one in an instance.
        const TopoDS_Shape atOrigin = BRepBuilderAPI_Transform(shape, toOrigin, true).Shape();
        const TDF_Label label = named(shapes->AddShape(atOrigin, false), name);
        colours->SetColor(label,
                          Quantity_Color(((rgb >> 16) & 0xFF) / 255.0, ((rgb >> 8) & 0xFF) / 255.0,
                                         (rgb & 0xFF) / 255.0, Quantity_TOC_sRGB),
                          XCAFDoc_ColorSurf);
        return Product{label, gp_Vec(x0, y0, z0)};
    };
    auto place = [&](const TDF_Label& parent, const Product& p, const gp_Vec& extra = {}) {
        gp_Trsf t;
        t.SetTranslation(p.at + extra);
        shapes->AddComponent(parent, p.label, TopLoc_Location(t));
    };
    auto sub = [&](const TDF_Label& parent, const TDF_Label& child) {
        shapes->AddComponent(parent, child, TopLoc_Location());
    };
    const double ci = kCradleInner, si = kSupportInner, top = kSpindleZ;
    const TDF_Label machine = assembly("Demo trunnion machine");

    const TDF_Label frame = assembly("Frame");
    place(frame, product("base", box(Vec3(-650, -450, -800), Vec3(650, 900, -420)), 0x5A6270));
    place(frame,
          product("column", box(Vec3(-320, kColumnFront, -420), Vec3(320, 900, 1300)), 0x6B7482));
    const TDF_Label bolts = assembly("Bolts");
    const Product bolt = product("bolt", zCylinder(0, 0, 16, -420, -380), 0x30343A);
    for (int i = 0; i < kBolts; ++i) {
        place(bolts, bolt, gp_Vec(i % 2 ? 600 : -600, -400 + 400.0 * (i / 2), 0));
    }
    sub(frame, bolts);
    sub(machine, frame);

    const TDF_Label saddle = assembly("Y saddle");
    place(saddle, product("saddle", box(Vec3(-420, -320, -420), Vec3(420, 320, -370)), 0x8A93A0));
    sub(machine, saddle);

    const TDF_Label xTable = assembly("X table");
    place(xTable, product("x table", box(Vec3(-520, -260, -370), Vec3(520, 260, -320)), 0x9AA3B0));
    const Product support =
        product("support", box(Vec3(-410, -130, -320), Vec3(-si, 130, 60)), 0x7D8794);
    place(xTable, support);
    place(xTable, support, gp_Vec(si + 410, 0, 0)); // the right one, from x = si
    sub(machine, xTable);

    const TDF_Label cradle = assembly("Cradle");
    place(cradle, product("cradle",
                          fuse(fuse(box(Vec3(-300, -125, -210), Vec3(300, 125, -170)),
                                    box(Vec3(-300, -125, -170), Vec3(-ci, 125, 60))),
                               box(Vec3(ci, -125, -170), Vec3(300, 125, 60))),
                          0xC28A3A));
    sub(machine, cradle);

    const TDF_Label table = assembly("Rotary table");
    place(table,
          product("rotary table",
                  fuse(zCylinder(0, 0, 2 * kTableRadius, -50, 0), zCylinder(0, 0, 220, -170, -50)),
                  0xB8BFC8));
    sub(machine, table);

    const TDF_Label head = assembly("Head");
    place(head, product("spindle nose", zCylinder(0, 0, 110, top, top + 110), 0xD0D4DA));
    place(head, product("spindle head",
                        fuse(box(Vec3(-150, -150, top + 110), Vec3(150, 400, top + 700)),
                             box(Vec3(-250, 400, top + 150), Vec3(250, kColumnFront, top + 800))),
                        0x4F6D8F));
    sub(machine, head);
    shapes->UpdateAssemblies();

    DESTEP_Parameters params;
    params.WriteUnit = UnitsMethods_LengthUnit_Inch;
    STEPCAFControl_Writer writer;
    writer.SetNameMode(true);
    writer.SetColorMode(true);
    const std::u8string u8 = file.u8string();
    if (!writer.Transfer(doc, params) ||
        writer.Write(std::string(u8.begin(), u8.end()).c_str()) != IFSelect_RetDone) {
        throw std::runtime_error("cannot write " + file.string());
    }
    XCAFApp_Application::GetApplication()->Close(doc);
    std::printf("wrote %s\n", file.string().c_str());
}

/// The bodies of the trunnion machine in machine coordinates (machine zero), with the axis that
/// carries each ("" = the base) and its colour; the bolts have no name of their own.
struct Body {
    const char* name;
    const char* carrier;
    std::uint32_t colour;
    TopoDS_Shape shape;
};
std::vector<Body> trunnionBodies() {
    using namespace trunnion_machine;
    const double ci = kCradleInner, si = kSupportInner, top = kSpindleZ;
    std::vector<Body> bodies = {
        {"base", "", 0x5A6270, box(Vec3(-650, -450, -800), Vec3(650, 900, -420))},
        {"enclosure", "", 0xC8CCD2,
         fuse(fuse(box(Vec3(-kEnclosureX, kEnclosureBack - 10, -800),
                       Vec3(kEnclosureX, kEnclosureBack, 1500)),
                   box(Vec3(-kEnclosureX, -800, -800),
                       Vec3(-kEnclosureX + 10, kEnclosureBack, 1500))),
              box(Vec3(kEnclosureX - 10, -800, -800), Vec3(kEnclosureX, kEnclosureBack, 1500)))},
        {"column", "", 0x6B7482, box(Vec3(-320, kColumnFront, -420), Vec3(320, 900, 1300))},
        {"saddle", "Y", 0x8A93A0, box(Vec3(-420, -320, -420), Vec3(420, 320, -370))},
        {"x table", "X", 0x9AA3B0, box(Vec3(-520, -260, -370), Vec3(520, 260, -320))},
        {"support left", "X", 0x7D8794, box(Vec3(-410, -130, -320), Vec3(-si, 130, 60))},
        {"support right", "X", 0x7D8794, box(Vec3(si, -130, -320), Vec3(410, 130, 60))},
        {"cradle", "A", 0xC28A3A,
         fuse(fuse(box(Vec3(-300, -125, -210), Vec3(300, 125, -170)),
                   box(Vec3(-300, -125, -170), Vec3(-ci, 125, 60))),
              box(Vec3(ci, -125, -170), Vec3(300, 125, 60)))},
        {"rotary table", "C", 0xB8BFC8,
         fuse(zCylinder(0, 0, 2 * kTableRadius, -50, 0), zCylinder(0, 0, 220, -170, -50))},
        {"spindle nose", "Z", 0xD0D4DA, zCylinder(0, 0, 110, top, top + 110)},
        {"spindle head", "Z", 0x4F6D8F,
         fuse(box(Vec3(-150, -150, top + 110), Vec3(150, 400, top + 700)),
              box(Vec3(-250, 400, top + 150), Vec3(250, kColumnFront, top + 800)))},
    };
    for (int i = 0; i < kBolts; ++i) {
        bodies.push_back({"", "", 0x30343A,
                          zCylinder(i % 2 ? 600 : -600, -400 + 400.0 * (i / 2), 16, -420, -380)});
    }
    return bodies;
}

/// The whole machine as one STEP part of many bodies, as machine makers often ship it: one
/// product, each body (solid) named (but the bolts) and coloured, written in inches.
void writeTrunnionBodies(const std::filesystem::path& file) {
    const std::vector<Body> bodies = trunnionBodies();
    occ::handle<TDocStd_Document> doc;
    XCAFApp_Application::GetApplication()->NewDocument("MDTV-XCAF", doc);
    const occ::handle<XCAFDoc_ShapeTool> shapes = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    const occ::handle<XCAFDoc_ColorTool> colours = XCAFDoc_DocumentTool::ColorTool(doc->Main());
    TopoDS_Compound all;
    BRep_Builder builder;
    builder.MakeCompound(all);
    for (const Body& b : bodies) {
        for (TopExp_Explorer e(b.shape, TopAbs_SOLID); e.More(); e.Next()) {
            builder.Add(all, e.Current());
        }
    }
    const TDF_Label machine = shapes->AddShape(all, false);
    TDataStd_Name::Set(machine, TCollection_ExtendedString("Demo trunnion machine", true));
    for (const Body& b : bodies) {
        for (TopExp_Explorer e(b.shape, TopAbs_SOLID); e.More(); e.Next()) {
            const TDF_Label body = shapes->AddSubShape(machine, e.Current());
            if (*b.name) {
                TDataStd_Name::Set(body, TCollection_ExtendedString(b.name, true));
            }
            colours->SetColor(body,
                              Quantity_Color(((b.colour >> 16) & 0xFF) / 255.0,
                                             ((b.colour >> 8) & 0xFF) / 255.0,
                                             (b.colour & 0xFF) / 255.0, Quantity_TOC_sRGB),
                              XCAFDoc_ColorSurf);
        }
    }
    DESTEP_Parameters params;
    params.WriteUnit = UnitsMethods_LengthUnit_Inch;
    params.WriteSubshapeNames = true;
    STEPCAFControl_Writer writer;
    writer.SetNameMode(true);
    writer.SetColorMode(true);
    const std::u8string u8 = file.u8string();
    if (!writer.Transfer(doc, params) ||
        writer.Write(std::string(u8.begin(), u8.end()).c_str()) != IFSelect_RetDone) {
        throw std::runtime_error("cannot write " + file.string());
    }
    XCAFApp_Application::GetApplication()->Close(doc);
    std::printf("wrote %s\n", file.string().c_str());
}

/// The trunnion machine built from its one-file STEP as the machine library editor does it:
/// every body a part of its own, each assigned to the base or the axis that carries it.
machine::Machine trunnionMachine(const std::filesystem::path& out) {
    using namespace trunnion_machine;
    const std::filesystem::path file = out / "trunnion_machine.step";
    writeTrunnionBodies(file);
    writeTrunnionAssembly(out / "trunnion_assembly.step");
    CancelToken cancel;
    NullProgress progress;
    const auto found = document::readMachinePartFile(file, 1.0, cancel, progress);
    machine::Machine m = machine::genericMill5TableAC();
    m.id = "trunnion-demo";
    m.name = "Demo 5-axis trunnion (one STEP file)";
    m.axes = {{"X", -400, 400, 30000 / 60.0},
              {"Y", -300, 300, 30000 / 60.0},
              {"Z", -560, 0, 24000 / 60.0},
              {"A", -110, 110, 15000 / 60.0},
              {"C", -360, 360, 30000 / 60.0}};
    m.components.clear();
    m.model = machine::defaultModel(m);
    m.model.spindlePoint = Vec3(0, 0, kSpindleZ);
    m.model.parts =
        document::machinePartsFrom(found, document::groupParts(found, 0), 0.2, cancel, progress);
    std::map<std::string, std::string> carriers;
    for (const Body& b : trunnionBodies()) {
        carriers[b.name] = b.carrier;
    }
    for (machine::ModelPart& p : m.model.parts) {
        // The bolts, unnamed in the file, take the machine's name ("Demo trunnion machine 12").
        const auto it = carriers.find(p.name);
        p.carrier = it == carriers.end() ? std::string() : it->second;
        p.shown = p.name != "enclosure"; // hidden in the views by default: it hides the work
    }
    if (const auto issues = m.validate(); !issues.empty()) {
        throw std::runtime_error("trunnion machine: " + issues.front());
    }
    machine::saveMachine(m, out / "machines" / "trunnion-demo.json");
    std::printf("wrote %s\n", (out / "machines" / "trunnion-demo.json").string().c_str());
    return m;
}

TopoDS_Shape viseJaws() {
    using namespace vise_jaws;
    const double w = block_pocket::kWidth;
    const TopoDS_Shape front =
        BRepPrimAPI_MakeBox(gp_Pnt(kX0, -kGap - kThickness, 0), gp_Pnt(kX1, -kGap, kTop)).Shape();
    const TopoDS_Shape back =
        BRepPrimAPI_MakeBox(gp_Pnt(kX0, w + kGap, 0), gp_Pnt(kX1, w + kGap + kThickness, kTop))
            .Shape();
    return fuse(front, back);
}

TopoDS_Shape box(double x0, double y0, double z0, double x1, double y1, double z1) {
    return BRepPrimAPI_MakeBox(gp_Pnt(x0, y0, z0), gp_Pnt(x1, y1, z1)).Shape();
}

TopoDS_Shape viseBase() {
    using namespace vise;
    return box(-kBaseHalfWidth, -10, 0, kBaseHalfWidth, kBaseLength - 10, kBaseTop);
}

TopoDS_Shape viseFixedJaw() {
    using namespace vise;
    return box(-kJawHalfWidth, kFixedFace - kJawThickness, kBaseTop, kJawHalfWidth, kFixedFace,
               kJawTop);
}

TopoDS_Shape viseMovingJaw() {
    using namespace vise;
    const double face = kFixedFace + kModelledOpening;
    return box(-kJawHalfWidth, face, kBaseTop, kJawHalfWidth, face + kJawThickness, kJawTop);
}

// Several solids as one shape (written as one multi-body STEP file).
TopoDS_Shape compound(const std::vector<TopoDS_Shape>& shapes) {
    BRep_Builder builder;
    TopoDS_Compound c;
    builder.MakeCompound(c);
    for (const auto& shape : shapes) {
        builder.Add(c, shape);
    }
    return c;
}

TopoDS_Shape selfCentringVise() {
    using namespace selfcentring;
    const double face = kModelledOpening / 2.0;
    return compound(
        {box(-60, -80, 0, 60, 80, kBaseTop),
         box(-kJawHalfWidth, -face - kJawThickness, kBaseTop, kJawHalfWidth, -face, kJawTop),
         box(-kJawHalfWidth, face, kBaseTop, kJawHalfWidth, face + kJawThickness, kJawTop)});
}

// The self-centring vise as a maker's file may have it: the first jaw in two bodies (a jaw and the
// plate on it) and a riser under the base the fixture does not have.
TopoDS_Shape selfCentringViseWithExtras() {
    using namespace selfcentring;
    const double face = kModelledOpening / 2.0;
    const double split = kJawTop - 10.0;
    return compound(
        {box(-60, -80, 0, 60, 80, kBaseTop),
         box(-kJawHalfWidth, -face - kJawThickness, kBaseTop, kJawHalfWidth, -face, split),
         box(-kJawHalfWidth, -face - kJawThickness, split, kJawHalfWidth, -face, kJawTop),
         box(-kJawHalfWidth, face, kBaseTop, kJawHalfWidth, face + kJawThickness, kJawTop),
         box(-60, -80, -40, 60, 80, 0)});
}

// A prism along X (from -length/2 to length/2) of a polygon in the YZ plane.
TopoDS_Shape prismYZ(const std::vector<std::pair<double, double>>& yz, double length) {
    BRepBuilderAPI_MakePolygon polygon;
    for (const auto& [y, z] : yz) {
        polygon.Add(gp_Pnt(-length / 2.0, y, z));
    }
    polygon.Close();
    const TopoDS_Face face = BRepBuilderAPI_MakeFace(polygon.Wire()).Face();
    return BRepPrimAPI_MakePrism(face, gp_Vec(length, 0, 0)).Shape();
}

TopoDS_Shape dovetailFixture() {
    using namespace dovetail;
    // Each flank leans in towards the top: the gap is narrower at the seat than at the base.
    const double top = kModelledWidth / 2.0;
    const double bottom =
        top + (kSeat - kBaseTop) * std::tan(kFlankAngle * std::numbers::pi / 180.0);
    return compound(
        {box(-kLength / 2.0, -kOuter, 0, kLength / 2.0, kOuter, kBaseTop),
         prismYZ({{-kOuter, kBaseTop}, {-bottom, kBaseTop}, {-top, kSeat}, {-kOuter, kSeat}},
                 kLength),
         prismYZ({{bottom, kBaseTop}, {kOuter, kBaseTop}, {kOuter, kSeat}, {top, kSeat}},
                 kLength)});
}

TopoDS_Shape fixturePlate() {
    using namespace plate;
    return compound({box(0, 0, 0, kLength, kWidth, kThickness),
                     zCylinder(kPinX, kPinY, kPinDiameter, kThickness, kThickness + kPinHeight)});
}

TopoDS_Shape blockPocket() {
    using namespace block_pocket;
    const TopoDS_Shape block = BRepPrimAPI_MakeBox(kLength, kWidth, kHeight).Shape();
    const TopoDS_Shape tool =
        BRepPrimAPI_MakeBox(gp_Pnt(kPocketX, kPocketY, kHeight - kPocketDepth), kPocketLength,
                            kPocketWidth, kPocketDepth + kOvershoot)
            .Shape();
    // Fillet the four vertical edges of the pocket tool.
    BRepFilletAPI_MakeFillet fillet(tool);
    for (TopExp_Explorer ex(tool, TopAbs_EDGE); ex.More(); ex.Next()) {
        const TopoDS_Edge& edge = TopoDS::Edge(ex.Current());
        TopExp_Explorer vx(edge, TopAbs_VERTEX);
        const gp_Pnt a = BRep_Tool::Pnt(TopoDS::Vertex(vx.Current()));
        vx.Next();
        const gp_Pnt b = BRep_Tool::Pnt(TopoDS::Vertex(vx.Current()));
        if (std::abs(a.X() - b.X()) < 1e-9 && std::abs(a.Y() - b.Y()) < 1e-9) {
            fillet.Add(kCornerRadius, edge);
        }
    }
    return cut(block, fillet.Shape());
}

TopoDS_Shape filletPocket() {
    using namespace fillet_pocket;
    const TopoDS_Shape block = BRepPrimAPI_MakeBox(kLength, kWidth, kHeight).Shape();
    const double overshoot = 1.0;
    const TopoDS_Shape box = BRepPrimAPI_MakeBox(gp_Pnt(kPocketX, kPocketY, kFloorZ), kPocketLength,
                                                 kPocketWidth, kPocketDepth + overshoot)
                                 .Shape();
    // The pocket tool: its vertical edges rounded, then the edges around its bottom face.
    BRepFilletAPI_MakeFillet corners(box);
    for (TopExp_Explorer ex(box, TopAbs_EDGE); ex.More(); ex.Next()) {
        const TopoDS_Edge& edge = TopoDS::Edge(ex.Current());
        TopExp_Explorer vx(edge, TopAbs_VERTEX);
        const gp_Pnt a = BRep_Tool::Pnt(TopoDS::Vertex(vx.Current()));
        vx.Next();
        const gp_Pnt b = BRep_Tool::Pnt(TopoDS::Vertex(vx.Current()));
        if (std::abs(a.X() - b.X()) < 1e-9 && std::abs(a.Y() - b.Y()) < 1e-9) {
            corners.Add(kCornerRadius, edge);
        }
    }
    const TopoDS_Shape rounded = corners.Shape();
    BRepFilletAPI_MakeFillet floor(rounded);
    for (TopExp_Explorer ex(rounded, TopAbs_EDGE); ex.More(); ex.Next()) {
        const TopoDS_Edge& edge = TopoDS::Edge(ex.Current());
        TopExp_Explorer vx(edge, TopAbs_VERTEX);
        const gp_Pnt a = BRep_Tool::Pnt(TopoDS::Vertex(vx.Current()));
        vx.Next();
        const gp_Pnt b = BRep_Tool::Pnt(TopoDS::Vertex(vx.Current()));
        if (std::abs(a.Z() - kFloorZ) < 1e-9 && std::abs(b.Z() - kFloorZ) < 1e-9) {
            floor.Add(kFloorRadius, edge);
        }
    }
    return cut(block, floor.Shape());
}

TopoDS_Shape holePlate() {
    using namespace hole_plate;
    const double top = kThickness;
    TopoDS_Shape s = BRepPrimAPI_MakeBox(kSize, kSize, kThickness).Shape();
    s = cut(s, zCylinder(kThroughX, kThroughY, kThroughDiameter, -kOvershoot, top + kOvershoot));
    s = cut(s, zCylinder(kFlatX, kFlatY, kFlatDiameter, top - kFlatDepth, top + kOvershoot));

    const double drillBottom = top - kDrillDepth;
    s = cut(s, zCylinder(kDrillX, kDrillY, kDrillDiameter, drillBottom, top + kOvershoot));
    const double tip = drillTipLength();
    const TopoDS_Shape cone =
        BRepPrimAPI_MakeCone(gp_Ax2(gp_Pnt(kDrillX, kDrillY, drillBottom - tip), gp_Dir(0, 0, 1)),
                             0.0, kDrillDiameter / 2, tip)
            .Shape();
    s = cut(s, cone);

    s = cut(s, zCylinder(kCboreX, kCboreY, kCboreDiameter, top - kCboreDepth, top + kOvershoot));
    s = cut(s, zCylinder(kCboreX, kCboreY, kCboreHoleDiameter, -kOvershoot, top + kOvershoot));
    return s;
}

TopoDS_Shape bossPlate() {
    using namespace boss_plate;
    TopoDS_Shape s = BRepPrimAPI_MakeBox(kSize, kSize, kThickness).Shape();
    s = fuse(s, zCylinder(kBossX, kBossY, kBossDiameter, kThickness, kThickness + kBossHeight));
    s = cut(s, zCylinder(kHoleX, kHoleY, kHoleDiameter, -kOvershoot, kThickness + kOvershoot));
    return s;
}

TopoDS_Shape domePart() {
    using namespace dome;
    TopoDS_Shape s = BRepPrimAPI_MakeBox(kBase, kBase, kBaseHeight).Shape();
    s = fuse(s, BRepPrimAPI_MakeSphere(gp_Pnt(kBase / 2, kBase / 2, kSphereCenterZ), kSphereRadius)
                    .Shape());
    // Keep only what is above the base's bottom face.
    const double big = 4 * kBase;
    return cut(s, BRepPrimAPI_MakeBox(gp_Pnt(-big, -big, -big), gp_Pnt(big, big, 0.0)).Shape());
}

TopoDS_Shape wedgeHole() {
    using namespace wedge_hole;
    BRepBuilderAPI_MakePolygon polygon;
    polygon.Add(gp_Pnt(0, 0, 0));
    polygon.Add(gp_Pnt(kLength, 0, 0));
    polygon.Add(gp_Pnt(kLength, 0, kLow));
    polygon.Add(gp_Pnt(0, 0, kHigh));
    polygon.Close();
    const TopoDS_Face face = BRepBuilderAPI_MakeFace(polygon.Wire()).Face();
    const TopoDS_Shape s = BRepPrimAPI_MakePrism(face, gp_Vec(0, kWidth, 0)).Shape();
    return cut(s, zCylinder(kHoleX, kHoleY, kHoleDiameter, -kOvershoot, kHigh + kOvershoot));
}

TopoDS_Shape swarfPart() {
    using namespace swarf;
    TopoDS_Shape s = BRepPrimAPI_MakeBox(kBaseX, kBaseY, kBaseHeight).Shape();
    s = fuse(s, BRepPrimAPI_MakeCone(gp_Ax2(gp_Pnt(kConeX, kConeY, kBaseHeight), gp_Dir(0, 0, 1)),
                                     kConeBottomRadius, kConeTopRadius, kBossHeight)
                    .Shape());
    // The twisted boss: ruled sides between the bottom square and the top one, turned.
    const auto square = [&](double z, double turn) {
        BRepBuilderAPI_MakePolygon poly;
        const double h = kTwistSide / 2;
        for (const auto& [cx, cy] : {std::pair{-h, -h}, {h, -h}, {h, h}, {-h, h}}) {
            const double x = cx * std::cos(turn) - cy * std::sin(turn);
            const double y = cx * std::sin(turn) + cy * std::cos(turn);
            poly.Add(gp_Pnt(kTwistX + x, kTwistY + y, z));
        }
        poly.Close();
        return poly.Wire();
    };
    BRepOffsetAPI_ThruSections loft(true, true);
    loft.AddWire(square(kBaseHeight, 0.0));
    loft.AddWire(square(kTop, kTwistAngle));
    loft.Build();
    if (!loft.IsDone()) {
        throw std::runtime_error("swarf: the twisted boss failed");
    }
    return fuse(s, loft.Shape());
}

TopoDS_Shape rotaryShaft() {
    using namespace rotary;
    const double R = kDiameter / 2;
    const gp_Dir x(1, 0, 0);
    TopoDS_Shape s = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, 0), x), R, kLength).Shape();
    // Annular sector centred on +Z: the sector starts at Vx and turns about +X.
    const double half = kGrooveAngle / 2;
    const gp_Dir vx(0, std::sin(half), std::cos(half));
    const gp_Ax2 grooveAxes(gp_Pnt(kGrooveX0, 0, 0), x, vx);
    const TopoDS_Shape sector =
        BRepPrimAPI_MakeCylinder(grooveAxes, R + kOvershoot, kGrooveX1 - kGrooveX0, kGrooveAngle)
            .Shape();
    const TopoDS_Shape core = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(kGrooveX0 - 1, 0, 0), x),
                                                       R - kGrooveDepth, kGrooveX1 - kGrooveX0 + 2)
                                  .Shape();
    s = cut(s, cut(sector, core));
    const TopoDS_Shape ring = cut(BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(kRingX0, 0, 0), x),
                                                           R + kOvershoot, kRingX1 - kRingX0)
                                      .Shape(),
                                  BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(kRingX0 - 1, 0, 0), x),
                                                           R - kRingDepth, kRingX1 - kRingX0 + 2)
                                      .Shape());
    return cut(s, ring);
}

TopoDS_Shape turnedAboutZ(const TopoDS_Shape& shape, double degrees) {
    gp_Trsf turn;
    turn.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), degrees * std::numbers::pi / 180.0);
    return BRepBuilderAPI_Transform(shape, turn, true).Shape();
}

TopoDS_Shape latheChuck() {
    using namespace chuck;
    std::vector<TopoDS_Shape> parts{
        cut(zCylinder(0, 0, kBodyDiameter, 0, kBodyLength),
            zCylinder(0, 0, kBore, -kOvershoot, kBodyLength + kOvershoot))};
    for (int i = 0; i < 3; ++i) {
        parts.push_back(turnedAboutZ(box(Vec3(kGripRadius, -kJawWidth / 2.0, kBodyLength),
                                         Vec3(kJawOuter, kJawWidth / 2.0, kJawFront)),
                                     120.0 * i));
    }
    return compound(parts);
}

TopoDS_Shape colletChuck() {
    using namespace collet;
    return compound({cut(zCylinder(0, 0, kBodyDiameter, 0, kBodyLength),
                         zCylinder(0, 0, kOuter, -kOvershoot, kBodyLength + kOvershoot)),
                     cut(zCylinder(0, 0, kOuter, 0, kFront),
                         zCylinder(0, 0, kBore, -kOvershoot, kFront + kOvershoot))});
}

TopoDS_Shape steppedShaft() {
    using namespace shaft;
    const TopoDS_Shape major = zCylinder(0, 0, kMajorDiameter, 0, kMajorLength);
    const TopoDS_Shape minor =
        zCylinder(0, 0, kMinorDiameter, kMajorLength, kMajorLength + kMinorLength);
    return fuse(major, minor);
}

TopoDS_Shape boredPart() {
    using namespace bored;
    const double neck = kLength - kShoulderLength;
    return cut(fuse(zCylinder(0, 0, kOuterDiameter, 0, neck),
                    zCylinder(0, 0, kShoulderDiameter, neck, kLength)),
               zCylinder(0, 0, kBoreDiameter, kLength - kBoreDepth, kLength + 1.0));
}

TopoDS_Shape livePart() {
    using namespace livetool;
    TopoDS_Shape part = zCylinder(0, 0, kDiameter, 0, kLength);
    for (int k = 0; k < 4; ++k) {
        const double a = k * std::numbers::pi / 2.0;
        part = cut(part, zCylinder(kBoltCircleRadius * std::cos(a), kBoltCircleRadius * std::sin(a),
                                   kBoltDiameter, kLength - kBoltDepth, kLength + 1.0));
    }
    const auto across = [](double d, double y, double z) {
        return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(-kDiameter, y, z), gp_Dir(1, 0, 0)), d / 2,
                                        2.0 * kDiameter)
            .Shape();
    };
    part = cut(part, across(kCrossDiameter, 0.0, kCrossZ));
    part = cut(part, across(kOffsetDiameter, kOffsetY, kOffsetZ));
    return part;
}

TopoDS_Shape millturnPart() {
    using namespace millturn;
    TopoDS_Shape part = zCylinder(0, 0, kDiameter, 0, kLength);
    const double half = kPocketSide / 2.0;
    part = cut(part, box(-half, -half, kLength - kPocketDepth, half, half, kLength + 1.0));
    part = cut(part, zCylinder(0, kBoreY, kBoreDiameter, kLength - kBoreDepth, kLength + 1.0));
    part = cut(part, zCylinder(kThreadX, 0, kThreadDrill, kLength - kThreadDepth, kLength + 1.0));
    part = cut(part, zCylinder(0, kTapY, kTapDrill, kLength - kTapDepth, kLength + 1.0));
    part = cut(part, box(kFlatX, -kDiameter, -1.0, kDiameter, kDiameter, kLength + 1.0));
    return part;
}

TopoDS_Shape studBar() {
    using namespace stud_bar;
    TopoDS_Shape part = zCylinder(0, 0, kDiameter, 0, kLength);
    part = cut(part, box(kFlatX, -kDiameter, -1.0, kDiameter, kDiameter, kLength + 1.0));
    const TopoDS_Shape stud =
        BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(kFlatX - 1.0, 0, kStudZ), gp_Dir(1, 0, 0)),
                                 kStudDiameter / 2, kStudLength + 1.0)
            .Shape();
    return fuse(part, stud);
}

TopoDS_Shape wrappedPart() {
    using namespace wrapped;
    const double R = kDiameter / 2.0;
    TopoDS_Shape part = fuse(zCylinder(0, 0, kNeckDiameter, kLength - kNeckLength, kLength),
                             zCylinder(0, 0, kDiameter, 0, kLength - kNeckLength));
    // An annular sector centred on +X.
    const double half = kSlotAngle / 2.0;
    const gp_Ax2 axes(gp_Pnt(0, 0, kSlotZ0), gp_Dir(0, 0, 1), gp_Dir(std::cos(half), -std::sin(half), 0));
    const TopoDS_Shape sector =
        BRepPrimAPI_MakeCylinder(axes, R + 1.0, kSlotZ1 - kSlotZ0, kSlotAngle).Shape();
    return cut(part, cut(sector, zCylinder(0, 0, kDiameter - 2.0 * kSlotDepth, kSlotZ0 - 1.0, kSlotZ1 + 1.0)));
}

TopoDS_Shape spoolPart() {
    using namespace spool;
    return fuse(fuse(zCylinder(0, 0, kEndDiameter, 0, kEndLength),
                     zCylinder(0, 0, kFlangeDiameter, kEndLength, kEndLength + kFlangeLength)),
                zCylinder(0, 0, kEndDiameter, kEndLength + kFlangeLength, kLength));
}

TopoDS_Shape groovedPart() {
    using namespace grooved;
    TopoDS_Shape part = zCylinder(0, 0, kOuterDiameter, 0, kLength);
    const auto ring = [](double d0, double d1, double z0, double z1) {
        return cut(zCylinder(0, 0, d1, z0, z1), zCylinder(0, 0, d0, z0 - 1.0, z1 + 1.0));
    };
    part = cut(part, zCylinder(0, 0, kBoreDiameter, -1.0, kLength + 1.0));
    part = cut(part, ring(kOutsideFloor, kOuterDiameter + 2.0, kOutsideLow, kOutsideHigh));
    part = cut(part, ring(kBoreDiameter - 2.0, kInsideRoof, kInsideLow, kInsideHigh));
    part = cut(part, ring(kFaceInner, kFaceOuter, kLength - kFaceDepth, kLength + 1.0));
    return part;
}

TopoDS_Shape taperedPart() {
    using namespace tapered;
    const double straight = kLength - kTaperLength;
    return fuse(zCylinder(0, 0, kDiameter, 0, straight),
                BRepPrimAPI_MakeCone(gp_Ax2(gp_Pnt(0, 0, straight), gp_Dir(0, 0, 1)),
                                     kDiameter / 2.0, kFrontDiameter / 2.0, kTaperLength)
                    .Shape());
}

TopoDS_Shape bushingPart() {
    using namespace bushing;
    return cut(zCylinder(0, 0, kOuterDiameter, 0, kLength),
               zCylinder(0, 0, kBoreDiameter, -1.0, kLength + 1.0));
}

} // namespace

int main(int argc, char** argv) {
    const bool fittingModels = argc == 3 && std::string_view(argv[2]) == "--fitting-models";
    if (argc != 2 && !fittingModels) {
        std::fprintf(stderr, "usage: oxcam-fixture-gen <output-dir> [--fitting-models]\n");
        return 1;
    }
    try {
        const std::filesystem::path out(argv[1]);
        std::filesystem::create_directories(out);
        if (fittingModels) {
            // What data/fittings holds: written once, so the built-ins' hashes stay the same.
            writeFittingModels(out);
            return 0;
        }

        const auto write = [&](const TopoDS_Shape& shape, const char* name,
                               LengthUnit unit = LengthUnit::Millimeter) {
            cad::modelFromShape(shape).exportStep(out / name, unit);
            std::printf("wrote %s\n", (out / name).string().c_str());
        };
        const TopoDS_Shape pocket = blockPocket();
        write(pocket, "block_pocket.step");
        write(pocket, "block_pocket_inch.step", LengthUnit::Inch);
        write(holePlate(), "hole_plate.step");
        write(bossPlate(), "boss_plate.step");
        write(steppedShaft(), "shaft.step");
        write(filletPocket(), "fillet_pocket.step");
        write(boredPart(), "bored.step");
        write(livePart(), "live.step");
        write(millturnPart(), "millturn.step");
        write(studBar(), "stud_bar.step");
        write(wrappedPart(), "wrapped.step");
        write(bushingPart(), "bushing.step");
        write(spoolPart(), "spool.step");
        write(groovedPart(), "grooved.step");
        write(taperedPart(), "tapered.step");
        write(domePart(), "dome.step");
        write(swarfPart(), "swarf.step");
        write(rotaryShaft(), "rotary_shaft.step");
        write(wedgeHole(), "wedge_hole.step");
        write(viseJaws(), "vise_jaws.step");
        write(viseBase(), "vise_base.step");
        write(viseFixedJaw(), "vise_fixed_jaw.step");
        write(viseMovingJaw(), "vise_moving_jaw.step");
        write(compound({viseBase(), viseFixedJaw(), viseMovingJaw()}), "vise_assembly.step");
        write(selfCentringVise(), "selfcentring_assembly.step");
        write(selfCentringViseWithExtras(), "selfcentring_extras.step");
        write(dovetailFixture(), "dovetail_assembly.step");
        write(fixturePlate(), "plate_assembly.step");
        write(latheChuck(), "chuck_assembly.step");
        write(colletChuck(), "collet_assembly.step");

        // A small project embedding block_pocket.step, for project-level tests.
        document::Document doc;
        CancelToken cancel;
        NullProgress progress;
        const Uuid model = doc.importModel(out / "block_pocket.step", cancel, progress);
        const Uuid setup = doc.addSetup(model);
        const cad::Model& blockModel = *doc.model(model);
        auto faceAt = [&](double height) {
            document::ShapeRef ref;
            for (const auto& f : cad::findPlanarFaces(blockModel)) {
                if (std::abs(f.height - height) < 1e-6) {
                    ref = {model, cad::SubShapeType::Face, f.face,
                           cad::signatureOf(blockModel, cad::SubShapeType::Face, f.face)};
                }
            }
            return ref;
        };
        doc.modify("Add tooling", [&](document::Project& p) {
            tooling::Tool mill = withPreset(tooling::makeFlatEndMill(1, 10.0, 22.0, 72.0));
            tooling::Tool small = withPreset(tooling::makeFlatEndMill(2, 6.0, 15.0, 57.0));
            auto op = [&](const char* name, const char* type, const tooling::Tool& tool,
                          std::vector<document::ShapeRef> geometry, Json params,
                          document::PresetOverrides overrides = {}) {
                document::Operation o;
                o.name = name;
                o.type = type;
                o.toolId = tool.id;
                o.presetId = tool.presets.front().id;
                o.geometry = std::move(geometry);
                o.parameters = std::move(params);
                o.presetOverrides = overrides;
                p.findSetup(setup)->operations.push_back(o);
            };
            using namespace block_pocket;
            op("Face top", "facing", mill, {}, Json::object());
            op("Pocket", "pocket2d", small, {faceAt(kHeight - kPocketDepth)}, Json::object(),
               stepDown(5.0));
            op("Profile", "contour2d", mill, {faceAt(0.0)}, Json{{"bottom_offset", 0.5}},
               stepDown(8.0));
            p.tools.tools.push_back(mill);
            p.tools.tools.push_back(small);
        });
        doc.save(out / "sample_project.ocam");
        std::printf("wrote %s\n", (out / "sample_project.ocam").string().c_str());

        // The block faced, then 2D traces (DL-0088) along two top edges meeting at a corner (an
        // open chain) and beside the pocket's mouth (a closed one, outside it).
        {
            document::Document traced;
            const Uuid block = traced.importModel(out / "block_pocket.step", cancel, progress);
            const Uuid tracedSetup = traced.addSetup(block);
            const cad::Model& m = *traced.model(block);
            using namespace block_pocket;
            std::vector<document::ShapeRef> corner;
            std::vector<document::ShapeRef> mouth;
            for (int i = 1; i <= cad::subShapeCount(m, cad::SubShapeType::Edge); ++i) {
                const auto sig = cad::signatureOf(m, cad::SubShapeType::Edge, i);
                const Vec3 c = sig.centroid;
                if (std::abs(c.z() - kHeight) > 1e-6) {
                    continue;
                }
                const document::ShapeRef ref{block, cad::SubShapeType::Edge, i, sig};
                if ((std::abs(c.y()) < 1e-6 && std::abs(sig.measure - kLength) < 1e-6) ||
                    (std::abs(c.x() - kLength) < 1e-6 && std::abs(sig.measure - kWidth) < 1e-6)) {
                    corner.push_back(ref);
                } else if (c.x() > kPocketX - 1e-6 && c.x() < kPocketX + kPocketLength + 1e-6 &&
                           c.y() > kPocketY - 1e-6 && c.y() < kPocketY + kPocketWidth + 1e-6) {
                    mouth.push_back(ref);
                }
            }
            traced.modify("Add tooling", [&](document::Project& p) {
                tooling::Tool mill = withPreset(tooling::makeFlatEndMill(1, 10.0, 22.0, 72.0));
                tooling::Tool ball = withPreset(tooling::makeBallEndMill(2, 6.0, 15.0, 57.0));
                auto op = [&](const char* name, const char* type, const tooling::Tool& tool,
                              std::vector<document::ShapeRef> geometry, Json params) {
                    document::Operation o;
                    o.name = name;
                    o.type = type;
                    o.toolId = tool.id;
                    o.presetId = tool.presets.front().id;
                    o.geometry = std::move(geometry);
                    o.parameters = std::move(params);
                    o.presetOverrides = stepDown(0.5);
                    p.findSetup(tracedSetup)->operations.push_back(o);
                };
                op("Face top", "facing", mill, {}, Json::object());
                op("Engrave corner", "trace2d", ball, corner, Json{{"depth", 0.6}});
                op("Beside the pocket", "trace2d", ball, mouth,
                   Json{{"depth", 0.3}, {"side", "right"}, {"offset", 1.0}, {"entry", "ramp"}});
                p.tools.tools.push_back(mill);
                p.tools.tools.push_back(ball);
            });
            traced.save(out / "trace_project.ocam");
            std::printf("wrote %s\n", (out / "trace_project.ocam").string().c_str());
        }

        // The block held in vise jaws: the profile keeps clear of the jaws, the tool sits in a
        // collet chuck.
        document::Document held;
        const Uuid heldModel = held.importModel(out / "block_pocket.step", cancel, progress);
        const Uuid heldSetup = held.addSetup(heldModel);
        held.addImportedFixture(document::importModelFile(out / "vise_jaws.step", cancel, progress),
                                heldSetup);
        const cad::Model& heldBlock = *held.model(heldModel);
        held.modify("Add tooling", [&](document::Project& p) {
            using namespace block_pocket;
            p.name = "block in a vise";
            tooling::HolderDimensions chuck;
            chuck.noseDiameter = 42.0;
            chuck.noseTopDiameter = 50.0;
            chuck.noseLength = 35.0;
            chuck.bodyDiameter = 0.0;
            chuck.flangeDiameter = 63.0;
            chuck.flangeLength = 16.0;
            tooling::Holder holder = tooling::makeHolder("ER32 collet chuck", chuck);
            tooling::Tool mill = withPreset(tooling::makeFlatEndMill(1, 10.0, 22.0, 72.0));
            mill.holderId = holder.id;
            mill.stickout = 40.0;
            auto faceAtHeight = [&](double height) {
                document::ShapeRef ref;
                for (const auto& f : cad::findPlanarFaces(heldBlock)) {
                    if (std::abs(f.height - height) < 1e-6) {
                        ref = {heldModel, cad::SubShapeType::Face, f.face,
                               cad::signatureOf(heldBlock, cad::SubShapeType::Face, f.face)};
                    }
                }
                return ref;
            };
            auto op = [&](const char* name, const char* type,
                          std::vector<document::ShapeRef> geometry, Json params,
                          document::PresetOverrides overrides = {}) {
                document::Operation o;
                o.name = name;
                o.type = type;
                o.toolId = mill.id;
                o.presetId = mill.presets.front().id;
                o.geometry = std::move(geometry);
                o.parameters = std::move(params);
                o.presetOverrides = overrides;
                p.findSetup(heldSetup)->operations.push_back(o);
            };
            op("Face top", "facing", {}, Json::object());
            op("Profile", "contour2d", {faceAtHeight(0.0)}, Json::object(), stepDown(6.0));
            op("Pocket", "pocket2d", {faceAtHeight(kHeight - kPocketDepth)}, Json::object(),
               stepDown(5.0));
            p.tools.holders = {holder};
            p.tools.tools = {mill};
        });
        held.save(out / "fixture_project.ocam");
        std::printf("wrote %s\n", (out / "fixture_project.ocam").string().c_str());

        // Workholding templates from one multi-body STEP file each (DL-0035): every body a part.
        auto importParts = [&](const char* file, document::WorkholdingPackage& package,
                               const std::vector<std::string>& names) {
            std::vector<std::pair<document::WorkholdingPart, geom3d::Aabb3>> parts;
            const auto bodies = cad::importStepAssembly(out / file, cancel, progress);
            if (bodies.size() != names.size()) {
                throw std::runtime_error(std::string("unexpected bodies in ") + file);
            }
            for (std::size_t i = 0; i < bodies.size(); ++i) {
                document::WorkholdingPart part =
                    document::addWorkholdingPart(package, names[i], bodies[i].model, file);
                parts.emplace_back(part, bodies[i].model.info().bounds);
            }
            return parts;
        };
        auto viseFrom = [&](const char* file, const char* name, const char* description,
                            bool centring, double maxOpening, const char* target) {
            document::WorkholdingPackage package;
            const auto parts =
                importParts(file, package,
                            centring ? std::vector<std::string>{"Base", "Jaw -Y", "Jaw +Y"}
                                     : std::vector<std::string>{"Base", "Fixed jaw", "Moving jaw"});
            document::ViseParts vise{parts, parts[1].first.id, parts[2].first.id, centring};
            std::string why;
            auto w =
                document::viseTemplate(name, vise, Vec3::UnitY(), Vec3::UnitZ(), maxOpening, &why);
            if (!w) {
                throw std::runtime_error(why);
            }
            w->description = description;
            package.workholding = std::move(*w);
            document::saveWorkholdingFile(out / target, package);
            std::printf("wrote %s\n", (out / target).string().c_str());
            return package;
        };
        const document::WorkholdingPackage vise =
            viseFrom("vise_assembly.step", "Test vise", "150 mm machine vise (test fixture)", false,
                     ocam::fixtures::vise::kMaxOpening, "vise.ocamhold");
        viseFrom("selfcentring_assembly.step", "Test self-centring vise",
                 "Self-centring vise (test fixture)", true,
                 ocam::fixtures::selfcentring::kMaxOpening, "selfcentring.ocamhold");
        {
            // The dovetail fixture: datums picked on the flanks and the seat.
            using namespace ocam::fixtures::dovetail;
            document::WorkholdingPackage package;
            const auto parts =
                importParts("dovetail_assembly.step", package, {"Base", "Fixed jaw", "Moving jaw"});
            document::WorkholdingTemplate& w = package.workholding;
            w.name = "Test dovetail";
            w.description = "Dovetail fixture, 30 mm tenon (test fixture)";
            w.members.push_back({"Moving jaw", Vec3::UnitY(), -5.0, 20.0, {}, 1.0});
            w.parts[2].member = "Moving jaw";
            auto pickFace = [&](std::size_t partIndex, const char* name, auto&& wanted) {
                const cad::Model model = cad::Model::importStepData(
                    package.files.at(w.parts[partIndex].sha256), name, cancel, progress);
                for (int f = 1; f <= cad::subShapeCount(model, cad::SubShapeType::Face); ++f) {
                    const document::WorkholdingDatum d = document::pickedDatum(
                        name, w.parts[partIndex], model, cad::SubShapeType::Face, f);
                    if (d.shape == document::DatumShape::Plane && wanted(d.direction)) {
                        w.datums.push_back(d);
                        return;
                    }
                }
                throw std::runtime_error(std::string("no face for ") + name);
            };
            auto leaning = [](double y) {
                return [y](const Vec3& n) {
                    return std::abs(n.z()) > 0.2 && n.y() * y > 0.5;
                };
            };
            pickFace(1, "Fixed flank", leaning(1.0));
            pickFace(2, "Moving flank", leaning(-1.0));
            pickFace(1, "Seat", [](const Vec3& n) { return n.z() > 0.99; });
            document::WorkholdingDatum bottom;
            bottom.name = "Bottom centre";
            bottom.direction = -Vec3::UnitZ();
            w.datums.push_back(bottom);
            w.mount = "Bottom centre";
            document::StockHolding h;
            h.name = "Dovetail";
            h.kind = document::HoldingKind::Dovetail;
            h.faceA = "Fixed flank";
            h.faceB = "Moving flank";
            h.seat = "Seat";
            h.depth = kDepth;
            w.holdings.push_back(h);
            (void)parts;
            document::saveWorkholdingFile(out / "dovetail.ocamhold", package);
            std::printf("wrote %s\n", (out / "dovetail.ocamhold").string().c_str());
        }
        {
            // The fixture plate: the stock's bottom centre on the plate's top centre, its X
            // along the plate's front edge.
            document::WorkholdingPackage package;
            importParts("plate_assembly.step", package, {"Plate", "Pin"});
            document::WorkholdingTemplate& w = package.workholding;
            w.name = "Test plate";
            w.description = "Fixture plate with a locating pin (test fixture)";
            const cad::Model plateModel = cad::Model::importStepData(
                package.files.at(w.parts[0].sha256), "plate", cancel, progress);
            for (int f = 1; f <= cad::subShapeCount(plateModel, cad::SubShapeType::Face); ++f) {
                document::WorkholdingDatum d = document::pickedDatum("Top", w.parts[0], plateModel,
                                                                     cad::SubShapeType::Face, f);
                if (d.direction.z() > 0.99) {
                    w.datums.push_back(d);
                    break;
                }
            }
            for (int e = 1; e <= cad::subShapeCount(plateModel, cad::SubShapeType::Edge); ++e) {
                document::WorkholdingDatum d = document::pickedDatum(
                    "Front edge", w.parts[0], plateModel, cad::SubShapeType::Edge, e);
                if (d.shape == document::DatumShape::Line && std::abs(d.direction.x()) > 0.99 &&
                    std::abs(d.point.y()) < 1e-6 && d.point.z() > 1.0) {
                    w.datums.push_back(d);
                    break;
                }
            }
            document::WorkholdingDatum corner;
            corner.name = "Corner";
            corner.direction = -Vec3::UnitZ();
            w.datums.push_back(corner);
            w.mount = "Corner";
            document::StockHolding h;
            h.name = "Seat";
            h.kind = document::HoldingKind::Seat;
            h.seat = "Top";
            h.xDatum = "Front edge";
            w.holdings.push_back(h);
            document::saveWorkholdingFile(out / "plate.ocamhold", package);
            std::printf("wrote %s\n", (out / "plate.ocamhold").string().c_str());
        }
        // The chuck and the collet, set up from the wizard's answers (as clicks would give them).
        auto wizardTemplate = [&](const char* file, const std::vector<std::string>& names,
                                  const char* target, auto&& answer) {
            document::WorkholdingPackage package;
            importParts(file, package, names);
            std::map<std::string, cad::Model> models;
            for (const auto& part : package.workholding.parts) {
                models.emplace(part.sha256,
                               cad::Model::importStepData(package.files.at(part.sha256), part.name,
                                                          cancel, progress));
            }
            const auto& parts = package.workholding.parts;
            // The face of part `i` whose geometry satisfies `wanted`.
            auto face = [&](std::size_t i, auto&& wanted) {
                const cad::Model& m = models.at(parts[i].sha256);
                for (int f = 1; f <= cad::subShapeCount(m, cad::SubShapeType::Face); ++f) {
                    if (wanted(cad::subShapeGeometry(m, cad::SubShapeType::Face, f))) {
                        return document::WorkholdingPick{parts[i].id, cad::SubShapeType::Face, f,
                                                         std::nullopt};
                    }
                }
                throw std::runtime_error(std::string("no such face in ") + file);
            };
            document::WorkholdingAnswers a = answer(parts, face);
            package.workholding = document::buildWorkholding(
                a, package.workholding.parts,
                [&](const document::WorkholdingPart& p) { return &models.at(p.sha256); });
            document::saveWorkholdingFile(out / target, package);
            std::printf("wrote %s\n", (out / target).string().c_str());
            return package;
        };
        using Geometry = cad::SubShapeGeometry;
        auto round = [](double radius) {
            return [radius](const Geometry& g) {
                return g.kind == Geometry::Kind::Cylinder && std::abs(g.radius - radius) < 1e-6;
            };
        };
        auto flat = [](const Vec3& normal, double along) {
            return [normal, along](const Geometry& g) {
                return g.kind == Geometry::Kind::Plane && g.direction.isApprox(normal, 1e-6) &&
                       std::abs(g.point.dot(normal) - along) < 1e-6;
            };
        };
        const document::WorkholdingPackage chuckPackage =
            wizardTemplate("chuck_assembly.step", {"Body", "Jaw 1", "Jaw 2", "Jaw 3"},
                           "chuck.ocamhold", [&](const auto& parts, auto&& face) {
                               using namespace ocam::fixtures::chuck;
                               document::WorkholdingAnswers a;
                               a.type = document::FixtureType::Chuck;
                               a.name = "Test 3-jaw chuck";
                               a.description = "160 mm 3-jaw lathe chuck (test fixture)";
                               a.moving = {parts[1].id, parts[2].id, parts[3].id};
                               a.axis = face(0, round(kBore / 2.0));
                               a.faceA = face(1, flat(-Vec3::UnitX(), -kGripRadius));
                               a.top = face(1, flat(Vec3::UnitZ(), kJawFront));
                               a.minOpening = kMinDiameter;
                               a.maxOpening = kMaxDiameter;
                               a.gripDepth = kGripLength;
                               return a;
                           });
        wizardTemplate("collet_assembly.step", {"Body", "Collet"}, "collet.ocamhold",
                       [&](const auto& parts, auto&& face) {
                           using namespace ocam::fixtures::collet;
                           (void)parts;
                           document::WorkholdingAnswers a;
                           a.type = document::FixtureType::Collet;
                           a.name = "Test collet";
                           a.description = "20 mm collet (test fixture)";
                           a.axis = face(1, round(kBore / 2.0));
                           a.faceA = face(1, round(kBore / 2.0));
                           a.top = face(1, flat(Vec3::UnitZ(), kFront));
                           a.gripRange = kGripRange;
                           a.gripDepth = kGripLength;
                           return a;
                       });

        // block_pocket's stock in the vise, the jaws closed on its width (WCS Y).
        document::Document inVise;
        const Uuid viseModel = inVise.importModel(out / "block_pocket.step", cancel, progress);
        const Uuid viseSetup = inVise.addSetup(viseModel);
        inVise.useWorkholding(vise, document::importWorkholdingBodies(vise, cancel, progress),
                              viseSetup);
        const cad::Model& viseBlock = *inVise.model(viseModel);
        inVise.modify("Add tooling", [&](document::Project& p) {
            p.name = "block in a vise template";
            document::Setup& s = *p.findSetup(viseSetup);
            s.workholding->rotation = 90;
            s.workholding->gripDepth = ocam::fixtures::vise::kGripDepth;
            tooling::Tool mill = withPreset(tooling::makeFlatEndMill(1, 10.0, 22.0, 72.0));
            document::ShapeRef bottom;
            for (const auto& f : cad::findPlanarFaces(viseBlock)) {
                if (std::abs(f.height) < 1e-6) {
                    bottom = {viseModel, cad::SubShapeType::Face, f.face,
                              cad::signatureOf(viseBlock, cad::SubShapeType::Face, f.face)};
                }
            }
            auto op = [&](const char* name, const char* type,
                          std::vector<document::ShapeRef> geometry,
                          document::PresetOverrides overrides = {}) {
                document::Operation o;
                o.name = name;
                o.type = type;
                o.toolId = mill.id;
                o.presetId = mill.presets.front().id;
                o.geometry = std::move(geometry);
                o.presetOverrides = overrides;
                s.operations.push_back(o);
            };
            op("Face top", "facing", {});
            op("Profile", "contour2d", {bottom}, stepDown(6.0));
            p.tools.tools = {mill};
        });
        inVise.save(out / "vise_project.ocam");
        std::printf("wrote %s\n", (out / "vise_project.ocam").string().c_str());
        // The same, its WCS origin at the bottom centre of the vise (a zero-point plate's WCS).
        inVise.modify("WCS origin on the vise", [&](document::Project& p) {
            document::WcsOrigin o;
            o.from = document::OriginFrom::FixtureBox;
            o.box = {1, 1, 0};
            p.findSetup(viseSetup)->originFrom = o;
        });
        inVise.save(out / "vise_origin_project.ocam");
        std::printf("wrote %s\n", (out / "vise_origin_project.ocam").string().c_str());

        // A 3D project on the dome: roughing, rest roughing, parallel and waterline finishing.
        document::Document dome;
        const Uuid domeModel = dome.importModel(out / "dome.step", cancel, progress);
        const Uuid domeSetup = dome.addSetup(domeModel);
        dome.modify("Add 3D operations", [&](document::Project& p) {
            p.name = "dome";
            const tooling::Tool rougher = withPreset(tooling::makeFlatEndMill(1, 12.0, 40.0, 90.0));
            const tooling::Tool rester = withPreset(tooling::makeFlatEndMill(2, 6.0, 40.0, 90.0));
            const tooling::Tool ball = withPreset(tooling::makeBallEndMill(3, 6.0, 40.0, 90.0));
            auto op = [&](const char* name, const char* type, const tooling::Tool& tool,
                          Json params, document::PresetOverrides overrides = {}) {
                document::Operation o;
                o.name = name;
                o.type = type;
                o.toolId = tool.id;
                o.presetId = tool.presets.front().id;
                o.parameters = std::move(params);
                o.presetOverrides = overrides;
                p.findSetup(domeSetup)->operations.push_back(o);
            };
            op("Rough", "zlevel_rough", rougher, Json::object(), stepDown(3.0));
            op("Rest rough", "zlevel_rough", rester,
               Json{{"rest_machining", true}, {"rest_resolution", 0.5}}, stepDown(1.0));
            op("Finish", "parallel3d", ball, Json{{"scallop_height", 0.02}});
            op("Walls", "waterline3d", ball, Json::object(), stepDown(1.0));
            p.tools.tools = {rougher, rester, ball};
        });
        dome.save(out / "dome_project.ocam");
        std::printf("wrote %s\n", (out / "dome_project.ocam").string().c_str());

        // An external thread milled on the boss plate's boss, after a pocket clears around it
        // (DL-0081).
        document::Document bossThread;
        const Uuid bossModel = bossThread.importModel(out / "boss_plate.step", cancel, progress);
        const Uuid bossSetup = bossThread.addSetup(bossModel);
        bossThread.modify("Add the operations", [&](document::Project& p) {
            using namespace boss_plate;
            p.name = "boss thread";
            const cad::Model& model = *bossThread.model(bossModel);
            document::ShapeRef plateTop;
            for (const auto& f : cad::findPlanarFaces(model)) {
                if (std::abs(f.height - kThickness) < 1e-6) {
                    plateTop = {bossModel, cad::SubShapeType::Face, f.face,
                                cad::signatureOf(model, cad::SubShapeType::Face, f.face)};
                }
            }
            const tooling::Tool mill = withPreset(tooling::makeFlatEndMill(1, 8.0, 25.0, 70.0));
            tooling::Tool thread = tooling::makeDefaultTool(tooling::ToolType::ThreadMill, 2, 6.0);
            thread.mill.threadPitch = 2.5;
            thread.mill.fluteLength = 20.0;
            thread.presets = {tooling::defaultPreset(thread)};
            auto op = [&](const char* name, const char* type, const tooling::Tool& tool,
                          std::vector<document::ShapeRef> geometry, Json params,
                          document::PresetOverrides overrides = {}) {
                document::Operation o;
                o.name = name;
                o.type = type;
                o.toolId = tool.id;
                o.presetId = tool.presets.front().id;
                o.geometry = std::move(geometry);
                o.parameters = std::move(params);
                o.presetOverrides = overrides;
                p.findSetup(bossSetup)->operations.push_back(o);
            };
            op("Clear around the boss", "pocket2d", mill, {plateTop}, Json::object(),
               stepDown(5.0));
            op("M20 thread", "threadmill", thread, {},
               Json{{"side", "external"}, {"major_diameter", kBossDiameter}, {"thread_depth", 12.0}});
            p.tools.tools = {mill, thread};
        });
        bossThread.save(out / "boss_thread_project.ocam");
        std::printf("wrote %s\n", (out / "boss_thread_project.ocam").string().c_str());

        // A part bored from solid bar on a lathe: face, drill, then rough and finish both sides.
        document::Document boredLathe;
        // The dome roughed, then finished in one steep and shallow operation; and the pocket
        // with filleted floor edges roughed, finished with a 10 mm ball and cleaned up in its
        // fillets with pencil passes of a 4 mm ball.
        auto finishing = [&](const char* step, const char* name, const char* file,
                             const std::function<void(document::Project&, const Uuid&)>& add) {
            document::Document d;
            const Uuid model = d.importModel(out / step, cancel, progress);
            const Uuid setup = d.addSetup(model);
            d.modify("Add 3D operations", [&](document::Project& p) {
                p.name = name;
                add(p, setup);
            });
            d.save(out / file);
            std::printf("wrote %s\n", (out / file).string().c_str());
        };
        const auto addOp = [](document::Project& p, const Uuid& setup, const char* name,
                              const char* type, const tooling::Tool& tool, Json params,
                              document::PresetOverrides overrides = {}) {
            document::Operation o;
            o.name = name;
            o.type = type;
            o.toolId = tool.id;
            o.presetId = tool.presets.front().id;
            o.parameters = std::move(params);
            o.presetOverrides = overrides;
            p.findSetup(setup)->operations.push_back(o);
        };
        finishing(
            "dome.step", "dome steep and shallow", "dome_steep_shallow_project.ocam",
            [&](document::Project& p, const Uuid& setup) {
                const tooling::Tool rougher =
                    withPreset(tooling::makeFlatEndMill(1, 12.0, 40.0, 90.0));
                const tooling::Tool ball = withPreset(tooling::makeBallEndMill(2, 6.0, 40.0, 90.0));
                addOp(p, setup, "Rough", "zlevel_rough", rougher, Json::object(), stepDown(3.0));
                addOp(p, setup, "Finish", "steep_shallow3d", ball,
                      Json{{"threshold_angle", 40.0},
                           {"overlap", 1.0},
                           {"steep_scallop", 0.02},
                           {"shallow_scallop", 0.02}});
                p.tools.tools = {rougher, ball};
            });
        finishing(
            "fillet_pocket.step", "fillet pocket", "fillet_pocket_project.ocam",
            [&](document::Project& p, const Uuid& setup) {
                const tooling::Tool rougher =
                    withPreset(tooling::makeFlatEndMill(1, 8.0, 40.0, 90.0));
                const tooling::Tool big = withPreset(tooling::makeBallEndMill(2, 10.0, 40.0, 90.0));
                const tooling::Tool small =
                    withPreset(tooling::makeBallEndMill(3, 4.0, 30.0, 90.0));
                addOp(p, setup, "Rough", "zlevel_rough", rougher, Json::object(), stepDown(3.0));
                addOp(p, setup, "Finish", "steep_shallow3d", big,
                      Json{{"steep_scallop", 0.02}, {"shallow_scallop", 0.02}});
                addOp(p, setup, "Pencil", "pencil3d", small,
                      Json{{"reference_diameter", 10.0},
                           {"offset_passes", 8},
                           {"scallop_height", 0.01}});
                p.tools.tools = {rougher, big, small};
            });

        const Uuid boredModel = boredLathe.importModel(out / "bored.step", cancel, progress);
        const Uuid boredSetup = boredLathe.addSetup(boredModel);
        boredLathe.modify("Add the operations", [&](document::Project& p) {
            p.name = "bored from solid";
            document::Setup& s = *p.findSetup(boredSetup);
            s.name = "Lathe";
            s.machine = "generic-2axis-lathe";
            s.originFrom.reset();
            s.wcs.origin = Vec3(0, 0, bored::kLength); // front face centre
            s.wcs.zAxis = Vec3::UnitZ();
            s.wcs.xAxis = Vec3::UnitX();
            s.stock.kind = document::StockKind::Box;
            s.stock.size = document::StockSize::OffsetFromPart;
            s.stock.offsetSide = 1.0;
            s.stock.offsetTop = 1.0;
            s.stock.offsetBottom = 0.0;
            tooling::Tool turn = tooling::makeDefaultTool(tooling::ToolType::TurningInsert, 1);
            turn.name = "CNMG 120404";
            turn.insert.noseRadius = 0.4;
            tooling::Tool drill = tooling::makeDefaultTool(tooling::ToolType::Drill, 2);
            drill.name = "Drill 16";
            drill.mill.diameter = 16.0;
            drill.mill.shankDiameter = 16.0;
            drill.mill.fluteLength = 70.0;
            drill.mill.overallLength = 110.0;
            drill.presets = {tooling::defaultPreset(drill)};
            tooling::Tool boring = tooling::makeDefaultTool(tooling::ToolType::TurningInsert, 3);
            boring.name = "Boring CCMT 0602";
            boring.insert.noseRadius = 0.4;
            auto op = [&](const char* name, const char* type, const tooling::Tool& tool,
                          Json params, document::PresetOverrides overrides = {}) {
                document::Operation o;
                o.name = name;
                o.type = type;
                o.toolId = tool.id;
                o.presetId = tool.presets.front().id;
                o.parameters = std::move(params);
                o.presetOverrides = overrides;
                o.heights = document::defaultHeights(o);
                s.operations.push_back(o);
            };
            const Json inside{{"side", "inside"}};
            op("Face", "turn_face", turn, Json::object());
            op("Drill", "drill", drill, Json::object());
            op("Rough outside", "turn_rough", turn, Json::object(), stepDown(1.5));
            op("Rough inside", "turn_rough", boring, inside, stepDown(1.0));
            op("Finish outside", "turn_finish", turn, Json::object());
            op("Finish inside", "turn_finish", boring, inside);
            p.tools.tools = {turn, drill, boring};
        });
        boredLathe.save(out / "bored_project.ocam");
        std::printf("wrote %s\n", (out / "bored_project.ocam").string().c_str());

        // The same part with its tools in holders: a shank outside, a bar in the bore.
        boredLathe.modify("Add the holders", [&](document::Project& p) {
            const tooling::Holder shank =
                tooling::makeTurningHolder(*tooling::parseDesignation("PCLNR 2525M 12"));
            const tooling::Holder bar =
                tooling::makeTurningHolder(*tooling::parseDesignation("S08K-SCLCR 06"));
            p.tools.holders = {shank, bar};
            p.tools.tools[0].holderId = shank.id;
            tooling::fitToHolder(p.tools.tools[0], shank);
            p.tools.tools[2].holderId = bar.id;
            tooling::fitToHolder(p.tools.tools[2], bar);
        });
        boredLathe.save(out / "holders_project.ocam");
        std::printf("wrote %s\n", (out / "holders_project.ocam").string().c_str());

        // A shaft with holes only a live tool can drill, on a lathe with live tooling and Y.
        document::Document liveLathe;
        const Uuid liveModel = liveLathe.importModel(out / "live.step", cancel, progress);
        const Uuid liveSetup = liveLathe.addSetup(liveModel);
        liveLathe.modify("Add the operations", [&](document::Project& p) {
            p.name = "live tooling";
            document::Setup& s = *p.findSetup(liveSetup);
            s.name = "Lathe";
            s.machine = "generic-lathe-live-y";
            s.post = document::PostChoice{"haas-lathe", false};
            s.originFrom.reset();
            s.wcs.origin = Vec3(0, 0, livetool::kLength); // front face centre
            s.wcs.zAxis = Vec3::UnitZ();
            s.wcs.xAxis = Vec3::UnitX();
            s.stock.kind = document::StockKind::Box;
            s.stock.size = document::StockSize::OffsetFromPart;
            s.stock.offsetSide = 1.0;
            s.stock.offsetTop = 1.0;
            s.stock.offsetBottom = 0.0;
            tooling::Tool turn = tooling::makeDefaultTool(tooling::ToolType::TurningInsert, 1);
            turn.name = "CNMG 120404";
            turn.insert.noseRadius = 0.4;
            const auto makeDrill = [](int number, double diameter) {
                tooling::Tool drill = tooling::makeDefaultTool(tooling::ToolType::Drill, number);
                drill.name = "Drill " + std::to_string(diameter);
                drill.mill.diameter = diameter;
                drill.mill.shankDiameter = diameter;
                drill.mill.fluteLength = 60.0;
                drill.mill.overallLength = 100.0;
                drill.presets = {tooling::defaultPreset(drill)};
                return drill;
            };
            const tooling::Tool bolt = makeDrill(2, livetool::kBoltDiameter);
            const tooling::Tool cross = makeDrill(3, livetool::kCrossDiameter);
            const tooling::Tool offset = makeDrill(4, livetool::kOffsetDiameter);
            auto op = [&](const char* name, const char* type, const tooling::Tool& tool,
                          Json params, std::optional<document::ToolOrientation> orientation = {},
                          document::PresetOverrides overrides = {}) {
                document::Operation o;
                o.name = name;
                o.type = type;
                o.toolId = tool.id;
                o.presetId = tool.presets.front().id;
                o.parameters = std::move(params);
                o.presetOverrides = overrides;
                o.orientation = orientation;
                o.heights = document::defaultHeights(o);
                s.operations.push_back(o);
            };
            // Across the part: the tool along the WCS's +X.
            document::ToolOrientation fromX;
            fromX.frame.zAxis = Vec3::UnitX();
            fromX.frame.xAxis = Vec3::UnitY();
            const Json live{{"drive", "live"}};
            op("Face", "turn_face", turn, Json::object());
            op("Rough", "turn_rough", turn, Json::object(), {}, stepDown(1.5));
            op("Bolt circle", "drill", bolt, live);
            op("Cross hole", "drill", cross, live, fromX);
            op("Off centre", "drill", offset, live, fromX);
            p.tools.tools = {turn, bolt, cross, offset};
        });
        liveLathe.save(out / "live_project.ocam");
        std::printf("wrote %s\n", (out / "live_project.ocam").string().c_str());

        // A bar turned and then milled by live tools, on a lathe with live tooling and Y.
        document::Document millTurn;
        const Uuid millTurnModel = millTurn.importModel(out / "millturn.step", cancel, progress);
        const Uuid millTurnSetup = millTurn.addSetup(millTurnModel);
        const cad::Model& millTurnPart = *millTurn.model(millTurnModel);
        const auto floorAt = [&](double height) {
            document::ShapeRef ref;
            for (const auto& f : cad::findPlanarFaces(millTurnPart)) {
                if (std::abs(f.height - height) < 1e-6) {
                    ref = {millTurnModel, cad::SubShapeType::Face, f.face,
                           cad::signatureOf(millTurnPart, cad::SubShapeType::Face, f.face)};
                }
            }
            return ref;
        };
        millTurn.modify("Add the operations", [&](document::Project& p) {
            using namespace millturn;
            p.name = "mill turn";
            document::Setup& s = *p.findSetup(millTurnSetup);
            s.name = "Lathe";
            s.machine = "generic-lathe-live-y";
            s.post = document::PostChoice{"haas-lathe", false};
            s.originFrom.reset();
            s.wcs.origin = Vec3(0, 0, kLength);
            s.wcs.zAxis = Vec3::UnitZ();
            s.wcs.xAxis = Vec3::UnitX();
            s.stock.kind = document::StockKind::Box;
            s.stock.size = document::StockSize::OffsetFromPart;
            s.stock.offsetSide = 1.0;
            s.stock.offsetTop = 1.0;
            s.stock.offsetBottom = 0.0;
            tooling::Tool turn = tooling::makeDefaultTool(tooling::ToolType::TurningInsert, 1);
            turn.name = "CNMG 120404";
            turn.insert.noseRadius = 0.4;
            const auto makeMill = [](int number, double diameter) {
                tooling::Tool t = withPreset(tooling::makeFlatEndMill(number, diameter, 20.0, 60.0));
                t.presets.front().surfaceSpeed = 1000.0; // 60 m/min: within the live tool's limit
                return t;
            };
            const tooling::Tool mill = makeMill(2, 6.0);
            const tooling::Tool boringMill = makeMill(7, 9.0);
            tooling::Tool threadMill = tooling::makeDefaultTool(tooling::ToolType::ThreadMill, 3, 6.0);
            threadMill.mill.threadPitch = kThreadPitch;
            threadMill.mill.shankDiameter = 4.0; // it has to fit the hole off the hole's centre
            threadMill.mill.fluteLength = 20.0;
            threadMill.mill.overallLength = 60.0;
            threadMill.presets = {tooling::defaultPreset(threadMill)};
            threadMill.presets.front().surfaceSpeed = 1000.0; // 60 m/min, as the mills
            const auto makeDrill = [](int number, double diameter) {
                tooling::Tool drill = tooling::makeDefaultTool(tooling::ToolType::Drill, number,
                                                               diameter);
                drill.mill.shankDiameter = diameter;
                drill.mill.fluteLength = 40.0;
                drill.mill.overallLength = 80.0;
                drill.presets = {tooling::defaultPreset(drill)};
                return drill;
            };
            const tooling::Tool threadDrill = makeDrill(4, kThreadDrill);
            const tooling::Tool tapDrill = makeDrill(5, kTapDrill);
            tooling::Tool tap = tooling::makeDefaultTool(tooling::ToolType::Tap, 6, kTapMajor);
            tap.mill.threadPitch = kTapPitch;
            tap.presets = {tooling::defaultPreset(tap)};
            auto op = [&](const char* name, const char* type, const tooling::Tool& tool,
                          Json params, std::vector<document::ShapeRef> geometry = {},
                          std::optional<document::ToolOrientation> orientation = {},
                          document::PresetOverrides overrides = {}) {
                document::Operation o;
                o.name = name;
                o.type = type;
                o.toolId = tool.id;
                o.presetId = tool.presets.front().id;
                o.parameters = std::move(params);
                o.geometry = std::move(geometry);
                o.presetOverrides = overrides;
                o.orientation = orientation;
                o.heights = document::defaultHeights(o);
                s.operations.push_back(o);
            };
            // The flat: from outside along the WCS's +X, its top the flat (WCS z 0 is the front).
            const document::ToolOrientation flat{
                document::Frame{Vec3(kFlatX, 0.0, -kLength / 2.0), Vec3::UnitY(), Vec3::UnitX()}, 0};
            const Json liveHole{{"drive", "live"}};
            const Json liveDrill{{"drive", "live"}, {"depth_mode", "tip"}};
            op("Face", "turn_face", turn, Json::object());
            op("Rough", "turn_rough", turn, Json::object(), {}, {}, stepDown(1.5));
            op("Pocket", "pocket2d", mill, Json::object(), {floorAt(kLength - kPocketDepth)});
            op("Counterbore", "bore", boringMill, Json{{"drive", "live"}});
            op("Thread drill", "drill", threadDrill, liveDrill);
            op("Thread mill", "threadmill", threadMill,
               Json{{"major_diameter", kThreadMajor}, {"depth", kThreadDepth - 1.0}});
            op("Tap drill", "drill", tapDrill, liveDrill);
            op("Tap", "tap", tap, liveHole);
            op("Flat", "facing", mill, Json{{"live_axes", "y"}}, {}, flat);
            op("Finish", "turn_finish", turn, Json::object());
            p.tools.tools = {turn, mill, boringMill, threadMill, threadDrill, tapDrill, tap};
        });
        millTurn.save(out / "millturn_project.ocam");
        std::printf("wrote %s\n", (out / "millturn_project.ocam").string().c_str());

        // A shaft turned, then a slot wrapped around its big diameter by cylindrical interpolation.
        document::Document wrappedDoc;
        const Uuid wrappedModel = wrappedDoc.importModel(out / "wrapped.step", cancel, progress);
        const Uuid wrappedSetup = wrappedDoc.addSetup(wrappedModel);
        wrappedDoc.modify("Add the operations", [&](document::Project& p) {
            using namespace wrapped;
            p.name = "wrapped";
            document::Setup& s = *p.findSetup(wrappedSetup);
            s.name = "Lathe";
            s.machine = "generic-lathe-live-y";
            s.post = document::PostChoice{"haas-lathe", false};
            s.originFrom.reset();
            s.wcs.origin = Vec3(0, 0, kLength);
            s.wcs.zAxis = Vec3::UnitZ();
            s.wcs.xAxis = Vec3::UnitX();
            s.stock.kind = document::StockKind::Box;
            s.stock.size = document::StockSize::OffsetFromPart;
            s.stock.offsetSide = 1.0;
            s.stock.offsetTop = 1.0;
            s.stock.offsetBottom = 0.0;
            tooling::Tool turn = tooling::makeDefaultTool(tooling::ToolType::TurningInsert, 1);
            turn.name = "CNMG 120404";
            turn.insert.noseRadius = 0.4;
            turn = withPreset(turn);
            tooling::Tool mill = withPreset(tooling::makeFlatEndMill(2, 6.0, 20.0, 60.0));
            mill.presets.front().surfaceSpeed = 1000.0; // 60 m/min: within the live tool's limit
            auto op = [&](const char* name, const char* type, const tooling::Tool& tool, Json params) {
                document::Operation o;
                o.name = name;
                o.type = type;
                o.toolId = tool.id;
                o.presetId = tool.presets.front().id;
                o.parameters = std::move(params);
                o.heights = document::defaultHeights(o);
                s.operations.push_back(o);
            };
            op("Face", "turn_face", turn, Json::object());
            op("Rough", "turn_rough", turn, Json::object());
            op("Finish", "turn_finish", turn, Json::object());
            op("Slot", "rotary_pocket", mill, Json::object());
            p.tools.tools = {turn, mill};
        });
        wrappedDoc.save(out / "wrapped_project.ocam");
        std::printf("wrote %s\n", (out / "wrapped_project.ocam").string().c_str());

        // A spool roughed across Z, then each side of its flange with a Z range and a flip.
        document::Document ranged;
        const Uuid rangedModel = ranged.importModel(out / "spool.step", cancel, progress);
        const Uuid rangedSetup = ranged.addSetup(rangedModel);
        ranged.modify("Add the operations", [&](document::Project& p) {
            using namespace spool;
            p.name = "Z ranges";
            document::Setup& s = *p.findSetup(rangedSetup);
            s.name = "Lathe";
            s.machine = "generic-2axis-lathe";
            s.originFrom.reset();
            s.wcs.origin = Vec3(0, 0, kLength);
            s.wcs.zAxis = Vec3::UnitZ();
            s.wcs.xAxis = Vec3::UnitX();
            s.stock.kind = document::StockKind::Box;
            s.stock.size = document::StockSize::OffsetFromPart;
            s.stock.offsetSide = 1.0;
            s.stock.offsetTop = 1.0;
            s.stock.offsetBottom = 0.0;
            tooling::Tool turn = tooling::makeDefaultTool(tooling::ToolType::TurningInsert, 1);
            turn.name = "CNMG 120404";
            turn.insert.noseRadius = 0.4;
            auto op = [&](const char* name, const char* type, Json params,
                          document::PresetOverrides overrides = {}) {
                document::Operation o;
                o.name = name;
                o.type = type;
                o.toolId = turn.id;
                o.presetId = turn.presets.front().id;
                o.parameters = std::move(params);
                o.presetOverrides = overrides;
                o.heights = document::defaultHeights(o);
                s.operations.push_back(o);
            };
            op("Face", "turn_face", Json::object());
            op("Rough across to the flange", "turn_rough",
               Json{{"pattern", "across"},
                    {"z_range", "limited"},
                    {"z_front", 0.0},
                    {"z_back", -kEndLength}},
               stepDown(2.0));
            op("Finish to the flange", "turn_finish",
               Json{{"z_range", "limited"}, {"z_front", 0.0}, {"z_back", -kEndLength}});
            p.tools.tools = {turn};
        });
        ranged.save(out / "ranges_project.ocam");
        std::printf("wrote %s\n", (out / "ranges_project.ocam").string().c_str());

        // A spool with both ends turned towards the flange: the near one towards the chuck, the
        // far one away from it, with a left-hand tool from behind.
        document::Document backTurn;
        const Uuid backModel = backTurn.importModel(out / "spool.step", cancel, progress);
        const Uuid backSetup = backTurn.addSetup(backModel);
        backTurn.modify("Add the operations", [&](document::Project& p) {
            using namespace spool;
            p.name = "back turning";
            document::Setup& s = *p.findSetup(backSetup);
            s.name = "Lathe";
            s.machine = "generic-2axis-lathe";
            s.originFrom.reset();
            s.wcs.origin = Vec3(0, 0, kLength);
            s.wcs.zAxis = Vec3::UnitZ();
            s.wcs.xAxis = Vec3::UnitX();
            s.stock.kind = document::StockKind::Box;
            s.stock.size = document::StockSize::OffsetFromPart;
            s.stock.offsetSide = 1.0;
            s.stock.offsetTop = 1.0;
            s.stock.offsetBottom = 0.0;
            tooling::Tool right = tooling::makeDefaultTool(tooling::ToolType::TurningInsert, 1);
            right.name = "Right hand";
            right.insert.noseRadius = 0.4;
            right.insert.hand = "R";
            tooling::Tool left = tooling::makeDefaultTool(tooling::ToolType::TurningInsert, 2);
            left.name = "Left hand";
            left.insert.noseRadius = 0.4;
            left.insert.hand = "L";
            auto op = [&](const char* name, const char* type, const tooling::Tool& tool,
                          Json params) {
                document::Operation o;
                o.name = name;
                o.type = type;
                o.toolId = tool.id;
                o.presetId = tool.presets.front().id;
                o.parameters = std::move(params);
                o.presetOverrides = stepDown(1.5);
                o.heights = document::defaultHeights(o);
                s.operations.push_back(o);
            };
            const Json front{{"z_range", "limited"}, {"z_front", 0.0}, {"z_back", -kEndLength}};
            const Json back{{"z_range", "limited"},
                            {"z_front", -kEndLength - kFlangeLength},
                            {"z_back", -kLength},
                            {"direction", "away_from_chuck"}};
            op("Face", "turn_face", right, Json::object());
            op("Rough front", "turn_rough", right, front);
            op("Finish front", "turn_finish", right, front);
            op("Rough back", "turn_rough", left, back);
            op("Finish back", "turn_finish", left, back);
            p.tools.tools = {right, left};
        });
        backTurn.save(out / "backturn_project.ocam");
        std::printf("wrote %s\n", (out / "backturn_project.ocam").string().c_str());

        // A taper finished with the nose compensated in the computer, then by the control.
        document::Document nose;
        const Uuid noseModel = nose.importModel(out / "tapered.step", cancel, progress);
        const Uuid noseSetup = nose.addSetup(noseModel);
        nose.modify("Add the operations", [&](document::Project& p) {
            using namespace tapered;
            p.name = "nose compensation";
            document::Setup& s = *p.findSetup(noseSetup);
            s.name = "Lathe";
            s.machine = "generic-2axis-lathe";
            s.originFrom.reset();
            s.wcs.origin = Vec3(0, 0, kLength);
            s.wcs.zAxis = Vec3::UnitZ();
            s.wcs.xAxis = Vec3::UnitX();
            s.stock.kind = document::StockKind::Box;
            s.stock.size = document::StockSize::OffsetFromPart;
            s.stock.offsetSide = 1.0;
            s.stock.offsetTop = 1.0;
            s.stock.offsetBottom = 0.0;
            tooling::Tool turn = tooling::makeDefaultTool(tooling::ToolType::TurningInsert, 1);
            turn.name = "CNMG 120408";
            turn.insert.noseRadius = 0.8;
            auto op = [&](const char* name, const char* type, Json params,
                          document::PresetOverrides overrides = {}) {
                document::Operation o;
                o.name = name;
                o.type = type;
                o.toolId = turn.id;
                o.presetId = turn.presets.front().id;
                o.parameters = std::move(params);
                o.presetOverrides = overrides;
                o.heights = document::defaultHeights(o);
                s.operations.push_back(o);
            };
            op("Face", "turn_face", Json::object());
            op("Rough", "turn_rough", Json::object(), stepDown(1.5));
            op("Finish in computer", "turn_finish", Json{{"stock_to_leave", 0.1}});
            op("Finish in control", "turn_finish", Json{{"compensation", "control"}});
            p.tools.tools = {turn};
        });
        nose.save(out / "nose_project.ocam");
        std::printf("wrote %s\n", (out / "nose_project.ocam").string().c_str());

        // A shaft faced and centre drilled in one setup, then turned on a tailstock centre.
        document::Document centres;
        const Uuid centresModel = centres.importModel(out / "shaft.step", cancel, progress);
        const Uuid firstOp = centres.addSetup(centresModel);
        const Uuid betweenCentres = centres.addSetup(centresModel);
        centres.modify("Add the operations", [&](document::Project& p) {
            using namespace shaft;
            p.name = "on a tailstock";
            tooling::Tool turn = tooling::makeDefaultTool(tooling::ToolType::TurningInsert, 1);
            turn.name = "CNMG 120404";
            turn.insert.noseRadius = 0.4;
            tooling::Tool centre = tooling::makeDefaultTool(tooling::ToolType::CenterDrill, 2);
            centre.name = "Centre drill";
            centre.mill.diameter = 6.3;
            centre.mill.tipDiameter = 2.5;
            centre.mill.tipLength = 3.0;
            centre.mill.fluteLength = 10.0;
            centre.mill.overallLength = 50.0;
            centre.presets = {tooling::defaultPreset(centre)};
            p.tools.tools = {turn, centre};
            const auto op = [](document::Setup& s, const char* name, const char* type,
                               const tooling::Tool& tool, Json params,
                               document::PresetOverrides overrides = {}) {
                document::Operation o;
                o.name = name;
                o.type = type;
                o.toolId = tool.id;
                o.presetId = tool.presets.front().id;
                o.parameters = std::move(params);
                o.presetOverrides = overrides;
                o.heights = document::defaultHeights(o);
                s.operations.push_back(o);
            };
            const auto lathe = [](document::Setup& s, const char* name) {
                s.name = name;
                s.machine = "generic-2axis-lathe";
                s.originFrom.reset();
                s.wcs.origin = Vec3(0, 0, kMajorLength + kMinorLength);
                s.wcs.zAxis = Vec3::UnitZ();
                s.wcs.xAxis = Vec3::UnitX();
            };
            document::Setup& one = *p.findSetup(firstOp);
            lathe(one, "Face and centre");
            one.stock.kind = document::StockKind::Box;
            one.stock.size = document::StockSize::OffsetFromPart;
            one.stock.offsetSide = 1.0;
            one.stock.offsetTop = 1.0;
            one.stock.offsetBottom = 5.0;
            op(one, "Face", "turn_face", turn, Json::object());
            op(one, "Centre drill", "drill", centre,
               Json{{"hole_depth", 5.0}, {"depth_mode", "tip"}});
            document::Setup& two = *p.findSetup(betweenCentres);
            lathe(two, "On the tailstock");
            two.stock.kind = document::StockKind::FromSetup;
            two.stock.sourceSetup = firstOp;
            two.tailstock = document::LatheTailstock{};
            two.tailstock->tipZ = -2.0;
            two.fixtureClearance = 0.2;
            two.tailstock->centreDiameter = 30.0;
            two.tailstock->quillDiameter = 30.0;
            op(two, "Rough", "turn_rough", turn, Json::object(), stepDown(1.5));
            op(two, "Finish", "turn_finish", turn, Json::object());
        });
        centres.save(out / "centres_project.ocam");
        std::printf("wrote %s\n", (out / "centres_project.ocam").string().c_str());

        // Grooves of each kind on a part turned from tube.
        document::Document grooves;
        const Uuid groovedModel = grooves.importModel(out / "grooved.step", cancel, progress);
        const Uuid grooveSetup = grooves.addSetup(groovedModel);
        grooves.modify("Add the operations", [&](document::Project& p) {
            p.name = "grooves";
            document::Setup& s = *p.findSetup(grooveSetup);
            s.name = "Lathe";
            s.machine = "generic-2axis-lathe";
            s.originFrom.reset();
            s.wcs.origin = Vec3(0, 0, grooved::kLength);
            s.wcs.zAxis = Vec3::UnitZ();
            s.wcs.xAxis = Vec3::UnitX();
            s.stock.kind = document::StockKind::Tube;
            s.stock.innerDiameter = grooved::kBoreDiameter - 2.0;
            s.stock.size = document::StockSize::OffsetFromPart;
            s.stock.offsetSide = 1.0;
            s.stock.offsetTop = 1.0;
            s.stock.offsetBottom = 0.0;
            tooling::Tool turn = tooling::makeDefaultTool(tooling::ToolType::TurningInsert, 1);
            turn.name = "CNMG 120404";
            turn.insert.noseRadius = 0.4;
            tooling::Tool groove = tooling::makeDefaultTool(tooling::ToolType::GroovingInsert, 2);
            groove.name = "Groove 2 mm";
            groove.insert.width = 2.0;
            groove.insert.noseRadius = 0.0;
            auto op = [&](const char* name, const char* type, const tooling::Tool& tool,
                          Json params, document::PresetOverrides overrides = {}) {
                document::Operation o;
                o.name = name;
                o.type = type;
                o.toolId = tool.id;
                o.presetId = tool.presets.front().id;
                o.parameters = std::move(params);
                o.presetOverrides = overrides;
                o.heights = document::defaultHeights(o);
                s.operations.push_back(o);
            };
            const Json inside{{"side", "inside"}};
            op("Face", "turn_face", turn, Json::object());
            op("Finish outside", "turn_finish", turn, Json::object());
            op("Finish inside", "turn_finish", turn, inside);
            op("Groove outside", "turn_groove", groove, Json::object());
            op("Groove inside", "turn_groove", groove, inside);
            op("Groove face", "turn_groove", groove, Json{{"side", "face"}});
            p.tools.tools = {turn, groove};
        });
        grooves.save(out / "grooved_project.ocam");
        std::printf("wrote %s\n", (out / "grooved_project.ocam").string().c_str());

        // The kinds of thread on a bushing: inside, left hand, two starts and tapered.
        document::Document threads;
        const Uuid bushingModel = threads.importModel(out / "bushing.step", cancel, progress);
        const Uuid threadSetup = threads.addSetup(bushingModel);
        threads.modify("Add the operations", [&](document::Project& p) {
            p.name = "threads";
            document::Setup& s = *p.findSetup(threadSetup);
            s.name = "Lathe";
            s.machine = "generic-2axis-lathe";
            s.originFrom.reset();
            s.wcs.origin = Vec3(0, 0, bushing::kLength);
            s.wcs.zAxis = Vec3::UnitZ();
            s.wcs.xAxis = Vec3::UnitX();
            s.stock.kind = document::StockKind::Box;
            s.stock.size = document::StockSize::OffsetFromPart;
            s.stock.offsetSide = 1.0;
            s.stock.offsetTop = 1.0;
            s.stock.offsetBottom = 3.0;
            tooling::Tool drill = tooling::makeDefaultTool(tooling::ToolType::Drill, 2);
            drill.name = "Tap drill";
            drill.mill.diameter = bushing::kBoreDiameter;
            drill.mill.shankDiameter = bushing::kBoreDiameter;
            drill.mill.fluteLength = 70.0;
            drill.mill.overallLength = 110.0;
            drill.presets = {tooling::defaultPreset(drill)};
            tooling::Tool insert = tooling::makeDefaultTool(tooling::ToolType::ThreadingInsert, 3);
            insert.name = "Threading 1.5";
            insert.insert.threadPitch = 1.5;
            auto op = [&](const char* name, const char* type, const tooling::Tool& tool,
                          Json params) {
                document::Operation o;
                o.name = name;
                o.type = type;
                o.toolId = tool.id;
                o.presetId = tool.presets.front().id;
                o.parameters = std::move(params);
                o.heights = document::defaultHeights(o);
                s.operations.push_back(o);
            };
            const double major = bushing::kOuterDiameter;
            op("Drill", "drill", drill, Json::object());
            op("Inside", "turn_thread", insert, Json{{"side", "inside"}, {"length", 20.0}});
            op("Left hand", "turn_thread", insert,
               Json{{"major_diameter", major},
                    {"length", 15.0},
                    {"hand", "left"},
                    {"start_offset", 10.0}});
            op("Two starts", "turn_thread", insert,
               Json{{"major_diameter", major}, {"length", 15.0}, {"starts", 2}});
            op("Tapered", "turn_thread", insert,
               Json{{"major_diameter", major}, {"length", 20.0}, {"taper", 1.5}});
            p.tools.tools = {drill, insert};
        });
        threads.save(out / "threads_project.ocam");
        std::printf("wrote %s\n", (out / "threads_project.ocam").string().c_str());

        // A part turned from both sides: the second setup flips it and takes up the first's stock.
        document::Document flipLathe;
        const Uuid spoolModel = flipLathe.importModel(out / "spool.step", cancel, progress);
        const Uuid firstSide = flipLathe.addSetup(spoolModel);
        const Uuid secondSide = flipLathe.addSetup(spoolModel);
        flipLathe.modify("Add the operations", [&](document::Project& p) {
            p.name = "turned from both sides";
            tooling::Tool turn = tooling::makeDefaultTool(tooling::ToolType::TurningInsert, 1);
            turn.name = "CNMG 120404";
            turn.insert.noseRadius = 0.4;
            p.tools.tools = {turn};
            const auto side = [&](document::Setup& s, const char* name, double z,
                                  double direction) {
                s.name = name;
                s.machine = "generic-2axis-lathe";
                s.originFrom.reset();
                s.wcs.origin = Vec3(0, 0, z); // the face this side starts from
                s.wcs.zAxis = direction * Vec3::UnitZ();
                s.wcs.xAxis = Vec3::UnitX();
                for (const char* type : {"turn_face", "turn_rough", "turn_finish"}) {
                    document::Operation o;
                    o.name = type;
                    o.type = type;
                    o.toolId = turn.id;
                    o.presetId = turn.presets.front().id;
                    o.parameters = Json::object();
                    o.presetOverrides = stepDown(1.5);
                    o.heights = document::defaultHeights(o);
                    s.operations.push_back(o);
                }
            };
            document::Setup& one = *p.findSetup(firstSide);
            side(one, "First side", spool::kLength, 1.0);
            one.stock.kind = document::StockKind::Box;
            one.stock.size = document::StockSize::OffsetFromPart;
            one.stock.offsetSide = 1.0;
            one.stock.offsetTop = 1.0;
            one.stock.offsetBottom = 3.0;
            document::Setup& two = *p.findSetup(secondSide);
            side(two, "Second side", 0.0, -1.0);
            two.stock.kind = document::StockKind::FromSetup;
            two.stock.sourceSetup = firstSide;
        });
        flipLathe.save(out / "flip_project.ocam");
        std::printf("wrote %s\n", (out / "flip_project.ocam").string().c_str());
        writeLatheMachineFixtures(out, cancel, progress);

        // A turning project on the shaft: face, rough, finish, thread and part off on a lathe.
        document::Document lathe;
        const Uuid shaftModel = lathe.importModel(out / "shaft.step", cancel, progress);
        const Uuid latheSetup = lathe.addSetup(shaftModel);
        lathe.modify("Add turning operations", [&](document::Project& p) {
            using namespace shaft;
            p.name = "shaft";
            document::Setup& s = *p.findSetup(latheSetup);
            s.name = "Lathe";
            s.machine = "generic-2axis-lathe";
            s.originFrom.reset();                                   // the origin as set
            s.wcs.origin = Vec3(0, 0, kMajorLength + kMinorLength); // front face centre
            s.wcs.zAxis = Vec3::UnitZ();
            s.wcs.xAxis = Vec3::UnitX();
            s.stock.kind = document::StockKind::Box;
            s.stock.size = document::StockSize::OffsetFromPart;
            s.stock.offsetSide = 1.0;
            s.stock.offsetTop = 1.0;
            s.stock.offsetBottom = 5.0;
            tooling::Tool turn = tooling::makeDefaultTool(tooling::ToolType::TurningInsert, 1);
            turn.name = "CNMG 120404";
            turn.insert.noseRadius = 0.4;
            tooling::Tool thread = tooling::makeDefaultTool(tooling::ToolType::ThreadingInsert, 3);
            thread.name = "Threading 1.5";
            thread.insert.threadPitch = 1.5;
            tooling::Tool parting = tooling::makeDefaultTool(tooling::ToolType::PartingInsert, 5);
            parting.name = "Parting 3 mm";
            parting.insert.width = 3.0;
            parting.insert.noseRadius = 0.1;
            auto op = [&](const char* name, const char* type, const tooling::Tool& tool,
                          Json params, document::PresetOverrides overrides = {}) {
                document::Operation o;
                o.name = name;
                o.type = type;
                o.toolId = tool.id;
                o.presetId = tool.presets.front().id;
                o.parameters = std::move(params);
                o.presetOverrides = overrides;
                s.operations.push_back(o);
            };
            op("Face", "turn_face", turn, Json::object());
            op("Rough", "turn_rough", turn, Json::object(), stepDown(1.5));
            op("Finish", "turn_finish", turn, Json::object());
            op("Thread M20", "turn_thread", thread, Json{{"length", 20.0}});
            op("Part off", "turn_partoff", parting, Json::object());
            p.tools.tools = {turn, thread, parting};
        });
        lathe.save(out / "shaft_project.ocam");
        std::printf("wrote %s\n", (out / "shaft_project.ocam").string().c_str());

        // The same shaft in the 3-jaw chuck: a longer bar, 15 mm of it in the jaws, so the jaw
        // front is 10 mm behind the part's back end.
        lathe.modify("Hold it in the chuck", [&](document::Project& p) {
            p.name = "shaft in a chuck";
            p.setups.front().stock.offsetBottom = 25.0;
        });
        lathe.useWorkholding(chuckPackage,
                             document::importWorkholdingBodies(chuckPackage, cancel, progress),
                             latheSetup);
        lathe.save(out / "chuck_project.ocam");
        std::printf("wrote %s\n", (out / "chuck_project.ocam").string().c_str());

        // A 4th-axis project: wrapped pocketing of the grooves of the rotary shaft.
        document::Document rotaryDoc;
        const Uuid rotaryModel = rotaryDoc.importModel(out / "rotary_shaft.step", cancel, progress);
        const Uuid rotarySetup = rotaryDoc.addSetup(rotaryModel);
        // The bar on the A axis from the shaft's end: moved from centred on the part's box.
        const Vec3 rotaryShift =
            Vec3(rotary::kLength / 2, 0, 0) - rotaryDoc.modelBounds(rotaryModel)->center();
        rotaryDoc.modify("Add 4th-axis operations", [&](document::Project& p) {
            using namespace rotary;
            p.name = "rotary";
            document::Setup& s = *p.findSetup(rotarySetup);
            s.name = "4th axis";
            s.machine = "generic-4axis-vmc";
            s.originFrom.reset();      // the origin as set
            s.wcs = document::Frame{}; // on the A axis at the shaft's end
            s.stock.kind = document::StockKind::Cylinder;
            s.stock.size = document::StockSize::Fixed;
            s.stock.axis = 0;
            s.stock.shift = rotaryShift;
            s.stock.diameter = kDiameter;
            s.stock.length = kLength;
            s.clearanceHeight = kDiameter / 2 + 10;
            s.retractHeight = kDiameter / 2 + 3;
            const tooling::Tool mill = withPreset(tooling::makeFlatEndMill(1, 6.0, 15.0, 50.0));
            document::Operation o;
            o.name = "Wrapped grooves";
            o.type = "rotary_pocket";
            o.toolId = mill.id;
            o.presetId = mill.presets.front().id;
            o.presetOverrides = stepDown(1.5);
            s.operations.push_back(o);
            p.tools.tools = {mill};
        });
        rotaryDoc.save(out / "rotary_project.ocam");
        std::printf("wrote %s\n", (out / "rotary_project.ocam").string().c_str());

        // A 5-axis project: the dome centred on a trunnion table on a 20 mm riser, roughed in
        // 3 axes, finished with a ball end mill leaning 10 degrees in the feed direction, and
        // its base walls finished in 3 axes again (the table turns back to zero).
        document::Document five;
        const Uuid fiveModel = five.importModel(out / "dome.step", cancel, progress);
        const Uuid fiveSetup = five.addSetup(fiveModel);
        five.modify("Add 5-axis operations", [&](document::Project& p) {
            using namespace dome;
            p.name = "dome 5-axis";
            constexpr double kRiser = 20.0;
            p.models.front().placement.origin = Vec3(-kBase / 2, -kBase / 2, kRiser);
            document::Setup& s = *p.findSetup(fiveSetup);
            s.name = "Trunnion";
            s.machine = "generic-5axis-trunnion-ac";
            s.originFrom.reset(); // the origin as set
            s.wcs = document::Frame{};
            s.wcs.origin = Vec3(0, 0, kRiser); // the part's base, above the table centre
            s.stock.kind = document::StockKind::Box;
            s.stock.size = document::StockSize::OffsetFromPart;
            s.stock.offsetSide = 1.0;
            s.stock.offsetTop = 1.0;
            s.stock.offsetBottom = 0.0;
            s.clearanceHeight = kTop + 15;
            s.retractHeight = kTop + 5;
            tooling::Tool rougher = withPreset(tooling::makeFlatEndMill(1, 12.0, 40.0, 90.0));
            tooling::Tool ball = withPreset(tooling::makeBallEndMill(3, 6.0, 20.0, 60.0));
            auto op = [&](const char* name, const char* type, const tooling::Tool& tool,
                          Json params, document::PresetOverrides overrides = {}) {
                document::Operation o;
                o.name = name;
                o.type = type;
                o.toolId = tool.id;
                o.presetId = tool.presets.front().id;
                o.parameters = std::move(params);
                o.presetOverrides = overrides;
                s.operations.push_back(o);
            };
            op("Rough", "zlevel_rough", rougher, Json::object(), stepDown(3.0));
            // One-way passes: with a lead, reversing the feed would swing the table round.
            document::PresetOverrides finishing;
            finishing.radialDepth = 1.0; // the stepover
            op("5-axis finish", "multiaxis_finish", ball,
               Json{{"pattern", "one_way"},
                    {"axis_mode", "lead_tilt"},
                    {"lead_angle", 10.0},
                    {"max_tilt", 45.0}},
               finishing);
            op("Walls", "waterline3d", ball, Json::object(), stepDown(1.0));
            p.tools.tools = {rougher, ball};
        });
        five.save(out / "dome5_project.ocam");
        std::printf("wrote %s\n", (out / "dome5_project.ocam").string().c_str());

        // The same project on a trunnion machine built from STEP parts.
        const machine::Machine demo = trunnionMachine(out);
        five.modify("Use the demo trunnion", [&](document::Project& p) {
            p.name = "dome 5-axis on the demo trunnion";
            p.machines = {demo};
            p.findSetup(fiveSetup)->machine = demo.id;
        });
        five.save(out / "dome5_machine_project.ocam");
        std::printf("wrote %s\n", (out / "dome5_machine_project.ocam").string().c_str());

        // Swarf milling (DL-0085) on the trunnion: the tapered and the twisted boss roughed in
        // 3 axes, a rest roughing with the tool tilted 20 degrees about X (3+2), then each
        // boss's walls cut with the side of the tool, staying above the base plate's top.
        document::Document sw;
        const Uuid swModel = sw.importModel(out / "swarf.step", cancel, progress);
        const Uuid swSetup = sw.addSetup(swModel);
        {
            using namespace swarf;
            const cad::Model& part = *sw.model(swModel);
            const auto ref = [&](int face) {
                return document::ShapeRef{swModel, cad::SubShapeType::Face, face,
                                          cad::signatureOf(part, cad::SubShapeType::Face, face)};
            };
            std::vector<document::ShapeRef> cone, twist;
            std::optional<document::ShapeRef> floor;
            for (int f = 1; f <= static_cast<int>(part.info().faces); ++f) {
                const document::ShapeRef r = ref(f);
                if (const auto plane = cad::facePlane(part, f)) {
                    if (plane->normal.z() > 0.99 &&
                        std::abs(plane->origin.z() - kBaseHeight) < 1e-6) {
                        floor = r;
                    }
                    continue;
                }
                (r.signature.centroid.x() < kBaseX / 2 ? cone : twist).push_back(r);
            }
            if (cone.size() != 1 || twist.size() != 4 || !floor) {
                throw std::runtime_error("swarf: unexpected faces");
            }
            sw.modify("Add swarf operations", [&](document::Project& p) {
                p.name = "swarf";
                constexpr double kRiser = 20.0;
                p.models.front().placement.origin = Vec3(-kBaseX / 2, -kBaseY / 2, kRiser);
                document::Setup& s = *p.findSetup(swSetup);
                s.name = "Trunnion";
                s.machine = "generic-5axis-trunnion-ac";
                s.originFrom.reset(); // the origin as set
                s.wcs = document::Frame{};
                s.wcs.origin = Vec3(0, 0, kRiser); // the plate's bottom, above the table centre
                s.stock.kind = document::StockKind::Box;
                s.stock.size = document::StockSize::OffsetFromPart;
                s.stock.offsetSide = 1.0;
                s.stock.offsetTop = 1.0;
                s.stock.offsetBottom = 0.0;
                s.clearanceHeight = kTop + 15;
                s.retractHeight = kTop + 5;
                const tooling::Tool rougher =
                    withPreset(tooling::makeFlatEndMill(1, 10.0, 30.0, 80.0));
                const tooling::Tool side = withPreset(tooling::makeFlatEndMill(2, 6.0, 25.0, 60.0));
                auto op = [&](const char* name, const char* type, const tooling::Tool& tool,
                              std::vector<document::ShapeRef> geometry, Json params,
                              document::PresetOverrides overrides = {}) -> document::Operation& {
                    document::Operation o;
                    o.name = name;
                    o.type = type;
                    o.toolId = tool.id;
                    o.presetId = tool.presets.front().id;
                    o.geometry = std::move(geometry);
                    o.parameters = std::move(params);
                    o.presetOverrides = overrides;
                    s.operations.push_back(o);
                    return s.operations.back();
                };
                // No fine levels: the swarf takes the walls' stairs.
                op("Rough", "zlevel_rough", rougher, {}, Json{{"fine_step_down", 0.0}},
                   stepDown(3.0));
                document::Operation& tilted =
                    op("Tilted rest rough", "zlevel_rough", side, {},
                       Json{{"rest_machining", true}, {"fine_step_down", 0.0}},
                       stepDown(2.0));
                const double a = 20.0 * std::numbers::pi / 180.0;
                tilted.orientation =
                    document::ToolOrientation{document::Frame{Vec3::Zero(), Vec3::UnitX(),
                                                              Vec3(0.0, -std::sin(a), std::cos(a))},
                                              0};
                std::vector<document::ShapeRef> taper = cone;
                taper.push_back(*floor);
                op("Swarf taper", "swarf5x", side, taper,
                   Json{{"depth_passes", 2}, {"axis_offset", 0.1}});
                std::vector<document::ShapeRef> twisted = twist;
                twisted.push_back(*floor);
                op("Swarf twist", "swarf5x", side, twisted,
                   Json{{"axis_offset", 0.1}, {"tolerance", 0.02}});
                p.tools.tools = {rougher, side};
            });
        }
        sw.save(out / "swarf_project.ocam");
        std::printf("wrote %s\n", (out / "swarf_project.ocam").string().c_str());

        // One setup on the trunnion, operations on five faces of the hole plate standing on a
        // 60 mm riser, its holes facing the front (-Y): each face is machined with a tool
        // orientation of its own (3+2), the front hole drilled with a canned cycle, the back
        // face with a work offset of its own.
        document::Document faces;
        const Uuid facesModel = faces.importModel(out / "hole_plate.step", cancel, progress);
        const Uuid facesSetup = faces.addSetup(facesModel);
        faces.modify("Add operations on five faces", [&](document::Project& p) {
            using namespace hole_plate;
            p.name = "plate 3+2";
            constexpr double kRiser = 60.0;
            // Model Z (the holes' axis) along -Y, model Y up.
            p.models.front().placement = document::Frame{Vec3(-kSize / 2, kThickness / 2, kRiser),
                                                         Vec3::UnitX(), -Vec3::UnitY()};
            document::Setup& s = *p.findSetup(facesSetup);
            s.name = "Trunnion 3+2";
            s.machine = "generic-5axis-trunnion-ac";
            s.originFrom.reset(); // the origin as set
            s.wcs = document::Frame{};
            s.wcs.origin = Vec3(0, 0, kRiser + kSize); // the plate's top, above the table centre
            s.stock.kind = document::StockKind::Box;
            s.stock.size = document::StockSize::OffsetFromPart;
            s.stock.offsetSide = 1.0;
            s.stock.offsetTop = 1.0;
            s.stock.offsetBottom = 0.0;
            s.stock.roundTo = 0.0;
            const tooling::Tool mill = withPreset(tooling::makeFlatEndMill(1, 10.0, 22.0, 72.0));
            const tooling::Tool drill =
                withPreset(tooling::makeDrill(2, kDrillDiameter, kDrillTipAngle, 30.0, 70.0));
            auto op = [&](const char* name, const char* type, const tooling::Tool& tool,
                          std::optional<document::ToolOrientation> orientation) {
                document::Operation o;
                o.name = name;
                o.type = type;
                o.toolId = tool.id;
                o.presetId = tool.presets.front().id;
                o.orientation = orientation;
                s.operations.push_back(o);
            };
            // Frames in the WCS (origin at the plate's top centre): each face's centre.
            auto at = [](const Vec3& origin, const Vec3& z, const Vec3& x, int workOffset = 0) {
                return document::ToolOrientation{document::Frame{origin, x, z}, workOffset};
            };
            const double mid = -kSize / 2;
            const auto front = at({0, -kThickness / 2, mid}, -Vec3::UnitY(), Vec3::UnitX());
            op("Face top", "facing", mill, std::nullopt);
            op("Face front", "facing", mill, front);
            op("Drill front", "drill", drill, front);
            op("Face right", "facing", mill, at({kSize / 2, 0, mid}, Vec3::UnitX(), Vec3::UnitY()));
            op("Face back", "facing", mill,
               at({0, kThickness / 2, mid}, Vec3::UnitY(), -Vec3::UnitX(), 2));
            op("Face left", "facing", mill,
               at({-kSize / 2, 0, mid}, -Vec3::UnitX(), -Vec3::UnitY()));
            p.tools.tools = {mill, drill};
        });
        faces.save(out / "plate32_project.ocam");
        std::printf("wrote %s\n", (out / "plate32_project.ocam").string().c_str());

        // The same on the demo trunnion: its model parts move to each operation's index. Its
        // spindle head is 300 mm wide, so the plate stands on a 160 mm riser: tilted to the
        // front face, the head clears the 420 mm table (on the 60 mm one it hits it). The back
        // face is left out: at its index (A90 C0, DL-0019's choice) the head's column side reaches
        // over the table; A-90 C180 would clear it, but the index is not chosen per operation.
        faces.modify("Use the demo trunnion", [&](document::Project& p) {
            p.name = "plate 3+2 on the demo trunnion";
            p.machines = {demo};
            p.models.front().placement.origin.z() += 100.0;
            document::Setup& s = *p.findSetup(facesSetup);
            s.machine = demo.id;
            s.wcs.origin.z() += 100.0;
            for (document::Operation& o : s.operations) {
                o.enabled = o.name != "Face back";
            }
        });
        faces.save(out / "plate32_machine_project.ocam");
        std::printf("wrote %s\n", (out / "plate32_machine_project.ocam").string().c_str());
    } catch (const std::exception& e) {
        std::fprintf(stderr, "fixture generation failed: %s\n", e.what());
        return 2;
    }
    return 0;
}
