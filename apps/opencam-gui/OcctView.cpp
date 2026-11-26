// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "OcctView.hpp"

#include "SimPrs.hpp"
#include "ToolpathPrs.hpp"
#include "Units.hpp"

#include <ocam/cad/occt/ShapeAccess.hpp>
#include <ocam/foundation/Error.hpp>

#include <AIS_AnimationCamera.hxx>
#include <AIS_TextLabel.hxx>
#include <AIS_Trihedron.hxx>
#include <AIS_ViewCube.hxx>
#include <Aspect_DisplayConnection.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <GProp_GProps.hxx>
#include <Geom_Axis2Placement.hxx>
#include <Graphic3d_Camera.hxx>
#include <Graphic3d_TransformPers.hxx>
#include <NCollection_IndexedMap.hxx>
#include <OpenGl_GraphicDriver.hxx>
#include <Prs3d_Drawer.hxx>
#include <Prs3d_LineAspect.hxx>
#include <Prs3d_ShadingAspect.hxx>
#include <PrsMgr_PresentationManager.hxx>
#include <QCoreApplication>
#include <QGuiApplication>
#include <QMouseEvent>
#include <QWheelEvent>
#include <SelectMgr_Selection.hxx>
#include <SelectMgr_SensitiveEntity.hxx>
#include <StdSelect_BRepOwner.hxx>
#include <StdSelect_ViewerSelector3d.hxx>
#include <TopExp.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <V3d.hxx>
#include <V3d_AmbientLight.hxx>
#include <V3d_DirectionalLight.hxx>
#include <algorithm>
#include <functional>
#include <gp_Ax2.hxx>
#include <map>
#include <type_traits>

namespace ocam::gui {
namespace {

unsigned int toOcctButtons(Qt::MouseButtons buttons) {
    unsigned int result = Aspect_VKeyMouse_NONE;
    if (buttons & Qt::LeftButton) {
        result |= Aspect_VKeyMouse_LeftButton;
    }
    if (buttons & Qt::MiddleButton) {
        result |= Aspect_VKeyMouse_MiddleButton;
    }
    if (buttons & Qt::RightButton) {
        result |= Aspect_VKeyMouse_RightButton;
    }
    return result;
}

unsigned int toOcctModifiers(Qt::KeyboardModifiers modifiers) {
    unsigned int result = Aspect_VKeyFlags_NONE;
    if (modifiers & Qt::ShiftModifier) {
        result |= Aspect_VKeyFlags_SHIFT;
    }
    if (modifiers & Qt::ControlModifier) {
        result |= Aspect_VKeyFlags_CTRL;
    }
    if (modifiers & Qt::AltModifier) {
        result |= Aspect_VKeyFlags_ALT;
    }
    return result;
}

// Aspect_Drawable is a pointer (HWND, NSView*) on Windows and macOS but an integer (X11 Window)
// on Linux; WId is always an integer.
template <typename Drawable = Aspect_Drawable> Drawable toDrawable(WId id) {
    if constexpr (std::is_pointer_v<Drawable>) {
        return reinterpret_cast<Drawable>(id);
    } else {
        return static_cast<Drawable>(id);
    }
}

const char* surfaceTypeName(GeomAbs_SurfaceType type) {
    switch (type) {
    case GeomAbs_Plane:
        return QT_TRANSLATE_NOOP("ocam::gui::OcctView", "plane");
    case GeomAbs_Cylinder:
        return QT_TRANSLATE_NOOP("ocam::gui::OcctView", "cylinder");
    case GeomAbs_Cone:
        return QT_TRANSLATE_NOOP("ocam::gui::OcctView", "cone");
    case GeomAbs_Sphere:
        return QT_TRANSLATE_NOOP("ocam::gui::OcctView", "sphere");
    case GeomAbs_Torus:
        return QT_TRANSLATE_NOOP("ocam::gui::OcctView", "torus");
    case GeomAbs_BSplineSurface:
        return QT_TRANSLATE_NOOP("ocam::gui::OcctView", "B-spline surface");
    default:
        return QT_TRANSLATE_NOOP("ocam::gui::OcctView", "surface");
    }
}

QString describe(const TopoDS_Shape& shape, LengthUnit unit) {
    if (shape.ShapeType() == TopAbs_FACE) {
        const TopoDS_Face& face = TopoDS::Face(shape);
        BRepAdaptor_Surface surf(face, true);
        GProp_GProps props;
        BRepGProp::SurfaceProperties(face, props);
        QString text = QCoreApplication::translate("ocam::gui::OcctView", "Face: %1, area %2")
                           .arg(QCoreApplication::translate("ocam::gui::OcctView",
                                                            surfaceTypeName(surf.GetType())),
                                formatAreaBoth(props.Mass(), unit));
        if (surf.GetType() == GeomAbs_Cylinder) {
            text += QStringLiteral(", Ø %1").arg(
                formatLengthBoth(2.0 * surf.Cylinder().Radius(), unit));
        }
        return text;
    }
    if (shape.ShapeType() == TopAbs_EDGE) {
        GProp_GProps props;
        BRepGProp::LinearProperties(shape, props);
        const BRepAdaptor_Curve curve(TopoDS::Edge(shape));
        if (curve.GetType() == GeomAbs_Circle) {
            const double d = 2.0 * curve.Circle().Radius();
            const bool full =
                std::abs(curve.LastParameter() - curve.FirstParameter() - 2.0 * kPi) < 1e-6;
            return full ? QCoreApplication::translate("ocam::gui::OcctView", "Edge: circle, Ø %1")
                              .arg(formatLengthBoth(d, unit))
                        : QCoreApplication::translate("ocam::gui::OcctView",
                                                      "Edge: arc, Ø %1, R %2, length %3")
                              .arg(formatLengthBoth(d, unit), formatLengthBoth(d / 2.0, unit),
                                   formatLengthBoth(props.Mass(), unit));
        }
        return QCoreApplication::translate("ocam::gui::OcctView", "Edge: length %1")
            .arg(formatLengthBoth(props.Mass(), unit));
    }
    return QCoreApplication::translate("ocam::gui::OcctView", "Body selected");
}

/// A view cube whose clicks turn the view through a callback (in the setup's frame) instead of
/// the cube's own animation in world axes.
class SetupViewCube : public AIS_ViewCube {
    DEFINE_STANDARD_RTTI_INLINE(SetupViewCube, AIS_ViewCube)

public:
    std::function<void(V3d_TypeOfOrientation)> onClick;

    void HandleClick(const occ::handle<AIS_ViewCubeOwner>& owner) override {
        if (onClick && owner) {
            onClick(owner->MainOrientation());
        }
    }
};

} // namespace

OcctView::OcctView(QWidget* parent) : QWidget(parent) {
    setAttribute(Qt::WA_PaintOnScreen);
    setAttribute(Qt::WA_NoSystemBackground);
    setAttribute(Qt::WA_NativeWindow);
    setMouseTracking(true);
    setFocusPolicy(Qt::StrongFocus);
    setMinimumSize(320, 240);

    ChangeMouseGestureMap().Bind(Aspect_VKeyMouse_MiddleButton, AIS_MouseGesture_Pan);
}

OcctView::~OcctView() {
    if (view_) {
        view_->Remove();
    }
}

void OcctView::ensureViewer() {
    if (view_) {
        return;
    }
    occ::handle<Aspect_DisplayConnection> display = new Aspect_DisplayConnection();
    occ::handle<OpenGl_GraphicDriver> driver = new OpenGl_GraphicDriver(display, false);

    viewer_ = new V3d_Viewer(driver);
    // Ambient light plus a key light from the viewer's upper left: faces that point different
    // ways shade differently in every view (a light along the view axis lights the top, front
    // and side of a box alike in an isometric view, which flattens the simulated stock).
    occ::handle<V3d_AmbientLight> ambient = new V3d_AmbientLight(Quantity_NOC_WHITE);
    ambient->SetIntensity(0.45f);
    occ::handle<V3d_DirectionalLight> key =
        new V3d_DirectionalLight(gp_Dir(0.45, -0.6, -1.0), Quantity_NOC_WHITE, true);
    key->SetIntensity(0.75f);
    viewer_->AddLight(ambient);
    viewer_->AddLight(key);
    viewer_->SetLightOn();

    context_ = new AIS_InteractiveContext(viewer_);
    context_->SetAutomaticHilight(true);
    // Edges and vertices are thin: pick them within a few pixels of the cursor (default 2).
    context_->SetPixelTolerance(5);
    // Selected items in strong orange, items under the cursor in light blue, with thick edges so
    // both stand out against the grey part.
    for (const auto type : {Prs3d_TypeOfHighlight_Selected, Prs3d_TypeOfHighlight_LocalSelected,
                            Prs3d_TypeOfHighlight_Dynamic, Prs3d_TypeOfHighlight_LocalDynamic}) {
        const bool selected =
            type == Prs3d_TypeOfHighlight_Selected || type == Prs3d_TypeOfHighlight_LocalSelected;
        const occ::handle<Prs3d_Drawer>& style = context_->HighlightStyle(type);
        style->SetMethod(Aspect_TOHM_COLOR);
        style->SetDisplayMode(AIS_Shaded);
        style->SetTransparency(selected ? 0.0f : 0.25f);
        style->SetZLayer(Graphic3d_ZLayerId_Top);
    }

    window_ = new Aspect_NeutralWindow();
    window_->SetNativeHandle(toDrawable(winId()));
    drawnInto_ = static_cast<std::uintptr_t>(winId());
    updateWindowSize();

    view_ = viewer_->CreateView();
    // Multisampling smooths the edges of faces and lines.
    view_->ChangeRenderingParams().NbMsaaSamples = 4;
    view_->ChangeRenderingParams().IsAntialiasingEnabled = true;
    view_->SetWindow(window_);
    applyViewColors(); // the background and the highlight colours
    view_->SetProj(V3d_XposYnegZpos);

    occ::handle<SetupViewCube> cube = new SetupViewCube();
    cube->onClick = [this](V3d_TypeOfOrientation o) {
        // The cube stands as the machine does: its sides are the world's.
        const gp_Dir proj = V3d::GetProjAxis(o);
        snapWorld(Vec3(proj.X(), proj.Y(), proj.Z()), true);
    };
    cube->SetSize(56.0);
    cube->SetBoxFacetExtension(6.0);
    cube->SetAxesPadding(4.0);
    cube->SetFontHeight(11.0);
    cube->SetBoxColor(Quantity_Color(0.62, 0.66, 0.72, Quantity_TOC_sRGB));
    cube->SetTextColor(Quantity_Color(0.10, 0.10, 0.12, Quantity_TOC_sRGB));
    cube->SetTransparency(0.15);
    cube->SetTransformPersistence(new Graphic3d_TransformPers(
        Graphic3d_TMF_TriedronPers, Aspect_TOTP_LEFT_LOWER, NCollection_Vec2<int>(90, 90)));
    context_->Display(cube, 0, 0, false);
    viewCube_ = cube;
}

void OcctView::updateWindowSize() {
    if (window_) {
        const qreal ratio = devicePixelRatioF();
        window_->SetSize(qRound(width() * ratio), qRound(height() * ratio));
    }
}

NCollection_Vec2<int> OcctView::devicePoint(const QPointF& p) const {
    const qreal ratio = devicePixelRatioF();
    return NCollection_Vec2<int>(qRound(p.x() * ratio), qRound(p.y() * ratio));
}

namespace {

/// Selected machine parts in the machine library's preview.
constexpr std::uint32_t kHighlightColour = 0x2EC4FF;

gp_Trsf toTrsf(const Transform3& t) {
    gp_Trsf trsf;
    const auto& m = t.matrix();
    trsf.SetValues(m(0, 0), m(0, 1), m(0, 2), m(0, 3), m(1, 0), m(1, 1), m(1, 2), m(1, 3), m(2, 0),
                   m(2, 1), m(2, 2), m(2, 3));
    return trsf;
}

} // namespace

void OcctView::setModels(const std::vector<ViewModel>& models) {
    ensureViewer();
    const bool wasEmpty = shown_.empty();

    // Remove models that are gone, whose geometry changed or that now take picks (or no longer
    // do: the selection mode is set when displayed).
    for (auto it = shown_.begin(); it != shown_.end();) {
        const auto found = std::find_if(models.begin(), models.end(), [&](const ViewModel& m) {
            return m.id == it->first && &m.model.impl() == &it->second.model.impl() &&
                   m.selectable == it->second.selectable;
        });
        if (found == models.end()) {
            context_->Remove(it->second.ais, false);
            it = shown_.erase(it);
        } else {
            ++it;
        }
    }
    for (const auto& m : models) {
        auto it = shown_.find(m.id);
        if (it == shown_.end()) {
            occ::handle<AIS_Shape> ais = new AIS_Shape(cad::occtShape(m.model));
            // A matte material: the default one washes faces facing the viewer out to white.
            ais->SetMaterial(Graphic3d_MaterialAspect(Graphic3d_NameOfMaterial_Satin));
            // Parts in the colour the other models show in, fixtures in theirs, models with a
            // colour of their own in it.
            ais->SetColor(m.colour ? toQuantity(QColor::fromRgb(*m.colour & 0xFFFFFFu))
                                   : toQuantity(m.fixture ? colors_.fixture : modelColor_));
            ais->Attributes()->SetFaceBoundaryDraw(true);
            // The object's own display mode: re-displaying it after the simulation hid it would
            // otherwise fall back to the context default, wireframe.
            ais->SetDisplayMode(AIS_Shaded);
            if (m.selectable) {
                // Display with the selection mode: in OCCT 8 an object displayed without one is
                // not loaded into the selection manager, and activating a mode later does nothing.
                context_->Display(ais, AIS_Shaded, AIS_Shape::SelectionMode(TopAbs_FACE), false);
                activateSelection(ais); // edges too
            } else {
                context_->Display(ais, AIS_Shaded, -1, false);
            }
            context_->SetLocation(ais, TopLoc_Location(toTrsf(viewFromProject_ * m.placement)));
            if (m.fixture ? fixturesHidden_ : modelsHidden_) {
                context_->Erase(ais, false);
            }
            shown_.emplace(m.id,
                           Shown{m.model, m.placement, ais, m.fixture, m.selectable, m.colour});
        } else {
            if (!it->second.placement.isApprox(m.placement)) {
                it->second.placement = m.placement;
                context_->SetLocation(it->second.ais,
                                      TopLoc_Location(toTrsf(viewFromProject_ * m.placement)));
            }
            if (it->second.fixture != m.fixture || it->second.colour != m.colour) {
                it->second.fixture = m.fixture;
                it->second.colour = m.colour;
                context_->SetColor(it->second.ais,
                                   m.colour ? toQuantity(QColor::fromRgb(*m.colour & 0xFFFFFFu))
                                            : toQuantity(m.fixture ? colors_.fixture : modelColor_),
                                   false);
            }
        }
    }
    if (wasEmpty && !shown_.empty()) {
        view_->FitAll(0.05, false);
    }
    requestRedraw();
}

void OcctView::setStock(const std::optional<document::StockShape>& stock) {
    ensureViewer();
    if (stock_) {
        context_->Remove(stock_, false);
        stock_.Nullify();
    }
    if (stock) {
        TopoDS_Shape shape;
        if (stock->kind == document::StockShape::Kind::Box) {
            const Vec3 size = stock->box.size();
            if ((size.array() > 1e-9).all()) {
                shape = BRepPrimAPI_MakeBox(
                            gp_Pnt(stock->box.min.x(), stock->box.min.y(), stock->box.min.z()),
                            size.x(), size.y(), size.z())
                            .Shape();
            }
        } else if (stock->diameter > 0.0 && stock->length > 0.0) {
            const gp_Ax2 axis(gp_Pnt(stock->base.x(), stock->base.y(), stock->base.z()),
                              gp_Dir(stock->axis.x(), stock->axis.y(), stock->axis.z()));
            shape = BRepPrimAPI_MakeCylinder(axis, stock->diameter / 2.0, stock->length).Shape();
            if (stock->innerDiameter > 0.0) {
                // A tube: the bore cut through, a little past both ends.
                gp_Ax2 through = axis;
                through.Translate(gp_Vec(axis.Direction()) * -1.0);
                shape = BRepAlgoAPI_Cut(shape, BRepPrimAPI_MakeCylinder(through,
                                                                        stock->innerDiameter / 2.0,
                                                                        stock->length + 2.0)
                                                   .Shape())
                            .Shape();
            }
        }
        if (!shape.IsNull()) {
            occ::handle<AIS_Shape> ais = new AIS_Shape(shape);
            if (stock->kind == document::StockShape::Kind::Box) {
                ais->SetLocalTransformation(toTrsf(stock->boxToProject())); // box coordinates
            }
            ais->SetColor(toQuantity(colors_.stockBox));
            ais->SetTransparency(0.75);
            ais->SetDisplayMode(AIS_Shaded);
            context_->Display(ais, AIS_Shaded, -1, false);
            context_->Deactivate(ais); // not selectable
            if (simActive_) {
                context_->Erase(ais, false);
            }
            stock_ = ais;
        }
    }
    requestRedraw();
}

void OcctView::setWcs(const std::optional<document::Frame>& wcs) {
    ensureViewer();
    if (wcs_) {
        context_->Remove(wcs_, false);
        wcs_.Nullify();
    }
    if (wcs) {
        try {
            const document::Frame f = wcs->normalized();
            occ::handle<Geom_Axis2Placement> placement =
                new Geom_Axis2Placement(gp_Ax2(gp_Pnt(f.origin.x(), f.origin.y(), f.origin.z()),
                                               gp_Dir(f.zAxis.x(), f.zAxis.y(), f.zAxis.z()),
                                               gp_Dir(f.xAxis.x(), f.xAxis.y(), f.xAxis.z())));
            occ::handle<AIS_Trihedron> triad = new AIS_Trihedron(placement);
            triad->SetSize(20.0);
            // Over everything (a depth of its own): the origin and axes show from any
            // side, inside the stock, the part or the vise.
            triad->SetZLayer(Graphic3d_ZLayerId_Topmost);
            context_->Display(triad, false);
            context_->Deactivate(triad);
            context_->SetLocation(triad, TopLoc_Location(toTrsf(viewFromProject_)));
            wcs_ = triad;
        } catch (const Error&) {
            // invalid frame: nothing to show
        }
    }
    requestRedraw();
}

void OcctView::setToolpaths(const std::vector<ToolpathView>& paths) {
    ensureViewer();
    for (auto it = toolpaths_.begin(); it != toolpaths_.end();) {
        const auto found = std::find_if(paths.begin(), paths.end(), [&](const ToolpathView& v) {
            return v.operationId == it->first && v.path == it->second.path &&
                   v.emphasized == it->second.emphasized;
        });
        if (found == paths.end()) {
            context_->Remove(it->second.prs, false);
            it = toolpaths_.erase(it);
        } else {
            ++it;
        }
    }
    for (const auto& v : paths) {
        if (!v.path || toolpaths_.contains(v.operationId)) {
            continue;
        }
        occ::handle<ToolpathPrs> prs =
            new ToolpathPrs(*v.path, v.wcsToProject, v.emphasized, colors_);
        context_->Display(prs, 0, -1, false);
        context_->Deactivate(prs);
        context_->SetLocation(prs, TopLoc_Location(toTrsf(viewFromProject_)));
        if (toolpathsHidden_) {
            context_->Erase(prs, false);
        }
        toolpaths_.emplace(v.operationId, ShownToolpath{v.path, v.emphasized, v.wcsToProject, prs});
    }
    requestRedraw();
}

void OcctView::setSimulation(const std::optional<SimulationScene>& scene) {
    ensureViewer();
    auto remove = [&](occ::handle<AIS_InteractiveObject>& prs) {
        if (prs) {
            context_->Remove(prs, false);
            prs.Nullify();
        }
    };
    auto removeStock = [&] {
        for (auto& prs : simStockPrs_) {
            remove(prs);
        }
        simStockPrs_.clear();
        simMeshes_.clear();
    };
    const bool movesChanged =
        scene ? scene->modelMoves.size() != modelMoves_.size() || !scene->modelMoves.empty()
              : !modelMoves_.empty();
    if (movesChanged) {
        modelMoves_ = scene ? scene->modelMoves : std::map<Uuid, Transform3>{};
    }
    setViewFromProject(scene ? scene->viewFromProject : Transform3::Identity());
    if (movesChanged) {
        placeModels();
    }
    if (!scene) {
        removeStock();
        remove(simToolPrs_);
        remove(simMarkersPrs_);
        simTool_.reset();
        simInsert_.reset();
        simCollisions_.clear();
        if (simActive_ && stock_) {
            context_->Display(stock_, false);
            context_->Deactivate(stock_);
        }
        simActive_ = false;
    } else {
        if (!simActive_ && stock_) {
            context_->Erase(stock_, false);
        }
        simActive_ = true;
        const bool moved = !simXf_.isApprox(scene->wcsToProject);
        // Only the chunks whose meshes changed are rebuilt.
        if (moved || scene->stock.size() != simMeshes_.size()) {
            removeStock();
            simMeshes_.resize(scene->stock.size());
            simStockPrs_.resize(scene->stock.size());
        }
        for (std::size_t c = 0; c < scene->stock.size(); ++c) {
            if (scene->stock[c] == simMeshes_[c]) {
                continue;
            }
            remove(simStockPrs_[c]);
            simMeshes_[c] = scene->stock[c];
            if (simMeshes_[c] && !simMeshes_[c]->indices.empty()) {
                simStockPrs_[c] =
                    new StockMeshPrs(simMeshes_[c], scene->wcsToProject, stockTransparency_);
                context_->Display(simStockPrs_[c], 0, -1, false);
                context_->Deactivate(simStockPrs_[c]);
                context_->SetLocation(simStockPrs_[c], TopLoc_Location(toTrsf(viewFromProject_)));
            }
        }
        if (scene->tool != simTool_ || scene->insert != simInsert_) {
            remove(simToolPrs_);
            simTool_ = scene->tool;
            simInsert_ = scene->tool ? nullptr : scene->insert;
            if (simTool_) {
                simToolPrs_ = new ToolBodyPrs(simTool_);
            } else if (simInsert_) {
                simToolPrs_ = new InsertPrs(simInsert_, scene->insertThickness, scene->toolParts);
            }
            if (simToolPrs_) {
                context_->Display(simToolPrs_, 0, -1, false);
                context_->Deactivate(simToolPrs_);
            }
        }
        if (simToolPrs_) {
            Transform3 at = scene->wcsToProject;
            at.translate(scene->toolTip);
            at.rotate(Eigen::Quaterniond::FromTwoVectors(Vec3::UnitZ(), scene->toolAxis));
            simToolAt_ = at;
            context_->SetLocation(simToolPrs_, TopLoc_Location(toTrsf(viewFromProject_ * at)));
        }
        if (scene->collisions != simCollisions_ || moved) {
            remove(simMarkersPrs_);
            simCollisions_ = scene->collisions;
            if (!simCollisions_.empty()) {
                std::vector<Vec3> points;
                for (const Vec3& p : simCollisions_) {
                    points.push_back(scene->wcsToProject * p);
                }
                simMarkersPrs_ = new MarkersPrs(std::move(points));
                context_->Display(simMarkersPrs_, 0, -1, false);
                context_->Deactivate(simMarkersPrs_);
                context_->SetLocation(simMarkersPrs_, TopLoc_Location(toTrsf(viewFromProject_)));
            }
        }
        simXf_ = scene->wcsToProject;
    }
    const bool hidePaths = scene && !scene->showToolpaths;
    if (hidePaths != toolpathsHidden_) {
        toolpathsHidden_ = hidePaths;
        for (auto& [id, shown] : toolpaths_) {
            if (hidePaths) {
                context_->Erase(shown.prs, false);
            } else {
                context_->Display(shown.prs, false);
                context_->Deactivate(shown.prs);
            }
        }
    }
    sceneShowsModels_ = !scene || scene->showModels;
    sceneShowsFixtures_ = !scene || scene->showFixtures;
    applyModelVisibility();
    setMachine(scene ? scene->machine : std::nullopt);
    requestRedraw();
}

void OcctView::lookFrom(const Vec3& eye, const Vec3& up) {
    ensureViewer();
    view_->SetProj(eye.x(), eye.y(), eye.z());
    view_->SetUp(up.x(), up.y(), up.z());
}

void OcctView::placeModels() {
    for (auto& [id, shown] : shown_) {
        if (shown.ais) {
            const auto move = modelMoves_.find(id);
            const Transform3 at = viewFromProject_ * shown.placement;
            context_->SetLocation(
                shown.ais,
                TopLoc_Location(toTrsf(move == modelMoves_.end() ? at : move->second * at)));
        }
    }
}

void OcctView::setViewFromProject(const Transform3& xf) {
    if (xf.isApprox(viewFromProject_, 1e-12)) {
        return;
    }
    viewFromProject_ = xf;
    auto place = [&](const occ::handle<AIS_InteractiveObject>& prs, const Transform3& at) {
        if (prs) {
            context_->SetLocation(prs, TopLoc_Location(toTrsf(at)));
        }
    };
    placeModels();
    for (auto& [id, shown] : toolpaths_) {
        place(shown.prs, xf);
    }
    for (const auto& prs : simStockPrs_) {
        place(prs, xf);
    }
    place(simToolPrs_, xf * simToolAt_);
    place(simMarkersPrs_, xf);
    place(wcs_, xf);
}

void OcctView::setMachine(const std::optional<MachineScene>& scene) {
    ensureViewer();
    // Presentations are kept for parts whose mesh and colour stay the same (B-rep ones also
    // when only their colour changes).
    std::vector<ShownPart> next;
    if (scene && scene->parts) {
        for (std::size_t i = 0; i < scene->parts->size(); ++i) {
            const machine::ModelPart& part = (*scene->parts)[i];
            const bool highlight = i < scene->highlighted.size() && scene->highlighted[i];
            const cad::Model solid = i < scene->solids.size() ? scene->solids[i] : cad::Model();
            ShownPart shown{part.geometry,
                            solid,
                            highlight ? kHighlightColour : part.colour,
                            scene->transparency,
                            {}};
            if (i < scene->hidden.size() && scene->hidden[i]) {
                next.push_back(std::move(shown)); // not drawn
                continue;
            }
            if (!solid.isNull()) {
                const auto old =
                    std::find_if(machinePrs_.begin(), machinePrs_.end(), [&](const ShownPart& s) {
                        return s.prs && !s.solid.isNull() && &s.solid.impl() == &solid.impl();
                    });
                occ::handle<AIS_Shape> ais;
                if (old != machinePrs_.end()) {
                    ais = occ::down_cast<AIS_Shape>(old->prs);
                    old->prs.Nullify();
                }
                if (!ais) {
                    ais = new AIS_Shape(cad::occtShape(solid));
                    ais->SetMaterial(Graphic3d_MaterialAspect(Graphic3d_NameOfMaterial_Satin));
                    ais->Attributes()->SetFaceBoundaryDraw(true);
                    ais->SetDisplayMode(AIS_Shaded);
                    context_->Display(ais, AIS_Shaded,
                                      scene->pickable ? AIS_Shape::SelectionMode(TopAbs_FACE) : -1,
                                      false);
                }
                const std::uint32_t c = shown.colour;
                context_->SetColor(ais,
                                   Quantity_Color(((c >> 16) & 0xFF) / 255.0,
                                                  ((c >> 8) & 0xFF) / 255.0, (c & 0xFF) / 255.0,
                                                  Quantity_TOC_sRGB),
                                   false);
                context_->SetTransparency(ais, shown.transparency, false);
                if (scene->pickable) {
                    // Faces, edges and vertices at once: a click picks whatever is under it.
                    for (const TopAbs_ShapeEnum kind : {TopAbs_FACE, TopAbs_EDGE, TopAbs_VERTEX}) {
                        context_->SetSelectionModeActive(ais, AIS_Shape::SelectionMode(kind), true,
                                                         AIS_SelectionModesConcurrency_Multiple);
                    }
                } else {
                    context_->Deactivate(ais);
                }
                shown.prs = ais;
                next.push_back(std::move(shown));
                continue;
            }
            const auto old =
                std::find_if(machinePrs_.begin(), machinePrs_.end(), [&](const ShownPart& s) {
                    return s.prs && s.solid.isNull() && s.mesh == shown.mesh &&
                           s.colour == shown.colour && s.transparency == shown.transparency;
                });
            if (old != machinePrs_.end()) {
                shown.prs = old->prs;
                old->prs.Nullify();
            } else if (part.geometry && !part.geometry->isEmpty()) {
                shown.prs = new MachinePartPrs(part.geometry, shown.colour, scene->transparency);
                // Displayed with its selection mode when pickable: OCCT 8 loads an object into
                // the selection manager only then.
                context_->Display(shown.prs, 0, scene->pickable ? 0 : -1, false);
            }
            if (const auto prs = occ::down_cast<MachinePartPrs>(shown.prs)) {
                prs->setIndex(i);
                if (scene->pickable) {
                    context_->Activate(prs, 0);
                } else {
                    context_->Deactivate(prs);
                }
            }
            next.push_back(std::move(shown));
        }
    }
    for (auto& s : machinePrs_) {
        if (s.prs) {
            context_->Remove(s.prs, false);
        }
    }
    machinePrs_ = std::move(next);
    if (scene) {
        for (std::size_t i = 0; i < machinePrs_.size() && i < scene->placements.size(); ++i) {
            if (machinePrs_[i].prs) {
                context_->SetLocation(machinePrs_[i].prs,
                                      TopLoc_Location(toTrsf(scene->placements[i])));
            }
        }
    }
    for (auto& prs : machineLines_) {
        context_->Remove(prs, false);
    }
    machineLines_.clear();
    if (scene) {
        // One presentation per colour and weight.
        std::map<std::pair<std::uint32_t, bool>, std::vector<std::pair<Vec3, Vec3>>> groups;
        for (const auto& line : scene->lines) {
            const std::uint32_t colour = line.colour        ? line.colour
                                         : line.highlighted ? kHighlightColour
                                                            : 0xF2C230u;
            groups[{colour, line.highlighted}].emplace_back(line.from, line.to);
        }
        for (auto& [key, segments] : groups) {
            occ::handle<AIS_InteractiveObject> prs =
                new LinesPrs(std::move(segments), key.first, key.second ? 3.0 : 1.5);
            context_->Display(prs, 0, -1, false);
            context_->Deactivate(prs);
            machineLines_.push_back(prs);
        }
        for (const auto& text : scene->labels) {
            occ::handle<AIS_TextLabel> label = new AIS_TextLabel();
            label->SetPosition(gp_Pnt(text.at.x(), text.at.y(), text.at.z()));
            label->SetHJustification(Graphic3d_HTA_LEFT);
            label->SetVJustification(Graphic3d_VTA_BOTTOM);
            label->SetHeight(13.0);
            label->SetColor(Quantity_Color(((text.colour >> 16) & 0xFF) / 255.0,
                                           ((text.colour >> 8) & 0xFF) / 255.0,
                                           (text.colour & 0xFF) / 255.0, Quantity_TOC_sRGB));
            label->SetDisplayType(Aspect_TODT_SHADOW);
            label->SetColorSubTitle(Quantity_Color(0.0, 0.0, 0.0, Quantity_TOC_sRGB));
            label->SetZLayer(Graphic3d_ZLayerId_Topmost);
            label->SetText(TCollection_ExtendedString(text.text.c_str(), true));
            context_->Display(label, 0, -1, false);
            context_->Deactivate(label);
            machineLines_.push_back(label);
        }
    }
    for (auto& prs : machineFrames_) {
        context_->Remove(prs, false);
    }
    machineFrames_.clear();
    if (scene) {
        for (const Transform3& f : scene->frames) {
            const Vec3 o = f.translation();
            const Vec3 z = f.linear().col(2);
            const Vec3 x = f.linear().col(0);
            occ::handle<AIS_Trihedron> triad = new AIS_Trihedron(new Geom_Axis2Placement(
                gp_Ax2(gp_Pnt(o.x(), o.y(), o.z()), gp_Dir(z.x(), z.y(), z.z()),
                       gp_Dir(x.x(), x.y(), x.z()))));
            triad->SetSize(60.0);
            context_->Display(triad, false);
            context_->Deactivate(triad);
            machineFrames_.push_back(triad);
        }
    }
    requestRedraw();
}

void OcctView::applyModelVisibility() {
    applyModelColor(); // see-through stock or not
    const bool hideModels = simActive_ && !sceneShowsModels_ && stockTransparency_ <= 0.0;
    const bool hideFixtures = simActive_ && !sceneShowsFixtures_;
    if (hideModels == modelsHidden_ && hideFixtures == fixturesHidden_) {
        return;
    }
    modelsHidden_ = hideModels;
    fixturesHidden_ = hideFixtures;
    for (auto& [id, shown] : shown_) {
        if (shown.fixture ? hideFixtures : hideModels) {
            context_->Erase(shown.ais, false);
        } else if (!context_->IsDisplayed(shown.ais)) {
            if (shown.selectable) {
                context_->Display(shown.ais, false);
                activateSelection(shown.ais);
            } else {
                // No selection mode: workholding bodies are not project models to pick from.
                context_->Display(shown.ais, AIS_Shaded, -1, false);
            }
        }
    }
}

void OcctView::moveSimulationTool(const Vec3& tip, const Vec3& axis) {
    if (!context_ || !simToolPrs_) {
        return;
    }
    Transform3 at = simXf_;
    at.translate(tip);
    at.rotate(Eigen::Quaterniond::FromTwoVectors(Vec3::UnitZ(), axis));
    simToolAt_ = at;
    context_->SetLocation(simToolPrs_, TopLoc_Location(toTrsf(viewFromProject_ * at)));
    requestRedraw();
}

void OcctView::selectShapes(const std::vector<PickedShape>& shapes) {
    if (!context_) {
        return;
    }
    context_->ClearSelected(false);
    for (const PickedShape& wanted : shapes) {
        const auto it = shown_.find(wanted.model);
        if (it == shown_.end()) {
            continue;
        }
        const TopAbs_ShapeEnum kind = wanted.type == cad::SubShapeType::Face   ? TopAbs_FACE
                                      : wanted.type == cad::SubShapeType::Edge ? TopAbs_EDGE
                                                                               : TopAbs_VERTEX;
        NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> subShapes;
        TopExp::MapShapes(cad::occtShape(it->second.model), kind, subShapes);
        if (wanted.index < 1 || wanted.index > subShapes.Extent()) {
            continue;
        }
        const TopoDS_Shape& target = subShapes(wanted.index);
        // The owner of that sub-shape in the object's face or edge selection.
        const occ::handle<SelectMgr_Selection>& selection =
            it->second.ais->Selection(AIS_Shape::SelectionMode(kind));
        if (selection.IsNull()) {
            continue;
        }
        for (const auto& entity : selection->Entities()) {
            const auto owner =
                occ::down_cast<StdSelect_BRepOwner>(entity->BaseSensitive()->OwnerId());
            if (owner && owner->Shape().TShape() == target.TShape()) {
                context_->AddOrRemoveSelected(owner, false);
                break;
            }
        }
    }
    requestRedraw();
}

void OcctView::setOutlineHover(OutlineFn outline) {
    outlineHover_ = std::move(outline);
    clearOutlineHover();
}

occ::handle<SelectMgr_EntityOwner> OcctView::edgeOwner(const Uuid& model, int index) const {
    const auto it = shown_.find(model);
    if (it == shown_.end()) {
        return {};
    }
    NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> edges;
    TopExp::MapShapes(cad::occtShape(it->second.model), TopAbs_EDGE, edges);
    if (index < 1 || index > edges.Extent()) {
        return {};
    }
    const occ::handle<SelectMgr_Selection>& selection =
        it->second.ais->Selection(AIS_Shape::SelectionMode(TopAbs_EDGE));
    if (selection.IsNull()) {
        return {};
    }
    for (const auto& entity : selection->Entities()) {
        const auto owner = occ::down_cast<StdSelect_BRepOwner>(entity->BaseSensitive()->OwnerId());
        if (owner && owner->Shape().TShape() == edges(index).TShape()) {
            return occ::handle<SelectMgr_EntityOwner>(owner);
        }
    }
    return {};
}

void OcctView::showHoverHighlight(const std::vector<occ::handle<SelectMgr_EntityOwner>>& extra) {
    // Mouse highlighting lives in the immediate layer, which hover redraws refresh; restarting it
    // drops the context's own highlight, so the detected owner is drawn again with the extra ones.
    const occ::handle<PrsMgr_PresentationManager>& pm = context_->MainPrsMgr();
    const occ::handle<Prs3d_Drawer>& style =
        context_->HighlightStyle(Prs3d_TypeOfHighlight_LocalDynamic);
    pm->BeginImmediateDraw();
    if (context_->HasDetected() && !context_->DetectedOwner()->IsSelected()) {
        context_->DetectedOwner()->HilightWithColor(pm, style);
    }
    for (const auto& owner : extra) {
        owner->HilightWithColor(pm, style);
    }
    pm->EndImmediateDraw(viewer_);
}

void OcctView::clearOutlineHover() {
    const bool had = !hoverOwners_.empty();
    hoverOwners_.clear();
    hoverKey_ = {Uuid(), 0};
    if (had && context_) {
        showHoverHighlight({});
    }
}

void OcctView::handleMoveTo(const occ::handle<AIS_InteractiveContext>& context,
                            const occ::handle<V3d_View>& view) {
    AIS_ViewController::handleMoveTo(context, view);
    updateOutlineHover();
}

void OcctView::updateOutlineHover() {
    if (!context_ || !outlineHover_) {
        return;
    }
    // The edge under the cursor, unless Shift asks for single edges.
    std::pair<Uuid, int> key{Uuid(), 0};
    const bool shift = QGuiApplication::queryKeyboardModifiers().testFlag(Qt::ShiftModifier);
    if (!shift && context_->HasDetected()) {
        const auto owner = occ::down_cast<StdSelect_BRepOwner>(context_->DetectedOwner());
        if (owner && owner->Shape().ShapeType() == TopAbs_EDGE) {
            for (const auto& [id, shown] : shown_) {
                if (owner->Selectable() != shown.ais) {
                    continue;
                }
                NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> edges;
                TopExp::MapShapes(cad::occtShape(shown.model), TopAbs_EDGE, edges);
                for (int i = 1; i <= edges.Extent(); ++i) {
                    if (edges(i).TShape() == owner->Shape().TShape()) {
                        key = {id, i};
                        break;
                    }
                }
            }
        }
    }
    if (key == hoverKey_) {
        return;
    }
    clearOutlineHover();
    hoverKey_ = key;
    if (key.second == 0) {
        return;
    }
    for (const int index : outlineHover_(key.first, key.second)) {
        if (index == key.second) {
            continue; // the detected owner itself
        }
        const auto owner = edgeOwner(key.first, index);
        if (owner && !owner->IsSelected()) {
            hoverOwners_.push_back(owner);
        }
    }
    if (!hoverOwners_.empty()) {
        showHoverHighlight(hoverOwners_);
    }
}

void OcctView::setVertexPicking(bool on) {
    vertexPicking_ = on;
    for (const auto& [id, shown] : shown_) {
        if (shown.selectable) {
            activateSelection(shown.ais);
        }
    }
}

void OcctView::setStockTransparency(double transparency) {
    stockTransparency_ = transparency;
    if (!context_) {
        return;
    }
    for (const auto& prs : simStockPrs_) {
        if (const auto stock = occ::down_cast<StockMeshPrs>(prs)) {
            stock->setStockTransparency(transparency);
            context_->Redisplay(stock, false);
        }
    }
    applyModelVisibility();
    requestRedraw();
}

void OcctView::setColors(const ViewColors& colors) {
    if (colors == colors_) {
        return;
    }
    colors_ = colors;
    if (!context_) {
        modelColor_ = colors_.part; // the rest is applied when the viewer is made
        return;
    }
    applyViewColors();
    applyModelColor();
    for (auto& [id, shown] : shown_) {
        if (shown.fixture && !shown.colour) {
            context_->SetColor(shown.ais, toQuantity(colors_.fixture), false);
        }
    }
    if (stock_) {
        context_->SetColor(stock_, toQuantity(colors_.stockBox), false);
    }
    // Toolpaths: drawn again in the new colours.
    for (auto& [id, shown] : toolpaths_) {
        context_->Remove(shown.prs, false);
        shown.prs = new ToolpathPrs(*shown.path, shown.wcsToProject, shown.emphasized, colors_);
        context_->Display(shown.prs, 0, -1, false);
        context_->Deactivate(shown.prs);
        if (toolpathsHidden_) {
            context_->Erase(shown.prs, false);
        }
    }
    requestRedraw();
}

void OcctView::applyViewColors() {
    if (view_) {
        view_->SetBgGradientColors(toQuantity(colors_.backgroundTop),
                                   toQuantity(colors_.backgroundBottom),
                                   Aspect_GradientFillMethod_Vertical);
    }
    // Selected items and the one under the cursor, unlit (the colour stays the same whichever
    // way a face points), with thick edges.
    for (const auto type : {Prs3d_TypeOfHighlight_Selected, Prs3d_TypeOfHighlight_LocalSelected,
                            Prs3d_TypeOfHighlight_Dynamic, Prs3d_TypeOfHighlight_LocalDynamic}) {
        const bool selected =
            type == Prs3d_TypeOfHighlight_Selected || type == Prs3d_TypeOfHighlight_LocalSelected;
        const occ::handle<Prs3d_Drawer>& style = context_->HighlightStyle(type);
        const Quantity_Color colour = toQuantity(selected ? colors_.selected : colors_.hover);
        style->SetColor(colour);
        occ::handle<Prs3d_ShadingAspect> shading = new Prs3d_ShadingAspect();
        shading->SetColor(colour);
        shading->Aspect()->SetShadingModel(Graphic3d_TypeOfShadingModel_Unlit);
        style->SetShadingAspect(shading);
        style->SetWireAspect(new Prs3d_LineAspect(colour, Aspect_TOL_SOLID, 3.0));
        style->SetLineAspect(new Prs3d_LineAspect(colour, Aspect_TOL_SOLID, 3.0));
    }
}

void OcctView::applyModelColor() {
    if (!context_) {
        return;
    }
    // Called for every simulation frame: recolour (and so recompute) the models only when their
    // colour changes.
    const QColor wanted =
        simActive_ && stockTransparency_ > 0.0 ? colors_.partThroughStock : colors_.part;
    if (wanted == modelColor_) {
        return;
    }
    modelColor_ = wanted;
    for (auto& [id, shown] : shown_) {
        if (!shown.fixture && !shown.colour) {
            context_->SetColor(shown.ais, toQuantity(wanted), false);
        }
    }
}

void OcctView::setToggleSelection(bool on) {
    ChangeMouseSelectionSchemes().Bind(Aspect_VKeyMouse_LeftButton,
                                       on ? AIS_SelectionScheme_XOR : AIS_SelectionScheme_Replace);
}

std::vector<std::pair<Uuid, int>> OcctView::selectedFaces() const {
    std::vector<std::pair<Uuid, int>> result;
    for (const PickedShape& s : selectedShapes()) {
        if (s.type == cad::SubShapeType::Face) {
            result.emplace_back(s.model, s.index);
        }
    }
    return result;
}

std::vector<OcctView::PickedShape> OcctView::selectedShapes() const {
    std::vector<PickedShape> result;
    if (!context_) {
        return result;
    }
    for (context_->InitSelected(); context_->MoreSelected(); context_->NextSelected()) {
        if (!context_->HasSelectedShape()) {
            continue;
        }
        const TopoDS_Shape picked = context_->SelectedShape();
        const TopAbs_ShapeEnum kind = picked.ShapeType();
        if (kind != TopAbs_FACE && kind != TopAbs_EDGE && kind != TopAbs_VERTEX) {
            continue;
        }
        const occ::handle<AIS_InteractiveObject> owner = context_->SelectedInteractive();
        for (const auto& [id, shown] : shown_) {
            if (shown.ais != owner) {
                continue;
            }
            // Match on the underlying geometry (TShape): the picked sub-shape may carry the
            // presentation location. Indices follow cad::subShapeCount's TopExp map order.
            NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> shapes;
            TopExp::MapShapes(cad::occtShape(shown.model), kind, shapes);
            for (int i = 1; i <= shapes.Extent(); ++i) {
                if (shapes(i).TShape() == picked.TShape()) {
                    result.push_back({id,
                                      kind == TopAbs_FACE   ? cad::SubShapeType::Face
                                      : kind == TopAbs_EDGE ? cad::SubShapeType::Edge
                                                            : cad::SubShapeType::Vertex,
                                      i});
                    break;
                }
            }
        }
    }
    return result;
}

void OcctView::fitBox(const geom3d::Aabb3& box) {
    if (!view_ || box.isEmpty()) {
        return;
    }
    Bnd_Box b;
    b.Update(box.min.x(), box.min.y(), box.min.z(), box.max.x(), box.max.y(), box.max.z());
    view_->FitAll(b, 0.05, false);
    requestRedraw();
}

void OcctView::fitAll() {
    if (view_) {
        view_->FitAll(0.05, false);
        requestRedraw();
    }
}

bool OcctView::setReferenceFrame(const Mat3& axes, const Mat3& standing) {
    ensureViewer();
    const bool turned = !standing.isApprox(standing_);
    if (axes.isApprox(reference_) && !turned) {
        return false;
    }
    reference_ = axes;
    standing_ = standing;
    // The cube stands as the machine does (its Top up, its Front towards the operator); its triad
    // names the machine's axes along the world's right, back and up.
    Transform3 rotation = Transform3::Identity();
    rotation.linear() = axes * standing;
    viewCube_->SetLocalTransformation(toTrsf(rotation));
    if (turned) {
        labelCube();
    }
    context_->Redisplay(viewCube_, false);
    requestRedraw();
    return turned;
}

namespace {

/// The machine axis along a column of a standing ("Z", "-Y"): a signed unit axis.
TCollection_AsciiString axisName(const Vec3& column) {
    int i = 0;
    column.cwiseAbs().maxCoeff(&i);
    const char* names[] = {"X", "Y", "Z"};
    return TCollection_AsciiString(column[i] < 0.0 ? "-" : "") + names[i];
}

} // namespace

void OcctView::labelCube() {
    if (auto* cube = dynamic_cast<AIS_ViewCube*>(viewCube_.get())) {
        cube->SetAxesLabels(axisName(standing_.col(0)), axisName(standing_.col(1)),
                            axisName(standing_.col(2)));
    }
}

QString OcctView::cubeAxes() const {
    const auto* cube = dynamic_cast<const AIS_ViewCube*>(viewCube_.get());
    if (!cube) {
        return {};
    }
    QStringList names;
    for (const Prs3d_DatumParts part :
         {Prs3d_DatumParts_XAxis, Prs3d_DatumParts_YAxis, Prs3d_DatumParts_ZAxis}) {
        names << QString::fromUtf8(cube->AxisLabel(part).ToCString());
    }
    return names.join(QLatin1Char(' '));
}

Vec3 OcctView::viewDirection() const {
    if (!view_) {
        return Vec3::Zero();
    }
    const gp_Dir d = view_->Camera()->Direction();
    return Vec3(d.X(), d.Y(), d.Z());
}

Vec3 OcctView::viewUp() const {
    if (!view_) {
        return Vec3::Zero();
    }
    const gp_Dir u = view_->Camera()->Up();
    return Vec3(u.X(), u.Y(), u.Z());
}

void OcctView::lookFromWorld(const Vec3& eye) {
    const Mat3 world = reference_ * standing_;
    lookFrom(world * eye, world.col(2));
}

void OcctView::snapView(V3d_TypeOfOrientation orientation, bool animate) {
    const gp_Dir proj = V3d::GetProjAxis(orientation);
    snapWorld(Vec3(proj.X(), proj.Y(), proj.Z()), animate);
}

void OcctView::snapWorld(const Vec3& worldEye, bool animate) {
    ensureViewer();
    // Directions in the world as the machine stands, from the target towards the eye. Looking
    // down or up, away from the operator is up the view; otherwise up is up.
    const Vec3 localUp = std::abs(worldEye.z()) > 0.999 ? Vec3::UnitY() : Vec3::UnitZ();
    const Mat3 world = reference_ * standing_;
    const Vec3 eye = (world * worldEye).normalized();
    Vec3 up = world * localUp;
    up = (up - up.dot(eye) * eye).normalized();

    const occ::handle<Graphic3d_Camera> start = new Graphic3d_Camera(view_->Camera());
    view_->Camera()->SetDirection(gp_Dir(-eye.x(), -eye.y(), -eye.z()));
    view_->Camera()->SetUp(gp_Dir(up.x(), up.y(), up.z()));
    view_->FitAll(0.05, false);
    const occ::handle<Graphic3d_Camera> end = new Graphic3d_Camera(view_->Camera());
    if (animate) {
        view_->Camera()->Copy(start);
        AbortViewAnimation();
        ViewAnimation()->SetView(view_);
        ViewAnimation()->SetCameraStart(start);
        ViewAnimation()->SetCameraEnd(end);
        ViewAnimation()->SetOwnDuration(0.3);
        ViewAnimation()->StartTimer(0.0, 1.0, true, false);
    }
    requestRedraw();
}

void OcctView::activateSelection(const occ::handle<AIS_Shape>& shape) {
    if (!context_ || !shape) {
        return;
    }
    // Faces and edges at once: a click picks whatever is under the cursor.
    context_->SetSelectionModeActive(shape, AIS_Shape::SelectionMode(TopAbs_FACE), true,
                                     AIS_SelectionModesConcurrency_Multiple);
    context_->SetSelectionModeActive(shape, AIS_Shape::SelectionMode(TopAbs_EDGE), true,
                                     AIS_SelectionModesConcurrency_Multiple);
    context_->SetSelectionModeActive(shape, AIS_Shape::SelectionMode(TopAbs_VERTEX), vertexPicking_,
                                     AIS_SelectionModesConcurrency_Multiple);
}

QString OcctView::pickAt(const QPoint& position, bool toggle) {
    ensureViewer();
    view_->Redraw();
    const NCollection_Vec2<int> p = devicePoint(QPointF(position));
    context_->MoveTo(p.x(), p.y(), view_, false);
    context_->SelectDetected(toggle ? AIS_SelectionScheme_XOR : AIS_SelectionScheme_Replace);
    context_->InitSelected();
    if (emitMachinePick(context_)) {
        return QStringLiteral("machine part");
    }
    if (!context_->MoreSelected() || !context_->HasSelectedShape()) {
        return {};
    }
    return describe(context_->SelectedShape(), unit_);
}

void OcctView::setBusyText(const QString& text) {
    ensureViewer();
    if (text.isEmpty()) {
        if (busyLabel_) {
            context_->Remove(busyLabel_, false);
            busyLabel_.Nullify();
            requestRedraw();
        }
        return;
    }
    occ::handle<AIS_TextLabel> label = occ::down_cast<AIS_TextLabel>(busyLabel_);
    if (!label) {
        label = new AIS_TextLabel();
        label->SetPosition(gp_Pnt(0.0, 0.0, 0.0));
        label->SetHJustification(Graphic3d_HTA_LEFT);
        label->SetVJustification(Graphic3d_VTA_TOP);
        label->SetHeight(15.0);
        label->SetColor(Quantity_Color(1.0, 0.82, 0.30, Quantity_TOC_sRGB));
        label->SetDisplayType(Aspect_TODT_SHADOW);
        label->SetColorSubTitle(Quantity_Color(0.0, 0.0, 0.0, Quantity_TOC_sRGB));
        label->SetTransformPersistence(new Graphic3d_TransformPers(
            Graphic3d_TMF_2d, Aspect_TOTP_LEFT_UPPER, NCollection_Vec2<int>(14, 14)));
        label->SetZLayer(Graphic3d_ZLayerId_TopOSD);
        label->SetText(TCollection_ExtendedString(text.toUtf8().constData(), true));
        busyLabel_ = label;
        context_->Display(label, 0, -1, false);
        context_->Deactivate(label);
    } else {
        label->SetText(TCollection_ExtendedString(text.toUtf8().constData(), true));
        context_->Redisplay(label, false);
    }
    requestRedraw();
}

int OcctView::hoverAt(const QPoint& position) {
    ensureViewer();
    UpdateMousePosition(devicePoint(QPointF(position)), Aspect_VKeyMouse_NONE,
                        Aspect_VKeyFlags_NONE, false);
    FlushViewEvents(context_, view_, true);
    return static_cast<int>(hoverOwners_.size());
}

bool OcctView::dumpImage(const QString& path) {
    ensureViewer();
    view_->Redraw();
    return view_->Dump(path.toUtf8().constData());
}

void OcctView::requestRedraw() {
    // Presentations changed: the whole frame must be redrawn, not only the immediate layer
    // that mouse highlighting uses.
    if (view_) {
        view_->Invalidate();
    }
    update();
}

void OcctView::paintEvent(QPaintEvent* /*event*/) {
    ensureViewer();
    view_->InvalidateImmediate();
    FlushViewEvents(context_, view_, true);
}

void OcctView::resizeEvent(QResizeEvent* /*event*/) {
    if (view_) {
        updateWindowSize();
        view_->MustBeResized();
        update();
    }
}

void OcctView::mousePressEvent(QMouseEvent* event) {
    if (UpdateMouseButtons(devicePoint(event->position()), toOcctButtons(event->buttons()),
                           toOcctModifiers(event->modifiers()), false)) {
        update();
    }
}

void OcctView::mouseReleaseEvent(QMouseEvent* event) {
    lastClickShift_ = event->modifiers().testFlag(Qt::ShiftModifier);
    if (UpdateMouseButtons(devicePoint(event->position()), toOcctButtons(event->buttons()),
                           toOcctModifiers(event->modifiers()), false)) {
        update();
    }
}

void OcctView::mouseMoveEvent(QMouseEvent* event) {
    if (UpdateMousePosition(devicePoint(event->position()), toOcctButtons(event->buttons()),
                            toOcctModifiers(event->modifiers()), false)) {
        update();
    }
}

void OcctView::wheelEvent(QWheelEvent* event) {
    // One notch (120 units) is one zoom step.
    if (UpdateZoom(
            Aspect_ScrollDelta(devicePoint(event->position()), event->angleDelta().y() / 120.0))) {
        update();
    }
}

void OcctView::handleViewRedraw(const occ::handle<AIS_InteractiveContext>& context,
                                const occ::handle<V3d_View>& view) {
    AIS_ViewController::handleViewRedraw(context, view);
    if (toAskNextFrame()) {
        // Continue animations (e.g. smooth zoom) on the next event loop iteration.
        QMetaObject::invokeMethod(this, qOverload<>(&QWidget::update), Qt::QueuedConnection);
    }
}

bool OcctView::emitMachinePick(const occ::handle<AIS_InteractiveContext>& context) {
    context->InitSelected();
    if (!context->MoreSelected()) {
        return false;
    }
    int index = -1;
    if (const auto part = occ::down_cast<MachinePartPrs>(context->SelectedInteractive())) {
        index = static_cast<int>(part->index());
    } else {
        const occ::handle<AIS_InteractiveObject> picked = context->SelectedInteractive();
        for (std::size_t i = 0; i < machinePrs_.size(); ++i) {
            if (!machinePrs_[i].solid.isNull() && machinePrs_[i].prs == picked) {
                index = static_cast<int>(i);
            }
        }
    }
    if (index < 0) {
        return false;
    }
    const gp_Pnt at = context->MainSelector()->NbPicked() > 0
                          ? context->MainSelector()->PickedPoint(1)
                          : gp_Pnt(0, 0, 0);
    const ShownPart& shown = machinePrs_[static_cast<std::size_t>(index)];
    if (!shown.solid.isNull() && context->HasSelectedShape()) {
        const TopoDS_Shape sub = context->SelectedShape();
        const int subIndex = cad::subShapeIndex(shown.solid, sub);
        if (subIndex > 0) {
            const cad::SubShapeType type = sub.ShapeType() == TopAbs_FACE ? cad::SubShapeType::Face
                                           : sub.ShapeType() == TopAbs_EDGE
                                               ? cad::SubShapeType::Edge
                                               : cad::SubShapeType::Vertex;
            context->ClearSelected(false);
            emit machineShapeClicked(index, static_cast<int>(type), subIndex, at.X(), at.Y(),
                                     at.Z());
        }
    }
    context->ClearSelected(false); // the owner of the scene highlights it
    emit machinePartPicked(index);
    emit machinePartClicked(index, at.X(), at.Y(), at.Z());
    return true;
}

QPoint OcctView::widgetPoint(const Vec3& point) {
    ensureViewer();
    int x = 0, y = 0;
    view_->Convert(point.x(), point.y(), point.z(), x, y);
    const qreal ratio = devicePixelRatioF();
    return QPoint(qRound(x / ratio), qRound(y / ratio));
}

void OcctView::OnSelectionChanged(const occ::handle<AIS_InteractiveContext>& context,
                                  const occ::handle<V3d_View>& /*view*/) {
    lastPick_.reset();
    if (context->MainSelector()->NbPicked() > 0) {
        const gp_Pnt at = context->MainSelector()->PickedPoint(1);
        lastPick_ = Vec3(at.X(), at.Y(), at.Z());
    }
    context->InitSelected();
    if (emitMachinePick(context)) {
        return;
    }
    if (!context->MoreSelected() || !context->HasSelectedShape()) {
        emit selectionChanged(QString());
        return;
    }
    emit selectionChanged(describe(context->SelectedShape(), unit_));
}

} // namespace ocam::gui
