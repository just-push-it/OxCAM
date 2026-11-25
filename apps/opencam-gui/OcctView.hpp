// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// 3D viewport: a native-window QWidget that OpenCASCADE renders into directly. Mouse input is
// forwarded to AIS_ViewController (left drag: rotate, middle drag: pan, wheel: zoom, click:
// select). A view cube in the lower left corner shows the orientation of the reference frame
// (the current setup's WCS); clicking its faces, edges and corners turns the view to them.

#pragma once

#include "ViewColors.hpp"

#include <ocam/cad/Model.hpp>
#include <ocam/cad/SubShape.hpp>
#include <ocam/document/Project.hpp>
#include <ocam/document/Stock.hpp>
#include <ocam/geom2d/Polyline.hpp>
#include <ocam/machine/Machine.hpp>
#include <ocam/sim/Compare.hpp>
#include <ocam/sim/ToolShape.hpp>
#include <ocam/tooling/TurningHolder.hpp>
#include <ocam/toolpath/Toolpath.hpp>

#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <AIS_ViewController.hxx>
#include <Aspect_NeutralWindow.hxx>
#include <QWidget>
#include <TopAbs_ShapeEnum.hxx>
#include <V3d_TypeOfOrientation.hxx>
#include <V3d_View.hxx>
#include <V3d_Viewer.hxx>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <vector>

namespace ocam::gui {

class OcctView : public QWidget, protected AIS_ViewController {
    Q_OBJECT

public:
    explicit OcctView(QWidget* parent = nullptr);
    ~OcctView() override;

    struct ViewModel {
        Uuid id;
        cad::Model model;
        Transform3 placement = Transform3::Identity();
        bool fixture = false; ///< Drawn in the fixture colour.
        /// Faces and edges can be picked (not for workholding bodies, which are no project
        /// models).
        bool selectable = true;
        /// Its own colour, 0xRRGGBB (workholding parts, highlighted parts in an editor).
        std::optional<std::uint32_t> colour;
    };

    /// Shows exactly these models (incremental: unchanged models are not re-displayed). The
    /// view fits all when the first model appears.
    void setModels(const std::vector<ViewModel>& models);

    /// Translucent stock overlay, or none.
    void setStock(const std::optional<document::StockShape>& stock);

    /// Work coordinate system triad, or none.
    void setWcs(const std::optional<document::Frame>& wcs);

    struct ToolpathView {
        Uuid operationId;
        std::shared_ptr<const toolpath::Toolpath> path;
        Transform3 wcsToProject = Transform3::Identity();
        bool emphasized = false;
    };

    /// Shows exactly these toolpaths (unchanged ones are not re-displayed).
    void setToolpaths(const std::vector<ToolpathView>& paths);

    /// A machine in the view (DL-0037): its parts' meshes and where each one is.
    struct MachineScene {
        /// The parts; the same pointer keeps the shown meshes (only their places change).
        std::shared_ptr<const std::vector<machine::ModelPart>> parts;
        /// Per part, its exact geometry (part coordinates), drawn and picked as B-rep instead of
        /// its mesh: faces, edges and vertices (machineShapeClicked). Missing or null entries
        /// show the mesh. The views that pick on the machine set them; the simulation draws
        /// meshes.
        std::vector<cad::Model> solids;
        std::vector<Transform3> placements; ///< Per part: mesh -> view.
        std::vector<bool> hidden;           ///< Per part: not drawn (missing entries are drawn).
        std::vector<bool> highlighted;      ///< Per part: drawn in the highlight colour.
        bool pickable = false;              ///< Clicking a part emits machinePartPicked.
        /// Lines to draw (the rotary axes), highlighted ones brighter and thicker.
        struct Line {
            Vec3 from = Vec3::Zero();
            Vec3 to = Vec3::Zero();
            bool highlighted = false;
            std::uint32_t colour = 0; ///< 0xRRGGBB; 0 = the axis line colour
        };
        std::vector<Line> lines;
        /// Words at points, drawn over everything (what a point or a line means).
        struct Label {
            Vec3 at = Vec3::Zero();
            std::string text;
            std::uint32_t colour = 0xFFFFFF; ///< 0xRRGGBB
        };
        std::vector<Label> labels;
        std::vector<Transform3> frames; ///< Axis triads to show (spindle, table), view.
        double transparency = 0.0;
    };
    /// Shows a machine, or none. The machine library's preview uses it alone; the simulation
    /// sets it through its scene.
    void setMachine(const std::optional<MachineScene>& scene);

    /// Simulation display: the simulated stock replaces the translucent stock overlay, the tool
    /// is drawn at its tip position and collisions are marked. Coordinates are WCS.
    struct SimulationScene {
        /// The stock mesh in chunks; chunks that did not change keep their pointers.
        std::vector<std::shared_ptr<const sim::DisplayMesh>> stock;
        Transform3 wcsToProject = Transform3::Identity();
        std::shared_ptr<const sim::ToolShape> tool; ///< nullptr hides the tool.
        /// Turning (without `tool`): the insert outline in the lathe plane (u = Z, v = radius,
        /// about the programmed point), drawn in the WCS XZ plane at the tool tip, and its
        /// thickness (0: drawn at a thickness from its size).
        std::shared_ptr<const geom2d::Polyline> insert;
        double insertThickness = 0.0;
        /// With `insert`: the tool's holder, placed like it.
        std::shared_ptr<const std::vector<tooling::TurningToolPart>> toolParts;
        Vec3 toolTip = Vec3::Zero();
        Vec3 toolAxis = Vec3::UnitZ(); ///< Multi-axis toolpaths tilt the tool.
        std::vector<Vec3> collisions;
        bool showModels = true;
        bool showFixtures = true; ///< Fixtures and workholding.
        bool showToolpaths = false;
        std::optional<MachineScene> machine;
        /// Project frame -> view for the models, stock, toolpaths, tool and markers: identity
        /// keeps the part still (the machine moves around it); the table's motion keeps the
        /// machine still.
        Transform3 viewFromProject = Transform3::Identity();
        /// Models that move on their own, in the view: a lathe's tailstock coming in.
        std::map<Uuid, Transform3> modelMoves;
    };
    void setSimulation(const std::optional<SimulationScene>& scene);
    /// Moves the simulated tool (WCS tip and axis) without touching the rest of the scene, for
    /// smooth playback between stock updates.
    void moveSimulationTool(const Vec3& tip, const Vec3& axis);
    bool simulationShown() const noexcept { return simActive_; }

    /// Faces currently selected in the view: (model id, 1-based face index).
    std::vector<std::pair<Uuid, int>> selectedFaces() const;
    /// Faces and edges selected in the view: model, kind and 1-based index.
    struct PickedShape {
        Uuid model;
        cad::SubShapeType type = cad::SubShapeType::Face;
        int index = 0;
    };
    std::vector<PickedShape> selectedShapes() const;
    /// Selects exactly these faces and edges in the view (to show an operation's geometry).
    void selectShapes(const std::vector<PickedShape>& shapes);
    /// While on, a plain click adds or removes the face or edge under the cursor instead of
    /// replacing the selection (picking an operation's geometry).
    void setToggleSelection(bool on);
    /// Vertices become pickable too (picking a start point).
    void setVertexPicking(bool on);
    /// While set, hovering an edge (without Shift) also highlights the other edges `outline`
    /// returns for it (the edge indices of its closed outline); nullptr turns it off.
    using OutlineFn = std::function<std::vector<int>(const Uuid& model, int edgeIndex)>;
    void setOutlineHover(OutlineFn outline);
    /// Shift was held for the last click in the view.
    bool lastClickWithShift() const noexcept { return lastClickShift_; }
    /// Transparency of the simulated stock (0 = opaque). While it is see-through, the part
    /// shows in its "under transparent stock" colour.
    void setStockTransparency(double transparency);
    /// The colours of the part, stock, toolpaths, selection and background.
    void setColors(const ViewColors& colors);
    const ViewColors& colors() const noexcept { return colors_; }

    void fitAll();
    /// Looks from `eye` (a direction) with `up` up the view.
    void lookFrom(const Vec3& eye, const Vec3& up);
    /// Fits the view to a box (view frame).
    void fitBox(const geom3d::Aabb3& box);
    /// Test hook: where a point of the view frame shows in the widget (pixels).
    QPoint widgetPoint(const Vec3& point);

    /// The frame standard views are taken in (the current setup's WCS axes, as columns, in the
    /// project frame), and how the machine stands in it (DL-0077): the world's right, away from
    /// the operator and up, as columns in those axes (machine::MachineOrientation::worldAxes;
    /// identity for a machine with Z up and its front at -Y). The view cube stands as the machine
    /// does, its triad naming the setup's axes. True when the standing changed (the caller turns
    /// the view to it).
    bool setReferenceFrame(const Mat3& axes, const Mat3& standing = Mat3::Identity());

    /// Turns the view to a standard orientation as the machine stands (faces, edges and corners of
    /// the view cube; V3d_Zpos is the top view, looking down; V3d_Yneg the front, from where the
    /// operator stands) and fits all.
    void snapView(V3d_TypeOfOrientation orientation, bool animate = true);
    /// Looks from `eye`, a direction in the world as the machine stands (x right, y away from the
    /// operator, z up), with up up the view. Does not fit.
    void lookFromWorld(const Vec3& eye);
    /// Test hooks: the camera's direction (from the eye) and up, in the view frame; the names on
    /// the view cube's triad along the world's right, back and up ("Z X Y" on a horizontal lathe).
    Vec3 viewDirection() const;
    Vec3 viewUp() const;
    QString cubeAxes() const;

    /// Unit of the measurements shown for picked faces and edges.
    void setDisplayUnit(LengthUnit unit) { unit_ = unit; }

    /// Renders the current view into an image file (PNG on Windows; PPM everywhere).
    bool dumpImage(const QString& path);
    /// Whether the view draws into the window it has now: false once Qt gave the widget another
    /// native window (its top-level window closed and shown again), which it never follows.
    bool drawsIntoItsWindow() const noexcept {
        return !window_ || drawnInto_ == static_cast<std::uintptr_t>(internalWinId());
    }

    /// Test hook: picks at a widget position as a click would and returns what was selected
    /// (empty when nothing). `toggle`: as a click while picking an operation's geometry, adding
    /// to the selection or taking out of it.
    QString pickAt(const QPoint& position, bool toggle = false);
    /// Where the last click in the view hit a model's surface (view frame), if it hit one.
    std::optional<Vec3> lastPickedPoint() const noexcept { return lastPick_; }
    /// A note in the top-left corner of the view (e.g. stock still being computed); empty
    /// removes it.
    void setBusyText(const QString& text);
    /// Test hook: moves the cursor to `position` (widget pixels) and returns how many edges the
    /// outline hover highlights besides the one under the cursor.
    int hoverAt(const QPoint& position);

signals:
    void selectionChanged(const QString& description);
    /// A part of the machine scene (its index in MachineScene::parts) was clicked, at a point
    /// of its surface (view frame).
    void machinePartPicked(int index);
    void machinePartClicked(int index, double x, double y, double z);
    /// A face, edge or vertex of a part drawn as B-rep (MachineScene::solids) was clicked:
    /// emitted just before machinePartPicked/Clicked. `type` is a cad::SubShapeType, `subIndex`
    /// its 1-based index in the part's model; the point is in the view frame.
    void machineShapeClicked(int index, int type, int subIndex, double x, double y, double z);

protected:
    QPaintEngine* paintEngine() const override { return nullptr; }
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

    void handleViewRedraw(const occ::handle<AIS_InteractiveContext>& context,
                          const occ::handle<V3d_View>& view) override;
    void OnSelectionChanged(const occ::handle<AIS_InteractiveContext>& context,
                            const occ::handle<V3d_View>& view) override;

private:
    void ensureViewer();
    void requestRedraw();
    void updateWindowSize();
    NCollection_Vec2<int> devicePoint(const QPointF& p) const;
    void activateSelection(const occ::handle<AIS_Shape>& shape);

    occ::handle<V3d_Viewer> viewer_;
    occ::handle<V3d_View> view_;
    occ::handle<AIS_InteractiveContext> context_;
    occ::handle<Aspect_NeutralWindow> window_;
    std::uintptr_t drawnInto_ = 0; ///< The native window the view was made for.
    struct Shown {
        cad::Model model;
        Transform3 placement;
        occ::handle<AIS_Shape> ais;
        bool fixture = false;
        bool selectable = true;
        std::optional<std::uint32_t> colour;
    };
    std::map<Uuid, Shown> shown_;
    struct ShownToolpath {
        std::shared_ptr<const toolpath::Toolpath> path;
        bool emphasized = false;
        Transform3 wcsToProject = Transform3::Identity();
        occ::handle<AIS_InteractiveObject> prs;
    };
    ViewColors colors_;
    QColor modelColor_ = ViewColors().part; ///< The colour the models show in now.
    /// The selected and hovered highlight styles and the background, in colors_.
    void applyViewColors();
    /// The part's colour: its own, or the "under transparent stock" one while the simulated
    /// stock is see-through.
    void applyModelColor();
    std::map<Uuid, ShownToolpath> toolpaths_;
    occ::handle<AIS_InteractiveObject> stock_;
    bool simActive_ = false;
    bool modelsHidden_ = false;
    bool fixturesHidden_ = false;
    std::optional<Vec3> lastPick_;
    bool toolpathsHidden_ = false;
    std::vector<std::shared_ptr<const sim::DisplayMesh>> simMeshes_;
    Transform3 simXf_ = Transform3::Identity();
    std::shared_ptr<const sim::ToolShape> simTool_;
    std::shared_ptr<const geom2d::Polyline> simInsert_;
    std::vector<Vec3> simCollisions_;
    std::vector<occ::handle<AIS_InteractiveObject>> simStockPrs_; ///< One per chunk.
    double stockTransparency_ = 0.0;
    bool sceneShowsModels_ = true;   ///< The simulation scene's own choice.
    bool sceneShowsFixtures_ = true; ///< The simulation scene's own choice.
    /// Models hidden while simulating, unless the scene shows them or the stock is see-through
    /// (the part then shows as a solid inside it); fixtures as the scene says.
    void applyModelVisibility();
    bool vertexPicking_ = false;
    bool lastClickShift_ = false;
    OutlineFn outlineHover_;
    occ::handle<AIS_InteractiveObject> busyLabel_;
    std::pair<Uuid, int> hoverKey_{Uuid(), 0};
    std::vector<occ::handle<SelectMgr_EntityOwner>> hoverOwners_;
    void updateOutlineHover();
    void clearOutlineHover();
    /// Redraws the mouse highlight: the context's detected owner plus `extra`.
    void showHoverHighlight(const std::vector<occ::handle<SelectMgr_EntityOwner>>& extra);
    /// The owner of edge `index` of a shown model in its edge selection (null if none).
    occ::handle<SelectMgr_EntityOwner> edgeOwner(const Uuid& model, int index) const;

protected:
    void handleMoveTo(const occ::handle<AIS_InteractiveContext>& context,
                      const occ::handle<V3d_View>& view) override;

private:
    occ::handle<AIS_InteractiveObject> simToolPrs_;
    occ::handle<AIS_InteractiveObject> simMarkersPrs_;
    /// The machine shown (setMachine), per part.
    struct ShownPart {
        std::shared_ptr<const geom3d::Mesh> mesh;
        cad::Model solid; ///< Drawn as B-rep (an AIS_Shape) when set.
        std::uint32_t colour = 0;
        double transparency = 0.0;
        occ::handle<AIS_InteractiveObject> prs;
    };
    std::vector<ShownPart> machinePrs_;
    std::vector<occ::handle<AIS_InteractiveObject>> machineFrames_;
    std::vector<occ::handle<AIS_InteractiveObject>> machineLines_;
    /// Emits machinePartPicked/Clicked for a selected machine part; false for anything else.
    bool emitMachinePick(const occ::handle<AIS_InteractiveContext>& context);
    /// Project frame -> view (SimulationScene::viewFromProject), applied to everything placed
    /// in the project frame.
    Transform3 viewFromProject_ = Transform3::Identity();
    std::map<Uuid, Transform3> modelMoves_;
    void placeModels();
    void setViewFromProject(const Transform3& xf);
    /// The simulated tool's place (project frame) before viewFromProject_.
    Transform3 simToolAt_ = Transform3::Identity();
    occ::handle<AIS_InteractiveObject> wcs_;
    occ::handle<AIS_InteractiveObject> viewCube_;
    Mat3 reference_ = Mat3::Identity();
    Mat3 standing_ = Mat3::Identity(); ///< setReferenceFrame's standing.
    /// Names the cube's triad with the machine's axes as it stands.
    void labelCube();
    /// Turns to a direction in the world (from the target towards the eye) and fits all.
    void snapWorld(const Vec3& worldEye, bool animate);
    LengthUnit unit_ = LengthUnit::Millimeter;
};

} // namespace ocam::gui
