// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The workholding setup wizard (DL-0035): sets up a common fixture from its parts by asking what
// it is (a vise, a self-centring vise, a dovetail fixture, a plate, a chuck, a collet) and then
// for clicks in a 3D view, one question at a time: which parts move, which faces clamp, where the
// jaw tops are, what it stands on. Parts the fixture does not need (another jaw set, a riser) are
// removed and parts that are one are merged in its parts list. The type decides the motion (a
// self-centring vise's jaws move together), the last page sets the opening range and lets the
// jaws be tried on a trial stock.

#pragma once

#include <ocam/cad/Datum.hpp>
#include <ocam/cad/Model.hpp>
#include <ocam/document/Workholding.hpp>

#include <QDialog>
#include <QImage>
#include <map>
#include <optional>
#include <string>
#include <vector>

class QButtonGroup;
class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QListWidget;
class QPushButton;
class QSlider;
class QWidget;

namespace ocam::gui {

class OcctView;

class WorkholdingWizard : public QDialog {
    Q_OBJECT

public:
    /// `start`: the parts (with their files) to set up; its name and description are kept.
    /// `models`: the parts' models by file hash (read from the files).
    WorkholdingWizard(document::WorkholdingPackage start, std::map<std::string, cad::Model> models,
                      QWidget* parent = nullptr);

    /// The template set up, with its part files (after the dialog was accepted).
    const document::WorkholdingPackage& result() const noexcept { return result_; }

    // Test hooks: what a user does, without the mouse.
    void setType(document::FixtureType type);
    /// Clicks a part (on a parts question) or one of its faces or edges (on a pick question).
    bool clickPart(const std::string& partName);
    bool clickShape(const std::string& partName, cad::SubShapeType type, int index);
    /// Removes a part, or merges parts into the first one's place (named after them).
    bool removePart(const std::string& partName);
    bool mergeParts(const std::vector<std::string>& partNames);
    std::vector<std::string> partNames() const;
    const cad::Model* partModel(const std::string& partName) const;
    bool next();
    /// Closes the jaws on a trial stock of this size on the last page (nullopt: the slider).
    void setTrialStock(const std::optional<Vec3>& size);
    QString stepTitle() const;
    QString problemText() const;
    /// Builds the template from the answers (the Finish button); false with a problem shown.
    bool finish();
    /// The dialog with its 3D view drawn in.
    QImage snapshot();

private:
    enum class Ask { Type, Parts, Face, Edge, Finish };
    enum class Slot { None, Moving, Opposite, Axis, FaceA, FaceB, Top, XEdge, Mount };
    struct Step {
        Ask ask = Ask::Type;
        Slot slot = Slot::None;
        QString title;
        QString help;
        bool optional = false;
    };

    void buildSteps();
    void showStep(int index);
    bool answered(const Step& step) const;
    void picked();
    void answerPart(const document::WorkholdingPart& part);
    void answerShape(const document::WorkholdingPart& part, cad::SubShapeType type, int index);
    QString answerText(const Step& step) const;
    void refreshSteps();
    /// The template from the answers so far (Finish page), or nullopt with the problem shown.
    void rebuild();
    void updateView();
    void setFinishDefaults();
    /// The parts list (its selection kept), after a part is removed or merged.
    void refreshParts();
    std::vector<Uuid> selectedParts() const;
    void removeParts(const std::vector<Uuid>& ids);
    void mergeParts(const std::vector<Uuid>& ids);
    /// Answers that named removed parts: picks on them are cleared, parts lists take `merged`.
    void forgetParts(const std::vector<Uuid>& gone, const std::optional<Uuid>& merged);
    bool chuck() const noexcept;
    /// The trial stock (WCS, its top at Z 0): a box, or a round bar's box for chucks.
    geom3d::Aabb3 trialStock() const;
    const document::WorkholdingPart* partByName(const std::string& name) const;
    const document::WorkholdingPart* partById(const Uuid& id) const;
    const cad::Model* modelOf(const document::WorkholdingPart& part) const;

    document::WorkholdingPackage start_;
    std::map<std::string, cad::Model> models_;
    document::WorkholdingAnswers answers_;
    std::vector<Step> steps_;
    int current_ = 0;
    bool defaultsSet_ = false;
    /// The template from the clicked answers (the last page's values are applied to a copy:
    /// built_), made again when an answer changes.
    std::optional<document::WorkholdingTemplate> base_;
    std::optional<document::WorkholdingTemplate> built_;
    std::map<std::string, cad::DatumGeometry> pickGeometry_; ///< By part, type and index.
    std::map<std::string, double> travel_;                   ///< Members' travel shown in the view.
    document::WorkholdingPackage result_;
    bool fitted_ = false;

    OcctView* view_ = nullptr;
    QListWidget* stepList_ = nullptr;
    QListWidget* parts_ = nullptr;
    QPushButton* removePart_ = nullptr;
    QPushButton* mergeParts_ = nullptr;
    QLabel* title_ = nullptr;
    QLabel* help_ = nullptr;
    QLabel* answer_ = nullptr;
    QLabel* problem_ = nullptr;
    QWidget* typePage_ = nullptr;
    QButtonGroup* types_ = nullptr;
    QWidget* finishPage_ = nullptr;
    QLineEdit* name_ = nullptr;
    QLineEdit* description_ = nullptr;
    QWidget* openingRow_ = nullptr;
    QDoubleSpinBox* minOpening_ = nullptr;
    QDoubleSpinBox* maxOpening_ = nullptr;
    QDoubleSpinBox* gripDepth_ = nullptr;
    QDoubleSpinBox* tenonDepth_ = nullptr;
    QDoubleSpinBox* gripRange_ = nullptr;
    QComboBox* anchor_ = nullptr;
    QSlider* opening_ = nullptr;
    QLabel* openingValue_ = nullptr;
    QCheckBox* trial_ = nullptr;
    QDoubleSpinBox* trialSize_[3] = {nullptr, nullptr, nullptr};
    QPushButton* back_ = nullptr;
    QPushButton* next_ = nullptr;
    QPushButton* skip_ = nullptr;
    QPushButton* finish_ = nullptr;
};

} // namespace ocam::gui
