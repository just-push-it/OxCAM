// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "HolderEditorCheck.hpp"

#include "ToolLibraryDialog.hpp"

#include <ocam/foundation/Units.hpp>
#include <ocam/tooling/ToolLibrary.hpp>
#include <ocam/tooling/TurningHolder.hpp>

#include <QApplication>
#include <QComboBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QTableWidget>
#include <cstdio>

namespace ocam::gui {
namespace {

/// The dialog, its widgets by name, and the checks.
class Check {
public:
    Check(ToolLibraryDialog& dialog, QString directory)
        : dialog_(dialog), directory_(std::move(directory)) {}

    template <class Widget> Widget* widget(const char* name) {
        auto* found = dialog_.findChild<Widget*>(QString::fromLatin1(name));
        if (!found) {
            fail(std::string("no widget ") + name);
        }
        return found;
    }

    /// Chooses the combo's item whose text starts with `text`, as picking it from the list does.
    void choose(const char* combo, const QString& text) {
        auto* c = widget<QComboBox>(combo);
        for (int k = 0; c && k < c->count(); ++k) {
            if (c->itemText(k).startsWith(text)) {
                c->setCurrentIndex(k);
                QApplication::processEvents();
                return;
            }
        }
        fail(std::string(combo) + " has no item " + text.toStdString());
    }

    bool offers(const char* combo, const QString& text) {
        auto* c = widget<QComboBox>(combo);
        for (int k = 0; c && k < c->count(); ++k) {
            if (c->itemText(k).startsWith(text)) {
                return true;
            }
        }
        return false;
    }

    void that(bool ok, const std::string& what) {
        if (!ok) {
            fail(what);
        }
    }

    void shot(const QString& name) {
        QApplication::processEvents();
        if (!dialog_.grab().save(QDir(directory_).filePath(name + QStringLiteral(".png")))) {
            fail("could not save " + name.toStdString());
        }
    }

    int failures() const { return failures_; }

private:
    void fail(const std::string& what) {
        std::fprintf(stderr, "holder editor: %s\n", what.c_str());
        ++failures_;
    }

    ToolLibraryDialog& dialog_;
    QString directory_;
    int failures_ = 0;
};

tooling::Holder iso(const char* code) {
    return tooling::makeTurningHolder(*tooling::parseDesignation(code));
}

/// What a tool's holder list offers follows its insert, and a tool that stops fitting its holder
/// keeps it and says so.
void checkFilter(Check& c, ToolLibraryDialog& dialog) {
    const auto tool = [&]() -> const tooling::Tool& {
        return dialog.library().tools.front();
    };
    dialog.selectToolNumber(1);
    QApplication::processEvents();
    // A new insert (C, 12.7, 0 degrees, no locking style set), as an older library has it.
    c.that(c.offers("toolHolder", QStringLiteral("PCLNR 2525M 12")), "PCLNR is offered");
    c.that(c.offers("toolHolder", QStringLiteral("SCLNR 2525M 12")), "SCLNR is offered, not set");
    c.that(c.offers("toolHolder", QStringLiteral("Turning shank")), "the plain shank is offered");
    c.that(!c.offers("toolHolder", QStringLiteral("SDJCR")), "SDJCR is hidden");
    c.that(!c.offers("toolHolder", QStringLiteral("PDJNR")), "PDJNR is hidden");
    c.that(!c.offers("toolHolder", QStringLiteral("Mill holder")), "the milling holder is hidden");
    auto* note = c.widget<QLabel>("toolHolderNote");
    c.that(note && note->text().contains(QStringLiteral("2 holder")), "two holders are noted");
    c.that(note && note->toolTip().contains(QStringLiteral("SDJCR")), "the tooltip says which");
    c.shot(QStringLiteral("01_new_insert"));

    // Another shape: another list.
    c.choose("insertShape", QStringLiteral("D"));
    c.that(tool().insert.shape == "D", "the shape changed");
    c.that(c.offers("toolHolder", QStringLiteral("PDJNR")), "PDJNR is offered for a D insert");
    c.that(!c.offers("toolHolder", QStringLiteral("PCLNR")), "PCLNR is hidden for a D insert");
    c.shot(QStringLiteral("02_d_insert"));
    c.choose("insertShape", QStringLiteral("C"));

    // The locking style narrows it.
    c.choose("toolLocking", QStringLiteral("Countersunk"));
    c.that(tool().insert.locking == tooling::InsertLocking::CountersunkHole, "locking is set");
    c.that(c.offers("toolHolder", QStringLiteral("SCLNR")), "a screw holder takes it");
    c.that(!c.offers("toolHolder", QStringLiteral("PCLNR")), "a lever holder does not");
    c.that(c.offers("toolHolder", QStringLiteral("Turning shank")), "a plain holder says nothing");
    c.shot(QStringLiteral("03_countersunk"));
    c.choose("toolLocking", QStringLiteral("Not set"));

    // Picking a holder fits the insert to it.
    c.choose("toolHolder", QStringLiteral("PCLNR 2525M 12"));
    c.that(tool().holderId.has_value(), "the holder is taken");
    c.that(std::abs(radToDeg(tool().insert.cuttingEdgeAngle) - 95.0) < 1e-9,
           "the holder sets the cutting edge angle");
    const Uuid held = *tool().holderId;
    const tooling::TurningInsert fitted = tool().insert;

    // The insert stops fitting: nothing changes but the locking style, and the editor says so.
    c.choose("toolLocking", QStringLiteral("Countersunk"));
    c.that(tool().holderId == held, "the tool keeps its holder");
    tooling::TurningInsert expected = fitted;
    expected.locking = tooling::InsertLocking::CountersunkHole;
    c.that(tool().insert == expected, "the rest of the insert stays");
    c.that(c.offers("toolHolder", QStringLiteral("PCLNR 2525M 12 (does not fit)")),
           "the list says it does not fit");
    c.that(note && note->text().contains(QStringLiteral("keeps its holder")), "the note says so");
    c.shot(QStringLiteral("04_no_longer_fits"));

    // Editing something else leaves it alone.
    auto* nose = c.widget<QDoubleSpinBox>("noseRadius");
    if (nose) {
        nose->setValue(1.2);
    }
    QApplication::processEvents();
    c.that(tool().holderId == held, "still its holder after another edit");
    c.that(std::abs(tool().insert.noseRadius - 1.2) < 1e-9, "the nose radius is edited");
    c.that(tool().insert.cuttingEdgeAngle == fitted.cuttingEdgeAngle, "the angle stays exact");

    // Back to fitting.
    c.choose("toolLocking", QStringLiteral("Cylindrical"));
    c.that(!c.offers("toolHolder", QStringLiteral("PCLNR 2525M 12 (does not fit)")),
           "it fits again");
    c.that(note && note->text().indexOf(QStringLiteral("keeps its holder")) < 0,
           "the note is gone");
    c.shot(QStringLiteral("05_fits_again"));

    // A grooving tool: plain holders, as before, and no locking style.
    dialog.selectToolNumber(2);
    QApplication::processEvents();
    c.that(c.offers("toolHolder", QStringLiteral("Turning shank")), "grooving: the plain shank");
    c.that(!c.offers("toolHolder", QStringLiteral("PCLNR")), "grooving: no ISO holder");
    c.shot(QStringLiteral("06_grooving"));
}

/// Custom values in the holder's letters: a value field shows with "custom", starts from what the
/// letter stood for, and what it holds reaches the holder, the tools in it and the file.
void checkCustom(Check& c, ToolLibraryDialog& dialog) {
    const auto holder = [&]() -> const tooling::Holder& {
        return dialog.library().holders[6];
    };
    const auto tool = [&]() -> const tooling::Tool& {
        return dialog.library().tools[2];
    };
    dialog.showHolderTab();
    if (auto* table = c.widget<QTableWidget>("holderTable")) {
        table->setCurrentCell(6, 0);
    }
    QApplication::processEvents();
    auto* styleAngle = c.widget<QDoubleSpinBox>("isoStyleAngle");
    auto* clearanceAngle = c.widget<QDoubleSpinBox>("isoClearanceAngle");
    auto* lengthValue = c.widget<QDoubleSpinBox>("isoLengthValue");
    if (!styleAngle || !clearanceAngle || !lengthValue) {
        return;
    }
    c.that(holder().iso->style == 'L', "the holder starts as a standard one");
    c.that(!styleAngle->isVisible() && !clearanceAngle->isVisible() && !lengthValue->isVisible(),
           "no custom fields for standard letters");
    c.shot(QStringLiteral("07_standard_letters"));

    c.choose("isoStyle", QStringLiteral("X"));
    c.that(styleAngle->isVisible(), "the style's field shows");
    c.that(holder().iso->customStyle == 95.0, "the style starts from the letter's angle");
    styleAngle->setValue(70.0);
    QApplication::processEvents();
    c.that(holder().iso->customStyle == 70.0, "the style's value reaches the holder");
    c.that(std::abs(radToDeg(tool().insert.cuttingEdgeAngle) - 70.0) < 1e-9,
           "the tool in it takes the angle");

    c.choose("isoClearance", QStringLiteral("O"));
    c.that(clearanceAngle->isVisible(), "the clearance's field shows");
    c.that(holder().iso->customClearance == 0.0, "the clearance starts from the letter's");
    clearanceAngle->setValue(11.0);
    QApplication::processEvents();
    c.that(std::abs(radToDeg(tool().insert.reliefAngle) - 11.0) < 1e-9,
           "the tool in it takes the relief");

    c.choose("isoLength", QStringLiteral("X"));
    c.that(lengthValue->isVisible(), "the length's field shows");
    c.that(holder().iso->customLength == 125.0, "the length starts from the letter's");
    lengthValue->setValue(175.0);
    QApplication::processEvents();
    c.that(holder().length() == 175.0, "the length's value reaches the holder");
    c.that(tooling::designation(*holder().iso) == "PCXOR 2020X 12", "the designation says custom");
    c.that(holder().validate().empty(), "it is a valid holder");
    c.shot(QStringLiteral("08_custom_values"));

    // Typing the designation keeps the values that it cannot say.
    if (auto* line = c.widget<QLineEdit>("isoDesignation")) {
        line->setText(QStringLiteral("PCXOR 2525X 12"));
        Q_EMIT line->editingFinished();
    }
    QApplication::processEvents();
    c.that(holder().iso->height == 25.0, "the designation sets the height");
    c.that(holder().iso->customStyle == 70.0, "and keeps the custom style");
    c.that(holder().iso->customLength == 175.0, "and the custom length");

    // Files keep them.
    const tooling::ToolLibrary back =
        tooling::toolLibraryFromJson(tooling::toJson(dialog.library()));
    c.that(back.holders[6].iso == holder().iso, "a file keeps the custom values");

    // A value that cannot work is said, not changed.
    styleAngle->setValue(170.0);
    QApplication::processEvents();
    c.that(holder().iso->customStyle == 170.0, "the value stays as typed");
    c.that(!holder().validate().empty(), "and the holder has a problem");
    c.shot(QStringLiteral("09_custom_problem"));

    // A letter again: the value goes.
    c.choose("isoStyle", QStringLiteral("L"));
    c.that(!holder().iso->customStyle.has_value(), "a letter drops the custom style");
    c.that(!styleAngle->isVisible(), "and its field");
    c.that(std::abs(radToDeg(tool().insert.cuttingEdgeAngle) - 95.0) < 1e-9, "the tool follows");
}

/// The field for dimension f says what it is, for a shank and for a bar, ISO and plain.
void checkDimensionF(Check& c) {
    auto* table = c.widget<QTableWidget>("holderTable");
    if (!table) {
        return;
    }
    const auto says = [&](const char* name, const char* text) {
        auto* label = c.widget<QLabel>(name);
        c.that(label && label->text().contains(QString::fromLatin1(text)),
               std::string(name) + " says " + text);
    };
    table->setCurrentCell(1, 0); // an ISO shank that no tool uses
    QApplication::processEvents();
    says("isoFLabel", "back of shank");
    c.shot(QStringLiteral("10_f_iso_shank"));
    c.choose("isoType", QStringLiteral("Boring"));
    says("isoFLabel", "bar's axis");
    c.shot(QStringLiteral("11_f_iso_bar"));
    table->setCurrentCell(4, 0); // a plain shank
    QApplication::processEvents();
    says("stickFLabel", "back of shank");
    c.shot(QStringLiteral("12_f_plain_shank"));
    c.choose("stickType", QStringLiteral("Bar"));
    says("stickFLabel", "bar's axis");
    c.shot(QStringLiteral("13_f_plain_bar"));
}

} // namespace

int checkHolderEditor(const QString& directory) {
    QDir().mkpath(directory);
    tooling::ToolLibrary lib;
    tooling::Holder milling;
    milling.kind = tooling::HolderKind::Milling;
    milling.name = "Mill holder";
    milling.profile =
        geom2d::Polyline::fromPoints({Vec2(0, 0), Vec2(20, 0), Vec2(20, 50), Vec2(0, 50)}, false);
    lib.holders = {iso("PCLNR 2525M 12"), iso("SCLNR 2525M 12"),         iso("SDJCR 1212F 07"),
                   iso("PDJNR 2525M 15"), tooling::makeTurningStick({}), milling,
                   iso("PCLNR 2020K 12")};
    lib.tools = {tooling::makeDefaultTool(tooling::ToolType::TurningInsert, 1),
                 tooling::makeDefaultTool(tooling::ToolType::GroovingInsert, 2),
                 tooling::makeDefaultTool(tooling::ToolType::TurningInsert, 3)};
    lib.tools[2].holderId = lib.holders[6].id;
    tooling::fitToHolder(lib.tools[2], lib.holders[6]);
    ToolLibraryDialog dialog(lib, ToolLibraryDialog::Mode::LibraryFile);
    dialog.show();
    QApplication::processEvents();
    Check check(dialog, directory);
    checkFilter(check, dialog);
    checkCustom(check, dialog);
    checkDimensionF(check);
    std::fprintf(stderr, "holder editor: %d check(s) failed\n", check.failures());
    return check.failures();
}

} // namespace ocam::gui
