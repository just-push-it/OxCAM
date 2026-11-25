// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "HolderImportDialog.hpp"

#include "ToolProfileView.hpp"
#include "Units.hpp"

#include <ocam/foundation/Error.hpp>

#include <QComboBox>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>

namespace ocam::gui {

HolderImportDialog::HolderImportDialog(cad::Model model, const QString& name, QWidget* parent)
    : QDialog(parent), model_(std::move(model)), name_(name) {
    setWindowTitle(tr("Import holder - %1").arg(name));
    resize(620, 520);
    auto* root = new QHBoxLayout(this);
    auto* left = new QVBoxLayout;
    root->addLayout(left, 1);
    auto* form = new QFormLayout;
    left->addLayout(form);

    axis_ = new QComboBox;
    axis_->addItem(tr("Automatic"), static_cast<int>(cad::RevolveAxis::Auto));
    axis_->addItem(tr("Model X"), static_cast<int>(cad::RevolveAxis::X));
    axis_->addItem(tr("Model Y"), static_cast<int>(cad::RevolveAxis::Y));
    axis_->addItem(tr("Model Z"), static_cast<int>(cad::RevolveAxis::Z));
    axis_->setToolTip(tr("The model axis the holder turns about"));
    form->addRow(tr("Axis"), axis_);

    toolEnd_ = new QComboBox;
    toolEnd_->addItem(tr("Automatic"), static_cast<int>(cad::ToolEnd::Auto));
    toolEnd_->addItem(tr("Low end"), static_cast<int>(cad::ToolEnd::Low));
    toolEnd_->addItem(tr("High end"), static_cast<int>(cad::ToolEnd::High));
    toolEnd_->setToolTip(tr("The end of the model along the axis that the tool comes out of. "
                            "Automatic takes the end farther from the flange."));
    form->addRow(tr("Tool end"), toolEnd_);

    length_ = new LengthSpinBox(0.0, preferredUnit(), 0.0, 10000.0);
    length_->setToolTip(tr("From the tool end to the gauge line (the spindle face): the taper "
                           "above it sits in the spindle and is left out"));
    auto* lengthRow = new QHBoxLayout;
    lengthRow->addWidget(length_, 1);
    auto* suggested = new QPushButton(tr("Top of flange"));
    suggested->setToolTip(tr("The usual gauge line: where the widest part ends"));
    auto* whole = new QPushButton(tr("Whole model"));
    lengthRow->addWidget(suggested);
    lengthRow->addWidget(whole);
    form->addRow(tr("Gauge length"), lengthRow);

    info_ = new QLabel;
    info_->setWordWrap(true);
    left->addWidget(info_);
    left->addStretch();
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    left->addWidget(buttons);

    preview_ = new ToolProfileView;
    root->addWidget(preview_, 1);

    connect(axis_, &QComboBox::currentIndexChanged, this, [this] { recompute(false); });
    connect(toolEnd_, &QComboBox::currentIndexChanged, this, [this] { recompute(false); });
    connect(length_, &LengthSpinBox::mmChanged, this, [this] {
        if (!loading_) {
            recompute(true);
        }
    });
    connect(suggested, &QPushButton::clicked, this, [this] { recompute(false); });
    connect(whole, &QPushButton::clicked, this, [this] {
        const QSignalBlocker block(length_);
        length_->setValue(outline_.fullLength / (preferredUnit() == LengthUnit::Inch ? 25.4 : 1.0));
        recompute(true);
    });
    recompute(false);
}

void HolderImportDialog::recompute(bool keepLength) {
    cad::HolderOutlineOptions options;
    options.axis = static_cast<cad::RevolveAxis>(axis_->currentData().toInt());
    options.toolEnd = static_cast<cad::ToolEnd>(toolEnd_->currentData().toInt());
    options.length = keepLength ? length_->mm() : 0.0;
    try {
        outline_ = cad::holderOutline(model_, options);
    } catch (const Error& e) {
        info_->setText(QString::fromUtf8(e.what()));
        return;
    }
    if (!keepLength) {
        loading_ = true;
        length_->setValue(outline_.length / (preferredUnit() == LengthUnit::Inch ? 25.4 : 1.0));
        loading_ = false;
    }
    const LengthUnit unit = preferredUnit();
    const char* axisName = outline_.axis == cad::RevolveAxis::X   ? "X"
                           : outline_.axis == cad::RevolveAxis::Y ? "Y"
                                                                  : "Z";
    QString text =
        tr("The model is %1 long along its %2 axis; the tool comes out of its %3 end. The "
           "outline is %4 long (the top of the flange is at %5), up to Ø%6.")
            .arg(formatLength(outline_.fullLength, unit), QLatin1String(axisName),
                 outline_.toolEnd == cad::ToolEnd::Low ? tr("low") : tr("high"),
                 formatLength(outline_.length, unit), formatLength(outline_.suggestedLength, unit),
                 formatLength(2.0 * holder().maxRadius(), unit));
    if (outline_.asymmetry > 0.02) {
        text += QStringLiteral("<br><span style=\"color:#c0392b\">") +
                tr("The model is not round about this axis (its sides differ by %1%): choose "
                   "another axis.")
                    .arg(100.0 * outline_.asymmetry, 0, 'f', 0) +
                QStringLiteral("</span>");
    }
    info_->setText(text);
    preview_->setHolder(holder());
}

tooling::Holder HolderImportDialog::holder() const {
    tooling::Holder h;
    h.name = name_.toStdString();
    h.comment = "Imported from " + name_.toStdString();
    h.profile = outline_.profile;
    return h;
}

} // namespace ocam::gui
