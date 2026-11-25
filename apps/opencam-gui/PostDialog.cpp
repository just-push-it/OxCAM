// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "PostDialog.hpp"

#include <ocam/foundation/Error.hpp>
#include <ocam/foundation/Json.hpp>

#include <QDialogButtonBox>
#include <QFileDialog>
#include <QFontDatabase>
#include <QLabel>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <filesystem>

namespace ocam::gui {

PostDialog::PostDialog(const QString& gcode, const QStringList& warnings,
                       const QString& suggestedName, const QString& extension, QWidget* parent)
    : QDialog(parent), gcode_(gcode), suggestedName_(suggestedName), extension_(extension) {
    setWindowTitle(tr("Posted program - %1").arg(suggestedName));
    resize(820, 700);
    auto* layout = new QVBoxLayout(this);

    auto* summary =
        new QLabel(tr("%n line(s).", nullptr, static_cast<int>(gcode.count(QLatin1Char('\n')))));
    layout->addWidget(summary);
    if (!warnings.isEmpty()) {
        auto* list = new QLabel(QStringLiteral("• ") + warnings.join(QStringLiteral("\n• ")));
        list->setWordWrap(true);
        list->setStyleSheet(QStringLiteral("color: #d68910;"));
        layout->addWidget(list);
    }

    auto* text = new QPlainTextEdit(gcode);
    text->setReadOnly(true);
    text->setLineWrapMode(QPlainTextEdit::NoWrap);
    text->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
    layout->addWidget(text, 1);

    auto* note =
        new QLabel(tr("Verify every program on the machine (dry run, single block, reduced rapids) "
                      "before cutting. OxCAM is pre-release software."));
    note->setWordWrap(true);
    layout->addWidget(note);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Close);
    QPushButton* saveButton = buttons->addButton(tr("Save As..."), QDialogButtonBox::AcceptRole);
    connect(saveButton, &QPushButton::clicked, this, &PostDialog::save);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    layout->addWidget(buttons);
}

void PostDialog::save() {
    const QString path = QFileDialog::getSaveFileName(
        this, tr("Save G-code"), suggestedName_ + QLatin1Char('.') + extension_,
        tr("G-code (*.%1 *.nc *.tap *.txt);;All files (*)").arg(extension_));
    if (path.isEmpty()) {
        return;
    }
    try {
        writeFileAtomic(std::filesystem::path(path.toStdU16String()), gcode_.toStdString());
        accept();
    } catch (const Error& e) {
        QMessageBox::warning(this, tr("Save G-code"), QString::fromUtf8(e.what()));
    }
}

} // namespace ocam::gui
