// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Translations of the interface. The texts live in translations/opencam_<language>.ts (Qt
// Linguist files); the build compiles them and embeds them under :/i18n when it finds Qt's
// lrelease (docs/translations.md). Texts that come from the libraries are translated in their
// own contexts: "Strategies" (the strategies' names, descriptions, parameters and choices) and
// "Tooling" (tool type names). i18n/LibraryStrings.cpp lists them for lupdate.

#pragma once

#include <ocam/tooling/Tool.hpp>

#include <QString>
#include <QStringList>
#include <string>

namespace ocam::gui {

/// Installs the translation for the chosen language (the system's when none is chosen). Call
/// once, after the QApplication is made and before any window.
void installTranslations();

/// The languages there are translations for, as locale names ("de", "pt_BR").
QStringList availableLanguages();
/// The language chosen in the preferences (settings "language"); empty: the system's.
QString chosenLanguage();
void setChosenLanguage(const QString& name);

/// A text from the strategies (a name, a description, a parameter's label or help, a choice),
/// translated.
QString trStrategy(const std::string& text);
/// A tool type's name, translated.
QString trToolType(tooling::ToolType type);

/// Writes the source that lists the libraries' texts for lupdate (`--write-library-strings`).
/// Returns false when the file cannot be written.
bool writeLibraryStrings(const QString& path);

} // namespace ocam::gui
