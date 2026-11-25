// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// OxCAM was called OxCAM before (ADR-0005). Its settings (preferences, recent projects, key
// bindings, the list of tool libraries) and its data folder (tool libraries, machines) were kept
// under that name; on the first start of OxCAM they are copied over.

#pragma once

namespace ocam::gui {

/// When OxCAM has no settings yet and OxCAM has, copies every OxCAM setting. With
/// `copyData`, also copies the files of OxCAM's data folder that OxCAM's does not have yet,
/// and settings naming files in OxCAM's data folder name the copies instead. OxCAM's own
/// settings and files stay as they are. Call once the application and organization names (and
/// any --settings-dir) are set. Returns whether settings were copied.
bool importOpenCamSettings(bool copyData);

} // namespace ocam::gui
