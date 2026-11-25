// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "MachineLibrary.hpp"

#include <ocam/document/MachineImport.hpp>
#include <ocam/foundation/Error.hpp>
#include <ocam/machine/Model.hpp>

#include <QSettings>
#include <QStandardPaths>
#include <algorithm>
#include <regex>

namespace ocam::gui {

std::filesystem::path MachineLibrary::folder() {
    const QSettings settings;
    QString dir = settings.value(QStringLiteral("machineLibraryFolder")).toString();
    if (dir.isEmpty()) {
        dir = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation) +
              QStringLiteral("/machines");
    }
    std::filesystem::path path(dir.toStdU16String());
    std::error_code ec;
    std::filesystem::create_directories(path, ec);
    return path;
}

std::vector<MachineLibrary::Entry> MachineLibrary::load(std::vector<std::string>* problems) {
    std::vector<Entry> out;
    std::error_code ec;
    for (const auto& item : std::filesystem::directory_iterator(folder(), ec)) {
        if (!item.is_regular_file() || item.path().extension() != ".json") {
            continue;
        }
        try {
            out.push_back({document::loadMachineFile(item.path()), item.path()});
        } catch (const Error& e) {
            if (problems) {
                problems->push_back(e.what());
            }
        }
    }
    std::sort(out.begin(), out.end(),
              [](const Entry& a, const Entry& b) { return a.machine.name < b.machine.name; });
    return out;
}

std::filesystem::path MachineLibrary::save(const machine::Machine& m) {
    if (!isUsableId(m.id)) {
        throw Error(ErrorCode::InvalidArgument,
                    "the machine id must be lower case letters, digits, '.', '_' or '-', and "
                    "differ from the built-in machines");
    }
    const std::filesystem::path path = folder() / (m.id + ".json");
    // The revision counts the changes saved: one more than the file's when this differs from
    // it, the file's when it is the same (saved again unchanged).
    machine::Machine counted = m;
    std::error_code ec;
    if (std::filesystem::exists(path, ec)) {
        try {
            const machine::Machine before = document::loadMachineFile(path);
            counted.revision = before.revision;
            if (counted != before) {
                counted.revision = std::max(before.revision, m.revision) + 1;
            }
        } catch (const Error&) {
            // An unreadable file is replaced: counted from the machine's own revision.
            counted.revision = m.revision + 1;
        }
    } else {
        counted.revision = m.revision + 1;
    }
    // Round-trip through the schema so only loadable files are written.
    const Json doc = machine::toJson(counted);
    (void)machine::machineFromJson(doc, counted.id);
    machine::saveMachine(counted, path);
    return path;
}

bool MachineLibrary::remove(const std::string& id) {
    std::error_code ec;
    const std::filesystem::path path = folder() / (id + ".json");
    std::filesystem::remove_all(machine::meshFolder(path), ec);
    return std::filesystem::remove(path, ec);
}

bool MachineLibrary::isUsableId(const std::string& id) {
    static const std::regex pattern("^[a-z0-9][a-z0-9._-]*$");
    if (!std::regex_match(id, pattern)) {
        return false;
    }
    const auto builtins = machine::builtinMachines();
    return std::none_of(builtins.begin(), builtins.end(),
                        [&](const machine::Machine* b) { return b->id == id; }) &&
           id != "generic-3axis";
}

} // namespace ocam::gui
