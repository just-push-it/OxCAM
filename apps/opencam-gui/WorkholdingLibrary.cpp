// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "WorkholdingLibrary.hpp"

#include <ocam/foundation/Error.hpp>

#include <QSettings>
#include <QStandardPaths>
#include <algorithm>

namespace ocam::gui {
namespace {

std::filesystem::path& folderOverride() {
    static std::filesystem::path path;
    return path;
}

} // namespace

void WorkholdingLibrary::setFolderOverride(std::filesystem::path folder) {
    folderOverride() = std::move(folder);
}

std::filesystem::path WorkholdingLibrary::folder() {
    if (!folderOverride().empty()) {
        return folderOverride();
    }
    const QSettings settings;
    QString dir = settings.value(QStringLiteral("workholdingLibraryFolder")).toString();
    if (dir.isEmpty()) {
        dir = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation) +
              QStringLiteral("/workholding");
    }
    std::filesystem::path path(dir.toStdU16String());
    std::error_code ec;
    std::filesystem::create_directories(path, ec);
    return path;
}

std::vector<WorkholdingLibrary::Entry>
WorkholdingLibrary::load(std::vector<std::string>* problems) {
    std::vector<Entry> out;
    std::error_code ec;
    for (const auto& item : std::filesystem::directory_iterator(folder(), ec)) {
        if (!item.is_regular_file() || item.path().extension() != ".ocamhold") {
            continue;
        }
        try {
            out.push_back({document::loadWorkholdingFile(item.path()), item.path()});
        } catch (const Error& e) {
            if (problems) {
                problems->push_back(e.what());
            }
        }
    }
    std::sort(out.begin(), out.end(), [](const Entry& a, const Entry& b) {
        return a.package.workholding.name < b.package.workholding.name;
    });
    return out;
}

std::filesystem::path WorkholdingLibrary::save(const document::WorkholdingPackage& package) {
    if (const auto issues = package.workholding.validate(); !issues.empty()) {
        throw Error(ErrorCode::InvalidArgument, issues.front());
    }
    const std::filesystem::path path = folder() / (package.workholding.id.toString() + ".ocamhold");
    document::saveWorkholdingFile(path, package);
    return path;
}

bool WorkholdingLibrary::remove(const Uuid& id) {
    std::error_code ec;
    return std::filesystem::remove(folder() / (id.toString() + ".ocamhold"), ec);
}

} // namespace ocam::gui
