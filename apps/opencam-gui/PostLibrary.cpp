// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "PostLibrary.hpp"

#include <ocam/foundation/Error.hpp>
#include <ocam/foundation/Json.hpp>

#include <QSettings>
#include <QStandardPaths>
#include <algorithm>

namespace ocam::gui {

std::filesystem::path PostLibrary::folder() {
    const QSettings settings;
    QString dir = settings.value(QStringLiteral("postLibraryFolder")).toString();
    if (dir.isEmpty()) {
        dir = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation) +
              QStringLiteral("/posts");
    }
    std::filesystem::path path(dir.toStdU16String());
    std::error_code ec;
    std::filesystem::create_directories(path, ec);
    return path;
}

std::vector<PostLibrary::Entry> PostLibrary::load(std::vector<std::string>* problems) {
    std::vector<Entry> out;
    std::error_code ec;
    for (const auto& item : std::filesystem::directory_iterator(folder(), ec)) {
        if (!item.is_regular_file() || item.path().extension() != ".js") {
            continue;
        }
        try {
            const std::u8string name = item.path().filename().u8string();
            std::string source = readFileBytes(item.path());
            CancelToken cancel;
            post::PostScript post =
                post::PostScript::load(source, std::string(name.begin(), name.end()), cancel);
            out.push_back({std::move(post), std::move(source), item.path()});
        } catch (const Error& e) {
            if (problems) {
                problems->push_back(e.what());
            }
        }
    }
    std::sort(out.begin(), out.end(), [](const Entry& a, const Entry& b) {
        return a.post.manifest().name < b.post.manifest().name;
    });
    return out;
}

namespace {

QString defaultsKey(const std::string& postId) {
    return QStringLiteral("postDefaults/") + QString::fromStdString(postId);
}

} // namespace

Json PostLibrary::userDefaults(const std::string& postId) {
    const QString text = QSettings().value(defaultsKey(postId)).toString();
    if (text.isEmpty()) {
        return Json::object();
    }
    try {
        Json values = parseJson(text.toStdString(), "post defaults");
        return values.is_object() ? values : Json::object();
    } catch (const Error&) {
        return Json::object(); // unreadable: as if none were saved
    }
}

void PostLibrary::setUserDefaults(const std::string& postId, const Json& values) {
    QSettings().setValue(defaultsKey(postId), QString::fromStdString(dumpJson(values)));
}

} // namespace ocam::gui
