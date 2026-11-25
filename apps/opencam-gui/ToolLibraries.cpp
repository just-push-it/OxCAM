// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ToolLibraries.hpp"

#include <ocam/foundation/Error.hpp>

#include <QSettings>
#include <QStandardPaths>
#include <QStringList>
#include <algorithm>
#include <cctype>
#include <map>
#include <mutex>

namespace ocam::gui {
namespace {

std::filesystem::path toPath(const QString& s) {
    return std::filesystem::path(s.toStdU16String());
}

QString fromPath(const std::filesystem::path& p) {
    return QString::fromStdU16String(p.u16string());
}

bool samePath(const std::filesystem::path& a, const std::filesystem::path& b) {
    std::error_code ec;
    if (std::filesystem::equivalent(a, b, ec)) {
        return true;
    }
    return a.lexically_normal() == b.lexically_normal();
}

/// Libraries read so far, with the file time they were read at.
struct CacheEntry {
    std::filesystem::file_time_type time;
    tooling::ToolLibrary library;
};
std::map<std::filesystem::path, CacheEntry>& cache() {
    static std::map<std::filesystem::path, CacheEntry> entries;
    return entries;
}
std::mutex& cacheMutex() {
    static std::mutex mutex;
    return mutex;
}

} // namespace

std::filesystem::path ToolLibraries::folder() {
    const std::filesystem::path path(
        (QStandardPaths::writableLocation(QStandardPaths::AppDataLocation) +
         QStringLiteral("/tool-libraries"))
            .toStdU16String());
    std::error_code ec;
    std::filesystem::create_directories(path, ec);
    return path;
}

std::vector<std::filesystem::path> ToolLibraries::files() {
    QSettings settings;
    QStringList list = settings.value(QStringLiteral("toolLibraries")).toStringList();
    // Earlier versions kept a single library.
    const QString legacy = settings.value(QStringLiteral("toolLibraryPath")).toString();
    if (!legacy.isEmpty()) {
        if (!list.contains(legacy)) {
            list.prepend(legacy);
        }
        settings.setValue(QStringLiteral("toolLibraries"), list);
        if (settings.value(QStringLiteral("defaultToolLibrary")).toString().isEmpty()) {
            settings.setValue(QStringLiteral("defaultToolLibrary"), legacy);
        }
        settings.remove(QStringLiteral("toolLibraryPath"));
    }
    std::vector<std::filesystem::path> out;
    for (const QString& s : list) {
        if (!s.isEmpty()) {
            out.push_back(toPath(s));
        }
    }
    return out;
}

void ToolLibraries::setFiles(const std::vector<std::filesystem::path>& files) {
    QStringList list;
    for (const auto& f : files) {
        list << fromPath(f);
    }
    QSettings().setValue(QStringLiteral("toolLibraries"), list);
}

void ToolLibraries::add(const std::filesystem::path& file) {
    auto list = files();
    if (std::none_of(list.begin(), list.end(),
                     [&](const std::filesystem::path& f) { return samePath(f, file); })) {
        list.push_back(file);
        setFiles(list);
    }
    if (!defaultFile()) {
        setDefaultFile(file);
    }
}

void ToolLibraries::remove(const std::filesystem::path& file) {
    auto list = files();
    std::erase_if(list, [&](const std::filesystem::path& f) { return samePath(f, file); });
    setFiles(list);
    const auto current = defaultFile();
    if (current && samePath(*current, file)) {
        setDefaultFile(list.empty() ? std::nullopt : std::optional(list.front()));
    }
}

std::optional<std::filesystem::path> ToolLibraries::defaultFile() {
    const auto list = files(); // takes over the legacy setting first
    const QString value = QSettings().value(QStringLiteral("defaultToolLibrary")).toString();
    if (!value.isEmpty()) {
        const auto path = toPath(value);
        if (std::any_of(list.begin(), list.end(),
                        [&](const std::filesystem::path& f) { return samePath(f, path); })) {
            return path;
        }
    }
    return list.empty() ? std::nullopt : std::optional(list.front());
}

void ToolLibraries::setDefaultFile(const std::optional<std::filesystem::path>& file) {
    QSettings().setValue(QStringLiteral("defaultToolLibrary"), file ? fromPath(*file) : QString());
}

std::vector<ToolLibraries::Entry> ToolLibraries::load(std::vector<std::string>* problems) {
    const auto def = defaultFile();
    std::vector<Entry> out;
    const std::scoped_lock lock(cacheMutex());
    for (const auto& file : files()) {
        std::error_code ec;
        const auto time = std::filesystem::last_write_time(file, ec);
        if (ec) {
            if (problems) {
                problems->push_back(fromPath(file).toStdString() + ": the file is missing");
            }
            continue;
        }
        auto it = cache().find(file);
        if (it == cache().end() || it->second.time != time) {
            try {
                cache()[file] = {time, tooling::loadToolLibrary(file)};
            } catch (const Error& e) {
                if (problems) {
                    problems->push_back(fromPath(file).toStdString() + ": " + e.what());
                }
                cache().erase(file);
                continue;
            }
            it = cache().find(file);
        }
        const bool isDefault = def && samePath(*def, file);
        Entry entry{file, it->second.library, isDefault};
        if (isDefault) {
            out.insert(out.begin(), std::move(entry));
        } else {
            out.push_back(std::move(entry));
        }
    }
    return out;
}

std::filesystem::path ToolLibraries::create(const std::string& name) {
    // A file name from the library name: letters, digits, '-' and '_'.
    std::string stem;
    for (const char c : name) {
        const auto u = static_cast<unsigned char>(c);
        stem += std::isalnum(u) || c == '-' || c == '_' ? c : '_';
    }
    if (stem.empty()) {
        stem = "tools";
    }
    const std::filesystem::path dir = folder();
    std::filesystem::path file = dir / (stem + ".ocamtools");
    for (int n = 2; std::filesystem::exists(file); ++n) {
        file = dir / (stem + "-" + std::to_string(n) + ".ocamtools");
    }
    tooling::ToolLibrary library;
    library.name = name;
    tooling::saveToolLibrary(library, file);
    add(file);
    return file;
}

} // namespace ocam::gui
