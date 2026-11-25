// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/document/ToolpathCache.hpp"

#include <ocam/foundation/Error.hpp>

#include <algorithm>
#include <chrono>

namespace ocam::document {

ToolpathCache::Path ToolpathCache::find(const Uuid& operationId, const std::string& key) const {
    const std::scoped_lock lock(mutex_);
    return findLocked(operationId, key);
}

ToolpathCache::Path ToolpathCache::findLocked(const Uuid& operationId,
                                              const std::string& key) const {
    const auto it = entries_.find(operationId);
    if (it == entries_.end()) {
        return nullptr;
    }
    for (const Entry& e : it->second) {
        if (e.key == key) {
            return e.path;
        }
    }
    return nullptr;
}

void ToolpathCache::store(const Uuid& operationId, const std::string& key, Path path) {
    if (!path) {
        return;
    }
    const std::scoped_lock lock(mutex_);
    storeLocked(operationId, key, std::move(path));
}

void ToolpathCache::storeLocked(const Uuid& operationId, const std::string& key, Path path) {
    std::vector<Entry>& list = entries_[operationId];
    std::erase_if(list, [&](const Entry& e) { return e.key == key; });
    list.insert(list.begin(), Entry{key, std::move(path)});
    if (list.size() > kPerOperation) {
        list.resize(kPerOperation);
    }
}

ToolpathCache::Path ToolpathCache::findOrBegin(const Uuid& operationId, const std::string& key,
                                               CancelToken& cancel) {
    std::unique_lock lock(mutex_);
    const auto id = std::make_pair(operationId, key);
    for (;;) {
        if (Path path = findLocked(operationId, key)) {
            return path;
        }
        if (!generating_.contains(id)) {
            generating_.insert(id);
            return nullptr;
        }
        finished_.wait_for(lock, std::chrono::milliseconds(100));
        if (cancel.isCancelled()) {
            lock.unlock();
            cancel.throwIfCancelled();
        }
    }
}

void ToolpathCache::finish(const Uuid& operationId, const std::string& key, Path path) {
    {
        const std::scoped_lock lock(mutex_);
        generating_.erase(std::make_pair(operationId, key));
        if (path) {
            storeLocked(operationId, key, std::move(path));
        }
    }
    finished_.notify_all();
}

std::size_t ToolpathCache::size() const {
    const std::scoped_lock lock(mutex_);
    std::size_t n = 0;
    for (const auto& [id, list] : entries_) {
        n += list.size();
    }
    return n;
}

void ToolpathCache::clear() {
    const std::scoped_lock lock(mutex_);
    entries_.clear();
}

Json ToolpathCache::toJson(const std::map<Uuid, std::string>& current) const {
    Json entries = Json::array();
    for (const auto& [id, key] : current) {
        if (const Path path = find(id, key)) {
            entries.push_back({{"operation", id.toString()},
                               {"key", key},
                               {"toolpath", toolpath::toJson(*path)}});
        }
    }
    return {{"format", "oxcam.toolpath-cache"},
            {"version", kToolpathCacheVersion},
            {"entries", std::move(entries)}};
}

std::size_t ToolpathCache::load(const Json& doc) {
    // The name projects saved by OxCAM, formerly OpenCAM, used (ADR-0005).
    constexpr std::string_view kLegacyFormat = "opencam.toolpath-cache";
    const std::string format = doc.is_object() ? doc.value("format", "") : std::string();
    if ((format != "oxcam.toolpath-cache" && format != kLegacyFormat) ||
        doc.value("version", 0) != kToolpathCacheVersion || !doc.contains("entries") ||
        !doc.at("entries").is_array()) {
        return 0;
    }
    std::size_t added = 0;
    for (const Json& e : doc.at("entries")) {
        try {
            const auto id = Uuid::parse(e.at("operation").get<std::string>());
            if (!id) {
                continue;
            }
            store(*id, e.at("key").get<std::string>(),
                  std::make_shared<const toolpath::Toolpath>(
                      toolpath::toolpathFromJson(e.at("toolpath"))));
            ++added;
        } catch (const Json::exception&) {
            // generated again
        } catch (const Error&) {
            // generated again
        }
    }
    return added;
}

} // namespace ocam::document
