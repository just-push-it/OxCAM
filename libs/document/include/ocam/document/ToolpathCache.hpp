// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Generated toolpaths by operation and input key (toolpathInputKey), shared by a document and
// its snapshots and saved in the project archive (cache/toolpaths.json), so reopening a project
// or rest machining after earlier operations does not generate them again. The key includes the
// program version and kToolpathRevision: toolpaths from a version that cuts differently simply
// no longer match and are generated again.

#pragma once

#include <ocam/foundation/Cancel.hpp>
#include <ocam/foundation/Json.hpp>
#include <ocam/foundation/Uuid.hpp>
#include <ocam/toolpath/Toolpath.hpp>

#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace ocam::document {

inline constexpr int kToolpathCacheVersion = 1;

class ToolpathCache {
public:
    using Path = std::shared_ptr<const toolpath::Toolpath>;

    /// The toolpath stored for the operation under this key, or nullptr.
    Path find(const Uuid& operationId, const std::string& key) const;
    /// Keeps a few keys per operation (the latest first), so undoing an edit finds the toolpath
    /// of the inputs before it.
    void store(const Uuid& operationId, const std::string& key, Path path);

    /// The stored toolpath, waiting first while another thread generates it (the GUI's
    /// background generation and a post, say), or nullptr when nobody does: the caller then
    /// generates it and must call finish() whatever happens. Throws OperationCancelled when
    /// `cancel` is requested while waiting.
    Path findOrBegin(const Uuid& operationId, const std::string& key, CancelToken& cancel);
    /// Ends a generation begun by findOrBegin, storing its toolpath (nullptr: it failed, and a
    /// waiting thread generates it itself).
    void finish(const Uuid& operationId, const std::string& key, Path path);
    std::size_t size() const;
    void clear();

    /// The entries whose key is the operation's current one (operation -> key), as saved in the
    /// project archive.
    Json toJson(const std::map<Uuid, std::string>& current) const;
    /// Adds the entries of a saved cache. A cache of another format version, or entries that do
    /// not read, are skipped (the toolpaths are generated again). Returns the entries added.
    std::size_t load(const Json& doc);

private:
    struct Entry {
        std::string key;
        Path path;
    };
    static constexpr std::size_t kPerOperation = 4;
    Path findLocked(const Uuid& operationId, const std::string& key) const;
    void storeLocked(const Uuid& operationId, const std::string& key, Path path);
    mutable std::mutex mutex_;
    std::condition_variable finished_;
    std::map<Uuid, std::vector<Entry>> entries_; ///< Latest first.
    std::set<std::pair<Uuid, std::string>> generating_;
};

} // namespace ocam::document
