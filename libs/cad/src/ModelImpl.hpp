// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "ocam/cad/Model.hpp"

#include <Message_ProgressIndicator.hxx>
#include <Message_ProgressScope.hxx>
#include <NCollection_IndexedMap.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS_Shape.hxx>
#include <memory>
#include <mutex>

namespace ocam::cad {

struct Model::Impl {
    TopoDS_Shape shape;
    /// Model::info(), worked out once: a model's shape never changes, and its exact bounds,
    /// volume and area take long for large models.
    struct InfoCache {
        std::once_flag once;
        ModelInfo info;
    };
    std::shared_ptr<InfoCache> info = std::make_shared<InfoCache>();
};

using ShapeIndexMap = NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher>;

// Forwards OCCT progress to an ocam::ProgressSink and polls the CancelToken.
class ProgressAdapter : public Message_ProgressIndicator {
public:
    ProgressAdapter(CancelToken& cancel, ProgressSink& sink, double begin, double end)
        : cancel_(cancel), sink_(sink, begin, end) {}

    bool UserBreak() override { return cancel_.isCancelled(); }

    void Show(const Message_ProgressScope& /*scope*/, const bool /*isForce*/) override {
        sink_.report(GetPosition(), "Transferring STEP entities");
    }

private:
    CancelToken& cancel_;
    ProgressRange sink_;
};

// OCCT file APIs take UTF-8 narrow strings.
inline std::string utf8Path(const std::filesystem::path& path) {
    const std::u8string u8 = path.u8string();
    return {u8.begin(), u8.end()};
}

} // namespace ocam::cad
