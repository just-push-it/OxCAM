// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/sim/StockCodec.hpp"

#include <ocam/foundation/Error.hpp>

#include <unordered_map>

namespace ocam::sim {

struct StockCodec {
    static constexpr std::uint32_t kMagic = 0x4b545358; // "XSTK"
    static constexpr std::uint32_t kVersion = 1;
    static constexpr std::uint32_t kNoTile = 0xffffffffU;

    static void write(ByteWriter& w, const std::vector<const Stock*>& stocks) {
        w.u32(kMagic);
        w.u32(kVersion);
        std::unordered_map<const DexelGrid::Tile*, std::uint32_t> ids;
        std::vector<const DexelGrid::Tile*> tiles;
        w.u64(stocks.size());
        for (const Stock* s : stocks) {
            w.vec3(s->bounds_.min);
            w.vec3(s->bounds_.max);
            w.f64(s->resolution_);
            w.boolean(s->triDexel_);
            for (const DexelGrid& g : s->grids_) {
                w.u32(static_cast<std::uint32_t>(g.axis_));
                w.f64(g.u0_);
                w.f64(g.v0_);
                w.f64(g.h_);
                w.u32(static_cast<std::uint32_t>(g.nu_));
                w.u32(static_cast<std::uint32_t>(g.nv_));
                w.u32(static_cast<std::uint32_t>(g.tilesU_));
                w.u32(static_cast<std::uint32_t>(g.tilesV_));
                w.u64(g.tiles_.size());
                for (const auto& tile : g.tiles_) {
                    if (!tile) {
                        w.u32(kNoTile);
                        continue;
                    }
                    const auto [it, added] =
                        ids.emplace(tile.get(), static_cast<std::uint32_t>(tiles.size()));
                    if (added) {
                        tiles.push_back(tile.get());
                    }
                    w.u32(it->second);
                }
            }
        }
        w.u64(tiles.size());
        for (const DexelGrid::Tile* tile : tiles) {
            for (const IntervalList& ray : tile->rays) {
                w.array(ray);
            }
        }
    }

    static std::vector<Stock> read(ByteReader& r) {
        if (r.u32() != kMagic || r.u32() != kVersion) {
            throw Error(ErrorCode::Parse, "not a stock encoding of this version");
        }
        const std::size_t count = r.count(1);
        std::vector<Stock> stocks(count);
        std::vector<std::vector<std::uint32_t>> refs; // per grid, in order
        for (Stock& s : stocks) {
            s.bounds_.min = r.vec3();
            s.bounds_.max = r.vec3();
            s.resolution_ = r.f64();
            s.triDexel_ = r.boolean();
            for (DexelGrid& g : s.grids_) {
                g.axis_ = static_cast<int>(r.u32());
                g.u0_ = r.f64();
                g.v0_ = r.f64();
                g.h_ = r.f64();
                g.nu_ = static_cast<int>(r.u32());
                g.nv_ = static_cast<int>(r.u32());
                g.tilesU_ = static_cast<int>(r.u32());
                g.tilesV_ = static_cast<int>(r.u32());
                const std::size_t n = r.count(4);
                if (g.axis_ < 0 || g.axis_ > 2 || g.nu_ < 0 || g.nv_ < 0 ||
                    n !=
                        static_cast<std::size_t>(g.tilesU_) * static_cast<std::size_t>(g.tilesV_)) {
                    throw Error(ErrorCode::Parse, "damaged stock grid");
                }
                std::vector<std::uint32_t> ids(n);
                for (std::uint32_t& id : ids) {
                    id = r.u32();
                }
                refs.push_back(std::move(ids));
            }
        }
        const std::size_t tileCount = r.count(1);
        std::vector<std::shared_ptr<DexelGrid::Tile>> tiles(tileCount);
        for (auto& tile : tiles) {
            tile = std::make_shared<DexelGrid::Tile>();
            for (IntervalList& ray : tile->rays) {
                ray = r.array<Interval>();
            }
        }
        std::size_t gridIndex = 0;
        for (Stock& s : stocks) {
            for (DexelGrid& g : s.grids_) {
                const auto& ids = refs[gridIndex++];
                g.tiles_.resize(ids.size());
                for (std::size_t k = 0; k < ids.size(); ++k) {
                    if (ids[k] == kNoTile) {
                        continue;
                    }
                    if (ids[k] >= tiles.size()) {
                        throw Error(ErrorCode::Parse, "damaged stock tile reference");
                    }
                    g.tiles_[k] = tiles[ids[k]];
                }
            }
        }
        return stocks;
    }
};

void writeStocks(ByteWriter& w, const std::vector<const Stock*>& stocks) {
    StockCodec::write(w, stocks);
}

std::vector<Stock> readStocks(ByteReader& r) {
    return StockCodec::read(r);
}

} // namespace ocam::sim
