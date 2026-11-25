// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Tool solid for simulation, derived from the canonical revolved profile (tooling::toolProfile)
// plus the holder outline. Heights are measured from the tool tip along the tool axis.

#pragma once

#include "ocam/sim/Dexel.hpp"

#include <ocam/tooling/Profile.hpp>
#include <ocam/tooling/Tool.hpp>

#include <array>
#include <vector>

namespace ocam::sim {

/// Up to four height intervals (relative to the tip) where the cutting part of the tool covers
/// a given radius.
struct HeightSpans {
    std::array<Interval, 4> spans{};
    int count = 0;
};

/// A convex slice of the tool solid (phase 9, tilted sweeps): heights [h0, h1] above the tip,
/// with the radius growing linearly from r0 to r1 (a frustum), or, when `sphere` > 0, the zone
/// of the sphere of that radius centred on the axis at height `centre`.
struct SolidPiece {
    double h0 = 0.0;
    double h1 = 0.0;
    double r0 = 0.0;
    double r1 = 0.0;
    double sphere = 0.0;
    double centre = 0.0;
};

/// A cylindrical slice of the holder: heights [h0, h1] above the tip, radius `radius`.
struct HolderBand {
    double h0 = 0.0;
    double h1 = 0.0;
    double radius = 0.0;
};

class ToolShape {
public:
    ToolShape() = default;

    /// Builds the shape of a rotating tool with an optional holder. The visible tool length is
    /// the stickout when set, otherwise the overall length. Throws like tooling::toolProfile.
    static ToolShape fromTool(const tooling::Tool& tool, const tooling::Holder* holder = nullptr);
    static ToolShape fromProfile(const tooling::ToolProfile& profile, double stickout,
                                 const tooling::Holder* holder = nullptr);

    double cutRadius() const noexcept { return cutRadius_; }
    double cuttingLength() const noexcept { return cuttingLength_; }
    /// Largest radius of the non-cutting tool body between the cutting length and the holder.
    double shankRadius() const noexcept { return shankRadius_; }
    /// The non-cutting body as bands of one radius each (the largest over the band): a neck
    /// narrower than the shank is its own band, so it can pass where the shank cannot.
    const std::vector<HolderBand>& shankBands() const noexcept { return shankBands_; }
    /// Height of the holder face above the tip.
    double visibleLength() const noexcept { return visibleLength_; }
    const std::vector<HolderBand>& holder() const noexcept { return holder_; }
    double holderRadius() const noexcept { return holderRadius_; }

    /// Heights (above the tip) where the cutting part reaches radius `d` from the axis.
    HeightSpans spansAt(double d) const noexcept;
    /// Largest cutting radius at any height in [h0, h1] (0 when the range misses the cutter).
    double maxRadiusIn(double h0, double h1) const noexcept;
    /// True when every radius is covered by at most one height span (convex lower and upper
    /// envelopes: end mills, drills, chamfer mills; not slot cutters or lollipops).
    bool singleSpan() const noexcept { return singleSpan_; }

    const tooling::ToolProfile& profile() const noexcept { return profile_; }

    /// The solid as convex slices: the cutting part (below the cutting length), the rest of the
    /// tool body up to the holder face, and the holder bands. Used for tilted tools.
    const std::vector<SolidPiece>& cutterPieces() const noexcept { return cutterPieces_; }
    const std::vector<SolidPiece>& shankPieces() const noexcept { return shankPieces_; }
    const std::vector<SolidPiece>& holderPieces() const noexcept { return holderPieces_; }

private:
    tooling::ToolProfile profile_;
    double cutRadius_ = 0.0;
    double cuttingLength_ = 0.0;
    double shankRadius_ = 0.0;
    double visibleLength_ = 0.0;
    double holderRadius_ = 0.0;
    bool singleSpan_ = true;
    std::vector<HolderBand> holder_;
    std::vector<HolderBand> shankBands_;
    std::vector<SolidPiece> cutterPieces_;
    std::vector<SolidPiece> shankPieces_;
    std::vector<SolidPiece> holderPieces_;
    std::vector<HeightSpans> spanTable_; ///< Samples at d = k * cutRadius / (size - 1).
    std::vector<float> radiusTable_;     ///< Radius at the centres of equal height cells.
};

} // namespace ocam::sim
