// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The colours of the 3D view: the part, the stock, the toolpath move types, the selection and
// the background. The user sets them in Edit > Preferences > Colors (settings "colors/<key>").

#pragma once

#include <ocam/toolpath/Toolpath.hpp>

#include <QColor>
#include <Quantity_Color.hxx>
#include <vector>

namespace ocam::gui {

/// A colour as OpenCASCADE takes it (sRGB, as the user picked it).
inline Quantity_Color toQuantity(const QColor& c) {
    return Quantity_Color(static_cast<double>(c.redF()), static_cast<double>(c.greenF()),
                          static_cast<double>(c.blueF()), Quantity_TOC_sRGB);
}

struct ViewColors {
    QColor part{184, 191, 204};
    /// The part while transparent stock is shown over it: something that stands out from the
    /// stock, so what is left to cut is easy to see.
    QColor partThroughStock{124, 214, 46};
    QColor fixture{107, 128, 158}; ///< Vises, clamps and plates holding the part.
    QColor stock{196, 178, 150};   ///< Uncut material in the simulation and in-process stock.
    QColor stockBox{217, 166, 89}; ///< The setup's stock, shown see-through.
    QColor rapid{230, 77, 64};
    QColor cut{64, 140, 255};
    QColor plunge{51, 204, 77};
    QColor ramp{51, 230, 179};
    QColor lead{217, 89, 230}; ///< Lead-ins and lead-outs.
    QColor link{255, 179, 26};
    QColor probe{0, 210, 210}; ///< A spindle probe's moves.
    QColor selected{255, 115, 0};
    QColor hover{89, 204, 255};
    QColor backgroundTop{77, 84, 97};
    QColor backgroundBottom{20, 23, 28};

    /// The colour of a move type in the toolpaths.
    QColor forMove(toolpath::FeedClass feedClass) const;

    /// One colour as the preferences list it.
    struct Entry {
        const char* key;   ///< Settings key, "colors/<key>".
        const char* group; ///< Untranslated; QCoreApplication::translate("ViewColors", ...).
        const char* label; ///< Untranslated, in the same context.
        QColor ViewColors::*field;
    };
    static const std::vector<Entry>& entries();

    /// The saved colours (the defaults where none is saved).
    static ViewColors load();
    void save() const;

    bool operator==(const ViewColors&) const = default;
};

} // namespace ocam::gui
