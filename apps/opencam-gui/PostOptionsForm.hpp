// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The program options of a post as editors, each with where its value comes from (the setup,
// the machine, the user's defaults, the post). Used by the Post Process dialog and the machine
// library.

#pragma once

#include <ocam/foundation/Json.hpp>
#include <ocam/post/Engine.hpp>

#include <QString>
#include <QWidget>
#include <map>
#include <string>
#include <vector>

class QFormLayout;
class QLabel;

namespace ocam::gui {

class PostOptionsForm : public QWidget {
    Q_OBJECT

public:
    /// A source of values ({id: value}) and how the form names it ("this setup").
    struct Layer {
        QString name;
        QString tip;
        Json values;
    };

    explicit PostOptionsForm(QWidget* parent = nullptr);

    /// Shows the program options of `post` (none: empty), each with its value from the first
    /// layer that has one the post takes, else the post's default.
    void show(const post::PostScript* post, const std::vector<Layer>& layers);

    /// The values shown, every program option ({id: value}).
    const Json& values() const { return values_; }

    /// Marks every value as coming from `layer` (after the values were saved there).
    void markAll(const Layer& layer);

    /// Whether the post shown has per-operation options too.
    bool hasOperationOptions() const { return operationOptions_; }

signals:
    void changed();

private:
    void setSource(const std::string& id, const QString& name, const QString& tip);

    QFormLayout* form_ = nullptr;
    Json values_ = Json::object();
    std::map<std::string, QLabel*> sources_;
    bool operationOptions_ = false;
};

} // namespace ocam::gui
