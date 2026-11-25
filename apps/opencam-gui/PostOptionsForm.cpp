// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "PostOptionsForm.hpp"

#include "PostChoices.hpp"

#include <ocam/foundation/Error.hpp>

#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>

namespace ocam::gui {

PostOptionsForm::PostOptionsForm(QWidget* parent) : QWidget(parent) {
    form_ = new QFormLayout(this);
    form_->setContentsMargins(0, 0, 0, 0);
}

void PostOptionsForm::show(const post::PostScript* post, const std::vector<Layer>& layers) {
    while (form_->rowCount() > 0) {
        form_->removeRow(0);
    }
    sources_.clear();
    values_ = Json::object();
    operationOptions_ = false;
    if (!post) {
        return;
    }
    // A value the post would refuse (another type, out of range, an option it no longer has) is
    // not used.
    auto usable = [&](const Json& values, const std::string& id) {
        if (!values.is_object() || !values.contains(id)) {
            return false;
        }
        try {
            (void)post->optionValues(Json{{id, values.at(id)}});
            return true;
        } catch (const Error&) {
            return false;
        }
    };
    for (const post::PostOption& option : post->options()) {
        if (option.scope != post::PostOption::Scope::Program) {
            operationOptions_ = true;
            continue;
        }
        const std::string id = option.id;
        QString source = tr("post default");
        QString tip = tr("The post's own default");
        values_[id] = option.defaultValue;
        for (const Layer& layer : layers) {
            if (usable(layer.values, id)) {
                values_[id] = layer.values.at(id);
                source = layer.name;
                tip = layer.tip;
                break;
            }
        }
        auto* row = new QWidget;
        auto* rowLayout = new QHBoxLayout(row);
        rowLayout->setContentsMargins(0, 0, 0, 0);
        rowLayout->addWidget(postOptionEditor(option, values_[id],
                                              [this, id](const Json& v) {
                                                  values_[id] = v;
                                                  setSource(id, tr("changed"),
                                                            tr("Changed here, not saved yet"));
                                                  emit changed();
                                              }),
                             1);
        auto* label = new QLabel;
        label->setStyleSheet(QStringLiteral("color: gray;"));
        label->setMinimumWidth(90);
        rowLayout->addWidget(label);
        sources_[id] = label;
        form_->addRow(QString::fromStdString(option.label), row);
        setSource(id, source, tip);
    }
    if (form_->rowCount() == 0) {
        form_->addRow(new QLabel(tr("This post has no program options.")));
    }
}

void PostOptionsForm::markAll(const Layer& layer) {
    for (const auto& [id, label] : sources_) {
        label->setText(layer.name);
        label->setToolTip(layer.tip);
    }
}

void PostOptionsForm::setSource(const std::string& id, const QString& name, const QString& tip) {
    if (const auto it = sources_.find(id); it != sources_.end()) {
        it->second->setText(name);
        it->second->setToolTip(tip);
    }
}

} // namespace ocam::gui
