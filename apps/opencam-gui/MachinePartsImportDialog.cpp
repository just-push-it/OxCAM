// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "MachinePartsImportDialog.hpp"

#include <ocam/foundation/Error.hpp>

#include <QApplication>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QEventLoop>
#include <QFileInfo>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QInputDialog>
#include <QLabel>
#include <QMessageBox>
#include <QProgressDialog>
#include <QPushButton>
#include <QSettings>
#include <QTableWidget>
#include <QThread>
#include <QTimer>
#include <QVBoxLayout>
#include <algorithm>
#include <atomic>
#include <exception>
#include <mutex>
#include <set>

namespace ocam::gui {
namespace {

enum Column { kUse, kName, kAssembly, kFile, kSolids, kGroup };

QString qs(const std::string& s) {
    return QString::fromStdString(s);
}

/// Progress reported from the worker thread, read by the dialog's timer.
class SharedProgress final : public ProgressSink {
public:
    void report(double fraction, std::string_view stage) override {
        fraction_.store(fraction);
        const std::lock_guard lock(mutex_);
        stage_ = std::string(stage);
    }
    double fraction() const { return fraction_.load(); }
    std::string stage() const {
        const std::lock_guard lock(mutex_);
        return stage_;
    }

private:
    std::atomic<double> fraction_{0.0};
    mutable std::mutex mutex_;
    std::string stage_;
};

} // namespace

bool runWithProgress(QWidget* parent, const QString& title,
                     const std::function<void(CancelToken&, ProgressSink&)>& work) {
    CancelToken cancel;
    SharedProgress progress;
    std::exception_ptr failure;
    QProgressDialog dialog(title, MachinePartsImportDialog::tr("Cancel"), 0, 1000, parent);
    dialog.setWindowTitle(title);
    dialog.setWindowModality(Qt::WindowModal);
    dialog.setMinimumDuration(300);
    dialog.setAutoClose(false);
    dialog.setAutoReset(false);
    QThread* thread = QThread::create([&] {
        try {
            work(cancel, progress);
        } catch (...) {
            failure = std::current_exception();
        }
    });
    QEventLoop loop;
    QObject::connect(thread, &QThread::finished, &loop, &QEventLoop::quit);
    QObject::connect(&dialog, &QProgressDialog::canceled, [&] { cancel.requestCancel(); });
    QTimer timer;
    timer.setInterval(50);
    QObject::connect(&timer, &QTimer::timeout, [&] {
        dialog.setValue(static_cast<int>(std::clamp(progress.fraction(), 0.0, 1.0) * 1000.0));
        const std::string stage = progress.stage();
        if (!stage.empty()) {
            dialog.setLabelText(title + QStringLiteral("\n") + qs(stage));
        }
    });
    QApplication::setOverrideCursor(Qt::BusyCursor);
    thread->start();
    timer.start();
    loop.exec();
    thread->wait();
    delete thread;
    QApplication::restoreOverrideCursor();
    if (failure) {
        try {
            std::rethrow_exception(failure);
        } catch (const OperationCancelled&) {
            return false;
        } catch (const std::exception& e) {
            QMessageBox::warning(parent, title, QString::fromUtf8(e.what()));
            return false;
        }
    }
    return !cancel.isCancelled();
}

MachinePartsImportDialog::MachinePartsImportDialog(std::vector<document::ImportedPart> parts,
                                                   QWidget* parent, bool solids)
    : QDialog(parent), parts_(std::move(parts)) {
    setWindowTitle(solids ? tr("Import workholding parts") : tr("Import machine parts"));
    resize(900, 640);
    auto* layout = new QVBoxLayout(this);
    std::set<std::string> files;
    for (const auto& p : parts_) {
        files.insert(p.file);
    }
    auto* intro = new QLabel(
        tr("Bodies found: %1, in %2. Each becomes a part of its own, in its place in the file "
           "(the file's origin is the machine's), to be carried by the base or an axis. Parts "
           "that move together can also be merged into one: by subassembly, or select rows and "
           "merge them; untick bodies to leave them out.")
            .arg(parts_.size())
            .arg(QStringList([&] {
                     QStringList l;
                     for (const auto& f : files) {
                         l << qs(f);
                     }
                     return l;
                 }())
                     .join(QStringLiteral(", "))));
    intro->setWordWrap(true);
    if (solids) {
        intro->setText(tr("Bodies found: %1. Each becomes a part of its own, in its place in the "
                          "file, to sit on the fixture's base or on a moving member (a jaw, a "
                          "clamp). Parts that move together can be merged: by subassembly, or "
                          "select rows and merge them; untick bodies to leave them out.")
                           .arg(parts_.size()));
    }
    layout->addWidget(intro);

    auto* options = new QFormLayout;
    level_ = new QComboBox;
    level_->addItem(tr("Each body on its own"), 0);
    const int depth = document::assemblyDepth(parts_);
    for (int k = 1; k <= depth; ++k) {
        level_->addItem(k == 1 ? tr("One part per subassembly")
                               : tr("One part per subassembly %1 levels down").arg(k),
                        k);
    }
    options->addRow(tr("Merge"), level_);
    auto* exactNote = new QLabel(tr("STEP parts keep their exact geometry; their meshes (for the "
                                    "views and collision checks) are made from it at the "
                                    "machine's mesh tolerance."));
    exactNote->setWordWrap(true);
    exactNote->setStyleSheet(QStringLiteral("color: gray"));
    options->addRow(exactNote);
    stlUnit_ = new QComboBox;
    stlUnit_->addItem(tr("Millimetres"), 1.0);
    stlUnit_->addItem(tr("Inches"), 25.4);
    stlUnit_->addItem(tr("Metres"), 1000.0);
    stlUnit_->setToolTip(tr("STL files carry no unit; STEP files do"));
    stlUnit_->setEnabled(
        std::any_of(parts_.begin(), parts_.end(),
                    [](const document::ImportedPart& p) { return p.mesh != nullptr; }));
    options->addRow(tr("STL unit"), stlUnit_);
    if (solids) {
        // Kept as solids: neither the machine's mesh tolerance nor an STL unit applies.
        exactNote->hide();
        options->labelForField(stlUnit_)->hide();
        stlUnit_->hide();
    }
    layout->addLayout(options);

    table_ = new QTableWidget(static_cast<int>(parts_.size()), 6);
    table_->setHorizontalHeaderLabels(
        {tr("Import"), tr("Body"), tr("Assembly"), tr("File"), tr("Solids"), tr("Part")});
    table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Interactive);
    table_->horizontalHeader()->setSectionResizeMode(kGroup, QHeaderView::Stretch);
    table_->verticalHeader()->hide();
    table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    table_->setSelectionMode(QAbstractItemView::ExtendedSelection);
    for (int r = 0; r < table_->rowCount(); ++r) {
        const auto& p = parts_[static_cast<std::size_t>(r)];
        auto* use = new QTableWidgetItem;
        use->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable | Qt::ItemIsSelectable);
        use->setCheckState(Qt::Checked);
        table_->setItem(r, kUse, use);
        auto readOnly = [](const QString& text) {
            auto* item = new QTableWidgetItem(text);
            item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
            return item;
        };
        table_->setItem(r, kName, readOnly(qs(p.name)));
        QStringList path;
        for (const auto& a : p.path) {
            path << qs(a);
        }
        table_->setItem(r, kAssembly, readOnly(path.join(QStringLiteral(" / "))));
        table_->setItem(r, kFile, readOnly(qs(p.file)));
        table_->setItem(r, kSolids, readOnly(p.mesh ? tr("mesh") : QString::number(p.solids)));
        table_->setItem(r, kGroup, new QTableWidgetItem);
    }
    table_->resizeColumnsToContents();
    layout->addWidget(table_, 1);

    auto* row = new QHBoxLayout;
    auto* merge = new QPushButton(tr("Merge selected..."));
    merge->setToolTip(tr("Give the selected parts one name: they become one machine part"));
    row->addWidget(merge);
    summary_ = new QLabel;
    row->addWidget(summary_, 1);
    layout->addLayout(row);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    buttons->button(QDialogButtonBox::Ok)->setText(tr("Import"));
    layout->addWidget(buttons);

    connect(level_, &QComboBox::currentIndexChanged, this, &MachinePartsImportDialog::applyLevel);
    connect(merge, &QPushButton::clicked, this, &MachinePartsImportDialog::mergeSelected);
    connect(table_, &QTableWidget::itemChanged, this, &MachinePartsImportDialog::updateSummary);
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);

    // Every body on its own; merging is the user's choice.
    level_->setCurrentIndex(0);
    applyLevel();
}

void MachinePartsImportDialog::setLevel(int level) {
    level_->setCurrentIndex(std::max(0, level_->findData(level)));
    applyLevel();
}

void MachinePartsImportDialog::applyLevel() {
    const auto groups = document::groupParts(parts_, level_->currentData().toInt());
    const QSignalBlocker block(table_);
    for (int r = 0; r < table_->rowCount(); ++r) {
        table_->item(r, kGroup)->setText(qs(groups[static_cast<std::size_t>(r)]));
    }
    updateSummary();
}

void MachinePartsImportDialog::mergeSelected() {
    std::vector<int> rows;
    for (const QModelIndex& index : table_->selectionModel()->selectedRows()) {
        rows.push_back(index.row());
    }
    if (rows.empty()) {
        return;
    }
    std::sort(rows.begin(), rows.end());
    bool ok = false;
    const QString name =
        QInputDialog::getText(this, tr("Merge parts"), tr("Machine part:"), QLineEdit::Normal,
                              table_->item(rows.front(), kGroup)->text(), &ok);
    if (!ok || name.trimmed().isEmpty()) {
        return;
    }
    const QSignalBlocker block(table_);
    for (const int r : rows) {
        table_->item(r, kGroup)->setText(name.trimmed());
        table_->item(r, kUse)->setCheckState(Qt::Checked);
    }
    updateSummary();
}

std::vector<std::string> MachinePartsImportDialog::groups() const {
    std::vector<std::string> out;
    for (int r = 0; r < table_->rowCount(); ++r) {
        const bool use = table_->item(r, kUse)->checkState() == Qt::Checked;
        const QString group = table_->item(r, kGroup)->text().trimmed();
        out.push_back(use ? group.toStdString() : std::string());
    }
    return out;
}

double MachinePartsImportDialog::stlScale() const {
    return stlUnit_->currentData().toDouble();
}

void MachinePartsImportDialog::updateSummary() {
    std::set<std::string> groups;
    for (const auto& g : this->groups()) {
        if (!g.empty()) {
            groups.insert(g);
        }
    }
    summary_->setText(tr("Machine parts to import: %1").arg(groups.size()));
}

std::vector<document::SolidPart> MachinePartsImportDialog::runSolids(const QStringList& files,
                                                                     QWidget* parent) {
    std::vector<document::ImportedPart> found;
    QStringList problems;
    const bool read = runWithProgress(
        parent, tr("Reading workholding CAD"), [&](CancelToken& cancel, ProgressSink& progress) {
            for (qsizetype i = 0; i < files.size(); ++i) {
                ProgressRange range(progress, static_cast<double>(i) / files.size(),
                                    static_cast<double>(i + 1) / files.size());
                const std::filesystem::path path(files[i].toStdU16String());
                try {
                    for (auto& p : document::readMachinePartFile(path, 1.0, cancel, range)) {
                        found.push_back(std::move(p));
                    }
                } catch (const OperationCancelled&) {
                    throw;
                } catch (const std::exception& e) {
                    problems << QStringLiteral("%1: %2").arg(QFileInfo(files[i]).fileName(),
                                                             QString::fromUtf8(e.what()));
                }
            }
        });
    if (!read) {
        return {};
    }
    if (!problems.isEmpty()) {
        QMessageBox::warning(
            parent, tr("Import workholding parts"),
            tr("Some files could not be read:\n\n%1").arg(problems.join(QLatin1Char('\n'))));
    }
    if (found.empty()) {
        return {};
    }
    MachinePartsImportDialog dialog(std::move(found), parent, true);
    if (dialog.exec() != QDialog::Accepted) {
        return {};
    }
    try {
        return document::solidPartsFrom(dialog.parts_, dialog.groups());
    } catch (const Error& e) {
        QMessageBox::warning(parent, tr("Import workholding parts"), QString::fromUtf8(e.what()));
        return {};
    }
}

std::vector<machine::ModelPart> MachinePartsImportDialog::run(const QStringList& files,
                                                              double tolerance, QWidget* parent) {
    std::vector<document::ImportedPart> found;
    QStringList problems;
    const bool read = runWithProgress(
        parent, tr("Reading machine CAD"), [&](CancelToken& cancel, ProgressSink& progress) {
            for (qsizetype i = 0; i < files.size(); ++i) {
                ProgressRange range(progress, static_cast<double>(i) / files.size(),
                                    static_cast<double>(i + 1) / files.size());
                const std::filesystem::path path(files[i].toStdU16String());
                try {
                    for (auto& p : document::readMachinePartFile(path, 1.0, cancel, range)) {
                        found.push_back(std::move(p));
                    }
                } catch (const OperationCancelled&) {
                    throw;
                } catch (const std::exception& e) {
                    problems << QStringLiteral("%1: %2").arg(QFileInfo(files[i]).fileName(),
                                                             QString::fromUtf8(e.what()));
                }
            }
        });
    if (!read) {
        return {};
    }
    if (!problems.isEmpty()) {
        QMessageBox::warning(
            parent, tr("Import machine parts"),
            tr("Some files could not be read:\n\n%1").arg(problems.join(QLatin1Char('\n'))));
    }
    if (found.empty()) {
        return {};
    }
    MachinePartsImportDialog dialog(std::move(found), parent);
    if (dialog.exec() != QDialog::Accepted) {
        return {};
    }
    std::vector<document::ImportedPart> parts = dialog.parts_;
    if (const double scale = dialog.stlScale(); scale != 1.0) {
        for (auto& p : parts) {
            if (p.mesh) {
                geom3d::Mesh scaled = *p.mesh;
                for (Vec3& v : scaled.vertices) {
                    v *= scale;
                }
                p.mesh = std::make_shared<const geom3d::Mesh>(std::move(scaled));
            }
        }
    }
    const std::vector<std::string> groups = dialog.groups();
    std::vector<machine::ModelPart> out;
    if (!runWithProgress(parent, tr("Tessellating machine parts"),
                         [&](CancelToken& cancel, ProgressSink& progress) {
                             out = document::machinePartsFrom(parts, groups, tolerance, cancel,
                                                              progress);
                         })) {
        return {};
    }
    return out;
}

} // namespace ocam::gui
