// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The user's library of tooling blocks and tailstock centres (DL-0068): a STEP model and the points
// that say how it goes on, picked on the model's exact geometry or typed, shown marked on it.

#pragma once

#include <ocam/document/LatheFitting.hpp>

#include <QDialog>
#include <array>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QGroupBox;
class QLabel;
class QLineEdit;
class QListWidget;
class QPushButton;

namespace ocam::gui {

class OcctView;

/// Fittings on disk: the built-in examples first, then the user's.
class LatheFittingLibrary {
public:
    /// The library folder (created on demand). Settings key "fittingLibraryFolder" overrides the
    /// default <app data>/fittings.
    static std::filesystem::path folder();
    /// Uses `folder` instead of the settings' or the default one (test hooks); empty clears it.
    static void setFolderOverride(std::filesystem::path folder);
    static std::vector<document::FittingPackage> load(std::vector<std::string>* problems = nullptr);
};

class LatheFittingsDialog : public QDialog {
    Q_OBJECT

public:
    explicit LatheFittingsDialog(QWidget* parent = nullptr);

    /// Shows the fitting with this name (for tests); false when there is none.
    bool selectByName(const QString& name);
    /// Test hooks: picks on the model as a click would. `seat` chooses which point.
    /// `at`: where the click hit the model (its coordinates); a round face then gives the centre
    /// of its end nearest it. Without it, the end away from the fitting's middle.
    void pickForTest(bool seat, int type, int subIndex, std::optional<Vec3> at = std::nullopt);
    const document::LatheFitting* current() const;
    /// A new fitting from a STEP file, as New does without asking for a file.
    void addFromFile(const std::filesystem::path& file);
    /// Saves what is shown to the library, as Save does; false with a message when it cannot.
    bool saveCurrent(QString* why = nullptr);
    QImage snapshot();

private:
    struct PointEditors {
        QGroupBox* box = nullptr;
        std::array<QDoubleSpinBox*, 3> point{};
        std::array<QDoubleSpinBox*, 3> direction{};
        QLabel* hint = nullptr;
        QLabel* on = nullptr;
        QPushButton* pick = nullptr;
    };

    PointEditors buildPoint(const QString& title, bool seat);
    void reload();
    /// Hides the built-in fittings unless Show built-ins is ticked; a hidden current one gives
    /// way to the first one shown.
    void applyShowBuiltins();
    void showFitting(int row);
    void fieldsChanged();
    void updateScene(bool fit);
    void updateLabels();
    /// The suggested names of mounts (a block) or quill interfaces (a centre).
    void fillMountTypes(bool block);
    void clicked(int part, int type, int subIndex, const Vec3& at);
    void setPicking(std::optional<bool> seat);

    std::vector<document::FittingPackage> fittings_;
    int currentRow_ = -1;
    bool loading_ = false;
    bool dirty_ = false;
    std::optional<bool> picking_; ///< True: the seat; false: the other point.
    cad::Model model_;

    QListWidget* list_ = nullptr;
    QCheckBox* showBuiltins_ = nullptr;
    QWidget* editor_ = nullptr;
    QLineEdit* name_ = nullptr;
    QComboBox* kind_ = nullptr;
    QLineEdit* description_ = nullptr;
    PointEditors seat_;
    PointEditors mount_;
    QDoubleSpinBox* seatTurn_ = nullptr;
    QDoubleSpinBox* overhang_ = nullptr;
    QComboBox* mountType_ = nullptr;
    QLabel* mountTypeLabel_ = nullptr;
    QComboBox* takes_ = nullptr;
    QCheckBox* shankStated_ = nullptr;
    QDoubleSpinBox* shank_ = nullptr;
    QWidget* shankRow_ = nullptr;
    QCheckBox* boreStated_ = nullptr;
    QDoubleSpinBox* bore_ = nullptr;
    QWidget* boreRow_ = nullptr;
    QWidget* blockOnly_ = nullptr;
    QLabel* status_ = nullptr;
    QLabel* readOnly_ = nullptr;
    QPushButton* save_ = nullptr;
    QPushButton* remove_ = nullptr;
    OcctView* preview_ = nullptr;
    QCheckBox* onLathe_ = nullptr; ///< The preview shows it on the generic lathe.
};

} // namespace ocam::gui
