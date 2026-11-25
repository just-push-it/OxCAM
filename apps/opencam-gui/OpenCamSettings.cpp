// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "OpenCamSettings.hpp"

#include <QCoreApplication>
#include <QDir>
#include <QDirIterator>
#include <QFile>
#include <QFileInfo>
#include <QSettings>
#include <QStandardPaths>
#include <QStringList>
#include <QVariant>

namespace ocam::gui {

namespace {

// The organization and application name OpenCAM kept its settings under.
const QString kOpenCam = QStringLiteral("OpenCAM");

// The data folder (QStandardPaths::AppDataLocation) of the application `name`.
QString dataFolder(const QString& name) {
    const QString organization = QCoreApplication::organizationName();
    const QString application = QCoreApplication::applicationName();
    QCoreApplication::setOrganizationName(name);
    QCoreApplication::setApplicationName(name);
    const QString folder = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    QCoreApplication::setOrganizationName(organization);
    QCoreApplication::setApplicationName(application);
    return folder;
}

// Copies the files of `from` (and its subfolders) that `to` does not have.
void copyMissingFiles(const QString& from, const QString& to) {
    QDirIterator it(from, QDir::Files | QDir::Hidden | QDir::NoDotAndDotDot,
                    QDirIterator::Subdirectories);
    while (it.hasNext()) {
        const QString file = it.next();
        const QString target = to + file.mid(from.size());
        if (!QFileInfo::exists(target)) {
            QDir().mkpath(QFileInfo(target).absolutePath());
            QFile::copy(file, target);
        }
    }
}

// A path in folder `from` names the same file in `to`; anything else stays as it is.
QString repointed(const QString& path, const QString& from, const QString& to) {
    const QString normalized = QDir::fromNativeSeparators(path);
    const QString prefix = QDir::fromNativeSeparators(from) + QLatin1Char('/');
    if (!normalized.startsWith(prefix, Qt::CaseInsensitive)) {
        return path;
    }
    return QDir::toNativeSeparators(QDir::fromNativeSeparators(to) + QLatin1Char('/') +
                                    normalized.mid(prefix.size()));
}

QVariant repointed(const QVariant& value, const QString& from, const QString& to) {
    if (value.typeId() == QMetaType::QString) {
        return repointed(value.toString(), from, to);
    }
    if (value.typeId() == QMetaType::QStringList) {
        QStringList list = value.toStringList();
        for (QString& s : list) {
            s = repointed(s, from, to);
        }
        return list;
    }
    return value;
}

} // namespace

bool importOpenCamSettings(bool copyData) {
    QSettings settings;
    if (!settings.allKeys().isEmpty()) {
        return false;
    }
    const QSettings old(QSettings::defaultFormat(), QSettings::UserScope, kOpenCam, kOpenCam);
    const QStringList keys = old.allKeys();
    if (keys.isEmpty()) {
        return false;
    }
    QString oldData;
    QString newData;
    if (copyData) {
        oldData = dataFolder(kOpenCam);
        newData = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
        if (oldData.isEmpty() || newData.isEmpty() || oldData == newData ||
            !QFileInfo(oldData).isDir()) {
            oldData.clear();
        } else {
            copyMissingFiles(oldData, newData);
        }
    }
    for (const QString& key : keys) {
        const QVariant value = old.value(key);
        settings.setValue(key, oldData.isEmpty() ? value : repointed(value, oldData, newData));
    }
    settings.sync();
    return true;
}

} // namespace ocam::gui
