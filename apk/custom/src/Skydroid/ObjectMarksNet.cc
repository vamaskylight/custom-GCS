// The app's object finder: the network, from the two files in the app's
// resources (custom.qrc, apk/custom/res/ai). The marks' own test runs with a
// stand-in in place of this file; marks_net_test builds it with the real files.

#include "MarksWorker.h"

#include <QtCore/QByteArray>
#include <QtCore/QCryptographicHash>
#include <QtCore/QFile>

#include <string>

namespace {

using skydroid::find::ObjectFinder;

constexpr const char *kLayersFile = ":/custom/ai/finder.param";
constexpr const char *kWeightsFile = ":/custom/ai/finder.bin";
constexpr const char *kNoticesFile = ":/custom/ai/NOTICES.txt";
// The SHA-256 of the two files as they came from their makers' release (apk/custom/res/ai/MODEL_SOURCE.md).
// Other files are other weights: the app would mark something else, or nothing, and say nothing.
constexpr const char *kLayersSha256 = "d79e18ecd8595081bb29fdf0c790a47fcea96fbf58598350569a92e71e915b72";
constexpr const char *kWeightsSha256 = "f7555ca6cb9650ee41e2f0a53fdf46b6e187a2a09792208d8c3eec033b6ac615";

bool readChecked(const QString &path, const char *sha256, QByteArray &into)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        return false;
    }
    into = file.readAll();
    return QCryptographicHash::hash(into, QCryptographicHash::Sha256).toHex() == QByteArray(sha256);
}

class NetPictureFinder : public PictureFinder
{
public:
    NetPictureFinder(const QString &layersFile, const QString &weightsFile)
        : _layersFile(layersFile)
        , _weightsFile(weightsFile)
    {
    }

    bool load(QString &whyNot) override
    {
        if (!ObjectFinder::inThisBuild()) {
            whyNot = QStringLiteral("this build has no object finding in it");
            return false;
        }
        QByteArray layers;
        QByteArray weights;
        if (!readChecked(_layersFile, kLayersSha256, layers) || !readChecked(_weightsFile, kWeightsSha256, weights)) {
            whyNot = QStringLiteral("the network's files are missing from the app, or are not the ones it was made with");
            return false;
        }
        auto finder = std::make_unique<ObjectFinder>();
        if (!finder->load(layers.toStdString(), reinterpret_cast<const std::uint8_t *>(weights.constData()),
                          static_cast<std::size_t>(weights.size()))) {
            whyNot = QString::fromStdString(finder->lastError());
            return false;
        }
        _finder = std::move(finder);
        return true;
    }

    std::vector<skydroid::find::Found> find(const skydroid::find::Picture &picture, const skydroid::find::Ask &ask,
                                            QString &whyNot) const override
    {
        if (!_finder) {
            whyNot = QStringLiteral("the network is not loaded");
            return {};
        }
        std::string why;
        std::vector<skydroid::find::Found> found = _finder->find(picture, ask, &why);
        whyNot = QString::fromStdString(why);
        return found;
    }

    void rest() override { _finder.reset(); }

private:
    QString _layersFile;
    QString _weightsFile;
    std::unique_ptr<ObjectFinder> _finder;
};

}  // namespace

PictureFinder *vamaMakePictureFinder()
{
    return new NetPictureFinder(QString::fromLatin1(kLayersFile), QString::fromLatin1(kWeightsFile));
}

PictureFinder *vamaMakePictureFinderFrom(const QString &layersFile, const QString &weightsFile)
{
    return new NetPictureFinder(layersFile, weightsFile);
}

QString vamaFinderName()
{
    if (!ObjectFinder::inThisBuild()) {
        return QString();
    }
    return QString::fromStdString(ObjectFinder::libraryName()) + QStringLiteral(", NanoDet-Plus-m");
}

QString vamaFinderNotices()
{
    QFile file(QString::fromLatin1(kNoticesFile));
    if (!file.open(QIODevice::ReadOnly)) {
        return QString();
    }
    return QString::fromUtf8(file.readAll());
}
