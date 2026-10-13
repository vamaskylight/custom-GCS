#include "MarksWorker.h"

#include <QtCore/QElapsedTimer>

#include <algorithm>
#include <cmath>
#include <future>
#include <system_error>

using skydroid::find::Area;
using skydroid::find::Ask;
using skydroid::find::Found;
using skydroid::find::Picture;
using skydroid::find::Plan;

namespace {

// A picture smaller than this, in points, is not looked at.
constexpr int kLeastPictureSide = 64;
// A picture of one flat colour (each colour's bytes this near each other, at every eighth
// point each way) is no picture of the scene: the video has not started, or the device
// cannot copy its video picture. The camera link takes it the same way (kBlankPictureSpread).
constexpr int kBlankSpread = 2;
constexpr int kBlankStep = 8;
constexpr double kUnitPoints = static_cast<double>(skydroid::find::kWholeInputW) * skydroid::find::kWholeInputH;

struct Look
{
    std::vector<Found> found;
    double ms = 0.0;
    QString why;
};

bool isBlank(const QImage &rgb, const Area &area)
{
    int lowest[3] = {255, 255, 255};
    int highest[3] = {0, 0, 0};
    for (int y = area.top; y < area.top + area.height; y += kBlankStep) {
        const uchar *row = rgb.constScanLine(y) + static_cast<qsizetype>(area.left) * 3;
        for (int x = 0; x < area.width; x += kBlankStep) {
            for (int c = 0; c < 3; ++c) {
                const int value = row[static_cast<qsizetype>(x) * 3 + c];
                lowest[c] = std::min(lowest[c], value);
                highest[c] = std::max(highest[c], value);
                if (highest[c] - lowest[c] > kBlankSpread) {
                    return false;
                }
            }
        }
    }
    return true;
}

// How many inputs of 640 x 384 points a look is worth.
double unitsOf(const Ask &ask)
{
    const skydroid::find::Fit fit = skydroid::find::fitArea(ask.area.width, ask.area.height, ask.inputW, ask.inputH,
                                                            ask.keepShape);
    return static_cast<double>(fit.inputW) * fit.inputH / kUnitPoints;
}

}  // namespace

MarksWorker::MarksWorker(PictureFinder *finder, bool sideBySide, QObject *parent)
    : QObject(parent)
    , _finder(finder)
    , _sideBySide(sideBySide)
{
}

MarksWorker::~MarksWorker() = default;

void MarksWorker::rest()
{
    if (_loaded) {
        _finder->rest();
        _loaded = false;
    }
}

void MarksWorker::look(const QImage &image, const QRectF &pictureRect, double unitMs, qint64 atMs)
{
    MarksAnswer answer;
    answer.atMs = atMs;
    if (!_loaded && !_broken) {
        QString why;
        if (_finder->load(why)) {
            _loaded = true;
        } else {
            _broken = true;
            _whyNot = why.isEmpty() ? QStringLiteral("the finder did not start") : why;
        }
    }
    if (_broken) {
        answer.whyNot = _whyNot;
        emit looked(answer);
        return;
    }

    // The finder reads red, green, blue: three bytes a point.
    const QImage rgb = image.format() == QImage::Format_RGB888 ? image : image.convertToFormat(QImage::Format_RGB888);
    Area area;
    const bool rectIsNumbers = std::isfinite(pictureRect.x()) && std::isfinite(pictureRect.y()) &&
                               std::isfinite(pictureRect.width()) && std::isfinite(pictureRect.height());
    if (!rgb.isNull() && rectIsNumbers && pictureRect.isValid()) {
        const int left = std::clamp(static_cast<int>(std::lround(pictureRect.x() * rgb.width())), 0, rgb.width());
        const int top = std::clamp(static_cast<int>(std::lround(pictureRect.y() * rgb.height())), 0, rgb.height());
        const int right = std::clamp(static_cast<int>(std::lround(pictureRect.right() * rgb.width())), left, rgb.width());
        const int bottom = std::clamp(static_cast<int>(std::lround(pictureRect.bottom() * rgb.height())), top, rgb.height());
        area = Area{left, top, right - left, bottom - top};
    }
    if (area.width < kLeastPictureSide || area.height < kLeastPictureSide) {
        emit looked(answer);  // nothing to look at: nothing found
        return;
    }
    answer.aspect = static_cast<double>(area.width) / area.height;
    if (isBlank(rgb, area)) {
        answer.blank = true;  // nothing to find, and the screen says why
        emit looked(answer);
        return;
    }
    const Picture picture{rgb.constBits(), rgb.width(), rgb.height(), static_cast<int>(rgb.bytesPerLine()), 3};
    const Plan plan = skydroid::find::planLooks(area, unitMs);
    const PictureFinder *finder = _finder.get();
    const auto lookAt = [finder, &picture](const Ask &ask) {
        Look look;
        QElapsedTimer taken;
        taken.start();
        look.found = finder->find(picture, ask, look.why);
        look.ms = static_cast<double>(taken.nsecsElapsed()) / 1.0e6;
        return look;
    };

    QElapsedTimer all;
    all.start();
    Look whole;
    Look middle;
    bool middleDone = false;
    if (plan.closer && _sideBySide) {
        // The middle on a thread of its own, the whole picture here.
        try {
            std::future<Look> other = std::async(std::launch::async, lookAt, plan.middle);
            whole = lookAt(plan.whole);
            middle = other.get();
            middleDone = true;
        } catch (const std::system_error &) {
            // No thread to be had: one after the other, as below.
        }
    }
    if (!middleDone) {
        whole = lookAt(plan.whole);
        if (plan.closer) {
            middle = lookAt(plan.middle);
        }
    }
    answer.lookMs = static_cast<double>(all.nsecsElapsed()) / 1.0e6;
    // How fast the device is: from the looks that ran. (One that failed took no time.)
    double unitSum = 0.0;
    int unitCount = 0;
    if (whole.why.isEmpty()) {
        unitSum += whole.ms / unitsOf(plan.whole);
        ++unitCount;
    }
    if (plan.closer && middle.why.isEmpty()) {
        unitSum += middle.ms / unitsOf(plan.middle);
        ++unitCount;
    }
    answer.unitMs = unitCount > 0 ? unitSum / unitCount : 0.0;
    answer.trouble = !whole.why.isEmpty() ? whole.why : middle.why;
    if (plan.closer && (plan.middle.area.width < area.width || plan.middle.area.height < area.height)) {
        answer.closer = QRectF(static_cast<double>(plan.middle.area.left - area.left) / area.width,
                               static_cast<double>(plan.middle.area.top - area.top) / area.height,
                               static_cast<double>(plan.middle.area.width) / area.width,
                               static_cast<double>(plan.middle.area.height) / area.height);
    }

    std::vector<Found> found = plan.closer ? skydroid::find::merge(whole.found, middle.found, plan.middle.area, area)
                                           : whole.found;
    for (Found &f : found) {
        f.cx = (f.cx - area.left) / area.width;
        f.cy = (f.cy - area.top) / area.height;
        f.w /= area.width;
        f.h /= area.height;
    }
    answer.found = std::move(found);
    emit looked(answer);
}
