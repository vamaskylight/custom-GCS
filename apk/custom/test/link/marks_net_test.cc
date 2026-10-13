// The app's object finder with its real network and the real ncnn library, on this PC:
//  - the plain finder (ObjectFinder): what it takes and what it refuses,
//  - the app's own finder (ObjectMarksNet.cc), from the app's resource file custom.qrc,
//  - three pictures from the air (../ai_pictures): what the app marks in them, with
//    the looks it takes on a fast and on a slow device (MarksWorker).
// The same sources and the same library options build the phone's finder. What this
// cannot prove is the phone's own processor.
//
// VAMA_MARKS_PRINT=1 prints every box found in the pictures (for choosing what to expect).

#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QTemporaryDir>
#include <QtGui/QImage>
#include <QtTest/QtTest>

#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "MarksWorker.h"
#include "ObjectFinder.h"

using namespace skydroid::find;

namespace {

constexpr int kPictureW = 1280;
constexpr int kPictureH = 720;
// The speed of a fast device (the whole picture is looked at at its own size) and of a
// slow one (only the middle 640 x 384 points are).
constexpr double kFastUnitMs = 45.0;
constexpr double kSlowUnitMs = 900.0;
// The car that stands alone in the car park, as the network gives it for the picture's middle
// at its own size (theNetworkAnswersAsItDidWhenItWasChosen).
constexpr double kCarScore = 0.717;
constexpr double kCarX = 725.4;
constexpr double kCarY = 409.4;
constexpr double kCarW = 108.2;
constexpr double kCarH = 48.1;

/// Something that should be marked: its kind, and its box in points of the picture.
struct Want
{
    Kind kind;
    double cx;
    double cy;
    double w;
    double h;
    const char *what;
};

bool sameBoxes(const std::vector<Found> &a, const std::vector<Found> &b)
{
    if (a.size() != b.size()) {
        return false;
    }
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (a[i].kind != b[i].kind || a[i].label != b[i].label || a[i].score != b[i].score || a[i].cx != b[i].cx ||
            a[i].cy != b[i].cy || a[i].w != b[i].w || a[i].h != b[i].h) {
            return false;
        }
    }
    return true;
}

}  // namespace

class MarksNetTest : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase()
    {
        qRegisterMetaType<MarksAnswer>("MarksAnswer");
        QFile layers(QStringLiteral(":/custom/ai/finder.param"));
        QFile weights(QStringLiteral(":/custom/ai/finder.bin"));
        QVERIFY2(layers.open(QIODevice::ReadOnly) && weights.open(QIODevice::ReadOnly),
                 "the network's files are not in the app's resources (custom.qrc)");
        _layers = layers.readAll().toStdString();
        _weights = weights.readAll();
        QVERIFY(_finder.load(_layers, _bytes(_weights), static_cast<std::size_t>(_weights.size())));
        _bridge = _load("bridge_walkers.jpg");
        _carPark = _load("car_park.jpg");
        _seaFront = _load("sea_front.jpg");
        QVERIFY(!_bridge.isNull() && !_carPark.isNull() && !_seaFront.isNull());
        _worker = std::make_unique<MarksWorker>(vamaMakePictureFinder(), true);
        connect(_worker.get(), &MarksWorker::looked, this, [this](const MarksAnswer &answer) { _answer = answer; });
    }

    // --- The plain finder ------------------------------------------------------

    void theLibraryIsInThisBuild()
    {
        QVERIFY(ObjectFinder::inThisBuild());
        // The release the app is built with (apk/build_ncnn.ps1).
        QCOMPARE(QString::fromStdString(ObjectFinder::libraryName()), QStringLiteral("ncnn 1.0.20260526"));
    }

    void aFinderWithoutItsNetworkFindsNothingAndSaysWhy()
    {
        ObjectFinder empty;
        QVERIFY(!empty.ready());
        std::string why;
        QVERIFY(empty.find(_view(_bridge), Ask{}, &why).empty());
        QCOMPARE(QString::fromStdString(why), QStringLiteral("the network is not loaded"));
        QVERIFY(empty.find(_view(_bridge), Ask{}).empty());  // and with nowhere to say it
    }

    void filesThatDoNotFitAreRefused()
    {
        const std::size_t size = static_cast<std::size_t>(_weights.size());
        ObjectFinder finder;
        QVERIFY2(!finder.load(_layers, _bytes(_weights), size / 2), "half of the weights");
        QVERIFY(!finder.ready());
        QVERIFY(QString::fromStdString(finder.lastError()).contains("weights"));
        QByteArray longer = _weights;
        longer.append(4, '\0');
        QVERIFY2(!finder.load(_layers, _bytes(longer), static_cast<std::size_t>(longer.size())), "four bytes too many");
        QVERIFY(QString::fromStdString(finder.lastError()).contains("longer"));
        QVERIFY2(!finder.load(_layers, _bytes(_weights), 0), "no weights");
        QVERIFY(QString::fromStdString(finder.lastError()).contains("empty"));
        QVERIFY2(!finder.load(std::string(), _bytes(_weights), size), "no layer list");
        QVERIFY(QString::fromStdString(finder.lastError()).contains("empty"));
        QVERIFY2(!finder.load(_layers, nullptr, size), "weights that are nowhere");
        QVERIFY(QString::fromStdString(finder.lastError()).contains("empty"));
        QVERIFY2(!finder.load("this is no layer list", _bytes(_weights), size), "a text that is no layer list");
        QVERIFY(!finder.ready());
        // After all that it still takes the right files.
        QVERIFY(finder.load(_layers, _bytes(_weights), size));
        QVERIFY(finder.ready() && finder.lastError().empty());
        QVERIFY(!finder.find(_view(_carPark), Ask{}).empty());
        // And a load that fails leaves no old network behind.
        QVERIFY(!finder.load(_layers, _bytes(_weights), size / 2));
        QVERIFY(!finder.ready());
        QVERIFY(finder.find(_view(_carPark), Ask{}).empty());
    }

    void picturesThatCannotBeReadAreRefused()
    {
        const Picture good = _view(_bridge);
        std::string why;
        Picture bad = good;
        bad.data = nullptr;
        QVERIFY(_finder.find(bad, Ask{}, &why).empty());
        QCOMPARE(QString::fromStdString(why), QStringLiteral("the picture cannot be read"));
        bad = good;
        bad.width = 0;
        QVERIFY(_finder.find(bad, Ask{}, &why).empty() && !why.empty());
        bad = good;
        bad.height = -5;
        QVERIFY(_finder.find(bad, Ask{}, &why).empty() && !why.empty());
        bad = good;
        bad.channels = 2;
        QVERIFY(_finder.find(bad, Ask{}, &why).empty() && !why.empty());
        bad = good;
        bad.channels = 1;
        QVERIFY(_finder.find(bad, Ask{}, &why).empty() && !why.empty());
        bad = good;
        bad.stride = good.width * 3 - 1;  // rows that would run into each other
        QVERIFY(_finder.find(bad, Ask{}, &why).empty() && !why.empty());
        Ask tiny;
        tiny.area = Area{100, 100, 10, 10};
        QVERIFY(_finder.find(good, tiny, &why).empty());
        QCOMPARE(QString::fromStdString(why), QStringLiteral("the picture is too small"));
        Ask outside;
        outside.area = Area{5000, 5000, 640, 384};
        QVERIFY(_finder.find(good, outside, &why).empty() && !why.empty());
        // A good picture after all of them: the finder is as it was.
        QVERIFY(!_finder.find(good, Ask{}, &why).empty());
        QVERIFY(why.empty());
    }

    void aPartThatReachesOutOfThePictureIsCutAtItsBorder()
    {
        Ask ask;
        ask.area = Area{-200, -100, 900, 600};  // the picture's left top 700 x 500 points
        std::string why;
        const std::vector<Found> found = _finder.find(_view(_bridge), ask, &why);
        QVERIFY2(why.empty(), why.c_str());
        for (const Found &f : found) {
            QVERIFY(f.cx - f.w / 2.0 >= -1e-6 && f.cy - f.h / 2.0 >= -1e-6);
            QVERIFY(f.cx + f.w / 2.0 <= 700.0 + 1e-6 && f.cy + f.h / 2.0 <= 500.0 + 1e-6);
        }
        // Over the right and the bottom side: the same as the part that is inside the picture.
        Ask over;
        over.area = Area{320, 400, 1200, 500};
        Ask inside;
        inside.area = Area{320, 400, 960, 320};
        const std::vector<Found> cut = _finder.find(_view(_carPark), over, &why);
        QVERIFY2(why.empty(), why.c_str());
        QVERIFY(!cut.empty());
        QVERIFY(sameBoxes(cut, _finder.find(_view(_carPark), inside)));
        Ask left;
        left.area = Area{-300, -200, 940, 584};
        Ask leftInside;
        leftInside.area = Area{0, 0, 640, 384};
        QVERIFY(sameBoxes(_finder.find(_view(_carPark), left), _finder.find(_view(_carPark), leftInside)));
    }

    void aPictureWithNothingInItHoldsNothing()
    {
        QImage black(kPictureW, kPictureH, QImage::Format_RGB888);
        black.fill(Qt::black);
        std::string why;
        QVERIFY(_finder.find(_view(black), Ask{}, &why).empty());
        QVERIFY2(why.empty(), why.c_str());
        QImage grey(kPictureW, kPictureH, QImage::Format_RGB888);
        grey.fill(QColor(128, 128, 128));
        QVERIFY(_finder.find(_view(grey), Ask{}, &why).empty());
        QImage white(kPictureW, kPictureH, QImage::Format_RGB888);
        white.fill(Qt::white);
        QVERIFY(_finder.find(_view(white), Ask{}, &why).empty());
    }

    void everyInputSizeIsAnswered_data()
    {
        QTest::addColumn<int>("inputW");
        QTest::addColumn<int>("inputH");
        QTest::newRow("the smallest") << 64 << 64;
        QTest::newRow("the network's own") << 416 << 416;
        QTest::newRow("the whole picture's") << 640 << 384;
        QTest::newRow("a thermal picture's") << 480 << 384;
        QTest::newRow("a middle") << 832 << 480;
        QTest::newRow("another middle") << 960 << 544;
        QTest::newRow("the camera's own size") << 1280 << 736;
        QTest::newRow("a size that is rounded") << 630 << 370;
    }

    void everyInputSizeIsAnswered()
    {
        QFETCH(int, inputW);
        QFETCH(int, inputH);
        Ask ask;
        ask.inputW = inputW;
        ask.inputH = inputH;
        std::string why;
        const std::vector<Found> found = _finder.find(_view(_carPark), ask, &why);
        QVERIFY2(why.empty(), why.c_str());
        for (const Found &f : found) {
            QVERIFY(f.score >= ask.scoreMin && f.score <= 1.0);
            QVERIFY(f.w >= kLeastSide && f.h >= kLeastSide);
            QVERIFY(f.cx - f.w / 2.0 >= -1e-6 && f.cx + f.w / 2.0 <= kPictureW + 1e-6);
            QVERIFY(f.cy - f.h / 2.0 >= -1e-6 && f.cy + f.h / 2.0 <= kPictureH + 1e-6);
        }
        if (inputW >= 416) {
            QVERIFY2(!found.empty(), "a car park with nothing found in it");
        }
    }

    void theSamePictureGivesTheSameBoxesEveryTime()
    {
        const std::vector<Found> first = _finder.find(_view(_carPark), Ask{});
        QVERIFY(!first.empty());
        for (int i = 0; i < 3; ++i) {
            QVERIFY(sameBoxes(first, _finder.find(_view(_carPark), Ask{})));
        }
        // The surest first.
        for (std::size_t i = 1; i < first.size(); ++i) {
            QVERIFY(first[i - 1].score >= first[i].score);
        }
    }

    void fourBytesAPointAreReadAsThree()
    {
        const QImage four = _carPark.convertToFormat(QImage::Format_RGBA8888);
        const Picture picture{four.constBits(), four.width(), four.height(), static_cast<int>(four.bytesPerLine()), 4};
        QVERIFY(sameBoxes(_finder.find(_view(_carPark), Ask{}), _finder.find(picture, Ask{})));
    }

    void redAndBlueAreNotMixedUp()
    {
        // The network was taught with the colours in their places. With red and blue changed
        // over it still finds things, but not the same: this is what a wrong order would do.
        const std::vector<Found> right = _finder.find(_view(_seaFront), Ask{});
        const QImage swapped = _seaFront.rgbSwapped();
        const std::vector<Found> wrong = _finder.find(_view(swapped), Ask{});
        QVERIFY(!right.empty());
        QVERIFY2(!sameBoxes(right, wrong), "the picture's colour order does not reach the network");
    }

    void twoThreadsAtOnceFindWhatOneFinds()
    {
        Ask ask;
        const std::vector<Found> bridgeAlone = _finder.find(_view(_bridge), ask);
        const std::vector<Found> parkAlone = _finder.find(_view(_carPark), ask);
        QVERIFY(!bridgeAlone.empty() && !parkAlone.empty() && !sameBoxes(bridgeAlone, parkAlone));
        int bridgeWrong = 0;
        int parkWrong = 0;
        std::thread one([&]() {
            for (int i = 0; i < 8; ++i) {
                bridgeWrong += sameBoxes(bridgeAlone, _finder.find(_view(_bridge), ask)) ? 0 : 1;
            }
        });
        std::thread two([&]() {
            for (int i = 0; i < 8; ++i) {
                parkWrong += sameBoxes(parkAlone, _finder.find(_view(_carPark), ask)) ? 0 : 1;
            }
        });
        one.join();
        two.join();
        QCOMPARE(bridgeWrong, 0);
        QCOMPARE(parkWrong, 0);
    }

    // --- The app's own finder ----------------------------------------------------

    void theAppsFinderLoadsFromTheAppsResources()
    {
        std::unique_ptr<PictureFinder> finder(vamaMakePictureFinder());
        QString why;
        QVERIFY(finder->find(_view(_carPark), Ask{}, why).empty());  // before it is loaded
        QCOMPARE(why, QStringLiteral("the network is not loaded"));
        QVERIFY2(finder->load(why), qPrintable(why));
        const std::vector<Found> found = finder->find(_view(_carPark), Ask{}, why);
        QVERIFY2(why.isEmpty(), qPrintable(why));
        QVERIFY(sameBoxes(found, _finder.find(_view(_carPark), Ask{})));
        finder->rest();
        QVERIFY(finder->find(_view(_carPark), Ask{}, why).empty());
        QVERIFY(finder->load(why));  // and again after a rest
        QVERIFY(sameBoxes(found, finder->find(_view(_carPark), Ask{}, why)));
    }

    void filesThatAreNotTheAppsOwnAreRefused()
    {
        QTemporaryDir dir;
        QVERIFY(dir.isValid());
        const QString layersPath = dir.filePath("finder.param");
        const QString weightsPath = dir.filePath("finder.bin");
        const auto write = [](const QString &path, const QByteArray &bytes) {
            QFile file(path);
            return file.open(QIODevice::WriteOnly) && file.write(bytes) == bytes.size();
        };
        const QByteArray layers = QByteArray::fromStdString(_layers);
        QString why;
        // The app's own files, from a folder: taken.
        QVERIFY(write(layersPath, layers) && write(weightsPath, _weights));
        {
            std::unique_ptr<PictureFinder> finder(vamaMakePictureFinderFrom(layersPath, weightsPath));
            QVERIFY2(finder->load(why), qPrintable(why));
        }
        // One byte of the weights changed: the network would still run, and mark something else.
        QByteArray changed = _weights;
        changed[changed.size() / 2] = static_cast<char>(changed.at(changed.size() / 2) ^ 0x10);
        QVERIFY(write(weightsPath, changed));
        {
            std::unique_ptr<PictureFinder> finder(vamaMakePictureFinderFrom(layersPath, weightsPath));
            QVERIFY(!finder->load(why));
            QVERIFY2(why.contains("not the ones"), qPrintable(why));
            QVERIFY(finder->find(_view(_carPark), Ask{}, why).empty());
        }
        // One letter of the layer list changed.
        QByteArray otherLayers = layers;
        otherLayers[otherLayers.size() - 2] = 'x';
        QVERIFY(write(layersPath, otherLayers) && write(weightsPath, _weights));
        {
            std::unique_ptr<PictureFinder> finder(vamaMakePictureFinderFrom(layersPath, weightsPath));
            QVERIFY(!finder->load(why));
        }
        // No files at all.
        {
            std::unique_ptr<PictureFinder> finder(vamaMakePictureFinderFrom(dir.filePath("none.param"), dir.filePath("none.bin")));
            QVERIFY(!finder->load(why));
            QVERIFY(!why.isEmpty());
        }
    }

    void theAppsFinderHasAName()
    {
        // In the camera settings: which library and which network a build has.
        QCOMPARE(vamaFinderName(), QStringLiteral("ncnn 1.0.20260526, NanoDet-Plus-m"));
    }

    void theLicenceNoticesAreInTheApp()
    {
        const QString notices = vamaFinderNotices();
        QVERIFY(notices.contains("ncnn"));
        QVERIFY(notices.contains("BSD 3-Clause"));
        QVERIFY(notices.contains("Redistributions in binary form must reproduce the above copyright notice"));
        QVERIFY(notices.contains("Tencent"));
        QVERIFY(notices.contains("NanoDet"));
        QVERIFY(notices.contains("Apache License"));
        QVERIFY(notices.contains("RangiLyu"));
        QVERIFY(notices.contains("zlib License"));
    }

    // --- Three pictures from the air ---------------------------------------------

    // What is asked for below was chosen by eye from what the finder marks (VAMA_MARKS_PRINT=1):
    // people and vehicles that are plainly there, and that it is clearly sure of (0.47 and more,
    // where 0.40 is the bar). Each is asked for by its place and about its size.

    void walkersOnABridge()
    {
        // People walking on a footbridge, 30 to 80 points tall.
        const Want nearLeft{Kind::Person, 206, 491, 24, 51, "the walker at the left, 51 points tall"};
        const Want farRight{Kind::Person, 1108, 35, 16, 32, "a walker far up the bridge, 32 points tall"};
        const std::vector<Want> inTheMiddle = {
            {Kind::Person, 671, 349, 27, 54, "the two walkers in the middle of the bridge"},
            {Kind::Person, 906, 207, 19, 42, "a walker at the railing"},
            {Kind::Person, 844, 196, 10, 37, "the first of three walkers side by side"},
            {Kind::Person, 860, 202, 11, 34, "the second of them"},
            {Kind::Person, 877, 199, 14, 34, "the third of them"},
        };
        const std::vector<Want> elsewhere = {
            nearLeft,
            {Kind::Person, 137, 516, 26, 45, "a walker behind them"},
            {Kind::Person, 103, 570, 31, 78, "a cyclist"},
            farRight,
        };

        // A fast device looks at all of the picture at its own size: they are all marked.
        const MarksAnswer fast = _look(_bridge, kFastUnitMs);
        _print("bridge, fast", fast);
        QVERIFY2(fast.whyNot.isEmpty() && fast.trouble.isEmpty(), qPrintable(fast.whyNot + fast.trouble));
        QVERIFY(fast.closer.isEmpty());
        for (const Want &want : inTheMiddle) {
            QVERIFY2(_marked(fast, want), want.what);
        }
        for (const Want &want : elsewhere) {
            QVERIFY2(_marked(fast, want), want.what);
        }
        QVERIFY2(fast.found.size() >= 14 && fast.found.size() <= 30, qPrintable(QString::number(fast.found.size())));
        QCOMPARE(_count(fast, Kind::Vehicle), 0);  // there is no vehicle on the bridge

        // A slow device looks at the middle 640 x 384 points at their own size, and at the whole
        // picture at half its size. The walkers in the middle are marked. Those outside it are
        // 25 points tall and less in the smaller picture: not marked.
        const MarksAnswer slow = _look(_bridge, kSlowUnitMs);
        _print("bridge, slow", slow);
        QVERIFY(qAbs(slow.closer.x() - 0.25) < 1e-9 && qAbs(slow.closer.width() - 0.5) < 1e-9);
        for (const Want &want : inTheMiddle) {
            QVERIFY2(_marked(slow, want), want.what);
        }
        QVERIFY2(!_marked(slow, nearLeft), "a walker 51 points tall outside the middle, on a slow device");
        QVERIFY2(!_marked(slow, farRight), "a walker 32 points tall outside the middle, on a slow device");
        QVERIFY(slow.found.size() < fast.found.size());
    }

    void aCarPark()
    {
        const std::vector<Want> everywhere = {
            {Kind::Vehicle, 725, 410, 108, 48, "the car that stands alone"},
            {Kind::Vehicle, 617, 355, 91, 44, "the car behind it"},
            {Kind::Vehicle, 797, 575, 129, 74, "the dark car at the right"},
            {Kind::Person, 415, 463, 14, 42, "the person between the rows"},
        };
        const Want whiteCar{Kind::Vehicle, 122, 380, 59, 45, "the white car at the left"};
        const Want walkerOne{Kind::Person, 877, 624, 15, 45, "the first of two people walking at the bottom"};
        const Want walkerTwo{Kind::Person, 909, 619, 22, 55, "the second of them"};

        const MarksAnswer fast = _look(_carPark, kFastUnitMs);
        _print("car park, fast", fast);
        QVERIFY2(fast.whyNot.isEmpty() && fast.trouble.isEmpty(), qPrintable(fast.whyNot + fast.trouble));
        for (const Want &want : everywhere) {
            QVERIFY2(_marked(fast, want), want.what);
        }
        QVERIFY2(_marked(fast, whiteCar), whiteCar.what);
        QVERIFY2(_marked(fast, walkerOne), walkerOne.what);
        QVERIFY2(_marked(fast, walkerTwo), walkerTwo.what);
        // Cars in a row, door to door, are not told apart well: some get one box for two, some none.
        // So their number is only held in bounds.
        QVERIFY2(fast.found.size() >= 14 && fast.found.size() <= 32, qPrintable(QString::number(fast.found.size())));
        QVERIFY(_count(fast, Kind::Vehicle) >= 9);

        // On a slow device: what is large, or in the middle. The two people at the bottom are
        // 45 and 55 points tall and outside the middle: not marked.
        const MarksAnswer slow = _look(_carPark, kSlowUnitMs);
        _print("car park, slow", slow);
        for (const Want &want : everywhere) {
            QVERIFY2(_marked(slow, want), want.what);
        }
        QVERIFY2(!_marked(slow, walkerOne) && !_marked(slow, walkerTwo), "people 50 points tall outside the middle, on a slow device");
    }

    void aSeaFront()
    {
        const std::vector<Want> wanted = {
            {Kind::Person, 107, 524, 24, 70, "the person at the left of the promenade"},
            {Kind::Person, 529, 183, 12, 29, "a person at the sea wall, 29 points tall"},
            {Kind::Person, 437, 146, 11, 30, "another one there"},
            {Kind::Person, 603, 139, 13, 33, "a person on the pavement"},
            {Kind::Person, 392, 383, 13, 37, "a person at the fence"},
            {Kind::Vehicle, 707, 200, 56, 43, "the white car on the road"},
            {Kind::Vehicle, 625, 53, 41, 32, "the car further up the road"},
            {Kind::Vehicle, 867, 237, 49, 42, "the dark car on the other lane"},
        };
        const MarksAnswer fast = _look(_seaFront, kFastUnitMs);
        _print("sea front, fast", fast);
        QVERIFY2(fast.whyNot.isEmpty() && fast.trouble.isEmpty(), qPrintable(fast.whyNot + fast.trouble));
        for (const Want &want : wanted) {
            QVERIFY2(_marked(fast, want), want.what);
        }
        // This is a building site, and the finder is wrong here more than elsewhere: it marks a
        // pallet of bricks as a vehicle (at 789, 295, as sure as of the cars), and two traffic
        // cones as people. The test does not ask for these marks and does not forbid them: a
        // mark is an offer to the operator. It only holds the number of marks in bounds.
        QVERIFY2(fast.found.size() >= 12 && fast.found.size() <= 28, qPrintable(QString::number(fast.found.size())));

        const MarksAnswer slow = _look(_seaFront, kSlowUnitMs);
        _print("sea front, slow", slow);
        const std::vector<Want> inTheMiddle = {wanted[1], wanted[4], wanted[7]};
        for (const Want &want : inTheMiddle) {
            QVERIFY2(_marked(slow, want), want.what);
        }
    }

    void theNetworkAnswersAsItDidWhenItWasChosen()
    {
        // The middle of the car park at its own size: the car that stands alone, with the numbers
        // of 2026-10-13. Another colour order, other means or another scale for the picture, or
        // other weights, move them. (The processor's own arithmetic moves them by a hair.)
        Ask ask;
        ask.area = Area{320, 168, 640, 384};
        const std::vector<Found> found = _finder.find(_view(_carPark), ask);
        QVERIFY(!found.empty());
        const Found &car = found.front();
        const QString got = QStringLiteral("%1 %2 at %3, %4 size %5 x %6").arg(kindName(car.kind)).arg(car.score, 0, 'f', 4)
                                .arg(car.cx, 0, 'f', 1).arg(car.cy, 0, 'f', 1).arg(car.w, 0, 'f', 1).arg(car.h, 0, 'f', 1);
        if (qEnvironmentVariableIsSet("VAMA_MARKS_PRINT")) {
            qInfo("the middle of the car park: %s", qPrintable(got));
        }
        QVERIFY2(car.kind == Kind::Vehicle && car.label == 2, qPrintable(got));
        QVERIFY2(std::abs(car.score - kCarScore) < 0.01, qPrintable(got));
        QVERIFY2(std::abs(car.cx - kCarX) < 1.5 && std::abs(car.cy - kCarY) < 1.5 && std::abs(car.w - kCarW) < 2.0 &&
                     std::abs(car.h - kCarH) < 2.0,
                 qPrintable(got));
    }

    void aNetworkWhoseAnswerHasAnotherNameIsRefusedAtOnce()
    {
        // The weights fit, but the answer the app reads is not there: refused when it is loaded,
        // not when the first picture comes.
        std::string other = _layers;
        const std::size_t at = other.rfind(" output ");
        QVERIFY(at != std::string::npos);
        other.replace(at, 8, " answer ");
        ObjectFinder finder;
        QVERIFY(!finder.load(other, _bytes(_weights), static_cast<std::size_t>(_weights.size())));
        QVERIFY2(QString::fromStdString(finder.lastError()).contains("does not run"), finder.lastError().c_str());
        QVERIFY(!finder.ready());
    }

    void aPictureAsTheScreenGivesItIsReadWithItsColoursInPlace()
    {
        // The screen hands over four bytes a point, blue first (ARGB32 as Qt keeps it). The marks
        // are the same as for three bytes, red first.
        const MarksAnswer plain = _look(_seaFront, kFastUnitMs);
        const MarksAnswer screen = _look(_seaFront.convertToFormat(QImage::Format_ARGB32_Premultiplied), kFastUnitMs);
        QVERIFY(!plain.found.empty());
        QVERIFY(sameBoxes(plain.found, screen.found));
        const MarksAnswer rgba = _look(_seaFront.convertToFormat(QImage::Format_RGBA8888_Premultiplied), kFastUnitMs);
        QVERIFY(sameBoxes(plain.found, rgba.found));
    }

    void aBlackPictureIsToldAsNoVideo()
    {
        QImage black(kPictureW, kPictureH, QImage::Format_RGB888);
        black.fill(Qt::black);
        const MarksAnswer none = _look(black, kFastUnitMs);
        QVERIFY(none.whyNot.isEmpty() && none.trouble.isEmpty());
        QVERIFY(none.blank && none.found.empty());
        const MarksAnswer some = _look(_carPark, kFastUnitMs);
        QVERIFY(!some.blank && !some.found.empty());
    }

    void theSameIsMarkedInAPictureWithBarsBesideIt()
    {
        // The car park in the middle of a wider image, as the thermal picture lies in the video
        // item: marks are in parts of the picture, so they are the same as without the bars.
        QImage wide(1600, 720, QImage::Format_RGB888);
        wide.fill(Qt::black);
        for (int y = 0; y < 720; ++y) {
            std::memcpy(wide.scanLine(y) + 160 * 3, _carPark.constScanLine(y), 1280 * 3);
        }
        const MarksAnswer framed = _look(wide, kFastUnitMs, QRectF(0.1, 0.0, 0.8, 1.0));
        QVERIFY2(framed.whyNot.isEmpty() && framed.trouble.isEmpty(), qPrintable(framed.whyNot + framed.trouble));
        QVERIFY(qAbs(framed.aspect - 16.0 / 9.0) < 1e-9);
        const Want alone{Kind::Vehicle, 725, 410, 108, 48, "the car that stands alone"};
        const Want person{Kind::Person, 415, 463, 14, 42, "the person between the rows"};
        QVERIFY2(_marked(framed, alone), alone.what);
        QVERIFY2(_marked(framed, person), person.what);
        for (const Found &f : framed.found) {
            QVERIFY(f.cx - f.w / 2.0 >= -1e-9 && f.cx + f.w / 2.0 <= 1.0 + 1e-9);
        }
        // A picture of the thermal one's shape, 5 to 4: its shape is told with the marks.
        const MarksAnswer narrow = _look(_carPark, kFastUnitMs, QRectF(190.0 / 1280.0, 0.0, 900.0 / 1280.0, 1.0));
        QVERIFY(qAbs(narrow.aspect - 1.25) < 1e-9);
        QVERIFY(!narrow.found.empty());
    }

    void howLongALookTakes()
    {
        // Told, not judged: the test PC is not the remote control.
        Ask ask;
        _finder.find(_view(_carPark), ask);
        QElapsedTimer taken;
        taken.start();
        for (int i = 0; i < 5; ++i) {
            _finder.find(_view(_carPark), ask);
        }
        const double unit = static_cast<double>(taken.nsecsElapsed()) / 5.0e6;
        const MarksAnswer fast = _look(_carPark, kFastUnitMs);
        const MarksAnswer slow = _look(_carPark, kSlowUnitMs);
        qInfo("one look of 640 x 384 points: %.0f ms on this PC, one thread. A picture's two looks: %.0f ms with all "
              "of it at its own size, %.0f ms with its middle only.",
              unit, fast.lookMs, slow.lookMs);
        QVERIFY(unit > 0.0);
    }

private:
    static const std::uint8_t *_bytes(const QByteArray &bytes)
    {
        return reinterpret_cast<const std::uint8_t *>(bytes.constData());
    }

    static QImage _load(const char *name)
    {
        const QImage image(QStringLiteral(VAMA_TEST_PICTURES "/") + QLatin1String(name));
        return image.isNull() ? image : image.convertToFormat(QImage::Format_RGB888);
    }

    static Picture _view(const QImage &rgb)
    {
        return Picture{rgb.constBits(), rgb.width(), rgb.height(), static_cast<int>(rgb.bytesPerLine()), 3};
    }

    /// The looks the app takes at a picture on a device of this speed.
    MarksAnswer _look(const QImage &image, double unitMs, const QRectF &pictureRect = QRectF(0.0, 0.0, 1.0, 1.0))
    {
        _answer = MarksAnswer{};
        _answer.whyNot = QStringLiteral("no answer came");
        _worker->look(image, pictureRect, unitMs, 0);
        return _answer;
    }

    static int _count(const MarksAnswer &answer, Kind kind)
    {
        int n = 0;
        for (const Found &f : answer.found) {
            n += f.kind == kind ? 1 : 0;
        }
        return n;
    }

    /// True when the object is marked: a box of its kind over its centre, of about its size.
    static bool _marked(const MarksAnswer &answer, const Want &want)
    {
        for (const Found &f : answer.found) {
            const double cx = f.cx * kPictureW;
            const double cy = f.cy * kPictureH;
            const double w = f.w * kPictureW;
            const double h = f.h * kPictureH;
            if (f.kind == want.kind && std::abs(want.cx - cx) <= w / 2.0 && std::abs(want.cy - cy) <= h / 2.0 &&
                w * h <= 3.0 * want.w * want.h && w * h >= want.w * want.h / 3.0) {
                return true;
            }
        }
        return false;
    }

    static void _print(const char *name, const MarksAnswer &answer)
    {
        if (!qEnvironmentVariableIsSet("VAMA_MARKS_PRINT")) {
            return;
        }
        qInfo("%s: %d marks in %.0f ms", name, static_cast<int>(answer.found.size()), answer.lookMs);
        for (const Found &f : answer.found) {
            qInfo("   %s %.2f at %.0f, %.0f size %.0f x %.0f", kindName(f.kind), f.score, f.cx * kPictureW,
                  f.cy * kPictureH, f.w * kPictureW, f.h * kPictureH);
        }
    }

    std::string _layers;
    QByteArray _weights;
    ObjectFinder _finder;
    QImage _bridge;
    QImage _carPark;
    QImage _seaFront;
    std::unique_ptr<MarksWorker> _worker;
    MarksAnswer _answer;
};

QTEST_GUILESS_MAIN(MarksNetTest)
#include "marks_net_test.moc"
