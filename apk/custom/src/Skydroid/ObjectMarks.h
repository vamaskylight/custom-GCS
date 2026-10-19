#pragma once

// Marks the people and vehicles in the video while the lock button waits for
// an object, so the operator taps a mark instead of drawing a box.
//
// How it goes:
//  - QML sets "looking" while the lock button waits, and hands over a picture
//    of the video whenever wantsPicture() says so
//    (FlightDisplayViewVideoOutput.qml).
//  - Each picture gets two looks by the object finder (ObjectFinder.h,
//    planLooks): all of it made smaller, and its middle at its own size. The
//    looks run on a thread of their own, side by side on a device with four
//    processor cores or more. The video and the controls never wait for them.
//    After a look the finder rests a moment, so it never takes a whole core.
//  - What is found is shown as marks: QML draws a thin box for each. A mark is
//    where the object was in the picture it was found in, some tenths of a
//    second ago.
//  - A tap on a mark locks that object. The lock starts on the picture the
//    object was found in (SkydroidLink::lockAtFound), so it is on the object
//    also when that has moved on since.
//
// A mark is an offer to the operator. Nothing is locked, and no drone is
// flown, without their tap. What the finder does not mark can still be locked
// with a drawn box.
//
// The looking itself is MarksWorker's (MarksWorker.h), with the finder behind
// PictureFinder, so tests run all of this with a stand-in for the network.
//
// QML uses it as the singleton "ObjectMarks" from the QGC module.

#include <QtCore/QObject>
#include <QtCore/QRectF>
#include <QtCore/QString>
#include <QtCore/QThread>
#include <QtCore/QTimer>
#include <QtCore/QVariantList>
#include <QtGui/QImage>
#include <QtQmlIntegration/QtQmlIntegration>

#include <vector>

#include "MarksWorker.h"
#include "ObjectFinder.h"

class QJSEngine;
class QQmlEngine;
class SkydroidLink;

class ObjectMarks : public QObject
{
    Q_OBJECT
    QML_ELEMENT
    QML_SINGLETON

    /// The operator's switch, saved between runs: mark people and vehicles.
    Q_PROPERTY(bool enabled READ enabled WRITE setEnabled NOTIFY enabledChanged)
    /// QML sets this while the lock button waits for an object and the video is the main picture.
    Q_PROPERTY(bool looking READ looking WRITE setLooking NOTIFY lookingChanged)
    /// Objects are being marked: enabled, looking, and the finder works.
    Q_PROPERTY(bool running READ running NOTIFY runningChanged)
    /// False once the finder has said that it cannot work (a build without it).
    Q_PROPERTY(bool usable READ usable NOTIFY stateChanged)
    /// The marks: for each, u and v (its centre) and w and h (its size) in parts
    /// of the video picture, kind ("person" or "vehicle"), score (how sure the
    /// finder is, 0 to 1) and sure (score kSureScore or more).
    Q_PROPERTY(QVariantList marks READ marks NOTIFY marksChanged)
    Q_PROPERTY(int count READ count NOTIFY marksChanged)
    /// One line for the screen: what is marked, and how long a look takes. Empty when not running.
    Q_PROPERTY(QString state READ state NOTIFY stateChanged)
    /// The part of the video picture the finder looks at at its own size, in parts
    /// of the picture: small objects are found there only. Empty when that is all
    /// of the picture, and before the first look.
    Q_PROPERTY(QRectF closeLook READ closeLook NOTIFY stateChanged)
    /// How wide the pictures should be, in points (or as wide as the video is shown, when that is less).
    Q_PROPERTY(int pictureWidth READ pictureWidth CONSTANT)

public:
    /// One marked object.
    struct Mark
    {
        skydroid::find::Found box;  // in parts of the video picture
        qint64 atMs = 0;            // the time of the picture it was found in
        int missed = 0;             // how many looks since then did not find it again
    };

    /// The pictures the finder gets: the camera's own size.
    static constexpr int kPictureWidth = 1280;
    /// A mark the finder is less sure of is shown as unsure: thin and faint, with a "?".
    /// (Task AD, client video 2026-10-19: upright board stacks marked as people. Measured
    /// on 1169 photo views on 2026-10-20: a higher score limit takes away real people about
    /// as fast as wrong marks, so all marks stay, and the operator sees how sure each one is.)
    static constexpr double kSureScore = 0.50;

    /// link: the camera link, which keeps the pictures a lock can start on.
    /// finder: taken over. ownThread false (tests): the looks run on the caller's
    /// thread, still asked for and answered through the event loop.
    ObjectMarks(SkydroidLink *link, PictureFinder *finder, bool ownThread, QObject *parent = nullptr);
    ~ObjectMarks() override;

    static ObjectMarks *instance();
    static ObjectMarks *create(QQmlEngine *qmlEngine, QJSEngine *jsEngine);

    bool enabled() const { return _enabled; }
    void setEnabled(bool enabled);
    bool looking() const { return _looking; }
    void setLooking(bool looking);
    bool running() const { return _running; }
    bool usable() const { return _usable; }
    QVariantList marks() const;
    int count() const { return static_cast<int>(_marks.size()); }
    QString state() const;
    QRectF closeLook() const { return _running ? _closeLook : QRectF(); }
    int pictureWidth() const { return kPictureWidth; }
    /// How long the last looks took, in milliseconds (0: none yet).
    double lookMs() const { return _lookMs; }
    const std::vector<Mark> &marksNow() const { return _marks; }

    /// True when a picture is wanted now: running, no look under way, and the rest after the last one is over.
    Q_INVOKABLE bool wantsPicture() const;
    /// A picture of the video, with where the video picture is inside it (as for SkydroidLink.lockPicture).
    /// grabMs: how long the screen took to get it, when it knows (the state line tells it).
    Q_INVOKABLE void picture(const QImage &image, double pictureX, double pictureY, double pictureW, double pictureH,
                             double grabMs = 0.0);
    /// A tap at u, v of the video picture. When a mark is there, a lock on its
    /// object is asked for and true comes back (when the link cannot lock now,
    /// its lock message says why). False: no mark there.
    Q_INVOKABLE bool lockAt(double u, double v);
    /// The licences of the network and of the library that runs it, to show in the app.
    Q_INVOKABLE QString notices() const;
    /// What the finder is (the library with its version, and the network). Empty: this build has none.
    Q_INVOKABLE QString finderName() const;

    /// The mark a tap at u, v means, or -1: the smallest mark the tap is inside,
    /// else the nearest one it is just beside. aspect: the picture's width over its height.
    static int markAt(const std::vector<Mark> &marks, double u, double v, double aspect);
    /// How long the finder rests after looks that took lookMs, in milliseconds.
    static int restAfter(double lookMs);
    /// True when a newly found object stands for an old mark: the same kind, and no
    /// further from it than twice the old mark's size, each way. (An object that
    /// moved on, or its neighbour in a crowd: the old mark is not shown beside it.)
    static bool takesThePlaceOf(const skydroid::find::Found &found, const skydroid::find::Found &oldMark);
    /// How long the finder may lie unused before it gives its memory back (tests set a short time).
    void setIdleRestMs(int ms);

signals:
    void enabledChanged();
    void lookingChanged();
    void runningChanged();
    void stateChanged();
    void marksChanged();
    /// The finder has answered for a picture (also when nothing on the screen changed).
    void lookDone();
    /// To the worker.
    void lookWanted(const QImage &image, const QRectF &pictureRect, double unitMs, qint64 atMs);
    void restWanted();

private:
    void _looked(const MarksAnswer &answer);
    void _update();
    void _dropOldMarks();
    void _clearMarks();
    qint64 _now() const;

    SkydroidLink *_link = nullptr;
    QThread _thread;
    MarksWorker *_worker = nullptr;
    bool _enabled = true;
    bool _looking = false;
    bool _running = false;
    bool _usable = true;
    QString _whyNot;
    bool _busy = false;        // a picture is with the worker
    bool _answered = false;    // since marking started
    qint64 _startedAtMs = 0;   // when marking started: older pictures are not shown
    qint64 _nextLookAtMs = 0;  // no picture is wanted before this
    QString _trouble;          // of the last looks, when one failed
    double _unitMs = 0.0;
    double _lookMs = 0.0;
    double _grabMs = 0.0;      // how long the screen took for the last picture (0: not told)
    bool _blank = false;       // the last picture was one flat colour
    double _aspect = 16.0 / 9.0;
    QRectF _closeLook;
    std::vector<Mark> _marks;
    QTimer _ageTimer;   // drops marks that are too old to show
    QTimer _idleTimer;  // lets the finder give its memory back
};
