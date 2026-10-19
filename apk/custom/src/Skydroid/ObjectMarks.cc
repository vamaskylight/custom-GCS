#include "ObjectMarks.h"

#include <QtCore/QCoreApplication>
#include <QtCore/QSettings>
#include <QtCore/QVariantMap>
#include <QtQml/QJSEngine>

#include <algorithm>
#include <cmath>
#include <limits>

#include "SkydroidLink.h"

using skydroid::find::Found;
using skydroid::find::Kind;

namespace {

constexpr const char *kSettingsGroup = "VamaMarks";
// After its looks the finder rests this share of the time they took, and at
// least this long: it never takes a whole processor core for itself.
constexpr double kRestShare = 0.3;
constexpr int kLeastRestMs = 60;
// After a picture with no video in it the next one is asked for only after this long:
// getting a picture costs the screen time, and a black video does not come back sooner.
constexpr int kBlankRestMs = 500;
// An object is now and then missed in one picture. A mark that the next looks
// do not find again stays for those looks, and goes with the looks after.
constexpr int kMarkMisses = 1;
// But not when something of its kind is found this near to it, in sizes of the
// old mark: that is the object itself, moved on since the last look (a walker
// goes a body's width in half a second), and its old mark would stand beside it.
constexpr double kSamePlaceSizes = 2.0;
// No mark is shown that is older than this. The camera link keeps pictures for
// 2.5 s: a lock on an older mark could not start on the mark's own picture.
constexpr int kMarkMaxAgeMs = 2200;
constexpr int kAgeTickMs = 200;
// A tap this far outside a mark still means it: a share of the mark's size, and
// at least a share of the picture's width (a finger is wider than a small mark).
constexpr double kTapGrowShare = 0.3;
constexpr double kTapGrowLeast = 0.012;
// The finder gives its memory back when it was not used this long.
constexpr int kIdleRestMs = 30000;
// The two looks at a picture run side by side on a device with this many processor cores.
constexpr int kSideBySideCores = 4;

}  // namespace

ObjectMarks::ObjectMarks(SkydroidLink *link, PictureFinder *finder, bool ownThread, QObject *parent)
    : QObject(parent)
    , _link(link)
{
    qRegisterMetaType<MarksAnswer>("MarksAnswer");
    _worker = new MarksWorker(finder, ownThread && QThread::idealThreadCount() >= kSideBySideCores);
    if (ownThread) {
        _worker->moveToThread(&_thread);
        connect(&_thread, &QThread::finished, _worker, &QObject::deleteLater);
        _thread.setObjectName(QStringLiteral("VamaMarks"));
        // Below the threads that show the video and answer the operator.
        _thread.start(QThread::LowPriority);
    } else {
        _worker->setParent(this);
    }
    connect(this, &ObjectMarks::lookWanted, _worker, &MarksWorker::look, Qt::QueuedConnection);
    connect(this, &ObjectMarks::restWanted, _worker, &MarksWorker::rest, Qt::QueuedConnection);
    connect(_worker, &MarksWorker::looked, this, &ObjectMarks::_looked, Qt::QueuedConnection);
    // A zoom step, or the other lens: what was found is somewhere else in the picture now.
    connect(_link, &SkydroidLink::zoomAsked, this, &ObjectMarks::_clearMarks);
    connect(_link, &SkydroidLink::thermalPictureChanged, this, &ObjectMarks::_clearMarks);
    _ageTimer.setInterval(kAgeTickMs);
    connect(&_ageTimer, &QTimer::timeout, this, &ObjectMarks::_dropOldMarks);
    _idleTimer.setSingleShot(true);
    _idleTimer.setInterval(kIdleRestMs);
    connect(&_idleTimer, &QTimer::timeout, this, &ObjectMarks::restWanted);

    QSettings settings;
    settings.beginGroup(QLatin1String(kSettingsGroup));
    _enabled = settings.value(QStringLiteral("enabled"), true).toBool();
    settings.endGroup();
}

ObjectMarks::~ObjectMarks()
{
    _thread.quit();
    _thread.wait();
}

ObjectMarks *ObjectMarks::instance()
{
    static ObjectMarks *marks = new ObjectMarks(SkydroidLink::instance(), vamaMakePictureFinder(), true,
                                                QCoreApplication::instance());
    return marks;
}

ObjectMarks *ObjectMarks::create(QQmlEngine *qmlEngine, QJSEngine *jsEngine)
{
    Q_UNUSED(qmlEngine);
    Q_UNUSED(jsEngine);
    ObjectMarks *marks = instance();
    QJSEngine::setObjectOwnership(marks, QJSEngine::CppOwnership);
    return marks;
}

qint64 ObjectMarks::_now() const
{
    return _link->pictureClockMs();
}

void ObjectMarks::setEnabled(bool enabled)
{
    if (enabled == _enabled) {
        return;
    }
    _enabled = enabled;
    QSettings settings;
    settings.beginGroup(QLatin1String(kSettingsGroup));
    settings.setValue(QStringLiteral("enabled"), _enabled);
    settings.endGroup();
    emit enabledChanged();
    _update();
}

void ObjectMarks::setLooking(bool looking)
{
    if (looking == _looking) {
        return;
    }
    _looking = looking;
    emit lookingChanged();
    _update();
}

void ObjectMarks::_update()
{
    const bool run = _enabled && _looking && _usable;
    if (run == _running) {
        return;
    }
    _running = run;
    // The link keeps the pictures a lock on a mark starts with, for as long as marks are shown.
    _link->keepPictures(run);
    _answered = false;
    _trouble.clear();
    _closeLook = QRectF();
    if (run) {
        _startedAtMs = _now();
        _nextLookAtMs = 0;
        _ageTimer.start();
        _idleTimer.stop();
    } else {
        _ageTimer.stop();
        _idleTimer.start();
    }
    _clearMarks();
    emit runningChanged();
    emit stateChanged();
}

bool ObjectMarks::wantsPicture() const
{
    return _running && !_busy && _now() >= _nextLookAtMs;
}

void ObjectMarks::picture(const QImage &image, double pictureX, double pictureY, double pictureW, double pictureH,
                          double grabMs)
{
    if (!wantsPicture() || image.isNull()) {
        return;
    }
    _grabMs = grabMs > 0.0 && std::isfinite(grabMs) ? grabMs : 0.0;
    _busy = true;
    emit lookWanted(image, QRectF(pictureX, pictureY, pictureW, pictureH), _unitMs, _now());
}

void ObjectMarks::setIdleRestMs(int ms)
{
    _idleTimer.setInterval(std::max(ms, 1));
}

bool ObjectMarks::takesThePlaceOf(const Found &found, const Found &oldMark)
{
    return found.kind == oldMark.kind && std::abs(found.cx - oldMark.cx) <= kSamePlaceSizes * oldMark.w &&
           std::abs(found.cy - oldMark.cy) <= kSamePlaceSizes * oldMark.h;
}

int ObjectMarks::restAfter(double lookMs)
{
    if (!(lookMs > 0.0) || !std::isfinite(lookMs)) {
        return kLeastRestMs;
    }
    return std::max(kLeastRestMs, static_cast<int>(std::lround(std::min(lookMs, 60000.0) * kRestShare)));
}

void ObjectMarks::_looked(const MarksAnswer &answer)
{
    _busy = false;
    if (!answer.whyNot.isEmpty()) {
        // The finder cannot work in this app: say so once, and ask for no more pictures.
        if (_usable) {
            _usable = false;
            _whyNot = answer.whyNot;
            emit stateChanged();
            _update();
        }
        emit lookDone();
        return;
    }
    _lookMs = answer.lookMs;
    if (answer.unitMs > 0.0 && std::isfinite(answer.unitMs)) {
        _unitMs = _unitMs > 0.0 ? 0.7 * _unitMs + 0.3 * answer.unitMs : answer.unitMs;
    }
    _nextLookAtMs = _now() + (answer.blank ? kBlankRestMs : restAfter(answer.lookMs));
    if (!_running || answer.atMs < _startedAtMs) {
        emit lookDone();
        return;  // a picture from before the operator asked: its marks are not shown
    }
    _answered = true;
    _blank = answer.blank;
    _trouble = answer.trouble;
    _closeLook = answer.closer;
    if (answer.aspect > 0.0 && std::isfinite(answer.aspect)) {
        _aspect = answer.aspect;
    }
    std::vector<Mark> next;
    next.reserve(answer.found.size() + _marks.size());
    for (const Found &f : answer.found) {
        next.push_back(Mark{f, answer.atMs, 0});
    }
    for (const Mark &old : _marks) {
        if (old.missed >= kMarkMisses || answer.blank) {
            continue;  // (with no video in the picture no mark is held: there is nothing under it)
        }
        bool foundAgain = false;
        for (const Found &f : answer.found) {
            if (takesThePlaceOf(f, old.box)) {
                foundAgain = true;
                break;
            }
        }
        if (!foundAgain) {
            next.push_back(Mark{old.box, old.atMs, old.missed + 1});
        }
    }
    _marks.swap(next);
    _dropOldMarks();
    emit marksChanged();
    emit stateChanged();
    emit lookDone();
}

void ObjectMarks::_dropOldMarks()
{
    const qint64 now = _now();
    const auto old = [now](const Mark &mark) { return now - mark.atMs > kMarkMaxAgeMs; };
    const auto firstOld = std::remove_if(_marks.begin(), _marks.end(), old);
    if (firstOld != _marks.end()) {
        _marks.erase(firstOld, _marks.end());
        emit marksChanged();
        emit stateChanged();
    }
}

void ObjectMarks::_clearMarks()
{
    if (!_marks.empty()) {
        _marks.clear();
        emit marksChanged();
        emit stateChanged();
    }
}

QVariantList ObjectMarks::marks() const
{
    QVariantList list;
    list.reserve(static_cast<qsizetype>(_marks.size()));
    for (const Mark &mark : _marks) {
        QVariantMap map;
        map.insert(QStringLiteral("u"), mark.box.cx);
        map.insert(QStringLiteral("v"), mark.box.cy);
        map.insert(QStringLiteral("w"), mark.box.w);
        map.insert(QStringLiteral("h"), mark.box.h);
        map.insert(QStringLiteral("kind"), QString::fromLatin1(skydroid::find::kindName(mark.box.kind)));
        map.insert(QStringLiteral("score"), mark.box.score);
        map.insert(QStringLiteral("sure"), mark.box.score >= kSureScore);
        list.append(map);
    }
    return list;
}

QString ObjectMarks::state() const
{
    if (!_usable) {
        return tr("People and vehicles cannot be marked: %1.").arg(_whyNot);
    }
    if (!_running) {
        return QString();
    }
    if (!_answered) {
        return tr("Looking for people and vehicles...");
    }
    if (_blank) {
        return tr("The video picture does not reach the app (it is black), so nothing can be marked.");
    }
    int people = 0;
    int vehicles = 0;
    int unsurePeople = 0;
    int unsureVehicles = 0;
    for (const Mark &mark : _marks) {
        const bool unsure = mark.box.score < kSureScore;
        if (mark.box.kind == Kind::Person) {
            ++people;
            unsurePeople += unsure ? 1 : 0;
        } else {
            ++vehicles;
            unsureVehicles += unsure ? 1 : 0;
        }
    }
    const auto unsureNote = [](int unsure) {
        return unsure > 0 ? QStringLiteral(" (") + tr("%1 unsure").arg(unsure) + QLatin1Char(')') : QString();
    };
    QStringList parts;
    if (people > 0) {
        parts.append((people == 1 ? tr("1 person") : tr("%1 people").arg(people)) + unsureNote(unsurePeople));
    }
    if (vehicles > 0) {
        parts.append((vehicles == 1 ? tr("1 vehicle") : tr("%1 vehicles").arg(vehicles)) + unsureNote(unsureVehicles));
    }
    QString text = parts.isEmpty() ? tr("No person or vehicle found.") : tr("Marked: %1.").arg(parts.join(QStringLiteral(", ")));
    if (!_trouble.isEmpty()) {
        text += QLatin1Char(' ') + tr("The finder has trouble: %1.").arg(_trouble);
    }
    text += QLatin1Char(' ') + tr("A look takes %1 s.").arg(_lookMs / 1000.0, 0, 'f', 1);
    if (_grabMs > 0.0) {
        text += QLatin1Char(' ') + tr("Getting a picture takes %1 s.").arg(_grabMs / 1000.0, 0, 'f', 2);
    }
    return text;
}

int ObjectMarks::markAt(const std::vector<Mark> &marks, double u, double v, double aspect)
{
    if (!std::isfinite(u) || !std::isfinite(v)) {
        return -1;
    }
    // Inside a mark: that one. Inside several (a rider on a motorcycle): the smallest.
    int best = -1;
    double smallest = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < marks.size(); ++i) {
        const Found &box = marks[i].box;
        if (std::abs(u - box.cx) <= box.w / 2.0 && std::abs(v - box.cy) <= box.h / 2.0 && box.w * box.h < smallest) {
            smallest = box.w * box.h;
            best = static_cast<int>(i);
        }
    }
    if (best >= 0) {
        return best;
    }
    // Just beside one: the nearest.
    const double shape = aspect > 0.0 && std::isfinite(aspect) ? aspect : 16.0 / 9.0;
    double nearest = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < marks.size(); ++i) {
        const Found &box = marks[i].box;
        const double besideU = std::max(0.0, std::abs(u - box.cx) - box.w / 2.0);
        const double besideV = std::max(0.0, std::abs(v - box.cy) - box.h / 2.0);
        if (besideU > std::max(kTapGrowShare * box.w, kTapGrowLeast) ||
            besideV > std::max(kTapGrowShare * box.h, kTapGrowLeast * shape)) {
            continue;
        }
        // Both in parts of the picture's width.
        const double gap = std::hypot(besideU, besideV / shape);
        if (gap < nearest) {
            nearest = gap;
            best = static_cast<int>(i);
        }
    }
    return best;
}

QString ObjectMarks::notices() const
{
    return vamaFinderNotices();
}

QString ObjectMarks::finderName() const
{
    return vamaFinderName();
}

bool ObjectMarks::lockAt(double u, double v)
{
    // (No marks, no lock: when the marks do not run, there are none.)
    const int index = markAt(_marks, u, v, _aspect);
    if (index < 0) {
        return false;
    }
    const Mark mark = _marks[static_cast<std::size_t>(index)];
    _link->lockAtFound(mark.box.cx, mark.box.cy, mark.box.w, mark.box.h, mark.atMs);
    return true;
}
