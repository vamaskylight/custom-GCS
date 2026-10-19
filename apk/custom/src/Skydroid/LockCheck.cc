#include "LockCheck.h"

#include <algorithm>
#include <cmath>

namespace skydroid::check {

using find::Found;
using find::Kind;

void LockCheck::start(std::optional<Kind> kind, double score)
{
    _kind = kind;
    _score = kind && std::isfinite(score) ? std::clamp(score, 0.0, 1.0) : 0.0;
    _adopting = !kind.has_value();
    _adoptLooks = 0;
    _misses = 0;
}

void LockCheck::stop()
{
    _kind.reset();
    _score = 0.0;
    _adopting = false;
    _adoptLooks = 0;
    _misses = 0;
}

bool LockCheck::largeEnough(Kind kind, const Box &box, double aspect)
{
    const double shape = aspect > 0.0 && std::isfinite(aspect) ? aspect : 16.0 / 9.0;
    // In points of a picture 720 points high, whose width is 720 times its shape.
    const double side = kind == Kind::Person ? box.h : std::max(box.h, box.w * shape);
    return std::isfinite(side) && side * kPictureHeightPoints >= kLeastSidePoints;
}

bool LockCheck::onTheBox(const Found &found, const Box &box)
{
    if (!(box.w > 0.0) || !(box.h > 0.0) || !(found.h > 0.0)) {
        return false;
    }
    const double ratio = found.h / box.h;
    return std::abs(found.cx - box.cx) <= box.w * (0.5 + kOnBoxGrow) &&
           std::abs(found.cy - box.cy) <= box.h * (0.5 + kOnBoxGrow) && ratio >= 1.0 / kSizeFactor &&
           ratio <= kSizeFactor;
}

double LockCheck::reachWidths(double sinceSeenS)
{
    const double s = std::isfinite(sinceSeenS) ? std::max(0.0, sinceSeenS) : 0.0;
    return std::min(kReachWidthsMost, kReachWidths + kReachWidthsPerS * s);
}

double LockCheck::reachHeights(double sinceSeenS)
{
    const double s = std::isfinite(sinceSeenS) ? std::max(0.0, sinceSeenS) : 0.0;
    return std::min(kReachHeightsMost, kReachHeights + kReachHeightsPerS * s);
}

bool LockCheck::_sureEnough(const Found &found) const
{
    return found.score >= std::max(kScoreLeast, _score - kScoreDrop);
}

Answer LockCheck::look(const std::vector<Found> &found, const Box &box, bool seen, const Box &lastSeen,
                       double sinceSeenS, double aspect)
{
    Answer answer;
    if (_adopting) {
        // A drawn box: the surest person or vehicle on it says what the box holds.
        const Found *best = nullptr;
        for (const Found &f : found) {
            if (onTheBox(f, box) && largeEnough(f.kind, box, aspect) && (!best || f.score > best->score)) {
                best = &f;
            }
        }
        if (best) {
            _kind = best->kind;
            _score = std::clamp(best->score, 0.0, 1.0);
            _adopting = false;
            _misses = 0;
            answer.verdict = Verdict::OnTheBox;
        } else if (++_adoptLooks >= kAdoptLooks) {
            _adopting = false;  // something else: followed as before, without the finder
        }
        return answer;
    }
    if (!_kind) {
        return answer;
    }
    // Too small for the finder to see well: no check, and no doubt from before.
    if (!largeEnough(*_kind, seen ? box : lastSeen, aspect)) {
        _misses = 0;
        return answer;
    }
    for (const Found &f : found) {
        if (f.kind == *_kind && _sureEnough(f) && onTheBox(f, box)) {
            _misses = 0;
            answer.verdict = Verdict::OnTheBox;
            return answer;
        }
    }
    ++_misses;
    if ((!seen || _misses >= kMissesNotThere) && lastSeen.w > 0.0 && lastSeen.h > 0.0) {
        // Not seen, or not there: one of its kind, of about its size, near where it was seen last.
        // (One on the box itself would have been taken above: the finder is not sure enough of it.)
        const double reachW = reachWidths(sinceSeenS) * lastSeen.w;
        const double reachH = reachHeights(sinceSeenS) * lastSeen.h;
        const Found *nearest = nullptr;
        double nearestGap = 0.0;
        for (const Found &f : found) {
            const double dx = std::abs(f.cx - lastSeen.cx);
            const double dy = std::abs(f.cy - lastSeen.cy);
            const double ratio = f.h / lastSeen.h;
            if (f.kind != *_kind || !_sureEnough(f) || dx > reachW || dy > reachH || !(ratio >= 1.0 / kSizeFactor) ||
                ratio > kSizeFactor) {
                continue;
            }
            const double gap = std::hypot(dx / lastSeen.w, dy / lastSeen.h);
            if (!nearest || gap < nearestGap) {
                nearest = &f;
                nearestGap = gap;
            }
        }
        if (nearest) {
            _misses = 0;
            _score = std::clamp(nearest->score, 0.0, 1.0);
            answer.verdict = Verdict::FoundAgain;
            answer.again = Box{nearest->cx, nearest->cy, nearest->w, nearest->h};
            answer.againScore = nearest->score;
            return answer;
        }
    }
    answer.verdict = _misses >= kMissesNotThere ? Verdict::NotThere : Verdict::Missed;
    return answer;
}

}  // namespace skydroid::check
