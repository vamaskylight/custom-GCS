#pragma once

// The object finder's check of an object lock (task AC, client videos of 2026-10-19).
//
// The app's tracker (ObjectTracker.h) follows what looks like the object. Past a
// look-alike (a stack of brown boards) or behind a thin tree it can stay on the
// look-alike, learn it, and say that it follows. While a person or a vehicle is
// locked, the object finder (ObjectFinder.h) looks at the video about once a
// second, and this decides from each of its looks:
//  - the object of the lock's kind is on the box: all is well,
//  - it has not been on the box in kMissesNotThere looks in a row: the box holds
//    something else. The screen says NOT SEEN, the camera waits, and the lock ends
//    as for an object that is not seen (the lock's wait),
//  - the object is not seen (the tracker lost it, or after such misses), and the
//    finder sees one of its kind near where it went: the lock goes on there. (The
//    operator asked for this: "if the object appears again then it will detect
//    again and track that person automatically".)
// Once the lock has ended, nothing is locked by itself: a mark is an offer to the
// operator.
//
// Objects the finder sees badly are not checked: under kLeastSidePoints in the
// camera's picture it misses most people (measured 2026-10-13 and 2026-10-20). A
// look-alike it is much less sure of than of the object at the lock does not count
// as the object (kScoreDrop): the board stacks were marked as people, but less sure.
// A lock on a drawn box is checked when one of the finder's first looks sees a
// person or a vehicle on the box; on anything else (a boat, a roof) it is not.
//
// Plain C++, no Qt. Boxes are in parts of the video picture (0 to 1).

#include <optional>
#include <vector>

#include "ObjectFinder.h"

namespace skydroid::check {

/// A box in parts of the video picture: its centre and its size.
struct Box
{
    double cx = 0.0;
    double cy = 0.0;
    double w = 0.0;
    double h = 0.0;
};

/// What a look of the finder says about the lock.
enum class Verdict
{
    Nothing,     // no check: the lock has no kind (yet), or the object is too small to check
    OnTheBox,    // the object of the lock's kind is on the box
    Missed,      // it is not, in fewer than kMissesNotThere looks in a row: too few to say
    NotThere,    // it has not been on the box in kMissesNotThere looks in a row or more
    FoundAgain,  // the object is not seen, and one of its kind is near where it went: lock that
};

struct Answer
{
    Verdict verdict = Verdict::Nothing;
    Box again;               // FoundAgain: where the object is, in the picture looked at
    double againScore = 0.0;  // and how sure the finder is of it
};

// Looks in a row without the object on the box before the box counts as holding something else.
// (The finder misses an object it sees well now and then: one or two missed looks are no sign.)
constexpr int kMissesNotThere = 3;
// Looks at the start of a lock on a drawn box in which the finder may say what the box holds.
constexpr int kAdoptLooks = 3;
// The smallest object checked: its height (a vehicle: its longer side), in points of the camera's
// picture of 720 points. The finder found about 7 in 10 people of 48 to 96 points, fewer of smaller.
constexpr double kLeastSidePoints = 48.0;
constexpr double kPictureHeightPoints = 720.0;
// A box of the finder is on the lock's box when its centre lies inside the lock box grown by this
// share of its size each way, and its height is within this factor of the lock box's.
constexpr double kOnBoxGrow = 0.25;
constexpr double kSizeFactor = 2.0;
// How sure the finder must be of a box for it to count as the object: no less than this under how
// sure it was at the lock, and never under its own limit (scoreMin in ObjectFinder.h).
constexpr double kScoreDrop = 0.20;
constexpr double kScoreLeast = 0.40;
// Where the object may come out again: around where it was last seen, this many of its widths to
// each side and heights up and down, growing for each second it is not seen, up to the most.
constexpr double kReachWidths = 1.5;
constexpr double kReachWidthsPerS = 2.0;
constexpr double kReachWidthsMost = 8.0;
constexpr double kReachHeights = 1.0;
constexpr double kReachHeightsPerS = 0.5;
constexpr double kReachHeightsMost = 3.0;

class LockCheck
{
public:
    /// A new lock. kind and score: of the mark it was made from. Without a kind (a drawn box) the
    /// finder's first looks may say what the box holds.
    void start(std::optional<find::Kind> kind = std::nullopt, double score = 0.0);
    /// No lock: no looks are wanted.
    void stop();

    /// One look of the finder at a picture.
    /// found: what it found there. box: the lock box in that picture. seen: the app saw the object
    /// there (the tracker found it, and no NotThere since). lastSeen and sinceSeenS: where the app saw
    /// it last, and how long ago in seconds (0 while it is seen). aspect: the picture's width over
    /// its height.
    Answer look(const std::vector<find::Found> &found, const Box &box, bool seen, const Box &lastSeen,
                double sinceSeenS, double aspect);

    /// The finder's looks are wanted: the lock has a kind, or may still get one.
    bool wanted() const { return _kind.has_value() || _adopting; }
    std::optional<find::Kind> kind() const { return _kind; }
    /// How sure the finder was of the object at the lock (or when it was found again).
    double score() const { return _score; }
    int misses() const { return _misses; }

    /// True when a box is large enough for the finder to see an object of this kind in it well.
    static bool largeEnough(find::Kind kind, const Box &box, double aspect);
    /// True when a box of the finder is on the lock's box (kOnBoxGrow, kSizeFactor).
    static bool onTheBox(const find::Found &found, const Box &box);
    /// How far to the side and up and down the object may have gone after this long unseen, in its
    /// widths and heights.
    static double reachWidths(double sinceSeenS);
    static double reachHeights(double sinceSeenS);

private:
    bool _sureEnough(const find::Found &found) const;

    std::optional<find::Kind> _kind;
    double _score = 0.0;
    bool _adopting = false;  // a drawn box: what it holds is not known yet
    int _adoptLooks = 0;
    int _misses = 0;
};

}  // namespace skydroid::check
