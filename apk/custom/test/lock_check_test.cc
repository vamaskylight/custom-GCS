// Host unit test for the finder's check of an object lock (LockCheck.h, task AC):
// made-up looks of the finder, so every case knows what should come out.

#include "LockCheck.h"

#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

using namespace skydroid::check;
using skydroid::find::Found;
using skydroid::find::Kind;

namespace {

int g_checks = 0;
int g_failures = 0;
bool g_verbose = false;  // "-v": print every case

void check(bool ok, const char *what)
{
    ++g_checks;
    if (!ok) {
        ++g_failures;
        std::printf("FAIL %s\n", what);
    } else if (g_verbose) {
        std::printf("ok   %s\n", what);
    }
}

bool near(double a, double b, double by = 1e-9)
{
    return std::abs(a - b) <= by;
}

constexpr double kAspect = 16.0 / 9.0;

/// A box of the finder, in parts of the picture.
Found found(Kind kind, double score, double cx, double cy, double w, double h)
{
    Found f;
    f.kind = kind;
    f.label = kind == Kind::Person ? 0 : 2;
    f.score = score;
    f.cx = cx;
    f.cy = cy;
    f.w = w;
    f.h = h;
    return f;
}

// A person 0.2 of the picture high (144 of 720 points), a third as wide, in the middle.
const Box kPersonBox{0.5, 0.5, 0.0375, 0.2};

Found personOn(double score = 0.7, double cx = 0.5, double cy = 0.5, double h = 0.2)
{
    return found(Kind::Person, score, cx, cy, 0.0375 * h / 0.2, h);
}

Verdict lookAt(LockCheck &c, const std::vector<Found> &f, bool seen = true, double sinceS = 0.0,
               const Box &box = kPersonBox, const Box &lastSeen = kPersonBox)
{
    return c.look(f, box, seen, lastSeen, sinceS, kAspect).verdict;
}

void aLockOnAMark()
{
    LockCheck c;
    check(!c.wanted(), "no lock: no looks wanted");
    c.start(Kind::Person, 0.7);
    check(c.wanted() && c.kind() == Kind::Person && near(c.score(), 0.7), "a lock on a person's mark wants the finder's looks");
    check(lookAt(c, {personOn()}) == Verdict::OnTheBox, "the person on the box: on the box");
    check(lookAt(c, {}) == Verdict::Missed && c.misses() == 1, "a look without it: one miss, too few to say");
    check(lookAt(c, {}) == Verdict::Missed && c.misses() == 2, "two misses: still too few");
    check(lookAt(c, {}) == Verdict::NotThere && c.misses() == 3, "three looks in a row without it: not there");
    check(lookAt(c, {}) == Verdict::NotThere && c.misses() == 4, "and more: still not there");
    check(lookAt(c, {personOn()}) == Verdict::OnTheBox && c.misses() == 0, "the person on the box again: on the box, the misses forgotten");
    check(lookAt(c, {}) == Verdict::Missed, "a miss after that counts from one again");
    c.stop();
    check(!c.wanted() && !c.kind(), "no lock any more: no looks wanted");
}

void whatCountsAsTheObject()
{
    LockCheck c;
    c.start(Kind::Person, 0.7);
    check(lookAt(c, {found(Kind::Vehicle, 0.9, 0.5, 0.5, 0.1, 0.2)}) == Verdict::Missed,
          "a vehicle on a person's box is not the person");
    c.start(Kind::Person, 0.7);
    // The board stacks of the client's video: marked as people, but much less sure than the man.
    check(lookAt(c, {personOn(0.45)}) == Verdict::Missed, "a look-alike far less sure (0.45 against 0.70) is not the object");
    check(lookAt(c, {personOn(0.50)}) == Verdict::OnTheBox, "0.20 under how sure it was at the lock still is");
    c.start(Kind::Person, 0.45);
    check(lookAt(c, {personOn(0.40)}) == Verdict::OnTheBox, "a lock made at 0.45: the finder's own limit (0.40) counts");
    c.start(Kind::Person, 0.45);
    check(lookAt(c, {personOn(0.39)}) == Verdict::Missed, "under the finder's own limit nothing counts");
    c.start(Kind::Person, 0.7);
    check(lookAt(c, {personOn(0.7, 0.5, 0.5, 0.41)}) == Verdict::Missed, "a box more than twice as high is something else");
    c.start(Kind::Person, 0.7);
    check(lookAt(c, {personOn(0.7, 0.5, 0.5, 0.1)}) == Verdict::OnTheBox, "half as high still counts (a drawn box may be loose)");
    c.start(Kind::Person, 0.7);
    check(lookAt(c, {personOn(0.7, 0.5, 0.5, 0.099)}) == Verdict::Missed, "less than half as high does not");
    // The box grown by a quarter of its size each way: 0.0375 wide, so 0.028 from its centre.
    c.start(Kind::Person, 0.7);
    check(lookAt(c, {personOn(0.7, 0.5 + 0.0281)}) == Verdict::OnTheBox, "a centre just inside the grown box is on it");
    c.start(Kind::Person, 0.7);
    check(lookAt(c, {personOn(0.7, 0.5 + 0.0282)}) == Verdict::Missed, "just outside is not");
    c.start(Kind::Person, 0.7);
    check(lookAt(c, {personOn(0.7, 0.5, 0.5 + 0.1499)}) == Verdict::OnTheBox, "up and down the same: inside");
    c.start(Kind::Person, 0.7);
    check(lookAt(c, {personOn(0.7, 0.5, 0.5 + 0.1501)}) == Verdict::Missed, "outside");
    c.start(Kind::Person, 0.7);
    check(lookAt(c, {personOn(0.45), personOn(0.8, 0.9)}) == Verdict::Missed,
          "a sure person elsewhere and a look-alike on the box: a miss");
    check(!LockCheck::onTheBox(personOn(), Box{0.5, 0.5, 0.0, 0.2}) && !LockCheck::onTheBox(personOn(0.7, 0.5, 0.5, 0.0), kPersonBox),
          "a box without a size is on nothing");
}

void tooSmallToCheck()
{
    // 48 points of 720 is the least: under that the finder misses most people.
    check(LockCheck::largeEnough(Kind::Person, Box{0.5, 0.5, 0.02, 48.0 / 720.0}, kAspect), "a person 48 points high is checked");
    check(!LockCheck::largeEnough(Kind::Person, Box{0.5, 0.5, 0.02, 47.0 / 720.0}, kAspect), "47 points is too small");
    check(LockCheck::largeEnough(Kind::Vehicle, Box{0.5, 0.5, 49.0 / 1280.0, 30.0 / 720.0}, kAspect),
          "a vehicle counts by its longer side: 49 points long, 30 high");
    check(!LockCheck::largeEnough(Kind::Vehicle, Box{0.5, 0.5, 47.0 / 1280.0, 30.0 / 720.0}, kAspect), "47 long and 30 high is too small");
    check(!LockCheck::largeEnough(Kind::Person, Box{0.5, 0.5, 0.02, std::numeric_limits<double>::quiet_NaN()}, kAspect),
          "no size is too small");

    LockCheck c;
    c.start(Kind::Person, 0.7);
    const Box small{0.5, 0.5, 0.01, 40.0 / 720.0};
    lookAt(c, {}, true, 0.0, kPersonBox, kPersonBox);
    lookAt(c, {}, true, 0.0, kPersonBox, kPersonBox);
    check(c.misses() == 2, "two misses on a large box");
    check(lookAt(c, {}, true, 0.0, small, small) == Verdict::Nothing && c.misses() == 0,
          "the object too small to check: nothing to say, and the misses forgotten");
    check(lookAt(c, {}, false, 1.0, kPersonBox, small) == Verdict::Nothing,
          "not seen: the size where it was seen last decides");
}

void foundAgain()
{
    LockCheck c;
    c.start(Kind::Person, 0.7);
    // Not seen (the tracker lost it): one of its kind near where it was seen last is the object.
    const Answer a = c.look({personOn(0.66, 0.5 + 1.4 * 0.0375)}, kPersonBox, false, kPersonBox, 0.0, kAspect);
    check(a.verdict == Verdict::FoundAgain && near(a.again.cx, 0.5 + 1.4 * 0.0375) && near(a.again.h, 0.2) &&
              near(a.againScore, 0.66),
          "not seen: a person 1.4 widths from where it went is found again, with its box");
    check(near(c.score(), 0.66) && c.misses() == 0, "how sure the finder is of it counts from then on");

    c.start(Kind::Person, 0.7);
    check(lookAt(c, {personOn(0.7, 0.5 + 1.6 * 0.0375)}, false, 0.0) == Verdict::Missed,
          "at once, 1.6 widths away is too far");
    check(lookAt(c, {personOn(0.7, 0.5 + 3.4 * 0.0375)}, false, 1.0) == Verdict::FoundAgain,
          "a second unseen: 3.4 widths is near enough (1.5 and 2 a second)");
    c.start(Kind::Person, 0.7);
    check(lookAt(c, {personOn(0.7, 0.5 + 7.9 * 0.0375)}, false, 10.0) == Verdict::FoundAgain &&
              near(LockCheck::reachWidths(10.0), 8.0),
          "after long, up to 8 widths");
    c.start(Kind::Person, 0.7);
    check(lookAt(c, {personOn(0.7, 0.5 + 8.1 * 0.0375)}, false, 10.0) == Verdict::Missed, "not further");
    c.start(Kind::Person, 0.7);
    check(lookAt(c, {personOn(0.7, 0.5, 0.5 + 0.19)}, false, 0.0) == Verdict::FoundAgain,
          "up and down: within a height at once");
    c.start(Kind::Person, 0.7);
    check(lookAt(c, {personOn(0.7, 0.5, 0.5 + 0.21)}, false, 0.0) == Verdict::Missed, "not more");
    check(near(LockCheck::reachHeights(1.0), 1.5) && near(LockCheck::reachHeights(10.0), 3.0) &&
              near(LockCheck::reachWidths(-1.0), 1.5) && near(LockCheck::reachWidths(std::nan("")), 1.5),
          "the reach up and down grows by half a height a second, to 3; a time that is no time counts as none");

    c.start(Kind::Person, 0.7);
    const Answer two = c.look({personOn(0.7, 0.5 + 1.2 * 0.0375), personOn(0.7, 0.5 - 0.8 * 0.0375)}, kPersonBox, false, kPersonBox,
                              0.0, kAspect);
    check(two.verdict == Verdict::FoundAgain && near(two.again.cx, 0.5 - 0.8 * 0.0375), "of two, the nearest");
    c.start(Kind::Person, 0.7);
    check(lookAt(c, {found(Kind::Vehicle, 0.9, 0.52, 0.5, 0.0375, 0.2)}, false, 0.0) == Verdict::Missed, "only its own kind");
    c.start(Kind::Person, 0.7);
    check(lookAt(c, {personOn(0.45, 0.53)}, false, 0.0) == Verdict::Missed, "only one the finder is sure enough of");
    c.start(Kind::Person, 0.7);
    check(lookAt(c, {personOn(0.7, 0.53, 0.5, 0.45)}, false, 0.0) == Verdict::Missed, "only one of about its size");
    c.start(Kind::Person, 0.7);
    check(c.look({personOn(0.7, 0.53)}, kPersonBox, false, Box{}, 0.0, kAspect).verdict == Verdict::Nothing,
          "not seen, and nowhere seen before: nothing to say");

    // Seen by the tracker (on a look-alike): another person beside it is not taken until the box
    // has been without the object three times.
    c.start(Kind::Person, 0.7);
    const std::vector<Found> beside{personOn(0.45), personOn(0.7, 0.5 + 1.2 * 0.0375)};
    check(lookAt(c, beside, true) == Verdict::Missed, "seen: a person beside the box is no reason after one miss");
    check(lookAt(c, beside, true) == Verdict::Missed, "nor after two");
    check(lookAt(c, beside, true) == Verdict::FoundAgain, "after the third, the person beside it is the object");
    // Not there for a while: the reach grows from where it was seen last.
    c.start(Kind::Person, 0.7);
    lookAt(c, {}, true);
    lookAt(c, {}, true);
    check(lookAt(c, {}, true) == Verdict::NotThere, "three misses: not there");
    const Box lookAlike{0.5, 0.5, 0.0375, 0.2};
    check(c.look({personOn(0.7, 0.5 + 4.0 * 0.0375)}, lookAlike, false, kPersonBox, 2.0, kAspect).verdict == Verdict::FoundAgain,
          "two seconds later the person 4 widths on is found again");
}

void aDrawnBox()
{
    LockCheck c;
    c.start();
    check(c.wanted() && !c.kind(), "a drawn box: looks are wanted, what it holds is not known yet");
    const Box drawn{0.5, 0.5, 0.06, 0.25};
    check(c.look({}, drawn, true, drawn, 0.0, kAspect).verdict == Verdict::Nothing, "a look with nothing on it says nothing");
    check(c.look({found(Kind::Person, 0.48, 0.5, 0.52, 0.03, 0.22), found(Kind::Vehicle, 0.55, 0.5, 0.5, 0.05, 0.2)}, drawn, true,
                 drawn, 0.0, kAspect)
                  .verdict == Verdict::OnTheBox &&
              c.kind() == Kind::Vehicle && near(c.score(), 0.55),
          "the surest on it (not the first) says what it holds, and how sure the finder is of it");
    check(c.look({}, drawn, true, drawn, 0.0, kAspect).verdict == Verdict::Missed, "from then on it is checked");

    c.start();
    for (int i = 0; i < 2; ++i) {
        c.look({found(Kind::Person, 0.9, 0.9, 0.2, 0.03, 0.2)}, drawn, true, drawn, 0.0, kAspect);
    }
    check(c.wanted(), "nothing on it in two looks: still asked");
    c.look({}, drawn, true, drawn, 0.0, kAspect);
    check(!c.wanted() && !c.kind(), "nothing on it in three: something else, followed without the finder");
    check(c.look({personOn()}, drawn, true, drawn, 0.0, kAspect).verdict == Verdict::Nothing, "and later looks say nothing");

    c.start();
    const Box far{0.5, 0.5, 0.01, 0.05};
    c.look({found(Kind::Person, 0.9, 0.5, 0.5, 0.01, 0.05)}, far, true, far, 0.0, kAspect);
    check(!c.kind(), "a person too small to check does not make it a person's lock");
    c.start(std::nullopt, 0.9);
    check(near(c.score(), 0.0), "without a kind, no score");
    c.start(Kind::Person, std::nan(""));
    check(near(c.score(), 0.0) && lookAt(c, {personOn(0.40)}) == Verdict::OnTheBox, "a score that is no number counts as none");
}

}  // namespace

int main(int argc, char **argv)
{
    g_verbose = argc > 1 && argv[1][0] == '-' && argv[1][1] == 'v';
    aLockOnAMark();
    whatCountsAsTheObject();
    tooSmallToCheck();
    foundAgain();
    aDrawnBox();
    std::printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
