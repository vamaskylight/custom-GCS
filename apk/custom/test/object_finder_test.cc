// Host unit test for what the object finder makes of the network's answer:
// made-up answers, so every case knows what should come out. The network itself
// (and the ncnn library) is in object_finder_net_test.cc, with real pictures.

#include "ObjectFinder.h"

#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

using namespace skydroid::find;

namespace {

int g_checks = 0;
int g_failures = 0;
bool g_verbose = false;  // "-v": print every case

void check(bool ok, const char *what, const char *detail = "")
{
    ++g_checks;
    if (!ok) {
        ++g_failures;
        std::printf("FAIL %s %s\n", what, detail);
    } else if (g_verbose) {
        std::printf("ok   %s %s\n", what, detail);
    }
}

bool near(double a, double b, double by = 1e-6)
{
    return std::abs(a - b) <= by;
}

/// A network answer for an input of this size: nothing found until places are set.
struct Answer
{
    int inputW;
    int inputH;
    std::vector<float> rows;

    Answer(int w, int h) : inputW(w), inputH(h), rows(static_cast<std::size_t>(rowsFor(w, h)) * kRowLength, 0.0f) {}

    int count() const { return static_cast<int>(rows.size()) / kRowLength; }

    /// The row of a place: grid 0 to 3, x and y in grid steps.
    float *row(int grid, int x, int y)
    {
        int before = 0;
        for (int g = 0; g < grid; ++g) {
            const int step = kGridStep[g];
            before += ((inputW + step - 1) / step) * ((inputH + step - 1) / step);
        }
        const int across = (inputW + kGridStep[grid] - 1) / kGridStep[grid];
        return rows.data() + static_cast<std::size_t>(before + y * across + x) * kRowLength;
    }

    /// An object at a place: its class, how sure, and how many grid steps each side is away
    /// (left, top, right, bottom). Whole steps are told by one sure number; a part of a
    /// step by two numbers that share the weight.
    void set(int grid, int x, int y, int label, float score, double left, double top, double right, double bottom)
    {
        float *r = row(grid, x, y);
        r[label] = score;
        const double sides[4] = {left, top, right, bottom};
        for (int s = 0; s < 4; ++s) {
            float *steps = r + kClasses + s * kSideSteps;
            for (int i = 0; i < kSideSteps; ++i) {
                steps[i] = -50.0f;
            }
            const int whole = static_cast<int>(std::floor(sides[s]));
            const double part = sides[s] - whole;
            if (part < 1e-9) {
                steps[whole] = 0.0f;
            } else {
                steps[whole] = static_cast<float>(std::log(1.0 - part));
                steps[whole + 1] = static_cast<float>(std::log(part));
            }
        }
    }

    std::vector<Found> decoded(double scoreMin = 0.4, bool *ok = nullptr) const
    {
        std::vector<Found> out;
        const bool done = decode(rows.data(), count(), kRowLength, inputW, inputH, scoreMin, out);
        if (ok) {
            *ok = done;
        }
        return out;
    }
};

Found box(Kind kind, double score, double cx, double cy, double w, double h)
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

void theAnswersSize()
{
    check(rowsFor(416, 416) == 52 * 52 + 26 * 26 + 13 * 13 + 7 * 7, "an input of 416 x 416 has 3598 places");
    check(rowsFor(640, 384) == 80 * 48 + 40 * 24 + 20 * 12 + 10 * 6, "an input of 640 x 384 has 5100 places");
    check(rowsFor(64, 64) == 64 + 16 + 4 + 1, "the smallest input has 85 places");
    check(kRowLength == 112, "a row holds 80 classes and 4 sides of 8 steps");
}

void kinds()
{
    Kind kind = Kind::Vehicle;
    check(kindOf(0, kind) && kind == Kind::Person, "class 0 is a person");
    const int vehicles[] = {2, 3, 5, 7};
    bool all = true;
    for (const int label : vehicles) {
        Kind k = Kind::Person;
        all = all && kindOf(label, k) && k == Kind::Vehicle;
    }
    check(all, "car, motorcycle, bus and truck are vehicles");
    int others = 0;
    for (int label = -1; label <= kClasses; ++label) {
        Kind k = Kind::Person;
        if (label != 0 && label != 2 && label != 3 && label != 5 && label != 7 && kindOf(label, k)) {
            ++others;
        }
    }
    check(others == 0, "no other class is marked (a bicycle, a boat, a dog)");
    check(std::string(kindName(Kind::Person)) == "person" && std::string(kindName(Kind::Vehicle)) == "vehicle",
          "the kinds have their names");
}

void layingAPictureIn()
{
    {
        const Fit fit = fitArea(1280, 720, 640, 384, true);
        check(fit.inputW == 640 && fit.inputH == 384 && fit.filledW == 640 && fit.filledH == 360 && fit.padLeft == 0 &&
                  fit.padTop == 12 && near(fit.scaleX, 0.5) && near(fit.scaleY, 0.5),
              "a 16:9 picture keeps its shape in a 640 x 384 input: bars of 12 points above and below");
    }
    {
        const Fit fit = fitArea(640, 512, 640, 384, true);
        check(fit.filledW == 480 && fit.filledH == 384 && fit.padLeft == 80 && fit.padTop == 0 && near(fit.scaleX, 0.75) &&
                  near(fit.scaleY, 0.75),
              "a 5:4 thermal picture gets bars at its sides");
    }
    {
        const Fit fit = fitArea(1280, 720, 512, 384, false);
        check(fit.filledW == 512 && fit.filledH == 384 && fit.padLeft == 0 && fit.padTop == 0 && near(fit.scaleX, 0.4) &&
                  near(fit.scaleY, 384.0 / 720.0),
              "a picture that fills the input is stretched, each way by its own factor");
    }
    {
        const Fit fit = fitArea(640, 384, 640, 384, true);
        check(fit.filledW == 640 && fit.filledH == 384 && near(fit.scaleX, 1.0) && near(fit.scaleY, 1.0),
              "a part of the input's own size goes in point for point");
    }
    {
        const Fit a = fitArea(100, 100, 630, 100, true);
        const Fit b = fitArea(100, 100, 10, 5000, true);
        const Fit c = fitArea(100, 100, -5, 0, true);
        check(a.inputW == 640 && a.inputH == 96, "an input size is brought to the nearest multiple of 32");
        check(b.inputW == kInputMin && b.inputH == kInputMax, "and into the sizes the network takes");
        check(c.inputW == kInputMin && c.inputH == kInputMin, "also from a size that is none");
    }
    {
        const Fit fit = fitArea(0, 720, 640, 384, true);
        check(fit.filledW == 0 && fit.filledH == 0, "nothing is laid in from an empty part");
    }
    {
        const Fit fit = fitArea(4000, 10, 640, 384, true);
        check(fit.filledW == 640 && fit.filledH >= 1, "a very flat part still gets a row");
    }
}

void readingTheAnswer()
{
    {
        Answer answer(640, 384);
        // Grid 1 (16 points a step), place 10, 6: the place is at 160, 96.
        answer.set(1, 10, 6, 0, 0.8f, 1, 2, 3, 4);
        bool ok = false;
        const std::vector<Found> found = answer.decoded(0.4, &ok);
        check(ok && found.size() == 1, "one object in the answer gives one box");
        if (found.size() == 1) {
            const Found &f = found[0];
            // Left 160 - 16, top 96 - 32, right 160 + 48, bottom 96 + 64.
            check(f.kind == Kind::Person && f.label == 0 && near(f.score, 0.8, 1e-6), "it is a person, as sure as told");
            check(near(f.cx, 176.0, 1e-3) && near(f.cy, 112.0, 1e-3) && near(f.w, 64.0, 1e-3) && near(f.h, 96.0, 1e-3),
                  "its box is the place and its four sides, in input points");
        }
    }
    {
        Answer answer(640, 384);
        answer.set(0, 20, 10, 2, 0.6f, 1.25, 0.5, 2.75, 3.0);
        const std::vector<Found> found = answer.decoded();
        // Place 160, 80, 8 points a step: left 150, top 76, right 182, bottom 104.
        check(found.size() == 1 && found[0].kind == Kind::Vehicle && near(found[0].cx, 166.0, 1e-3) &&
                  near(found[0].cy, 90.0, 1e-3) && near(found[0].w, 32.0, 1e-3) && near(found[0].h, 28.0, 1e-3),
              "a side between two steps is taken between them");
    }
    {
        // One object in each grid, each with its own step.
        Answer answer(640, 384);
        answer.set(0, 79, 47, 0, 0.9f, 1, 1, 0, 0);
        answer.set(1, 39, 23, 2, 0.9f, 1, 1, 0, 0);
        answer.set(2, 19, 11, 5, 0.9f, 1, 1, 0, 0);
        answer.set(3, 9, 5, 7, 0.9f, 1, 1, 0, 0);
        const std::vector<Found> found = answer.decoded();
        bool right = found.size() == 4;
        for (std::size_t i = 0; right && i < 4; ++i) {
            const double step = kGridStep[i];
            const double lastX = (640 / kGridStep[i] - 1) * step;
            const double lastY = (384 / kGridStep[i] - 1) * step;
            right = near(found[i].w, step, 1e-3) && near(found[i].h, step, 1e-3) &&
                    near(found[i].cx, lastX - step / 2.0, 1e-3) && near(found[i].cy, lastY - step / 2.0, 1e-3);
        }
        check(right, "the last place of each of the four grids is read with that grid's step");
        check(found.size() == 4 && found[0].label == 0 && found[1].label == 2 && found[2].label == 5 && found[3].label == 7 &&
                  found[0].kind == Kind::Person && found[1].kind == Kind::Vehicle && found[3].kind == Kind::Vehicle,
              "each box keeps the network's own class: person, car, bus, truck");
    }
    {
        Answer answer(640, 384);
        answer.set(1, 0, 0, 0, 0.9f, 5, 5, 2, 2);
        answer.set(1, 39, 23, 0, 0.9f, 2, 2, 7, 7);
        const std::vector<Found> found = answer.decoded();
        check(found.size() == 2 && near(found[0].cx, 16.0, 1e-3) && near(found[0].w, 32.0, 1e-3) && near(found[0].cy, 16.0, 1e-3),
              "a box that reaches over the left top corner is cut at it");
        check(found.size() == 2 && near(found[1].cx + found[1].w / 2.0, 640.0, 1e-3) &&
                  near(found[1].cy + found[1].h / 2.0, 384.0, 1e-3),
              "and one over the right bottom corner at that");
    }
    {
        Answer answer(640, 384);
        answer.set(1, 5, 5, 0, 0.39f, 1, 1, 1, 1);
        answer.set(1, 9, 5, 0, 0.40f, 1, 1, 1, 1);
        check(answer.decoded(0.40).size() == 1, "an object less sure than asked for is left out, one just as sure is in");
        check(answer.decoded(0.30).size() == 2 && answer.decoded(0.95).empty(), "the bar can be set lower and higher");
    }
    {
        // 0.5 is the same number in the network's answer and in the bar (0.4 is not).
        Answer answer(640, 384);
        answer.set(1, 5, 5, 0, 0.5f, 1, 1, 1, 1);
        check(answer.decoded(0.5).size() == 1, "an object exactly as sure as asked for is in");
    }
    {
        Answer answer(640, 384);
        const int notMarked[] = {1, 4, 6, 8, 9, 16, 79};
        int x = 2;
        for (const int label : notMarked) {
            answer.set(1, x, 3, label, 0.9f, 1, 1, 1, 1);
            x += 4;
        }
        check(answer.decoded().empty(), "a bicycle, a plane, a train, a boat, a dog are not marked");
    }
    {
        Answer answer(640, 384);
        answer.set(1, 5, 5, 2, 0.6f, 1, 1, 1, 1);
        answer.row(1, 5, 5)[8] = 0.7f;  // a boat, surer than the car
        answer.set(1, 12, 5, 2, 0.6f, 1, 1, 1, 1);
        answer.row(1, 12, 5)[8] = 0.5f;  // a boat, less sure than the car
        const std::vector<Found> found = answer.decoded();
        check(found.size() == 1 && near(found[0].cx, 12 * 16.0, 1e-3), "a place where another class is surer is not marked as a vehicle");
    }
    {
        Answer answer(640, 384);
        answer.set(1, 5, 5, 0, 0.9f, 0, 0, 0, 0);
        check(answer.decoded().empty(), "a box with no size is nothing");
    }
    {
        Answer answer(640, 384);
        answer.set(1, 5, 5, 0, 0.9f, 1, 1, 1, 1);
        answer.row(1, 5, 5)[0] = std::numeric_limits<float>::quiet_NaN();
        answer.set(1, 9, 5, 2, 0.9f, 1, 1, 1, 1);
        answer.row(1, 9, 5)[kClasses + 3] = std::numeric_limits<float>::quiet_NaN();
        answer.set(1, 20, 5, 2, 0.9f, 1, 1, 1, 1);
        const std::vector<Found> found = answer.decoded();
        check(found.size() == 1 && near(found[0].cx, 320.0, 1e-3), "numbers that are none give no box, and do not spoil the others");
    }
    {
        Answer answer(640, 384);
        answer.set(1, 5, 5, 0, 5.0f, 1, 1, 1, 1);
        const std::vector<Found> found = answer.decoded();
        check(found.size() == 1 && near(found[0].score, 1.0), "a score is never more than 1");
    }
    {
        Answer answer(640, 384);
        std::vector<Found> out;
        check(!decode(answer.rows.data(), answer.count() - 1, kRowLength, 640, 384, 0.4, out),
              "an answer with a row too few is refused");
        check(!decode(answer.rows.data(), answer.count(), kRowLength - 1, 640, 384, 0.4, out),
              "an answer with shorter rows is refused");
        check(!decode(answer.rows.data(), answer.count(), kRowLength, 640, 416, 0.4, out),
              "an answer for another input size is refused");
        check(!decode(nullptr, answer.count(), kRowLength, 640, 384, 0.4, out), "no answer is refused");
        check(!decode(answer.rows.data(), 0, kRowLength, 0, 0, 0.4, out), "an input of no size is refused");
        check(out.empty(), "and nothing comes out of a refused answer");
    }
    {
        // 416 is not a multiple of 64: the coarsest grid has 7 places a side, the last one over the edge.
        Answer answer(416, 416);
        answer.set(3, 6, 6, 0, 0.9f, 1, 1, 1, 1);
        const std::vector<Found> found = answer.decoded();
        check(found.size() == 1 && near(found[0].cx + found[0].w / 2.0, 416.0, 1e-3) && near(found[0].w, 96.0, 1e-3),
              "the network's own size of 416 is read too, with its last place over the edge");
    }
}

void backToThePicture()
{
    {
        // The whole 1280 x 720 picture in a 640 x 384 input.
        const Fit fit = fitArea(1280, 720, 640, 384, true);
        std::vector<Found> found = {box(Kind::Person, 0.9, 320.0, 192.0, 20.0, 40.0)};
        toPicture(found, fit, Area{0, 0, 1280, 720});
        check(found.size() == 1 && near(found[0].cx, 640.0) && near(found[0].cy, 360.0) && near(found[0].w, 40.0) &&
                  near(found[0].h, 80.0),
              "a box in the middle of the input is in the middle of the picture, twice the size");
    }
    {
        // A part of the picture, point for point.
        const Fit fit = fitArea(640, 384, 640, 384, true);
        std::vector<Found> found = {box(Kind::Vehicle, 0.9, 100.0, 50.0, 30.0, 20.0)};
        toPicture(found, fit, Area{320, 168, 640, 384});
        check(found.size() == 1 && near(found[0].cx, 420.0) && near(found[0].cy, 218.0) && near(found[0].w, 30.0) &&
                  near(found[0].h, 20.0),
              "a box in a part of the picture is moved by where the part starts");
    }
    {
        // A thermal picture: bars of 80 points at the sides.
        const Fit fit = fitArea(640, 512, 640, 384, true);
        std::vector<Found> found = {box(Kind::Person, 0.9, 80.0 + 240.0, 192.0, 30.0, 60.0),
                                    box(Kind::Person, 0.8, 40.0, 192.0, 30.0, 60.0),
                                    box(Kind::Person, 0.7, 90.0, 192.0, 60.0, 60.0)};
        toPicture(found, fit, Area{0, 0, 640, 512});
        check(found.size() == 2 && near(found[0].cx, 320.0) && near(found[0].cy, 256.0) && near(found[0].w, 40.0) &&
                  near(found[0].h, 80.0),
              "bars at the sides are taken off, and a box that lies in a bar is dropped");
        check(found.size() == 2 && near(found[1].cx - found[1].w / 2.0, 0.0) && near(found[1].w, 40.0 / 0.75),
              "a box half in a bar is cut at the picture's side");
    }
    {
        const Fit fit = fitArea(1280, 720, 512, 384, false);
        std::vector<Found> found = {box(Kind::Vehicle, 0.9, 256.0, 192.0, 40.0, 32.0)};
        toPicture(found, fit, Area{0, 0, 1280, 720});
        check(found.size() == 1 && near(found[0].cx, 640.0) && near(found[0].cy, 360.0) && near(found[0].w, 100.0) &&
                  near(found[0].h, 60.0),
              "from a stretched input each way goes back by its own factor");
    }
    {
        const Fit fit = fitArea(1280, 720, 640, 384, true);
        std::vector<Found> found = {box(Kind::Person, 0.9, 320.0, 192.0, 1.5, 30.0),
                                    box(Kind::Person, 0.9, 400.0, 192.0, 30.0, 1.5),
                                    box(Kind::Person, 0.9, 500.0, 192.0, 2.0, 2.0)};
        toPicture(found, fit, Area{0, 0, 1280, 720});
        check(found.size() == 1 && near(found[0].w, 4.0), "a box narrower or lower than 4 picture points is dropped");
    }
    {
        // The camera's picture has a bar of 12 input points below it: a box that reaches into that bar.
        const Fit fit = fitArea(1280, 720, 640, 384, true);
        std::vector<Found> found = {box(Kind::Vehicle, 0.9, 320.0, 370.0, 40.0, 28.0)};
        toPicture(found, fit, Area{0, 0, 1280, 720});
        check(found.size() == 1 && near(found[0].cy + found[0].h / 2.0, 720.0) && near(found[0].h, 32.0),
              "a box that reaches into the bar below the picture is cut at the picture's bottom");
        // The thermal picture has a bar of 80 input points at its right: a box that reaches into it.
        const Fit thermal = fitArea(640, 512, 640, 384, true);
        found = {box(Kind::Vehicle, 0.9, 570.0, 192.0, 60.0, 30.0)};
        toPicture(found, thermal, Area{0, 0, 640, 512});
        check(found.size() == 1 && near(found[0].cx + found[0].w / 2.0, 640.0) && near(found[0].w, 640.0 - 460.0 / 0.75, 1e-6),
              "and one into the bar at the right at the picture's right side");
    }
    {
        // A fit with no scale: a box across where the picture would start must not come back as wide as the picture.
        Fit none;
        none.scaleX = 0.0;
        none.padLeft = 320;
        std::vector<Found> found = {box(Kind::Person, 0.9, 320.0, 192.0, 20.0, 40.0),
                                    box(Kind::Person, 0.9, 100.0, 192.0, 20.0, 40.0)};
        toPicture(found, none, Area{0, 0, 1280, 720});
        check(found.empty(), "nothing comes back through a fit that is none");
        Fit flat;
        flat.scaleY = 0.0;
        flat.padTop = 192;
        found = {box(Kind::Person, 0.9, 320.0, 192.0, 20.0, 40.0)};
        toPicture(found, flat, Area{0, 0, 1280, 720});
        check(found.empty(), "nor through one that is none downwards");
    }
}

void sharing()
{
    const Found a = box(Kind::Person, 0.9, 50.0, 50.0, 20.0, 20.0);
    check(near(shared(a, a), 1.0), "a box shares all with itself");
    check(near(shared(a, box(Kind::Person, 0.9, 80.0, 50.0, 20.0, 20.0)), 0.0), "boxes apart share nothing");
    check(near(shared(a, box(Kind::Person, 0.9, 70.0, 50.0, 20.0, 20.0)), 0.0), "boxes that only touch share nothing");
    check(near(shared(a, box(Kind::Person, 0.9, 60.0, 50.0, 20.0, 20.0)), 1.0 / 3.0), "boxes moved by half their width share a third");
    check(near(shared(a, box(Kind::Person, 0.9, 50.0, 50.0, 10.0, 10.0)), 0.25), "a box of half the size inside shares a quarter");
    check(near(shared(a, box(Kind::Person, 0.9, 50.0, 50.0, 0.0, 0.0)), 0.0), "a box of no size shares nothing");
}

void pickingOneBoxForAnObject()
{
    {
        const std::vector<Found> picked = pick({box(Kind::Person, 0.5, 100.0, 100.0, 20.0, 40.0),
                                                box(Kind::Person, 0.9, 102.0, 101.0, 20.0, 40.0),
                                                box(Kind::Person, 0.7, 99.0, 100.0, 22.0, 42.0)});
        check(picked.size() == 1 && near(picked[0].score, 0.9) && near(picked[0].cx, 102.0),
              "of three boxes on one object the surest is kept");
    }
    {
        const std::vector<Found> picked = pick({box(Kind::Person, 0.5, 100.0, 100.0, 20.0, 40.0),
                                                box(Kind::Vehicle, 0.9, 100.0, 100.0, 20.0, 40.0)});
        check(picked.size() == 2 && picked[0].kind == Kind::Vehicle, "a person and a vehicle in the same place are both kept");
    }
    {
        // Moved by half its width: a third shared. Moved by a third: a half shared, which is not more than a half.
        const std::vector<Found> apart = pick({box(Kind::Vehicle, 0.9, 100.0, 100.0, 30.0, 20.0),
                                               box(Kind::Vehicle, 0.8, 115.0, 100.0, 30.0, 20.0)});
        const std::vector<Found> onTheLine = pick({box(Kind::Vehicle, 0.9, 100.0, 100.0, 30.0, 20.0),
                                                   box(Kind::Vehicle, 0.8, 110.0, 100.0, 30.0, 20.0)});
        const std::vector<Found> same = pick({box(Kind::Vehicle, 0.9, 100.0, 100.0, 30.0, 20.0),
                                              box(Kind::Vehicle, 0.8, 109.0, 100.0, 30.0, 20.0)});
        check(apart.size() == 2 && onTheLine.size() == 2 && same.size() == 1,
              "two vehicles side by side stay two: only boxes that share more than half are one object");
    }
    {
        const std::vector<Found> picked = pick({box(Kind::Person, 0.3, 10.0, 10.0, 8.0, 8.0),
                                                box(Kind::Person, 0.8, 50.0, 10.0, 8.0, 8.0),
                                                box(Kind::Vehicle, 0.6, 90.0, 10.0, 8.0, 8.0)});
        check(picked.size() == 3 && near(picked[0].score, 0.8) && near(picked[1].score, 0.6) && near(picked[2].score, 0.3),
              "the surest come first");
    }
    {
        std::vector<Found> many;
        for (int i = 0; i < 30; ++i) {
            many.push_back(box(Kind::Person, 0.5 + 0.01 * i, 20.0 * i, 10.0, 8.0, 8.0));
        }
        const std::vector<Found> picked = pick(many, 10);
        check(picked.size() == 10 && near(picked[0].score, 0.79) && near(picked[9].score, 0.70), "no more than asked for, the surest of them");
        check(pick({}).empty() && pick(many, 0).empty(), "nothing in, or none asked for, gives none");
    }
}

void joiningTheWholeAndThePart()
{
    const Area picture{0, 0, 1280, 720};
    const Area part{320, 168, 640, 384};  // the middle of the picture
    {
        // The same person in both: the part saw them larger.
        const std::vector<Found> joined = merge({box(Kind::Person, 0.9, 640.0, 360.0, 22.0, 50.0)},
                                                {box(Kind::Person, 0.6, 641.0, 361.0, 20.0, 46.0)}, part, picture);
        check(joined.size() == 1 && near(joined[0].cx, 641.0) && near(joined[0].score, 0.6),
              "an object found in the whole picture and in the part: the part's box is kept, whichever is surer");
    }
    {
        const std::vector<Found> joined = merge({box(Kind::Vehicle, 0.7, 100.0, 600.0, 80.0, 40.0)},
                                                {box(Kind::Person, 0.6, 640.0, 360.0, 20.0, 46.0)}, part, picture);
        check(joined.size() == 2 && joined[0].kind == Kind::Vehicle && joined[1].kind == Kind::Person,
              "what only one of them found is kept, the surest first");
    }
    {
        // A car across the part's left border: the part sees its right half only.
        const Found wholeCar = box(Kind::Vehicle, 0.8, 320.0, 360.0, 120.0, 60.0);
        const Found halfCar = box(Kind::Vehicle, 0.7, 350.0, 360.0, 60.0, 60.0);
        const std::vector<Found> joined = merge({wholeCar}, {halfCar}, part, picture);
        check(joined.size() == 1 && near(joined[0].w, 120.0), "a box cut by the part's border is dropped when the whole picture has the object");
        const std::vector<Found> alone = merge({}, {halfCar}, part, picture);
        check(alone.size() == 1 && near(alone[0].w, 60.0), "and kept when it is all there is of the object");
    }
    {
        // Each border of the part cuts.
        const Found whole = box(Kind::Vehicle, 0.8, 640.0, 360.0, 700.0, 440.0);
        int dropped = 0;
        const Found cuts[4] = {box(Kind::Vehicle, 0.7, 350.0, 360.0, 60.0, 60.0),   // left
                               box(Kind::Vehicle, 0.7, 930.0, 360.0, 60.0, 60.0),   // right
                               box(Kind::Vehicle, 0.7, 640.0, 198.0, 60.0, 60.0),   // top
                               box(Kind::Vehicle, 0.7, 640.0, 522.0, 60.0, 60.0)};  // bottom
        for (const Found &c : cuts) {
            dropped += merge({whole}, {c}, part, picture).size() == 1 ? 1 : 0;
        }
        const Found inside = box(Kind::Vehicle, 0.7, 640.0, 360.0, 60.0, 60.0);
        check(dropped == 4 && merge({whole}, {inside}, part, picture).size() == 2,
              "all four borders of the part cut; a box clear of them is whole (and may lie inside a larger one)");
    }
    {
        // A part that starts at the picture's own left top corner: those two borders cut nothing.
        const Area corner{0, 0, 640, 384};
        const Found atCorner = box(Kind::Person, 0.6, 10.0, 20.0, 20.0, 40.0);
        const Found wholeOne = box(Kind::Person, 0.9, 11.0, 21.0, 22.0, 42.0);
        const std::vector<Found> joined = merge({wholeOne}, {atCorner}, corner, picture);
        check(joined.size() == 1 && near(joined[0].score, 0.6), "the picture's own border does not cut: the part's box is kept there");
    }
    {
        // A thermal picture starts 190 points into its image: a part at its left side is not cut there.
        const Area thermal{190, 0, 900, 720};
        const Area atItsSide{190, 168, 640, 384};
        const Found atSide = box(Kind::Person, 0.6, 200.0, 360.0, 20.0, 40.0);
        const Found wholeOne = box(Kind::Person, 0.9, 201.0, 361.0, 22.0, 42.0);
        check(merge({wholeOne}, {atSide}, atItsSide, thermal).size() == 1 &&
                  near(merge({wholeOne}, {atSide}, atItsSide, thermal)[0].score, 0.6),
              "a picture that starts inside its image has its own borders, which do not cut either");
        const Found halfThere = box(Kind::Person, 0.6, 201.0, 361.0, 22.0, 20.0);
        check(merge({wholeOne}, {halfThere}, atItsSide, picture).size() == 1 &&
                  near(merge({wholeOne}, {halfThere}, atItsSide, picture)[0].score, 0.9),
              "the same part in a picture that goes on to its left is cut there");
    }
    {
        const std::vector<Found> joined = merge({box(Kind::Vehicle, 0.9, 640.0, 360.0, 22.0, 50.0)},
                                                {box(Kind::Person, 0.6, 641.0, 361.0, 20.0, 46.0)}, part, picture);
        check(joined.size() == 2, "a person in the part does not replace a vehicle in the whole picture");
    }
    {
        // A person cut by the part's border, in front of a car that the whole picture has: the
        // car is not the person, so the person's box stays.
        const Found car = box(Kind::Vehicle, 0.8, 320.0, 360.0, 160.0, 90.0);
        const Found cutPerson = box(Kind::Person, 0.7, 330.0, 360.0, 20.0, 60.0);
        const std::vector<Found> joined = merge({car}, {cutPerson}, part, picture);
        check(joined.size() == 2 && joined[0].kind == Kind::Vehicle && joined[1].kind == Kind::Person,
              "a cut box is dropped only for a box of its own kind");
    }
    {
        // 20 points clear of the part's border is not cut: the part's box replaces the whole picture's.
        const Found inWhole = box(Kind::Person, 0.9, 350.0, 360.0, 22.0, 50.0);
        const Found clear = box(Kind::Person, 0.6, 350.0, 360.0, 20.0, 46.0);  // its left side at 340, the border at 320
        const std::vector<Found> joined = merge({inWhole}, {clear}, part, picture);
        check(joined.size() == 1 && near(joined[0].score, 0.6), "a box 20 points from the part's border is whole");
        // Two points from it, it may be cut.
        const Found atBorder = box(Kind::Person, 0.6, 332.0, 360.0, 20.0, 46.0);  // its left side at 322
        const Found there = box(Kind::Person, 0.9, 328.0, 360.0, 30.0, 50.0);
        const std::vector<Found> cutOne = merge({there}, {atBorder}, part, picture);
        check(cutOne.size() == 1 && near(cutOne[0].score, 0.9), "a box 2 points from it counts as cut");
        const Found threePoints = box(Kind::Person, 0.6, 333.0, 360.0, 20.0, 46.0);  // its left side at 323
        const std::vector<Found> whole = merge({there}, {threePoints}, part, picture);
        check(whole.size() == 1 && near(whole[0].score, 0.6), "and one 3 points from it does not");
    }
    {
        check(merge({}, {}, part, picture).empty(), "nothing and nothing joins to nothing");
    }
}

bool sameArea(const Area &a, int left, int top, int width, int height)
{
    return a.left == left && a.top == top && a.width == width && a.height == height;
}

void planningTheLooks()
{
    const Area camera{0, 0, 1280, 720};
    {
        const Plan plan = planLooks(camera, 0.0);
        check(sameArea(plan.whole.area, 0, 0, 1280, 720) && plan.whole.inputW == 640 && plan.whole.inputH == 384 &&
                  plan.whole.keepShape,
              "the camera's whole picture is looked at in 640 x 384 points, in its own shape");
        check(plan.closer && sameArea(plan.middle.area, 320, 168, 640, 384) && plan.middle.inputW == 640 &&
                  plan.middle.inputH == 384,
              "and, on a device of unknown speed, its middle 640 x 384 points at their own size");
    }
    {
        const Plan slow = planLooks(camera, 900.0);
        check(slow.closer && sameArea(slow.middle.area, 320, 168, 640, 384),
              "a slow device still looks at the smallest middle");
        const Plan a = planLooks(camera, 250.0);
        const Plan b = planLooks(camera, 240.0);
        check(sameArea(a.middle.area, 320, 168, 640, 384) && sameArea(b.middle.area, 224, 120, 832, 480) &&
                  b.middle.inputW == 832 && b.middle.inputH == 480,
              "the middle grows to 832 x 480 when that takes no more than 0.4 s");
        const Plan c = planLooks(camera, 190.0);
        const Plan c2 = planLooks(camera, 185.0);
        check(sameArea(c.middle.area, 224, 120, 832, 480) && sameArea(c2.middle.area, 160, 88, 960, 544) &&
                  c2.middle.inputW == 960 && c2.middle.inputH == 544,
              "and to 960 x 544 on a faster device");
        const Plan d0 = planLooks(camera, 105.0);
        const Plan d = planLooks(camera, 100.0);
        check(sameArea(d0.middle.area, 160, 88, 960, 544) && d.closer && sameArea(d.middle.area, 0, 0, 1280, 720) &&
                  d.middle.inputW == 1280 && d.middle.inputH == 736,
              "a fast device looks at all of the picture at its own size");
        check(sameArea(d.whole.area, 0, 0, 1280, 720) && d.whole.inputW == 640, "and still at the whole of it made smaller, for what is large");
    }
    {
        // The 5:4 thermal picture inside a 16:9 image.
        const Plan plan = planLooks(Area{190, 0, 900, 720}, 0.0);
        check(sameArea(plan.whole.area, 190, 0, 900, 720) && plan.whole.inputW == 480 && plan.whole.inputH == 384,
              "a thermal picture is looked at in 480 x 384 points, with no bars beside it");
        check(plan.closer && sameArea(plan.middle.area, 320, 168, 640, 384), "its middle is the middle of the picture, not of the image");
    }
    {
        const Plan small = planLooks(Area{0, 0, 640, 360}, 0.0);
        check(!small.closer && small.whole.inputW == 640 && small.whole.inputH == 384,
              "a picture of 640 x 360 points is looked at once: it is at its own size already");
        const Plan nearly = planLooks(Area{0, 0, 700, 394}, 0.0);
        const Plan larger = planLooks(Area{0, 0, 760, 428}, 0.0);
        check(!nearly.closer && larger.closer, "one look is enough when the picture is made no more than a tenth smaller for it");
    }
    {
        const Plan tiny = planLooks(Area{0, 0, 320, 180}, 0.0);
        check(!tiny.closer && tiny.whole.inputW == 320 && tiny.whole.inputH == 192,
              "a picture smaller than the look is not made larger for it");
        const Plan tall = planLooks(Area{40, 0, 500, 900}, 0.0);
        check(tall.closer && sameArea(tall.middle.area, 40, 258, 500, 384) && tall.middle.inputW == 512 && tall.middle.inputH == 384,
              "the middle of a narrow picture is as wide as the picture, no wider");
    }
    {
        const Plan flat = planLooks(Area{10, 20, 1000, 300}, 0.0);
        check(flat.closer && sameArea(flat.middle.area, 190, 20, 640, 300) && flat.middle.inputW == 640 &&
                  flat.middle.inputH == 320,
              "the middle is never larger than the picture");
        const Plan large = planLooks(Area{0, 0, 1920, 1080}, 45.0);
        check(sameArea(large.middle.area, 320, 172, 1280, 736) && large.whole.inputW == 640 && large.whole.inputH == 384,
              "of a larger picture the middle is at most 1280 x 736 points");
    }
    {
        const Plan none = planLooks(Area{0, 0, 0, 720}, 0.0);
        check(!none.closer && none.whole.area.width == 0, "no picture, no second look");
    }
}

} // namespace

int main(int argc, char **argv)
{
    g_verbose = argc > 1 && argv[1][0] == '-' && argv[1][1] == 'v';
    planningTheLooks();
    theAnswersSize();
    kinds();
    layingAPictureIn();
    readingTheAnswer();
    backToThePicture();
    sharing();
    pickingOneBoxForAnObject();
    joiningTheWholeAndThePart();
    std::printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
