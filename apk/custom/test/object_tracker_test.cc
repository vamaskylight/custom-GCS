// Host unit test for ObjectTracker: made-up pictures, so every case knows where
// the object really is. A scene larger than the picture is the world, the
// picture is a window on it (moving the window is the camera turning), and the
// object is a small patch with its own pattern drawn over the scene.
//
// These are grey pictures and stiff objects. A walking, turning person in colour
// and in thermal is in object_tracker_walk_test.cc.

#include "ObjectTracker.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

using namespace skydroid::track;

namespace {

int g_checks = 0;
int g_failures = 0;
bool g_verbose = false;  // "-v": print every case with its numbers

constexpr int kFrameW = 384;
constexpr int kFrameH = 216;
constexpr int kSceneW = 1400;
constexpr int kSceneH = 700;

struct Random
{
    std::uint32_t state;
    explicit Random(std::uint32_t seed) : state(seed) {}
    std::uint32_t next()
    {
        state = state * 1664525u + 1013904223u;
        return state >> 8;
    }
    int below(int n) { return static_cast<int>(next() % static_cast<std::uint32_t>(n)); }
};

/// The world: soft light and dark areas, some buildings, and fine grain.
std::vector<std::uint8_t> makeScene(std::uint32_t seed)
{
    Random rnd(seed);
    const int cell = 24;
    const int gw = kSceneW / cell + 2;
    const int gh = kSceneH / cell + 2;
    std::vector<float> grid(gw * gh);
    for (float &v : grid) {
        v = 60.0f + rnd.below(110);
    }
    std::vector<std::uint8_t> scene(kSceneW * kSceneH);
    for (int y = 0; y < kSceneH; ++y) {
        for (int x = 0; x < kSceneW; ++x) {
            const float fx = static_cast<float>(x) / cell;
            const float fy = static_cast<float>(y) / cell;
            const int x0 = static_cast<int>(fx);
            const int y0 = static_cast<int>(fy);
            const float ax = fx - x0;
            const float ay = fy - y0;
            const float top = grid[y0 * gw + x0] * (1 - ax) + grid[y0 * gw + x0 + 1] * ax;
            const float bottom = grid[(y0 + 1) * gw + x0] * (1 - ax) + grid[(y0 + 1) * gw + x0 + 1] * ax;
            scene[y * kSceneW + x] = static_cast<std::uint8_t>(top * (1 - ay) + bottom * ay);
        }
    }
    for (int i = 0; i < 90; ++i) {  // buildings
        const int w = 20 + rnd.below(80);
        const int h = 15 + rnd.below(60);
        const int x0 = rnd.below(kSceneW - w);
        const int y0 = rnd.below(kSceneH - h);
        const int grey = 30 + rnd.below(200);
        for (int y = y0; y < y0 + h; ++y) {
            for (int x = x0; x < x0 + w; ++x) {
                scene[y * kSceneW + x] = static_cast<std::uint8_t>(grey);
            }
        }
    }
    for (std::uint8_t &v : scene) {  // grain
        v = static_cast<std::uint8_t>(std::clamp(static_cast<int>(v) + rnd.below(13) - 6, 0, 255));
    }
    return scene;
}

/// The object: its own pattern of light and dark blocks.
std::vector<std::uint8_t> makeObject(int w, int h, std::uint32_t seed, int block)
{
    Random rnd(seed);
    std::vector<std::uint8_t> object(w * h);
    std::vector<int> blocks((w / block + 1) * (h / block + 1));
    for (int &b : blocks) {
        b = rnd.below(2) ? 225 - rnd.below(40) : 25 + rnd.below(40);
    }
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            object[y * w + x] = static_cast<std::uint8_t>(blocks[(y / block) * (w / block + 1) + x / block]);
        }
    }
    return object;
}

struct World
{
    std::vector<std::uint8_t> scene = makeScene(1234);
    std::vector<std::uint8_t> object;
    int objectW = 0;
    int objectH = 0;
    std::vector<std::uint8_t> frame = std::vector<std::uint8_t>(kFrameW * kFrameH);
    Random grain{99};
    // A second look of the object, and how much of it shows (0: none, 1: only it).
    std::vector<std::uint8_t> otherLook;
    double otherShare = 0.0;
    // A shadow: everything left of this column of the picture is this much darker (1: no shadow).
    int shadowEdge = 0;
    double shadow = 1.0;

    /// "block": the size of its light and dark parts.
    void setObject(int w, int h, std::uint32_t seed = 7, int block = 4)
    {
        objectW = w;
        objectH = h;
        object = makeObject(w, h, seed, block);
    }

    /// The picture for a camera looking at (camX, camY) of the scene, with the
    /// object's centre at (objX, objY) of the PICTURE.
    void render(double camX, double camY, double objX, double objY, bool objectThere = true, double gain = 1.0,
                double lift = 0.0, int noise = 0)
    {
        const int cx = static_cast<int>(std::lround(camX));
        const int cy = static_cast<int>(std::lround(camY));
        for (int y = 0; y < kFrameH; ++y) {
            const int sy = std::clamp(y + cy, 0, kSceneH - 1);
            for (int x = 0; x < kFrameW; ++x) {
                const int sx = std::clamp(x + cx, 0, kSceneW - 1);
                frame[y * kFrameW + x] = scene[sy * kSceneW + sx];
            }
        }
        if (objectThere) {
            const int left = static_cast<int>(std::lround(objX - objectW / 2.0));
            const int top = static_cast<int>(std::lround(objY - objectH / 2.0));
            for (int y = 0; y < objectH; ++y) {
                for (int x = 0; x < objectW; ++x) {
                    const int px = left + x;
                    const int py = top + y;
                    if (px >= 0 && px < kFrameW && py >= 0 && py < kFrameH) {
                        double value = object[y * objectW + x];
                        if (otherShare > 0.0 && !otherLook.empty()) {
                            value = value * (1.0 - otherShare) + otherLook[y * objectW + x] * otherShare;
                        }
                        frame[py * kFrameW + px] = static_cast<std::uint8_t>(value);
                    }
                }
            }
        }
        if (shadow != 1.0) {
            for (int y = 0; y < kFrameH; ++y) {
                for (int x = 0; x < std::min(shadowEdge, kFrameW); ++x) {
                    frame[y * kFrameW + x] = static_cast<std::uint8_t>(frame[y * kFrameW + x] * shadow);
                }
            }
        }
        if (gain != 1.0 || lift != 0.0 || noise > 0) {
            for (std::uint8_t &v : frame) {
                double value = v * gain + lift;
                if (noise > 0) {
                    value += grain.below(2 * noise + 1) - noise;
                }
                v = static_cast<std::uint8_t>(std::clamp(value, 0.0, 255.0));
            }
        }
    }
};

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

struct Run
{
    double worst = 0.0;  // largest distance of the box from the object, in points
    int lost = 0;        // pictures in which it was not found
    double lowestPsr = 1e9;
};

/// Starts on the object and follows it through "frames" pictures.
/// camera(i) and place(i) give the camera's look and the object's place in the picture for picture i.
template <typename Camera, typename Place>
Run follow(World &world, int boxW, int boxH, int frames, Camera camera, Place place, double gain = 1.0,
           double lift = 0.0, int noise = 0, int changeAt = 0)
{
    ObjectTracker tracker;
    Run run;
    const auto [cam0x, cam0y] = camera(0);
    const auto [obj0x, obj0y] = place(0);
    world.render(cam0x, cam0y, obj0x, obj0y, true, 1.0, 0.0, noise);
    if (!tracker.start(world.frame.data(), kFrameW, kFrameH, kFrameW, Box{obj0x, obj0y, double(boxW), double(boxH)})) {
        run.lost = frames;
        run.worst = 1e9;
        return run;
    }
    for (int i = 1; i < frames; ++i) {
        const auto [camX, camY] = camera(i);
        const auto [objX, objY] = place(i);
        const bool changed = changeAt > 0 && i >= changeAt;
        world.render(camX, camY, objX, objY, true, changed ? gain : 1.0, changed ? lift : 0.0, noise);
        const Result r = tracker.update(world.frame.data(), kFrameW, kFrameH, kFrameW);
        if (!r.found) {
            ++run.lost;
        }
        run.lowestPsr = std::min(run.lowestPsr, r.psr);
        run.worst = std::max(run.worst, std::hypot(r.box.cx - objX, r.box.cy - objY));
    }
    return run;
}

struct Point
{
    double x;
    double y;
};

void report(const char *name, const Run &run, double allowed, int allowedLost = 0)
{
    char detail[160];
    std::snprintf(detail, sizeof detail, "(worst %.1f points, allowed %.1f; lost %d of the pictures; lowest PSR %.1f)",
                  run.worst, allowed, run.lost, run.lowestPsr);
    check(run.worst <= allowed && run.lost <= allowedLost, name, detail);
}

} // namespace

int main(int argc, char **argv)
{
    g_verbose = argc > 1 && argv[1][0] == '-' && argv[1][1] == 'v';
    World world;

    // --- It follows --------------------------------------------------------------
    world.setObject(20, 40);
    report("an object that walks across a still picture",
           follow(world, 20, 40, 90,
                  [](int) { return Point{400, 200}; },
                  [](int i) { return Point{80.0 + 2.0 * i, 110.0}; }),
           3.0);
    report("a still object while the camera pans",
           follow(world, 20, 40, 60,
                  [](int i) { return Point{300.0 + 5.0 * i, 200.0}; },
                  [](int i) { return Point{340.0 - 5.0 * i, 100.0}; }),
           3.0);
    report("the object held in the middle while the ground moves behind it",
           follow(world, 20, 40, 90,
                  [](int i) { return Point{200.0 + 4.0 * i, 180.0 + 1.0 * i}; },
                  [](int) { return Point{192.0, 108.0}; }),
           3.0);
    report("up and down as well",
           follow(world, 20, 40, 60,
                  [](int) { return Point{500, 250}; },
                  [](int i) { return Point{190.0 + 20.0 * std::sin(i * 0.2), 40.0 + 2.2 * i}; }),
           3.5);
    report("a fast object: half its width in every picture",
           follow(world, 20, 40, 30,
                  [](int) { return Point{600, 300}; },
                  [](int i) { return Point{40.0 + 10.0 * i, 108.0}; }),
           5.0);
    report("a sudden start and a sudden stop",
           follow(world, 20, 40, 60,
                  [](int) { return Point{250, 150}; },
                  [](int i) { return Point{100.0 + 8.0 * std::clamp(i - 15, 0, 20), 100.0}; }),
           5.0);
    report("the camera swings fast to bring the object to the middle",
           follow(world, 20, 40, 40,
                  [](int i) { return Point{300.0 + 14.0 * std::min(i, 12), 200.0}; },
                  [](int i) { return Point{330.0 - 11.0 * std::min(i, 12), 100.0}; }),
           6.0);

    {
        // Faster than its own width in every picture, after speeding up over four pictures:
        // it is found because the tracker looks first where the object should be by now.
        ObjectTracker tracker;
        world.setObject(20, 40);
        double x = 40.0;
        double speed = 0.0;
        world.render(400, 200, x, 110);
        tracker.start(world.frame.data(), kFrameW, kFrameH, kFrameW, Box{x, 110, 20, 40});
        double worst = 0.0;
        int lost = 0;
        for (int i = 1; i <= 13; ++i) {
            speed = std::min(26.0, speed + 6.5);
            x += speed;
            world.render(400, 200, x, 110);
            const Result r = tracker.update(world.frame.data(), kFrameW, kFrameH, kFrameW);
            lost += r.found ? 0 : 1;
            worst = std::max(worst, std::abs(r.box.cx - x));
        }
        char detail[100];
        std::snprintf(detail, sizeof detail, "(worst %.1f points, lost %d of 13)", worst, lost);
        check(worst <= 3.0 && lost == 0, "an object that moves 26 points a picture, more than its width of 20", detail);
    }
    {
        // A step of 16 points across and 14 up between two pictures, from standing still:
        // past the reach of the patch in the middle, so it is found by looking around.
        ObjectTracker tracker;
        world.setObject(20, 40);
        world.render(400, 200, 150, 110);
        tracker.start(world.frame.data(), kFrameW, kFrameH, kFrameW, Box{150, 110, 20, 40});
        for (int i = 0; i < 7; ++i) {
            world.render(400, 200, 150, 110);
            tracker.update(world.frame.data(), kFrameW, kFrameH, kFrameW);
        }
        world.render(400, 200, 166, 96);
        const Result r = tracker.update(world.frame.data(), kFrameW, kFrameH, kFrameW);
        char detail[100];
        std::snprintf(detail, sizeof detail, "(found %d, box at %.1f, %.1f, wanted 166, 96)", r.found, r.box.cx, r.box.cy);
        check(r.found && std::abs(r.box.cx - 166) <= 2.0 && std::abs(r.box.cy - 96) <= 2.0,
              "a sudden step of most of its width is found in the very next picture", detail);
    }

    // --- Boxes of other shapes ---------------------------------------------------
    world.setObject(14, 48, 21);
    report("a tall narrow box (a person)",
           follow(world, 14, 48, 80,
                  [](int) { return Point{700, 320}; },
                  [](int i) { return Point{60.0 + 2.5 * i, 110.0}; }),
           3.0);
    world.setObject(60, 24, 22);
    report("a wide box (a car)",
           follow(world, 60, 24, 80,
                  [](int) { return Point{900, 100}; },
                  [](int i) { return Point{70.0 + 3.0 * i, 120.0 - 0.5 * i}; }),
           4.0);
    world.setObject(8, 8, 23);
    report("a box smaller than the smallest the tracker takes",
           follow(world, 8, 8, 60,
                  [](int) { return Point{820, 420}; },
                  [](int i) { return Point{100.0 + 1.5 * i, 100.0}; }),
           3.0);
    {
        // A very small object that moves more than its own size in every picture, at six
        // places. The tracker widens such a box to 12 points, and so it has the reach:
        // with the box as drawn (4 points) it kept the object at one place of the six.
        int keptPlaces = 0;
        int lost = 0;
        for (int place = 0; place < 6; ++place) {
            world.setObject(4, 4, 23 + place, 2);
            const double camX = 300 + 90.0 * place;
            const double camY = 100 + 40.0 * place;
            const Run run = follow(world, 4, 4, 40,
                                   [&](int) { return Point{camX, camY}; },
                                   [](int i) { return Point{60.0 + 6.0 * i, 100.0}; });
            keptPlaces += (run.lost <= 2 && run.worst <= 8.0) ? 1 : 0;
            lost += run.lost;
        }
        char detail[100];
        std::snprintf(detail, sizeof detail, "(kept at %d of 6 places, not found in %d of 234 pictures)", keptPlaces, lost);
        check(keptPlaces == 6, "a 4 point object that moves 6 points a picture", detail);
    }
    world.setObject(150, 120, 24, 20);  // large parts: fine ones vanish when it is shrunk
    report("a large box, shrunk into the patch",
           follow(world, 150, 120, 50,
                  [](int) { return Point{450, 380}; },
                  [](int i) { return Point{110.0 + 3.0 * i, 108.0}; }),
           6.0);

    {
        // A large box is shrunk five times into the patch. Averaged, grain of 90 grey levels
        // leaves the match as sharp as it was; picked point by point, it took 40 % off it.
        const Run run = follow(world, 150, 120, 50,
                               [](int) { return Point{450, 380}; },
                               [](int i) { return Point{110.0 + 3.0 * i, 108.0}; }, 1.0, 0.0, 90);
        char detail[100];
        std::snprintf(detail, sizeof detail, "(worst %.1f points, lowest PSR %.1f, lost %d)", run.worst, run.lowestPsr, run.lost);
        check(run.worst <= 1.5 && run.lowestPsr >= 42.0 && run.lost == 0, "a large box in a very noisy picture stays sharp", detail);
    }

    // --- The picture changes -----------------------------------------------------
    world.setObject(20, 40);
    report("the picture gets darker and flatter half way",
           follow(world, 20, 40, 70,
                  [](int) { return Point{400, 200}; },
                  [](int i) { return Point{90.0 + 2.0 * i, 110.0}; }, 0.6, 25.0, 0, 35),
           3.0);
    report("a noisy picture",
           follow(world, 20, 40, 70,
                  [](int) { return Point{400, 200}; },
                  [](int i) { return Point{90.0 + 2.0 * i, 110.0}; }, 1.0, 0.0, 10),
           3.5);

    {
        // The object turns: its look changes into another one over 80 pictures. The
        // tracker keeps it, because what it remembers of the object's look is kept up to date.
        ObjectTracker tracker;
        world.setObject(24, 44, 61);
        world.otherLook = makeObject(24, 44, 62, 4);
        world.otherShare = 0.0;
        world.render(400, 200, 120, 110);
        tracker.start(world.frame.data(), kFrameW, kFrameH, kFrameW, Box{120, 110, 24, 44});
        int lost = 0;
        double worst = 0.0;
        double lowestAlike = 1.0;
        for (int i = 1; i <= 100; ++i) {
            world.otherShare = std::min(1.0, i / 80.0);
            const double x = 120.0 + 1.5 * i;
            world.render(400, 200, x, 110);
            const Result r = tracker.update(world.frame.data(), kFrameW, kFrameH, kFrameW);
            lost += r.found ? 0 : 1;
            worst = std::max(worst, std::abs(r.box.cx - x));
            lowestAlike = std::min(lowestAlike, r.alike);
        }
        world.otherShare = 0.0;
        world.otherLook.clear();
        char detail[120];
        std::snprintf(detail, sizeof detail, "(lost %d of 100, worst %.1f points, lowest look-alike %.2f)", lost, worst, lowestAlike);
        check(lost == 0 && worst <= 3.0, "an object that slowly changes its look is kept", detail);
    }
    {
        // A deep shadow (three times darker) creeps over the object and the ground, 2 points
        // a picture. The object is kept, and the match stays sharp: the logarithm makes the
        // dark side count like the bright one (without it the sharpness fell to 17).
        // What this tracker does NOT take: such a shadow standing still across the middle
        // of the object. Then it reports the object as not seen.
        ObjectTracker tracker;
        world.setObject(24, 44, 63);
        world.render(400, 200, 190, 110);
        tracker.start(world.frame.data(), kFrameW, kFrameH, kFrameW, Box{190, 110, 24, 44});
        for (int i = 0; i < 5; ++i) {
            world.render(400, 200, 190, 110);
            tracker.update(world.frame.data(), kFrameW, kFrameH, kFrameW);
        }
        world.shadow = 0.33;
        int lost = 0;
        double worst = 0.0;
        double lowestPsr = 1e9;
        for (int i = 1; i <= 60; ++i) {
            world.shadowEdge = std::min(260, 150 + 2 * i);
            world.render(400, 200, 190, 110);
            const Result r = tracker.update(world.frame.data(), kFrameW, kFrameH, kFrameW);
            lost += r.found ? 0 : 1;
            worst = std::max(worst, std::hypot(r.box.cx - 190, r.box.cy - 110));
            lowestPsr = std::min(lowestPsr, r.psr);
        }
        world.shadow = 1.0;
        char detail[120];
        std::snprintf(detail, sizeof detail, "(lost %d of 60, worst %.1f points, lowest PSR %.1f)", lost, worst, lowestPsr);
        check(lost == 0 && worst <= 1.0 && lowestPsr >= 21.0, "a deep shadow creeping over the object does not shake it off", detail);
    }

    // --- It says when it does not see the object ---------------------------------
    {
        ObjectTracker tracker;
        world.setObject(20, 40);
        world.render(400, 200, 190, 110);
        check(tracker.start(world.frame.data(), kFrameW, kFrameH, kFrameW, Box{190, 110, 20, 40}), "starts on an object");
        for (int i = 0; i < 10; ++i) {
            world.render(400, 200, 190, 110);
            tracker.update(world.frame.data(), kFrameW, kFrameH, kFrameW);
        }
        // Hidden for 15 pictures: not found, and the box waits where it was.
        int notFound = 0;
        double moved = 0.0;
        for (int i = 0; i < 15; ++i) {
            world.render(400, 200, 190, 110, false);
            const Result r = tracker.update(world.frame.data(), kFrameW, kFrameH, kFrameW);
            notFound += r.found ? 0 : 1;
            moved = std::max(moved, std::hypot(r.box.cx - 190, r.box.cy - 110));
        }
        char detail[120];
        std::snprintf(detail, sizeof detail, "(not found in %d of 15, box moved %.1f, missed() %d)", notFound, moved,
                      tracker.missed());
        check(notFound >= 13 && moved <= 6.0, "a hidden object is reported as not found, and the box waits", detail);
        check(tracker.missed() >= 10, "missed() counts the pictures without the object", detail);
        // Back at the same place: found again at once.
        int foundAfter = -1;
        for (int i = 0; i < 5 && foundAfter < 0; ++i) {
            world.render(400, 200, 190, 110);
            if (tracker.update(world.frame.data(), kFrameW, kFrameH, kFrameW).found) {
                foundAfter = i;
            }
        }
        std::snprintf(detail, sizeof detail, "(found after %d pictures, missed() %d)", foundAfter, tracker.missed());
        check(foundAfter >= 0 && foundAfter <= 2 && tracker.missed() == 0, "it is found again when it comes back", detail);
    }
    {
        // The object walks out of the picture: not found, and the box stays inside the picture.
        ObjectTracker tracker;
        world.setObject(20, 40);
        world.render(400, 200, 330, 110);
        tracker.start(world.frame.data(), kFrameW, kFrameH, kFrameW, Box{330, 110, 20, 40});
        Result last;
        int notFoundAtEnd = 0;
        double furthest = 0.0;
        bool foundHalfOut = false;
        for (int i = 1; i < 40; ++i) {
            // Out slowly at first: at picture 27 its centre is on the picture's last column.
            const double x = i <= 27 ? 330 + 2.0 * i : 384 + 6.0 * (i - 27);
            world.render(400, 200, x, 110);
            last = tracker.update(world.frame.data(), kFrameW, kFrameH, kFrameW);
            furthest = std::max(furthest, last.box.cx);
            foundHalfOut = foundHalfOut || (i == 27 && last.found);
            if (i >= 30 && !last.found) {
                ++notFoundAtEnd;
            }
        }
        char detail[140];
        std::snprintf(detail, sizeof detail, "(not found in %d of the last 10, box at %.1f, %.1f, furthest right %.1f)",
                      notFoundAtEnd, last.box.cx, last.box.cy, furthest);
        check(foundHalfOut, "an object half out of the picture is still found", detail);
        check(notFoundAtEnd >= 9, "an object that left the picture is not found", detail);
        check(furthest <= kFrameW - 1 && last.box.cy >= 0 && last.box.cy <= kFrameH - 1,
              "the box's centre never leaves the picture", detail);
    }
    {
        // A second object that looks the same, a little to the side, does not take the box.
        ObjectTracker tracker;
        world.setObject(20, 40);
        auto withTwin = [&](double x) {
            world.render(400, 200, 280, 110);  // the twin, standing still
            std::vector<std::uint8_t> twin = world.frame;
            world.render(400, 200, x, 110);
            for (int y = 0; y < kFrameH; ++y) {
                for (int px = 255; px < 305; ++px) {
                    world.frame[y * kFrameW + px] = twin[y * kFrameW + px];
                }
            }
        };
        withTwin(120);
        tracker.start(world.frame.data(), kFrameW, kFrameH, kFrameW, Box{120, 110, 20, 40});
        double worst = 0.0;
        for (int i = 1; i < 50; ++i) {
            const double x = 120.0 + 1.5 * i;  // walks towards the twin and stops 60 points short
            withTwin(std::min(x, 195.0));
            const Result r = tracker.update(world.frame.data(), kFrameW, kFrameH, kFrameW);
            worst = std::max(worst, std::abs(r.box.cx - std::min(x, 195.0)));
        }
        char detail[80];
        std::snprintf(detail, sizeof detail, "(worst %.1f points)", worst);
        check(worst <= 4.0, "a look-alike beside the object does not take the box", detail);
    }

    {
        // Many places, shapes and speeds in three worlds: 24 pictures with the object, then 12
        // with the object taken away. While the object is whole in the picture it is found
        // every time. After it went, a patch of ground was taken for it in 64 of the 864
        // pictures: in 65 of the 72 places the tracker said "not seen" at once, in 5 it held on
        // to ground with the object's grey levels and pattern (a grey picture gives the colours
        // little to tell). The first tracker of the app said so in 24 pictures, but it lost a
        // walking, turning person in most walks; the rules that keep that person cost this.
        int missedWhileThere = 0;
        int foundWhenGone = 0;
        int gonePictures = 0;
        for (int scene = 0; scene < 3; ++scene) {
            World other;
            other.scene = makeScene(1234 + 77 * scene);
            for (int place = 0; place < 24; ++place) {
                const int shapes[4][2] = {{20, 40}, {14, 48}, {60, 24}, {30, 30}};
                const int *shape = shapes[place % 4];
                other.setObject(shape[0], shape[1], 5 + place, place % 3 == 0 ? 8 : 4);
                const double camX = 100 + 43.0 * place;
                const double camY = 60 + 17.0 * place;
                const double x0 = 80 + 9.0 * place;
                const double y0 = 60 + 4.0 * place;
                const double speed = (place % 5) * 1.5;
                const int noise = place % 2 ? 6 : 0;
                ObjectTracker tracker;
                other.render(camX, camY, x0, y0);
                if (!tracker.start(other.frame.data(), kFrameW, kFrameH, kFrameW, Box{x0, y0, double(shape[0]), double(shape[1])})) {
                    ++missedWhileThere;
                    continue;
                }
                double x = x0;
                for (int i = 1; i <= 25; ++i) {
                    x = x0 + speed * i;
                    other.render(camX, camY, x, y0, true, 1.0, 0.0, noise);
                    const Result r = tracker.update(other.frame.data(), kFrameW, kFrameH, kFrameW);
                    const bool whole = x + shape[0] / 2.0 < kFrameW;
                    missedWhileThere += (whole && !r.found) ? 1 : 0;
                }
                for (int i = 0; i < 12; ++i) {
                    other.render(camX, camY, x, y0, false, 1.0, 0.0, noise);
                    foundWhenGone += tracker.update(other.frame.data(), kFrameW, kFrameH, kFrameW).found ? 1 : 0;
                    ++gonePictures;
                }
            }
        }
        char detail[120];
        std::snprintf(detail, sizeof detail, "(missed %d times while it was there; found in %d of %d pictures after it went)",
                      missedWhileThere, foundWhenGone, gonePictures);
        check(missedWhileThere == 0, "72 places: the object is found in every picture while it is whole in the picture", detail);
        check(foundWhenGone <= 72, "72 places: ground is seldom taken for an object that has gone", detail);
    }

    // --- What it refuses ---------------------------------------------------------
    {
        ObjectTracker tracker;
        std::vector<std::uint8_t> plain(kFrameW * kFrameH, 128);
        check(!tracker.start(plain.data(), kFrameW, kFrameH, kFrameW, Box{190, 100, 30, 30}),
              "a plain area is refused: there is nothing to follow");
        check(!tracker.active(), "and the tracker stays off");
        // Grain of one grey level is not something to follow either.
        Random grain(5);
        for (std::uint8_t &v : plain) {
            v = static_cast<std::uint8_t>(127 + grain.below(3));
        }
        check(!tracker.start(plain.data(), kFrameW, kFrameH, kFrameW, Box{190, 100, 30, 30}),
              "an area with nothing but grain is refused");
        world.render(400, 200, 190, 110);
        check(!tracker.start(world.frame.data(), kFrameW, kFrameH, kFrameW, Box{-5, 100, 30, 30}),
              "a box outside the picture is refused");
        check(!tracker.start(world.frame.data(), kFrameW, kFrameH, kFrameW, Box{100, 100, 0, 30}),
              "a box with no width is refused");
        check(!tracker.start(world.frame.data(), kFrameW, kFrameH, kFrameW, Box{100, 100, std::nan(""), 30}),
              "a box that is not a number is refused");
        check(!tracker.start(nullptr, kFrameW, kFrameH, kFrameW, Box{100, 100, 30, 30}), "no picture is refused");
        const Result r = tracker.update(world.frame.data(), kFrameW, kFrameH, kFrameW);
        check(!r.found, "update without a start finds nothing");
        check(tracker.start(world.frame.data(), kFrameW, kFrameH, kFrameW, Box{190, 110, 20, 40}), "a good box starts");
        tracker.stop();
        check(!tracker.active() && !tracker.update(world.frame.data(), kFrameW, kFrameH, kFrameW).found,
              "after stop it finds nothing");
    }
    {
        // A box at the very edge and one larger than the picture: no crash, and a sane box.
        ObjectTracker tracker;
        world.setObject(20, 40);
        world.render(400, 200, 5, 5);
        const bool started = tracker.start(world.frame.data(), kFrameW, kFrameH, kFrameW, Box{5, 5, 20, 40});
        for (int i = 0; i < 5 && started; ++i) {
            tracker.update(world.frame.data(), kFrameW, kFrameH, kFrameW);
        }
        check(tracker.start(world.frame.data(), kFrameW, kFrameH, kFrameW, Box{190, 108, 5000, 5000}),
              "a box larger than the picture is cut to the picture");
        check(tracker.box().w <= 0.9 * kFrameW + 1e-6 && tracker.box().h <= 0.9 * kFrameH + 1e-6, "its size is inside the picture");
        // A picture row longer than its width (as QImage gives).
        std::vector<std::uint8_t> padded((kFrameW + 13) * kFrameH, 0);
        world.render(400, 200, 190, 110);
        for (int y = 0; y < kFrameH; ++y) {
            std::copy(world.frame.begin() + y * kFrameW, world.frame.begin() + (y + 1) * kFrameW,
                      padded.begin() + y * (kFrameW + 13));
        }
        ObjectTracker a;
        ObjectTracker b;
        a.start(world.frame.data(), kFrameW, kFrameH, kFrameW, Box{190, 110, 20, 40});
        b.start(padded.data(), kFrameW, kFrameH, kFrameW + 13, Box{190, 110, 20, 40});
        world.render(400, 200, 196, 113);
        for (int y = 0; y < kFrameH; ++y) {
            std::copy(world.frame.begin() + y * kFrameW, world.frame.begin() + (y + 1) * kFrameW,
                      padded.begin() + y * (kFrameW + 13));
        }
        const Result ra = a.update(world.frame.data(), kFrameW, kFrameH, kFrameW);
        const Result rb = b.update(padded.data(), kFrameW, kFrameH, kFrameW + 13);
        check(ra.found && rb.found && std::abs(ra.box.cx - rb.box.cx) < 1e-9 && std::abs(ra.box.cy - rb.box.cy) < 1e-9,
              "rows longer than the picture's width are read right");
        char detail[80];
        std::snprintf(detail, sizeof detail, "(%.2f, %.2f)", ra.box.cx, ra.box.cy);
        check(std::abs(ra.box.cx - 196) <= 1.0 && std::abs(ra.box.cy - 113) <= 1.0, "a move of 6 and 3 points is measured",
              detail);
    }

    {
        // A colour picture, three bytes a point, whose first colour is the same everywhere:
        // all that is to be seen is in the other two. The brightness must take all three, or
        // this picture is a plain area to the tracker.
        auto inColour = [&](std::vector<std::uint8_t> &out) {
            out.resize(static_cast<size_t>(kFrameW) * kFrameH * 3);
            for (int k = 0; k < kFrameW * kFrameH; ++k) {
                out[3 * k] = 128;
                out[3 * k + 1] = world.frame[k];
                out[3 * k + 2] = world.frame[k];
            }
        };
        ObjectTracker tracker;
        std::vector<std::uint8_t> colour;
        world.setObject(20, 40);
        world.render(400, 200, 190, 110);
        inColour(colour);
        const bool started = tracker.start(Picture{colour.data(), kFrameW, kFrameH, kFrameW * 3, 3}, Box{190, 110, 20, 40});
        world.render(400, 200, 196, 113);
        inColour(colour);
        const Result r = tracker.update(Picture{colour.data(), kFrameW, kFrameH, kFrameW * 3, 3});
        char detail[80];
        std::snprintf(detail, sizeof detail, "(started %d, found %d, box at %.2f, %.2f)", started, r.found, r.box.cx, r.box.cy);
        check(started && r.found && std::abs(r.box.cx - 196) <= 1.0 && std::abs(r.box.cy - 113) <= 1.0,
              "a colour picture with nothing in its first colour is followed by the other two", detail);
        check(!tracker.start(Picture{colour.data(), kFrameW, kFrameH, kFrameW * 3, 2}, Box{190, 110, 20, 40}),
              "a picture with two bytes a point is refused");
    }

    // How long one picture takes (told, not judged: the test PC is not the remote).
    {
        ObjectTracker tracker;
        world.setObject(20, 40);
        world.render(400, 200, 100, 110);
        tracker.start(world.frame.data(), kFrameW, kFrameH, kFrameW, Box{100, 110, 20, 40});
        const auto t0 = std::chrono::steady_clock::now();
        for (int i = 1; i <= 100; ++i) {
            world.render(400, 200, 100 + 1.5 * i, 110);
            tracker.update(world.frame.data(), kFrameW, kFrameH, kFrameW);
        }
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        std::printf("(100 pictures with their drawing: %.0f ms on this PC) ", ms);
    }

    std::printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
