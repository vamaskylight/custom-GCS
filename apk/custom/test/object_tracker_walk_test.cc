// Host test for ObjectTracker with a walking figure that is NOT stiff: its legs and
// arms swing, and it turns from facing the camera to side-on, which makes it
// narrower and squeezes what is on its shirt. The first tracker of the app passed
// every test with stiff patterns and lost such a figure in most walks, so this
// test is the one that matters for "the camera follows a walking person".
//
// The figure is drawn by hand into a made-up world, in colour (clothes that stand
// out, and clothes in the colours of the ground), in a grey thermal picture (a warm
// figure) and in a grey day picture. Each case knows where the figure really is.
// The camera is still, or it is turned by what the tracker says, as the app does:
// the picture the tracker gets is 0.3 s old, and the camera waits when the figure
// is not seen.
//
// The walks are those of the bench the rules were chosen with (3456 walks there:
// six ways, four kinds of picture, three cameras, eight boxes, six places).
// Here: some sums over the main cases, and single walks that one rule decides.
// "-v" prints every walk.

#include "ObjectTracker.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <string>
#include <vector>

using namespace skydroid::track;

namespace {

int g_checks = 0;
int g_failures = 0;
bool g_verbose = false;

constexpr int kFrameW = 384;
constexpr int kFrameH = 216;
constexpr int kSceneW = 2200;
constexpr int kSceneH = 700;
constexpr double kPicturesPerSecond = 14.0;

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

struct Rgb
{
    int r = 0;
    int g = 0;
    int b = 0;
};

Rgb mix(const Rgb &a, const Rgb &b, double t)
{
    return Rgb{static_cast<int>(a.r + (b.r - a.r) * t), static_cast<int>(a.g + (b.g - a.g) * t),
               static_cast<int>(a.b + (b.b - a.b) * t)};
}

Rgb scaled(const Rgb &a, double f)
{
    return Rgb{static_cast<int>(a.r * f), static_cast<int>(a.g * f), static_cast<int>(a.b * f)};
}

// The kinds of picture.
enum Kind { kStandOut = 0, kGroundColours = 1, kThermal = 2, kGreyDay = 3 };
const char *kKindNames[] = {"colour, clothes that stand out", "colour, clothes in the colours of the ground",
                            "thermal, a warm figure", "grey day picture"};

// The world: soft areas, buildings, grain. kind 0: a field in colour. 1: a town in colour. 2: grey, for thermal.
std::vector<Rgb> makeWorld(std::uint32_t seed, int kind)
{
    Random rnd(seed);
    const int cell = 24;
    const int gw = kSceneW / cell + 2;
    const int gh = kSceneH / cell + 2;
    const Rgb earth[] = {{72, 104, 48}, {150, 140, 88}, {124, 96, 62}, {40, 62, 34}, {178, 160, 120}, {96, 110, 70}};
    const Rgb town[] = {{120, 120, 124}, {160, 158, 150}, {90, 92, 96}, {140, 128, 110}, {70, 100, 60}, {180, 176, 168}};
    std::vector<Rgb> grid(gw * gh);
    for (Rgb &v : grid) {
        if (kind == 2) {
            const int g = 60 + rnd.below(90);
            v = Rgb{g, g, g};
        } else {
            const Rgb *palette = kind == 0 ? earth : town;
            v = scaled(mix(palette[rnd.below(6)], palette[rnd.below(6)], rnd.below(100) / 100.0), 0.8 + rnd.below(36) / 100.0);
        }
    }
    std::vector<Rgb> world(kSceneW * kSceneH);
    for (int y = 0; y < kSceneH; ++y) {
        for (int x = 0; x < kSceneW; ++x) {
            const double fx = static_cast<double>(x) / cell;
            const double fy = static_cast<double>(y) / cell;
            const int x0 = static_cast<int>(fx);
            const int y0 = static_cast<int>(fy);
            const Rgb top = mix(grid[y0 * gw + x0], grid[y0 * gw + x0 + 1], fx - x0);
            const Rgb bottom = mix(grid[(y0 + 1) * gw + x0], grid[(y0 + 1) * gw + x0 + 1], fx - x0);
            world[y * kSceneW + x] = mix(top, bottom, fy - y0);
        }
    }
    const int buildings = kind == 1 ? 260 : 140;
    for (int i = 0; i < buildings; ++i) {
        const int w = 20 + rnd.below(80);
        const int h = 15 + rnd.below(60);
        const int x0 = rnd.below(kSceneW - w);
        const int y0 = rnd.below(kSceneH - h);
        Rgb colour;
        const int pick = rnd.below(100);
        const int grey = 40 + rnd.below(180);
        if (kind == 2 || pick < 50) {
            colour = Rgb{grey, grey, grey};
        } else if (pick < 70) {
            colour = Rgb{150 + rnd.below(30), 60 + rnd.below(30), 44 + rnd.below(20)};  // a red roof
        } else if (pick < 85) {
            colour = Rgb{225, 224, 214};
        } else {
            colour = Rgb{58 + rnd.below(20), 80 + rnd.below(20), 130 + rnd.below(30)};  // a blue roof
        }
        for (int y = y0; y < y0 + h; ++y) {
            for (int x = x0; x < x0 + w; ++x) {
                world[y * kSceneW + x] = colour;
            }
        }
    }
    for (Rgb &v : world) {  // grain
        if (kind == 2) {
            const int n = rnd.below(13) - 6;
            v = Rgb{v.r + n, v.g + n, v.b + n};
        } else {
            v = Rgb{v.r + rnd.below(13) - 6, v.g + rnd.below(13) - 6, v.b + rnd.below(13) - 6};
        }
    }
    return world;
}

struct Frame
{
    int channels = 3;
    std::vector<std::uint8_t> data;
    void reset(int ch)
    {
        channels = ch;
        data.assign(static_cast<size_t>(kFrameW) * kFrameH * ch, 0);
    }
    void put(int x, int y, const Rgb &c)
    {
        if (x < 0 || x >= kFrameW || y < 0 || y >= kFrameH) {
            return;
        }
        std::uint8_t *p = data.data() + (static_cast<size_t>(y) * kFrameW + x) * channels;
        if (channels == 1) {
            p[0] = static_cast<std::uint8_t>(std::clamp((c.r + 2 * c.g + c.b) / 4, 0, 255));
        } else {
            p[0] = static_cast<std::uint8_t>(std::clamp(c.r, 0, 255));
            p[1] = static_cast<std::uint8_t>(std::clamp(c.g, 0, 255));
            p[2] = static_cast<std::uint8_t>(std::clamp(c.b, 0, 255));
        }
    }
    Picture picture() const { return Picture{data.data(), kFrameW, kFrameH, kFrameW * channels, channels}; }
};

void renderWorld(Frame &frame, const std::vector<Rgb> &world, double camX, double camY)
{
    const int cx = static_cast<int>(std::lround(camX));
    const int cy = static_cast<int>(std::lround(camY));
    for (int y = 0; y < kFrameH; ++y) {
        const int sy = std::clamp(y + cy, 0, kSceneH - 1);
        for (int x = 0; x < kFrameW; ++x) {
            const int sx = std::clamp(x + cx, 0, kSceneW - 1);
            frame.put(x, y, world[sy * kSceneW + sx]);
        }
    }
}

// The figure: head, body, two arms, two legs.
struct Figure
{
    double height = 60.0;  // head to feet, picture points
    double phase = 0.0;    // of the step, radians
    double side = 0.0;     // 0: facing the camera, 1: seen from the side
    double swing = 0.0;    // 0: standing, 1: walking
    Rgb shirt{205, 205, 205};
    Rgb trousers{45, 45, 50};
    Rgb skin{196, 152, 122};
    Rgb hair{36, 28, 22};
    std::vector<int> marks;  // what is on the shirt, 6 x 8 fields: 1 = a dark mark
};

void dress(Figure &f, int kind, int place)
{
    const Rgb shirts[] = {{228, 226, 220}, {40, 72, 170}, {186, 44, 40}, {34, 32, 34}, {220, 190, 50}, {170, 170, 172}};
    const Rgb legs[] = {{38, 44, 78}, {28, 28, 30}, {112, 112, 116}, {62, 86, 130}, {86, 62, 44}, {40, 40, 44}};
    const Rgb groundShirts[] = {{84, 104, 58}, {150, 138, 92}, {110, 100, 66}, {70, 88, 50}, {130, 120, 84}, {96, 108, 70}};
    const Rgb groundLegs[] = {{96, 98, 62}, {124, 98, 66}, {80, 92, 54}, {120, 112, 78}, {90, 80, 56}, {140, 128, 90}};
    if (kind == kGroundColours) {
        f.shirt = groundShirts[place % 6];
        f.trousers = groundLegs[place % 6];
    } else if (kind == kThermal) {
        f.shirt = Rgb{215, 215, 215};
        f.trousers = Rgb{198, 198, 198};
        f.skin = Rgb{240, 240, 240};
        f.hair = Rgb{226, 226, 226};
    } else {
        f.shirt = shirts[place % 6];
        f.trousers = legs[place % 6];
    }
    Random rnd(5 + place);
    f.marks.resize(48);
    for (int &m : f.marks) {
        m = rnd.below(3) == 0 ? 1 : 0;
    }
}

void thickLine(Frame &frame, double x0, double y0, double x1, double y1, double thick, const Rgb &colour)
{
    const int left = static_cast<int>(std::floor(std::min(x0, x1) - thick));
    const int right = static_cast<int>(std::ceil(std::max(x0, x1) + thick));
    const int top = static_cast<int>(std::floor(std::min(y0, y1) - thick));
    const int bottom = static_cast<int>(std::ceil(std::max(y0, y1) + thick));
    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double len2 = std::max(1e-9, dx * dx + dy * dy);
    for (int y = top; y <= bottom; ++y) {
        for (int x = left; x <= right; ++x) {
            const double t = std::clamp(((x - x0) * dx + (y - y0) * dy) / len2, 0.0, 1.0);
            if (std::hypot(x - (x0 + t * dx), y - (y0 + t * dy)) <= thick / 2.0) {
                frame.put(x, y, colour);
            }
        }
    }
}

// Draws the figure with its middle at (cx, cy).
void drawFigure(Frame &frame, const Figure &f, double cx, double cy)
{
    const double H = f.height;
    const double top = cy - H / 2.0 + 0.012 * H * std::abs(std::sin(f.phase)) * f.swing;
    const double bodyW = H * (0.26 * (1.0 - f.side) + 0.15 * f.side);
    const double shoulderY = top + 0.17 * H;
    const double hipY = top + 0.52 * H;
    const double armLen = 0.33 * H;
    const double legLen = 0.48 * H;
    const double armSwing = 0.52 * f.swing * f.side * std::sin(f.phase);
    const double legSwing = 0.48 * f.swing * f.side * std::sin(f.phase);
    const Rgb shirt = scaled(f.shirt, 1.0 - 0.18 * f.side);  // the light falls on it differently
    const double farX = cx - bodyW / 2.0 * (1.0 - f.side);
    const double nearX = cx + bodyW / 2.0 * (1.0 - f.side);
    const double legX = bodyW / 4.0 * (1.0 - f.side);
    // The leg and the arm on the far side first.
    thickLine(frame, cx - legX, hipY, cx - legX + legLen * std::sin(-legSwing), hipY + legLen * std::cos(legSwing), 0.085 * H,
              scaled(f.trousers, 1.15));
    thickLine(frame, farX, shoulderY, farX + armLen * std::sin(armSwing), shoulderY + armLen * std::cos(armSwing), 0.06 * H,
              scaled(shirt, 0.88));
    const int bx0 = static_cast<int>(std::lround(cx - bodyW / 2.0));
    const int bx1 = static_cast<int>(std::lround(cx + bodyW / 2.0));
    const int by0 = static_cast<int>(std::lround(top + 0.14 * H));
    const int by1 = static_cast<int>(std::lround(hipY + 0.03 * H));
    for (int y = by0; y < by1; ++y) {
        for (int x = bx0; x < bx1; ++x) {
            const int u = std::clamp(static_cast<int>(6.0 * (x - bx0) / std::max(1, bx1 - bx0)), 0, 5);
            const int v = std::clamp(static_cast<int>(8.0 * (y - by0) / std::max(1, by1 - by0)), 0, 7);
            frame.put(x, y, f.marks[v * 6 + u] ? scaled(shirt, 0.6) : shirt);
        }
    }
    thickLine(frame, cx + legX, hipY, cx + legX + legLen * std::sin(legSwing), hipY + legLen * std::cos(legSwing), 0.085 * H,
              f.trousers);
    thickLine(frame, nearX, shoulderY, nearX + armLen * std::sin(-armSwing), shoulderY + armLen * std::cos(armSwing), 0.06 * H,
              scaled(shirt, 0.96));
    const double headR = 0.068 * H;
    const double headY = top + 0.07 * H;
    for (int y = static_cast<int>(headY - headR - 1); y <= static_cast<int>(headY + headR + 1); ++y) {
        for (int x = static_cast<int>(cx - headR - 1); x <= static_cast<int>(cx + headR + 1); ++x) {
            if (std::hypot(x - cx, (y - headY) * 0.85) <= headR) {
                const bool isHair = (y < headY - 0.35 * headR) || (f.side > 0.5 && x < cx - 0.1 * headR);
                frame.put(x, y, isHair ? f.hair : f.skin);
            }
        }
    }
}

// A blotchy surface that belongs to the world: the same at the same place in every picture.
double blotch(int worldX, int y)
{
    std::uint32_t h = static_cast<std::uint32_t>(worldX / 3) * 73856093u ^ static_cast<std::uint32_t>(y / 3) * 19349663u;
    h ^= h >> 13;
    h *= 1274126177u;
    h ^= h >> 16;
    return 0.86 + (h % 1000) / 1000.0 * 0.28;
}

// The box the operator puts on the figure.
struct BoxOnFigure
{
    const char *name;
    double height;  // of the figure, points
    double boxW;
    double boxH;
    double dx;  // the box's middle from the figure's middle
    double dy;
};

const BoxOnFigure kNearHeadAndChest{"near (110 high), box on head and chest", 110, 34, 46, 0, -32};
const BoxOnFigure kNearWhole{"near (110 high), box on the whole figure", 110, 36, 112, 0, 0};
const BoxOnFigure kMiddleWhole{"middle (48 high), box on the whole figure", 48, 16, 50, 0, 0};
const BoxOnFigure kMiddleBody{"middle (48 high), box on the body only", 48, 14, 20, 0, -8};
const BoxOnFigure kFarWhole{"far (22 high), box on the whole figure", 22, 8, 24, 0, 0};
const BoxOnFigure kFarTap{"far (22 high), a tap: box 26 x 26", 22, 26, 26, 0, 0};
const BoxOnFigure kMiddleTap{"middle (48 high), a tap: 26 x 26 on the body", 48, 26, 26, 0, -8};
const BoxOnFigure kNearTap{"near (110 high), a tap: 26 x 26 on the chest", 110, 26, 26, 0, -22};

enum Camera { kStill = 0, kFollowsExactly = 1, kTurnedByTracker = 2 };
const char *kCameraNames[] = {"camera still", "camera follows exactly", "camera turned by the tracker"};

// What is in the figure's way.
enum Way { kOpen = 0, kPost = 1, kWidePost = 2, kCross = 3, kWall = 4, kShadow = 5 };
const char *kWayNames[] = {"walk", "post", "wide post", "cross", "wall", "shadow"};

struct Outcome
{
    int lost = 0;  // pictures in which the figure was in sight and not found
    int pictures = 0;
    int foundWhileHidden = 0;
    int hiddenPictures = 0;
    int foundLately = 0;  // of the last 8 pictures
    double endDx = 0.0;   // at the end, how far the box is from its place on the figure
    double endDy = 0.0;
    bool refused = false;
    bool outOfPicture = false;

    // The middle of the box ends on the figure (the cross would be on the person), and the
    // figure was found in at least nine of ten pictures (in any of them, with something in the way).
    bool kept(const BoxOnFigure &b, bool gapsAllowed) const
    {
        if (refused || outOfPicture) {
            return false;
        }
        const bool onIt = endDx <= std::max(6.0, 0.15 * b.height) && endDy <= std::max(6.0, 0.5 * b.height);
        return onIt && (gapsAllowed || lost <= pictures / 10);
    }
    // Not kept, and the tracker still says "found" at the end: it follows something else.
    bool wrong(const BoxOnFigure &b, bool gapsAllowed) const { return !kept(b, gapsAllowed) && foundLately >= 6; }
};

// One walk: the figure stands for 8 pictures, turns to the side while it starts to walk,
// walks, stops and turns back.
//  kPost: a dark post half as wide as the figure stands in the way; kWidePost: one and a half times.
//  kCross: another figure comes the other way and passes in front. kWall: the figure walks behind a
//  wall and stays there. kShadow: it walks into a deep shadow.
Outcome walk(const std::vector<Rgb> &world, int kind, int place, const BoxOnFigure &b, Camera camera, Way way)
{
    Outcome out;
    ObjectTracker tracker;
    Random grain(17);
    Frame frame;
    frame.reset(kind >= kThermal ? 1 : 3);
    Figure f;
    f.height = b.height;
    dress(f, kind, place);
    Figure other;
    other.height = b.height;
    dress(other, kind, place + 7);
    other.side = 1.0;
    other.swing = 1.0;
    const int noise = place % 2 ? 6 : 0;
    const double camY = 100 + 40.0 * place;
    double camX = 300 + 90.0 * place;
    const double speed = 0.057 * b.height;  // points a picture: a walk
    const int turnPictures = 6;
    const int walkPictures = std::min(90, static_cast<int>((camera == kStill ? 250 : 500) / speed));
    const int total = 8 + turnPictures + walkPictures + turnPictures + 8;
    double figX = camX + (camera == kStill ? 60.0 : 192.0);
    const double figY = camY + 108.0;
    double otherX = figX + 2.0 * speed * (8 + turnPictures + walkPictures / 2.0);  // they meet half way
    const int post = way == kPost ? std::max(3, static_cast<int>(0.5 * 0.2 * b.height))
                                  : (way == kWidePost ? std::max(3, static_cast<int>(1.5 * 0.2 * b.height)) : 0);
    const double postX = figX + speed * (turnPictures / 2.0 + walkPictures * 0.45);
    const double edgeX = figX + speed * (turnPictures / 2.0 + walkPictures * 0.4);  // of the wall and of the shadow

    struct State
    {
        double figX;
        double camX;
        double otherX;
        Figure f;
        Figure other;
    };
    std::deque<State> history;
    std::deque<double> moves;  // how far the camera turned in each of the last pictures
    const int delay = camera == kTurnedByTracker ? 4 : 0;  // the picture the tracker gets is this many pictures old

    for (int i = 0; i < total; ++i) {
        double moving = 0.0;
        if (i >= 8 && i < 8 + turnPictures) {
            f.side = (i - 8 + 1) / static_cast<double>(turnPictures);
            moving = f.side;
        } else if (i >= 8 + turnPictures && i < 8 + turnPictures + walkPictures) {
            f.side = 1.0;
            moving = 1.0;
        } else if (i >= 8 + turnPictures + walkPictures && i < 8 + 2 * turnPictures + walkPictures) {
            f.side = 1.0 - (i - 8 - turnPictures - walkPictures + 1) / static_cast<double>(turnPictures);
            moving = f.side;
        } else {
            f.side = 0.0;
        }
        f.swing = moving;
        f.phase += 0.42 * moving;
        figX += speed * moving;
        if (camera == kFollowsExactly) {
            camX += speed * moving;
        }
        other.phase += 0.42;
        otherX -= speed;
        history.push_back(State{figX, camX, otherX, f, other});
        if (static_cast<int>(history.size()) > delay + 1) {
            history.pop_front();
        }
        const State &seen = history.front();

        renderWorld(frame, world, seen.camX, camY);
        drawFigure(frame, seen.f, seen.figX - seen.camX, figY - camY);
        if (way == kCross) {
            drawFigure(frame, seen.other, seen.otherX - seen.camX, figY - camY + 2);
        }
        bool hidden = false;
        if (post > 0 || way == kWall) {
            const double from = way == kWall ? edgeX : postX - post / 2.0;
            const double to = way == kWall ? edgeX + 4000 : postX + post / 2.0;
            const Rgb dark = kind == kThermal ? Rgb{70, 70, 70} : Rgb{64, 52, 44};
            const int worldLeft = static_cast<int>(std::lround(seen.camX));
            for (int py = 0; py < kFrameH; ++py) {
                for (int px = std::max(0, static_cast<int>(from - seen.camX)); px < static_cast<int>(to - seen.camX) && px < kFrameW; ++px) {
                    frame.put(px, py, scaled(dark, blotch(px + worldLeft, py)));
                }
            }
            hidden = seen.figX - 0.15 * b.height > from && seen.figX + 0.15 * b.height < to;
        }
        if (way == kShadow) {
            const int from = std::max(0, static_cast<int>(edgeX - seen.camX));
            for (int py = 0; py < kFrameH; ++py) {
                for (int px = from; px < kFrameW; ++px) {
                    std::uint8_t *v = frame.data.data() + (static_cast<size_t>(py) * kFrameW + px) * frame.channels;
                    for (int c = 0; c < frame.channels; ++c) {
                        v[c] = static_cast<std::uint8_t>(v[c] * 0.55);
                    }
                }
            }
        }
        if (noise > 0) {
            for (std::uint8_t &v : frame.data) {
                v = static_cast<std::uint8_t>(std::clamp(static_cast<int>(v) + grain.below(2 * noise + 1) - noise, 0, 255));
            }
        }
        const double wantX = seen.figX - seen.camX + b.dx;
        const double wantY = figY - camY + b.dy;
        if (i == 0) {
            if (!tracker.start(frame.picture(), Box{wantX, wantY, b.boxW, b.boxH})) {
                out.refused = true;
                return out;
            }
            moves.push_back(0.0);
            continue;
        }
        const Result r = tracker.update(frame.picture());
        ++out.pictures;
        if (hidden) {
            ++out.hiddenPictures;
            out.foundWhileHidden += r.found ? 1 : 0;
        } else {
            out.lost += r.found ? 0 : 1;
        }
        out.endDx = std::abs(r.box.cx - wantX);
        out.endDy = std::abs(r.box.cy - wantY);
        out.outOfPicture = wantX < 0 || wantX > kFrameW;
        if (i >= total - 8) {
            out.foundLately += r.found ? 1 : 0;
        }
        if (camera == kTurnedByTracker) {
            // As the app does: turn towards the object, less what the camera turned since the
            // picture was taken (8 points are a degree here). Not seen: the camera waits.
            double turned = 0.0;
            for (double m : moves) {
                turned += m;
            }
            double move = 0.0;
            if (r.found) {
                const double offPoints = (r.box.cx - b.dx) - kFrameW / 2.0 - turned;
                const double most = std::clamp(0.5 * b.boxW * kPicturesPerSecond, 80.0, 320.0) / kPicturesPerSecond;
                if (std::abs(offPoints) > 4.0) {
                    move = std::clamp(2.5 * offPoints / kPicturesPerSecond, -most, most);
                }
            }
            camX += move;
            moves.push_back(move);
            if (static_cast<int>(moves.size()) > 4) {
                moves.pop_front();
            }
        }
    }
    return out;
}

struct Tally
{
    int kept = 0;
    int wrong = 0;
    int cases = 0;
    int foundWhileHidden = 0;
    int hiddenPictures = 0;
};

// A walk is good when the figure is kept. With a post or another figure in the way it may be
// not seen for a while. Behind a wall for good, it is good when the tracker says "not seen".
bool good(const Outcome &o, const BoxOnFigure &b, Way way)
{
    if (way == kWall) {
        return !o.refused && o.foundWhileHidden * 10 <= o.hiddenPictures;
    }
    return o.kept(b, way == kPost || way == kWidePost || way == kCross);
}

bool add(Tally &t, const Outcome &o, const BoxOnFigure &b, Way way, int kind, int place, Camera camera)
{
    ++t.cases;
    const bool ok = good(o, b, way);
    const bool wrong = !ok && way != kWall && o.foundLately >= 6;
    t.kept += ok ? 1 : 0;
    t.wrong += wrong ? 1 : 0;
    t.foundWhileHidden += o.foundWhileHidden;
    t.hiddenPictures += o.hiddenPictures;
    if (g_verbose) {
        std::printf("     %-9s %-44s %-28s %-44s place %d: %s (not found %3d of %3d, found while hidden %2d of %2d, at the end %.1f across, %.1f down)\n",
                    kWayNames[way], kKindNames[kind], kCameraNames[camera], b.name, place, ok ? "good " : (wrong ? "WRONG" : "lost "),
                    o.lost, o.pictures, o.foundWhileHidden, o.hiddenPictures, o.endDx, o.endDy);
    }
    return ok;
}

// One walk of the bench, by its numbers.
struct Walk
{
    Way way;
    int kind;
    Camera camera;
    const BoxOnFigure *box;
    int place;
};

} // namespace

int main(int argc, char **argv)
{
    g_verbose = argc > 1 && argv[1][0] == '-' && argv[1][1] == 'v';
    const std::vector<Rgb> worlds[3] = {makeWorld(1234, 0), makeWorld(4321, 1), makeWorld(777, 2)};
    auto worldFor = [&](int kind, int place) -> const std::vector<Rgb> & { return kind == kThermal ? worlds[2] : worlds[place % 2]; };
    auto run = [&](Tally &tally, const Walk &w) {
        return add(tally, walk(worldFor(w.kind, w.place), w.kind, w.place, *w.box, w.camera, w.way), *w.box, w.way, w.kind, w.place, w.camera);
    };
    char detail[200];

    {
        // The main case: a box drawn around the figure (or a part of it), and the camera
        // turned by the tracker, as the app does.
        const BoxOnFigure *boxes[] = {&kNearHeadAndChest, &kMiddleWhole, &kMiddleBody, &kFarWhole};
        Tally all;
        for (int kind = 0; kind < 4; ++kind) {
            for (const BoxOnFigure *b : boxes) {
                for (int place = 0; place < 2; ++place) {
                    run(all, Walk{kOpen, kind, kTurnedByTracker, b, place});
                }
            }
        }
        std::snprintf(detail, sizeof detail, "(kept in %d of %d walks, %d ended on something else)", all.kept, all.cases, all.wrong);
        check(all.kept >= all.cases - 2 && all.wrong <= 1, "a walking, turning figure: the camera is turned to it in nearly every walk", detail);
    }
    {
        // The first second of every lock: the camera has not turned yet, and the figure walks
        // off over still ground (a filter on the bare picture holds on to the ground here).
        const BoxOnFigure *boxes[] = {&kMiddleWhole, &kFarWhole};
        Tally all;
        for (int kind : {kStandOut, kThermal}) {
            for (const BoxOnFigure *b : boxes) {
                for (int place = 0; place < 2; ++place) {
                    run(all, Walk{kOpen, kind, kStill, b, place});
                }
            }
        }
        std::snprintf(detail, sizeof detail, "(kept in %d of %d walks, %d ended on something else)", all.kept, all.cases, all.wrong);
        check(all.kept >= all.cases - 1 && all.wrong == 0, "a figure that walks off over still ground is followed", detail);
    }
    {
        // Behind a wall for good: the app must say "not seen", not hold a box on the wall.
        // (A wall in the clothes' own colours can take the box: a known limit.)
        const BoxOnFigure *boxes[] = {&kNearHeadAndChest, &kMiddleWhole};
        Tally all;
        for (const BoxOnFigure *b : boxes) {
            for (int place : {0, 2, 4}) {
                run(all, Walk{kWall, kStandOut, kStill, b, place});
            }
        }
        std::snprintf(detail, sizeof detail, "(said found in %d of %d pictures while the figure was behind the wall)", all.foundWhileHidden,
                      all.hiddenPictures);
        check(all.hiddenPictures > 0 && all.foundWhileHidden * 10 <= all.hiddenPictures,
              "a figure that walks behind a wall is reported as not seen", detail);
    }

    // Single walks of the bench, each of them decided by one rule of the tracker: good as the
    // tracker is, and not good with that rule taken out (the bench found them). They pin the
    // rules that the sums above do not need. Chosen on 2026-10-09.
    struct Rule
    {
        const char *name;
        std::vector<Walk> walks;
    };
    const std::vector<Rule> rules = {
// --- the walks each rule decides: BEGIN ---
        {"the filter also looks at the picture of the object's colours",
         {
             Walk{kCross, kStandOut, kStill, &kMiddleWhole, 2},
             Walk{kPost, kStandOut, kStill, &kFarWhole, 2},
             Walk{kPost, kGroundColours, kStill, &kMiddleWhole, 0},
             Walk{kPost, kThermal, kStill, &kNearWhole, 1},
         }},
        {"a match that is not sharp is learned from less",
         {
             Walk{kCross, kGroundColours, kStill, &kMiddleWhole, 5},
             Walk{kCross, kGreyDay, kStill, &kNearWhole, 0},
             Walk{kWidePost, kStandOut, kStill, &kFarWhole, 1},
             Walk{kWidePost, kGroundColours, kStill, &kMiddleWhole, 2},
         }},
        {"a match that is not sharp is still learned from a little",
         {
             Walk{kCross, kGroundColours, kStill, &kMiddleBody, 1},
             Walk{kCross, kThermal, kStill, &kMiddleBody, 0},
             Walk{kCross, kGreyDay, kStill, &kNearHeadAndChest, 0},
             Walk{kPost, kGroundColours, kStill, &kNearWhole, 3},
         }},
        {"what the box held of the colours is kept up to date",
         {
             Walk{kPost, kThermal, kStill, &kMiddleBody, 5},
             Walk{kWidePost, kStandOut, kStill, &kNearWhole, 1},
             Walk{kShadow, kGreyDay, kStill, &kFarWhole, 4},
             Walk{kOpen, kThermal, kStill, &kMiddleBody, 5},
         }},
        {"a sure match with other colours: the light changed, the colours are learned anew fast",
         {
             Walk{kShadow, kGreyDay, kStill, &kMiddleBody, 0},
             Walk{kShadow, kThermal, kFollowsExactly, &kNearWhole, 1},
             Walk{kWall, kGreyDay, kStill, &kNearWhole, 3},
             Walk{kShadow, kThermal, kFollowsExactly, &kNearWhole, 5},
         }},
        {"a colour is spread to its neighbours in the table",
         {
             Walk{kCross, kStandOut, kStill, &kNearWhole, 2},
             Walk{kCross, kThermal, kStill, &kMiddleBody, 4},
             Walk{kCross, kGreyDay, kStill, &kMiddleWhole, 3},
             Walk{kPost, kGreyDay, kStill, &kMiddleWhole, 3},
         }},
        {"and further along brighter and darker",
         {
             Walk{kWidePost, kStandOut, kStill, &kMiddleWhole, 0},
             Walk{kWidePost, kThermal, kStill, &kMiddleWhole, 1},
             Walk{kOpen, kStandOut, kStill, &kNearHeadAndChest, 5},
             Walk{kOpen, kThermal, kStill, &kMiddleWhole, 3},
         }},
        {"a sure layout is believed whatever the colours say",
         {
             Walk{kShadow, kThermal, kStill, &kFarWhole, 3},
             Walk{kShadow, kGreyDay, kFollowsExactly, &kMiddleBody, 4},
             Walk{kCross, kStandOut, kFollowsExactly, &kMiddleBody, 5},
             Walk{kWall, kGreyDay, kStill, &kNearWhole, 3},
         }},
        {"the colours are asked only when they tell the object from the ground",
         {
             Walk{kWidePost, kThermal, kFollowsExactly, &kFarWhole, 5},
             Walk{kShadow, kThermal, kStill, &kFarTap, 0},
             Walk{kShadow, kGreyDay, kFollowsExactly, &kFarTap, 0},
             Walk{kWall, kGreyDay, kStill, &kMiddleWhole, 4},
         }},
        {"not seen: the box goes on as the object was moving",
         {
             Walk{kPost, kThermal, kStill, &kNearWhole, 3},
             Walk{kWidePost, kThermal, kStill, &kNearWhole, 2},
             Walk{kWall, kGroundColours, kStill, &kMiddleWhole, 0},
             Walk{kWall, kThermal, kFollowsExactly, &kFarWhole, 3},
         }},
        {"and where the object was last seen",
         {
             Walk{kPost, kThermal, kStill, &kNearHeadAndChest, 5},
             Walk{kPost, kGreyDay, kStill, &kNearWhole, 2},
             Walk{kWidePost, kGroundColours, kStill, &kMiddleWhole, 0},
             Walk{kWidePost, kGreyDay, kStill, &kNearWhole, 2},
         }},
// --- the walks each rule decides: END ---
    };
    for (const Rule &rule : rules) {
        Tally tally;
        std::string failed;
        for (const Walk &w : rule.walks) {
            if (!run(tally, w)) {
                char one[120];
                std::snprintf(one, sizeof one, " [%s, %s, %s, %s, place %d]", kWayNames[w.way], kKindNames[w.kind], kCameraNames[w.camera],
                              w.box->name, w.place);
                failed += one;
            }
        }
        std::snprintf(detail, sizeof detail, "(good in %d of %d walks%s%.120s)", tally.kept, tally.cases, failed.empty() ? "" : ", not in", failed.c_str());
        check(tally.kept == tally.cases, rule.name, detail);
    }

    std::printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
