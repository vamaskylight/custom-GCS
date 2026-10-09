#pragma once

// Follows one object from picture to picture, for the object lock.
//
// Why the app has its own tracker: the lock first used the camera's own
// tracker (GOT and SUM). In two field videos (a parked car on 2026-10-06, a
// walking person on 2026-10-09) the V13 did not follow, and the camera reports
// nothing about what it tracks. VGCS follows a Skydroid camera the same way,
// with its own tracker and gimbal speed commands (its M14).
//
// Two things are learned about the object, because each fails where the other holds:
//  - its COLOURS: which colours are found inside the box and not around it. A
//    person who turns, and whose legs and arms swing, keeps their colours.
//  - its LAYOUT: a correlation filter (MOSSE, Bolme et al. 2010) whose answer to
//    the patch around the object is one sharp peak at its centre. That is exact
//    to a fraction of a point, and it is all there is for an object in the
//    colours of the ground (and in a grey picture such as the thermal one).
// The filter does not look at the bare picture: every point is first weighed by
// how much its colour belongs to the object. So the ground behind the object,
// which a filter on the bare picture learns as well and then holds on to when
// the object walks away, hardly counts. (The first tracker here was the filter
// alone. It followed a stiff test pattern and lost a walking, turning figure in
// most cases.)
//
// How it is done, picture by picture:
//  - the box, with as much again around it, is stretched to 64 x 64 points
//    (each axis by its own factor, so a tall person fills the patch as a car does),
//  - the filter is asked first where the object should be by now; when its answer
//    is not sharp, also half a box to each side and where the object was last seen,
//  - a sharp match (PSR) is believed where it is. A weaker one only where the object
//    should be: over ground, the highest of the filter's answers lies anywhere,
//  - when the colours tell the object from the ground, the box must also hold enough
//    of them. When the layout is not sure, the colours are asked where they are,
//  - the colours and the filter keep learning, fully from sharp matches and a
//    quarter as much from weaker ones (so a turning person is kept, and a wall
//    that covers them is not learned as the object),
//  - when the object is not seen, nothing is learned and the box goes on as the
//    object was moving, slower each picture.
//
// On a test bench (a drawn walking figure; in colour, in the colours of the
// ground, in a grey thermal and a grey day picture; the camera still, following,
// or turned by this tracker as the app does) a box drawn around the figure was
// kept in 339 of 360 walks. Each rule above was measured there with the rule
// taken out, and the walks it decides are in object_tracker_walk_test.cc.
// Known limits, seen there too: a wall or a post in the object's colours (or, in
// a grey picture, in its grey levels) can take the box, and then the app says it
// follows something that is not the object. Watch the box.
//
// No Qt in here: the host test (apk/custom/test/object_tracker_test.cc) builds
// this file alone.

#include <complex>
#include <cstdint>
#include <vector>

namespace skydroid::track {

/// A box in picture points: its centre and its size.
struct Box
{
    double cx = 0.0;
    double cy = 0.0;
    double w = 0.0;
    double h = 0.0;
};

/// A picture: rows of points.
struct Picture
{
    const std::uint8_t *data = nullptr;
    int width = 0;
    int height = 0;
    int stride = 0;    // bytes from one row to the next
    int channels = 1;  // 1: grey. 3 or 4: the first three bytes of a point are its colours, in any order
};

struct Result
{
    bool found = false;  // the object was found in this picture
    Box box;             // where it is (where it was last seen when not found)
    double psr = 0.0;    // how sharp the match is
    double alike = 0.0;  // how much of the box has the object's colours: 1 as lately, 0 none (1 when the colours do not tell)
};

class ObjectTracker
{
public:
    static constexpr int kSize = 64;             // the patch is kSize x kSize points
    static constexpr double kPadding = 2.0;      // the patch covers the box times this
    static constexpr double kMinBox = 12.0;      // a smaller box is widened to this, in picture points
    static constexpr double kPsrFound = 6.0;     // at least this sharp: the object may be there
    static constexpr double kPsrLearn = 10.0;    // sharp: believed anywhere in reach, and learned from fully
    static constexpr double kPsrSure = 20.0;     // this sharp: it is the object, whatever its colours say
    static constexpr double kAlikeFound = 0.15;  // the box holds at least this much of the object's colours
    static constexpr double kAlikeLearn = 0.5;   // colours found by themselves: at least this much of them
    static constexpr double kMinContrast = 2.0;  // grey levels: a flatter box holds nothing to follow

    ObjectTracker();

    /// Starts on the box in this picture. False when the box cannot be followed:
    /// outside the picture, or a plain area with nothing in it.
    bool start(const Picture &picture, const Box &box);
    /// Finds the object in the next picture.
    Result update(const Picture &picture);
    /// The same for a grey picture, one byte for a point.
    bool start(const std::uint8_t *gray, int width, int height, int stride, const Box &box);
    Result update(const std::uint8_t *gray, int width, int height, int stride);
    void stop();

    bool active() const { return _active; }
    const Box &box() const { return _box; }
    /// Pictures in a row in which the object was not found.
    int missed() const { return _missed; }
    /// How much the colours tell the object from the ground around it: 0 nothing, 1 everything.
    double colourTells() const;

private:
    using Complex = std::complex<float>;
    using Field = std::vector<Complex>;  // kSize * kSize, row by row
    static constexpr int kBins = 4096;   // 16 steps of each of the three colours

    /// One look at the picture: the patch around a centre.
    struct Look
    {
        std::vector<float> levels;        // the brightness of each point, levelled
        std::vector<std::uint16_t> bins;  // the colour of each point, as its place in the colour table
    };

    struct Answer
    {
        double psr = 0.0;
        double dx = 0.0;  // the peak's place from the patch centre, in patch points
        double dy = 0.0;
    };

    /// Samples the patch around a centre. False for a plain area.
    bool _sample(const Picture &picture, double cx, double cy, Look &look) const;
    /// What the filter sees of a look: its spectrum, with every point weighed by its colour.
    void _features(const Look &look, Field &layout, Field &colour) const;
    Answer _respond(const Field &layout, const Field &colour) const;
    void _learn(const Field &layout, const Field &colour, float rate);
    void _learnColours(const Look &look, float rate);
    /// How much of the box of this look has the object's colours, and how much of what is around it.
    void _fill(const Look &look, double &inside, double &around) const;
    double _alike(const Look &look) const;
    bool _coloursTell() const;
    /// Moves a window of the box's size to the middle of the object's colours near a place.
    /// False when too little of them is in sight there.
    bool _shiftToColours(const Picture &picture, double cx, double cy, double &outX, double &outY) const;
    static void _fft(Field &field, bool inverse);

    bool _active = false;
    Box _box;
    double _vx = 0.0;  // how far the object moved per picture, lately
    double _vy = 0.0;
    double _seenX = 0.0;  // where the object was last seen
    double _seenY = 0.0;
    int _missed = 0;
    std::vector<float> _window;  // Hann window, kSize * kSize
    Field _target;               // the wanted answer (one peak at the centre), as a spectrum
    Field _layout;               // the filter is (_layout, _colour) / (_power + small)
    Field _colour;
    std::vector<float> _power;
    std::vector<float> _inBox;   // how often each colour is found in the box, sums to 1
    std::vector<float> _around;  // and around it
    std::vector<float> _belongs; // from the two: how much a colour belongs to the object, 0 to 1
    std::vector<float> _own;     // and how much it is the object's own: 0 for a colour as common around it
    double _fillInside = 0.0;    // how much of the box has the object's own colours, lately
    double _fillAround = 0.0;    // and how much of what is around it
    mutable Field _scratch;
    mutable std::vector<float> _counts;
};

} // namespace skydroid::track
