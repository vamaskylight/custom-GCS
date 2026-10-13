#pragma once

// Finds people and vehicles in a picture of the video, so the operator can tap
// a marked object to lock it instead of drawing a box around it.
//
// The finding is done by a small neural network, NanoDet-Plus-m (Apache-2.0,
// trained on the COCO pictures: people and things photographed from the ground
// for the most part). The ncnn library (BSD 3-Clause) runs it on the processor.
// Where both come from, their hashes and their licences: apk/custom/res/ai.
//
// What to expect of it (measured on 2026-10-13 on 809 views of photos from the
// air, against a network seven times as large: DOCS/APK-V1-DEV-PLAN.md, phase 8):
//  - an object has to be large enough in the picture the network is given. Of
//    people 32 to 48 points tall it finds about two of three, of vehicles that
//    long three of four, and of smaller ones fewer,
//  - straight from above a person is a head and two shoulders, which the network
//    has hardly seen: it finds people best when the camera looks at them from the side,
//  - about 6 of 100 boxes are on something else (a pallet of bricks, a traffic cone),
//  - it marks what looks like a person or a vehicle. It does not know who or what it is.
// So a mark is an offer to the operator, never a decision of the app.
//
// No Qt in here. The network itself is asked in ObjectFinderNcnn.cc only. What
// is made of its answer (decode, pick, merge) and what it is given to look at
// (planLooks) is plain arithmetic in ObjectFinder.cc, and the host test builds
// that file alone.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace skydroid::find {

enum class Kind
{
    Person = 0,
    Vehicle = 1,
};

/// One object found in a picture.
struct Found
{
    Kind kind = Kind::Person;
    int label = 0;       // the network's own class: 0 person, 2 car, 3 motorcycle, 5 bus, 7 truck
    double score = 0.0;  // how sure the network is, 0 to 1
    double cx = 0.0;     // the box in picture points: its centre
    double cy = 0.0;
    double w = 0.0;      // and its size
    double h = 0.0;
};

/// A picture: rows of points, three or four bytes a point, red, green and blue first.
struct Picture
{
    const std::uint8_t *data = nullptr;
    int width = 0;
    int height = 0;
    int stride = 0;  // bytes from one row to the next
    int channels = 3;
};

/// A part of a picture, in its points.
struct Area
{
    int left = 0;
    int top = 0;
    int width = 0;
    int height = 0;
};

/// How a part of a picture lies in the network's input.
struct Fit
{
    int inputW = 0;  // the input's size
    int inputH = 0;
    int filledW = 0;  // the size of the picture part inside it
    int filledH = 0;
    int padLeft = 0;  // and where it starts: the rest of the input is black
    int padTop = 0;
    double scaleX = 1.0;  // input points for one picture point
    double scaleY = 1.0;
};

// The network's answer has one row for every place of its four grids, 8, 16, 32
// and 64 input points apart. A row holds 80 class scores (0 to 1), then for each
// of the box's four sides (left, top, right, bottom) 8 numbers: how likely the
// side is 0, 1, ... 7 grid steps from the place.
constexpr int kClasses = 80;
constexpr int kSideSteps = 8;
constexpr int kRowLength = kClasses + 4 * kSideSteps;
constexpr int kGrids = 4;
constexpr int kGridStep[kGrids] = {8, 16, 32, 64};
/// The input's sides are multiples of this, and at least this.
constexpr int kInputStep = 32;
constexpr int kInputMin = 64;
constexpr int kInputMax = 1280;

/// Boxes of one kind that share more than this much of their area are one object.
constexpr double kSameObjectShare = 0.5;
/// When two finds are merged: a box this much inside a larger box of its kind is a part of that object.
constexpr double kPartOfShare = 0.7;
/// A box that ends this near to the border of the looked-at part may be cut there, in points.
constexpr double kCutMargin = 2.0;
/// The smallest box that counts, in picture points: less is noise.
constexpr double kLeastSide = 4.0;

/// The kind of a network class, or false for a class that is not marked.
bool kindOf(int label, Kind &kind);
const char *kindName(Kind kind);

/// The rows the network answers with for an input of this size.
int rowsFor(int inputW, int inputH);

/// Lays a part of a picture into an input.
///  - keepShape true: the part keeps its shape, as large as the input allows,
///    in the middle of it (black bars at two sides).
///  - keepShape false: the part fills the input, stretched as needed.
/// An input size that the network cannot take is brought to the nearest that it can.
Fit fitArea(int areaW, int areaH, int inputW, int inputH, bool keepShape);

/// Reads the network's answer: every row whose best class is a person or a
/// vehicle and at least scoreMin sure, as a box in INPUT points.
/// False when the answer does not have the size that belongs to the input.
bool decode(const float *rows, int rowCount, int rowLength, int inputW, int inputH, double scoreMin,
            std::vector<Found> &out);

/// Brings boxes from input points to picture points, and cuts them at the part's border.
void toPicture(std::vector<Found> &found, const Fit &fit, const Area &area);

/// How much two boxes share: the shared area over the area of both together, 0 to 1.
double shared(const Found &a, const Found &b);

/// Of boxes that are one object the surest is kept. The surest come first in the result.
std::vector<Found> pick(std::vector<Found> found, std::size_t most = 100);

/// Joins what was found in the whole picture with what was found in a part of
/// it, looked at more closely. A box cut by the part's border is left out when
/// the whole picture has the object, and of two boxes for one object the
/// part's is kept (it was seen larger). A border of the part that is also a
/// border of the picture cuts nothing.
std::vector<Found> merge(const std::vector<Found> &whole, const std::vector<Found> &part, const Area &partArea,
                         const Area &picture);

/// What is asked of the finder for one picture.
struct Ask
{
    Area area;               // the part to look at. Width 0: all of the picture
    int inputW = 640;        // the network's input for it
    int inputH = 384;
    bool keepShape = true;
    double scoreMin = 0.40;  // boxes the network is less sure of are left out
};

// The network finds an object that is some 30 input points large or more. A
// person 50 m away is 30 points high in the camera's whole picture (1280 x 720,
// its widest lens), so the picture must not be made smaller for them. Looking
// at all of it at its own size takes four times as long as at half its size.
// So one picture gets two looks:
//  - the whole picture, at about half its size: what is large, anywhere,
//  - its middle, at its own size: what is small, where the operator has
//    pointed the camera. As much of the middle as the device has time for,
//    and all of the picture on a fast one.

/// The whole picture is made no larger than this for its look.
constexpr int kWholeInputW = 640;
constexpr int kWholeInputH = 384;
/// The look at the middle should take no longer than this, in milliseconds. (It is the
/// longer of the two looks, so about this long passes from a picture to its marks.)
constexpr double kMiddleBudgetMs = 400.0;
/// The sizes the middle can have, in picture points, the smallest first.
constexpr int kMiddleSizes[4][2] = {{640, 384}, {832, 480}, {960, 544}, {1280, 736}};
/// A whole picture that goes into its look at this scale or larger needs no second look.
constexpr double kOwnSizeScale = 0.9;

/// What the finder looks at in one picture.
struct Plan
{
    Ask whole;            // all of the video picture, made smaller
    bool closer = false;  // also the middle, at the picture's own size
    Ask middle;
};

/// The looks for a video picture (given as a part of the image it comes in), on
/// a device that needs unitMs milliseconds for an input of 640 x 384 points
/// (0: not known yet, the middle is then its smallest size).
Plan planLooks(const Area &picture, double unitMs);

class ObjectFinder
{
public:
    ObjectFinder();
    ~ObjectFinder();
    ObjectFinder(const ObjectFinder &) = delete;
    ObjectFinder &operator=(const ObjectFinder &) = delete;

    /// True when this build holds the network's library.
    static bool inThisBuild();
    /// The library and its version, for the notices. Empty when it is not in this build.
    static std::string libraryName();

    /// Takes the network: its layer list (text) and its weights. Both are copied.
    /// False when they do not fit together, with the reason in lastError().
    bool load(const std::string &layers, const std::uint8_t *weights, std::size_t weightsSize);
    bool ready() const;
    const std::string &lastError() const { return _error; }

    /// The people and vehicles in a part of a picture, the surest first, in
    /// picture points. Empty when nothing is found, and when the finder is not
    /// ready or the picture cannot be read: whyNot, when given, then says why
    /// (and is empty when all went well).
    /// Two threads may call this at the same time, each with its own picture
    /// (the app looks at the whole picture and at its middle side by side).
    /// Not while load() runs.
    std::vector<Found> find(const Picture &picture, const Ask &ask, std::string *whyNot = nullptr) const;

private:
    struct Net;
    std::unique_ptr<Net> _net;
    std::string _error;  // of load() only
};

} // namespace skydroid::find
