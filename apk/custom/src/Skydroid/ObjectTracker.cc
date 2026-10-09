#include "ObjectTracker.h"

#include <algorithm>
#include <cmath>

namespace skydroid::track {

namespace {

constexpr int N = ObjectTracker::kSize;
constexpr int kCells = N * N;
constexpr double kPi = 3.14159265358979323846;
constexpr double kTargetSigma = 2.0;   // width of the wanted peak, in patch points
constexpr float kLearnRate = 0.125f;   // how much of the filter a sharp match replaces
constexpr float kLearnWeak = 0.25f;    // of that, for a match that is not sharp: the object turns, or something covers it
constexpr float kColourRate = 0.06f;   // and how much of what is known of the colours
constexpr float kColourCatchUp = 0.25f;  // the same when the layout is sure and the colours are not: the light changed
// Of the mean power: keeps empty frequencies from blowing up. Its size made no
// difference on the bench (from 1e-6 to 1e-2), so no test pins it.
constexpr float kRegularise = 1e-2f;
constexpr int kPeakGuard = 5;          // the peak's own neighbourhood (11 x 11) is not "the rest"
constexpr int kMaxSubSamples = 4;      // per axis, when the patch is shrunk from a large box
// The box is the middle half of the patch. A little inside it is surely the
// object, a little outside it surely the ground: what lies between is left out
// when the colours are learned.
constexpr int kBoxFrom = N / 4;
constexpr int kBoxTo = 3 * N / 4;
constexpr int kInsideMargin = 4;
constexpr int kAroundMargin = 3;
constexpr int kSteps = 16;             // of each of the three colours in the colour table
constexpr float kColourGain = 2.0f;    // how much the colour picture counts beside the brightness in the filter
constexpr float kPriorWeight = 0.006f; // a colour seen about four times in the box is believed half
constexpr float kPrior = 0.4f;         // a colour never seen: rather the ground than the object
constexpr double kTellsFill = 0.15;    // the colours tell when at least this much of the box has colours of its own
constexpr double kTellsAround = 0.5;   // and what is around it has under this share of as much
constexpr int kShiftSteps = 8;         // how often the window is moved to the middle of the object's colours
constexpr double kShiftLeast = 0.25;   // of the object's colours as the box held them: with less in sight it is not there
constexpr double kCoastKeep = 0.9;     // of its speed, each picture the object is not seen
constexpr double kWeakReach = 0.2;     // of the box: a match that is not sharp is believed only this near where the object should be

/// One colour of a point at a place between points, with the edge value outside the picture.
inline float sampleBilinear(const Picture &p, int channel, double x, double y)
{
    x = std::clamp(x, 0.0, static_cast<double>(p.width - 1));
    y = std::clamp(y, 0.0, static_cast<double>(p.height - 1));
    const int x0 = static_cast<int>(x);
    const int y0 = static_cast<int>(y);
    const int x1 = std::min(x0 + 1, p.width - 1);
    const int y1 = std::min(y0 + 1, p.height - 1);
    const float fx = static_cast<float>(x - x0);
    const float fy = static_cast<float>(y - y0);
    const std::uint8_t *row0 = p.data + static_cast<std::ptrdiff_t>(y0) * p.stride + channel;
    const std::uint8_t *row1 = p.data + static_cast<std::ptrdiff_t>(y1) * p.stride + channel;
    const int a = x0 * p.channels;
    const int b = x1 * p.channels;
    const float top = row0[a] + (row0[b] - row0[a]) * fx;
    const float bottom = row1[a] + (row1[b] - row1[a]) * fx;
    return top + (bottom - top) * fy;
}

/// One row or column of 64 values, in place.
void fft1d(std::complex<float> *data, int step, bool inverse)
{
    // Bit reversal for 64 values.
    for (int i = 0, j = 0; i < N; ++i) {
        if (i < j) {
            std::swap(data[i * step], data[j * step]);
        }
        int bit = N >> 1;
        while (bit != 0 && (j & bit) != 0) {
            j ^= bit;
            bit >>= 1;
        }
        j |= bit;
    }
    for (int len = 2; len <= N; len <<= 1) {
        const double angle = (inverse ? 2.0 : -2.0) * kPi / len;
        const std::complex<float> root(static_cast<float>(std::cos(angle)), static_cast<float>(std::sin(angle)));
        for (int start = 0; start < N; start += len) {
            std::complex<float> w(1.0f, 0.0f);
            for (int k = 0; k < len / 2; ++k) {
                std::complex<float> &a = data[(start + k) * step];
                std::complex<float> &b = data[(start + k + len / 2) * step];
                const std::complex<float> t = b * w;
                b = a - t;
                a = a + t;
                w *= root;
            }
        }
    }
}

/// Spreads each colour of the table over its neighbours. A shirt in one colour fills
/// one place of the table, and a little more or less light moves it to another place:
/// without this it would then count as a colour never seen.
///  - a quarter to each side along each of the three colours,
///  - and further along "all three brighter or darker", which is what light and shade do.
void spreadColours(float *table, float *scratch)
{
    constexpr int kTable = kSteps * kSteps * kSteps;
    const int strides[3] = {1, kSteps, kSteps * kSteps};
    for (int stride : strides) {
        for (int i = 0; i < kTable; ++i) {
            const int at = (i / stride) % kSteps;
            const float before = at > 0 ? table[i - stride] : table[i];
            const float after = at < kSteps - 1 ? table[i + stride] : table[i];
            scratch[i] = 0.5f * table[i] + 0.25f * (before + after);
        }
        std::copy(scratch, scratch + kTable, table);
    }
    const int diagonal = 1 + kSteps + kSteps * kSteps;
    for (int i = 0; i < kTable; ++i) {
        const int c0 = i % kSteps;
        const int c1 = (i / kSteps) % kSteps;
        const int c2 = i / (kSteps * kSteps);
        const int low = std::min({c0, c1, c2});
        const int high = std::max({c0, c1, c2});
        float sum = 0.4f * table[i];
        for (int d = 1; d <= 2; ++d) {
            const float share = d == 1 ? 0.2f : 0.1f;
            sum += share * (low >= d ? table[i - d * diagonal] : table[i]);
            sum += share * (high < kSteps - d ? table[i + d * diagonal] : table[i]);
        }
        scratch[i] = sum;
    }
    std::copy(scratch, scratch + kTable, table);
}

inline bool insideBox(int x, int y)
{
    return x >= kBoxFrom + kInsideMargin && x < kBoxTo - kInsideMargin && y >= kBoxFrom + kInsideMargin &&
           y < kBoxTo - kInsideMargin;
}

inline bool aroundBox(int x, int y)
{
    return x < kBoxFrom - kAroundMargin || x >= kBoxTo + kAroundMargin || y < kBoxFrom - kAroundMargin ||
           y >= kBoxTo + kAroundMargin;
}

} // namespace

ObjectTracker::ObjectTracker()
    : _window(kCells)
    , _target(kCells)
    , _layout(kCells)
    , _colour(kCells)
    , _power(kCells, 0.0f)
    , _inBox(kBins, 0.0f)
    , _around(kBins, 0.0f)
    , _belongs(kBins, kPrior)
    , _own(kBins, 0.0f)
    , _scratch(kCells)
    , _counts(3 * kBins)
{
    for (int y = 0; y < N; ++y) {
        const double wy = 0.5 * (1.0 - std::cos(2.0 * kPi * y / (N - 1)));
        for (int x = 0; x < N; ++x) {
            const double wx = 0.5 * (1.0 - std::cos(2.0 * kPi * x / (N - 1)));
            _window[y * N + x] = static_cast<float>(wx * wy);
            const double dx = x - N / 2;
            const double dy = y - N / 2;
            _target[y * N + x] = Complex(
                static_cast<float>(std::exp(-(dx * dx + dy * dy) / (2.0 * kTargetSigma * kTargetSigma))), 0.0f);
        }
    }
    _fft(_target, false);
}

void ObjectTracker::_fft(Field &field, bool inverse)
{
    for (int y = 0; y < N; ++y) {
        fft1d(field.data() + y * N, 1, inverse);
    }
    for (int x = 0; x < N; ++x) {
        fft1d(field.data() + x, N, inverse);
    }
    if (inverse) {
        const float scale = 1.0f / kCells;
        for (Complex &value : field) {
            value *= scale;
        }
    }
}

bool ObjectTracker::_sample(const Picture &picture, double cx, double cy, Look &look) const
{
    look.levels.resize(kCells);
    look.bins.resize(kCells);
    const double patchW = _box.w * kPadding;
    const double patchH = _box.h * kPadding;
    const double stepX = patchW / N;
    const double stepY = patchH / N;
    // A large box is shrunk into the patch: average the points each patch point covers.
    const int subX = std::clamp(static_cast<int>(std::ceil(stepX)), 1, kMaxSubSamples);
    const int subY = std::clamp(static_cast<int>(std::ceil(stepY)), 1, kMaxSubSamples);
    const float perSample = 1.0f / (subX * subY);
    const double left = cx - patchW / 2.0 - 0.5;
    const double top = cy - patchH / 2.0 - 0.5;
    const bool inColour = picture.channels >= 3;

    double rawSum = 0.0;
    double rawSquares = 0.0;
    double boxSum = 0.0;
    double boxSquares = 0.0;
    double sum = 0.0;
    double squares = 0.0;
    for (int j = 0; j < N; ++j) {
        for (int i = 0; i < N; ++i) {
            float c0 = 0.0f;
            float c1 = 0.0f;
            float c2 = 0.0f;
            for (int sy = 0; sy < subY; ++sy) {
                const double y = top + (j + (sy + 0.5) / subY) * stepY;
                for (int sx = 0; sx < subX; ++sx) {
                    const double x = left + (i + (sx + 0.5) / subX) * stepX;
                    c0 += sampleBilinear(picture, 0, x, y);
                    if (inColour) {
                        c1 += sampleBilinear(picture, 1, x, y);
                        c2 += sampleBilinear(picture, 2, x, y);
                    }
                }
            }
            c0 *= perSample;
            if (inColour) {
                c1 *= perSample;
                c2 *= perSample;
            } else {
                c1 = c0;
                c2 = c0;
            }
            // The middle colour counts double: right for red, green, blue in either order.
            const float raw = 0.25f * (c0 + 2.0f * c1 + c2);
            rawSum += raw;
            rawSquares += static_cast<double>(raw) * raw;
            if (i >= kBoxFrom && i < kBoxTo && j >= kBoxFrom && j < kBoxTo) {
                boxSum += raw;
                boxSquares += static_cast<double>(raw) * raw;
            }
            // The logarithm makes a shadow and a sunlit side count alike.
            const float value = std::log1p(raw);
            sum += value;
            squares += static_cast<double>(value) * value;
            look.levels[j * N + i] = value;
            const int b0 = std::clamp(static_cast<int>(c0), 0, 255) * kSteps / 256;
            const int b1 = std::clamp(static_cast<int>(c1), 0, 255) * kSteps / 256;
            const int b2 = std::clamp(static_cast<int>(c2), 0, 255) * kSteps / 256;
            look.bins[j * N + i] = static_cast<std::uint16_t>(b0 + kSteps * (b1 + kSteps * b2));
        }
    }
    const double rawMean = rawSum / kCells;
    const double rawSpread = std::sqrt(std::max(0.0, rawSquares / kCells - rawMean * rawMean));
    const int boxCells = (kBoxTo - kBoxFrom) * (kBoxTo - kBoxFrom);
    const double boxMean = boxSum / boxCells;
    const double boxSpread = std::sqrt(std::max(0.0, boxSquares / boxCells - boxMean * boxMean));
    if (rawSpread < kMinContrast || boxSpread < 0.5 * kMinContrast) {
        return false;  // a plain area: nothing to hold on to
    }
    const double mean = sum / kCells;
    const double spread = std::sqrt(std::max(1e-12, squares / kCells - mean * mean));
    for (int k = 0; k < kCells; ++k) {
        look.levels[k] = static_cast<float>((look.levels[k] - mean) / spread);
    }
    return true;
}

void ObjectTracker::_learnColours(const Look &look, float rate)
{
    float *inBox = _counts.data();
    float *around = _counts.data() + kBins;
    std::fill(_counts.begin(), _counts.end(), 0.0f);
    float inBoxTotal = 0.0f;
    float aroundTotal = 0.0f;
    for (int y = 0; y < N; ++y) {
        for (int x = 0; x < N; ++x) {
            const int bin = look.bins[y * N + x];
            if (insideBox(x, y)) {
                inBox[bin] += 1.0f;
                inBoxTotal += 1.0f;
            } else if (aroundBox(x, y)) {
                around[bin] += 1.0f;
                aroundTotal += 1.0f;
            }
        }
    }
    spreadColours(inBox, _counts.data() + 2 * kBins);
    spreadColours(around, _counts.data() + 2 * kBins);
    const float keep = 1.0f - rate;
    for (int b = 0; b < kBins; ++b) {
        _inBox[b] = keep * _inBox[b] + rate * inBox[b] / inBoxTotal;
        _around[b] = keep * _around[b] + rate * around[b] / aroundTotal;
        _belongs[b] = (_inBox[b] + kPriorWeight * kPrior) / (_inBox[b] + _around[b] + kPriorWeight);
        _own[b] = std::max(0.0f, (_inBox[b] - _around[b]) / (_inBox[b] + _around[b] + kPriorWeight));
    }
}

void ObjectTracker::_fill(const Look &look, double &inside, double &around) const
{
    double inSum = 0.0;
    double outSum = 0.0;
    int inCount = 0;
    int outCount = 0;
    for (int y = 0; y < N; ++y) {
        for (int x = 0; x < N; ++x) {
            const float own = _own[look.bins[y * N + x]];
            if (insideBox(x, y)) {
                inSum += own;
                ++inCount;
            } else if (aroundBox(x, y)) {
                outSum += own;
                ++outCount;
            }
        }
    }
    inside = inSum / inCount;
    around = outSum / outCount;
}

bool ObjectTracker::_coloursTell() const
{
    return _fillInside >= kTellsFill && _fillAround <= kTellsAround * _fillInside;
}

double ObjectTracker::colourTells() const
{
    return _fillInside >= kTellsFill ? std::clamp(1.0 - _fillAround / _fillInside, 0.0, 1.0) : 0.0;
}

double ObjectTracker::_alike(const Look &look) const
{
    if (!_coloursTell()) {
        return 1.0;  // the colours do not tell the object from the ground: they are not asked
    }
    double inside = 0.0;
    double around = 0.0;
    _fill(look, inside, around);
    return std::clamp(inside / _fillInside, 0.0, 1.5);
}

bool ObjectTracker::_shiftToColours(const Picture &picture, double cx, double cy, double &outX, double &outY) const
{
    const double stepX = _box.w * kPadding / N;
    const double stepY = _box.h * kPadding / N;
    const double half = (kBoxTo - kBoxFrom) / 2.0;
    const double least = kShiftLeast * _fillInside * (kBoxTo - kBoxFrom) * (kBoxTo - kBoxFrom);
    Look look;
    if (!_sample(picture, cx, cy, look)) {
        return false;
    }
    // A window of the box's size, in patch points. It starts in the middle.
    double wx = N / 2.0;
    double wy = N / 2.0;
    bool any = false;
    for (int step = 0; step < kShiftSteps; ++step) {
        double sum = 0.0;
        double sumX = 0.0;
        double sumY = 0.0;
        const int x0 = std::max(0, static_cast<int>(std::ceil(wx - half - 0.5)));
        const int x1 = std::min(N - 1, static_cast<int>(std::floor(wx + half - 0.5)));
        const int y0 = std::max(0, static_cast<int>(std::ceil(wy - half - 0.5)));
        const int y1 = std::min(N - 1, static_cast<int>(std::floor(wy + half - 0.5)));
        for (int j = y0; j <= y1; ++j) {
            for (int i = x0; i <= x1; ++i) {
                const double own = _own[look.bins[j * N + i]];
                sum += own;
                sumX += own * (i + 0.5);
                sumY += own * (j + 0.5);
            }
        }
        if (sum < least) {
            return false;  // too little of the object's colours here
        }
        const double nx = sumX / sum;
        const double ny = sumY / sum;
        const double moved = std::hypot(nx - wx, ny - wy);
        wx = nx;
        wy = ny;
        any = true;
        if (moved < 0.25) {
            break;
        }
    }
    outX = std::clamp(cx + (wx - N / 2.0) * stepX, 0.0, picture.width - 1.0);
    outY = std::clamp(cy + (wy - N / 2.0) * stepY, 0.0, picture.height - 1.0);
    return any;
}

void ObjectTracker::_features(const Look &look, Field &layout, Field &colour) const
{
    double sum = 0.0;
    for (int k = 0; k < kCells; ++k) {
        sum += _belongs[look.bins[k]];
    }
    const float mean = static_cast<float>(sum / kCells);
    for (int k = 0; k < kCells; ++k) {
        const float belongs = _belongs[look.bins[k]];
        layout[k] = Complex(look.levels[k] * belongs * _window[k], 0.0f);
        colour[k] = Complex((belongs - mean) * kColourGain * _window[k], 0.0f);
    }
    _fft(layout, false);
    _fft(colour, false);
}

ObjectTracker::Answer ObjectTracker::_respond(const Field &layout, const Field &colour) const
{
    float meanPower = 0.0f;
    for (float value : _power) {
        meanPower += value;
    }
    const float small = std::max(1e-6f, kRegularise * meanPower / kCells);
    for (int k = 0; k < kCells; ++k) {
        _scratch[k] = (_layout[k] * layout[k] + _colour[k] * colour[k]) / (_power[k] + small);
    }
    _fft(_scratch, true);

    int peakIndex = 0;
    float peak = _scratch[0].real();
    for (int k = 1; k < kCells; ++k) {
        if (_scratch[k].real() > peak) {
            peak = _scratch[k].real();
            peakIndex = k;
        }
    }
    const int peakX = peakIndex % N;
    const int peakY = peakIndex / N;

    // How far the peak stands above the rest of the answer, in spreads of the rest.
    double sum = 0.0;
    double squares = 0.0;
    int count = 0;
    for (int y = 0; y < N; ++y) {
        const bool nearY = std::abs(y - peakY) <= kPeakGuard;
        for (int x = 0; x < N; ++x) {
            if (nearY && std::abs(x - peakX) <= kPeakGuard) {
                continue;
            }
            const double value = _scratch[y * N + x].real();
            sum += value;
            squares += value * value;
            ++count;
        }
    }
    Answer answer;
    if (count > 0) {
        const double mean = sum / count;
        const double spread = std::sqrt(std::max(0.0, squares / count - mean * mean));
        answer.psr = (peak - mean) / (spread + 1e-9);
    }

    // Between points: the top of a parabola through the peak and its neighbours.
    auto refine = [](float before, float at, float after) {
        const float bend = before - 2.0f * at + after;
        if (bend >= -1e-12f) {
            return 0.0;
        }
        return std::clamp(0.5 * (before - after) / bend, -0.5, 0.5);
    };
    double subX = 0.0;
    double subY = 0.0;
    if (peakX > 0 && peakX < N - 1) {
        subX = refine(_scratch[peakIndex - 1].real(), peak, _scratch[peakIndex + 1].real());
    }
    if (peakY > 0 && peakY < N - 1) {
        subY = refine(_scratch[peakIndex - N].real(), peak, _scratch[peakIndex + N].real());
    }
    answer.dx = peakX - N / 2 + subX;
    answer.dy = peakY - N / 2 + subY;
    return answer;
}

void ObjectTracker::_learn(const Field &layout, const Field &colour, float rate)
{
    const float keep = 1.0f - rate;
    for (int k = 0; k < kCells; ++k) {
        _layout[k] = keep * _layout[k] + rate * _target[k] * std::conj(layout[k]);
        _colour[k] = keep * _colour[k] + rate * _target[k] * std::conj(colour[k]);
        _power[k] = keep * _power[k] + rate * (std::norm(layout[k]) + std::norm(colour[k]));
    }
}

bool ObjectTracker::start(const std::uint8_t *gray, int width, int height, int stride, const Box &box)
{
    return start(Picture{gray, width, height, stride, 1}, box);
}

Result ObjectTracker::update(const std::uint8_t *gray, int width, int height, int stride)
{
    return update(Picture{gray, width, height, stride, 1});
}

bool ObjectTracker::start(const Picture &picture, const Box &box)
{
    stop();
    const int width = picture.width;
    const int height = picture.height;
    if (picture.data == nullptr || width < 2 * static_cast<int>(kMinBox) || height < 2 * static_cast<int>(kMinBox) ||
        (picture.channels != 1 && picture.channels != 3 && picture.channels != 4) ||
        picture.stride < width * picture.channels) {
        return false;
    }
    if (!std::isfinite(box.cx) || !std::isfinite(box.cy) || !std::isfinite(box.w) || !std::isfinite(box.h) ||
        box.w <= 0.0 || box.h <= 0.0) {
        return false;
    }
    if (box.cx < 0.0 || box.cy < 0.0 || box.cx > width - 1 || box.cy > height - 1) {
        return false;
    }
    _box.cx = box.cx;
    _box.cy = box.cy;
    _box.w = std::clamp(box.w, kMinBox, 0.9 * width);
    _box.h = std::clamp(box.h, kMinBox, 0.9 * height);

    Look look;
    if (!_sample(picture, _box.cx, _box.cy, look)) {
        return false;  // the box holds a plain area
    }
    // The colours first: the filter looks through them.
    std::fill(_inBox.begin(), _inBox.end(), 0.0f);
    std::fill(_around.begin(), _around.end(), 0.0f);
    _learnColours(look, 1.0f);
    _fill(look, _fillInside, _fillAround);

    std::fill(_layout.begin(), _layout.end(), Complex(0.0f, 0.0f));
    std::fill(_colour.begin(), _colour.end(), Complex(0.0f, 0.0f));
    std::fill(_power.begin(), _power.end(), 0.0f);
    Field layout(kCells);
    Field colour(kCells);
    _features(look, layout, colour);
    _learn(layout, colour, 1.0f);
    _vx = 0.0;
    _vy = 0.0;
    _seenX = _box.cx;
    _seenY = _box.cy;
    _missed = 0;
    _active = true;
    return true;
}

Result ObjectTracker::update(const Picture &picture)
{
    Result result;
    result.box = _box;
    const int width = picture.width;
    const int height = picture.height;
    if (!_active || picture.data == nullptr || width < 2 || height < 2 ||
        (picture.channels != 1 && picture.channels != 3 && picture.channels != 4) ||
        picture.stride < width * picture.channels) {
        return result;
    }

    Answer best;
    best.psr = -1.0;
    double bestCx = _box.cx;
    double bestCy = _box.cy;
    Look look;
    Field layout(kCells);
    Field colour(kCells);
    auto tryAt = [&](double cx, double cy) {
        if (!_sample(picture, cx, cy, look)) {
            return;
        }
        _features(look, layout, colour);
        const Answer answer = _respond(layout, colour);
        if (answer.psr > best.psr) {
            best = answer;
            bestCx = cx;
            bestCy = cy;
        }
    };

    // First where it should be if it keeps moving as it did.
    const double guessX = std::clamp(_box.cx + _vx, 0.0, width - 1.0);
    const double guessY = std::clamp(_box.cy + _vy, 0.0, height - 1.0);
    tryAt(guessX, guessY);
    if (best.psr < kPsrLearn) {
        // Not sure there: look around, half a box to each side, and where it was last seen.
        const double stepX = _box.w * kPadding / 4.0;
        const double stepY = _box.h * kPadding / 4.0;
        for (int oy = -1; oy <= 1; ++oy) {
            for (int ox = -1; ox <= 1; ++ox) {
                if (ox != 0 || oy != 0) {
                    tryAt(guessX + ox * stepX, guessY + oy * stepY);
                }
            }
        }
        if (std::abs(guessX - _seenX) + std::abs(guessY - _seenY) > 0.5) {
            tryAt(_seenX, _seenY);
        }
    }
    result.psr = std::max(0.0, best.psr);

    // Where the layout says the object is, and whether the colours there are the object's.
    const bool tells = _coloursTell();
    const bool sharp = best.psr >= kPsrLearn;
    // A match that is not sharp is followed, but little is learned from it. An object that
    // turns keeps its new look, so the match gets sharp again; a wall that covers it does
    // not look like it for long enough to be learned.
    const float care = sharp ? 1.0f : kLearnWeak;
    bool found = false;
    float filterRate = 0.0f;
    float colourRate = 0.0f;
    double x = guessX;
    double y = guessY;
    bool candidate = best.psr >= kPsrFound;
    if (candidate) {
        x = std::clamp(bestCx + best.dx * (_box.w * kPadding / N), 0.0, width - 1.0);
        y = std::clamp(bestCy + best.dy * (_box.h * kPadding / N), 0.0, height - 1.0);
        // Over ground that is not the object, the highest of the filter's answers lies
        // anywhere. So a match that is not sharp counts only where the object should be.
        if (!sharp && (std::abs(x - guessX) > kWeakReach * _box.w || std::abs(y - guessY) > kWeakReach * _box.h)) {
            candidate = false;
            x = guessX;
            y = guessY;
        }
    }
    if (candidate) {
        if (_sample(picture, x, y, look)) {
            result.alike = _alike(look);
            if (!tells) {
                // The colours cannot be asked: the layout alone decides, and it keeps learning.
                found = true;
                filterRate = care * kLearnRate;
                colourRate = care * kColourRate;
            } else if (best.psr >= kPsrSure) {
                // The layout is sure by itself. If the colours are not as they were, the light on
                // the object changed: they are learned anew, fast.
                found = true;
                filterRate = kLearnRate;
                colourRate = result.alike >= kAlikeLearn ? kColourRate : kColourCatchUp;
            } else if (result.alike >= kAlikeFound) {
                // Both say yes, neither loudly.
                found = true;
                filterRate = care * kLearnRate;
                colourRate = care * kColourRate;
            }
        }
    }
    if (tells && !found) {
        // The layout is not sure: the object turned, or it is not there. Ask the colours, near where it should be.
        double shiftedX = 0.0;
        double shiftedY = 0.0;
        Look shifted;
        if (_shiftToColours(picture, x, y, shiftedX, shiftedY) && _sample(picture, shiftedX, shiftedY, shifted)) {
            const double alike = _alike(shifted);
            if (alike >= kAlikeLearn) {
                found = true;  // the object's colours are there: it changed its look
                x = shiftedX;
                y = shiftedY;
                look = shifted;
                result.alike = alike;
                filterRate = kLearnRate;
                colourRate = kColourRate;
            }
        }
    }
    if (found) {
        _vx = 0.5 * _vx + 0.5 * (x - _box.cx);
        _vy = 0.5 * _vy + 0.5 * (y - _box.cy);
        _box.cx = x;
        _box.cy = y;
        _seenX = x;
        _seenY = y;
        _missed = 0;
        if (colourRate > 0.0f) {
            _learnColours(look, colourRate);
            double inside = 0.0;
            double around = 0.0;
            _fill(look, inside, around);
            _fillInside = (1.0 - colourRate) * _fillInside + colourRate * inside;
            _fillAround = (1.0 - colourRate) * _fillAround + colourRate * around;
        }
        if (filterRate > 0.0f) {
            _features(look, layout, colour);
            _learn(layout, colour, filterRate);
        }
    } else {
        // Not seen. Nothing is learned, and the box goes on as the object was moving,
        // slower each picture: a walker who passes behind a post comes out on its far side.
        ++_missed;
        _box.cx = std::clamp(_box.cx + _vx, 0.0, width - 1.0);
        _box.cy = std::clamp(_box.cy + _vy, 0.0, height - 1.0);
        _vx *= kCoastKeep;
        _vy *= kCoastKeep;
    }
    result.found = found;
    result.box = _box;
    return result;
}

void ObjectTracker::stop()
{
    _active = false;
    _missed = 0;
    _vx = 0.0;
    _vy = 0.0;
}

} // namespace skydroid::track
