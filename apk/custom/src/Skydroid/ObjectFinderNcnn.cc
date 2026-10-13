// The object finder's network, run by the ncnn library. Nothing else in the app
// includes ncnn. A build without the library has ObjectFinderNone.cc in place of this file.

#include "ObjectFinder.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

#include <ncnn/datareader.h>
#include <ncnn/mat.h>
#include <ncnn/net.h>
#include <ncnn/platform.h>

namespace skydroid::find {

namespace {

// The network was taught with pictures prepared like this (nanodet-plus-m_416.yml):
// blue, green, red, each less its mean and times its scale.
const float kMean[3] = {103.53f, 116.28f, 123.675f};
const float kScale[3] = {0.017429f, 0.017507f, 0.017125f};
const char kInputName[] = "data";
const char kOutputName[] = "output";
// The library's name and version, after a word that apk/build_android.ps1 looks for in the
// finished app: it is there only when this file, and so the finder, is in the build.
const char kMarker[] = "VAMA-FINDER ncnn " NCNN_VERSION_STRING;
constexpr int kMarkerWord = 12;  // the length of "VAMA-FINDER "

// Reads the network's layer list from a text that ends, and never past its end.
class TextReader : public ncnn::DataReader
{
public:
    explicit TextReader(const std::string &text) : _text(text) {}

    int scan(const char *format, void *p) const override
    {
        if (_at >= _text.size()) {
            return 0;
        }
        // As ncnn's own reader does: "%n" tells how much of the text one read took.
        std::string withCount(format);
        withCount += "%n";
        int taken = 0;
        const int read = std::sscanf(_text.c_str() + _at, withCount.c_str(), p, &taken);
        if (taken <= 0) {
            return 0;
        }
        _at += static_cast<std::size_t>(taken);
        return read;
    }

    std::size_t read(void *, std::size_t) const override { return 0; }

private:
    const std::string &_text;
    mutable std::size_t _at = 0;
};

// Hands out the weights, and never more than there are. The library copies what it reads.
class BytesReader : public ncnn::DataReader
{
public:
    BytesReader(const std::uint8_t *bytes, std::size_t size) : _at(bytes), _left(size) {}

    int scan(const char *, void *) const override { return 0; }

    std::size_t read(void *buffer, std::size_t size) const override
    {
        if (size > _left) {
            _short = true;
            return 0;
        }
        std::memcpy(buffer, _at, size);
        _at += size;
        _left -= size;
        return size;
    }

    std::size_t left() const { return _left; }
    bool ranShort() const { return _short; }

private:
    mutable const std::uint8_t *_at;
    mutable std::size_t _left;
    mutable bool _short = false;
};

} // namespace

// The library keeps the memory of one run for the next by itself (its "local pool"): the
// runs are all of a few sizes. It is given back when the finder is deleted.
struct ObjectFinder::Net
{
    ncnn::Net net;
};

ObjectFinder::ObjectFinder() = default;
ObjectFinder::~ObjectFinder() = default;

bool ObjectFinder::inThisBuild()
{
    return true;
}

std::string ObjectFinder::libraryName()
{
    return std::string(kMarker + kMarkerWord);
}

bool ObjectFinder::ready() const
{
    return _net != nullptr;
}

bool ObjectFinder::load(const std::string &layers, const std::uint8_t *weights, std::size_t weightsSize)
{
    _net.reset();
    _error.clear();
    if (layers.empty() || !weights || weightsSize == 0) {
        _error = "the network's files are empty";
        return false;
    }
    auto net = std::make_unique<Net>();
    net->net.opt.num_threads = 1;  // the app has the video to show: one core is all the finder takes
    net->net.opt.use_vulkan_compute = false;
    net->net.opt.lightmode = true;

    const TextReader text(layers);
    if (net->net.load_param(text) != 0) {
        _error = "the network's layer list cannot be read";
        return false;
    }
    const BytesReader bytes(weights, weightsSize);
    if (net->net.load_model(bytes) != 0 || bytes.ranShort()) {
        _error = "the network's weights do not fit its layer list";
        return false;
    }
    if (bytes.left() != 0) {
        _error = "the network's weights are longer than its layer list asks for";
        return false;
    }

    // One run on a small black picture: the answer must have the size the decoding expects.
    ncnn::Mat input(kInputMin, kInputMin, 3);
    if (input.empty()) {
        _error = "no memory for the network";
        return false;
    }
    input.fill(0.f);
    input.substract_mean_normalize(kMean, kScale);
    ncnn::Extractor run = net->net.create_extractor();
    ncnn::Mat answer;
    if (run.input(kInputName, input) != 0 || run.extract(kOutputName, answer) != 0) {
        _error = "the network does not run";
        return false;
    }
    if (answer.dims != 2 || answer.w != kRowLength || answer.h != rowsFor(kInputMin, kInputMin)) {
        _error = "the network's answer is not the one this app reads";
        return false;
    }
    _net = std::move(net);
    return true;
}

std::vector<Found> ObjectFinder::find(const Picture &picture, const Ask &ask, std::string *whyNot) const
{
    // Nothing of the finder is written here: two threads may be in this function at once.
    const auto none = [whyNot](const char *why) {
        if (whyNot) {
            *whyNot = why;
        }
        return std::vector<Found>();
    };
    if (whyNot) {
        whyNot->clear();
    }
    if (!_net) {
        return none("the network is not loaded");
    }
    if (!picture.data || picture.width <= 0 || picture.height <= 0 || (picture.channels != 3 && picture.channels != 4) ||
        picture.stride < picture.width * picture.channels) {
        return none("the picture cannot be read");
    }
    // The part to look at, inside the picture.
    Area area = ask.area;
    if (area.width <= 0 || area.height <= 0) {
        area = Area{0, 0, picture.width, picture.height};
    }
    const int left = std::clamp(area.left, 0, picture.width);
    const int top = std::clamp(area.top, 0, picture.height);
    const int right = std::clamp(area.left + area.width, left, picture.width);
    const int bottom = std::clamp(area.top + area.height, top, picture.height);
    area = Area{left, top, right - left, bottom - top};
    if (area.width < kInputMin / 4 || area.height < kInputMin / 4) {
        return none("the picture is too small");
    }

    const Fit fit = fitArea(area.width, area.height, ask.inputW, ask.inputH, ask.keepShape);
    const int order = picture.channels == 4 ? ncnn::Mat::PIXEL_RGBA2BGR : ncnn::Mat::PIXEL_RGB2BGR;
    ncnn::Mat filled = ncnn::Mat::from_pixels_roi_resize(picture.data, order, picture.width, picture.height,
                                                         picture.stride, area.left, area.top, area.width, area.height,
                                                         fit.filledW, fit.filledH);
    if (filled.empty()) {
        return none("no memory for the picture");
    }
    ncnn::Mat input = filled;
    if (fit.filledW != fit.inputW || fit.filledH != fit.inputH) {
        // Black beside the picture, as in the pictures the network was taught with.
        ncnn::Option option;
        option.num_threads = 1;
        ncnn::Mat padded;
        ncnn::copy_make_border(filled, padded, fit.padTop, fit.inputH - fit.filledH - fit.padTop, fit.padLeft,
                               fit.inputW - fit.filledW - fit.padLeft, ncnn::BORDER_CONSTANT, 0.f, option);
        if (padded.empty()) {
            return none("no memory for the picture");
        }
        input = padded;
    }
    input.substract_mean_normalize(kMean, kScale);

    ncnn::Extractor run = _net->net.create_extractor();
    ncnn::Mat answer;
    if (run.input(kInputName, input) != 0 || run.extract(kOutputName, answer) != 0) {
        return none("the network did not run");
    }
    std::vector<Found> found;
    if (answer.dims != 2 || answer.elempack != 1 || answer.elemsize != sizeof(float) ||
        !decode(static_cast<const float *>(answer.data), answer.h, answer.w, fit.inputW, fit.inputH, ask.scoreMin,
                found)) {
        return none("the network's answer is not the one this app reads");
    }
    toPicture(found, fit, area);
    return pick(std::move(found));
}

} // namespace skydroid::find
