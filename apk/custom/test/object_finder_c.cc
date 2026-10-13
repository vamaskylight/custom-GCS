// ObjectFinder behind plain C functions, built as a DLL, so that the measuring script
// (object_finder_measure.py) asks the very code the app asks.

#include "ObjectFinder.h"

#include <chrono>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

using namespace skydroid::find;

namespace {

struct Session
{
    ObjectFinder finder;
    std::string error;
    double lastMs = 0.0;
};

bool readFile(const char *path, std::string &into)
{
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        return false;
    }
    into.assign(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
    return true;
}

// Seven numbers for each object: kind, label, score, centre x, centre y, width, height.
int write(const std::vector<Found> &found, double *out, int most)
{
    int n = 0;
    for (const Found &f : found) {
        if (n >= most) {
            break;
        }
        double *row = out + 7 * n++;
        row[0] = static_cast<int>(f.kind);
        row[1] = f.label;
        row[2] = f.score;
        row[3] = f.cx;
        row[4] = f.cy;
        row[5] = f.w;
        row[6] = f.h;
    }
    return n;
}

std::vector<Found> read(const double *rows, int count)
{
    std::vector<Found> found;
    for (int i = 0; i < count; ++i) {
        const double *row = rows + 7 * i;
        Found f;
        f.kind = row[0] != 0.0 ? Kind::Vehicle : Kind::Person;
        f.label = static_cast<int>(row[1]);
        f.score = row[2];
        f.cx = row[3];
        f.cy = row[4];
        f.w = row[5];
        f.h = row[6];
        found.push_back(f);
    }
    return found;
}

} // namespace

extern "C" {

__declspec(dllexport) void *of_new()
{
    return new Session();
}

__declspec(dllexport) void of_free(void *session)
{
    delete static_cast<Session *>(session);
}

__declspec(dllexport) int of_in_this_build()
{
    return ObjectFinder::inThisBuild() ? 1 : 0;
}

/// The network from its two files. 1 when it is loaded.
__declspec(dllexport) int of_load(void *session, const char *layersPath, const char *weightsPath)
{
    Session *s = static_cast<Session *>(session);
    std::string layers;
    std::string weights;
    if (!readFile(layersPath, layers) || !readFile(weightsPath, weights)) {
        s->error = "a file of the network cannot be opened";
        return 0;
    }
    const bool ok = s->finder.load(layers, reinterpret_cast<const std::uint8_t *>(weights.data()), weights.size());
    s->error = s->finder.lastError();
    return ok ? 1 : 0;
}

__declspec(dllexport) const char *of_error(void *session)
{
    return static_cast<Session *>(session)->error.c_str();
}

/// The time the last of_find took, in milliseconds.
__declspec(dllexport) double of_last_ms(void *session)
{
    return static_cast<Session *>(session)->lastMs;
}

/// Finds in a part of a picture (width 0: all of it). Returns the number of objects written to out.
__declspec(dllexport) int of_find(void *session, const std::uint8_t *data, int width, int height, int stride,
                                  int channels, int left, int top, int areaW, int areaH, int inputW, int inputH,
                                  int keepShape, double scoreMin, double *out, int most)
{
    Session *s = static_cast<Session *>(session);
    Picture picture;
    picture.data = data;
    picture.width = width;
    picture.height = height;
    picture.stride = stride;
    picture.channels = channels;
    Ask ask;
    ask.area = Area{left, top, areaW, areaH};
    ask.inputW = inputW;
    ask.inputH = inputH;
    ask.keepShape = keepShape != 0;
    ask.scoreMin = scoreMin;
    const auto before = std::chrono::steady_clock::now();
    const std::vector<Found> found = s->finder.find(picture, ask, &s->error);
    s->lastMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - before).count();
    return write(found, out, most);
}

/// merge() of ObjectFinder.h, on rows as of_find writes them.
__declspec(dllexport) int of_merge(const double *whole, int wholeCount, const double *part, int partCount, int left,
                                   int top, int areaW, int areaH, int pictureW, int pictureH, double *out, int most)
{
    const std::vector<Found> joined = merge(read(whole, wholeCount), read(part, partCount),
                                            Area{left, top, areaW, areaH}, Area{0, 0, pictureW, pictureH});
    return write(joined, out, most);
}

} // extern "C"
