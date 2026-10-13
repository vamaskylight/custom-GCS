// Uses the object finder as the app does. run_android_link_check.ps1 builds this
// into a shared library for each Android ABI: when that links with nothing
// left undefined, the finder and the ncnn library fit the app's compiler, its
// C++ library and its Android level. (It is not run: a PC cannot run it.)

#include "ObjectFinder.h"

using namespace skydroid::find;

extern "C" int vama_finder_check(const char *layers, const unsigned char *weights, unsigned long weightsSize,
                                 const unsigned char *rgb, int width, int height)
{
    ObjectFinder finder;
    if (!ObjectFinder::inThisBuild() || !finder.load(layers, weights, weightsSize)) {
        return -1;
    }
    Picture picture;
    picture.data = rgb;
    picture.width = width;
    picture.height = height;
    picture.stride = width * 3;
    picture.channels = 3;
    const std::vector<Found> whole = finder.find(picture, Ask{});
    Ask close;
    close.area = Area{width / 4, height / 4, width / 2, height / 2};
    const std::vector<Found> part = finder.find(picture, close);
    return static_cast<int>(merge(whole, part, close.area, Area{0, 0, width, height}).size());
}
