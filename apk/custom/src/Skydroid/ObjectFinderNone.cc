// The object finder of a build without the ncnn library: it finds nothing, and
// says so. apk/custom/CMakeLists.txt takes this file in place of
// ObjectFinderNcnn.cc when the build was asked for without the finder.

#include "ObjectFinder.h"

namespace skydroid::find {

struct ObjectFinder::Net
{
};

ObjectFinder::ObjectFinder() = default;
ObjectFinder::~ObjectFinder() = default;

bool ObjectFinder::inThisBuild()
{
    return false;
}

std::string ObjectFinder::libraryName()
{
    return std::string();
}

bool ObjectFinder::ready() const
{
    return false;
}

bool ObjectFinder::load(const std::string &, const std::uint8_t *, std::size_t)
{
    _error = "this build has no object finding in it";
    return false;
}

std::vector<Found> ObjectFinder::find(const Picture &, const Ask &, std::string *whyNot) const
{
    if (whyNot) {
        *whyNot = "this build has no object finding in it";
    }
    return {};
}

} // namespace skydroid::find
