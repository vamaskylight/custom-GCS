// Host unit test for SkydroidTop: every frame must match vgcs/skydroid/protocol.py.
//
// The expected values come from skydroid_top_vectors.inc, which
// gen_skydroid_vectors.py writes by running the Python code.
// Run with apk/custom/test/run_tests.ps1.

#include "SkydroidTop.h"

#include <cmath>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

using namespace skydroid::top;

namespace {

int g_checks = 0;
int g_failures = 0;

void fail(const char *expr, const std::string &got, const std::string &want)
{
    ++g_failures;
    std::printf("FAIL %s\n  got:  \"%s\"\n  want: \"%s\"\n", expr, got.c_str(), want.c_str());
}

std::string join(const std::vector<std::string> &frames)
{
    std::string out;
    for (size_t i = 0; i < frames.size(); ++i) {
        if (i) {
            out += "|";
        }
        out += frames[i];
    }
    return out;
}

template <typename T>
void checkDecode(const char *expr, const std::optional<T> &got, bool hasValue, double value)
{
    ++g_checks;
    if (got.has_value() != hasValue || (hasValue && std::fabs(static_cast<double>(*got) - value) > 1e-6)) {
        fail(expr, got ? std::to_string(static_cast<double>(*got)) : std::string("none"),
             hasValue ? std::to_string(value) : std::string("none"));
    }
}

template <typename T>
bool sameOptional(const std::optional<T> &got, bool has, double value)
{
    if (got.has_value() != has) {
        return false;
    }
    return !has || std::fabs(static_cast<double>(*got) - value) < 1e-6;
}

} // namespace

#define CHECK_FRAME(expr, want)                                     \
    do {                                                            \
        ++g_checks;                                                 \
        const std::string got_ = (expr);                            \
        if (got_ != (want)) {                                       \
            fail(#expr, got_, (want));                              \
        }                                                           \
    } while (0)

#define CHECK_FRAMES(expr, want)                                    \
    do {                                                            \
        ++g_checks;                                                 \
        const std::string got_ = join(expr);                        \
        if (got_ != (want)) {                                       \
            fail(#expr, got_, (want));                              \
        }                                                           \
    } while (0)

#define CHECK_DECODE(expr, has, value) checkDecode(#expr, (expr), (has), (value))

#define CHECK_SLR_MAX(metres, want)                                 \
    do {                                                            \
        ++g_checks;                                                 \
        const int got_ = slrMaxDmForRange(metres);                  \
        if (got_ != (want)) {                                       \
            fail("slrMaxDmForRange(" #metres ")", std::to_string(got_), std::to_string(want)); \
        }                                                           \
    } while (0)

#define CHECK_REPLY(w_raw, w_parsed, w_tag, w_ctrl, w_hy, w_yaw, w_hp, w_pitch, w_hr, w_roll, w_hs, w_slr, w_hd, w_dzm) \
    do {                                                            \
        ++g_checks;                                                 \
        const auto f_ = parseTpFrame(w_raw);                        \
        if (f_.has_value() != (w_parsed)) {                         \
            fail("parseTpFrame(" #w_raw ") parsed", f_ ? "yes" : "no", (w_parsed) ? "yes" : "no"); \
        } else if (f_) {                                            \
            const bool ok_ = f_->tag == (w_tag) && f_->ctrl == (w_ctrl) \
                && sameOptional(f_->yaw, (w_hy), (w_yaw))           \
                && sameOptional(f_->pitch, (w_hp), (w_pitch))       \
                && sameOptional(f_->roll, (w_hr), (w_roll))         \
                && sameOptional(f_->slrDm, (w_hs), (w_slr))         \
                && sameOptional(f_->dzmStep, (w_hd), (w_dzm));      \
            if (!ok_) {                                             \
                fail("parseTpFrame(" #w_raw ") fields", f_->tag + " " + f_->data, (w_tag)); \
            }                                                       \
        }                                                           \
    } while (0)

int main()
{
#include "skydroid_top_vectors.inc"

    // Things the Python vectors do not cover.
    {
        Options c14;
        c14.gClassUpperHeader = true;
        // Variable-length gimbal frames switch to "#TP"; fixed ones are "#TP" anyway.
        const std::string gsm = buildGimbalSpeed(10.0, -10.0, c14);
        ++g_checks;
        if (gsm.rfind("#TPUG4wGSM", 0) != 0) {
            fail("buildGimbalSpeed(c14 upper header)", gsm, "#TPUG4wGSM...");
        }
        // Non-gimbal variable frames keep "#tp".
        const std::string zoom = buildDzmAbsoluteZoomUd(2.0, 0, c14);
        ++g_checks;
        if (zoom.rfind("#tpUD", 0) != 0) {
            fail("buildDzmAbsoluteZoomUd(c14)", zoom, "#tpUD...");
        }
        // A 1200 m laser camera accepts 1100 m; the default does not.
        Options laser1200;
        laser1200.slrMaxDm = slrMaxDmForRange(1200.0);
        CHECK_DECODE(decodeSlrDecimeters("2AF8", laser1200), true, 11000.0);
        CHECK_DECODE(decodeSlrDecimeters("2AF8"), false, 0.0);
    }

    std::printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
