// Host unit test for ViewproProtocol: every byte must match vgcs/viewpro/protocol.py.
//
// The expected values come from viewpro_protocol_vectors.inc, which
// gen_viewpro_vectors.py writes by running the Python code.
// Run with apk/custom/test/run_tests.ps1.

#include "ViewproProtocol.h"

#include <cmath>
#include <cstdio>
#include <string>

using namespace viewpro;

namespace {

int g_checks = 0;
int g_failures = 0;

std::string toHex(const Bytes &bytes)
{
    static const char *digits = "0123456789ABCDEF";
    std::string out;
    for (std::uint8_t b : bytes) {
        out += digits[b >> 4];
        out += digits[b & 0x0F];
    }
    return out;
}

Bytes fromHex(const std::string &hex)
{
    Bytes out;
    for (size_t i = 0; i + 1 < hex.size(); i += 2) {
        out.push_back(static_cast<std::uint8_t>(std::stoi(hex.substr(i, 2), nullptr, 16)));
    }
    return out;
}

void check(const char *expr, bool ok, const std::string &got, const std::string &want)
{
    ++g_checks;
    if (!ok) {
        ++g_failures;
        std::printf("FAIL %s\n  got:  %s\n  want: %s\n", expr, got.c_str(), want.c_str());
    }
}

bool near(double a, double b)
{
    return std::fabs(a - b) < 1e-9;
}

std::string num(double v)
{
    char buf[64];
    std::snprintf(buf, sizeof buf, "%.12g", v);
    return buf;
}

std::string attitudeText(const Attitude &a)
{
    return num(a.yawDeg) + " " + num(a.pitchDeg) + " " + num(a.rollDeg) + " " + std::to_string(a.servoStatus);
}

std::string cameraText(const std::optional<CameraStatus> &c)
{
    if (!c) {
        return "no camera";
    }
    return "rec " + std::to_string(c->recordStatus) + " range " + (c->rangeM ? num(*c->rangeM) : std::string("none"))
        + " fov " + num(c->vfovDeg) + "/" + num(c->hfovDeg) + " zoom " + num(c->zoomX);
}

bool sameCamera(const std::optional<CameraStatus> &c, bool has, int rec, bool hasRange, double range, double vfov,
                double hfov, double zoom)
{
    if (c.has_value() != has) {
        return false;
    }
    return !has
        || (c->recordStatus == rec && c->rangeM.has_value() == hasRange && (!hasRange || near(*c->rangeM, range))
            && near(c->vfovDeg, vfov) && near(c->hfovDeg, hfov) && near(c->zoomX, zoom));
}

} // namespace

#define CHECK_HEX(expr, want) \
    do { \
        const std::string got_ = toHex(expr); \
        check(#expr, got_ == (want), got_, want); \
    } while (0)

#define CHECK_COMMAND(sets, want) \
    do { \
        GimbalCameraCommand c; \
        sets \
        const std::string got_ = toHex(encodeGimbalCameraCommand(c)); \
        check("encodeGimbalCameraCommand(" #sets ")", got_ == (want), got_, want); \
    } while (0)

#define CHECK_INT(expr, want) \
    check(#expr, (expr) == (want), std::to_string(expr), std::to_string(want))

#define CHECK_REAL(expr, want) \
    check(#expr, near((expr), (want)), num(expr), num(want))

#define CHECK_PARSE_SERIAL(hex, has, wantCmd, dataHex) \
    do { \
        const auto p_ = parseSerialFrame(fromHex(hex)); \
        const bool ok_ = p_.has_value() == (has) && (!(has) || (p_->first == (wantCmd) && toHex(p_->second) == (dataHex))); \
        check("parseSerialFrame(" hex ")", ok_, \
              p_ ? std::to_string(p_->first) + " " + toHex(p_->second) : std::string("none"), \
              (has) ? std::to_string(wantCmd) + " " + (dataHex) : std::string("none")); \
    } while (0)

#define CHECK_UNWRAP(hex, has, innerHex) \
    do { \
        const auto u_ = unwrapTcp(fromHex(hex)); \
        const bool ok_ = u_.has_value() == (has) && (!(has) || toHex(*u_) == (innerHex)); \
        check("unwrapTcp(" hex ")", ok_, u_ ? toHex(*u_) : std::string("none"), \
              (has) ? std::string(innerHex) : std::string("none")); \
    } while (0)

#define CHECK_B1(hex, has, yaw, pitch, roll, servo) \
    do { \
        const auto b_ = decodeB1(fromHex(hex)); \
        const bool ok_ = b_.has_value() == (has) \
            && (!(has) || (near(b_->yawDeg, yaw) && near(b_->pitchDeg, pitch) && near(b_->rollDeg, roll) \
                           && b_->servoStatus == (servo))); \
        check("decodeB1(" hex ")", ok_, b_ ? attitudeText(*b_) : std::string("none"), \
              (has) ? num(yaw) + " " + num(pitch) + " " + num(roll) + " " + std::to_string(servo) : std::string("none")); \
    } while (0)

#define CHECK_D1(hex, has, rec, hasRange, range, vfov, hfov, zoom) \
    do { \
        const auto d_ = decodeD1(fromHex(hex)); \
        check("decodeD1(" hex ")", sameCamera(d_, has, rec, hasRange, range, vfov, hfov, zoom), cameraText(d_), \
              (has) ? "rec " + std::to_string(rec) + " range " + ((hasRange) ? num(range) : std::string("none")) \
                          + " fov " + num(vfov) + "/" + num(hfov) + " zoom " + num(zoom) \
                    : std::string("no camera")); \
    } while (0)

#define CHECK_STATUS(hex, has, yaw, pitch, roll, servo, tSource, tStatus, tType, hasCam, rec, hasRange, range, vfov, \
                     hfov, zoom) \
    do { \
        const auto s_ = findStatusFrame(fromHex(hex)); \
        const bool ok_ = s_.has_value() == (has) \
            && (!(has) \
                || (near(s_->attitude.yawDeg, yaw) && near(s_->attitude.pitchDeg, pitch) \
                    && near(s_->attitude.rollDeg, roll) && s_->attitude.servoStatus == (servo) \
                    && s_->tracking.source == (tSource) && s_->tracking.status == (tStatus) \
                    && s_->tracking.targetType == (tType) \
                    && sameCamera(s_->camera, hasCam, rec, hasRange, range, vfov, hfov, zoom))); \
        check("findStatusFrame(" hex ")", ok_, \
              s_ ? attitudeText(s_->attitude) + " track " + std::to_string(s_->tracking.source) + "/" \
                       + std::to_string(s_->tracking.status) + "/" + std::to_string(s_->tracking.targetType) + " " \
                       + cameraText(s_->camera) \
                 : std::string("none"), \
              (has) ? std::string("see the vectors") : std::string("none")); \
    } while (0)

int main()
{
#include "viewpro_protocol_vectors.inc"

    // The vendor document's worked example: a serial frame read back through both layers.
    const Bytes doc = fromHex("55AADC051C038399");
    const auto inner = unwrapTcp(wrapTcp(doc));
    check("unwrapTcp(wrapTcp(doc))", inner && *inner == doc, inner ? toHex(*inner) : "none", toHex(doc));

    std::printf("%d checks, %d failed\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
