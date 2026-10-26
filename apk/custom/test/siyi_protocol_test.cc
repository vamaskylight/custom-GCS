// Host unit test for SiyiProtocol: every byte must match vgcs/siyi/protocol.py.
//
// The expected values come from siyi_protocol_vectors.inc, which
// gen_siyi_vectors.py writes by running the Python code.
// Run with apk/custom/test/run_tests.ps1.

#include "SiyiProtocol.h"

#include <cmath>
#include <cstdio>
#include <string>

using namespace siyi;

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

std::string frameText(const std::optional<Frame> &f)
{
    if (!f) {
        return "none";
    }
    return "ctrl " + std::to_string(f->ctrl) + " seq " + std::to_string(f->seq) + " cmd " + std::to_string(f->cmdId)
        + " data " + toHex(f->data);
}

} // namespace

#define CHECK_U16(expr, want) \
    check(#expr, (expr) == (want), std::to_string(expr), std::to_string(want))

#define CHECK_HEX(expr, want) \
    do { \
        const std::string got_ = toHex(expr); \
        check(#expr, got_ == (want), got_, want); \
    } while (0)

#define CHECK_PARSE(hex, has, wantCtrl, wantSeq, wantCmd, dataHex) \
    do { \
        const std::optional<Frame> f_ = parseFrame(fromHex(hex)); \
        const bool ok_ = f_.has_value() == (has) \
            && (!(has) \
                || (f_->ctrl == (wantCtrl) && f_->seq == (wantSeq) && f_->cmdId == (wantCmd) \
                    && toHex(f_->data) == (dataHex))); \
        check("parseFrame(" hex ")", ok_, frameText(f_), \
              (has) ? std::string("ctrl ") + std::to_string(wantCtrl) + " seq " + std::to_string(wantSeq) + " cmd " \
                          + std::to_string(wantCmd) + " data " + (dataHex) \
                    : std::string("none")); \
    } while (0)

#define CHECK_ATTITUDE(hex, has, yaw, pitch, roll) \
    do { \
        const std::optional<Attitude> a_ = decodeAttitude(fromHex(hex)); \
        const bool ok_ = a_.has_value() == (has) \
            && (!(has) || (near(a_->yawDeg, yaw) && near(a_->pitchDeg, pitch) && near(a_->rollDeg, roll))); \
        check("decodeAttitude(" hex ")", ok_, \
              a_ ? std::to_string(a_->yawDeg) + " " + std::to_string(a_->pitchDeg) + " " + std::to_string(a_->rollDeg) \
                 : std::string("none"), \
              (has) ? std::to_string(yaw) + " " + std::to_string(pitch) + " " + std::to_string(roll) \
                    : std::string("none")); \
    } while (0)

int main()
{
#include "siyi_protocol_vectors.inc"

    // A request the camera would answer: its own parser reads back what we built.
    const Bytes request = buildRequest(kCmdGimbalAngle, encodeAngle(12.3, -4.5), 42);
    const std::optional<Frame> back = parseFrame(request);
    check("parseFrame(buildRequest(...))",
          back && back->ctrl == kCtrlNeedAck && back->seq == 42 && back->cmdId == kCmdGimbalAngle
              && back->data == encodeAngle(12.3, -4.5),
          frameText(back), "the same request");

    std::printf("%d checks, %d failed\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
