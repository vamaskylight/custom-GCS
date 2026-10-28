// Host unit test for ViewproCamera: the app's Viewpro camera logic must send what
// VGCS sends (vgcs/viewpro/adapter.py, vgcs/video/camera_control.py).
//
// viewpro_camera_vectors.inc holds action scripts and the frames VGCS sent for
// them (gen_viewpro_camera_vectors.py runs the Python code with a fake clock).
// The checks below the scripts cover what VGCS reads in one piece and the app
// reads from a TCP stream. Run with apk/custom/test/run_tests.ps1.

#include "ViewproCamera.h"

#include <cmath>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

using namespace viewpro;

namespace {

int g_checks = 0;
int g_failures = 0;

struct Step
{
    long long t;
    const char *action;
    double a;
    double b;
    const char *text;
};

struct Sent
{
    long long t;
    const char *hex;
};

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

void check(const std::string &what, bool ok, const std::string &got = "", const std::string &want = "")
{
    ++g_checks;
    if (!ok) {
        ++g_failures;
        std::printf("FAIL %s\n  got:  %s\n  want: %s\n", what.c_str(), got.c_str(), want.c_str());
    }
}

using Frames = std::vector<std::pair<long long, std::string>>;

std::string text(const Frames &frames)
{
    std::string out;
    for (const auto &f : frames) {
        out += "\n    " + std::to_string(f.first) + " " + f.second;
    }
    return out.empty() ? std::string("(nothing)") : out;
}

Frames play(Camera &cam, const Step *steps, size_t count, long long finalMs)
{
    Frames out;
    auto add = [&out](long long t, const Bytes &frame) {
        if (!frame.empty()) {
            out.emplace_back(t, toHex(frame));
        }
    };
    for (size_t i = 0; i < count; ++i) {
        const Step &s = steps[i];
        for (const Bytes &due : cam.tick(s.t)) {
            add(s.t, due);
        }
        const std::string action = s.action;
        if (action == "speed") {
            add(s.t, cam.gimbalSpeed(s.a, s.b, s.t));
        } else if (action == "ptz") {
            add(s.t, cam.ptz(s.text, s.t));
        } else if (action == "center") {
            add(s.t, cam.center());
        } else if (action == "look_down") {
            add(s.t, cam.lookDown());
        } else if (action == "angle") {
            add(s.t, cam.setAngle(s.a, s.b));
        } else if (action == "zoom") {
            add(s.t, cam.zoomStep(static_cast<int>(s.a), s.t));
        } else if (action == "focus") {
            add(s.t, cam.focusStep(static_cast<int>(s.a)));
        } else if (action == "autofocus") {
            add(s.t, cam.autoFocus());
        } else if (action == "photo") {
            add(s.t, cam.photo());
        } else if (action == "record") {
            add(s.t, cam.setRecording(s.a != 0.0));
        } else if (action == "record_toggle") {
            add(s.t, cam.toggleRecording());
        } else if (action == "sensor") {
            add(s.t, cam.setSensor(s.a != 0.0));
        } else if (action == "laser") {
            add(s.t, cam.setLaser(s.a != 0.0));
        } else if (action == "status") {
            cam.feed(fromHex(s.text), s.t);
        } else if (action != "wait") {
            check(std::string("known action ") + action, false);
        }
    }
    for (const Bytes &due : cam.tick(finalMs)) {
        add(finalMs, due);
    }
    return out;
}

void checkScript(const char *name, const Step *steps, size_t stepCount, const Sent *sent, size_t sentCount,
                 long long finalMs, bool recording, bool ir)
{
    Camera cam;
    const Frames got = play(cam, steps, stepCount, finalMs);
    Frames want;
    for (size_t i = 0; i < sentCount; ++i) {
        if (sent[i].t >= 0) {
            want.emplace_back(sent[i].t, sent[i].hex);
        }
    }
    check(std::string("script ") + name, got == want, text(got), text(want));
    check(std::string("script ") + name + ": recording", cam.recording() == recording,
          cam.recording() ? "true" : "false", recording ? "true" : "false");
    check(std::string("script ") + name + ": sensor", cam.sensorIsIr() == ir, cam.sensorIsIr() ? "ir" : "eo",
          ir ? "ir" : "eo");
}

#define CHECK_SCRIPT(name, recording, ir) \
    checkScript(#name, kSteps_##name, sizeof(kSteps_##name) / sizeof(Step), kSent_##name, \
                sizeof(kSent_##name) / sizeof(Sent), kFinalMs, recording, ir)

/// A status frame in its TCP envelope: servo mode, recording, laser range (dm), zoom (tenths), FOV (hundredths).
Bytes statusFrame(int servoMode, int record, unsigned rangeDm, unsigned zoomTenths, unsigned vfov = 4310,
                  unsigned hfov = 7020)
{
    Bytes body(22, 0);
    body.push_back(0);  // F1
    const Bytes b1{static_cast<std::uint8_t>(((servoMode & 0x0F) << 4) | 0x07), 0xFF, 0x1B, 0xF0, 0xE3, 0x8C};
    body.insert(body.end(), b1.begin(), b1.end());
    const Bytes d1{0x00,
                   static_cast<std::uint8_t>((rangeDm >> 16) & 0xFF),
                   0x00,
                   static_cast<std::uint8_t>(record & 0x03),
                   static_cast<std::uint8_t>((rangeDm >> 8) & 0xFF),
                   static_cast<std::uint8_t>(rangeDm & 0xFF),
                   static_cast<std::uint8_t>(vfov >> 8),
                   static_cast<std::uint8_t>(vfov & 0xFF),
                   static_cast<std::uint8_t>(hfov >> 8),
                   static_cast<std::uint8_t>(hfov & 0xFF),
                   static_cast<std::uint8_t>(zoomTenths >> 8),
                   static_cast<std::uint8_t>(zoomTenths & 0xFF)};
    body.insert(body.end(), d1.begin(), d1.end());
    return wrapTcp(buildSerialFrame(kCmdStatus, body));
}

bool near(double a, double b)
{
    return std::fabs(a - b) < 1e-9;
}

void streamChecks()
{
    // The status request is a heartbeat.
    check("poll is a heartbeat", Camera().poll() == encodeHeartbeat());

    // A status in three pieces: read when its last piece comes.
    {
        Camera cam;
        const Bytes frame = statusFrame(0x0D, 1, 3375, 25);
        const Bytes p1(frame.begin(), frame.begin() + 5);
        const Bytes p2(frame.begin() + 5, frame.begin() + 30);
        const Bytes p3(frame.begin() + 30, frame.end());
        check("first piece: no status yet", !cam.feed(p1, 1000) && !cam.status());
        check("second piece: no status yet", !cam.feed(p2, 1010) && !cam.status());
        check("last piece: the status", cam.feed(p3, 1020) && cam.status() && cam.statusMs() == 1020);
        check("recording from the status", cam.recording());
        check("laser range", cam.rangeM() && near(*cam.rangeM(), 337.5) && cam.rangeMs() == 1020,
              cam.rangeM() ? std::to_string(*cam.rangeM()) : "none", "337.5");
        check("zoom and field of view", cam.zoomX() && near(*cam.zoomX(), 2.5) && cam.hfovDeg()
                                            && near(*cam.hfovDeg(), 70.2) && cam.vfovDeg() && near(*cam.vfovDeg(), 43.1));
    }

    // The header cut after its first byte, noise before it, two frames in one read.
    {
        Camera cam;
        const Bytes first = statusFrame(0x0D, 0, 0, 10);
        Bytes noise{0x00, 0x13, 0x37, 0xEB, 0x00, 0x55, 0x00};
        noise.push_back(first[0]);
        check("noise and a cut header: nothing yet", !cam.feed(noise, 1000));
        const Bytes rest(first.begin() + 1, first.end());
        check("the rest of the frame", cam.feed(rest, 1100) && cam.statusMs() == 1100);
        Bytes two = statusFrame(0x0D, 1, 0, 30);
        const Bytes second = statusFrame(0x0D, 0, 1000, 40);
        two.insert(two.end(), second.begin(), second.end());
        check("two frames in one read: the later one counts", cam.feed(two, 1200) && !cam.recording()
                                                                   && cam.zoomX() && near(*cam.zoomX(), 4.0));
        cam.feed(statusFrame(0x0D, 0, 0, 40), 1300);
        check("a range of 0 keeps the one measured before",
              cam.rangeM() && near(*cam.rangeM(), 100.0) && cam.rangeMs() == 1200);
    }

    // A bad envelope is skipped, the good one after it counts; a bare serial frame counts too.
    {
        Camera cam;
        Bytes bad = statusFrame(0x0D, 1, 0, 10);
        bad.back() ^= 0x01;
        Bytes both = bad;
        const Bytes good = statusFrame(0x0D, 0, 0, 20);
        both.insert(both.end(), good.begin(), good.end());
        check("bad then good envelope", cam.feed(both, 1000) && cam.zoomX() && near(*cam.zoomX(), 2.0));
        Camera bare;
        const Bytes inner = *unwrapTcp(statusFrame(0x0D, 1, 0, 50));
        check("a bare serial status frame", bare.feed(inner, 1000) && bare.recording());
        Camera cut;
        const Bytes head(inner.begin(), inner.begin() + 2);
        const Bytes tail(inner.begin() + 2, inner.end());
        check("a bare frame cut inside its header: nothing yet", !cut.feed(head, 1000));
        check("and whole with the rest", cut.feed(tail, 1100) && cut.recording());
    }

    // A field of view or zoom of 0 is "not reported": the last real values stay.
    {
        Camera cam;
        cam.feed(statusFrame(0x0D, 0, 0, 25, 4310, 7020), 1000);
        cam.feed(statusFrame(0x0D, 0, 0, 0, 0, 0), 1100);
        check("0 keeps the live optics", cam.zoomX() && near(*cam.zoomX(), 2.5) && cam.hfovDeg()
                                             && near(*cam.hfovDeg(), 70.2));
    }

    // Long noise is dropped, and a frame after it is still read.
    {
        Camera cam;
        Bytes noise(5000, 0x11);
        check("noise: no status", !cam.feed(noise, 1000));
        check("a frame after the noise", cam.feed(statusFrame(0x0D, 1, 0, 10), 1100) && cam.recording());
    }

    // Another frame (a heartbeat echo) is not a status.
    {
        Camera cam;
        check("a heartbeat is not a status", !cam.feed(encodeHeartbeat(), 1000) && !cam.status());
    }
}

} // namespace

int main()
{
#include "viewpro_camera_vectors.inc"

    streamChecks();

    std::printf("%d checks, %d failed\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
