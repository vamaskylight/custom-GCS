// Host unit test for SiyiCamera: the app's SIYI camera logic must send what VGCS
// sends (vgcs/siyi/adapter.py, vgcs/video/camera_control.py).
//
// siyi_camera_vectors.inc holds action scripts and the frames VGCS sent for them
// (gen_siyi_camera_vectors.py runs the Python code with a fake clock). The checks
// below the scripts cover the app's own zoom click and what the app reads from the
// camera. Run with apk/custom/test/run_tests.ps1.

#include "SiyiCamera.h"

#include <cmath>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

using namespace siyi;

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
    auto add = [&out](long long t, const std::vector<Bytes> &frames) {
        for (const Bytes &f : frames) {
            out.emplace_back(t, toHex(f));
        }
    };
    for (size_t i = 0; i < count; ++i) {
        const Step &s = steps[i];
        add(s.t, cam.tick(s.t));
        const std::string action = s.action;
        if (action == "speed") {
            add(s.t, cam.gimbalSpeed(s.a, s.b));
        } else if (action == "ptz") {
            add(s.t, cam.ptz(s.text));
        } else if (action == "center") {
            add(s.t, cam.center());
        } else if (action == "point_down") {
            add(s.t, cam.pointDown());
        } else if (action == "angle") {
            add(s.t, cam.setAngle(s.a, s.b));
        } else if (action == "focus") {
            add(s.t, cam.focusStep(static_cast<int>(s.a), s.t));
        } else if (action == "autofocus") {
            add(s.t, cam.autoFocus());
        } else if (action == "photo") {
            add(s.t, cam.photo());
        } else if (action == "record_toggle") {
            add(s.t, cam.toggleRecording());
        } else if (action == "poll") {
            add(s.t, {cam.poll()});
            if (s.text[0] != '\0') {
                cam.feed(fromHex(s.text), s.t);
            }
        } else if (action != "wait") {
            check(std::string("known action ") + action, false);
        }
    }
    add(finalMs, cam.tick(finalMs));
    return out;
}

void checkScript(const char *name, const Step *steps, size_t stepCount, const Sent *sent, size_t sentCount,
                 long long finalMs, bool hasYaw, double yaw)
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
    const bool sameYaw = cam.attitude().has_value() == hasYaw && (!hasYaw || std::fabs(cam.attitude()->yawDeg - yaw) < 1e-9);
    check(std::string("script ") + name + ": yaw", sameYaw,
          cam.attitude() ? std::to_string(cam.attitude()->yawDeg) : "none", hasYaw ? std::to_string(yaw) : "none");
}

#define CHECK_SCRIPT(name, hasYaw, yaw) \
    checkScript(#name, kSteps_##name, sizeof(kSteps_##name) / sizeof(Step), kSent_##name, \
                sizeof(kSent_##name) / sizeof(Sent), kFinalMs, hasYaw, yaw)

/// A datagram from the camera: the command, its data (an ACK).
Bytes answer(std::uint8_t cmd, const Bytes &data)
{
    Bytes out{0x55, 0x66, kCtrlIsAck, static_cast<std::uint8_t>(data.size() & 0xFF),
              static_cast<std::uint8_t>(data.size() >> 8), 0x00, 0x00, cmd};
    out.insert(out.end(), data.begin(), data.end());
    const std::uint16_t crc = crc16(out);
    out.push_back(static_cast<std::uint8_t>(crc & 0xFF));
    out.push_back(static_cast<std::uint8_t>(crc >> 8));
    return out;
}

Bytes attitudeData(int yawTenths, int pitchTenths, int rollTenths)
{
    auto le = [](int v) {
        const unsigned u = static_cast<unsigned>(v) & 0xFFFF;
        return std::pair<std::uint8_t, std::uint8_t>(static_cast<std::uint8_t>(u & 0xFF), static_cast<std::uint8_t>(u >> 8));
    };
    Bytes out;
    for (int v : {yawTenths, pitchTenths, rollTenths}) {
        const auto b = le(v);
        out.push_back(b.first);
        out.push_back(b.second);
    }
    out.insert(out.end(), 6, 0);
    return out;
}

std::uint16_t seqOf(const Bytes &frame)
{
    return static_cast<std::uint16_t>(frame[5] | (frame[6] << 8));
}

void appChecks()
{
    // A zoom click: the start now, the stop kZoomPulseMs later.
    {
        Camera cam;
        const std::vector<Bytes> start = cam.zoomStep(1, 1000);
        check("zoom in starts", start.size() == 1 && start[0] == buildRequest(kCmdManualZoom, encodeManualZoom(1), 0),
              start.empty() ? "" : toHex(start[0]));
        check("no stop before its time", cam.tick(1149).empty());
        const std::vector<Bytes> stop = cam.tick(1150);
        check("the stop at its time", stop.size() == 1 && stop[0] == buildRequest(kCmdManualZoom, encodeManualZoom(0), 1),
              stop.empty() ? "" : toHex(stop[0]));
        check("only once", cam.tick(5000).empty());
    }
    // A second click replaces the first one's stop; 0 stops now and takes the pending stop back.
    {
        Camera cam;
        cam.zoomStep(1, 1000);
        cam.zoomStep(-1, 1100);
        check("the first click's stop is gone", cam.tick(1200).empty());
        check("the second click's stop", cam.tick(1250).size() == 1);
        cam.zoomStep(-1, 2000);
        const std::vector<Bytes> now = cam.zoomStep(0, 2050);
        check("zoom 0 stops now", now.size() == 1 && toHex(now[0]).substr(16, 2) == "00", now.empty() ? "" : toHex(now[0]));
        check("and nothing comes later", cam.tick(9000).empty());
    }
    // The attitude comes from attitude frames only (the app gets every datagram).
    {
        Camera cam;
        check("an attitude frame", cam.feed(answer(kCmdGimbalAttitude, attitudeData(375, -120, 15)), 1000)
                                       && cam.attitude() && std::fabs(cam.attitude()->yawDeg - 37.5) < 1e-9
                                       && std::fabs(cam.attitude()->pitchDeg + 12.0) < 1e-9 && cam.attitudeMs() == 1000);
        check("the answer of another command is not the attitude",
              !cam.feed(answer(kCmdGimbalAngle, attitudeData(990, 990, 0)), 1100) && std::fabs(cam.attitude()->yawDeg - 37.5) < 1e-9);
        check("an attitude frame too short", !cam.feed(answer(kCmdGimbalAttitude, Bytes{1, 2, 3, 4, 5}), 1200)
                                                 && cam.attitudeMs() == 1000);
        Bytes broken = answer(kCmdGimbalAttitude, attitudeData(1, 2, 3));
        broken.back() ^= 0xFF;
        check("a wrong CRC", !cam.feed(broken, 1300) && cam.attitudeMs() == 1000);
        const std::vector<Bytes> down = cam.pointDown(-80.0);
        check("point down keeps the reported yaw", down.size() == 1 && down[0] == buildRequest(kCmdGimbalAngle, encodeAngle(37.5, -80.0), 0),
              down.empty() ? "" : toHex(down[0]));
    }
    // The sequence numbers count every request and wrap after 65535.
    {
        Camera cam;
        Bytes last;
        for (int i = 0; i <= 65536; ++i) {
            last = cam.poll();
        }
        check("the sequence wraps", seqOf(last) == 0, std::to_string(seqOf(last)), "0");
        check("and goes on", seqOf(cam.photo()[0]) == 1);
    }
}

} // namespace

int main()
{
#include "siyi_camera_vectors.inc"

    appChecks();

    std::printf("%d checks, %d failed\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
