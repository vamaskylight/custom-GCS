// Host unit test for C20Camera: the bytes of Skydroid's own SDK (rcsdk 1.9.2, C20Camera, read with
// javap, 2026-11-03), and every byte against vgcs/skydroid/c20_camera.py (c20_camera_vectors.inc,
// written by gen_c20_vectors.py).
// Run with apk/custom/test/run_tests.ps1.

#include "C20Camera.h"

#include <cstdio>
#include <string>

using namespace c20;

namespace {

int g_checks = 0;
int g_failures = 0;

void check(bool ok, const std::string &what)
{
    ++g_checks;
    if (!ok) {
        ++g_failures;
        std::printf("FAIL %s\n", what.c_str());
    }
}

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
    for (std::size_t i = 0; i + 1 < hex.size(); i += 2) {
        out.push_back(static_cast<std::uint8_t>(std::stoul(hex.substr(i, 2), nullptr, 16)));
    }
    return out;
}

std::vector<std::string> splitText(const std::string &text, char by)
{
    std::vector<std::string> out;
    if (text.empty()) {
        return out;
    }
    std::size_t start = 0;
    while (true) {
        const std::size_t at = text.find(by, start);
        out.push_back(text.substr(start, at == std::string::npos ? std::string::npos : at - start));
        if (at == std::string::npos) {
            return out;
        }
        start = at + 1;
    }
}

#include "c20_camera_vectors.inc"

} // namespace

int main()
{
    // Skydroid's SDK, as its bytecode builds them (and the client's PDF of 2026-11-02 says).
    check(toHex(zoomIn()) == "0000000000002231000" "30B0001", "the SDK's zoom in");
    check(toHex(zoomOut()) == "0000000000002231000" "30C0001", "the SDK's zoom out");
    check(toHex(zoomStop()) == "0000000000002231000" "30B0101", "the SDK's stop");
    check(toHex(zoomQuery()) == "000000000000224A00020000", "the SDK's zoom read");
    // Its getCameraState (row B29): group 0x33, command 0x55, no payload; asked every 15 s.
    check(toHex(stateQuery()) == "00000000000033550000" && kStateAskMs == 15000, "the SDK's state question");
    check(kPort == 8100, "the camera port of the SDK's demo");
    // Its takePicture, startRecordVideo, stopRecordVideo (row B26): 32 bytes, all zero but two.
    const std::string zeros(60, '0');
    check(toHex(photo()) == "00000000000033810020" "0A0C" + zeros, "the SDK's photo");
    check(toHex(recordStart()) == "00000000000033810020" "0A0B" + zeros, "the SDK's record start");
    check(toHex(recordStop()) == "00000000000033810020" "0A0A" + zeros, "the SDK's record stop");
    // C20Gimbal.toggleLED (row B27): text, on the gimbal connection.
    check(torch(true) == "AT+LED -t1\r\n" && torch(false) == "AT+LED -t0\r\n", "the SDK's torch text");

    for (const auto &v : kC20Packets) {
        Bytes got;
        const std::string name = v.name;
        if (name == "zoomIn") {
            got = zoomIn();
        } else if (name == "zoomOut") {
            got = zoomOut();
        } else if (name == "zoomStop") {
            got = zoomStop();
        } else if (name == "zoomQuery") {
            got = zoomQuery();
        } else if (name == "stateQuery") {
            got = stateQuery();
        } else if (name == "photo") {
            got = photo();
        } else if (name == "recordStart") {
            got = recordStart();
        } else if (name == "recordStop") {
            got = recordStop();
        } else if (name == "torchOn" || name == "torchOff") {
            const std::string text = torch(name == "torchOn");
            got = Bytes(text.begin(), text.end());
        }
        check(toHex(got) == v.hex, std::string("as VGCS: ") + v.name);
    }

    for (const auto &v : kC20Reads) {
        const std::vector<Bytes> packets = splitPackets(fromHex(v.read));
        const std::vector<std::string> wantPackets = splitText(v.packets, '|');
        const std::vector<std::string> wantZooms = splitText(v.zooms, ',');
        const std::vector<std::string> wantCaptures = splitText(v.captures, ',');
        bool same = packets.size() == wantPackets.size() && packets.size() == wantZooms.size() &&
                    packets.size() == wantCaptures.size();
        for (std::size_t i = 0; same && i < packets.size(); ++i) {
            const std::optional<int> zoom = zoomOfReply(packets[i]);
            const std::optional<bool> done = captureReplyOk(packets[i]);
            const std::string capture = done ? (*done ? "1" : "0") : "-";
            same = toHex(packets[i]) == wantPackets[i] && std::to_string(zoom.value_or(0)) == wantZooms[i] &&
                   capture == wantCaptures[i];
        }
        check(same, std::string("as VGCS reads it: ") + v.name);
    }

    std::printf("%d checks, %d failed\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
