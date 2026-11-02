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
    check(kPort == 8100, "the camera port of the SDK's demo");

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
        }
        check(toHex(got) == v.hex, std::string("as VGCS: ") + v.name);
    }

    for (const auto &v : kC20Reads) {
        const std::vector<Bytes> packets = splitPackets(fromHex(v.read));
        const std::vector<std::string> wantPackets = splitText(v.packets, '|');
        const std::vector<std::string> wantZooms = splitText(v.zooms, ',');
        bool same = packets.size() == wantPackets.size() && packets.size() == wantZooms.size();
        for (std::size_t i = 0; same && i < packets.size(); ++i) {
            const std::optional<int> zoom = zoomOfReply(packets[i]);
            same = toHex(packets[i]) == wantPackets[i] && std::to_string(zoom.value_or(0)) == wantZooms[i];
        }
        check(same, std::string("as VGCS reads it: ") + v.name);
    }

    std::printf("%d checks, %d failed\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
