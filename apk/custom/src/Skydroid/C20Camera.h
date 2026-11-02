#pragma once

// The Skydroid C20's camera port: its zoom (row B10, 2026-11-03).
//
// Skydroid's RC SDK (rcsdk 1.9.2 and 1.6.5, com.skydroid.rcsdk.common.payload.C20Camera, read with
// javap) opens the C20's camera on TCP port 8100 of the camera's address, and its gimbal on 5000
// (TOP). The camera's commands are binary packets: 6 zero bytes, a group byte, a command byte, the
// payload's length (2 bytes, high byte first), then the payload, written to the socket as they are.
// So the TOP lens frames on 5000 never zoomed it (the client, 2026-10-27 and 2026-10-30). The zoom
// moves while it is told to: start in or out, then stop. Its zoom can be read back: the reply's
// payload byte 9 runs from 1 to 66, three values for each 1x (the SDK's getZoom), so 1x to 22x.
// None of this is tried on a C20.
//
// A port of vgcs/skydroid/c20_camera.py, checked against it (gen_c20_vectors.py). Plain C++17 with no
// Qt, unit tested by apk/custom/test/c20_camera_test.cc.

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace c20 {

using Bytes = std::vector<std::uint8_t>;

constexpr int kPort = 8100;
constexpr std::size_t kHeaderLen = 10;
constexpr std::uint8_t kGroupZoom = 0x22;
constexpr std::uint8_t kCmdZoomMove = 0x31;
constexpr std::uint8_t kCmdZoomRead = 0x4A;
constexpr int kZoomMax = 22;
constexpr std::size_t kZoomReadMinPayload = 16;  // the SDK takes a zoom reply only with this much

/// 6 zero bytes, group, command, the payload's length (high byte first), the payload.
Bytes packet(std::uint8_t group, std::uint8_t command, const Bytes &payload = {});

Bytes zoomIn();
Bytes zoomOut();
/// The SDK's stopZoomOrFucus: it stops a zoom and a focus move.
Bytes zoomStop();
Bytes zoomQuery();

/// The packets in what one read brought, cut by their length. The SDK takes each read as one
/// packet and never looks at the length: so a packet whose length runs past the read, or a zoom
/// reply that says less than its 16 bytes, takes the rest of the read. Less than a header at the
/// end is dropped.
std::vector<Bytes> splitPackets(const Bytes &read);

/// The zoom (1 to 22) a zoom reply says, or nothing (another packet, too short, or a value the
/// SDK has no zoom for).
std::optional<int> zoomOfReply(const Bytes &packet);

} // namespace c20
