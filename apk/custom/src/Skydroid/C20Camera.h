#pragma once

// The Skydroid C20's camera port: its zoom (row B10, 2026-11-03), its photos and recordings (row
// B26, 2026-11-04), and the text that switches its torch (row B27).
//
// Skydroid's RC SDK (rcsdk 1.9.2 and 1.6.5, com.skydroid.rcsdk.common.payload.C20Camera, read with
// javap) opens the C20's camera on TCP port 8100 of the camera's address, and its gimbal on 5000
// (TOP). The camera's commands are binary packets: 6 zero bytes, a group byte, a command byte, the
// payload's length (2 bytes, high byte first), then the payload, written to the socket as they are.
// So the TOP lens frames on 5000 never zoomed it (the client, 2026-10-27 and 2026-10-30). The zoom
// moves while it is told to: start in or out, then stop. Its zoom can be read back: the reply's
// payload byte 9 runs from 1 to 66, three values for each 1x (the SDK's getZoom), so 1x to 22x. It
// zooms on the client's V20 (test build 19).
//
// A photo, a record start and a record stop are group 0x33, command 0x81, with 32 bytes of payload
// (the SDK's takePicture, startRecordVideo, stopRecordVideo). TOP's CAP and REC on 5000 made only the
// RC's own copies (the client, build 19). The SDK counts a reply with the same group and command and
// at least 32 bytes of payload as done, a shorter one as failed, and none as a time-out. What the
// payload says it does not read. The torch is not on this port: C20Gimbal.toggleLED writes the text
// "AT+LED -t1" (on) or "AT+LED -t0" (off) on the gimbal connection, port 5000, and reads nothing back.
//
// A port of vgcs/skydroid/c20_camera.py, checked against it (gen_c20_vectors.py). Plain C++17 with no
// Qt, unit tested by apk/custom/test/c20_camera_test.cc.

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
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
constexpr std::uint8_t kGroupCamera = 0x33;
constexpr std::uint8_t kCmdCapture = 0x81;            // a photo, a record start, a record stop
constexpr std::size_t kCapturePayloadLen = 32;
constexpr std::size_t kCaptureReplyMinPayload = 32;  // the SDK takes a reply to one as done only with this much

/// 6 zero bytes, group, command, the payload's length (high byte first), the payload.
Bytes packet(std::uint8_t group, std::uint8_t command, const Bytes &payload = {});

Bytes zoomIn();
Bytes zoomOut();
/// The SDK's stopZoomOrFucus: it stops a zoom and a focus move.
Bytes zoomStop();
Bytes zoomQuery();

Bytes photo();
Bytes recordStart();
Bytes recordStop();

/// The text that switches the torch, for the gimbal connection (TOP, port 5000), not this port.
std::string torch(bool on);

/// The packets in what one read brought, cut by their length. The SDK takes each read as one
/// packet and never looks at the length: so a packet whose length runs past the read, or a reply
/// that says less than the SDK wants in it (16 bytes for a zoom, 32 for a photo or record), takes
/// the rest of the read. Less than a header at the end is dropped.
std::vector<Bytes> splitPackets(const Bytes &read);

/// The zoom (1 to 22) a zoom reply says, or nothing (another packet, too short, or a value the
/// SDK has no zoom for).
std::optional<int> zoomOfReply(const Bytes &packet);

/// What a reply to a photo or record command says, as the SDK reads it: true (done), false (less
/// than 32 bytes of payload, the SDK's failure), nothing (not such a reply).
std::optional<bool> captureReplyOk(const Bytes &packet);

} // namespace c20
